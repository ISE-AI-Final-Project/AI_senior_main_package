import os
import time

import cv2
import imageio
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from rcl_interfaces.msg import Parameter, ParameterDescriptor, ParameterType
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformException, TransformListener

from custom_srv_pkg.msg import GraspPose, GraspPoses
from custom_srv_pkg.srv import GraspPoseSend, IMGSend, PointCloudSend
from main_pkg.utils import my_custom_socket, utils


def save_binary_masks(masks: np.ndarray, output_dir: str, prefix: str = "mask"):
    """
    Saves a batch of binary masks to image files.

    Args:
        masks (np.ndarray): Binary masks of shape [n, width, height]
        output_dir (str): Directory to save the images.
        prefix (str): Optional prefix for filenames.
    """
    os.makedirs(output_dir, exist_ok=True)
    n = masks.shape[0]

    for i in range(n):
        # Get the i-th mask and scale to 0–255 for visibility
        mask = (masks[i] * 255).astype(np.uint8)

        # Save as PNG
        path = os.path.join(output_dir, f"{prefix}_{i:03}.png")
        cv2.imwrite(path, mask)

    print(f"Saved {n} masks to: {output_dir}")


def save_rgb_images(images: np.ndarray, output_dir: str, prefix: str = "rgb"):
    os.makedirs(output_dir, exist_ok=True)
    path = os.path.join(output_dir, f"{prefix}.png")
    cv2.imwrite(path, cv2.cvtColor(images, cv2.COLOR_RGB2BGR))  # OpenCV uses BGR


def save_depth_uint16(depth_maps: np.ndarray, output_dir: str, prefix: str = "depth"):
    os.makedirs(output_dir, exist_ok=True)
    path = os.path.join(output_dir, f"{prefix}.png")

    # Save as 16-bit PNG (supports uint16)
    cv2.imwrite(path, depth_maps.astype(np.uint16))


class ZedSubNode(Node):
    def __init__(self):
        super().__init__("zed_sub_node")

        # Temp
        self.string_input = "sunscreen"
        self.output_dir = f"/home/icetenny/senior-2/results/run_{time.time_ns()}"
        self.save_prefix = f"{time.time_ns()}"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.bridge = CvBridge()
        # VARIABLE ######################################
        self.data_pointcloud = None
        self.data_pointcloud_xyz = None
        self.capture_pointcloud = False

        self.data_msg_rgb = None
        self.data_array_rgb = None
        self.capture_rgb = False

        self.data_msg_depth = None
        self.data_array_depth = None
        self.capture_depth = False

        # SUBSCRIBER ################################################
        self.sub_command = self.create_subscription(
            String, "/main/main_command", self.command_callback, 10
        )
        self.sub_zed_pointcloud = self.create_subscription(
            PointCloud2,
            "/zed/zed_node/point_cloud/cloud_registered",
            self.zed_pointcloud_callback,
            10,
        )

        self.sub_zed_rgb = self.create_subscription(
            Image,
            "/zed/zed_node/left/image_rect_color",
            self.zed_rgb_callback,
            10,
        )

        self.sub_zed_depth = self.create_subscription(
            Image,
            "/zed/zed_node/depth/depth_registered",
            self.zed_depth_callback,
            10,
        )

        # PUBLISHER ######################################
        self.pub_rviz_text = self.create_publisher(String, "/main/rviz_text", 10)

        # SOCKET CLIENT #################################
        self.client_ism = my_custom_socket.MyClient(host="127.0.0.1", port=65432)

        self.log("Zed Sub Node is Running. Ready for command.")

    def command_callback(self, msg: String):
        recv_command = msg.data.strip().lower()
        self.get_logger().info(f"Received command: {recv_command}")

        if recv_command == "capture":
            self.command_capture()

        elif recv_command == "req_ism":
            self.command_srv_req_ism()

    def zed_pointcloud_callback(self, msg: PointCloud2):
        """
        Transform and save pointcloud only when commanded.
        """
        if self.capture_pointcloud:
            try:
                tf = self.tf_buffer.lookup_transform(
                    "base_link",
                    msg.header.frame_id,
                    rclpy.time.Time(),
                    rclpy.duration.Duration(seconds=0.5),
                )
            except TransformException as ex:
                self.log(f"Could not get transform for point cloud: {ex}")
                return

            transformed_pointcloud, transformed_xyz = utils.transform_pointcloud(
                msg=msg, tf=tf, frame_id="base_link"
            )

            # self.pub_current_pointcloud.publish(transformed_pointcloud)

            self.data_pointcloud = transformed_pointcloud
            self.data_pointcloud_xyz = transformed_xyz
            self.capture_pointcloud = False
            self.log("Captured and saved pointcloud.")

    def zed_rgb_callback(self, msg: Image):
        """
        Transform and save RGB only when commanded.
        """
        if self.capture_rgb:
            # Save Msg
            self.data_msg_rgb = msg

            # Convert to array
            try:
                array_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                array_rgb = cv2.cvtColor(array_bgr, cv2.COLOR_BGR2RGB)
            except Exception as e:
                self.get_logger().error(f"CVBridge error: {e}")
                return

            self.data_array_rgb = array_rgb
            self.capture_rgb = False
            self.log("Captured RGB.")

            # Save Image
            save_rgb_images(
                images=self.data_array_rgb,
                output_dir=self.output_dir,
                prefix=f"{self.save_prefix}_rgb",
            )

    def zed_depth_callback(self, msg: Image):
        """
        Transform and save DEPTH only when commanded.
        """
        if self.capture_depth:
            # Save msg
            self.data_msg_depth = msg.data

            # Convert to array
            try:
                array_depth_m = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            except Exception as e:
                self.get_logger().error(f"CVBridge error: {e}")
                return

            # Replace NaNs with 0
            array_depth_m = np.nan_to_num(
                array_depth_m, nan=0.0, posinf=0.0, neginf=0.0
            )

            # SClip values to range 10m
            array_depth_m = np.clip(array_depth_m, 0.0, 10.0)

            # Convert to mm
            # array_depth_mm = (array_depth_m * 1000).astype(np.int32)
            array_depth_mm = (array_depth_m * 1000).astype(np.int16)

            self.data_array_depth = array_depth_mm
            self.capture_depth = False
            self.log("Captured depth.")

            # Save Depth
            save_depth_uint16(
                depth_maps=self.data_array_depth,
                output_dir=self.output_dir,
                prefix=f"{self.save_prefix}_depth",
            )

    def log(self, text):
        # Log in terminal
        self.get_logger().info(text)
        # Pub to RViz
        string_msg = String()
        string_msg.data = text
        self.pub_rviz_text.publish(string_msg)

    def command_capture(self):
        """
        Capture Current Point Cloud in self.data_pointcloud
        """
        self.save_prefix = time.time_ns()
        
        self.capture_pointcloud = False
        self.capture_rgb = True
        self.capture_depth = True
        self.log("Ready to capture next pointcloud.")

    def command_srv_req_ism(self):
        if self.data_array_rgb is None or self.data_array_depth is None:
            self.log("No RGB or Depth Data.")
            return

        client_connected = self.client_ism.connect()
        if not client_connected:
            self.get_logger().error(f"Server not started")
            return

        # Send request
        try:
            detections_boxes, detections_masks, detections_scores = (
                self.client_ism.request_msg(
                    msg_type_in=["numpyarray", "numpyarray", "string"],
                    msg_in=[self.data_array_rgb, self.data_array_depth, self.string_input],
                )
            )
            self.get_logger().info(f"Response: {detections_boxes} {detections_scores}")

            save_binary_masks(
                masks=detections_masks,
                output_dir=self.output_dir,
                prefix=f"{self.save_prefix}_masks_{self.string_input}",
            )

        except Exception as e:
            self.get_logger().error(f"Failed to send request: {e}")

        self.client_ism.disconnect()


def main(args=None):
    rclpy.init(args=args)
    node = ZedSubNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
