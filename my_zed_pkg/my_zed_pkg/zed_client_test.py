import cv2
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


class ZedSubNode(Node):
    def __init__(self):
        super().__init__("zed_sub_node")
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
                array_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception as e:
                self.get_logger().error(f"CVBridge error: {e}")
                return
            self.data_array_rgb = array_rgb
            self.capture_rgb = False
            self.log("Captured RGB.")

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
            array_depth_m = np.nan_to_num(array_depth_m, nan=0.0, posinf=0.0, neginf=0.0)

            # SClip values to range 10m
            array_depth_m = np.clip(array_depth_m, 0.0, 10.0)

            # Convert to mm
            array_depth_mm = (array_depth_m * 1000).astype(np.int32)

            self.data_array_depth = array_depth_mm
            self.capture_depth = False
            self.log("Captured depth.")

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

        string_input = "acnewash"

        # Send request
        try:
            result = self.client_ism.request_msg(
                msg_type_in=["numpyarray", "numpyarray", "string"],
                msg_in=[self.data_array_rgb, self.data_array_depth, string_input],
            )
            self.get_logger().info(f"Response: {result}")
        except Exception as e:
            self.get_logger().error(f"Failed to send request: {e}")

        self.client_ism.disconnect()


def main(args=None):
    rclpy.init(args=args)
    node = ZedSubNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
