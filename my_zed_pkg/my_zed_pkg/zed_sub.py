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
from main_pkg.utils import utils


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

        self.data_rgb = None
        self.capture_rgb = True

        self.data_depth = None
        self.capture_depth = True


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


        self.log("Zed Sub Node is Running. Ready for command.")


    def command_callback(self, msg: String):
        recv_command = msg.data.strip().lower()
        self.get_logger().info(f"Received command: {recv_command}")

        if recv_command == "capture":
            self.command_capture()

        elif recv_command == "make_collision":
            self.command_make_collision()

        elif recv_command == "srv_all_grasp":
            self.command_srv_all_grasp()

        # elif recv_command == "srv_all_grasp":
        #     self.command_srv_all_grasp()

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

            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as e:
                self.get_logger().error(f'CVBridge error: {e}')
                return
            
            print(type(cv_image), cv_image.shape)

            cv2.imshow('RGB', cv_image)
            cv2.waitKey(1)  # Refresh frame

            self.data_rgb = msg.data
            # self.capture_rgb = False
            # self.log("Captured RGB.")

    def zed_depth_callback(self, msg: Image):
        """
        Transform and save DEPTH only when commanded.
        """
        if self.capture_depth:
            try:
                cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            except Exception as e:
                self.get_logger().error(f'CVBridge error: {e}')
                return
            
            # print(type(cv_depth), cv_depth.shape)

            # Replace NaNs with 0
            cv_depth = np.nan_to_num(cv_depth, nan=0.0)
            # Clip values to 0.0 - 5.0 meters
            cv_depth = np.clip(cv_depth, 0.0, 5.0)

            vis = cv2.normalize(cv_depth, None, 0, 255, cv2.NORM_MINMAX)
            vis = vis.astype(np.uint8)
            cv2.imshow("Depth View", vis)
            cv2.waitKey(1)  # Refresh frame

            self.data_depth = msg.data
            # self.capture_depth = False
            # self.log("Captured depth.")

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
        self.capture_pointcloud = True
        self.capture_rgb = True
        self.capture_depth = True
        self.log("Ready to capture next pointcloud.")


def main(args=None):
    rclpy.init(args=args)
    node = ZedSubNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
