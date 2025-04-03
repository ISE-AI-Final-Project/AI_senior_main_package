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

from .utils import utils


class ZedSubNode(Node):
    def __init__(self):
        super().__init__("zed_sub_node")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # SUBSCRIBER ################################################
        self.sub_command = self.create_subscription(
            String, "/main/main_command", self.command_callback, 10
        )
        self.sub_pointcloud = self.create_subscription(
            PointCloud2,
            "/zed/zed_node/point_cloud/cloud_registered",
            self.pointcloud_callback,
            10,
        )

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

    def pointcloud_callback(self, msg: PointCloud2):
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

            self.pub_current_pointcloud.publish(transformed_pointcloud)

            self.data_pointcloud = transformed_pointcloud
            self.data_pointcloud_xyz = transformed_xyz
            self.capture_pointcloud = False
            self.log("Captured and saved pointcloud.")

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
        self.log("Ready to capture next pointcloud.")


    ## CLIENT: ALL_GRASP ########################################
    def command_srv_all_grasp(self):
        self.log("I need mumu.")
        request = GraspPoseSend.Request()
        request.target_obj = self.get_parameter("target_obj").get_parameter_value().string_value
        self.log("I need mumu2.")

        # Ensure client is connected before calling
        if not self.grasp_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("Service not available!")
            return
        future = self.grasp_client.call_async(request)
        future.add_done_callback(self.command_srv_all_grasp_response_callback)

    def command_srv_all_grasp_response_callback(self, future):
        print("im back")
        try:
            response = future.result()
            self.get_logger().info(f"Received response: {response}")
            num_poses = len(response.grasp_poses.grasp_poses)
            self.get_logger().info(f"Received {num_poses} grasp pose(s).")
        except Exception as e:
            self.get_logger().error(f'Failed to receive response: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ZedSubNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
