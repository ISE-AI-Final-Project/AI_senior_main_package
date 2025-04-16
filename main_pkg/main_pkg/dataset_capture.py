import os
from datetime import datetime

import cv2
import numpy as np
import PIL
import PIL.Image
import rclpy
import rclpy.client
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped, Quaternion
from moveit_msgs.msg import CollisionObject
from rcl_interfaces.msg import Parameter, ParameterDescriptor, ParameterType
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from custom_srv_pkg.msg import GraspPose, GraspPoses
from custom_srv_pkg.srv import (
    AimGripPlan,
    BestGraspPose,
    GraspPoseSend,
    Gripper,
    IMGSend,
    PointCloudSend,
)

from .utils import fake_utils, image_utils, utils
from .utils.my_custom_socket import MyClient


class MainNode(Node):
    def __init__(self):
        super().__init__("main")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.bridge = CvBridge()

        self.command_map = {
            "capture": self.command_capture,
        }

        """
        ################### VARIABLE ###########################
        """
        # File name
        self.node_run_folder_name = (
            f"Run-dataset-{datetime.now().strftime('%Y-%m-%d-%H-%M-%S-%f')[:-4]}"
        )
        self.capture_file_name = f"Cap-{self.get_current_time(with_date=True)}"
        self.req_ism_time = self.get_current_time

        # Point Cloud
        self.data_pointcloud = PointCloud2()
        self.data_pointcloud_xyz = np.array([])
        self.capture_pointcloud = False

        # RGB
        self.data_msg_rgb = Image()
        self.data_array_rgb = np.array([])
        self.capture_rgb = False

        # Depth
        self.data_msg_depth = Image()
        self.data_array_depth = np.array([])
        self.capture_depth = False

        # Mask
        self.data_best_mask = np.array([])
        self.data_msg_best_mask = Image()

        # Pose
        self.data_object_pose = PoseStamped()
        self.data_all_grasp_pose = GraspPoses()
        self.data_sorted_grasp_aim_pose = PoseArray()
        self.data_sorted_grasp_grip_pose = PoseArray()
        self.data_sorted_grasp_gripper_distance = []

        self.passed_index = None

        """
        ################### PARAM ###########################
        """
        param_descriptor_target_object = ParameterDescriptor(
            name="target_obj",
            type=ParameterType.PARAMETER_STRING,
            description="Target Object",
        )
        self.declare_parameter(
            "target_obj", "sunscreen", descriptor=param_descriptor_target_object
        )

        param_descriptor_dataset_path_prefix = ParameterDescriptor(
            name="dataset_path_prefix",
            type=ParameterType.PARAMETER_STRING,
            description="Path to dataset folder containing CAD file of object and templates.",
        )

        self.declare_parameter(
            "dataset_path_prefix",
            "/home/icetenny/senior-2/senior_dataset/",
            descriptor=param_descriptor_dataset_path_prefix,
        )

        param_descriptor_output_path_prefix = ParameterDescriptor(
            name="output_path_prefix",
            type=ParameterType.PARAMETER_STRING,
            description="Path to folder for output files.",
        )

        self.declare_parameter(
            "output_path_prefix",
            "/home/icetenny/senior-2/results/",
            descriptor=param_descriptor_output_path_prefix,
        )

        """
        ################### SUBSCRIBER ###########################
        """
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

        """
        ################### PUBLISHER ###########################
        """
        self.pub_rviz_text = self.create_publisher(String, "/main/rviz_text", 10)

        self.pub_captured_pointcloud = self.create_publisher(
            PointCloud2, "/main/captured_pointcloud", 10
        )

        self.pub_captured_rgb = self.create_publisher(Image, "/main/captured_rgb", 10)

        self.pub_captured_depth = self.create_publisher(
            Image, "/main/captured_depth", 10
        )

        # Finish Init
        self.log("Main Node is Running. Ready for command.")
        self.announce_param('target_obj')
        self.announce_param('dataset_path_prefix')
        self.announce_param('output_path_prefix')

    def log(self, text):
        """
        Log Text into Terminal and RViz label
        """
        # Log in terminal
        self.get_logger().info(str(text))
        # Pub to RViz
        string_msg = String()
        string_msg.data = str(text)
        self.pub_rviz_text.publish(string_msg)

    def tlog(self, text):
        """
        Log Text into Terminal only
        """
        # Log in terminal
        self.get_logger().info(str(text))

    def elog(self, text):
        """
        Log Error Text into Terminal and RViz label
        """
        # Log in terminal
        self.get_logger().error(str(text))
        # Pub to RViz
        string_msg = String()
        string_msg.data = str(text)
        self.pub_rviz_text.publish(string_msg)
    
    def announce_param(self, param_name):
        self.log(f"⚠️  Param '{param_name}' set to '{self.get_str_param(param_name)}'. Use ros2 param set or rqt to change.")

    def is_empty(self, obj):
        if obj is None:
            return True
        if isinstance(obj, np.ndarray):
            return obj.size == 0
        if hasattr(obj, "data"):
            return not obj.data
        try:
            return obj == type(obj)()
        except Exception:
            return False

    def get_str_param(self, param_name):
        return self.get_parameter(param_name).get_parameter_value().string_value

    def get_current_time(self, with_date=False):
        if with_date:
            return f"{datetime.now().strftime('%Y-%m-%d-%H-%M-%S-%f')[:-4]}"
        else:
            return f"{datetime.now().strftime('%H-%M-%S-%f')[:-4]}"

    def service_trigger_and_wait(self, client: rclpy.client.Client):
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda res: self.log(res.result()))

    def command_callback(self, msg: String):
        """
        RViz command callback
        """
        recv_command = msg.data.strip().lower()
        self.log(f"Received command: {recv_command}")

        if recv_command in self.command_map:
            self.command_map[recv_command]()
        else:
            self.get_logger().warn(f"Unknown command received: {recv_command}")

    def zed_pointcloud_callback(self, msg: PointCloud2):
        """
        Transform and save pointcloud only when commanded.
        """
        if self.capture_pointcloud:
            try:
                tf = self.tf_buffer.lookup_transform(
                    "world",
                    msg.header.frame_id,
                    rclpy.time.Time(),
                    rclpy.duration.Duration(seconds=0.5),
                )
            except TransformException as ex:
                self.log(f"Could not get transform for point cloud: {ex}")
                return

            transformed_pointcloud, transformed_xyz = utils.transform_pointcloud(
                msg=msg, tf=tf, frame_id="world"
            )

            self.pub_captured_pointcloud.publish(transformed_pointcloud)

            self.data_pointcloud = transformed_pointcloud
            self.data_pointcloud_xyz = transformed_xyz
            self.capture_pointcloud = False
            self.log("Captured and saved pointcloud.")

    def zed_rgb_callback(self, msg: Image):
        """
        Save RGB only when commanded.
        """
        if self.capture_rgb:
            # Save Msg
            self.data_msg_rgb = msg

            # Convert to array
            try:
                array_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                array_rgb = cv2.cvtColor(array_bgr, cv2.COLOR_BGR2RGB)
            except Exception as e:
                self.elog(f"CVBridge error: {e}")
                return
            self.data_array_rgb = array_rgb
            self.capture_rgb = False
            self.pub_captured_rgb.publish(msg)
            # Save Image
            image_utils.save_rgb_image(
                rgb_image=self.data_array_rgb,
                output_dir=os.path.join(
                    self.get_str_param("output_path_prefix"),
                    self.node_run_folder_name,
                    "rgb",
                ),
                file_name=f"{self.capture_file_name}.png",
            )

            self.log("Captured RGB.")

    def zed_depth_callback(self, msg: Image):
        """
        Save DEPTH only when commanded.
        """
        if self.capture_depth:
            # Save msg
            self.data_msg_depth = msg.data

            # Convert to array
            try:
                array_depth_m = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            except Exception as e:
                self.elog(f"CVBridge error: {e}")
                return

            # Replace NaNs with 0
            array_depth_m = np.nan_to_num(
                array_depth_m, nan=0.0, posinf=0.0, neginf=0.0
            )

            # SClip values to range 10m
            array_depth_m = np.clip(array_depth_m, 0.0, 10.0)

            # Convert to mm
            array_depth_mm = (array_depth_m * 1000).astype(np.int32)

            self.data_array_depth = array_depth_mm
            self.capture_depth = False
            self.pub_captured_depth.publish(msg)

            # Save Image
            image_utils.save_depth_uint16(
                depth_maps=self.data_array_depth,
                output_dir=os.path.join(
                    self.get_str_param("output_path_prefix"),
                    self.node_run_folder_name,
                    "depth",
                ),
                file_name=f"{self.capture_file_name}.png",
            )

            self.log("Captured depth.")

    def command_capture(self):
        """
        Capture Current Point Cloud in self.data_pointcloud
        """

        if not os.path.exists(
            os.path.join(
                self.get_str_param("output_path_prefix"), self.node_run_folder_name
            )
        ):
            os.makedirs(
                os.path.join(
                    self.get_str_param("output_path_prefix"), self.node_run_folder_name
                )
            )
        self.capture_file_name = f"{self.get_current_time(with_date=True)}"

        # self.capture_pointcloud = True
        self.capture_rgb = True
        self.capture_depth = True
        self.log("Ready to capture next pointcloud.")


def main(args=None):
    rclpy.init(args=args)
    node = MainNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
