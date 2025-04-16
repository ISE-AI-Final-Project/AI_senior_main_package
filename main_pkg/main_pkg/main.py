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
            "req_ism": self.command_srv_req_ism,
            "req_pem": self.command_srv_req_pem,
            "generate_all_grasp": self.command_srv_all_grasp,
            "generate_best_grasp": self.command_srv_best_grasp,
            "make_collision": self.command_srv_make_collision,
            "plan_aim": self.command_plan_aim,
            "trigger_aim": self.command_trigger_aim,
            # "plan_grip": self.command_plan_grip,
            "trigger_grip": self.command_trigger_grip,
            # "plan_home": self.command_plan_home,
            "trigger_home": self.command_trigger_home,
            "gripper_open": self.command_gripper_open,
            "gripper_close": self.command_gripper_close,
            "fake_point_cloud": self.command_fake_point_cloud,
            "fake_object_pose": self.command_fake_object_pose,
        }

        """
        ################### VARIABLE ###########################
        """
        # File name
        self.node_run_folder_name = (
            f"Run-{datetime.now().strftime('%Y-%m-%d-%H-%M-%S-%f')[:-4]}"
        )
        self.capture_folder_name = f"Cap-{self.get_current_time()}"
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

        self.pub_best_mask = self.create_publisher(Image, "/main/best_mask", 10)

        self.pub_object_pose = self.create_publisher(
            PoseStamped, "/main/object_pose", 10
        )

        self.pub_best_grasp_aim_poses = self.create_publisher(
            PoseArray, "/main/best_grasp_aim_poses", 10
        )

        self.pub_best_grasp_grip_poses = self.create_publisher(
            PoseArray, "/main/best_grasp_grip_poses", 10
        )

        # self.pub_collision = self.create_publisher(
        #     CollisionObject, "collision_object_topic", 10
        # )

        """
        ################### CLIENT ###########################
        """
        self.client_gripper_control = self.create_client(Gripper, "gripper_control")

        self.client_all_grasp = self.create_client(GraspPoseSend, "GraspPose")

        self.client_best_grasp = self.create_client(BestGraspPose, "BestGraspPose")

        self.client_make_collision = self.create_client(
            PointCloudSend, "CollisionMaker"
        )

        self.client_aim_grip_plan = self.create_client(AimGripPlan, "AimGripPlan")

        # Trigger Client
        self.client_trigger_aim = self.create_client(Trigger, "/aim_trigger_service")
        self.client_trigger_grip = self.create_client(Trigger, "/grip_trigger_service")
        self.client_trigger_home = self.create_client(Trigger, "/home_trigger_service")

        # Socket Client
        self.client_ism = MyClient(
            host="127.0.0.1", port=11111, client_name="Main ISM Client"
        )
        self.client_pem = MyClient(
            host="127.0.0.1", port=22222, client_name="Main PEM Client"
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

    def get_current_time(self):
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
                    self.capture_folder_name,
                ),
                file_name="rgb",
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
                    self.capture_folder_name,
                ),
                file_name="depth",
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
        self.capture_folder_name = f"Cap-{self.get_current_time()}"

        self.capture_pointcloud = True
        self.capture_rgb = True
        self.capture_depth = True
        self.log("Ready to capture next pointcloud.")

    ## CLIENT: ISM ########################################
    def command_srv_req_ism(self):
        if self.is_empty(self.data_array_rgb) or self.is_empty(self.data_array_depth):
            self.elog("No RGB or Depth Data.")
            return

        client_connected = self.client_ism.connect()
        if not client_connected:
            self.elog(f"Server ISM not started")
            return

        try:
            self.req_ism_time = self.get_current_time()
            target_obj = self.get_str_param("target_obj")

            # Service Request
            best_mask, best_box, best_score, combined_result = (
                self.client_ism.request_msg(
                    msg_type_in=["numpyarray", "numpyarray", "string", "string"],
                    msg_in=[
                        self.data_array_rgb,
                        self.data_array_depth,
                        target_obj,
                        self.get_str_param("dataset_path_prefix"),
                    ],
                )
            )

            # Save Best Mask
            self.data_best_mask = best_mask
            self.data_msg_best_mask = image_utils.mask_to_ros_image(self.data_best_mask)

            # Save Image
            image_utils.save_binary_mask(
                mask=best_mask,
                output_dir=os.path.join(
                    self.get_str_param("output_path_prefix"),
                    self.node_run_folder_name,
                    self.capture_folder_name,
                ),
                file_name=f"{self.req_ism_time}_best_mask_{target_obj}",
            )
            image_utils.save_rgb_image(
                rgb_image=combined_result,
                output_dir=os.path.join(
                    self.get_str_param("output_path_prefix"),
                    self.node_run_folder_name,
                    self.capture_folder_name,
                ),
                file_name=f"{self.req_ism_time}_ism_result_{target_obj}",
            )

            # Pub
            self.pub_best_mask.publish(self.data_msg_best_mask)

            self.log(f"Response Received from ISM with best score: {best_score}")
        except Exception as e:
            self.elog(f"Failed to send request: {e}")

        self.client_ism.disconnect()
        self.log(f"ISM Connection Closed.")

    ## CLIENT: PEM ########################################
    def command_srv_req_pem(self):
        if self.is_empty(self.data_array_rgb) or self.is_empty(self.data_array_depth):
            self.elog("No RGB or Depth Data. Capture First")
            return

        if self.is_empty(self.data_best_mask):
            self.elog("No Best Mask. Req ISM First.")
            return

        client_connected = self.client_pem.connect()
        if not client_connected:
            self.elog(f"Server PEM not started")
            return

        try:
            # TEMP
            # mask_image = PIL.Image.open(
            #     "/home/icetenny/senior-1/SAM-6D/SAM-6D/Data/test_scene/sunscreen.png"
            # ).convert("1")
            # self.data_best_mask = np.array(mask_image, dtype=np.uint8)

            target_obj = self.get_str_param("target_obj")

            # Service Request
            result_rot, result_trans, result_image = self.client_pem.request_msg(
                msg_type_in=[
                    "numpyarray",
                    "numpyarray",
                    "numpyarray",
                    "string",
                    "string",
                ],
                msg_in=[
                    self.data_best_mask,
                    self.data_array_rgb,
                    self.data_array_depth,
                    target_obj,
                    self.get_str_param("dataset_path_prefix"),
                ],
            )

            # result_trans[-1] *= -1

            # Object PoseStamped WRT Zed
            object_pose_wrt_cam = utils.rotation_translation_to_posestamped(
                rotation=result_rot,
                translation=result_trans,
                frame_id="zed_left_camera_optical_frame",
            )
            # Rotate around x-axis by 180 deg
            flip_x_pose = Pose()
            flip_x_pose.position.x, flip_x_pose.position.y, flip_x_pose.position.z = (
                0.0,
                0.0,
                0.0,
            )
            (
                flip_x_pose.orientation.x,
                flip_x_pose.orientation.y,
                flip_x_pose.orientation.z,
                flip_x_pose.orientation.w,
            ) = [0.5, 0.5, 0.5, -0.5]

            flipped_object_pose_wrt_cam = utils.chain_poses_stamped(
                object_pose_wrt_cam,
                flip_x_pose,
                target_frame="zed_left_camera_optical_frame",
            )

            # Transform to world
            self.data_object_pose = utils.transform_pose_stamped(
                self.tf_buffer,
                flipped_object_pose_wrt_cam,
                current_frame="zed_left_camera_optical_frame",
                new_frame="world",
            )

            # Save Image
            image_utils.save_rgb_image(
                rgb_image=result_image,
                output_dir=os.path.join(
                    self.get_str_param("output_path_prefix"),
                    self.node_run_folder_name,
                    self.capture_folder_name,
                ),
                file_name=f"{self.req_ism_time}_pem_result_{target_obj}",
            )

            # Pub
            self.pub_object_pose.publish(self.data_object_pose)

            self.log(f"Response Received from PEM")

            # self.log(f"Response Received from ISM with best score: {best_score}")
        except Exception as e:
            self.elog(f"Failed to send request: {e}")

        self.client_pem.disconnect()
        self.log(f"PEM Connection Closed.")

    ## CLIENT: ALL_GRASP ########################################
    def command_srv_all_grasp(self):
        if not self.client_all_grasp.wait_for_service(timeout_sec=3.0):
            self.elog("Service All Grasp not available!")
            return

        request = GraspPoseSend.Request()
        request.target_obj = self.get_str_param("target_obj")

        request.dataset_path_prefix = self.get_str_param("dataset_path_prefix")

        future = self.client_all_grasp.call_async(request)
        future.add_done_callback(self.command_srv_all_grasp_response_callback)

    def command_srv_all_grasp_response_callback(self, future):
        try:
            response = future.result()
            num_poses = len(response.grasp_poses.grasp_poses)
            self.log(f"Received {num_poses} grasp pose(s).")
            self.data_all_grasp_pose = response.grasp_poses
        except Exception as e:
            self.elog(f"Failed to receive response: {str(e)}")

    ## CLIENT: BEST_GRASP########################################
    def command_srv_best_grasp(self):
        if not self.client_best_grasp.wait_for_service(timeout_sec=3.0):
            self.elog("Service Best Grasp not available!")
            return

        if self.is_empty(self.data_all_grasp_pose):
            self.elog("NO all grasp data")
            return

        if self.is_empty(self.data_object_pose):
            self.elog("NO Object Pose Data, Req PEM First")
            return

        request = BestGraspPose.Request()

        request.all_grasp_poses = self.data_all_grasp_pose
        request.object_pose = self.data_object_pose

        self.log(f"Sending {len(request.all_grasp_poses.grasp_poses)} grasp poses.")

        future = self.client_best_grasp.call_async(request)
        future.add_done_callback(self.command_srv_best_grasp_response_callback)

    def command_srv_best_grasp_response_callback(self, future):
        try:
            response = future.result()
            self.data_sorted_grasp_aim_pose = response.sorted_aim_poses
            self.data_sorted_grasp_grip_pose = response.sorted_grip_poses
            self.data_sorted_grasp_gripper_distance = response.gripper_distance

            num_passed_grasp = len(self.data_sorted_grasp_aim_pose.poses)

            if num_passed_grasp == 0:
                self.elog("No grasp passed criteria")
                return

            self.log(f"Received {num_passed_grasp} best aim pose.")

            for i, pose in enumerate(self.data_sorted_grasp_aim_pose.poses):
                pos = pose.position
                ori = pose.orientation
                self.log(
                    f"[{i:02d}] Position: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f} | "
                    f"Orientation: x={ori.x:.3f}, y={ori.y:.3f}, z={ori.z:.3f}, w={ori.w:.3f}"
                )
            self.pub_best_grasp_aim_poses.publish(self.data_sorted_grasp_aim_pose)
            self.pub_best_grasp_grip_poses.publish(self.data_sorted_grasp_grip_pose)

        except Exception as e:
            self.elog(f"Failed to receive response: {str(e)}")

    ## CLIENT: MAKE COLLISION ########################################
    def command_srv_make_collision(self):
        if not self.client_make_collision.wait_for_service(timeout_sec=3.0):
            self.elog("Service Make Collision not available!")
            return

        if self.is_empty(self.data_pointcloud):
            self.elog("Cannot make Collision. Capture pointcloud first.")
            return

        request = PointCloudSend.Request()
        request.pointcloud = self.data_pointcloud  # Correct field assignment

        future = self.client_make_collision.call_async(request)
        future.add_done_callback(self.command_srv_make_collision_respone_callback)

    def command_srv_make_collision_respone_callback(self, fut):
        try:
            result = fut.result()
            self.log(f"Make Collision Success")
        except Exception as e:
            self.elog(f"Failed to make collision: -> {e}")

    ## CLIENT: PLAN AIM ############################################
    def command_plan_aim(self):
        if not self.client_aim_grip_plan.wait_for_service(timeout_sec=3.0):
            self.elog("Service Aim Grip Plan not available!")
            return
        if self.is_empty(self.data_sorted_grasp_aim_pose) or self.is_empty(
            self.data_sorted_grasp_grip_pose
        ):
            self.log("No Best Grasp Data")
            return

        request = AimGripPlan.Request()
        request.sorted_aim_poses = self.data_sorted_grasp_aim_pose
        request.sorted_grip_poses = self.data_sorted_grasp_grip_pose

        future = self.client_aim_grip_plan.call_async(request)
        future.add_done_callback(self.command_plan_aim_respone_callback)

    def command_plan_aim_respone_callback(self, fut):
        try:
            result = fut.result()
            self.passed_index = result.passed_index
            self.log(f"Plan Aim Grip Success with index: {self.passed_index}")
        except Exception as e:
            self.elog(f"Failed to plan aim and grip: -> {e}")

    ## CLIENT: TRIGGER AIM ############################################
    def command_trigger_aim(self):
        self.log("Going to AIM")
        self.service_trigger_and_wait(self.client_trigger_aim)


    ## CLIENT: TRIGGER GRIP ###########################################
    def command_trigger_grip(self):
        self.srv_gripper_control_send_distance(distance_mm=50)
        # future = self.client_trigger_grip.call_async(Trigger.Request())
        self.log("Going to GRIP")
        self.service_trigger_and_wait(self.client_trigger_grip)

        self.log("Gripping...")
        self.srv_gripper_control_send_distance(distance_mm=0)

    ## CLIENT: TRIGGER HOME ###########################################
    def command_trigger_home(self):
        self.log("Going to HOME")
        self.service_trigger_and_wait(self.client_trigger_home)

    ## CLIENT: GRIPPER CONTROL ########################################
    def srv_gripper_control_send_distance(self, distance_mm):
        if not self.client_gripper_control.wait_for_service(timeout_sec=3.0):
            self.elog("Service Gripper Control not available!")
            return

        request = Gripper.Request()
        request.gripper_distance = distance_mm
        future = self.client_gripper_control.call_async(request)
        future.add_done_callback(
            self.srv_gripper_control_send_distance_respone_callback
        )

    def srv_gripper_control_send_distance_respone_callback(self, fut):
        try:
            result = fut.result()
            self.log(f"Sent distance Success")
        except Exception as e:
            self.elog(f"Failed to send distance: -> {e}")

    ## GRIPPER OPEN ###########################################
    def command_gripper_open(self):
        self.srv_gripper_control_send_distance(distance_mm=55)

    ## GRIPPER CLOSE ########################################
    def command_gripper_close(self):
        self.srv_gripper_control_send_distance(distance_mm=0)

    ## FAKE POINT CLOUD ########################################
    def command_fake_point_cloud(self):
        self.data_pointcloud = fake_utils.get_random_pointcloud()
        self.pub_captured_pointcloud.publish(self.data_pointcloud)

    ## FAKE OBJECT POSE ########################################
    def command_fake_object_pose(self):
        self.data_object_pose = fake_utils.get_random_pose_stamped()
        self.data_object_pose.pose.position.z += 0.8
        self.pub_object_pose.publish(self.data_object_pose)


def main(args=None):
    rclpy.init(args=args)
    node = MainNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
