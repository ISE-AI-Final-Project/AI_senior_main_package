import asyncio
import os
from datetime import datetime
from threading import Thread

import cv2
import numpy as np
import PIL
import PIL.Image
import rclpy
import rclpy.client
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped, Quaternion
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from rcl_interfaces.msg import Parameter, ParameterDescriptor, ParameterType
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from custom_srv_pkg.msg import GraspPose, GraspPoses
from custom_srv_pkg.srv import (
    AimGripPlan,
    BestGraspPose,
    CameraIKJointState,
    GraspPoseSend,
    Gripper,
    IKJointState,
    IMGSend,
    JointPose,
    JointStateCollision,
    PCLFuse,
    PCLMani,
    PointCloudSend,
    PointCloudSendWithMask,
    TargetObjectSend,
)

from .utils import fake_utils, image_utils, utils
from .utils.my_custom_socket import MyClient


class MainNode(Node):
    def __init__(self):

        self.loop = asyncio.new_event_loop()
        thread = Thread(target=self.loop.run_forever)
        thread.daemon = True
        thread.start()

        super().__init__("main")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.bridge = CvBridge()

        self.command_map = {
            "capture": self.command_capture,
            "capture_to_fuse": self.command_capture_to_fuse,
            "req_ism": self.command_srv_req_ism,
            "req_pem": self.command_srv_req_pem,
            "generate_all_grasp": self.command_srv_all_grasp,
            "generate_best_grasp": self.command_srv_best_grasp,
            "make_collision": self.command_srv_make_collision,
            "make_collision_with_mask": self.command_srv_make_collision_with_mask,
            "ik_grasp": self.command_ik_grasp,
            "plan_aim_grip": self.command_plan_aim_grip,
            "trigger_aim": self.command_trigger_aim,
            "trigger_grip": self.command_trigger_grip,
            "plan_home": self.command_plan_home,
            "trigger_home": self.command_trigger_home,
            "attach_object": self.command_attach_object,
            "detach_object": self.command_detach_object,
            "gripper_open": self.command_gripper_open,
            "gripper_close": self.command_gripper_close,
            "fake_point_cloud": self.command_fake_point_cloud,
            "fake_object_pose": self.command_fake_object_pose,
            "fuse_pointcloud": self.command_fuse_pointcloud,
            "clear_pointcloud": self.command_clear_pointcloud,
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

        # Camera info
        self.camera_info = CameraInfo()
        self.capture_camera_info = False

        # Point Cloud
        self.data_pointcloud = PointCloud2()
        self.data_pointcloud_xyz = np.array([])
        self.capture_pointcloud = False

        self.capture_to_fuse = False
        self.data_pointcloud_fused = PointCloud2()

        # RGB
        self.data_msg_rgb = Image()
        self.data_array_rgb = np.array([])
        self.capture_rgb = False

        # Depth
        self.data_msg_depth = Image()
        self.data_array_depth = np.array([])
        self.capture_depth = False

        # ISM Result
        self.data_best_mask = np.array([])
        self.data_msg_best_mask = Image()

        # PEM Result
        self.data_pem_result = np.array([])
        self.data_msg_pem_result = Image()
        self.data_object_pose_wrt_cam = PoseStamped()
        self.data_object_pose = PoseStamped()

        # Pose
        self.data_all_grasp_pose = GraspPoses()
        self.data_sorted_grasp_aim_pose = PoseArray()
        self.data_sorted_grasp_grip_pose = PoseArray()
        self.data_sorted_grasp_gripper_distance = []

        self.passed_index = None

        # Planning Scene
        self.data_planning_scene = PlanningScene()

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

        self.sub_zed_cam_K = self.create_subscription(
            CameraInfo,
            "/zed/zed_node/left/camera_info",
            self.zed_camera_info_callback,
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

        self.pub_pem_result = self.create_publisher(Image, "/main/pem_result", 10)

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

        self.pub_pointcloud_no_object = self.create_publisher(
            PointCloud2, "/main/pointcloud_no_object", 10
        )

        self.pub_fused_pointcloud = self.create_publisher(
            PointCloud2, "/main/fused_pointcloud", 10
        )

        self.pub_pointcloud_raw = self.create_publisher(
            PointCloud2, "/main/pointcloud_raw", 10
        )

        """
        ################### CLIENT ###########################
        """
        self.client_gripper_control = self.create_client(Gripper, "gripper_control")

        self.client_all_grasp = self.create_client(GraspPoseSend, "GraspPose")

        self.client_best_grasp = self.create_client(BestGraspPose, "BestGraspPose")

        self.client_stl_collision = self.create_client(
            TargetObjectSend, "make_stl_collision"
        )

        self.client_make_collision = self.create_client(
            PointCloudSend, "CollisionMaker"
        )

        self.client_ik_grasp = self.create_client(IKJointState, "joint_state_from_ik")
        self.client_joint_state_collision = self.create_client(
            JointStateCollision, "joint_state_collision_check"
        )
        self.client_get_planning_scene = self.create_client(
            GetPlanningScene, "/get_planning_scene"
        )

        self.client_make_collision_with_mask = self.create_client(PCLMani, "pcl_mani")

        self.client_aim_grip_plan = self.create_client(AimGripPlan, "AimGripPlan")
        self.client_home_plan = self.create_client(Trigger, "home_plan_service")

        # Trigger Client
        self.client_trigger_aim = self.create_client(Trigger, "/aim_trigger_service")
        self.client_trigger_grip = self.create_client(Trigger, "/grip_trigger_service")
        self.client_trigger_home = self.create_client(Trigger, "/home_trigger_service")

        self.client_attach = self.create_client(Trigger, "attach_collision_object")
        self.client_detach = self.create_client(Trigger, "detach_collision_object")

        self.client_fuse_pointcloud = self.create_client(PCLFuse, "pcl_fuse")
        self.client_camera_ik_joint_state = self.create_client(
            CameraIKJointState, "camera_joint_state_from_ik"
        )

        # Socket Client
        self.client_ism = MyClient(
            host="127.0.0.1", port=11111, client_name="Main ISM Client"
        )
        self.client_pem = MyClient(
            host="127.0.0.1", port=22222, client_name="Main PEM Client"
        )

        # Finish Init
        self.log("Main Node is Running. Ready for command.")
        self.announce_param("target_obj")
        self.announce_param("dataset_path_prefix")
        self.announce_param("output_path_prefix")

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
        self.log(
            f"⚠️  Param '{param_name}' set to '{self.get_str_param(param_name)}'. Use ros2 param set or rqt to change."
        )

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

    async def call_service(self, client, request):
        try:
            future = client.call_async(request)
            await future
            return future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            return None

    def call_make_stl_collision(self, object_pose=None):
        req = TargetObjectSend.Request()
        if object_pose is None:
            req.object_pose = self.data_object_pose
        else:
            req.object_pose = object_pose
        req.dataset_path_prefix = self.get_str_param("dataset_path_prefix")
        req.target_obj = self.get_str_param("target_obj")

        future = self.client_stl_collision.call_async(req)

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

    def zed_camera_info_callback(self, msg: CameraInfo):
        if self.capture_camera_info:
            self.camera_info = msg

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

            self.log(self.data_pointcloud.width)

            if self.capture_to_fuse:
                # Fuse Pointcloud
                if self.is_empty(self.data_pointcloud_fused):
                    self.data_pointcloud_fused = self.data_pointcloud
                else:
                    self.data_pointcloud_fused = utils.combine_pointclouds(
                        self.data_pointcloud_fused, self.data_pointcloud
                    )

                self.pub_fused_pointcloud.publish(self.data_pointcloud_fused)
                self.log(
                    f"Fuse PointCloud, current width {self.data_pointcloud_fused.width}"
                )

                self.capture_to_fuse = False

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
            self.data_msg_depth = msg

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
        self.capture_camera_info = True
        self.log("Ready to capture next pointcloud.")

    def command_capture_to_fuse(self):
        self.capture_to_fuse = True
        self.command_capture()

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

            self.data_pem_result = result_image
            self.data_msg_pem_result = image_utils.rgb_to_ros_image(
                self.data_pem_result
            )

            # Object PoseStamped WRT Zed
            self.data_object_pose_wrt_cam = utils.rotation_translation_to_posestamped(
                rotation=result_rot,
                translation=result_trans,
                frame_id="zed_left_camera_optical_frame",
            )

            self.log(self.data_object_pose_wrt_cam)

            # Transform to world
            self.data_object_pose = utils.transform_pose_stamped(
                self.tf_buffer,
                self.data_object_pose_wrt_cam,
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
            self.pub_pem_result.publish(self.data_msg_pem_result)

            self.call_make_stl_collision()

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

            # for i, pose in enumerate(self.data_sorted_grasp_aim_pose.poses):
            #     pos = pose.position
            #     ori = pose.orientation
            #     self.log(
            #         f"[{i:02d}] Position: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f} | "
            #         f"Orientation: x={ori.x:.3f}, y={ori.y:.3f}, z={ori.z:.3f}, w={ori.w:.3f}"
            #     )
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
        future.add_done_callback(self.command_srv_make_collision_response_callback)

    def command_srv_make_collision_response_callback(self, fut):
        try:
            result = fut.result()
            self.log(f"Make Collision Success")
        except Exception as e:
            self.elog(f"Failed to make collision: -> {e}")

    ## CLIENT: MAKE COLLISION WITH MASK #####################################
    def command_srv_make_collision_with_mask(self):
        if not self.client_make_collision_with_mask.wait_for_service(timeout_sec=3.0):
            self.elog("Service Make Collision With Mask not available!")
            return

        if self.is_empty(self.data_pointcloud):
            self.elog("Cannot make Collision. Capture pointcloud first.")
            return

        if self.is_empty(self.data_msg_best_mask):
            self.elog("No Best Mask. Req ISM first.")
            return

        req = PCLMani.Request()
        req.rgb_image = self.data_msg_rgb
        req.depth_image = self.data_msg_depth
        req.mask_image = self.data_msg_best_mask
        req.sixd_pose = self.data_object_pose_wrt_cam
        req.target_object = self.get_str_param("target_obj")
        req.dataset_path_prefix = self.get_str_param("dataset_path_prefix")
        req.camera_info = self.camera_info

        future = self.client_make_collision_with_mask.call_async(req)
        future.add_done_callback(
            self.command_srv_make_collision_with_mask_response_callback
        )

    def command_srv_make_collision_with_mask_response_callback(self, fut):
        try:
            result = fut.result()
            self.log(f"Remove Target Object from Pointcloud success. Making Collision")

            # Transform to world
            tf = self.tf_buffer.lookup_transform(
                "world",
                result.pointcloud.header.frame_id,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.5),
            )

            transformed_pointcloud_no_object, transformed_xyz_no_object = (
                utils.transform_pointcloud(
                    msg=result.pointcloud, tf=tf, frame_id="world"
                )
            )

            self.data_pointcloud_no_object = transformed_pointcloud_no_object

            self.pub_pointcloud_no_object.publish(self.data_pointcloud_no_object)

            request_make_collision = PointCloudSend.Request()
            request_make_collision.pointcloud = transformed_pointcloud_no_object

            future_make_collision = self.client_make_collision.call_async(
                request_make_collision
            )
            future_make_collision.add_done_callback(
                self.command_srv_make_collision_response_callback
            )

        except Exception as e:
            self.elog(f"Failed to make collision with mask: -> {e}")

    ## CLIENT: IK GRASP #################################################
    #  1. Get Planning Scene
    #  2. Get all joint state by IK
    #  3. Filter joint state by planning scene

    def command_ik_grasp(self):
        # Get Planning Scene
        if not self.client_get_planning_scene.wait_for_service(timeout_sec=3.0):
            self.elog("Service /get_planning_scene not available!")
            return
        request_get_planning_scene = GetPlanningScene.Request()
        request_get_planning_scene.components.components = (
            PlanningSceneComponents.SCENE_SETTINGS
            | PlanningSceneComponents.ROBOT_STATE
            | PlanningSceneComponents.WORLD_OBJECT_NAMES
            | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        )
        future_get_planning_scene = self.client_get_planning_scene.call_async(
            request_get_planning_scene
        )
        future_get_planning_scene.add_done_callback(
            self.get_planning_scene_ik_response_callback
        )

    def get_planning_scene_ik_response_callback(self, future_get_planning_scene):
        try:
            result_get_planning_scene = future_get_planning_scene.result()
            self.data_planning_scene = result_get_planning_scene.scene
            self.log(
                f"Received Planning Scene of type: {type(self.data_planning_scene)}."
            )

            # IK Grasp
            if not self.client_ik_grasp.wait_for_service(timeout_sec=3.0):
                self.elog("Service IK Grasp not available!")
                return
            if self.is_empty(self.data_sorted_grasp_aim_pose) or self.is_empty(
                self.data_sorted_grasp_grip_pose
            ):
                self.log("No Best Grasp Data")
                return
            request_ik_grasp = IKJointState.Request()
            request_ik_grasp.sorted_aim_poses = self.data_sorted_grasp_aim_pose
            request_ik_grasp.sorted_grip_poses = self.data_sorted_grasp_grip_pose
            request_ik_grasp.gripper_distance = self.data_sorted_grasp_gripper_distance
            request_ik_grasp.current_joint_state = (
                self.data_planning_scene.robot_state.joint_state
            )

            future_ik_grasp = self.client_ik_grasp.call_async(request_ik_grasp)
            future_ik_grasp.add_done_callback(self.ik_grasp_response_callback)
        except Exception as e:
            self.elog(f"Failed to get planning scene: -> {e}")

    def ik_grasp_response_callback(self, future_ik_grasp):
        try:
            result_ik_grasp = future_ik_grasp.result()
            self.log(
                f"Num Possible Joint state: {len(result_ik_grasp.possible_aim_joint_state)}"
            )
            self.log(
                f"Possible Joint state: {result_ik_grasp.possible_aim_joint_state[0]}"
            )

            if not self.client_joint_state_collision.wait_for_service(timeout_sec=3.0):
                self.elog("Service Joint State Collision Check not available!")
                return

            request_joint_state_collision = JointStateCollision.Request()
            request_joint_state_collision.joint_state = (
                result_ik_grasp.possible_aim_joint_state
            )
            request_joint_state_collision.gripper_distance = (
                result_ik_grasp.gripper_distance
            )
            request_joint_state_collision.planning_scene = self.data_planning_scene

            future_joint_state_collision = self.client_joint_state_collision.call_async(
                request_joint_state_collision
            )
            future_joint_state_collision.add_done_callback(
                self.joint_state_collision_response_callback
            )

        except Exception as e:
            self.elog(f"Failed to get joint state from IK: -> {e}")

    def joint_state_collision_response_callback(self, fut):
        try:
            result_joint_state_collision = fut.result()
            self.log(
                f"Num Possible Joint state No Collsion: {len(result_joint_state_collision.possible_joint_state)}"
            )
            self.log(
                f"Possible Joint state No Collsion: {result_joint_state_collision.possible_joint_state[0]}"
            )

        except Exception as e:
            self.elog(f"Failed to check Joint State from Collsion : -> {e}")

    ## CLIENT: PLAN AIM GRIP ############################################
    def command_plan_aim_grip(self):
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
        future.add_done_callback(self.command_plan_aim_grip_response_callback)

    def command_plan_aim_grip_response_callback(self, fut):
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
        # self.srv_gripper_control_send_distance(
        #     distance_mm=self.data_sorted_grasp_gripper_distance[self.passed_index]
        # )

    ## CLIENT: PLAN HOME ###########################################
    def command_plan_home(self):
        self.log("Planning to HOME")
        self.service_trigger_and_wait(self.client_home_plan)

    ## CLIENT: TRIGGER HOME ###########################################
    def command_trigger_home(self):
        self.log("Going to HOME")
        self.service_trigger_and_wait(self.client_trigger_home)

    ## CLIENT: ATTACH OBJECT ###########################################
    def command_attach_object(self):
        self.log("Attach Object")
        self.service_trigger_and_wait(self.client_attach)

    ## CLIENT: DETACH OBJECT ###########################################
    def command_detach_object(self):
        self.log("Detach ObjectE")
        self.service_trigger_and_wait(self.client_detach)

    ## CLIENT: GRIPPER CONTROL ########################################
    def srv_gripper_control_send_distance(self, distance_mm):
        if not self.client_gripper_control.wait_for_service(timeout_sec=3.0):
            self.elog("Service Gripper Control not available!")
            return

        request = Gripper.Request()
        request.gripper_distance = int(distance_mm)
        future = self.client_gripper_control.call_async(request)
        future.add_done_callback(
            self.srv_gripper_control_send_distance_response_callback
        )

    def srv_gripper_control_send_distance_response_callback(self, fut):
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
        self.call_make_stl_collision()

    ## CLIENT: FUSE POINTCLOUD ########################################
    def command_fuse_pointcloud(self):
        if not self.client_fuse_pointcloud.wait_for_service(timeout_sec=3.0):
            self.elog("Service Fuse Pointcloud not available!")
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                "zed_left_camera_optical_frame",
                "world",
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.5),
            )
        except TransformException as ex:
            self.log(f"Could not get transform for point cloud: {ex}")
            return

        transformed_pointcloud_fused, _ = utils.transform_pointcloud(
            msg=self.data_pointcloud_fused,
            tf=tf,
            frame_id="zed_left_camera_optical_frame",
        )

        self.pub_pointcloud_raw.publish(transformed_pointcloud_fused)

        request = PCLFuse.Request()
        request.pointcloud = transformed_pointcloud_fused
        request.camera_info = self.camera_info
        future = self.client_fuse_pointcloud.call_async(request)
        future.add_done_callback(self.command_fuse_pointcloud_response_callback)

    def command_fuse_pointcloud_response_callback(self, fut):
        try:
            result = fut.result()
            self.log(f"Fuse Pointcloud Success")

            # Convert to array
            try:
                array_depth_m = self.bridge.imgmsg_to_cv2(
                    result.depth_image, desired_encoding="32FC1"
                )
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

            # Save Image
            image_utils.save_depth_uint16(
                depth_maps=array_depth_mm,
                output_dir=os.path.join(
                    self.get_str_param("output_path_prefix"),
                    self.node_run_folder_name,
                    self.capture_folder_name,
                ),
                file_name="depth_fused",
            )
            self.data_array_depth = array_depth_mm
            self.data_msg_depth = result.depth_image

        except Exception as e:
            self.elog(f"Failed to send distance: -> {e}")

    ## CLEAR POINTCLOUD
    def command_clear_pointcloud(self):
        self.data_pointcloud_fused = PointCloud2()

    ## AUTOMATED POINTCLOUD FUSING ###########################
    def command_auto_pointcloud_fuse(self):
        # Get Planning Scene
        if not self.client_get_planning_scene.wait_for_service(timeout_sec=3.0):
            self.elog("Service /get_planning_scene not available!")
            return
        request_get_planning_scene = GetPlanningScene.Request()
        request_get_planning_scene.components.components = (
            PlanningSceneComponents.SCENE_SETTINGS
            | PlanningSceneComponents.ROBOT_STATE
            | PlanningSceneComponents.WORLD_OBJECT_NAMES
            | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        )
        future_get_planning_scene = self.client_get_planning_scene.call_async(
            request_get_planning_scene
        )
        future_get_planning_scene.add_done_callback(
            self.get_planning_scene_pcl_fuse_response_callback
        )

    def get_planning_scene_pcl_fuse_response_callback(self, future_get_planning_scene):
        try:
            result_get_planning_scene = future_get_planning_scene.result()
            self.data_planning_scene = result_get_planning_scene.scene
            self.log(
                f"Received Planning Scene of type: {type(self.data_planning_scene)}."
            )

            # Camera IK
            if not self.client_camera_ik_joint_state.wait_for_service(timeout_sec=3.0):
                self.elog("Service Camera IK Joint State not available!")
                return

            request_camera_joint_state = CameraIKJointState.Request()
            request_camera_joint_state.current_joint_state = (
                self.data_planning_scene.robot_state.joint_state
            )

            future_camera_joint_state = self.client_camera_ik_joint_state.call_async(
                request_camera_joint_state
            )
            future_camera_joint_state.add_done_callback(
                self.camera_joint_state_response_callback
            )
        except Exception as e:
            self.elog(f"Failed to get planning scene: -> {e}")

    # def camera_joint_state_response_callback(self, future_camera_joint_state):
    #     try:
    #         result_camera_joint_state = future_get_planning_scene.result()
    #         self.data_planning_scene = result_get_planning_scene.scene
    #         self.log(
    #             f"Received Planning Scene of type: {type(self.data_planning_scene)}."
    #         )

    #         # Camera IK
    #         if not self.client_camera_ik_joint_state.wait_for_service(timeout_sec=3.0):
    #             self.elog("Service Camera IK Joint State not available!")
    #             return

    #         request_camera_joint_state = CameraIKJointState.Request()
    #         request_camera_joint_state.current_joint_state = self.data_planning_scene.robot_state.joint_state

    #         future_camera_joint_state = self.client_camera_ik_joint_state.call_async(request_camera_joint_state)
    #         future_camera_joint_state.add_done_callback(self.camera_joint_state_response_callback)
    #     except Exception as e:
    #         self.elog(f"Failed to camera joint state: -> {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MainNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
