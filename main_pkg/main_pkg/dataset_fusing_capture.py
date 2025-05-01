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
            "fuse_pointcloud": self.command_fuse_pointcloud,
            "clear_pointcloud": self.command_clear_pointcloud,
            "auto_fuse_pointcloud": self.command_auto_fuse_pointcloud,
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
        self.capture_index = None

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

        self.client_move_joint_pose = self.create_client(JointPose, "move_joint_pose")

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

        # if recv_command in self.command_map:
        #     self.command_map[recv_command]()
        # else:
        #     self.get_logger().warn(f"Unknown command received: {recv_command}")

        if recv_command in self.command_map:
            command_func = self.command_map[recv_command]
            if asyncio.iscoroutinefunction(command_func):
                # Run coroutine in event loop
                asyncio.run_coroutine_threadsafe(command_func(), self.loop)
            else:
                command_func()

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
                file_name=(f"rgb_{self.capture_index}", "rgb")[self.capture_index is None],
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
                file_name=(f"depth_{self.capture_index}", "depth")[self.capture_index is None],
            )

            self.log("Captured depth.")

    def command_capture(self, new_folder=True):
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

        if new_folder:
            self.capture_folder_name = f"Cap-{self.get_current_time()}"

        self.capture_pointcloud = True
        self.capture_rgb = True
        self.capture_depth = True
        self.capture_camera_info = True
        self.log("Ready to capture next pointcloud.")

    def command_capture_to_fuse(self, new_folder=False):
        self.capture_to_fuse = True
        self.command_capture(new_folder=new_folder)

    ## CLIENT: FUSE POINTCLOUD ########################################
    async def command_fuse_pointcloud(self):
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

        # Call
        fuse_pointcloud_response = await self.call_service(
            self.client_fuse_pointcloud, request
        )
        if not fuse_pointcloud_response:
            return

        # Response
        try:
            array_depth_m = self.bridge.imgmsg_to_cv2(
                fuse_pointcloud_response.depth_image, desired_encoding="32FC1"
            )
        except Exception as e:
            self.elog(f"CVBridge error: {e}")
            return

        # Replace NaNs with 0
        array_depth_m = np.nan_to_num(array_depth_m, nan=0.0, posinf=0.0, neginf=0.0)

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
        self.data_msg_depth = fuse_pointcloud_response.depth_image

        self.log("Fuse PointCloud Success")

    ## CLEAR POINTCLOUD
    def command_clear_pointcloud(self):
        self.data_pointcloud_fused = PointCloud2()

    ## AUTOMATED POINTCLOUD FUSING ###########################
    async def command_auto_fuse_pointcloud(self):
        self.data_pointcloud_fused = PointCloud2()
        # Get Planning Scene
        if not self.client_get_planning_scene.wait_for_service(timeout_sec=3.0):
            self.elog("Service /get_planning_scene not available!")
            return
        if not self.client_camera_ik_joint_state.wait_for_service(timeout_sec=3.0):
            self.elog("Service Camera IK Joint State not available!")
            return

        if not self.client_move_joint_pose.wait_for_service(timeout_sec=3.0):
            self.elog("Service Move Joint Pose not available!")
            return

        request_get_planning_scene = GetPlanningScene.Request()
        request_get_planning_scene.components.components = (
            PlanningSceneComponents.SCENE_SETTINGS
            | PlanningSceneComponents.ROBOT_STATE
            | PlanningSceneComponents.WORLD_OBJECT_NAMES
            | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        )

        # Call Get Planning Scene
        planning_scene_response = await self.call_service(
            self.client_get_planning_scene, request_get_planning_scene
        )
        if not planning_scene_response:
            return

        # Response Planning Scene
        self.data_planning_scene = planning_scene_response.scene
        self.log(f"Received Planning Scene of type: {type(self.data_planning_scene)}.")

        # Camera IK
        request_camera_joint_state = CameraIKJointState.Request()
        request_camera_joint_state.current_joint_state = (
            self.data_planning_scene.robot_state.joint_state
        )

        # Call Camera IK
        camera_ik_joint_state_response = await self.call_service(
            self.client_camera_ik_joint_state, request_camera_joint_state
        )
        if not camera_ik_joint_state_response:
            return

        # Response
        self.log(
            f"Received Camera IK Response with len: {len(camera_ik_joint_state_response.moving_joint_state)}."
        )

        for (
            camera_capture_joint_state
        ) in camera_ik_joint_state_response.moving_joint_state:

            req_move_joint = JointPose.Request()
            req_move_joint.joint_state = camera_capture_joint_state

            self.log(f"Moving To : {req_move_joint.joint_state}")

            # Call
            move_joint_response = await self.call_service(
                self.client_move_joint_pose, req_move_joint
            )
            if not move_joint_response:
                return

            # Response
            self.log(f"Move Joint -> success : {move_joint_response.success}")

            await asyncio.sleep(1)

            if self.capture_index is None:
                self.capture_index = 0
                self.command_capture_to_fuse(new_folder=True)
            else:
                self.capture_index += 1
                self.command_capture_to_fuse(new_folder=False)
            # break
        
        await asyncio.sleep(2)
        await self.command_fuse_pointcloud()

        self.capture_index = None

    ## CLIENT: PLAN HOME ###########################################
    def command_plan_home(self):
        self.log("Planning to HOME")
        self.service_trigger_and_wait(self.client_home_plan)

    ## CLIENT: TRIGGER HOME ###########################################
    def command_trigger_home(self):
        self.log("Going to HOME")
        self.service_trigger_and_wait(self.client_trigger_home)


def main(args=None):
    rclpy.init(args=args)
    node = MainNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
