import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit_msgs.msg import CollisionObject
from rcl_interfaces.msg import Parameter, ParameterDescriptor, ParameterType
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformException, TransformListener

from custom_srv_pkg.msg import GraspPose, GraspPoses
from custom_srv_pkg.srv import (
    BestGraspPose,
    GraspPoseSend,
    Gripper,
    IMGSend,
    PointCloudSend,
)
from main_pkg.my_custom_socket import MyClient
from main_pkg.utils import utils

from .utils import utils


class MainNode(Node):
    def __init__(self):
        super().__init__("main")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        """
        ################### VARIABLE ###########################
        """
        # Point Cloud
        self.data_pointcloud = None
        self.data_pointcloud_xyz = None
        self.capture_pointcloud = False

        # RGB
        self.data_msg_rgb = None
        self.data_array_rgb = None
        self.capture_rgb = False

        # Depth
        self.data_msg_depth = None
        self.data_array_depth = None
        self.capture_depth = False

        # Mask
        self.data_mask = None

        # Pose
        self.data_object_pose = None
        self.data_all_grasp_pose = None
        self.data_best_grasp_pose = None

        """
        ################### PARAM ###########################
        """
        param_descriptor = ParameterDescriptor(
            name="target_obj",
            type=ParameterType.PARAMETER_STRING,
            description="A sample parameter",
        )
        self.declare_parameter("target_obj", "sunscreen", descriptor=param_descriptor)

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

        self.pub_current_pointcloud = self.create_publisher(
            PointCloud2, "/main/current_pointcloud", 10
        )

        self.pub_collision = self.create_publisher(
            CollisionObject, "collision_object_topic", 10
        )

        self.data_pointcloud = None  # Only stores one when captured
        self.data_pointcloud_xyz = None
        self.capture_pointcloud = False
        self.data_all_grasp_pose = [1,2,3]
        self.data_object_pose = None

        """
        ################### CLIENT ###########################
        """
        self.client_gripper_control = self.create_client(Gripper, 'gripper_control')
        
        self.client_all_grasp = self.create_client(GraspPoseSend, "GraspPose")

        self.client_best_grasp = self.create_client(BestGraspPose, "BestGraspPose")

        self.client_make_collision = self.create_client(
            PointCloudSend, "CollisionMaker"
        )

        # self.client_ism = self.create_client(IMGSend, "image_service")

        # Socket Client
        self.client_ism = MyClient(host="127.0.0.1", port=65432)
        self.client_pem = MyClient(host="127.0.0.1", port=99999)

        # Finish Init
        self.log("Main Node is Running. Ready for command.")

    def log(self, text):
        """
        Log Text into Terminal and RViz label
        """
        # Log in terminal
        self.get_logger().info(text)
        # Pub to RViz
        string_msg = String()
        string_msg.data = text
        self.pub_rviz_text.publish(string_msg)

    def elog(self, text):
        """
        Log Error Text into Terminal and RViz label
        """
        # Log in terminal
        self.get_logger().error(text)
        # Pub to RViz
        string_msg = String()
        string_msg.data = text
        self.pub_rviz_text.publish(string_msg)

    def command_callback(self, msg: String):
        """
        RViz command callback
        """
        recv_command = msg.data.strip().lower()
        self.log(f"Received command: {recv_command}")

        if recv_command == "capture":
            self.command_capture()

        elif recv_command == "req_ism":
            self.command_srv_req_ism()

        elif recv_command == "req_pem":
            self.command_srv_req_pem()

        elif recv_command == "generate_all_grasp":
            self.command_srv_all_grasp()

        elif recv_command == "generate_best_grasp":
            self.command_srv_best_grasp()

        elif recv_command == "make_collision":
            self.command_srv_make_collision()

        elif recv_command == "gripper_open":
            self.command_gripper_open()

        elif recv_command == "gripper_close":
            self.command_gripper_close()

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

            self.pub_current_pointcloud.publish(transformed_pointcloud)

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
                array_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception as e:
                self.elog(f"CVBridge error: {e}")
                return
            self.data_array_rgb = array_rgb
            self.capture_rgb = False
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
            self.log("Captured depth.")

    def command_capture(self):
        """
        Capture Current Point Cloud in self.data_pointcloud
        """
        self.capture_pointcloud = True
        self.capture_rgb = True
        self.capture_depth = True
        self.log("Ready to capture next pointcloud.")

    def command_make_collision(self):
        """
        Make Collision from self.data_pointcloud_xyz
        """

        if self.data_pointcloud  is None:
            self.log("Cannot make Collision. Capture pointcloud first.")
            return

        # for idx, box_center in enumerate(collision_boxes_center):
        #     collision_object = CollisionObject()
        #     collision_object.header = Header()
        #     collision_object.header.frame_id = "base_link"
        #     collision_object.id = f"box_{idx}"

        #     box = SolidPrimitive()
        #     box.type = SolidPrimitive.BOX
        #     box.dimensions = [0.05, 0.05, 0.05]

        #     pose = Pose()
        #     pose.position.x = float(box_center[0])
        #     pose.position.y = float(box_center[1])
        #     pose.position.z = float(box_center[2])
        #     pose.orientation.x = 0.0
        #     pose.orientation.y = 0.0
        #     pose.orientation.z = 0.0
        #     pose.orientation.w = 1.0

        #     collision_object.primitives.append(box)
        #     collision_object.primitive_poses.append(pose)
        #     collision_object.operation = CollisionObject.ADD

        #     self.pub_collision.publish(collision_object)
        # self.log(f"Published CollisionObject")

    ## CLIENT: ISM ########################################
    def command_srv_req_ism(self):
        if self.data_array_rgb is None or self.data_array_depth is None:
            self.log("No RGB or Depth Data.")
            return

        client_connected = self.client_ism.connect()
        if not client_connected:
            self.elog(f"Server not started")
            return

        # Send request
        try:
            result = self.client_ism.request_msg(
                msg_type_in=["numpyarray", "numpyarray", "string"],
                msg_in=[
                    self.data_array_rgb,
                    self.data_array_depth,
                    self.get_parameter("target_obj").get_parameter_value().string_value,
                ],
            )
            self.log(f"Response: {result}")
        except Exception as e:
            self.elog(f"Failed to send request: {e}")

        self.client_ism.disconnect()

    ## CLIENT: PEM ########################################
    def command_srv_req_pem(self):
        self.log("TODO: REQ PEM")

    ## CLIENT: ALL_GRASP ########################################
    def command_srv_all_grasp(self):
        if not self.client_all_grasp.wait_for_service(timeout_sec=3.0):
            self.elog("Service All Grasp not available!")
            return

        request = GraspPoseSend.Request()
        request.target_obj = (
            self.get_parameter("target_obj").get_parameter_value().string_value
        )

        future = self.client_all_grasp.call_async(request)
        future.add_done_callback(self.command_srv_all_grasp_response_callback)

    def command_srv_all_grasp_response_callback(self, future):
        try:
            response = future.result()
            self.log(f"Received response: {response}")
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
        
        if self.data_all_grasp_pose is None:
            self.elog("NO all grasp data")
            return

        # self.data_all_grasp_pose = []

        # Create pose
        # pose = PoseStamped()
        # pose.header.frame_id = "world"
        # pose.pose.position.x = 0.00056425
        # pose.pose.position.y = -0.01169335
        # pose.pose.position.z = -0.00061085
        # pose.pose.orientation.x = -0.018395
        # pose.pose.orientation.y = 0.000926
        # pose.pose.orientation.z = -0.178108
        # pose.pose.orientation.w = 0.983798

        # Create GraspPose and assign pose
        # g = GraspPose()
        # g.gripper_score = 483.00
        # g.d_to_com = 0.012366076006890686
        # g.ht_in_meter = pose.pose
        # self.data_all_grasp_pose.append(g)

        self.data_object_pose = PoseStamped()
        self.data_object_pose.header.frame_id = "world"
        self.data_object_pose.pose.position.x = 1.0
        self.data_object_pose.pose.position.y = 1.0
        self.data_object_pose.pose.position.z = 1.0
        self.data_object_pose.pose.orientation.x = 0.0
        self.data_object_pose.pose.orientation.y = 0.0
        self.data_object_pose.pose.orientation.z = 0.0
        self.data_object_pose.pose.orientation.w = 1.0

        # Wrap in GraspPoses for service
        grasp_msg = GraspPoses()
        # grasp_msg.grasp_poses = self.data_all_grasp_pose
        request = BestGraspPose.Request()
        # request.all_grasp_poses = grasp_msg
        request.all_grasp_poses = self.data_all_grasp_pose

        request.object_pose = self.data_object_pose  # assuming this is already defined

        self.get_logger().info(f"Sending {len(request.all_grasp_poses.grasp_poses)} grasp poses.")
        self.log("I need mumu2.")

        future = self.client_best_grasp.call_async(request)
        future.add_done_callback(self.command_srv_best_grasp_response_callback)
        
            
        pose = PoseStamped()
        pose.header.frame_id = "world"  # or your actual TF frame
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = 1.0
        pose.pose.position.y = 1.0
        pose.pose.position.z = 1.0

        # Identity orientation (no rotation)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        self.data_object_pose = pose

        request = BestGraspPose.Request()
        request.all_grasp_poses.grasp_poses = self.data_all_grasp_pose
        request.object_pose = (
            self.data_object_pose
        )  # Replace with your actual object pose

        self.log(f"Sending {len(request.all_grasp_poses.grasp_poses)} grasp poses.")

        future = self.client_best_grasp.call_async(request)
        future.add_done_callback(self.command_srv_best_grasp_response_callback)

    def command_srv_best_grasp_response_callback(self, future):
        try:
            response = future.result()
            best_grasp = response.best_grasp_pose

            self.log(
                f"Received best grasp pose: x={best_grasp.pose.position.x:.3f}, "
                f"y={best_grasp.pose.position.y:.3f}, z={best_grasp.pose.position.z:.3f}"
            )
        except Exception as e:
            self.elog(f"Failed to receive response: {str(e)}")

    ## CLIENT: MAKE COLLISION ########################################
    def command_srv_make_collision(self):
        if not self.client_make_collision.wait_for_service(timeout_sec=3.0):
            self.elog("Service Make Collision not available!")
            return

        request = PointCloudSend.Request()
        print(type(self.data_pointcloud )) 
        request.pointcloud =  self.data_pointcloud  # Correct field assignment

        future = self.client_make_collision.call_async(request)
        future.add_done_callback(self.command_srv_all_grasp_response_callback)


    ## CLIENT: GRIPPER CONTROL ########################################
    def srv_gripper_control_send_distance(self, distance_mm):
        if not self.client_gripper_control.wait_for_service(timeout_sec=3.0):
            self.elog("Service Gripper Control not available!")
            return
        
        request = Gripper.Request()
        request.gripper_distance = distance_mm
        future = self.client_gripper_control.call_async(request)
        future.add_done_callback(self.srv_gripper_control_send_distance_respone_callback)

    def srv_gripper_control_send_distance_respone_callback(self, fut):
        try:
            result = fut.result()
            self.log(f"Sent distance Success")
        except Exception as e:
            self.elog(f"Failed to send distance: -> {e}")

    ## CLIENT: GRIPPER OPEN ###########################################
    def command_gripper_open(self):
        self.srv_gripper_control_send_distance(distance_mm=55)

    ## CLIENT: GRIPPER CLOSE ########################################
    def command_gripper_close(self):
        self.srv_gripper_control_send_distance(distance_mm=0)


def main(args=None):
    rclpy.init(args=args)
    node = MainNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
