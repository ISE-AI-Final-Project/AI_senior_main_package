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


class MainNode(Node):
    def __init__(self):
        super().__init__("main")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # PARAM #########################################
        param_descriptor = ParameterDescriptor(
            name="target_obj",
            type=ParameterType.PARAMETER_STRING,
            description="A sample parameter",
        )
        self.declare_parameter("target_obj", "obj_10", descriptor=param_descriptor)

        # SUBSCRIBER ################################################
        self.sub_command = self.create_subscription(
            String, "/main/main_command", self.command_callback, 10
        )
        self.sub_pointcloud = self.create_subscription(
            PointCloud2,
            '/pointcloud_zed_topic',#"/zed/zed_node/point_cloud/cloud_registered",
            self.pointcloud_callback,
            10,
        )

        # PUBLISHER ######################################
        self.pub_rviz_text = self.create_publisher(String, "/main/rviz_text", 10)

        self.pub_current_pointcloud = self.create_publisher(
            PointCloud2, "/main/current_pointcloud", 10
        )

        self.pub_collision = self.create_publisher(
            CollisionObject, "collision_object_topic", 10
        )

        # CLIENT ########################################
        self.client_all_grasp = self.create_client(GraspPoseSend, "GraspPose")

        self.client_make_collision = self.create_client(
            PointCloudSend, "CollisionMaker"
        )

        self.client_ism = self.create_client(IMGSend, "image_service")

        # VARIABLE ######################################
        self.data_pointcloud = None  # Only stores one when captured
        self.data_pointcloud_xyz = None
        self.capture_pointcloud = False

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

    def command_callback(self, msg: String):
        """
        RViz command callback
        """
        recv_command = msg.data.strip().lower()
        self.get_logger().info(f"Received command: {recv_command}")

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

    def pointcloud_callback(self, msg: PointCloud2):
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

    def command_capture(self):
        """
        Capture Current Point Cloud in self.data_pointcloud
        """
        self.capture_pointcloud = True
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
        if not self.client_ism.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("Service ISM not available!")
            return
        self.log("TODO: REQ ISM")

    ## CLIENT: PEM ########################################
    def command_srv_req_pem(self):
        self.log("TODO: REQ PEM")

    ## CLIENT: ALL_GRASP ########################################
    def command_srv_all_grasp(self):
        if not self.client_all_grasp.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("Service All Grasp not available!")
            return

        self.log("I need mumu.")
        request = GraspPoseSend.Request()
        request.target_obj = (
            self.get_parameter("target_obj").get_parameter_value().string_value
        )
        self.log("I need mumu2.")

        future = self.client_all_grasp.call_async(request)
        future.add_done_callback(self.command_srv_all_grasp_response_callback)

    def command_srv_all_grasp_response_callback(self, future):
        print("im back")
        try:
            response = future.result()
            self.get_logger().info(f"Received response: {response}")
            num_poses = len(response.grasp_poses.grasp_poses)
            self.get_logger().info(f"Received {num_poses} grasp pose(s).")
        except Exception as e:
            self.get_logger().error(f"Failed to receive response: {str(e)}")

    ## CLIENT: BEST_GRASP########################################
    def command_srv_best_grasp(self):
        self.log("TODO: BEST GRASP")

    ## CLIENT: MAKE COLLISION ########################################
    def command_srv_make_collision(self):
        # Currently Non service
        # self.command_make_collision()

        # TODO: To service
        if not self.client_make_collision.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("Service Make Collision not available!")
            return

       # Should print <class 'sensor_msgs.msg._PointCloud2.PointCloud2'>

        request = PointCloudSend.Request()
        print(type(self.data_pointcloud )) 
        request.pointcloud =  self.data_pointcloud  # Correct field assignment

        future = self.client_make_collision.call_async(request)
        future.add_done_callback(self.command_srv_all_grasp_response_callback)

        # self.log("TODO: MAKE COLLISION")

    ## CLIENT: GRIPPER OPEN ########################################
    def command_gripper_open(self):
        self.log("TODO: GRIPPER OPEN")

    ## CLIENT: GRIPPER CLOSE ########################################
    def command_gripper_close(self):
        self.log("TODO: GRIPPER CLOSE")


def main(args=None):
    rclpy.init(args=args)
    node = MainNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
