import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from rcl_interfaces.msg import Parameter, ParameterDescriptor, ParameterType
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener

from . import utils


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
        self.declare_parameter(
            "target_obj", "default_object", descriptor=param_descriptor
        )

        # Set a new value for the parameter
        # new_value = "mumu"
        # self.set_parameters(
        #     [
        #         rclpy.parameter.Parameter(
        #             "target_obj", rclpy.Parameter.Type.STRING, new_value
        #         )
        #     ]
        # )
        # self.get_logger().info(f"'target_obj' set to: {new_value}")

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

        # PUBLISHER ######################################
        self.pub_rviz_text = self.create_publisher(
            String, "/main/rviz_text", 10
        )

        self.pub_current_pointcloud = self.create_publisher(
            PointCloud2, "/main/current_pointcloud", 10
        )

        self.pub_collision = self.create_publisher(
            CollisionObject, "collision_object_topic", 10
        )

        self.data_pointcloud = None  # Only stores one when captured
        self.data_pointcloud_xyz = None
        self.capture_pointcloud = False

        self.log("Main Node is Running. Ready for command.")

    def log(self, text):
        # Log in terminal
        self.get_logger().info(text)
        # Pub to RViz
        string_msg = String()
        string_msg.data = text
        self.pub_rviz_text.publish(string_msg)


    def command_callback(self, msg: String):
        recv_command = msg.data.strip().lower()
        self.get_logger().info(f"Received command: {recv_command}")

        if recv_command == "capture":
            self.command_capture()

        elif recv_command == "make_collision":
            self.command_make_collision()

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
        if self.data_pointcloud_xyz is None:
            self.log("Cannot make Collision. Capture pointcloud first.")
            return

        collision_boxes_center = utils.pointcloud_xyz_to_simple_collision(
            self.data_pointcloud_xyz,
            max_distance=1.5,
            voxel_size=0.05,
            min_pcl_per_cube=1,
        )

        for idx, box_center in enumerate(collision_boxes_center):
            collision_object = CollisionObject()
            collision_object.header = Header()
            collision_object.header.frame_id = "base_link"
            collision_object.id = f"box_{idx}"

            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [0.05, 0.05, 0.05]

            pose = Pose()
            pose.position.x = float(box_center[0])
            pose.position.y = float(box_center[1])
            pose.position.z = float(box_center[2])
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0

            collision_object.primitives.append(box)
            collision_object.primitive_poses.append(pose)
            collision_object.operation = CollisionObject.ADD

            self.pub_collision.publish(collision_object)
        self.log(f"Published CollisionObject")


def main(args=None):
    rclpy.init(args=args)
    node = MainNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
