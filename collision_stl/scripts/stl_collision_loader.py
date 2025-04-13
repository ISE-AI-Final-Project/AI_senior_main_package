#!/usr/bin/env python3

import os

import rclpy
import trimesh
from geometry_msgs.msg import Point, PoseStamped
from moveit_msgs.msg import CollisionObject, PlanningScene
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from shape_msgs.msg import Mesh, MeshTriangle
from std_msgs.msg import Header


class STLCollisionPublisher(Node):
    def __init__(self):
        super().__init__('stl_collision_publisher')
        self.publisher_ = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.pose_subscription = self.create_subscription(PoseStamped, '/main/object_pose', self.pose_callback, 10)

        self.target_obj = self.get_param_from_main_node('target_obj')
        self.dataset_path_prefix = self.get_param_from_main_node('dataset_path_prefix')
        self.get_logger().info(f"Using object: {self.target_obj}")

        self.mesh_path = os.path.join(self.dataset_path_prefix, self.target_obj, f"{self.target_obj}_centered.stl") 
        self.ros_mesh = self.load_and_convert_mesh(self.mesh_path)

    def get_param_from_main_node(self, param_name):
        client = self.create_client(GetParameters, '/main_node/get_parameters')
        while not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting for /main_node/get_parameters service...")

        request = GetParameters.Request()
        request.names = [param_name]

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            return future.result().values[0].string_value
        else:
            self.get_logger().error(f"Failed to retrieve parameter '{param_name}' from /main_node.")
            return "default_object"

    def load_and_convert_mesh(self, path):
        self.get_logger().info(f"Loading mesh from: {path}")
        mesh = trimesh.load_mesh(path)

        if isinstance(mesh, trimesh.Scene):
            mesh = trimesh.util.concatenate(mesh.dump())
        elif not isinstance(mesh, trimesh.Trimesh):
            mesh = mesh.dump().sum()

        mesh.apply_scale(0.001)
        self.get_logger().info(f"Mesh loaded with {len(mesh.vertices)} vertices and {len(mesh.faces)} faces")

        ros_mesh = Mesh()
        for face in mesh.faces:
            ros_mesh.triangles.append(MeshTriangle(vertex_indices=face))
        for vertex in mesh.vertices:
            ros_mesh.vertices.append(Point(x=vertex[0], y=vertex[1], z=vertex[2]))
        return ros_mesh

    def pose_callback(self, msg: PoseStamped):
        self.get_logger().info(
            f"Received pose from /main/object_pose: "
            f"position=({msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z})"
        )

        collision = CollisionObject()
        collision.id = self.target_obj
        collision.header = Header(frame_id="world")
        collision.meshes = [self.ros_mesh]
        collision.mesh_poses = [msg.pose]

        remove_msg = PlanningScene()
        remove_msg.is_diff = True
        collision.operation = CollisionObject.REMOVE
        remove_msg.world.collision_objects.append(collision)
        self.publisher_.publish(remove_msg)
        self.get_logger().info(f"Removing previous object with id '{self.target_obj}'...")

        collision.operation = CollisionObject.ADD
        add_msg = PlanningScene()
        add_msg.is_diff = True
        add_msg.world.collision_objects.append(collision)
        self.publisher_.publish(add_msg)
        self.get_logger().info(f"Published object '{self.target_obj}' to /planning_scene at new pose")


def main():
    rclpy.init()
    node = STLCollisionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
