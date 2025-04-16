#!/usr/bin/env python3

import os
import time
import rclpy
import trimesh
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from copy import deepcopy

from geometry_msgs.msg import Point, PoseStamped, Pose
from moveit_msgs.msg import CollisionObject, PlanningScene, AttachedCollisionObject
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from shape_msgs.msg import Mesh, MeshTriangle
from std_msgs.msg import Header
from std_srvs.srv import Trigger


class STLCollisionPublisher(Node):
    def __init__(self):
        super().__init__('stl_collision_publisher')

        # Use TRANSIENT_LOCAL durability to latch the planning scene update
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.publisher_ = self.create_publisher(PlanningScene, '/planning_scene', qos)

        self.pose_subscription = self.create_subscription(PoseStamped, '/main/object_pose', self.pose_callback, 10)
        self.remove_service = self.create_service(Trigger, 'remove_collision_object', self.remove_collision_callback)
        self.add_service = self.create_service(Trigger, 'add_collision_object', self.add_collision_callback)
        self.attach_service = self.create_service(Trigger, 'attach_collision_object', self.attach_collision_callback)
        self.detach_service = self.create_service(Trigger, 'detach_collision_object', self.detach_collision_callback)

        self.last_pose = None
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
        self.last_pose = msg.pose
        self.get_logger().info(
            f"Pose stored from /main/object_pose: "
            f"({msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z})"
        )

    def add_collision_callback(self, request, response):
        if self.last_pose is None:
            self.get_logger().warn("No pose received yet. Cannot add collision object.")
            response.success = False
            response.message = "No stored pose available"
            return response

        remove_collision = CollisionObject()
        remove_collision.id = self.target_obj
        remove_collision.header = Header(frame_id="world")
        remove_collision.operation = CollisionObject.REMOVE

        remove_scene = PlanningScene()
        remove_scene.is_diff = True
        remove_scene.world.collision_objects.append(remove_collision)
        self.publisher_.publish(remove_scene)
        time.sleep(0.2)

        add_collision = CollisionObject()
        add_collision.id = self.target_obj
        add_collision.header = Header(frame_id="world")
        add_collision.meshes = [self.ros_mesh]
        add_collision.mesh_poses = [self.last_pose]
        add_collision.operation = CollisionObject.ADD

        add_scene = PlanningScene()
        add_scene.is_diff = True
        add_scene.world.collision_objects.append(add_collision)
        self.publisher_.publish(add_scene)
        time.sleep(0.2)

        self.get_logger().info(f"Added object '{self.target_obj}' to planning scene at stored pose")
        response.success = True
        response.message = f"Object '{self.target_obj}' added to planning scene"
        return response

    def remove_collision_callback(self, request, response):
        self.get_logger().info(f"Trigger received: removing object '{self.target_obj}' from scene")

        collision = CollisionObject()
        collision.id = self.target_obj
        collision.header = Header(frame_id="world")
        collision.operation = CollisionObject.REMOVE

        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.world.collision_objects.append(collision)
        self.publisher_.publish(scene_msg)
        time.sleep(0.2)

        self.get_logger().info(f"Published REMOVE for object '{self.target_obj}'")
        response.success = True
        response.message = f"Removed object '{self.target_obj}' from planning scene"
        return response

    def attach_collision_callback(self, request, response):
        if self.last_pose is None:
            self.get_logger().warn("No pose received yet.")
            response.success = False
            response.message = "No pose to attach"
            return response
        frozen_pose = self.last_pose

        # Step 1: Remove from world
        remove = CollisionObject()
        remove.id = self.target_obj
        remove.header.frame_id = "world"
        remove.operation = CollisionObject.REMOVE

        remove_scene = PlanningScene()
        remove_scene.is_diff = True
        remove_scene.world.collision_objects.append(remove)
        self.publisher_.publish(remove_scene)
        time.sleep(0.5)

        # Step 2: Attach to gripper
        attached = AttachedCollisionObject()
        attached.link_name = "robotiq_hande_end"  # <- confirm this is your actual EE link
        attached.object.id = self.target_obj
        attached.object.header.frame_id = 'world'
        attached.object.meshes = [self.ros_mesh]

        attached.object.mesh_poses = [frozen_pose]

        attached.object.operation = CollisionObject.ADD
        attached.touch_links = ["robotiq_hande_left_finger_link", "robotiq_hande_right_finger_link"]

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.attached_collision_objects.append(attached)
        self.publisher_.publish(scene)
        time.sleep(0.2)

        self.get_logger().info(f"Attached '{self.target_obj}' to {attached.link_name}")
        response.success = True
        response.message = "Object attached to gripper"
        return response
    
    def detach_collision_callback(self, request, response):
        self.get_logger().info(f"Detaching and removing object '{self.target_obj}' from the planning scene")

        # Step 1: Detach from robot (only id and link_name are needed)
        detached = AttachedCollisionObject()
        detached.object.id = self.target_obj
        detached.link_name = "robotiq_hande_end"
        detached.object.operation = CollisionObject.REMOVE

        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.robot_state.attached_collision_objects.append(detached)
        self.publisher_.publish(planning_scene)
        time.sleep(0.2)

        # Step 2: Remove from world (optional cleanup in case object appears in world)
        collision = CollisionObject()
        collision.id = self.target_obj
        collision.header.frame_id = "world"
        collision.operation = CollisionObject.REMOVE

        cleanup_scene = PlanningScene()
        cleanup_scene.is_diff = True
        cleanup_scene.world.collision_objects.append(collision)
        self.publisher_.publish(cleanup_scene)
        time.sleep(0.2)

        self.get_logger().info(f"Fully detached and removed '{self.target_obj}' from robot and world")
        response.success = True
        response.message = f"Object '{self.target_obj}' detached and removed"
        return response
            
def main():
    rclpy.init()
    node = STLCollisionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

