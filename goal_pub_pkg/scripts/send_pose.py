#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from visualization_msgs.msg import InteractiveMarkerFeedback, InteractiveMarkerPose

from custom_srv_pkg.srv import TCPPose


class TCPPoseClient(Node):
    def __init__(self):
        super().__init__("tcp_pose_client")
        self.client = self.create_client(TCPPose, "tcp_pose_service")
        self.publisher_marker_feedback = self.create_publisher(
            InteractiveMarkerFeedback,
            "/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback",
            10,
        )

        while not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Waiting for service...")

        self.send_request()


    def send_request(self):
        position = input("Enter Pose : ")
        x, y, z = map(float, position.split())

        request = TCPPose.Request()
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        target_pose.orientation.x = 0.0
        target_pose.orientation.y = 0.0
        target_pose.orientation.z = 0.0
        target_pose.orientation.w = 1.0
        request.target_pose = target_pose

        self.target_pose = target_pose

        self.get_logger().info("Sending target pose request...")
        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f"Service Response: Success - {response.message}"
                )
                # Update Marker
                self.pub_marker_goal(self.target_pose)

            else:
                self.get_logger().error(
                    f"Service Response: Failed - {response.message}"
                )
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
        finally:
            rclpy.shutdown()

    def pub_marker_goal(self, pose: Pose):
        msg = InteractiveMarkerFeedback()
        msg.header.frame_id = "world"
        msg.client_id = ""
        msg.marker_name = "EE:goal_robotiq_hande_end"
        msg.control_name = ""
        msg.event_type = 1
        msg.pose = pose
        msg.mouse_point_valid = False

        self.publisher_marker_feedback.publish(msg)
        self.get_logger().info("Published InteractiveMarkerFeedback with pose")


def main():
    rclpy.init()
    node = TCPPoseClient()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
