#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Header
from visualization_msgs.msg import InteractiveMarkerPose, InteractiveMarkerUpdate


class InteractiveMarkerPublisher(Node):
    def __init__(self):
        super().__init__("interactive_marker_publisher")

        self.publisher_ = self.create_publisher(
            InteractiveMarkerUpdate,
            "/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update",
            10,
        )

    def pub_start(self):
        msg = InteractiveMarkerUpdate()
        msg.server_id = ""
        msg.seq_num = 42
        msg.type = InteractiveMarkerUpdate.UPDATE  # or INIT, depending on context

        marker_pose = InteractiveMarkerPose()
        marker_pose.header.stamp = self.get_clock().now().to_msg()
        marker_pose.header.frame_id = "world"

        marker_pose.pose.position.x = 0.27944934461756904
        marker_pose.pose.position.y = 0.2667428855272408
        marker_pose.pose.position.z = 0.7904300091824594
        marker_pose.pose.orientation.x = 0.7072732053451001
        marker_pose.pose.orientation.y = -0.7067190013436039
        marker_pose.pose.orientation.z = 4.0995060091141186e-05
        marker_pose.pose.orientation.w = -0.017687975016157462

        marker_pose.name = "EE:start_robotiq_hande_end"

        msg.poses.append(marker_pose)
        self.publisher_.publish(msg)
        self.get_logger().info("Published InteractiveMarkerUpdate with pose")

    def pub_goal(self):
        msg = InteractiveMarkerUpdate()
        msg.server_id = ""
        msg.seq_num = 43
        msg.type = InteractiveMarkerUpdate.UPDATE  # or INIT, depending on context

        marker_pose = InteractiveMarkerPose()
        marker_pose.header.stamp = self.get_clock().now().to_msg()
        marker_pose.header.frame_id = "world"

        marker_pose.pose.position.x = 0.27944934461756904
        marker_pose.pose.position.y = 0.2667428855272408
        marker_pose.pose.position.z = 0.7904300091824594
        marker_pose.pose.orientation.x = 0.7072732053451001
        marker_pose.pose.orientation.y = -0.7067190013436039
        marker_pose.pose.orientation.z = 4.0995060091141186e-05
        marker_pose.pose.orientation.w = -0.017687975016157462

        marker_pose.name = "EE:goal_robotiq_hande_end"

        msg.poses.append(marker_pose)
        self.publisher_.publish(msg)
        self.get_logger().info("Published InteractiveMarkerUpdate with pose")


def main(args=None):
    rclpy.init(args=args)
    node = InteractiveMarkerPublisher()
    node.pub_start()
    node.pub_goal()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
