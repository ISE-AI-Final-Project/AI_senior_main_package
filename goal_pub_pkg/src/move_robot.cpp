#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_trajectory.h>
#include <geometry_msgs/msg/pose.h>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("move_ur_manipulator");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    // Define the MoveGroupInterface for the UR manipulator
    static const std::string PLANNING_GROUP = "ur_arm";
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

    // Define the Planning Scene Interface
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Set the target pose
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.6;
    target_pose.position.y = 0.6;
    target_pose.position.z = 1.6;
    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 1.0;

    move_group.setPoseTarget(target_pose);

    // Plan motion
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
        RCLCPP_INFO(node->get_logger(), "Motion plan successful, executing...");
        move_group.execute(my_plan);
    } else {
        RCLCPP_ERROR(node->get_logger(), "Motion planning failed");
    }

    rclcpp::shutdown();
    return 0;
}
