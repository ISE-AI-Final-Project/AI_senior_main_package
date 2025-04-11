#include <geometry_msgs/msg/pose_array.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "std_srvs/srv/trigger.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "custom_srv_pkg/srv/goal_grip_plan.hpp"
#include "custom_srv_pkg/srv/goal_grip_plan.hpp"

using Trigger = std_srvs::srv::Trigger;
using GoalGripPlan = custom_srv_pkg::srv::GoalGripPlan;

static const std::string PLANNING_GROUP = "ur_arm";
static const rclcpp::Logger LOGGER = rclcpp::get_logger("robot_server");

std::shared_ptr<rclcpp::Node> node;
std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools;
const moveit::core::JointModelGroup* joint_model_group;

geometry_msgs::msg::Pose successful_goal_pose;
geometry_msgs::msg::Pose successful_grip_pose;
bool goal_executed = false;

moveit::planning_interface::MoveGroupInterface::Plan my_plan;

std::map<std::string, double> home_joint_values = {
  {"shoulder_pan_joint", 0},
  {"shoulder_lift_joint", -1.570},
  {"elbow_joint", 0},
  {"wrist_1_joint", -1.570},
  {"wrist_2_joint", 0},
  {"wrist_3_joint", 0}
};



void handle_goal_grip_request(
  const std::shared_ptr<GoalGripPlan::Request> request,
  std::shared_ptr<GoalGripPlan::Response> response)
{
  const auto& goal_poses = request->sorted_goal_poses.poses;
  const auto& grip_poses = request->sorted_grip_poses.poses;

  if (goal_poses.size() != grip_poses.size()) {
    RCLCPP_WARN(LOGGER, "Goal and grip pose arrays must have the same size.");
    response->passed_index = -1;
    return;
  }

  for (size_t i = 0; i < goal_poses.size(); ++i) {
    RCLCPP_INFO(LOGGER, "Checking pose pair #%zu", i);

    move_group->setPoseTarget(grip_poses[i]);
    bool plan_goal = (move_group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!plan_goal) continue;
    const moveit::core::LinkModel* ee_link_model =
      move_group->getRobotModel()->getLinkModel(move_group->getEndEffectorLink());
    visual_tools->publishTrajectoryLine(my_plan.trajectory_, ee_link_model, joint_model_group, rviz_visual_tools::LIME_GREEN);
    visual_tools->trigger();

    move_group->setPoseTarget(goal_poses[i]);
    bool plan_grip = (move_group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!plan_grip) continue;
    RCLCPP_INFO(LOGGER, "Valid pair found at index %zu", i);
    successful_goal_pose = goal_poses[i];
    successful_grip_pose = grip_poses[i];
    visual_tools->publishTrajectoryLine(my_plan.trajectory_, ee_link_model, joint_model_group, rviz_visual_tools::LIME_GREEN);
    visual_tools->trigger();

    response->passed_index = static_cast<int8_t>(i);
    return;
  }

  RCLCPP_WARN(LOGGER, "No valid plan for any pair of poses.");
  response->passed_index = -1;
}

void handle_goal_trigger_request(
  const std::shared_ptr<Trigger::Request> req,
  std::shared_ptr<Trigger::Response> res)
{
  (void)req;

  moveit::core::MoveItErrorCode exec_result = move_group->execute(my_plan);
  if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "Execution failed");
    res->success = false;
    res->message = "Execution failed";
  } else {
    RCLCPP_INFO(LOGGER, "Motion executed successfully");
    res->success = true;
    res->message = "Motion executed successfully";
    goal_executed = true;
  }
}

void handle_grip_trigger_request(
  const std::shared_ptr<Trigger::Request> req,
  std::shared_ptr<Trigger::Response> res)
{
  (void)req;

  if (!goal_executed) {
    RCLCPP_ERROR(LOGGER, "Grip trigger called before goal was executed. Motion not allowed.");
    res->success = false;
    res->message = "Goal motion not executed. Grip motion aborted.";
    return;
  }

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(successful_goal_pose);
  waypoints.push_back(successful_grip_pose);

  moveit_msgs::msg::RobotTrajectory trajectory_msg;
  const double eef_step = 0.01;
  const double jump_threshold = 0.0;
  double fraction = 0.0;

  RCLCPP_INFO(LOGGER, "Computing Cartesian path from goal to grip pose...");

  while (fraction < 1.0) {
    fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory_msg);
    if (fraction < 1.0) {
      RCLCPP_WARN(LOGGER, "Cartesian path incomplete (%.2f%%), retrying...", fraction * 100.0);
      rclcpp::sleep_for(std::chrono::milliseconds(200));
    }
  }

  moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
  cartesian_plan.trajectory_ = trajectory_msg;

  RCLCPP_INFO(LOGGER, "Cartesian path succeeded. Executing...");
  moveit::core::MoveItErrorCode exec_result = move_group->execute(cartesian_plan);

  if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "Grip Cartesian execution failed.");
    res->success = false;
    res->message = "Grip Cartesian execution failed.";
    return;
  }

  RCLCPP_INFO(LOGGER, "Grip Cartesian motion executed successfully.");
  res->success = true;
  res->message = "Grip Cartesian motion executed successfully.";
  goal_executed = false;

  visual_tools->deleteAllMarkers();
  visual_tools->trigger();
}

void handle_home_trigger_request(
  const std::shared_ptr<Trigger::Request> req,
  std::shared_ptr<Trigger::Response> res)
{
  (void)req;

  RCLCPP_INFO(LOGGER, "Setting target joint values for home position...");

  move_group->setJointValueTarget(home_joint_values);

  moveit::planning_interface::MoveGroupInterface::Plan home_plan;
  bool success = (move_group->plan(home_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!success) {
    RCLCPP_ERROR(LOGGER, "Planning to home joint state failed.");
    res->success = false;
    res->message = "Planning failed.";
    return;
  }

  RCLCPP_INFO(LOGGER, "Planning succeeded. Executing...");
  moveit::core::MoveItErrorCode exec_result = move_group->execute(home_plan);
  if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "Execution to home position failed.");
    res->success = false;
    res->message = "Execution failed.";
    return;
  }

  RCLCPP_INFO(LOGGER, "Moved to home position.");
  res->success = true;
  res->message = "Moved to home position successfully.";

  visual_tools->deleteAllMarkers();
  visual_tools->trigger();
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node = rclcpp::Node::make_shared("robot_server", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, PLANNING_GROUP);
  move_group->setMaxVelocityScalingFactor(0.3);
  move_group->setMaxAccelerationScalingFactor(0.3);
  visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
    node, "base_link", "move_robot", move_group->getRobotModel());

  joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  visual_tools->deleteAllMarkers();
  visual_tools->trigger();

  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group->getPlanningFrame().c_str());
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group->getEndEffectorLink().c_str());

  auto goal_grip_plan_service = node->create_service<GoalGripPlan>(
    "GoalGripPlan", handle_goal_grip_request);

  auto goal_trigger_service = node->create_service<Trigger>(
    "goal_trigger_service", handle_goal_trigger_request);

  auto grip_trigger_service = node->create_service<Trigger>(
    "grip_trigger_service", handle_grip_trigger_request);

  auto home_trigger_service = node->create_service<Trigger>(
    "home_trigger_service", handle_home_trigger_request);

  RCLCPP_INFO(LOGGER, "Manipulation Server is ready.");

  spinner.join();
  rclcpp::shutdown();
  return 0;
}