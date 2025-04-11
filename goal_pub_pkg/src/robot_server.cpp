#include <geometry_msgs/msg/pose_array.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "std_srvs/srv/trigger.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using Trigger = std_srvs::srv::Trigger;

static const std::string PLANNING_GROUP = "ur_arm";
static const rclcpp::Logger LOGGER = rclcpp::get_logger("robot_server");

// Globals
std::shared_ptr<rclcpp::Node> node;
std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools;
const moveit::core::JointModelGroup* joint_model_group;

// Global plan (shared between services)
moveit::planning_interface::MoveGroupInterface::Plan my_plan;

void handleRequest(const geometry_msgs::msg::PoseArray & pose_array)
{
  RCLCPP_INFO(LOGGER, "Received %zu poses, checking for a valid plan...", pose_array.poses.size());

  bool found_valid_plan = false;

  for (size_t i = 0; i < pose_array.poses.size(); ++i)
  {
    auto pose = pose_array.poses[i];

    RCLCPP_INFO(LOGGER, "Trying pose #%zu: [%.3f, %.3f, %.3f]",
                i,
                pose.position.x,
                pose.position.y,
                pose.position.z);

    // tf2::Quaternion q_orig, q_rot_y, q_result;
    // tf2::fromMsg(pose.orientation, q_orig);
  
    // // Define +90° rotation around Y-axis (in radians)
    // q_rot_y.setRPY(0, M_PI / 2, 0);  // Yaw = 0, Pitch = 90°, Roll = 0
    // q_result = q_orig * q_rot_y;     // 🔁 LOCAL Y rotation
  
    // q_result.normalize();
    // pose.orientation = tf2::toMsg(q_result);

    move_group->setPoseTarget(pose);

    bool success = (move_group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
      const moveit::core::LinkModel* ee_link_model =
        move_group->getRobotModel()->getLinkModel(move_group->getEndEffectorLink());

      visual_tools->publishTrajectoryLine(my_plan.trajectory_, ee_link_model, joint_model_group, rviz_visual_tools::LIME_GREEN);
      visual_tools->trigger();

      RCLCPP_INFO(LOGGER, "Selected pose #%zu for execution", i);
      found_valid_plan = true;
      break;
    }
  }

  if (!found_valid_plan)
  {
    RCLCPP_WARN(LOGGER, "No valid plan found for any pose.");
  }

  // RCLCPP_INFO(LOGGER, "Received pose: [%.3f, %.3f, %.3f]",
  //             request->target_pose.position.x,
  //             request->target_pose.position.y,
  //             request->target_pose.position.z);

  // move_group->setPoseTarget(request->target_pose);

  // bool success = (move_group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  // RCLCPP_INFO(LOGGER, "Planning result: %s", success ? "SUCCESS" : "FAILED");

  // if (!success) {
  //   response->success = false;
  //   response->message = "Planning failed";
  //   return;
  // }

  // const moveit::core::LinkModel* ee_link_model =
  //   move_group->getRobotModel()->getLinkModel(move_group->getEndEffectorLink());

  // visual_tools->publishTrajectoryLine(my_plan.trajectory_, ee_link_model, joint_model_group, rviz_visual_tools::LIME_GREEN);
  // visual_tools->trigger();

  // response->success = true;
  // response->message = "Planning successful";
  // return;
}

void handle_trigger_request(
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
  }
  res->success = true;
  res->message = "Motion executed successfully";

  visual_tools->deleteAllMarkers();
  visual_tools->trigger();

  RCLCPP_INFO(LOGGER, "Trigger service completed");
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

  auto pose_array_sub = node->create_subscription<geometry_msgs::msg::PoseArray>(
    "/main/best_grasp_poses", 10, handleRequest);

  auto trigger_service = node->create_service<Trigger>(
    "my_trigger_service", handle_trigger_request);

  RCLCPP_INFO(LOGGER, "Manipulation Server is ready.");

  spinner.join();
  rclcpp::shutdown();
  return 0;
}
