#include <geometry_msgs/msg/pose_array.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "std_srvs/srv/trigger.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "custom_srv_pkg/srv/aim_grip_plan.hpp"
#include "custom_srv_pkg/srv/joint_pose.hpp"

using Trigger = std_srvs::srv::Trigger;
using AimGripPlan = custom_srv_pkg::srv::AimGripPlan;
using JointPose = custom_srv_pkg::srv::JointPose;

static const std::string PLANNING_GROUP = "ur_arm";
static const rclcpp::Logger LOGGER = rclcpp::get_logger("robot_server");

std::shared_ptr<rclcpp::Node> node;
std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools;
const moveit::core::JointModelGroup *joint_model_group;

geometry_msgs::msg::Pose successful_aim_pose;
geometry_msgs::msg::Pose successful_grip_pose;
bool aim_executed = false;
bool double_plan = false;
bool home_plan = false;

moveit::planning_interface::MoveGroupInterface::Plan my_plan;
moveit::planning_interface::MoveGroupInterface::Plan home_plan_global;

rclcpp::Client<Trigger>::SharedPtr client_add;
rclcpp::Client<Trigger>::SharedPtr client_remove;
rclcpp::Client<Trigger>::SharedPtr client_attach;
rclcpp::Client<Trigger>::SharedPtr client_detach;



std::map<std::string, double> home_joint_values = {
    {"shoulder_pan_joint", -1.570},
    {"shoulder_lift_joint", -1.570},
    {"elbow_joint", 0},
    {"wrist_1_joint", -1.570},
    {"wrist_2_joint", 0},
    {"wrist_3_joint", 0}};

std::set<std::string> allowed_joints = {
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
};

// void handle_aim_grip_request(
//     const std::shared_ptr<AimGripPlan::Request> request,
//     std::shared_ptr<AimGripPlan::Response> response)
// {
//     const auto &aim_poses = request->sorted_aim_poses.poses;
//     const auto &grip_poses = request->sorted_grip_poses.poses;

//     if (aim_poses.size() != grip_poses.size())
//     {
//         RCLCPP_WARN(LOGGER, "Aim and grip pose arrays must have the same size.");
//         response->passed_index = -1;
//         return;
//     }

//     for (size_t i = 0; i < aim_poses.size(); ++i)
//     {
//         RCLCPP_INFO(LOGGER, "Checking pose pair #%zu", i);

//         // Remove Collision
//         auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
//         auto result = client_remove->async_send_request(request);
//         RCLCPP_INFO(LOGGER, "Removing Collision");
//         rclcpp::sleep_for(std::chrono::milliseconds(500)); // Sleep for 0.5 seconds

//         move_group->setPoseTarget(grip_poses[i]);
//         bool plan_aim = (move_group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//         if (!plan_aim)
//             continue;
//         const moveit::core::LinkModel *ee_link_model =
//             move_group->getRobotModel()->getLinkModel(move_group->getEndEffectorLink());
//         visual_tools->publishTrajectoryLine(my_plan.trajectory_, ee_link_model, joint_model_group, rviz_visual_tools::LIME_GREEN);
//         visual_tools->trigger();

//         // Add Collision
//         auto request2 = std::make_shared<std_srvs::srv::Trigger::Request>();
//         auto result2 = client_add->async_send_request(request2);
//         RCLCPP_INFO(LOGGER, "Adding Collision");
//         rclcpp::sleep_for(std::chrono::milliseconds(500)); // Sleep for 0.5 seconds

//         move_group->setPoseTarget(aim_poses[i]);
//         bool plan_grip = (move_group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//         if (!plan_grip)
//             continue;
//         RCLCPP_INFO(LOGGER, "Valid pair found at index %zu", i);
//         successful_aim_pose = aim_poses[i];
//         successful_grip_pose = grip_poses[i];
//         visual_tools->publishTrajectoryLine(my_plan.trajectory_, ee_link_model, joint_model_group, rviz_visual_tools::LIME_GREEN);
//         visual_tools->trigger();

//         response->passed_index = static_cast<int8_t>(i);
//         double_plan = true;
//         return;
//     }

//     RCLCPP_WARN(LOGGER, "No valid plan for any pair of poses.");
//     response->passed_index = -1;
// }

void handle_aim_grip_request(
    const std::shared_ptr<AimGripPlan::Request> request,
    std::shared_ptr<AimGripPlan::Response> response)
{
    const auto &aim_poses = request->sorted_aim_poses.poses;
    const auto &aim_joint_states = request->aim_joint_states;
    const auto &grip_poses = request->sorted_grip_poses.poses;

    RCLCPP_INFO(LOGGER, "Received AimGripPlan Request");
    
    if (aim_poses.size() != aim_joint_states.size())
    {
        RCLCPP_WARN(LOGGER, "Mismatch between aim poses and joint states size.");
        response->passed_index = -1;
        return;
    }

    for (size_t i = 0; i < aim_joint_states.size(); ++i)
    {
        RCLCPP_INFO(LOGGER, "Checking joint state #%zu", i);

        const auto &joint_state = aim_joint_states[i];
        std::map<std::string, double> joint_values;

        for (size_t j = 0; j < joint_state.name.size(); ++j)
        {
            if (allowed_joints.count(joint_state.name[j]))
                {
                    joint_values[joint_state.name[j]] = joint_state.position[j];
                    // RCLCPP_INFO(LOGGER, "Added joint %s: %.4f", js.name[i].c_str(), js.position[i]);
                }
        }

        move_group->setJointValueTarget(joint_values);
        bool success = (move_group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            RCLCPP_INFO(LOGGER, "Valid joint plan found at index %zu", i);
            successful_aim_pose = aim_poses[i];
            successful_grip_pose = grip_poses[i];
            response->passed_index = static_cast<int8_t>(i);
            double_plan = true;
            return;
        }
    }

    RCLCPP_WARN(LOGGER, "No valid plan found for any joint state.");
    response->passed_index = -1;
}

void handle_aim_trigger_request(
    const std::shared_ptr<Trigger::Request> req,
    std::shared_ptr<Trigger::Response> res)
{
    (void)req;

    if (!double_plan)
    {
        res->success = false;
        res->message = "Both poses are not successfully planned";
        return;
    }

    moveit::core::MoveItErrorCode exec_result = move_group->execute(my_plan);
    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(LOGGER, "Execution failed");
        res->success = false;
        res->message = "Execution failed";
    }
    else
    {
        RCLCPP_INFO(LOGGER, "Motion executed successfully");
        res->success = true;
        res->message = "Motion executed successfully";
        aim_executed = true;
        double_plan = false;
    }
}

void handle_grip_trigger_request(
    const std::shared_ptr<Trigger::Request> req,
    std::shared_ptr<Trigger::Response> res)
{
    (void)req;

    if (!aim_executed)
    {
        RCLCPP_ERROR(LOGGER, "Grip trigger called before aim was executed. Motion not allowed.");
        res->success = false;
        res->message = "Aim motion not executed. Grip motion aborted.";
        return;
    }

    // Remove Collision
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = client_remove->async_send_request(request);
    RCLCPP_INFO(LOGGER, "Removing Collision");
    rclcpp::sleep_for(std::chrono::milliseconds(500)); // Sleep for 0.5 seconds

    std::vector<geometry_msgs::msg::Pose> waypoints;
    // waypoints.push_back(successful_aim_pose);
    waypoints.push_back(successful_grip_pose);

    moveit_msgs::msg::RobotTrajectory trajectory_msg;
    const double eef_step = 0.01;
    const double jump_threshold = 0.0;
    double fraction = 0.0;

    RCLCPP_INFO(LOGGER, "Computing Cartesian path from aim to grip pose...");

    int total_retry = 0;
    while (fraction < 1.0)
    {
        fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory_msg);
        if (fraction < 1.0 and total_retry < 10)
        {
            RCLCPP_WARN(LOGGER, "Cartesian path incomplete (%.2f%%), retrying...", fraction * 100.0);
            rclcpp::sleep_for(std::chrono::milliseconds(200));
        }
        total_retry++;
    }

    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
    cartesian_plan.trajectory_ = trajectory_msg;

    RCLCPP_INFO(LOGGER, "Cartesian path succeeded. Executing...");
    moveit::core::MoveItErrorCode exec_result = move_group->execute(cartesian_plan);

    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(LOGGER, "Grip Cartesian execution failed.");
        res->success = false;
        res->message = "Grip Cartesian execution failed.";
        return;
    }

    RCLCPP_INFO(LOGGER, "Grip Cartesian motion executed successfully.");
    res->success = true;
    res->message = "Grip Cartesian motion executed successfully.";
    aim_executed = false;

    visual_tools->deleteAllMarkers();
    visual_tools->trigger();

    // Add Collision
    auto request2 = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result2 = client_add->async_send_request(request2);
    RCLCPP_INFO(LOGGER, "Adding Collision");
    rclcpp::sleep_for(std::chrono::milliseconds(500)); // Sleep for 0.5 seconds
}


void handle_home_trigger_request(
    const std::shared_ptr<Trigger::Request> req,
    std::shared_ptr<Trigger::Response> res)
{
    (void)req;

    RCLCPP_INFO(LOGGER, "Lifting from successful grip pose by 3cm via Cartesian path...");

    geometry_msgs::msg::Pose start_pose = successful_grip_pose;
    geometry_msgs::msg::Pose lifted_pose = start_pose;
    lifted_pose.position.z += 0.03;

    std::vector<geometry_msgs::msg::Pose> waypoints = {lifted_pose};
    moveit_msgs::msg::RobotTrajectory trajectory_msg;
    const double eef_step = 0.01;
    const double jump_threshold = 0.0;
    double fraction = 0.0;

    rclcpp::Time lift_start_time = node->now();
    rclcpp::Duration lift_timeout = rclcpp::Duration::from_seconds(5.0);


    // Detach Collision
    auto request1 = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result1 = client_detach->async_send_request(request1);
    RCLCPP_INFO(LOGGER, "Detach Collision");
    rclcpp::sleep_for(std::chrono::milliseconds(500)); // Sleep for 0.5 seconds

    while (fraction < 1.0)
    {
        fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory_msg);

        if (fraction >= 1.0)
        {   
            RCLCPP_INFO(LOGGER, "Cartesian Path planned successfully.");
            break;
        }

        if ((node->now() - lift_start_time) > lift_timeout)
        {
            RCLCPP_WARN(LOGGER, "Cartesian path planning timed out after 5 seconds.");
            break;
        }

        RCLCPP_WARN(LOGGER, "Lift Cartesian path incomplete (%.2f%%), retrying...", fraction * 100.0);
        rclcpp::sleep_for(std::chrono::milliseconds(200));
    }


    if (fraction >= 1.0)
    {
        moveit::planning_interface::MoveGroupInterface::Plan lift_plan;
        lift_plan.trajectory_ = trajectory_msg;

        RCLCPP_INFO(LOGGER, "Executing lift trajectory (%.2f%% complete)...", fraction * 100.0);
        move_group->execute(lift_plan);
    }
    else
    {
        RCLCPP_WARN(LOGGER, "No valid lift path generated, skipping lift execution.");
    }

    RCLCPP_INFO(LOGGER, "Attempting to plan to home position (with 10s timeout)...");

    // Attach Collision
    auto request2 = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result2 = client_attach->async_send_request(request2);
    RCLCPP_INFO(LOGGER, "Attach Collision");
    rclcpp::sleep_for(std::chrono::milliseconds(500)); // Sleep for 0.5 seconds


    move_group->setJointValueTarget(home_joint_values);

    rclcpp::Time start_time = node->now();
    rclcpp::Duration timeout = rclcpp::Duration::from_seconds(5.0);
    bool success = false;

    while ((node->now() - start_time) < timeout)
    {
        if (move_group->plan(home_plan_global) == moveit::core::MoveItErrorCode::SUCCESS)
        {
            success = true;
            RCLCPP_INFO(LOGGER, "Successfully planned to home within timeout.");
            break;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(200));
    }

    if (success) {RCLCPP_INFO(LOGGER, "Successfully planned to home within timeout.");}
    else {RCLCPP_ERROR(LOGGER, "Failed to plan to home within 10 seconds.");}


    // Remove Collision

    // Detach Collision
    auto request3 = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result3 = client_detach->async_send_request(request3);
    RCLCPP_INFO(LOGGER, "Detach Collision");
    rclcpp::sleep_for(std::chrono::milliseconds(500)); // Sleep for 0.5 seconds


    rclcpp::Time start_time2 = node->now();
    rclcpp::Duration timeout2 = rclcpp::Duration::from_seconds(5.0);
    while ((node->now() - start_time2) < timeout2)
    {
        if (move_group->plan(home_plan_global) == moveit::core::MoveItErrorCode::SUCCESS)
        {
            success = true;
            RCLCPP_INFO(LOGGER, "Successfully planned to home within timeout.");
            break;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(200));
    }

    if (success) {RCLCPP_INFO(LOGGER, "Successfully planned to home within timeout.");}
    else {RCLCPP_ERROR(LOGGER, "Failed to plan to home within 10 seconds.");}


    if (!success)
    {
        res->success = false;
        res->message = "Home plan not available. Call home_plan_service first.";
        return;
    }

    RCLCPP_INFO(LOGGER, "Executing planned home trajectory...");

    moveit::core::MoveItErrorCode exec_result = move_group->execute(home_plan_global);
    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS)
    {
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
    home_plan = false;
}

void handle_joint_pose_request(
    const std::shared_ptr<custom_srv_pkg::srv::JointPose::Request> request,
    std::shared_ptr<custom_srv_pkg::srv::JointPose::Response> response)
{
    const auto &js = request->joint_state;

    if (js.name.size() != js.position.size())
    {
        RCLCPP_ERROR(LOGGER, "JointState name and position sizes do not match.");
        response->success = false;
        return;
    }

    std::map<std::string, double> joint_goal;
    for (size_t i = 0; i < js.name.size(); ++i)
    {
        if (allowed_joints.count(js.name[i]))
        {
            joint_goal[js.name[i]] = js.position[i];
            RCLCPP_INFO(LOGGER, "Added joint %s: %.4f", js.name[i].c_str(), js.position[i]);
        }
        else
        {
            RCLCPP_INFO(LOGGER, "Skipped joint %s", js.name[i].c_str());
        }
    }

    // std::map<std::string, double> joint_goal;
    // for (size_t i = 0; i < js.name.size(); ++i)
    // {
    //     joint_goal[js.name[i]] = js.position[i];
    // }

    // // Print full joint_goal map
    // RCLCPP_INFO(LOGGER, "Full joint goal:");
    // for (const auto &pair : joint_goal)
    // {
    //     RCLCPP_INFO(LOGGER, "  %s: %.4f", pair.first.c_str(), pair.second);
    // }


    move_group->setJointValueTarget(joint_goal);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(LOGGER, "Failed to plan to joint goal.");
        response->success = false;
        return;
    }

    if (move_group->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(LOGGER, "Failed to execute joint goal.");
        response->success = false;
        return;
    }

    RCLCPP_INFO(LOGGER, "Successfully executed joint goal.");
    response->success = true;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    node = rclcpp::Node::make_shared("robot_server", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]()
                        { executor.spin(); });

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

    auto aim_grip_plan_service = node->create_service<AimGripPlan>(
        "AimGripPlan", handle_aim_grip_request);

    auto move_joint_pose_service = node->create_service<JointPose>(
        "move_joint_pose", handle_joint_pose_request);

    auto aim_trigger_service = node->create_service<Trigger>(
        "aim_trigger_service", handle_aim_trigger_request);

    auto grip_trigger_service = node->create_service<Trigger>(
        "grip_trigger_service", handle_grip_trigger_request);

    auto home_trigger_service = node->create_service<Trigger>(
        "home_trigger_service", handle_home_trigger_request);

    client_add = node->create_client<Trigger>("add_collision_object");
    client_remove = node->create_client<Trigger>("remove_collision_object");

    client_attach = node->create_client<Trigger>("attach_collision_object");

    client_detach = node->create_client<Trigger>("detach_collision_object");


    RCLCPP_INFO(LOGGER, "Manipulation Server is ready.");

    spinner.join();
    rclcpp::shutdown();
    return 0;
}