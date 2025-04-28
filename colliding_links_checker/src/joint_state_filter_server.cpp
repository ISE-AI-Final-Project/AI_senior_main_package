#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/string.hpp>
#include <custom_srv_pkg/msg/string_array.hpp>
#include <custom_srv_pkg/srv/joint_state_collision.hpp>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class RobotPoseCollisionChecker : public rclcpp::Node
{
public:
  RobotPoseCollisionChecker()
      : Node("robot_pose_collision_check")
  {
    // Subscribe to /robot_description to build RobotModel
    robot_description_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/robot_description",
        rclcpp::QoS(1).transient_local().reliable(),
        std::bind(&RobotPoseCollisionChecker::robotDescriptionCallback, this, std::placeholders::_1));

    // Create a client for the /get_planning_scene service
    client_get_planning_scene = this->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");

    server_srv_ = this->create_service<custom_srv_pkg::srv::JointStateCollision>(
        "joint_state_collision_check",
        std::bind(&RobotPoseCollisionChecker::serviceCallback, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void setAllowAdjacentCollisions(planning_scene::PlanningScenePtr planning_scene_)
  {
    auto &acm = planning_scene_->getAllowedCollisionMatrixNonConst();
    acm.setEntry("base_link_inertia", "shoulder_link", true);
    acm.setEntry("base_link_inertia", "table", true);
    acm.setEntry("forearm_link", "upper_arm_link", true);
    acm.setEntry("forearm_link", "wrist_1_link", true);
    acm.setEntry("mount", "robotiq_hande_link", true);
    acm.setEntry("mount", "zed2i", true);
    acm.setEntry("mount", "robotiq_hande_coupler", true);
    acm.setEntry("robotiq_hande_coupler", "robotiq_hande_link", true);
    acm.setEntry("robotiq_hande_coupler", "wrist_3_link", true);
    acm.setEntry("robotiq_hande_left_finger", "robotiq_hande_link", true);
    acm.setEntry("robotiq_hande_link", "robotiq_hande_right_finger", true);
    acm.setEntry("shoulder_link", "upper_arm_link", true);
    acm.setEntry("wrist_1_link", "wrist_2_link", true);
    acm.setEntry("wrist_2_link", "wrist_3_link", true);
  }
  void robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received /robot_description. Parsing...");

    auto urdf_model = urdf::parseURDF(msg->data);
    if (!urdf_model)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF.");
      return;
    }

    auto srdf_model = std::make_shared<srdf::Model>();
    srdf_model->initString(*urdf_model, "");

    robot_model_ = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);
    robot_state = std::make_shared<moveit::core::RobotState>(robot_model_);

    RCLCPP_INFO(this->get_logger(), "RobotModel and PlanningScene initialized.");
  }

  // void get_planning_scene_response_callback(rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedFuture future)
  // {
  //   RCLCPP_INFO(this->get_logger(), "Got Request");

  //   auto response = future.get();

  //   planning_scene_->usePlanningSceneMsg(response->scene);
  //   setAllowAdjacentCollisions(planning_scene_);
  //   RCLCPP_INFO(this->get_logger(), "Received planning scene with %zu world collision objects.",
  //               response->scene.world.collision_objects.size());

  //   received_new_planning_scene_ = true;
  // };

  void serviceCallback(
      const std::shared_ptr<custom_srv_pkg::srv::JointStateCollision::Request> request,
      std::shared_ptr<custom_srv_pkg::srv::JointStateCollision::Response> response)
  {

    planning_scene_->usePlanningSceneMsg(request->planning_scene);
    setAllowAdjacentCollisions(planning_scene_);

    for (size_t i = 0; i < request->joint_state.size(); ++i)
    {
      // Set robot state
      const auto &joint_state = request->joint_state[i];
      robot_state->setVariablePositions(joint_state.name, joint_state.position);
      robot_state->update();

      planning_scene_->setCurrentState(*robot_state);

      collision_detection::CollisionRequest collision_request;
      collision_detection::CollisionResult collision_result;

      collision_request.contacts = true;
      collision_request.max_contacts = 1000;
      collision_request.max_contacts_per_pair = 5;

      planning_scene_->checkCollision(collision_request, collision_result);

      bool collision_ok = true;

      for (const auto &contact_pair : collision_result.contacts)
      {
        const std::string &link1 = contact_pair.first.first;
        const std::string &link2 = contact_pair.first.second;

        bool is_link1_robot = robot_model_->hasLinkModel(link1);
        bool is_link2_robot = robot_model_->hasLinkModel(link2);

        if (is_link1_robot && is_link2_robot)
        {
          collision_detection::AllowedCollision::Type allowed_collision_type;
          bool entry_exists = planning_scene_->getAllowedCollisionMatrix().getEntry(link1, link2, allowed_collision_type);

          if (!entry_exists || allowed_collision_type != collision_detection::AllowedCollision::ALWAYS)
          {
            collision_ok = false;
            break;
          }
        }
        else if ((is_link1_robot && !is_link2_robot && link1 != "table") ||
                 (is_link2_robot && !is_link1_robot && link2 != "table"))
        {
          collision_ok = false;
          break;
        }
      }

      if (collision_ok)
      {
        response->possible_joint_state.push_back(joint_state);
        response->gripper_distance.push_back(request->gripper_distance[i]);
      }
    }
    RCLCPP_INFO(this->get_logger(), "Checked %zu joint_states, %zu are collision-free.", request->joint_state.size(), response->possible_joint_state.size());
  }

  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr client_get_planning_scene;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  rclcpp::Service<custom_srv_pkg::srv::JointStateCollision>::SharedPtr server_srv_;

  planning_scene::PlanningScenePtr planning_scene_;

  moveit::core::RobotModelPtr robot_model_;
  moveit::core::RobotStatePtr robot_state;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  bool received_new_planning_scene_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotPoseCollisionChecker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
