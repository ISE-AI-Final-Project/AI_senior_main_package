#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <custom_srv_pkg/msg/string_array.hpp>
#include <std_srvs/srv/trigger.hpp>

// URDF and SRDF
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>

// MoveIt2 headers
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model/robot_model.h>

// TF2 for transforming poses1
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Standard libraries
#include <set>
#include <vector>
#include <string>
#include <unordered_set>

using std::placeholders::_1;
using std::placeholders::_2;
using Trigger = std_srvs::srv::Trigger;

class GetPlanningSceneClient : public rclcpp::Node
{
public:
  GetPlanningSceneClient()
      : Node("get_planning_scene_client")
  {
    robot_description_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/robot_description",
        rclcpp::QoS(1).transient_local().reliable(),
        std::bind(&GetPlanningSceneClient::robotDescriptionCallback, this, std::placeholders::_1));

    planning_scene_sub_ = this->create_subscription<moveit_msgs::msg::PlanningScene>(
        "/collision_object_scene_topic", 10,
        std::bind(&GetPlanningSceneClient::planningSceneCallback, this, std::placeholders::_1));

    client_get_planning_scene = this->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");

    // Service to clear the planning scene (world + attached)
    clear_scene_service_ = create_service<Trigger>(
        "clear_planning_scene",
        std::bind(&GetPlanningSceneClient::handleClearPlanningScene, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), "Node initialized.");
  }

private:
  void robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Parsing /robot_description...");

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
  }

  void planningSceneCallback(const moveit_msgs::msg::PlanningScene::SharedPtr msg)
  {
    if (!robot_model_)
    {
      RCLCPP_WARN(this->get_logger(), "Robot model not initialized yet.");
      return;
    }

    received_objects_ = msg->world.collision_objects;
    RCLCPP_INFO(this->get_logger(), "Received %zu collision objects from topic.", received_objects_.size());

    // Wait for service
    while (!client_get_planning_scene->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_INFO(this->get_logger(), "Waiting for /get_planning_scene service...");
    }

    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    request->components.components = moveit_msgs::msg::PlanningSceneComponents::SCENE_SETTINGS |
                                     moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE |
                                     moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS |
                                     moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES |
                                     moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;

    client_get_planning_scene->async_send_request(request,
                                                  std::bind(&GetPlanningSceneClient::getPlanningSceneCallback, this, std::placeholders::_1));
  }

  void getPlanningSceneCallback(rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedFuture future)
  {
    auto response = future.get();

    if (!planning_scene_)
    {
      RCLCPP_ERROR(this->get_logger(), "Planning scene not initialized.");
      return;
    }

    planning_scene_->usePlanningSceneMsg(response->scene);
    RCLCPP_INFO(this->get_logger(), "Received base planning scene with %zu objects.", response->scene.world.collision_objects.size());

    RCLCPP_INFO(this->get_logger(), "Received base planning scene with %zu  Attach Collision objects.", response->scene.robot_state.attached_collision_objects.size());

    for (const auto &obj : received_objects_)
    {
      planning_scene_->processCollisionObjectMsg(obj);
    }

    RCLCPP_INFO(this->get_logger(), "Merged %zu new collision objects into the planning scene.", received_objects_.size());

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    collision_request.contacts = true;
    collision_request.max_contacts = 30000;
    collision_request.max_contacts_per_pair = 10;

    planning_scene_->checkCollision(collision_request, collision_result);

    std::set<std::string> removal_ids;

    if (collision_result.collision)
    {
      RCLCPP_WARN(this->get_logger(), "Collisions detected.");
      for (const auto &contact_pair : collision_result.contacts)
      {
        const std::string &link1 = contact_pair.first.first;
        const std::string &link2 = contact_pair.first.second;

        bool is_link1_robot = robot_model_->hasLinkModel(link1);
        bool is_link2_robot = robot_model_->hasLinkModel(link2);

        // Remove collided Attached Object

        if (link1.find("object") != std::string::npos && !is_link2_robot)
        {
          removal_ids.insert(link2);
        }
        if (link2.find("object") != std::string::npos&& !is_link1_robot)
        {
          removal_ids.insert(link1);
        }

        //  Remove objects collided with Robot
        if (is_link1_robot && !is_link2_robot && (link2 != "object"))
        {
          removal_ids.insert(link2);
          // RCLCPP_WARN(this->get_logger(), "Contact: %s <--> %s : Removing %s", link1.c_str(), link2.c_str(), link2.c_str());
        }
        else if (is_link2_robot && !is_link1_robot && (link1 != "object"))
        {
          removal_ids.insert(link1);
          // RCLCPP_WARN(this->get_logger(), "Contact: %s <--> %s : Removing %s", link1.c_str(), link2.c_str(), link1.c_str());
        }
      }
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "No collisions detected.");
    }

    moveit::planning_interface::PlanningSceneInterface psi;

    if (!received_objects_.empty())
    {
      psi.applyCollisionObjects(received_objects_);
      RCLCPP_INFO(this->get_logger(), "Applied %zu collision objects to interface.", received_objects_.size());
    }

    if (!removal_ids.empty())
    {
      std::vector<std::string> removal_vector(removal_ids.begin(), removal_ids.end());
      psi.removeCollisionObjects(removal_vector);
      RCLCPP_INFO(this->get_logger(), "Removed %zu collision objects due to collision with robot.", removal_vector.size());
    }
  }

  // Handler: clear everything from the planning scene
  void handleClearPlanningScene(
      const Trigger::Request::SharedPtr /*req*/,
      Trigger::Response::SharedPtr res)
  {
    moveit::planning_interface::PlanningSceneInterface psi;

    // remove all world collision objects
    std::vector<std::string> world_objs = psi.getKnownObjectNames();
    if (!world_objs.empty())
      psi.removeCollisionObjects(world_objs);

    // // remove all attached objects
    // auto attached = psi.getAttachedObjects();
    // std::vector<std::string> attached_ids;
    // attached_ids.reserve(attached.size());
    // for (const auto &kv : attached)
    //   attached_ids.push_back(kv.first);
    // if (!attached_ids.empty())
    //   psi.removeAttachedObjects(attached_ids);

    res->success = true;
    res->message = "Planning scene cleared (world + attached objects).";
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }

  // ROS Interfaces
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr client_get_planning_scene;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  rclcpp::Subscription<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_sub_;
  rclcpp::Service<Trigger>::SharedPtr clear_scene_service_;

  // MoveIt Models
  moveit::core::RobotModelPtr robot_model_;
  planning_scene::PlanningScenePtr planning_scene_;

  // Tracked objects received from /collision_object_scene_topic
  std::vector<moveit_msgs::msg::CollisionObject> received_objects_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GetPlanningSceneClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
