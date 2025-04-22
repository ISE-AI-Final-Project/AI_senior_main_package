#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/string.hpp>
#include <custom_srv_pkg/msg/string_array.hpp>

#include <urdf_parser/urdf_parser.h>

add_executable(planning_scene_caller_with_collision src/planning_scene_caller_with_collision.cpp)

ament_target_dependencies(planning_scene_caller_with_collision
  rclcpp
  std_msgs
  urdf
  srdfdom
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  custom_srv_pkg
)
install(TARGETS planning_scene_caller_with_collision DESTINATION lib/${PROJECT_NAME})
NEW


Message #code
﻿





to select
;
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/string.hpp>
#include <custom_srv_pkg/msg/string_array.hpp>

#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/srv/get_planning_scene.hpp>

class GetPlanningSceneClient : public rclcpp::Node
{
public:
  GetPlanningSceneClient()
      : Node("get_planning_scene_client")
  {
    // Subscribe to /robot_description to build RobotModel
    robot_description_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/robot_description",
        rclcpp::QoS(1).transient_local().reliable(),
        std::bind(&GetPlanningSceneClient::robotDescriptionCallback, this, std::placeholders::_1));

    // Create a client for the /get_planning_scene service
    client_get_planning_scene = this->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
  }

private:
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

    RCLCPP_INFO(this->get_logger(), "RobotModel and PlanningScene initialized.");

    // Wait for the service to be available
    while (!client_get_planning_scene->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_INFO(this->get_logger(), "Waiting for /get_planning_scene service...");
    }

    // Create a request
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();

    // Specify the components you want to retrieve
    request->components.components = moveit_msgs::msg::PlanningSceneComponents::SCENE_SETTINGS |
                                     moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE |
                                     moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES |
                                     moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;

    // Send the request asynchronously
    auto future = client_get_planning_scene->async_send_request(request,
                                                                std::bind(&GetPlanningSceneClient::get_planning_scene_response_callback, this, std::placeholders::_1));
  }
  void get_planning_scene_response_callback(rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedFuture future)
  {
    auto response = future.get();

    planning_scene_->usePlanningSceneMsg(response->scene);

    RCLCPP_INFO(this->get_logger(), "Received planning scene with %zu world collision objects.",
                response->scene.world.collision_objects.size());


    // TEMP: CREATE MY OWN BOX
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = robot_model_->getModelFrame();
    collision_object.id = "my_box1";

    // Define a box primitive
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.2;
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.2;
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.2;

    // Define the pose of the box
    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = 0.5;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.1;
    box_pose.orientation.w = 1.0;

    // Add the primitive and pose to the collision object
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    // Add the object to the planning scene
    planning_scene_->processCollisionObjectMsg(collision_object);

    RCLCPP_INFO(this->get_logger(), "Added box1 to planning scene.");

    // Collision Results
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;

    // Optional: enable contacts
    collision_request.contacts = true;
    collision_request.max_contacts = 100; // Limit to 100 pairs
    collision_request.max_contacts_per_pair = 5;

    planning_scene_->checkCollision(collision_request, collision_result);
    if (!collision_result.collision)
    {
      RCLCPP_INFO(this->get_logger(), "No UR collisions detected.");
      return;
    }
    RCLCPP_WARN(this->get_logger(), "UR Collisions detected");

    // Create Set
    std::set<std::string> collision_boxes_to_remove_set;

    for (const auto &contact_pair : collision_result.contacts)
    {
      const std::string &link1 = contact_pair.first.first;
      const std::string &link2 = contact_pair.first.second;

      RCLCPP_WARN(this->get_logger(), "Collided: %s <--> %s", link1.c_str(), link2.c_str());

      bool is_link1_robot = robot_model_->hasLinkModel(link1);
      bool is_link2_robot = robot_model_->hasLinkModel(link2);

      if (is_link1_robot && !is_link2_robot && link1 != "table")
      {
        collision_boxes_to_remove_set.insert(link2);
      }
      else if (is_link2_robot && !is_link1_robot && link2 != "table")
      {
        collision_boxes_to_remove_set.insert(link1);
      }
    }
  }

  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr client_get_planning_scene;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;

  planning_scene::PlanningScenePtr planning_scene_;

  moveit::core::RobotModelPtr robot_model_;

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GetPlanningSceneClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}



// class URCollidingBoxesSceneManager : public rclcpp::Node
// {
// public:
//   URCollidingBoxesSceneManager()
//     : Node("ur_colliding_boxes_scene_manager"),
//       tf_buffer_(this->get_clock()),
//       tf_listener_(tf_buffer_)
//   {
//     // Subscribe to /robot_description to build the RobotModel and the PlanningScene.
//     robot_description_sub_ = this->create_subscription<std_msgs::msg::String>(
//       "/robot_description",
//       rclcpp::QoS(1).transient_local().reliable(),
//       std::bind(&URCollidingBoxesSceneManager::robotDescriptionCallback, this, std::placeholders::_1));

//     // Subscribe to /planning_scene topic.
//     planning_scene_sub_ = this->create_subscription<moveit_msgs::msg::PlanningScene>(
//       "/collision_object_scene_topic
// ", 10,
//       std::bind(&URCollidingBoxesSceneManager::planningSceneCallback, this, std::placeholders::_1));

//     RCLCPP_INFO(this->get_logger(), "URCollidingBoxesSceneManager is up and running.");
//   }

// private:
//   // Callback to parse the robot description and initialize the RobotModel and PlanningScene.
//   void robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg)
//   {
//     RCLCPP_INFO(this->get_logger(), "Received /robot_description. Parsing...");

//     // Parse the URDF model.
//     auto urdf_model = urdf::parseURDF(msg->data);
//     if (!urdf_model)
//     {
//       RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF.");
//       return;
//     }

//     // Initialize an (empty) SRDF model.
//     auto srdf_model = std::make_shared<srdf::Model>();
//     srdf_model->initString(*urdf_model, "");

//     // Create the RobotModel and the PlanningScene.
//     robot_model_ = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
//     planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);

//     RCLCPP_INFO(this->get_logger(), "RobotModel and PlanningScene initialized.");
//   }

//   // Callback to process planning scene messages.
//   void planningSceneCallback(const moveit_msgs::msg::PlanningScene::SharedPtr msg)
//   {
//     if (!planning_scene_)
//     {
//       RCLCPP_WARN(this->get_logger(), "Planning scene not initialized yet.");
//       return;
//     }

//     RCLCPP_INFO(this->get_logger(), "Received PlanningScene message.");

//     // Update the PlanningScene with the new message.
//     planning_scene_->usePlanningSceneMsg(*msg);
//     planning_scene_->getCurrentStateNonConst().update();

//     //
//     // Part 1: Collision Checking and Removing Unwanted Collision Objects
//     collision_detection::CollisionRequest collision_request;
//     collision_detection::CollisionResult collision_result;
//     collision_request.contacts = true;
//     collision_request.max_contacts = 1000;
//     collision_request.max_contacts_per_pair = 10;
    
//     // // Test self-collision on current state
//     // planning_scene_->checkSelfCollision(collision_request, collision_result);
//     // RCLCPP_INFO(this->get_logger(), "Test 1: Current state is %s self collision",
//     //             collision_result.collision ? "in" : "not in");

//     // moveit::core::RobotState& current_state = planning_scene_->getCurrentStateNonConst();    
//     // collision_result.clear();
//     // planning_scene_->checkSelfCollision(collision_request, collision_result);
//     // RCLCPP_INFO(this->get_logger(), "Test 2: Current state is %s self collision",
//     //             collision_result.collision ? "in" : "not in");
    
//     // collision_result.clear();
    
//     planning_scene_->checkCollision(collision_request, collision_result);

//     // Set to store collision object IDs that conflict with the robot.
//     std::set<std::string> removal_set;
//     if (collision_result.collision)
//     {
//       RCLCPP_WARN(this->get_logger(), "Collisions detected. Processing collision contacts...");
//       for (const auto &contact_pair : collision_result.contacts)
//       {
//         const std::string &link1 = contact_pair.first.first;
//         const std::string &link2 = contact_pair.first.second;

//         RCLCPP_WARN(this->get_logger(), "Contact: %s <--> %s", link1.c_str(), link2.c_str());

//         bool is_link1_robot = robot_model_->hasLinkModel(link1);
//         bool is_link2_robot = robot_model_->hasLinkModel(link2);

//         // Assume that when one of the links is part of the robot and the other is not,
//         // the non-robot link is associated with a collision object to remove.
//         // (Here we also ignore objects designated as "table".)
//         if (is_link1_robot && !is_link2_robot && link1 != "table")
//           removal_set.insert(link2);
//         else if (is_link2_robot && !is_link1_robot && link2 != "table")
//           removal_set.insert(link1);
//       }

//       if (!removal_set.empty())
//       {
//         std::vector<std::string> removal_ids(removal_set.begin(), removal_set.end());
//         RCLCPP_INFO(this->get_logger(), "Removing %zu colliding collision objects.", removal_ids.size());
//         for (const auto& id : removal_ids) {
//           RCLCPP_INFO(this->get_logger(), "Removing collision object: %s", id.c_str());
//       }
      
//         planning_scene_interface_.removeCollisionObjects(removal_ids);
//       }
//       else
//       {
//         RCLCPP_INFO(this->get_logger(), "No collision objects meet removal criteria.");
//       }
//     }
//     else
//     {
//       RCLCPP_INFO(this->get_logger(), "No collisions detected.");
//     }

//     //
//     // Part 2: Apply the Collision Objects to the MoveIt2 Scene
//     //
//     // This section follows the provided snippet.
//     //
//     // First, remove any previously tracked collision objects.
//     if (!tracked_objects_.empty())
//     {
//       std::vector<std::string> old_object_ids(tracked_objects_.begin(), tracked_objects_.end());
//       planning_scene_interface_.removeCollisionObjects(old_object_ids);
//       tracked_objects_.clear();
//       RCLCPP_INFO(this->get_logger(), "Removed previous scene with %zu objects", old_object_ids.size());
//     }

//     // Get the new collision objects from the planning scene message.
//     std::vector<moveit_msgs::msg::CollisionObject> new_collision_objects = msg->world.collision_objects;
//     if (new_collision_objects.empty())
//     {
//       RCLCPP_WARN(this->get_logger(), "New scene is empty, previous scene cleared.");
//       return;
//     }

//     RCLCPP_INFO(this->get_logger(), "visualizing collision objects.");

//     // Transform and apply each collision object.
//     for (auto &obj_collision : new_collision_objects)
//     {
//       try
//       {
//         // Transform each primitive's pose to the "world" frame.
//         for (auto &primitive_pose : obj_collision.primitive_poses)
//         {
//           geometry_msgs::msg::PoseStamped input_pose, transformed_pose;
//           input_pose.header = obj_collision.header;
//           input_pose.pose = primitive_pose;

//           tf_buffer_.transform(input_pose, transformed_pose, "world", tf2::durationFromSec(0.5));
//           primitive_pose = transformed_pose.pose;
//         }
//         // Update the object's header to reference the world frame.
//         obj_collision.header.frame_id = "world";
//         // Track the applied object.
//         tracked_objects_.insert(obj_collision.id);
//         // Apply the collision object to the planning scene.
//         planning_scene_interface_.applyCollisionObjects({obj_collision});
//         if (!removal_set.empty())
//       {
//         std::vector<std::string> removal_ids(removal_set.begin(), removal_set.end());
//         planning_scene_interface_.removeCollisionObjects(removal_ids);
//       }
//       }
//       catch (const tf2::TransformException &ex)
//       {
//         RCLCPP_WARN(this->get_logger(), "Could not transform collision object %s: %s",
//                     obj_collision.id.c_str(), ex.what());
//       }
//     }
//     RCLCPP_INFO(this->get_logger(), "Applied collision objects");


//   }

//   // Subscribers
//   rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
//   rclcpp::Subscription<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_sub_;

//   // MoveIt2 Robot Model and PlanningScene.
//   moveit::core::RobotModelPtr robot_model_;
//   planning_scene::PlanningScenePtr planning_scene_;

//   // MoveIt2 PlanningSceneInterface to apply or remove collision objects.
//   moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

//   // Tracking IDs of previously applied collision objects.
//   std::unordered_set<std::string> tracked_objects_;

//   // TF2 members for pose transformations.
//   tf2_ros::Buffer tf_buffer_;
//   tf2_ros::TransformListener tf_listener_;
// };

// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<URCollidingBoxesSceneManager>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }