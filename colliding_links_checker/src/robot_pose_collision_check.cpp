#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/string.hpp>
#include <custom_srv_pkg/msg/string_array.hpp>

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
    robot_state = std::make_shared<moveit::core::RobotState>(robot_model_);

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
                                                                std::bind(&RobotPoseCollisionChecker::get_planning_scene_response_callback, this, std::placeholders::_1));
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

    std::vector<moveit_msgs::msg::CollisionObject> collision_object_vector;
    collision_object_vector.push_back(collision_object);


    planning_scene_interface_.applyCollisionObjects(collision_object_vector);


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


    // Update robot state
    // Define desired joint positions
    std::map<std::string, double> joint_values;
    joint_values["shoulder_pan_joint"] = -1.6;
    joint_values["shoulder_lift_joint"] = -1.72;
    joint_values["elbow_joint"] = -2.2;
    joint_values["wrist_1_joint"] = -0.8;
    joint_values["wrist_2_joint"] = -1.59;
    joint_values["wrist_3_joint"] = -0.0309;

    // Set the joint positions
    for (const auto& joint : joint_values)
    {
        robot_state->setJointPositions(joint.first, &joint.second);
    }

    // Update the robot state
    robot_state->update();

    // Set the robot state in the planning scene
    planning_scene_->setCurrentState(*robot_state);

    // Collision Results
    collision_detection::CollisionRequest collision_request2;
    collision_detection::CollisionResult collision_result2;

    // Optional: enable contacts
    collision_request2.contacts = true;
    collision_request2.max_contacts = 100; // Limit to 100 pairs
    collision_request2.max_contacts_per_pair = 5;

    planning_scene_->checkCollision(collision_request2, collision_result2);
    if (!collision_result2.collision)
    {
      RCLCPP_INFO(this->get_logger(), "No UR collisions detected.");
      return;
    }
    RCLCPP_WARN(this->get_logger(), "UR Collisions detected");

    // Create Set
    std::set<std::string> collision_boxes_to_remove_set2;

    for (const auto &contact_pair : collision_result2.contacts)
    {
      const std::string &link1 = contact_pair.first.first;
      const std::string &link2 = contact_pair.first.second;

      RCLCPP_WARN(this->get_logger(), "Collided: %s <--> %s", link1.c_str(), link2.c_str());

      bool is_link1_robot = robot_model_->hasLinkModel(link1);
      bool is_link2_robot = robot_model_->hasLinkModel(link2);

      if (is_link1_robot && !is_link2_robot && link1 != "table")
      {
        collision_boxes_to_remove_set2.insert(link2);
      }
      else if (is_link2_robot && !is_link1_robot && link2 != "table")
      {
        collision_boxes_to_remove_set2.insert(link1);
      }
    }
  }

  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr client_get_planning_scene;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;

  planning_scene::PlanningScenePtr planning_scene_;

  moveit::core::RobotModelPtr robot_model_;
  moveit::core::RobotStatePtr robot_state;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;



};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotPoseCollisionChecker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
