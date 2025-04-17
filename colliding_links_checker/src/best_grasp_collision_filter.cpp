#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/string.hpp>
#include <custom_srv_pkg/msg/string_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Geometry>

class BestGraspCollisionFilter : public rclcpp::Node
{
public:
  BestGraspCollisionFilter()
      : Node("get_planning_scene_client")
  {
    // Subscribe to /robot_description to build RobotModel
    robot_description_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/robot_description",
        rclcpp::QoS(1).transient_local().reliable(),
        std::bind(&BestGraspCollisionFilter::robotDescriptionCallback, this, std::placeholders::_1));

    // Create a client for the /get_planning_scene service
    client_get_planning_scene = this->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");

    best_grasp_poses_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/main/best_grasp_aim_poses",
        10,
        std::bind(&BestGraspCollisionFilter::bestGraspPosesCallback, this, std::placeholders::_1));
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
  }

  void bestGraspPosesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    std::vector<moveit_msgs::msg::CollisionObject> grasp_objects;

    for (size_t i = 0; i < msg->poses.size(); ++i)
    {
      const auto &end_pose = msg->poses[i];

      // CYLINDER FOR LINK
      moveit_msgs::msg::CollisionObject obj_link;
      obj_link.header.frame_id = robot_model_->getModelFrame();
      obj_link.id = "grasp_link_" + std::to_string(i);

      shape_msgs::msg::SolidPrimitive primitive_cylinder;
      primitive_cylinder.type = primitive_cylinder.CYLINDER;
      primitive_cylinder.dimensions.resize(2);
      primitive_cylinder.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = 0.25;
      primitive_cylinder.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = 0.0325;

      // Offset by 15cm due to collision object center
      Eigen::Isometry3d tf_link_pose;
      tf2::fromMsg(end_pose, tf_link_pose);
      tf_link_pose.translate(Eigen::Vector3d(0.0, 0.0, -0.15)); // move -15 cm along local Z
      geometry_msgs::msg::Pose link_pose = tf2::toMsg(tf_link_pose);

      obj_link.primitives.push_back(primitive_cylinder);
      obj_link.primitive_poses.push_back(link_pose);
      obj_link.operation = obj_link.ADD;

      grasp_objects.push_back(obj_link);
      planning_scene_->processCollisionObjectMsg(obj_link);

      // BOX FOR MOUNT
      moveit_msgs::msg::CollisionObject obj_mount;
      obj_mount.header.frame_id = robot_model_->getModelFrame();
      obj_mount.id = "grasp_mount_" + std::to_string(i);

      shape_msgs::msg::SolidPrimitive primitive_box;
      primitive_box.type = primitive_box.BOX;
      primitive_box.dimensions.resize(3);
      primitive_box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.06;
      primitive_box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.04;
      primitive_box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.19;

      Eigen::Isometry3d tf_mount_pose;
      tf2::fromMsg(end_pose, tf_mount_pose);
      tf_mount_pose.translate(Eigen::Vector3d(0.06, 0.0, -0.15)); // move -15 cm along local Z
      geometry_msgs::msg::Pose mount_pose = tf2::toMsg(tf_mount_pose);

      obj_mount.primitives.push_back(primitive_box);
      obj_mount.primitive_poses.push_back(mount_pose);
      obj_mount.operation = obj_mount.ADD;

      grasp_objects.push_back(obj_mount);
      planning_scene_->processCollisionObjectMsg(obj_mount);
    }

    planning_scene_interface_.applyCollisionObjects(grasp_objects);
    RCLCPP_INFO(this->get_logger(), "Added %zu grasp collision to the planning scene.", msg->poses.size());

    // Call /get_planning_scene

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
                                                                std::bind(&BestGraspCollisionFilter::get_planning_scene_response_callback, this, std::placeholders::_1));
  }

  void get_planning_scene_response_callback(rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedFuture future)
  {
    auto response = future.get();

    planning_scene_->usePlanningSceneMsg(response->scene);

    RCLCPP_INFO(this->get_logger(), "Received planning scene with %zu world collision objects.",
                response->scene.world.collision_objects.size());

    // Collision Results
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;

    // Optional: enable contacts
    collision_request.contacts = true;
    collision_request.max_contacts = 1000; // Limit to 100 pairs
    collision_request.max_contacts_per_pair = 2;

    planning_scene_->checkCollision(collision_request, collision_result);

    for (const auto &contact_pair : collision_result.contacts)
    {
      const std::string &object1 = contact_pair.first.first;
      const std::string &object2 = contact_pair.first.second;

      RCLCPP_INFO(this->get_logger(), "Contact detected between: %s and %s", object1.c_str(), object2.c_str());

      for (const auto &contact : contact_pair.second)
      {
        RCLCPP_INFO(this->get_logger(), "Contact point at position: x=%f, y=%f, z=%f",
                    contact.pos.x(), contact.pos.y(), contact.pos.z());
      }
    }

    // Get all collision (including world vs world)
    const moveit::core::RobotState &state = planning_scene_->getCurrentState();
    planning_scene_->getCollisionEnv()->checkCollision(collision_request, collision_result, state);
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
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr best_grasp_poses_sub_;

  planning_scene::PlanningScenePtr planning_scene_;

  moveit::core::RobotModelPtr robot_model_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BestGraspCollisionFilter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
