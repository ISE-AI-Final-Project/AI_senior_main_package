#include <rclcpp/rclcpp.hpp>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/planning_scene.h>
#include <moveit_msgs/msg/collision_object.h>
// TF2
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // for tf2::doTransform
#include <unordered_set>

class CollisionSceneExample : public rclcpp::Node {
public:
  CollisionSceneExample()
  : Node("collision_scene_example"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_) {

    planning_scene_sub_ = this->create_subscription<moveit_msgs::msg::PlanningScene>(
      "collision_object_scene_topic", 10,
      std::bind(&CollisionSceneExample::planningSceneCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Subscribed to planning scene topic");
  }

private:
  void planningSceneCallback(const moveit_msgs::msg::PlanningScene::SharedPtr planning_scene_msg) {
    if (!planning_scene_msg) return;

    std::vector<moveit_msgs::msg::CollisionObject> new_collision_objects = planning_scene_msg->world.collision_objects;

    // Step 1: Remove previous objects before applying the new scene
    if (!tracked_objects_.empty()) {
      std::vector<std::string> old_object_ids(tracked_objects_.begin(), tracked_objects_.end());
      planning_scene_interface_.removeCollisionObjects(old_object_ids);
      tracked_objects_.clear();
      RCLCPP_INFO(this->get_logger(), "Removed previous scene with %zu objects", old_object_ids.size());
    }

    if (new_collision_objects.empty()) {
      RCLCPP_WARN(this->get_logger(), "New scene is empty, previous scene cleared.");
      return;
    }

    // Step 2: Transform and apply new collision objects
    for (auto& obj_collision : new_collision_objects) {
      try {
        geometry_msgs::msg::PoseStamped input_pose, transformed_pose;
        for (auto& primitive_pose : obj_collision.primitive_poses) {
          input_pose.header = obj_collision.header;
          input_pose.pose = primitive_pose;

          tf_buffer_.transform(input_pose, transformed_pose, "world", tf2::durationFromSec(0.5));
          primitive_pose = transformed_pose.pose;
        }

        obj_collision.header.frame_id = "world";
        tracked_objects_.insert(obj_collision.id); // Track new objects
        planning_scene_interface_.applyCollisionObjects({obj_collision});

        RCLCPP_INFO(this->get_logger(), "Added collision object: %s", obj_collision.id.c_str());

      } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform collision object: %s", ex.what());
      }
    }
  }

  rclcpp::Subscription<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_sub_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  std::unordered_set<std::string> tracked_objects_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CollisionSceneExample>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
