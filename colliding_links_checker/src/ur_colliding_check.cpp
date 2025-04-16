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

class URCollidingBoxesPub
    : public rclcpp::Node,
      public std::enable_shared_from_this<URCollidingBoxesPub>
{
public:
    URCollidingBoxesPub()
        : Node("ur_colliding_boxes_node")
    {
        // Subscribe to /robot_description to build RobotModel
        robot_description_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/robot_description",
            rclcpp::QoS(1).transient_local().reliable(),
            std::bind(&URCollidingBoxesPub::robotDescriptionCallback, this, std::placeholders::_1));

        // Subscribe to /planning_scene
        planning_scene_sub_ = this->create_subscription<moveit_msgs::msg::PlanningScene>(
            "/planning_scene", 10,
            std::bind(&URCollidingBoxesPub::planningSceneCallback, this, std::placeholders::_1));

        collision_boxes_to_remove_pub_ = this->create_publisher<custom_srv_pkg::msg::StringArray>(
            "/main/collision_boxes_to_remove", 10);
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

    void planningSceneCallback(const moveit_msgs::msg::PlanningScene::SharedPtr msg)
    {
        if (!planning_scene_)
        {
            RCLCPP_WARN(this->get_logger(), "Planning scene not initialized yet.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Received Planning Scene...");

        planning_scene_->usePlanningSceneMsg(*msg);
        planning_scene_->getCurrentStateNonConst().update();

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

        if (!collision_boxes_to_remove_set.empty())
        {
            custom_srv_pkg::msg::StringArray msg;
            for (const auto &link_name : collision_boxes_to_remove_set)
            {
                msg.data.push_back(link_name);
            }

            RCLCPP_INFO(this->get_logger(), "Publishing %zu colliding links", msg.data.size());
            collision_boxes_to_remove_pub_->publish(msg);
        }
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
    rclcpp::Subscription<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_sub_;
    rclcpp::Publisher<custom_srv_pkg::msg::StringArray>::SharedPtr collision_boxes_to_remove_pub_;

    moveit::core::RobotModelPtr robot_model_;
    planning_scene::PlanningScenePtr planning_scene_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<URCollidingBoxesPub>());
    rclcpp::shutdown();
    return 0;
}