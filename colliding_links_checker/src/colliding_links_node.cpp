#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit/robot_state/conversions.h>

class CollidingLinksChecker
    : public rclcpp::Node,
      public std::enable_shared_from_this<CollidingLinksChecker>
{
public:
    CollidingLinksChecker()
        : Node("colliding_links_checker")
    {
        // Step 1: Subscribe to /robot_description to build RobotModel
        robot_description_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/robot_description",
            rclcpp::QoS(1).transient_local().reliable(),
            std::bind(&CollidingLinksChecker::robotDescriptionCallback, this, std::placeholders::_1));

        // Step 2: Subscribe to /planning_scene
        planning_scene_sub_ = this->create_subscription<moveit_msgs::msg::PlanningScene>(
            "/planning_scene", 10,
            std::bind(&CollidingLinksChecker::planningSceneCallback, this, std::placeholders::_1));
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

        planning_scene_->usePlanningSceneMsg(*msg);
        planning_scene_->getCurrentStateNonConst().update();

        // Colliding Links
        std::vector<std::string> colliding_links;
        planning_scene_->getCollidingLinks(colliding_links);

        if (colliding_links.empty())
        {
            RCLCPP_INFO(this->get_logger(), "No colliding links found.");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Colliding links detected:");
            for (const auto &link : colliding_links)
            {
                RCLCPP_WARN(this->get_logger(), " - %s", link.c_str());
            }
        }

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
            RCLCPP_INFO(this->get_logger(), "No collisions detected.");
            return;
        }

        RCLCPP_WARN(this->get_logger(), "Collisions detected:");

        for (const auto &contact_pair : collision_result.contacts)
        {
            const std::string &link1 = contact_pair.first.first;
            const std::string &link2 = contact_pair.first.second;

            bool is_link1_robot = robot_model_->hasLinkModel(link1);
            bool is_link2_robot = robot_model_->hasLinkModel(link2);

            if (is_link1_robot && is_link2_robot)
            {
                RCLCPP_WARN(this->get_logger(), "Robot vs Robot: %s <--> %s", link1.c_str(), link2.c_str());
            }
            else if (is_link1_robot || is_link2_robot)
            {
                RCLCPP_WARN(this->get_logger(), "Robot vs World: %s <--> %s", link1.c_str(), link2.c_str());
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "World vs World: %s <--> %s", link1.c_str(), link2.c_str());
            }
        }

        planning_scene_->usePlanningSceneMsg(*msg);
        planning_scene_->getCurrentStateNonConst().update();

        // Extract the goal state from the message
        moveit::core::RobotState goal_state(robot_model_);
        moveit::core::robotStateMsgToRobotState(msg->robot_state, goal_state);
        goal_state.update();

        // Check for collisions
        collision_detection::CollisionRequest request;
        collision_detection::CollisionResult result;
        request.contacts = true;
        request.max_contacts = 100;
        request.max_contacts_per_pair = 5;

        planning_scene_->checkCollision(request, result, goal_state);

        if (!result.collision)
        {
            RCLCPP_INFO(this->get_logger(), "✅ RViz goal state is collision-free.");
            return;
        }

        RCLCPP_WARN(this->get_logger(), "🚨 RViz goal state has collisions:");

        for (const auto &contact_pair : result.contacts)
        {
            const auto &link1 = contact_pair.first.first;
            const auto &link2 = contact_pair.first.second;

            bool is1_robot = robot_model_->hasLinkModel(link1);
            bool is2_robot = robot_model_->hasLinkModel(link2);

            if (is1_robot && is2_robot)
            {
                RCLCPP_WARN(this->get_logger(), "🤖 Robot vs Robot: %s <--> %s", link1.c_str(), link2.c_str());
            }
            else if (is1_robot || is2_robot)
            {
                RCLCPP_WARN(this->get_logger(), "🤖 Robot vs World: %s <--> %s", link1.c_str(), link2.c_str());
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "🌍 World vs World: %s <--> %s", link1.c_str(), link2.c_str());
            }
        }
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
    rclcpp::Subscription<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_sub_;

    moveit::core::RobotModelPtr robot_model_;
    planning_scene::PlanningScenePtr planning_scene_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CollidingLinksChecker>());
    rclcpp::shutdown();
    return 0;
}
