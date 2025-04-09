#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/msg/planning_scene.hpp>

class CollidingLinksChecker
    : public rclcpp::Node,
      public std::enable_shared_from_this<CollidingLinksChecker>
{
public:
    CollidingLinksChecker()
        : Node("colliding_links_checker")
    {
        // Subscribe to /planning_scene
        planning_scene_sub_ = this->create_subscription<moveit_msgs::msg::PlanningScene>(
            "/planning_scene", 10,
            std::bind(&CollidingLinksChecker::planningSceneCallback, this, std::placeholders::_1));

        // Delay planning scene setup using a timer
        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]()
            {
                if (!robot_model_loader_)
                {
                    try
                    {
                        robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
                            std::enable_shared_from_this<CollidingLinksChecker>::shared_from_this(), "/robot_description");

                        planning_scene_ = std::make_shared<planning_scene::PlanningScene>(
                            robot_model_loader_->getModel());

                        RCLCPP_INFO(this->get_logger(), "Planning scene successfully initialized.");
                    }
                    catch (const std::exception &e)
                    {
                        RCLCPP_ERROR(this->get_logger(), "Failed to initialize planning scene: %s", e.what());
                    }
                }
            });
    }

private:
    void planningSceneCallback(const moveit_msgs::msg::PlanningScene::SharedPtr msg)
    {
        if (!planning_scene_)
        {
            RCLCPP_WARN(this->get_logger(), "Planning scene not initialized yet.");
            return;
        }

        planning_scene_->usePlanningSceneMsg(*msg);
        planning_scene_->getCurrentStateNonConst().update();

        std::vector<std::string> colliding_links;
        planning_scene_->getCollidingLinks(colliding_links);

        if (colliding_links.empty())
        {
            RCLCPP_INFO(this->get_logger(), "✅ No colliding links found.");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "🚨 Colliding links detected:");
            for (const auto &link : colliding_links)
            {
                RCLCPP_WARN(this->get_logger(), " - %s", link.c_str());
            }
        }
    }

    rclcpp::Subscription<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_sub_;
    rclcpp::TimerBase::SharedPtr init_timer_;

    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    planning_scene::PlanningScenePtr planning_scene_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CollidingLinksChecker>());
    rclcpp::shutdown();
    return 0;
}
