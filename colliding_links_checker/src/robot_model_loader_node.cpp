#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>

#include <moveit/robot_model/robot_model.h>

class RobotModelLoaderNode : public rclcpp::Node
{
public:
  RobotModelLoaderNode() : Node("robot_model_loader_node")
  {
    robot_description_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/robot_description",
        rclcpp::QoS(1).transient_local().reliable(),

        std::bind(&RobotModelLoaderNode::robotDescriptionCallback, this, std::placeholders::_1));
  }

private:
  void robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received /robot_description. Parsing URDF...");

    auto urdf_model = urdf::parseURDF(msg->data);
    if (!urdf_model)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF.");
      return;
    }

    auto srdf_model = std::make_shared<srdf::Model>();
    srdf_model->initString(*urdf_model, ""); // No SRDF provided, optional

    robot_model_ = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);

    RCLCPP_INFO(this->get_logger(), "✅ Successfully built RobotModel: %s", robot_model_->getName().c_str());

    for (const std::string &group_name : robot_model_->getJointModelGroupNames())
    {
      RCLCPP_INFO(this->get_logger(), "  ➤ Joint group: %s", group_name.c_str());
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  moveit::core::RobotModelPtr robot_model_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotModelLoaderNode>());
  rclcpp::shutdown();
  return 0;
}