#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>

class GetPlanningSceneClient : public rclcpp::Node
{
public:
  GetPlanningSceneClient()
  : Node("get_planning_scene_client")
  {
    // Create a client for the /get_planning_scene service
    client_ = this->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");

    // Wait for the service to be available
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
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
    auto future = client_->async_send_request(request,
      std::bind(&GetPlanningSceneClient::response_callback, this, std::placeholders::_1));
  }

private:
  void response_callback(rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedFuture future)
  {
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "Received planning scene with %zu world collision objects.",
                response->scene.world.collision_objects.size());
    // Further processing can be done here
  }

  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr client_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GetPlanningSceneClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
