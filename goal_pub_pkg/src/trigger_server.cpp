#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class TriggerServer : public rclcpp::Node {
public:
  TriggerServer() : Node("trigger_server") {
    service_ = create_service<std_srvs::srv::Trigger>(
      "my_trigger",
      std::bind(&TriggerServer::handle_trigger, this, _1, _2));
    RCLCPP_INFO(this->get_logger(), "Trigger service ready.");
  }

private:
  void handle_trigger(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request; // unused
    response->success = true;
    response->message = "Triggered successfully!";
    RCLCPP_INFO(this->get_logger(), "Trigger called!");
  }

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TriggerServer>());
  rclcpp::shutdown();
  return 0;
}
