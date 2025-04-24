#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class TriggerClient : public rclcpp::Node {
public:
  TriggerClient() : Node("trigger_client") {
    client_ = create_client<std_srvs::srv::Trigger>("my_trigger");

    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for service...");
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = client_->async_send_request(request);
    result.wait();
    
    if (result.get()->success) {
      RCLCPP_INFO(this->get_logger(), "Success: %s", result.get()->message.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed: %s", result.get()->message.c_str());
    }
  }

private:
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TriggerClient>());
  rclcpp::shutdown();
  return 0;
}
