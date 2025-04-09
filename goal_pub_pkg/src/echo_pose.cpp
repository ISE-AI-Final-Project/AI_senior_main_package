#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "custom_srv_pkg/srv/tcp_pose.hpp"

class TCPPoseServer : public rclcpp::Node {
public:
    TCPPoseServer() : Node("tcp_pose_server") {
        service_ = this->create_service<custom_srv_pkg::srv::TCPPose>(
            "tcp_pose_service", 
            std::bind(&TCPPoseServer::handle_request, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "TCP Pose Server Ready");
    }

private:
    void handle_request(const std::shared_ptr<custom_srv_pkg::srv::TCPPose::Request> request,
                        std::shared_ptr<custom_srv_pkg::srv::TCPPose::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Received target pose request");
        RCLCPP_INFO(this->get_logger(), "Position: [%.2f, %.2f, %.2f]", 
                    request->target_pose.position.x, 
                    request->target_pose.position.y, 
                    request->target_pose.position.z);
        RCLCPP_INFO(this->get_logger(), "Orientation: [%.2f, %.2f, %.2f, %.2f]", 
                    request->target_pose.orientation.x, 
                    request->target_pose.orientation.y, 
                    request->target_pose.orientation.z, 
                    request->target_pose.orientation.w);
        
        // Process the request (Currently, it just acknowledges the request)
        response->success = true;
        response->message = "Pose received successfully.";
    }

    rclcpp::Service<custom_srv_pkg::srv::TCPPose>::SharedPtr service_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TCPPoseServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}