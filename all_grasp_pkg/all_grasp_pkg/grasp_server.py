import rclpy
from rclpy.node import Node
from custom_srv_pkg.srv import PESend
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose

class AllGraspServer(Node):
    def __init__(self):
        super().__init__('all_grasp_service')
        self.srv = self.create_service(PESend, 'process_pose', self.handle_request)
        self.get_logger().info("All Grasp Server is ready.")

    def handle_request(self, request, response):
        self.get_logger().info("Received target object.")

        # Simulate pose estimation
        response.pose.position.x = 2.3
        response.pose.position.y = 2.4
        response.pose.position.z = 2.5
        response.pose.orientation.x = 4.0
        response.pose.orientation.y = 5.0
        response.pose.orientation.z = 6.0
        response.pose.orientation.w = 1.0

        self.get_logger().info("Sending 6d pose (all grasp).")
        return response

def main():
    rclpy.init()
    node = AllGraspServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
