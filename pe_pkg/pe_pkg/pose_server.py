import rclpy
from rclpy.node import Node
from custom_srv_pkg.srv import PESend
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose

class MaskPoseServer(Node):
    def __init__(self):
        super().__init__('pose_service')
        self.srv = self.create_service(PESend, 'process_mask', self.handle_request)
        self.get_logger().info("Mask Pose Server is ready.")

    def handle_request(self, request, response):
        self.get_logger().info("Received mask image.")

        # Simulate pose estimation
        response.pose.position.x = 0.3
        response.pose.position.y = 0.4
        response.pose.position.z = 0.5
        response.pose.orientation.x = 0.0
        response.pose.orientation.y = 0.0
        response.pose.orientation.z = 0.0
        response.pose.orientation.w = 1.0

        self.get_logger().info("Sending 6d pose.")
        return response

def main():
    rclpy.init()
    node = MaskPoseServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
