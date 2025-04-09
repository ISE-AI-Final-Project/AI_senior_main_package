#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from custom_srv_pkg.srv import TCPPose

class TCPPoseClient(Node):
    def __init__(self):
        super().__init__('tcp_pose_client')
        self.client = self.create_client(TCPPose, 'tcp_pose_service')
        while not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('Waiting for service...')
        self.send_request()

    def send_request(self):
        position = input("Enter Pose : ")
        x, y, z = map(float, position.split())

        request = TCPPose.Request()
        request.target_pose.position.x = x
        request.target_pose.position.y = y
        request.target_pose.position.z = z
        request.target_pose.orientation.x = 0.0
        request.target_pose.orientation.y = 0.0
        request.target_pose.orientation.z = 0.0
        request.target_pose.orientation.w = 1.0

        self.get_logger().info('Sending target pose request...')
        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Service Response: Success - {response.message}')
            else:
                self.get_logger().error(f'Service Response: Failed - {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
        finally:
            rclpy.shutdown()


def main():
    rclpy.init()
    node = TCPPoseClient()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
