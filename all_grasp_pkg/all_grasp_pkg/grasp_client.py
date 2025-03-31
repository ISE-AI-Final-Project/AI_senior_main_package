import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from custom_srv_pkg.srv import PESend
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
import cv2
from cv_bridge import CvBridge

class AllGraspClient(Node):
    def __init__(self):
        super().__init__('grasp_client')
        # get ros param
        self.param_client = self.create_client(GetParameters, "/main/get_parameters")
        while not self.param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.req = GetParameters.Request()
        param_to_read = ["target_obj"]
        response = self.send_request_param(param_to_read)
        print("Receive Param: %s" % (response.values[0].string_value))

    def send_request_param(self, params_name_list):
        self.req.names = params_name_list
        self.future = self.param_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()
    node = AllGraspClient()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
