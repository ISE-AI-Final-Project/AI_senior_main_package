import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from custom_srv_pkg.srv import PESend
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
import cv2
from cv_bridge import CvBridge

class MaskPoseClient(Node):
    def __init__(self):
        super().__init__('pose_client')
        self.client = self.create_client(PESend, 'process_mask')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for server...")
        self.send_request()

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

    def send_request(self):
        request = PESend.Request()
        bridge = CvBridge()

        # Load a dummy mask image
        mask = cv2.imread('/home/icynunnymumu/Desktop/a.jpg', cv2.IMREAD_GRAYSCALE)
        if mask is None:
            mask = cv2.cvtColor(cv2.imread('/home/icynunnymumu/Desktop/b.jpg'), cv2.COLOR_BGR2GRAY)

        request.mask = bridge.cv2_to_imgmsg(mask, encoding='mono8')

        # Call service
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            # self.get_logger().info(f"Received pose: [{response.pose.position.x}, {response.pose.position.y}, {response.pose.position.z}]")
            self.get_logger().info(f"Received pose: [{response.pose}]")
        else:
            self.get_logger().error("Failed to get response.")

def main():
    rclpy.init()
    node = MaskPoseClient()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
