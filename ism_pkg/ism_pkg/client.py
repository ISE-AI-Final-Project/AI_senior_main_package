import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from custom_srv_pkg.srv import IMGSend
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class ImageClient(Node):
    def __init__(self):
        super().__init__('image_client')
        self.client = self.create_client(IMGSend, 'image_service')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        self.bridge = CvBridge()
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
        img1 = cv2.imread('/home/icynunnymumu/Desktop/b.jpg')
        img2 = cv2.imread('/home/icynunnymumu/Desktop/a.jpg')

        if img1 is None or img2 is None:
            self.get_logger().error('Failed to load images. Check file paths.')
            return

        request = IMGSend.Request()
        request.rgb = self.bridge.cv2_to_imgmsg(img1, encoding='bgr8')
        request.depth = self.bridge.cv2_to_imgmsg(img2, encoding='bgr8')

        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            received_img = self.bridge.imgmsg_to_cv2(response.mask, desired_encoding='bgr8')
            cv2.imshow('Received Image', received_img)
            cv2.waitKey(0)
        except Exception as e:
            self.get_logger().error(f'Failed to receive response: {str(e)}')

def main():
    rclpy.init()
    node = ImageClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
