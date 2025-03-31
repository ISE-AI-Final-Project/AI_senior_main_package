import rclpy
from rclpy.node import Node
from custom_srv_pkg.srv import IMGSend
from sensor_msgs.msg import Image

class ImageService(Node):
    def __init__(self):
        super().__init__('image_service')
        self.srv = self.create_service(IMGSend, 'image_service', self.image_callback)
        self.get_logger().info('Image Service Ready')

    def image_callback(self, request, response):
        self.get_logger().info('Received two images, returning the first one')
        response.mask = request.rgb  # Return the first image
        return response

def main():
    rclpy.init()
    node = ImageService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
