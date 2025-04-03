import cv2
import rclpy
from rclpy.node import Node
from PIL import Image
import numpy as np
import matplotlib.pyplot as plt

from .utils.my_custom_socket import MyClient


class SocketClientNode(Node):
    def __init__(self):
        super().__init__("socket_client_node")
        self.client = MyClient(host="127.0.0.1", port=65432)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.send_once = False

    def timer_callback(self):
        # Read an image
        if self.send_once:
            return
        
        client_connected = self.client.connect()
        if not client_connected:
            self.get_logger().error(f"Server not started")
            return
        
        mask_path = '/home/bham/Desktop/Dataset/Linemod_preprocessed/data/01/mask/0010.png'
        mask_image = Image.open('/home/bham/Desktop/Dataset/Linemod_preprocessed/data/01/mask/0010.png').convert("1")
        mask = np.array(mask_image, dtype=np.uint8)

        if mask is None:
            self.get_logger().error(f"[Client] Failed to read mask at {mask_path}")
            return

        plt.imshow(mask, cmap = 'gray')
        plt.show()

        self.get_logger().info(f"[Client] Loaded mask with shape {mask.shape}")

        object_name = "gorilla"
        try:
            result = self.client.request_msg(
                msg_type_in=["image2d", "string"],
                msg_in=[mask, object_name],)
            self.get_logger().info(f"Response: {result}")
            self.sent_once = True
        except Exception as e:
            self.get_logger().error(f"Failed to send request: {e}")

        self.client.disconnect()


def main(args=None):
    rclpy.init(args=args)
    node = SocketClientNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
