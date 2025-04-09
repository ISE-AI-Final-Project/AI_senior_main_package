import cv2
import rclpy
from rclpy.node import Node

from .utils.my_custom_socket import MyClient


class SocketClientNode(Node):
    def __init__(self):
        super().__init__("socket_client_node")

        # Init client
        self.client = MyClient(host="127.0.0.1", port=65432)

        # Set up a periodic timer (adjust 1.0 to however often you want to send)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        # Read an image

        client_connected = self.client.connect()

        if not client_connected:
            self.get_logger().error(f"Server not started")
            return

        my_int = 2
        my_int2 = 5

        # For simplicity, simulate input
        string_input = "auto-message"

        # Send request
        try:
            result = self.client.request_msg(
                msg_type_in=["int", "int", "string"],
                msg_in=[my_int, my_int2, string_input],
            )
            self.get_logger().info(f"Response: {result}")
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
