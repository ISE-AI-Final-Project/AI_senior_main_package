import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

class TfListener(Node):
    def __init__(self):
        super().__init__('sub_tf')
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10
        )

    def tf_callback(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id == 'base_link':
                t = transform.transform.translation
                self.get_logger().info(f"base_link translation -> x: {t.x:.3f}, y: {t.y:.3f}, z: {t.z:.3f}")


def main(args=None):
    rclpy.init(args=args)
    node = TfListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
