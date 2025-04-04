import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R
import numpy as np


class PoseNumpyTransformer(Node):
    def __init__(self):
        super().__init__('pose_numpy_transformer')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        # Original pose in the 'world' frame
        pose_in_world = PoseStamped()
        pose_in_world.header.stamp = self.get_clock().now().to_msg()
        pose_in_world.header.frame_id = 'world'
        pose_in_world.pose.position.x = 1.0
        pose_in_world.pose.position.y = 2.0
        pose_in_world.pose.position.z = 0.0
        pose_in_world.pose.orientation.w = 1.0  # Identity quaternion

        try:
            # Lookup transform from world -> base_link
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'world', rclpy.time.Time()
            )

            # Convert TF to transformation matrix
            T = self.transform_to_matrix(transform)

            # Transform the position
            pos = pose_in_world.pose.position
            point_world = np.array([pos.x, pos.y, pos.z, 1.0])
            point_base = T @ point_world

            # Transform the orientation
            q_pose = pose_in_world.pose.orientation
            q_tf = transform.transform.rotation

            r_pose = R.from_quat([q_pose.x, q_pose.y, q_pose.z, q_pose.w])
            r_tf = R.from_quat([q_tf.x, q_tf.y, q_tf.z, q_tf.w])
            r_out = r_tf * r_pose  # Rotation composition

            q_out = r_out.as_quat()  # [x, y, z, w]

            self.get_logger().info(
                f"Position in base_link: x={point_base[0]:.2f}, y={point_base[1]:.2f}, z={point_base[2]:.2f}"
            )
            self.get_logger().info(
                f"Orientation in base_link: x={q_out[0]:.3f}, y={q_out[1]:.3f}, z={q_out[2]:.3f}, w={q_out[3]:.3f}"
            )

        except Exception as e:
            self.get_logger().warn(f"Transform failed: {e}")

    def transform_to_matrix(self, transform):
        """Convert a TransformStamped to a 4x4 transformation matrix"""
        t = transform.transform.translation
        q = transform.transform.rotation

        # Create rotation matrix
        r = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

        # Compose full 4x4 matrix
        T = np.eye(4)
        T[:3, :3] = r
        T[:3, 3] = [t.x, t.y, t.z]

        return T


def main(args=None):
    rclpy.init(args=args)
    node = PoseNumpyTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
