import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import numpy as np

class UR3eFKNode(Node):
    def __init__(self):
        super().__init__('ur3e_fk_from_joint_states')
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.pose_pub = self.create_publisher(Pose, 'end_effector_pose_from_joints', 10)
        self.get_logger().info("UR3e FK Node initialized")

        # UR3e standard DH parameters (a, alpha, d, theta_offset)
        self.dh_params = [
            (0.0,        np.pi/2,  0.1519,  0.0),    # Base to Shoulder (shoulder_pan_joint)
            (-0.24365,   0.0,      0.0,     0.0),    # Shoulder to Elbow (shoulder_lift_joint)
            (-0.21325,   0.0,      0.0,     0.0),    # Elbow to Wrist1 (elbow_joint)
            (0.0,        np.pi/2,  0.11235, 0.0),    # Wrist1 to Wrist2 (wrist_1_joint)
            (0.0,       -np.pi/2,  0.08535, 0.0),    # Wrist2 to Wrist3 (wrist_2_joint)
            (0.0,        0.0,      0.0819,  0.0),     # Wrist3 to tool0 (wrist_3_joint)
            (0.0,        0.0,      0.15695, 0.0)      # tool0 to end effector modify offset hhere!!!
        ]

    def dh_matrix(self, a, alpha, d, theta):
        """Build the individual DH transform matrix."""
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0,              np.sin(alpha),                np.cos(alpha),               d],
            [0,              0,                            0,                           1]
        ])

    def joint_callback(self, msg):
        # Extract joint angles by UR3e order
        joint_map = dict(zip(msg.name, msg.position))
        try:
            joint_angles = [
                joint_map['shoulder_pan_joint'],
                joint_map['shoulder_lift_joint'],
                joint_map['elbow_joint'],
                joint_map['wrist_1_joint'],
                joint_map['wrist_2_joint'],
                joint_map['wrist_3_joint'],
                0.0 #joint_map['ee']
            ]
        except KeyError as e:
            self.get_logger().warn(f"Missing joint in joint_states: {e}")
            return

        # Compute full transform
        T = np.eye(4)
        for i in range(7):
            a, alpha, d, theta_offset = self.dh_params[i]
            theta = joint_angles[i] + theta_offset
            T_i = self.dh_matrix(a, alpha, d, theta)
            T = T @ T_i

        # Extract position and orientation (quaternion)
        position = T[0:3, 3]
        R = T[0:3, 0:3]
        quat = self.rotation_matrix_to_quaternion(R)

        # Publish
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = position
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat
        self.pose_pub.publish(pose)

        self.get_logger().info(
            f"\nEnd-effector position:\n  x = {position[0]:.4f}, y = {position[1]:.4f}, z = {position[2]:.4f}\n"
        )
        self.print_dh_table(joint_angles)

    def print_dh_table(self, joint_angles):
        header = f"{'Joint':<20}{'a (m)':<12}{'alpha (deg)':<15}{'d (m)':<12}{'theta (deg)':<15}"
        print("\n--- UR3e DH Parameter Table (with current joint angles) ---")
        print(header)
        print("-" * len(header))

        for i, (a, alpha, d, theta_offset) in enumerate(self.dh_params):
            theta = joint_angles[i] + theta_offset
            print(f"{'Joint '+str(i+1):<20}{a:<12.5f}{np.degrees(alpha):<15.2f}{d:<12.5f}{np.degrees(theta):<15.2f}")

    def rotation_matrix_to_quaternion(self, R):
        """Convert rotation matrix to quaternion."""
        trace = np.trace(R)
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2,1] - R[1,2]) * s
            y = (R[0,2] - R[2,0]) * s
            z = (R[1,0] - R[0,1]) * s
        else:
            if R[0,0] > R[1,1] and R[0,0] > R[2,2]:
                s = 2.0 * np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
                w = (R[2,1] - R[1,2]) / s
                x = 0.25 * s
                y = (R[0,1] + R[1,0]) / s
                z = (R[0,2] + R[2,0]) / s
            elif R[1,1] > R[2,2]:
                s = 2.0 * np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
                w = (R[0,2] - R[2,0]) / s
                x = (R[0,1] + R[1,0]) / s
                y = 0.25 * s
                z = (R[1,2] + R[2,1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
                w = (R[1,0] - R[0,1]) / s
                x = (R[0,2] + R[2,0]) / s
                y = (R[1,2] + R[2,1]) / s
                z = 0.25 * s
        return [x, y, z, w]

def main(args=None):
    rclpy.init(args=args)
    node = UR3eFKNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
