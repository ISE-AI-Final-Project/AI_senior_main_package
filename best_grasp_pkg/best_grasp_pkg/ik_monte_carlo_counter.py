import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from custom_srv_pkg.srv import IKPassCount
import numpy as np
from scipy.spatial.transform import Rotation as R

def perturb_pose(
    pose: Pose,
    pos_sigma: float = 0.005,    # position noise STD [m]
    rot_sigma_deg: float = 5.0    # orientation noise STD [deg]
) -> Pose:
    """
    Return a new Pose equal to `pose` plus Gaussian noise
    on position (in meters) and orientation (in degrees).
    """
    p = Pose()
    # position
    p.position.x = pose.position.x + np.random.normal(scale=pos_sigma)
    p.position.y = pose.position.y + np.random.normal(scale=pos_sigma)
    p.position.z = pose.position.z + np.random.normal(scale=pos_sigma)

    # orientation
    base_q = [pose.orientation.x,
              pose.orientation.y,
              pose.orientation.z,
              pose.orientation.w]
    base_rot = R.from_quat(base_q)
    delta_rot = R.from_euler(
        'xyz',
        np.random.normal(scale=rot_sigma_deg, size=3),
        degrees=True
    )
    q_new = (delta_rot * base_rot).as_quat()
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q_new
    return p


class MonteCarloIKCounter(Node):
    def __init__(self):
        super().__init__('ik_monte_carlo_counter')

        # 1) Service client for ik_pass_count
        self.cli = self.create_client(IKPassCount, 'ik_pass_count')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for "ik_pass_count" service...')

        # 2) Base PoseArray (replace this with your actual data source)
        #    For example, if you have it as a member variable called
        #    self.data_sorted_grasp_aim_pose, you can assign it here:
        #
        #    self.base_poses = self.data_sorted_grasp_aim_pose
        #
        #    In this template we'll park an empty PoseArray:
        self.base_poses = PoseArray()
        self.base_poses.header.frame_id = 'world'
        # TODO: fill self.base_poses.poses with your sorted aim poses

        # 3) Monte Carlo parameters
        self.num_samples = 100

        # 4) Run once after a short delay
        self.create_timer(1.0, self._on_timer)

    def _on_timer(self):
        success_count = 0
        total = self.num_samples

        for i in range(total):
            # Build perturbed PoseArray
            req = IKPassCount.Request()
            pa = PoseArray()
            pa.header.frame_id = self.base_poses.header.frame_id
            pa.poses = [perturb_pose(p) for p in self.base_poses.poses]
            req.pose_array = pa

            # Call service and wait
            future = self.cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)

            if future.result() is not None:
                # Assume response.pass_count equals number of poses that solved
                if future.result().pass_count == len(pa.poses):
                    success_count += 1
            else:
                self.get_logger().error('Service call to ik_pass_count failed')

        ratio = success_count / float(total)
        self.get_logger().info(
            f'[Monte Carlo] {success_count}/{total} full passes → ratio={ratio:.2f}'
        )

        # If you only want to run once, shut down:
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = MonteCarloIKCounter()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
