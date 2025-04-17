import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from tf2_ros import Buffer, TransformListener

from custom_srv_pkg.msg import GraspPose, GraspPoses
from custom_srv_pkg.srv import BestGraspPose
from main_pkg.utils.utils import chain_poses, transform_pose


class BestGraspService(Node):
    def __init__(self):
        super().__init__("best_grasp_server")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.srv = self.create_service(
            BestGraspPose, "BestGraspPose", self.best_grasp_callback
        )
        self.get_logger().info("Best Grasp Server is ready.")

    def best_grasp_callback(self, request, response):
        num_grasps = len(request.all_grasp_poses.grasp_poses)

        self.get_logger().info(
            f"Received {num_grasps} grasp poses for object at: "
            f"x={request.object_pose.pose.position.x:.2f}, "
            f"y={request.object_pose.pose.position.y:.2f}, "
            f"z={request.object_pose.pose.position.z:.2f}"
        )

        if num_grasps == 0:
            self.get_logger().warn("No grasp poses received. Returning empty pose.")
            return response

        transformed_grasps = []
        for i, grasp in enumerate(request.all_grasp_poses.grasp_poses):
            transformed_pose = chain_poses(grasp.ht_in_meter, request.object_pose)

            if transformed_pose is not None:
                transformed_grasps.append((grasp, transformed_pose))
            else:
                self.get_logger().warn(f"Skipping grasp {i} due to failed transform.")

        if not transformed_grasps:
            self.get_logger().warn(
                "No grasp poses could be transformed. Returning empty pose."
            )
            return response

        # Condition for select best grasp
        # - Aim-robot and grip-robot must not higher than 50 cm (aim=goal)
        # - Aim-robot and grip-robot must not higher than 50 cm (aim=goal)
        # - Minimize distance grip to robot (50%)
        # - Gravity selection: select the one Z axis downward (30%)
        # - d_to_com lower better (10%)
        # - gripper_score higher better (10%)

        try:
            tf_robot = self.tf_buffer.lookup_transform(
                "world",
                "base_link",
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=1.0),
            )
            robot_base_position = np.array(
                [
                    tf_robot.transform.translation.x,
                    tf_robot.transform.translation.y,
                    tf_robot.transform.translation.z,
                ]
            )
        except TransformException as ex:
            self.get_logger().error(f"Failed to get robot base transform: {ex}")
            return response

        aim_offset = Pose()
        aim_offset.position.z = -0.04  # 4 cm backward
        gripper_offset = Pose()
        gripper_offset.position.z = -0.15

        grasp_candidates = []
        for i, (grasp_msg, grasp_grip_end) in enumerate(transformed_grasps):
            self.get_logger().info(f"i = {i}")

            grasp_aim_end = chain_poses(aim_offset, grasp_grip_end)

            # Position at Flange (Already offset Gripper)
            grasp_aim = chain_poses(gripper_offset, grasp_aim_end)
            grasp_grip = chain_poses(gripper_offset, grasp_grip_end)

            # Positions
            grip_pos = np.array(
                [grasp_grip.position.x, grasp_grip.position.y, grasp_grip.position.z]
            )
            aim_pos = np.array(
                [grasp_aim.position.x, grasp_aim.position.y, grasp_aim.position.z]
            )

            # Distances to robot
            grip_to_base = np.linalg.norm(grip_pos - robot_base_position)
            aim_to_base = np.linalg.norm(aim_pos - robot_base_position)

            self.get_logger().info(f"grip_to_base = {grip_to_base}")
            self.get_logger().info(f"aim_to_base = {aim_to_base}")
            # Reject if either is farther than 0.5m
            if grip_to_base > 0.5 or aim_to_base > 0.5:
                continue

            # Get Z-axis direction
            quat = [
                grasp_grip.orientation.x,
                grasp_grip.orientation.y,
                grasp_grip.orientation.z,
                grasp_grip.orientation.w,
            ]
            rot_matrix = R.from_quat(quat).as_matrix()
            z_axis = rot_matrix[:, 2]
            gravity_score = abs(
                np.dot(z_axis, np.array([0, 0, -1]))
            )  # 1 when pointing downward
            self.get_logger().info(f"gravity_score = {gravity_score}")
            self.get_logger().info(f"d_to_com = {grasp_msg.d_to_com:.3f}")
            self.get_logger().info(f"gripper_score = {grasp_msg.gripper_score:.3f}")

            # Score: smaller grip distance + Z-down preference
            score = (
                0.5 * (1 - (grip_to_base))  # closer grip = better  range 0.15-0.5
                + 0.3 * gravity_score  # downward Z-axis
                + 0.1 * (1 - (grasp_msg.d_to_com))  # closer to CoM
                + 0.1 * grasp_msg.gripper_score  # gripper success prediction
            )
            self.get_logger().info(f"score = {score}")
            grasp_candidates.append(
                (score, grasp_grip_end, grasp_aim_end, grasp_msg.grip_distance)
            )

        # Sort by score (descending)
        sorted_grasp_pair = sorted(grasp_candidates, key=lambda x: x[0], reverse=True)
        sorted_score = [s for s, _, _, _ in sorted_grasp_pair]
        sorted_grip = [g for _, g, _, _ in sorted_grasp_pair]
        sorted_aim = [g for _, _, g, _ in sorted_grasp_pair]
        sorted_grip_distance = [d for _, _, _, d in sorted_grasp_pair]

        # Pack into PoseArray
        sorted_grip_poses_array = PoseArray()
        sorted_grip_poses_array.header.frame_id = "world"
        sorted_grip_poses_array.poses = sorted_grip

        sorted_aim_poses_array = PoseArray()
        sorted_aim_poses_array.header.frame_id = "world"
        sorted_aim_poses_array.poses = sorted_aim

        # Fill response
        response.sorted_aim_poses = sorted_aim_poses_array
        response.sorted_grip_poses = sorted_grip_poses_array
        response.gripper_distance = sorted_grip_distance

        self.get_logger().info(
            f"{len(sorted_grip)} best grasp poses sorted and returned."
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = BestGraspService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
