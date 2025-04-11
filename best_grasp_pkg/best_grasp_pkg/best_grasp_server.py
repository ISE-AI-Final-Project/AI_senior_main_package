import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped, Pose
from rclpy.node import Node
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
                transformed_grasps.append(transformed_pose)
            else:
                self.get_logger().warn(f"Skipping grasp {i} due to failed transform.")

        if not transformed_grasps:
            self.get_logger().warn(
                "No grasp poses could be transformed. Returning empty pose."
            )
            return response


        # Condition for select best grasp
        # - Goal and grip must pass minimun 50 cm
        # - Minimize distance grip to arm robot
        # - Gravity selection: select the one Z axis downward 

        grasp_pair = []
        score_threshold = 0.5
        robot_base_position = np.array([0.0, 0.0, 0.0])  # Example robot base at origin

        # tf base_link to world
        # try:
        #     tf_robot = self.tf_buffer.lookup_transform(
        #         "world", "base_link", rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0)
        #     )
        #     robot_base_position = np.array([
        #         tf_robot.transform.translation.x,
        #         tf_robot.transform.translation.y,
        #         tf_robot.transform.translation.z
        #     ])
        # except TransformException as ex:
        #     self.get_logger().error(f"Failed to get robot base transform: {ex}")
        #     return response

        # for grasp in transformed_grasps:
        #     obj_pos = np.array([
        #         request.object_pose.pose.position.x,
        #         request.object_pose.pose.position.y,
        #         request.object_pose.pose.position.z
        #     ])
        #     grasp_pos = np.array([
        #         grasp.position.x,
        #         grasp.position.y,
        #         grasp.position.z
        #     ])
        #     goal_to_grip_distance = np.linalg.norm(obj_pos - grasp_pos + 0.08)

        #     if goal_to_grip_distance < 0.5:
        #         continue

        #     grip_to_robot_distance = np.linalg.norm(grasp_pos - robot_base_position)

        #     import tf_transformations
        #     quat = [
        #         grasp.orientation.x,
        #         grasp.orientation.y,
        #         grasp.orientation.z,
        #         grasp.orientation.w,
        #     ]
        #     rot_matrix = tf_transformations.quaternion_matrix(quat)[:3, :3]
        #     z_axis = rot_matrix[:, 2]
        #     gravity_score = np.dot(z_axis, np.array([0, 0, -1]))

        #     score = 0.4 * (1.0 / (1e-3 + grip_to_robot_distance)) + 0.6 * gravity_score

        #     if score > score_threshold:
        #         grasp_pair.append((score, grasp))


        # Calculate All Score
        grasp_list = []
        score_theshold = 0.5
        for grasp_grip in transformed_grasps:

            goal_offset = Pose()
            goal_offset.position.x = 0.0
            goal_offset.position.y = 0.0
            goal_offset.position.z = -0.04 # 4 cm back

            grasp_goal = chain_poses(goal_offset, grasp_grip)

            # Random
            grasp_score = np.random.random()
            if grasp_score > score_theshold:
                grasp_list.append((grasp_score, grasp_grip, grasp_goal))

        # Sort by score
        sorted_grasp_pair = sorted(grasp_list, key=lambda x: x[0], reverse=True)

        # Return only the sorted poses
        sorted_score = [score for score, grip, goal in sorted_grasp_pair]
        sorted_grip = [grip for score, grip, goal in sorted_grasp_pair]
        sorted_goal = [goal for score, grip, goal in sorted_grasp_pair]

        sorted_grip_poses_array = PoseArray()
        sorted_grip_poses_array.header.frame_id = "world"
        sorted_grip_poses_array.poses = sorted_grip

        sorted_goal_poses_array = PoseArray()
        sorted_goal_poses_array.header.frame_id = "world"
        sorted_goal_poses_array.poses = sorted_goal

        # Temp
        response.sorted_goal_poses = sorted_goal_poses_array
        response.sorted_grip_poses = sorted_grip_poses_array
        response.gripper_distance = sorted_score

        self.get_logger().info(f"{len(sorted_grip)} Best grasp pose sorted and returned.")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = BestGraspService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
