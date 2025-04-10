import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
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

        # Calculate All Score
        grasp_pair = []
        score_theshold = 0.5
        for grasp in transformed_grasps:
            # Random
            grasp_score = np.random.random()
            if grasp_score > score_theshold:
                grasp_pair.append((grasp_score, grasp))

        # Sort by score
        sorted_grasp_pair = sorted(grasp_pair, key=lambda x: x[0], reverse=True)

        # Return only the sorted poses
        sorted_grasp = [pose for _, pose in sorted_grasp_pair]

        sorted_poses_array = PoseArray()
        sorted_poses_array.header.frame_id = "world"
        sorted_poses_array.poses = sorted_grasp
        response.best_grasp_poses = sorted_poses_array

        self.get_logger().info(f"{len(sorted_grasp)} Best grasp pose sorted and returned.")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = BestGraspService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
