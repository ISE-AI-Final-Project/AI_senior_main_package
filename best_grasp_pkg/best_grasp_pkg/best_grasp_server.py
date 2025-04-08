import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from custom_srv_pkg.msg import GraspPose, GraspPoses
from custom_srv_pkg.srv import BestGraspPose 
from main_pkg.utils.utils import transform_pose, chain_poses
from tf2_ros import Buffer, TransformListener

class BestGraspService(Node):
    def __init__(self):
        super().__init__('best_grasp_server')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.srv = self.create_service(BestGraspPose, 'BestGraspPose', self.best_grasp_callback)
        self.get_logger().info("Best Grasp Server is ready.")


    def best_grasp_callback(self, request, response):
        num_grasps = len(request.all_grasp_poses.grasp_poses)

        self.get_logger().info(f"Received {num_grasps} grasp poses for object at: "
                            f"x={request.object_pose.pose.position.x:.2f}, "
                            f"y={request.object_pose.pose.position.y:.2f}, "
                            f"z={request.object_pose.pose.position.z:.2f}")

        if num_grasps == 0:
            self.get_logger().warn("No grasp poses received. Returning empty pose.")
            return response


        # best_grasp = request.all_grasp_poses.grasp_poses[0]
        # pose_stamped = PoseStamped()
        # pose_stamped.header.frame_id = request.object_pose.header.frame_id
        # pose_stamped.header.stamp = self.get_clock().now().to_msg()
        # pose_stamped.pose = best_grasp.ht_in_meter

        # response.best_grasp_pose = pose_stamped
        # self.get_logger().info("Best grasp pose selected and returned.")
        # return response
    
        transformed_grasps = []
        for i, grasp in enumerate(request.all_grasp_poses.grasp_poses):
            transformed_posestamped = chain_poses(grasp.ht_in_meter, request.object_pose, target_frame="world")
            
            if transformed_posestamped is not None:
                transformed_grasps.append(transformed_posestamped)
            else:
                self.get_logger().warn(f"Skipping grasp {i} due to failed transform.")

        if not transformed_grasps:
            self.get_logger().warn("No grasp poses could be transformed. Returning empty pose.")
            return response

        # Example: pick the first transformed pose as the "best"
        response.best_grasp_pose = transformed_grasps[0]
        self.get_logger().info("Best grasp pose selected and returned.")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = BestGraspService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
