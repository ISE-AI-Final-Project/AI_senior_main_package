from main_pkg.utils import transform_pose
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from custom_srv_pkg.msg import GraspPoses
from custom_srv_pkg.srv import BestGraspPose  # Replace with your actual package

class BestGraspService(Node):
    def __init__(self):
        super().__init__('best_grasp_server')
        self.srv = self.create_service(BestGraspPose, 'best_grasp_pose', self.best_grasp_callback)
        self.get_logger().info("Best Grasp Server is ready.")

    def best_grasp_callback(self, request, response):
        num_grasps = len(request.all_grasp_poses.grasp_poses)
        self.get_logger().info(f"Received {num_grasps} grasp poses for object at: "
                               f"x={request.object_pose.pose.position.x:.2f}, "
                               f"y={request.object_pose.pose.position.y:.2f}, "
                               f"z={request.object_pose.pose.position.z:.2f}")

        if num_grasps == 0:
            self.get_logger().warn("No grasp poses received. Returning empty pose.")
            return response  # best_grasp_pose will remain default/empty

        # Example logic: just pick the first pose as "best"
        best_grasp_pose = request.all_grasp_poses.grasp_poses[0]

        # Fill response
        response.best_grasp_pose = best_grasp_pose
        self.get_logger().info("Best grasp pose selected and returned.")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = BestGraspService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
