import rclpy
from rclpy.node import Node
from custom_srv_pkg.srv import GraspPoseSend
from custom_srv_pkg.msg import GraspPose, GraspPoses
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose

class AllGraspServer(Node):
    def __init__(self):
        super().__init__('all_grasp_service')
        self.srv = self.create_service(GraspPoseSend, 'GraspPose', self.handle_grasp_pose_request)
        self.get_logger().info('Grasp Pose Server is ready.')

    def handle_grasp_pose_request(self, request, response):
        self.get_logger().info(f'Received request for: {request.target_obj}')

        # Create the response message
        grasp_poses_msg = GraspPoses()

        grasp_pose1 = GraspPose()
        grasp_pose1.ht_in_meter = Pose()
        grasp_pose1.d_to_com = 0.05
        grasp_pose1.gripper_score = 0.9

        grasp_pose2 = GraspPose()
        grasp_pose2.ht_in_meter = Pose()
        grasp_pose2.d_to_com = 0.05
        grasp_pose2.gripper_score = 0.9

        grasp_poses_msg.grasp_poses.append(grasp_pose1)
        grasp_poses_msg.grasp_poses.append(grasp_pose2)

        response.grasp_poses = grasp_poses_msg  # Assign to response

        self.get_logger().info('Sending.')
        print(grasp_poses_msg)

        return response

def main():
    rclpy.init()
    node = AllGraspServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
