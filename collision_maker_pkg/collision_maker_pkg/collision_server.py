import rclpy
from rclpy.node import Node
from custom_srv_pkg.srv import PointCloudSend
from sensor_msgs.msg import PointCloud2

class CollisionMakerService(Node):
    def __init__(self):
        super().__init__('Collision_Maker_service')
        self.srv = self.create_service(PointCloudSend, 'CollisionMaker', self.collision_maker_callback)
        self.get_logger().info("CollisionMaker service is ready.")

    def collision_maker_callback(self, request, response):
        self.get_logger().info(
            f"Received point cloud with width: {request.pointcloud.width} and height: {request.pointcloud.height}"
        )
        response.send_status = True  
        return response

def main():
    rclpy.init()
    node = CollisionMakerService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


