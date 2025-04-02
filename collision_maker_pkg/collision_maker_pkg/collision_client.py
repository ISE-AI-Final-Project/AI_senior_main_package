#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import struct
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from custom_srv_pkg.srv import PointCloudSend  # Import the custom service

class CollisionMakerClient(Node):
    def __init__(self):
        super().__init__('Collision_Maker_client')
        self.client = self.create_client(PointCloudSend, 'CollisionMaker')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for service 'pointcloud_to_bool' to become available...")
        self.pointcloud_to_bool()

    def pointcloud_to_bool(self):
        request = PointCloudSend.Request()

        # Create a header with the current time and frame ID.
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "base_link"

        # Example point cloud data: three points (x, y, z)
        points = [
            (1.0, 2.0, 3.0),
            (4.0, 5.0, 6.0),
            (7.0, 8.0, 9.0)
        ]
        # Pack each point as three float32 values (little-endian).
        data = b''.join([struct.pack('fff', *pt) for pt in points])

        # Create and populate the PointCloud2 message.
        pc2 = PointCloud2()
        pc2.header = header
        pc2.height = 1            # Unorganized point cloud (1 row)
        pc2.width = len(points)   # Number of points
        pc2.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        pc2.is_bigendian = False
        pc2.point_step = 12       # Each point consists of 3 x 4 bytes
        pc2.row_step = pc2.point_step * pc2.width
        pc2.data = data
        pc2.is_dense = True

        request.pointcloud = pc2
        self.get_logger().info("Sending point cloud...")
        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Service responded with send_status: {response.send_status}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    client = CollisionMakerClient()
    rclpy.spin(client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
