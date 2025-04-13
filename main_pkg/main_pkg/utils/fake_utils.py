import struct

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def get_random_pointcloud(min_range=0, max_range=1, num_points=1000):
    header = Header()
    header.stamp = rclpy.clock.Clock().now().to_msg()
    header.frame_id = "world"

    # Generate random XYZ
    xyz = np.random.uniform(low=min_range, high=max_range, size=(num_points, 3)).astype(
        np.float32
    )

    # Generate random RGB as uint8 then pack into float
    rgb_uint8 = np.random.randint(0, 256, (num_points, 3), dtype=np.uint8)
    rgb_packed = (
        np.left_shift(rgb_uint8[:, 0].astype(np.uint32), 16)
        | np.left_shift(rgb_uint8[:, 1].astype(np.uint32), 8)
        | rgb_uint8[:, 2].astype(np.uint32)
    )
    rgb_float = np.array(
        [struct.unpack("f", struct.pack("I", val))[0] for val in rgb_packed],
        dtype=np.float32,
    )

    # Pack all into binary data (x, y, z, rgb)
    data = b"".join(
        [struct.pack("ffff", *xyz[i], rgb_float[i]) for i in range(num_points)]
    )

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = num_points
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16  # 4 floats: x, y, z, rgb
    msg.row_step = msg.point_step * num_points
    msg.data = data
    msg.is_dense = True

    return msg


def get_random_pose_stamped(min_range=0, max_range=1, frame_id="world"):
    pose_msg = PoseStamped()
    pose_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
    pose_msg.header.frame_id = frame_id

    # Random position in range [-1.0, 1.0]
    pos = np.random.uniform(min_range, max_range, size=3)
    pose_msg.pose.position.x = pos[0]
    pose_msg.pose.position.y = pos[1]
    pose_msg.pose.position.z = pos[2]

    # Random orientation as quaternion
    quat = R.random().as_quat()  # [x, y, z, w]
    pose_msg.pose.orientation.x = quat[0]
    pose_msg.pose.orientation.y = quat[1]
    pose_msg.pose.orientation.z = quat[2]
    pose_msg.pose.orientation.w = quat[3]

    return pose_msg
