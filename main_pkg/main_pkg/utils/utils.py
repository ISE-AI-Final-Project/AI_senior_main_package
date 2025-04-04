from collections import defaultdict

import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2, PointField


def transform_pointcloud(msg: PointCloud2, tf, frame_id="world") -> PointCloud2:
    """
    Transfrom input PointCloud2 to tf frame
    input:
        msg: input PointCloud2
        tf: tf to transform
        frame_id: related frame

    output:
        cloud_out: PointCloud2
        transformed_xyz: np.array

    """
    # Create transformation matrix
    t = np.eye(4)
    q = tf.transform.rotation
    x = tf.transform.translation
    t[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    t[:3, 3] = [x.x, x.y, x.z]

    # Read points (x, y, z, rgb)
    points_msg = pc2.read_points_numpy(
        msg, field_names=("x", "y", "z", "rgb"), skip_nans=True
    )

    # Homogeneous coordinates
    num_points = len(points_msg)
    points_hom = np.ones((num_points, 4), dtype=np.float32)
    points_hom[:, :3] = points_msg[:, :3]

    # Apply transform
    transformed_xyz = (t @ points_hom.T).T[:, :3]

    # Create structured array with x, y, z, rgb
    transformed_data = np.empty(
        num_points,
        dtype=[
            ("x", np.float32),
            ("y", np.float32),
            ("z", np.float32),
            ("rgb", np.float32),
        ],
    )
    transformed_data["x"] = transformed_xyz[:, 0]
    transformed_data["y"] = transformed_xyz[:, 1]
    transformed_data["z"] = transformed_xyz[:, 2]
    transformed_data["rgb"] = np.array(points_msg[:, -1], dtype=np.float32)

    # Output fields
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    ]

    # Build new message
    header = msg.header
    header.frame_id = frame_id
    cloud_out = pc2.create_cloud(header, fields, transformed_data)

    return cloud_out, transformed_xyz


def pointcloud_xyz_to_simple_collision(
    points_xyz: np.array, voxel_size=0.1, max_distance=1, min_pcl_per_cube=5
):
    """
    Convert pointcloud xyz to cube collision object

    output:
        cloud_out: PointCloud2
        transformed_xyz: np.array
    """

    # Get bounds and compute voxel indices
    min_bound = np.floor(points_xyz.min(axis=0) / voxel_size) * voxel_size
    voxel_indices = np.floor((points_xyz - min_bound) / voxel_size).astype(int)

    # Count points in each voxel and store minimum distance
    voxel_counts = defaultdict(int)
    voxel_distances = {}

    for pt, vidx in zip(points_xyz, voxel_indices):
        dist = np.linalg.norm(pt)
        if dist > max_distance:
            continue
        key = tuple(vidx)
        voxel_counts[key] += 1
        if key not in voxel_distances or dist < voxel_distances[key]:
            voxel_distances[key] = dist

    # Filter voxels that have 5 or more points
    filtered_voxels = {
        k: voxel_distances[k]
        for k in voxel_counts
        if voxel_counts[k] >= min_pcl_per_cube
    }

    # Normalize distances for color mapping
    distances = np.array(list(filtered_voxels.values()))
    min_dist = distances.min()
    max_dist = distances.max()
    norm_dists = (distances - min_dist) / (max_dist - min_dist + 1e-6)

    # Color function: red to blue
    def dist_to_color(norm_val):
        return [1 - norm_val, 0, norm_val]

    # Create and color cubes
    cubes = []
    for voxel, norm_dist in zip(filtered_voxels.keys(), norm_dists):
        cube_center = min_bound + np.array(voxel) * voxel_size + (voxel_size / 2)
        cubes.append(cube_center)

    print(len(cubes))

    return cubes

    # Visualize
    # if cubes:
    #     combined = cubes[0]
    #     for cube in cubes[1:]:
    #         combined += cube
    #     o3d.visualization.draw_geometries([combined, pcd])
    # else:
    #     print("No voxels with 5 or more points found.")


from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformException, Buffer
from scipy.spatial.transform import Rotation as R
import numpy as np
import rclpy


def transform_pose(
    tf_buffer: Buffer,
    pose_msg: PoseStamped,
    current_frame: str,
    new_frame: str,
    node_logger=None,
) -> PoseStamped | None:
    """
    Transforms a PoseStamped from current_frame to new_frame using tf_buffer and NumPy.

    Args:
        tf_buffer (tf2_ros.Buffer): TF buffer for lookup.
        pose_msg (PoseStamped): Pose in current_frame.
        current_frame (str): The source frame of the pose.
        new_frame (str): The target frame to transform into.
        node_logger (optional): Node logger for warnings (if available).

    Returns:
        PoseStamped or None: Transformed pose in new_frame, or None if transform fails.
    """
    try:
        tf = tf_buffer.lookup_transform(
            new_frame,             # target
            current_frame,         # source
            rclpy.time.Time(),     # latest available
            rclpy.duration.Duration(seconds=0.5)
        )
    except TransformException as ex:
        if node_logger:
            node_logger.warn(f"Could not get transform from {current_frame} to {new_frame}: {ex}")
        return None

    # Build 4x4 transformation matrix
    t = np.eye(4)
    q = tf.transform.rotation
    x = tf.transform.translation
    t[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    t[:3, 3] = [x.x, x.y, x.z]

    # Transform position
    p = pose_msg.pose.position
    pos_in = np.array([p.x, p.y, p.z, 1.0])
    pos_out = t @ pos_in

    # Transform orientation
    q_pose = pose_msg.pose.orientation
    r_pose = R.from_quat([q_pose.x, q_pose.y, q_pose.z, q_pose.w])
    r_tf = R.from_quat([q.x, q.y, q.z, q.w])
    r_out = r_tf * r_pose
    q_out = r_out.as_quat()

    # Build output PoseStamped
    transformed_pose = PoseStamped()
    transformed_pose.header = pose_msg.header
    transformed_pose.header.frame_id = new_frame
    transformed_pose.pose.position.x = pos_out[0]
    transformed_pose.pose.position.y = pos_out[1]
    transformed_pose.pose.position.z = pos_out[2]
    transformed_pose.pose.orientation.x = q_out[0]
    transformed_pose.pose.orientation.y = q_out[1]
    transformed_pose.pose.orientation.z = q_out[2]
    transformed_pose.pose.orientation.w = q_out[3]

    return transformed_pose
