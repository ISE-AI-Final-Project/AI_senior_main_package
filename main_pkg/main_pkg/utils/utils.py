from collections import defaultdict

import numpy as np
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Pose, PoseStamped
from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformException


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


def transform_pose_stamped(
    tf_buffer: Buffer,
    pose_msg: PoseStamped,
    current_frame: str,
    new_frame: str,
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
            new_frame,  # target
            current_frame,  # source
            rclpy.time.Time(),  # latest available
            rclpy.duration.Duration(seconds=0.5),
        )
    except TransformException as ex:
        print(f"Could not get transform from {current_frame} to {new_frame}: {ex}")
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


def transform_pose(
    tf_buffer: Buffer,
    pose_msg: Pose,
    current_frame: str,
    new_frame: str,
) -> Pose | None:
    """
    Transforms a Pose from current_frame to new_frame using tf_buffer and NumPy.

    Args:
        tf_buffer (tf2_ros.Buffer): TF buffer for lookup.
        pose_msg (Pose): Pose in current_frame.
        current_frame (str): The source frame of the pose.
        new_frame (str): The target frame to transform into.
        node_logger (optional): Node logger for warnings (if available).

    Returns:
        Pose or None: Transformed pose in new_frame, or None if transform fails.
    """
    try:
        tf = tf_buffer.lookup_transform(
            new_frame,  # target
            current_frame,  # source
            rclpy.time.Time(),  # latest available
            rclpy.duration.Duration(seconds=0.5),
        )
    except TransformException as ex:
        print(f"Could not get transform from {current_frame} to {new_frame}: {ex}")
        return None

    # Build 4x4 transformation matrix
    t = np.eye(4)
    q = tf.transform.rotation
    x = tf.transform.translation
    t[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    t[:3, 3] = [x.x, x.y, x.z]

    # Transform position
    p = pose_msg.position
    pos_in = np.array([p.x, p.y, p.z, 1.0])
    pos_out = t @ pos_in

    # Transform orientation
    q_pose = pose_msg.orientation
    r_pose = R.from_quat([q_pose.x, q_pose.y, q_pose.z, q_pose.w])
    r_tf = R.from_quat([q.x, q.y, q.z, q.w])
    r_out = r_tf * r_pose
    q_out = r_out.as_quat()

    # Create PoseStamped with generated header
    transformed = PoseStamped()
    transformed.header.frame_id = new_frame
    transformed.header.stamp = tf.header.stamp  # Use timestamp from the TF itself
    transformed.pose.position.x = pos_out[0]
    transformed.pose.position.y = pos_out[1]
    transformed.pose.position.z = pos_out[2]
    transformed.pose.orientation.x = q_out[0]
    transformed.pose.orientation.y = q_out[1]
    transformed.pose.orientation.z = q_out[2]
    transformed.pose.orientation.w = q_out[3]

    return transformed

def chain_poses(
    pose_obj_wrt_frame1: Pose | PoseStamped,
    pose_frame1_wrt_frame2: Pose | PoseStamped,
) -> Pose:
    """
    Computes pose of an object in frame2 given:
    - pose of the object in frame1
    - pose of frame1 in frame2

    Accepts both Pose and PoseStamped types as input.

    Args:
        pose_obj_wrt_frame1 (Pose or PoseStamped): Pose of the object in frame1.
        pose_frame1_wrt_frame2 (Pose or PoseStamped): Pose of frame1 in frame2.
        target_frame (str): Final frame ID (frame2).

    Returns:
        Pose: Pose of the object in frame2.
    """

    def extract_pose(p):
        return p.pose if isinstance(p, PoseStamped) else p

    def pose_to_matrix(pose: Pose) -> np.ndarray:
        T = np.eye(4)
        trans = [pose.position.x, pose.position.y, pose.position.z]
        rot = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        T[:3, :3] = R.from_quat(rot).as_matrix()
        T[:3, 3] = trans
        return T

    pose1 = extract_pose(pose_obj_wrt_frame1)
    pose2 = extract_pose(pose_frame1_wrt_frame2)

    T_obj_in_f1 = pose_to_matrix(pose1)
    T_f1_in_f2 = pose_to_matrix(pose2)
    T_obj_in_f2 = T_f1_in_f2 @ T_obj_in_f1

    # Extract final pose
    pos = T_obj_in_f2[:3, 3]
    rot = R.from_matrix(T_obj_in_f2[:3, :3]).as_quat()

    pose_out = Pose()

    pose_out.position.x = pos[0]
    pose_out.position.y = pos[1]
    pose_out.position.z = pos[2]
    pose_out.orientation.x = rot[0]
    pose_out.orientation.y = rot[1]
    pose_out.orientation.z = rot[2]
    pose_out.orientation.w = rot[3]

    return pose_out


def chain_poses_stamped(
    pose_obj_wrt_frame1: Pose | PoseStamped,
    pose_frame1_wrt_frame2: Pose | PoseStamped,
    target_frame: str,
) -> PoseStamped:
    """
    Computes pose of an object in frame2 given:
    - pose of the object in frame1
    - pose of frame1 in frame2

    Accepts both Pose and PoseStamped types as input.

    Args:
        pose_obj_wrt_frame1 (Pose or PoseStamped): Pose of the object in frame1.
        pose_frame1_wrt_frame2 (Pose or PoseStamped): Pose of frame1 in frame2.
        target_frame (str): Final frame ID (frame2).

    Returns:
        PoseStamped: Pose of the object in frame2.
    """
    transformed_pose = chain_poses(pose_obj_wrt_frame1, pose_frame1_wrt_frame2)

    pose_stamped_out = PoseStamped()
    pose_stamped_out.header.frame_id = target_frame

    # Set timestamp if either input had it
    if isinstance(pose_obj_wrt_frame1, PoseStamped):
        pose_stamped_out.header.stamp = pose_obj_wrt_frame1.header.stamp
    elif isinstance(pose_frame1_wrt_frame2, PoseStamped):
        pose_stamped_out.header.stamp = pose_frame1_wrt_frame2.header.stamp
    else:
        pose_stamped_out.header.stamp = rclpy.clock.Clock().now().to_msg()
        
    pose_stamped_out.pose = transformed_pose

    return pose_stamped_out


def pose_to_pose_stamped(pose: Pose, frame_id: str = "world", time = None) -> PoseStamped:
    pose_stamped = PoseStamped()
    pose_stamped.pose = pose
    pose_stamped.header.frame_id = frame_id
    pose_stamped.header.stamp = time if time is not None else rclpy.time.Time().to_msg()
    return pose_stamped

def pose_stamped_to_pose(pose_stamped: PoseStamped) -> Pose:
    return pose_stamped.pose


def rotation_translation_to_posestamped(rotation, translation, frame_id="world"):
    """
    Rotation Matrix and Translation to Pose Stamped

    Args:
        rotation (np.array): 3x3 Rotation Matrix
        transation (np.array): Translation of len 3
        frame_id (str): frame id

    Returns:
        PoseStamped
    """

    quat = R.from_matrix(rotation).as_quat()

    pose_msg = PoseStamped()
    pose_msg.header.frame_id = frame_id
    pose_msg.header.stamp = rclpy.clock.Clock().now().to_msg()

    pose_msg.pose.position.x = translation[0]
    pose_msg.pose.position.y = translation[1]
    pose_msg.pose.position.z = translation[2]

    pose_msg.pose.orientation.x = quat[0]
    pose_msg.pose.orientation.y = quat[1]
    pose_msg.pose.orientation.z = quat[2]
    pose_msg.pose.orientation.w = quat[3]
    return pose_msg