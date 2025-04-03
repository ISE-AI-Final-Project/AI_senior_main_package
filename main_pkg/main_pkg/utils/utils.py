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
