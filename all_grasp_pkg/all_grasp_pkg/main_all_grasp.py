import time
import timeit

import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R

from .my_gripper_with_contact import MyGripperWithContact

np.random.seed(10)


def matrix_to_quaternion(matrix):
    """
    Convert the rotation part of a 4x4 homogeneous transformation matrix to a quaternion.
    Returns a numpy array [x, y, z, w].
    """
    M = matrix[:3, :3]
    trace = np.trace(M)
    if trace > 0:
        S = np.sqrt(trace + 1.0) * 2  # S = 4 * qw
        qw = 0.25 * S
        qx = (M[2, 1] - M[1, 2]) / S
        qy = (M[0, 2] - M[2, 0]) / S
        qz = (M[1, 0] - M[0, 1]) / S
    elif (M[0, 0] > M[1, 1]) and (M[0, 0] > M[2, 2]):
        S = np.sqrt(1.0 + M[0, 0] - M[1, 1] - M[2, 2]) * 2  # S = 4 * qx
        qw = (M[2, 1] - M[1, 2]) / S
        qx = 0.25 * S
        qy = (M[0, 1] + M[1, 0]) / S
        qz = (M[0, 2] + M[2, 0]) / S
    elif M[1, 1] > M[2, 2]:
        S = np.sqrt(1.0 + M[1, 1] - M[0, 0] - M[2, 2]) * 2  # S = 4 * qy
        qw = (M[0, 2] - M[2, 0]) / S
        qx = (M[0, 1] + M[1, 0]) / S
        qy = 0.25 * S
        qz = (M[1, 2] + M[2, 1]) / S
    else:
        S = np.sqrt(1.0 + M[2, 2] - M[0, 0] - M[1, 1]) * 2  # S = 4 * qz
        qw = (M[1, 0] - M[0, 1]) / S
        qx = (M[0, 2] + M[2, 0]) / S
        qy = (M[1, 2] + M[2, 1]) / S
        qz = 0.25 * S
    return np.array([qx, qy, qz, qw])

def transform_vector_to_new_basis(v, new_n, old_n=np.array([0, 0, 1])):
    """
    Transform vector v from world basis old_n to a new world basis where n is the new normal.

    Parameters:
    - v: np.array([vx, vy, vz]) -> Input vector in the old world coordinates
    - new_n: np.array([nx, ny, nz]) -> New normal (assumed normalized)

    Returns:
    - transformed_v: np.array([vx', vy', vz']) -> Vector in the new coordinate system
    """

    # Compute rotation to align old_z to new_n
    rotation_axis = np.cross(old_n, new_n)  # Axis to rotate around
    rotation_angle = np.arccos(np.dot(old_n, new_n))  # Angle to rotate

    # Create rotation matrix (handle edge cases where n is nearly parallel)
    if np.linalg.norm(rotation_axis) < 1e-6:
        rotation_matrix = np.eye(3)  # No rotation needed (parallel vectors)
    else:
        rotation_axis /= np.linalg.norm(rotation_axis)  # Normalize axis
        rotation_matrix = R.from_rotvec(rotation_angle * rotation_axis).as_matrix()

    # Apply rotation to (1,0,0)
    transformed_v = rotation_matrix @ v

    return transformed_v


def points_to_line_distance(points, line_point, line_direction):
    """
    Computes the shortest distance from a 3D point to a line.

    Args:
        point: (3,) array-like, the 3D point (P).
        line_point: (3,) array-like, a point on the line (A).
        line_direction: (3,) array-like, the direction vector of the line (v).

    Returns:
        float: The perpendicular distance from the point to the line.
    """
    points = np.array(points)
    line_point = np.array(line_point)
    line_direction = np.array(line_direction)

    # Normalize direction vector
    line_direction = line_direction / np.linalg.norm(line_direction)

    # Vector from line point to given point
    vec_AP = points - line_point

    # Cross product to get perpendicular component
    cross_prod = np.cross(vec_AP, line_direction)

    # Distance formula
    distance = np.linalg.norm(cross_prod, axis=1) / np.linalg.norm(line_direction)
    return distance


def l2_distance(p1, p2):
    return np.sqrt(np.sum((p1 - p2) ** 2))


if __name__ == "__main__":
    # Constant Param ########################
    K_ESTIMATE_NORMAL = 10
    K_GROUPING = 5

    MIN_NUM_GROUP_MEMBER = 10

    ADJACENT_THRESHOLD_ANGLE = 3
    OPPOSITE_THRESHOLD_ANGLE = 15

    SAMPLE_GRID_SIZE = 10  # In mm

    ##########################################
    # Load a point cloud
    pcd = o3d.io.read_point_cloud("obj/obj_10.ply")

    # Build a KDTree for neighbor search
    kdtree = o3d.geometry.KDTreeFlann(pcd)

    colors = np.zeros_like(pcd.colors)

    # Estimate normals
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=K_ESTIMATE_NORMAL)
    )
    normals = np.asarray(pcd.normals)
    points = np.asarray(pcd.points)

    # 1. Point Cloud Grouping ##################################
    adjacent_cos_threshold = np.cos(np.radians(ADJACENT_THRESHOLD_ANGLE))

    group_index = np.zeros(colors.shape[0])
    group_counter = 1
    group_anchor_normal_vector = dict()

    print("Grouping Point Clouds...")
    for idx, point in enumerate(np.asarray(pcd.points)):
        current_point_group_idx = group_index[idx]

        if current_point_group_idx == 0:
            # New group
            current_point_group_idx = group_counter
            group_counter += 1
            group_anchor_normal_vector[current_point_group_idx] = normals[idx]
            group_index[idx] = current_point_group_idx

        current_point_normal = group_anchor_normal_vector.get(current_point_group_idx)

        [_, knn_idx, _] = kdtree.search_knn_vector_3d(point, K_GROUPING)

        for nn_point_idx in knn_idx[1:]:

            # Get group id of nn
            nn_group_idx = group_index[nn_point_idx]

            if nn_group_idx == 0:
                # Not in a group
                adjacent_dot_prod = np.dot(current_point_normal, normals[nn_point_idx])

                if adjacent_dot_prod > adjacent_cos_threshold:
                    # Assign to group
                    group_index[nn_point_idx] = current_point_group_idx
            else:
                # In a group
                anchor_vector = group_anchor_normal_vector.get(nn_group_idx)

                adjacent_dot_prod = np.dot(current_point_normal, anchor_vector)

                if adjacent_dot_prod > adjacent_cos_threshold:
                    # Join group
                    group_index[np.where(group_index == nn_group_idx)] = (
                        current_point_group_idx
                    )

    unique_group, unique_count = np.unique(group_index, return_counts=True)
    print(
        "Number of big groups:",
        len(unique_group[np.where(unique_count >= MIN_NUM_GROUP_MEMBER)]),
    )

    # 2. Sample points on surface ###############################
    target_idx_list = []

    print("Sample Target Points...")
    for big_group_index in unique_group[unique_count >= MIN_NUM_GROUP_MEMBER]:
        if big_group_index == 0:
            continue  # Skip index 0

        # Assign random color to the group
        group_mask = group_index == big_group_index
        colors[group_mask] = np.random.random(3) * np.array((1, 0.7, 1))

        # Find mean position & normal
        group_points = points[group_mask]
        group_normals = normals[group_mask]
        mean_coord = np.mean(group_points, axis=0)
        mean_normal = np.mean(group_normals, axis=0)

        # Find closest point to mean_coord
        _, center_idx, _ = kdtree.search_knn_vector_3d(mean_coord, 1)
        colors[center_idx] = (0, 1, 0)  # Mark center as green

        # Find max-min
        max_coord = np.max(group_points, axis=0)
        min_coord = np.min(group_points, axis=0)

        diff_max = np.abs(max_coord - mean_coord)
        diff_min = np.abs(min_coord - mean_coord)

        # Compute basis transformation once
        new_basis_increment_vector = np.array(
            [
                transform_vector_to_new_basis(
                    v=np.array([SAMPLE_GRID_SIZE, 0, 0]), new_n=mean_normal
                ),
                transform_vector_to_new_basis(
                    v=np.array([0, SAMPLE_GRID_SIZE, 0]), new_n=mean_normal
                ),
            ]
        )

        # Calculate max increment range
        num_increment = int(
            np.max((np.linalg.norm(diff_max), np.linalg.norm(diff_min)))
            // SAMPLE_GRID_SIZE
        )

        # Loop through x and y
        for new_y in range(-num_increment, num_increment + 1):
            for new_x in range(-num_increment, num_increment + 1):
                new_coord = (
                    mean_coord
                    + new_basis_increment_vector[0] * new_x
                    + new_basis_increment_vector[1] * new_y
                )

                _, new_p_idx, _ = kdtree.search_knn_vector_3d(new_coord, 1)
                new_p_idx = new_p_idx[0]
                if group_index[new_p_idx] == big_group_index:
                    colors[new_p_idx] = (0, 1, 0)  # Mark as green
                    target_idx_list.append(new_p_idx)

    print("Num target point:", len(target_idx_list))

    # 3. Generate lines from target_idx_list #######################
    opposite_cos_threshold = np.cos(np.radians(OPPOSITE_THRESHOLD_ANGLE))

    line_set_list = []
    line_set_index = []

    print("Generating Lines...")
    for point_idx in target_idx_list:
        start_point_coord = points[point_idx]
        line_direction = np.asarray(pcd.normals)[point_idx] * -1
        p2l_d = points_to_line_distance(points, start_point_coord, line_direction)

        for oppos_point_idx in np.where(p2l_d < 1)[0]:
            if oppos_point_idx == point_idx:
                continue

            opposite_dot_prod = np.dot(-normals[point_idx], normals[oppos_point_idx])

            if opposite_dot_prod > opposite_cos_threshold:
                line_set_id = tuple(sorted((point_idx, oppos_point_idx)))
                if line_set_id in line_set_index:
                    # Check for dupe
                    continue

                # Create an Open3D LineSet object
                line_set = o3d.geometry.LineSet()

                # Define the points and edges of the line
                line_set.points = o3d.utility.Vector3dVector(
                    [start_point_coord, points[oppos_point_idx]]
                )
                line_set.lines = o3d.utility.Vector2iVector(
                    [[0, 1]]
                )  # Connect the two points

                # Set line color (e.g., red)
                line_set.colors = o3d.utility.Vector3dVector([[1, 0, 0]])  # RGB: Red

                line_set_list.append(line_set)
                line_set_index.append(line_set_id)

    print("Number of Lines:", len(line_set_list))

    # Simulating Gripper ###################################

    # Hand -e
    # my_gripper = MyGripper(
    #     hand_width=25,
    #     hand_thickness=14,
    #     hand_length=35,
    #     gripper_min_d=0,
    #     gripper_max_d=50,
    #     back_thickness=40,
    # )

    # Simulate Gripper on all line
    possible_gripper = []
    all_gripper_geometries = []
    gripper_area_score = []

    print("Simulating Gripper...")
    for point1_idx, point2_idx in line_set_index:

        for y_angle in range(0, 360, 20):
            my_gripper = MyGripperWithContact(
                hand_width=25,
                hand_thickness=14,
                hand_length=55,
                gripper_min_d=0,
                gripper_max_d=120,
                back_thickness=40,
                back_width=60,
                back_height=60,
                contact_thickness=4,
                skeleton_radius=1,
            )

            # Align gripper with 2 points in line
            my_gripper.align_with_points(
                points[point1_idx],
                points[point2_idx],
                angle_around_y=y_angle,
                close=True,
            )
            if my_gripper.check_possible(pcd):
                possible_gripper.append(my_gripper)
                # all_gripper_geometries.extend(my_gripper.geometries())
                all_gripper_geometries.extend(my_gripper.get_skeleton(radius=1))

                gripper_area_score.append(my_gripper.count_contact_area(pcd))

    print("Number of possible Gripper:", len(possible_gripper))

    # Sort gripper with the highest scores ################
    highest_gripper_geometries = []
    gripper_coordinate_frames = []

    for index in np.argsort(gripper_area_score)[:-11:-1]:
        # highest_gripper_geometries.extend(possible_gripper[index].geometries())
        highest_gripper_geometries.extend(possible_gripper[index].get_skeleton())

        # Get HT and HT_in_meter
        HT = possible_gripper[index].HT.copy()
        HT_in_meter = possible_gripper[index].HT.copy()
        HT_in_meter[:3, 3] /= 1000

        # Calculate Distance to Center of mass
        center_of_mass = np.mean(points, axis=0)

        d_to_com = l2_distance(center_of_mass, HT[:3, 3]) / 1000

        # Create Coordinate Frame for visualization
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10.0)
        frame.transform(HT)
        gripper_coordinate_frames.append(frame)

        print(
            f"Contact Area : {gripper_area_score[index]}, Distance to CoM : {d_to_com}"
        )
        print(HT_in_meter)

    # Create Coordinate Frame at center
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10.0)
    gripper_coordinate_frames.append(frame)

    # Visualize the point cloud and fitted plane
    pcd.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw_geometries([pcd, *all_gripper_geometries, *line_set_list])
    o3d.visualization.draw_geometries(
        [pcd, *highest_gripper_geometries, *gripper_coordinate_frames]
    )

    o3d.visualization.draw_geometries(
        [*highest_gripper_geometries, *gripper_coordinate_frames]
    )