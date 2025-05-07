import copy

import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R


class MyGripperWithContact:
    def __init__(
        self,
        hand_width,
        hand_thickness,
        hand_length,
        gripper_min_d,
        gripper_max_d,
        back_thickness,
        back_width,
        back_height,
        contact_thickness,
        gripper_in_offset=0,
        skeleton_radius=1,
    ):
        self.hand_width = hand_width
        self.hand_thickness = hand_thickness
        self.hand_length = hand_length
        self.gripper_min_d = gripper_min_d
        self.gripper_max_d = gripper_max_d
        self.back_thickness = back_thickness
        self.contact_thickness = contact_thickness
        self.back_width = back_width
        self.back_height = back_height
        self.gripper_in_offset = gripper_in_offset
        self.skeleton_radius = skeleton_radius

        self.color = (1, 0, 0)

        self.obb_hand1 = o3d.geometry.OrientedBoundingBox(
            center=np.array(
                (
                    0,
                    gripper_max_d / 2 + hand_thickness / 2 + contact_thickness / 2,
                    -hand_length / 2 + gripper_in_offset,
                )
            ),
            R=np.identity(3),
            extent=np.array((hand_width, hand_thickness, hand_length)),
        )

        self.obb_hand2 = o3d.geometry.OrientedBoundingBox(
            center=np.array(
                (
                    0,
                    -(gripper_max_d / 2 + hand_thickness / 2 + contact_thickness / 2),
                    -hand_length / 2 + gripper_in_offset,
                )
            ),
            R=np.identity(3),
            extent=np.array((hand_width, hand_thickness, hand_length)),
        )

        self.obb_contact1 = o3d.geometry.OrientedBoundingBox(
            center=np.array(
                (
                    0,
                    gripper_max_d / 2,
                    -hand_length / 2 + gripper_in_offset,
                )
            ),
            R=np.identity(3),
            extent=np.array((hand_width, contact_thickness, hand_length)),
        )

        self.obb_contact2 = o3d.geometry.OrientedBoundingBox(
            center=np.array(
                (
                    0,
                    -gripper_max_d / 2,
                    -hand_length / 2 + gripper_in_offset,
                )
            ),
            R=np.identity(3),
            extent=np.array((hand_width, contact_thickness, hand_length)),
        )

        self.obb_gripper_range = o3d.geometry.OrientedBoundingBox(
            center=np.array((0, 0, -hand_length - back_thickness / 2 + gripper_in_offset)),
            R=np.identity(3),
            extent=np.array(
                (
                    hand_width,
                    gripper_max_d + 2 * hand_thickness + contact_thickness,
                    back_thickness,
                )
            ),
        )

        self.obb_back = o3d.geometry.OrientedBoundingBox(
            center=np.array((0, 0, -hand_length - back_thickness / 2 + gripper_in_offset)),
            R=np.identity(3),
            extent=np.array(
                (
                    back_width,
                    back_height,
                    back_thickness,
                )
            ),
        )

        self.obb_hand1.color = self.color
        self.obb_hand2.color = self.color
        self.obb_back.color = (0, 0, 0)
        self.obb_contact1.color = (0, 1, 0)
        self.obb_contact2.color = (0, 1, 0)
        self.obb_gripper_range.color = (0, 0, 0)

        self.axis = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=1, origin=[0, 0, 0]
        )

        self.HT = self.to_HT(rotation_matrix=np.eye(3), translation_vector=np.zeros(3))

    def to_HT(self, rotation_matrix=np.eye(3), translation_vector=np.zeros(3)):
        """Create a homogeneous transformation matrix from rotation and translation"""
        T = np.eye(4)  # Identity matrix of size 4x4
        T[:3, :3] = rotation_matrix  # Set rotation part
        T[:3, 3] = translation_vector  # Set translation part
        return T

    def geometries(self):
        return [
            self.obb_hand1,
            self.obb_hand2,
            self.axis,
            self.obb_back,
            self.obb_contact1,
            self.obb_contact2,
            # self.obb_gripper_range,
        ]

    def draw_geometries(self):
        o3d.visualization.draw_geometries([self.obb_hand1, self.obb_hand2])

    def draw_geometries_with_axis(self):
        o3d.visualization.draw_geometries(
            [self.obb_hand1, self.obb_hand2, self.axis, self.obb_back]
        )

    def translate(self, T):
        for geo in self.geometries():
            geo.translate(translation=T)

        # Calculate HT
        self.HT = self.to_HT(translation_vector=T) @ self.HT

    def rotate(self, R):
        for geo in self.geometries():
            geo.rotate(R=R, center=self.axis.get_center())

        # Calculate HT (Rotate Around Self)
        self.HT[:3, :3] = R @ self.HT[:3, :3]

    def check_possible(self, pcl):

        inside_hand1 = self.obb_hand1.get_point_indices_within_bounding_box(pcl.points)
        inside_hand2 = self.obb_hand2.get_point_indices_within_bounding_box(pcl.points)
        inside_back = self.obb_back.get_point_indices_within_bounding_box(pcl.points)

        # print(inside_hand1, inside_hand2)

        if len(inside_hand1) == 0 and len(inside_hand2) == 0 and len(inside_back) == 0:
            return True
        else:
            return False
        # inside_points = pcl.select_by_index(inside_indices)

    def count_contact_area(self, pcl):

        inside_contact1 = self.obb_contact1.get_point_indices_within_bounding_box(
            pcl.points
        )
        inside_contact2 = self.obb_contact2.get_point_indices_within_bounding_box(
            pcl.points
        )

        return len(inside_contact1) + len(inside_contact2)

    def close_to(self, d):
        """
        Move till center of both contact box have distance d

        Parameters:
            d (float): Desired distance between the two contact box.
        """
        # Ensure `d` is within the allowed range
        d = np.clip(d, self.gripper_min_d, self.gripper_max_d)

        # Get the current gripper center positions
        center1 = self.obb_contact1.center
        center2 = self.obb_contact2.center

        # Compute movement direction (gripper's Y-axis in world frame)
        gripper_y_dir = self.obb_hand1.R[:, 1]
        # print(gripper_y_dir)

        # Compute current distance
        current_distance = np.linalg.norm(center1 - center2)

        # Compute movement needed per hand (symmetrically)
        move_distance = (current_distance - d) / 2

        # Apply translation
        self.obb_contact1.translate(-gripper_y_dir * move_distance)
        self.obb_contact2.translate(gripper_y_dir * move_distance)

        self.obb_hand1.translate(-gripper_y_dir * move_distance)
        self.obb_hand2.translate(gripper_y_dir * move_distance)

    def rotation_matrix_from_y_axis(self, new_y):
        """
        Compute a rotation matrix that aligns the current Y-axis (0,1,0) to `new_y`.

        Parameters:
        - new_y: Target Y-axis direction (must be a 3D vector).

        Returns:
        - 3x3 rotation matrix.
        """
        # Normalize the new Y vector
        new_y = new_y / np.linalg.norm(new_y)

        # Define the original Y-axis
        current_y = np.array([0, 1, 0])

        # Compute the rotation axis (cross product)
        axis = np.cross(current_y, new_y)
        axis_norm = np.linalg.norm(axis)

        # If vectors are nearly aligned, return identity matrix (no rotation needed)
        if axis_norm < 1e-6:
            return np.eye(3) if np.dot(current_y, new_y) > 0 else -np.eye(3)

        # Normalize the rotation axis
        axis = axis / axis_norm

        # Compute the rotation angle (dot product gives cos(theta))
        angle = np.arccos(np.clip(np.dot(current_y, new_y), -1.0, 1.0))

        # Compute the skew-symmetric cross-product matrix
        K = np.array(
            [[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]]
        )

        # Rodrigues' rotation formula
        R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)

        return R

    def l2_distance(self, p1, p2):
        return np.sqrt(np.sum((p1 - p2) ** 2))

    def align_with_points(self, p1, p2, angle_around_y=0, close=False):
        """
        Align Axis with the center between p1 and p2.
        """

        line_rotation_matrix = self.rotation_matrix_from_y_axis(p1 - p2)

        self.translate(T=[(p1[i] + p2[i]) / 2 for i in range(3)])
        self.rotate(R=line_rotation_matrix)

        if close:
            self.close_to(d=self.l2_distance(p1, p2))

        if angle_around_y != 0:
            # Rotate around new y
            # Extract the current Y-axis
            current_y = self.obb_hand1.R[:, 1]

            # Compute the rotation matrix around the new Y-axis
            rotation_around_y = R.from_rotvec(
                np.radians(angle_around_y) * current_y
            ).as_matrix()

            # Rotate the OBB around its center
            self.rotate(R=rotation_around_y)

            # self.close_to(d=my_gripper.l2_distance(point[0], point[1]))

    # def to_home_state(self):
    #     self.obb_hand1, self.obb_hand2, self.obb_back, self.axis = self.home_state

    def create_cylinder_between_points(
        self, point1, point2, radius=0.01, resolution=10, split=1, color=(0, 0, 0)
    ):
        """
        Create a cylinder mesh between point1 and point2.

        Args:
            point1 (array-like): The starting point of the cylinder.
            point2 (array-like): The ending point of the cylinder.
            radius (float): The radius of the cylinder.
            resolution (int): The resolution of the cylinder (number of sides).
            split (int): The number of splits along the height.

        Returns:
            cylinder (o3d.geometry.TriangleMesh): The transformed cylinder mesh.
        """
        point1 = np.array(point1)
        point2 = np.array(point2)
        vec = point2 - point1
        height = np.linalg.norm(vec)
        if height < 1e-6:
            raise ValueError("The two points are too close to create a cylinder.")

        # Create a cylinder aligned along the z-axis, with base at (0,0,0)
        cylinder = o3d.geometry.TriangleMesh.create_cylinder(
            radius=radius, height=height, resolution=resolution, split=split
        )
        cylinder.paint_uniform_color(color)
        cylinder.compute_vertex_normals()

        # The default cylinder is along z; we need to rotate it so that it aligns with 'vec'
        # Calculate the unit direction vector
        direction = vec / height
        # z-axis unit vector
        z_axis = np.array([0, 0, 1])

        # Compute the rotation matrix that rotates z_axis to direction
        if np.allclose(direction, z_axis):
            R = np.eye(3)
        elif np.allclose(direction, -z_axis):
            # 180 degree rotation around x-axis (or any axis perpendicular to z)
            R = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        else:
            # Rodrigues' rotation formula
            v = np.cross(z_axis, direction)
            s = np.linalg.norm(v)
            c = np.dot(z_axis, direction)
            vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
            R = np.eye(3) + vx + vx @ vx * ((1 - c) / (s**2))

        # Rotate the cylinder around the origin
        cylinder.rotate(R, center=(0, 0, 0))

        # Translate the cylinder so its base is at point1
        cylinder.translate((point1 + point2) / 2)

        return cylinder

    def get_skeleton(self, radius=None):
        if radius is None:
            radius = self.skeleton_radius

        face_center1_obb1 = np.mean(
            np.asarray(self.obb_contact1.get_box_points())[[3, 4, 5, 6]], axis=0
        )
        face_center2_obb1 = np.mean(
            np.asarray(self.obb_contact1.get_box_points())[[0, 1, 2, 7]], axis=0
        )

        face_center1_obb2 = np.mean(
            np.asarray(self.obb_contact2.get_box_points())[[3, 4, 5, 6]], axis=0
        )
        face_center2_obb2 = np.mean(
            np.asarray(self.obb_contact2.get_box_points())[[0, 1, 2, 7]], axis=0
        )

        back_center = (face_center2_obb1 + face_center2_obb2) / 2

        back_center_extent = back_center + (face_center2_obb1 - face_center1_obb1) / 2

        cylinder_list = [
            self.create_cylinder_between_points(
                face_center1_obb1, face_center2_obb1, radius=radius, color=(0, 1, 0)
            ),
            self.create_cylinder_between_points(
                face_center1_obb2, face_center2_obb2, radius=radius, color=(0, 1, 0)
            ),
            self.create_cylinder_between_points(
                face_center2_obb1, face_center2_obb2, radius=radius, color=(1, 0, 0)
            ),
            self.create_cylinder_between_points(
                back_center, back_center_extent, radius=radius, color=(0, 0, 1)
            ),
        ]

        return cylinder_list
    

if __name__ == "__main__":
    my_gripper = MyGripperWithContact(
        hand_width=10,
        hand_thickness=2,
        hand_length=15,
        gripper_min_d=0,
        gripper_max_d=8,
        back_thickness=4,
        back_width=20,
        back_height=20,
        contact_thickness=1,
        gripper_in_offset=2,
        skeleton_radius=0.5,
    )

    # Create a point cloud object
    pcd = o3d.geometry.PointCloud()

    # Define a single point (x, y, z)
    point = np.array(
        [[0, 4, 0], [0, -4, 0], [0, 1, 0], [0, -1, 0]]
    )  # Example at (1, 2, 3)

    # Assign the point to the point cloud
    pcd.points = o3d.utility.Vector3dVector(point)

    # Optionally, set a color for the point (RGB format, values between 0 and 1)
    pcd.colors = o3d.utility.Vector3dVector([[0, 0, 1], [0, 0, 1]])  # Red color

    print(my_gripper.check_possible(pcd))
    print(my_gripper.count_contact_area(pcd))

    # my_gripper.close_to(d=2)

    # my_gripper.align_with_points(point[0], point[1], angle_around_y=180, close=True)
    # print(my_gripper.check_possible(pcd))
    world_axis = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=1, origin=[0, 0, 0]
    )

    pcd.points = o3d.utility.Vector3dVector(
        [np.mean(np.asarray(my_gripper.obb_contact1.get_box_points()[3:7]), axis=0)]
    )

    # my_gripper.to_home_state()
    o3d.visualization.draw_geometries([*my_gripper.geometries(), world_axis])

    o3d.visualization.draw_geometries([world_axis, *my_gripper.get_skeleton()])

    my_gripper.close_to(d=1)

    o3d.visualization.draw_geometries([*my_gripper.geometries(), world_axis])

    o3d.visualization.draw_geometries([world_axis, *my_gripper.get_skeleton()])