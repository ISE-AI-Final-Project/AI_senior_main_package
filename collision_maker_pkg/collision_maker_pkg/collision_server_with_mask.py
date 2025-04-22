from collections import defaultdict

import numpy as np
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Image, PointCloud2, PointField
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header

from custom_srv_pkg.srv import PointCloudSend, PointCloudSendWithMask
from main_pkg.utils import image_utils


class CollisionMakerService(Node):
    def __init__(self):
        super().__init__("Collision_Maker_service")
        self.publisher_ = self.create_publisher(
            PlanningScene, "/collision_object_scene_topic", 10
        )
        self.srv = self.create_service(
            PointCloudSendWithMask, "CollisionMakerWithMask", self.collision_maker_callback
        )
        self.get_logger().info("CollisionMakerWithMask service is ready.")

    def collision_maker_callback(self, request, response):
        self.get_logger().info(
            f"Received point cloud with width: {request.pointcloud.width} and height: {request.pointcloud.height}"
        )

        pcd = request.pointcloud
        transformed_pointcloud = self.transform_pointcloud(msg=pcd)
        cropout_pointcloud = self.cropout_xyz_from_mask(transformed_pointcloud, mask_msg=request.mask)
        self.publish_collision_objects_from_pcd(cropout_pointcloud)
        response.send_status = True

        return response

    def transform_pointcloud(self, msg: PointCloud2) -> PointCloud2:
        # Read points (x, y, z, rgb)
        points_msg = pc2.read_points_numpy(
            msg, field_names=("x", "y", "z", "rgb"), skip_nans=True
        )

        # Extract xyz and rgb
        transformed_xyz = points_msg[:, :3]
        rgb = points_msg[:, -1]

        return transformed_xyz
    
    def cropout_xyz_from_mask(self, xyz: np.array, mask_msg: Image):

        # Convert mask to np array
        mask = CvBridge().imgmsg_to_cv2(mask_msg, desired_encoding="mono8")  # shape (H, W)


        # Apply mask (non-zero values)
        rows, cols = np.where(mask < 1)  # (rows, cols)
        # filtered_points = xyz[keep_indices]  # shape (N, 3)

        xyz_img_array = np.array(list(xyz)).reshape(mask.shape[0], mask.shape[1], 3)  # shape (H, W, 3)

        filtered_xyz = xyz_img_array[rows, cols, :]

        self.get_logger().info(str(filtered_xyz) + str(xyz))

        return xyz

    def pointcloud_xyz_to_simple_collision(
        self, points_xyz: np.array, voxel_size=0.1, max_distance=1, min_pcl_per_cube=5, base_link=np.array([0.4, 0.53, 0.8])
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
            dist = np.linalg.norm(pt - base_link)
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

        if distances.size == 0:
            self.get_logger().warn("No voxels passed filtering (empty distance list). Skipping collision creation.")
            return []
        
        min_dist = np.min(distances)
        max_dist = np.max(distances)
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

    def publish_collision_objects_from_pcd(self, pcd):
        # collision_objects_data = self.process_pointcloud_to_boxes(pcd)

        VOXEL_SIZE = 0.01

        # Create a PlanningScene message and mark it as a diff
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        collision_boxes_center = self.pointcloud_xyz_to_simple_collision(
            pcd,
            max_distance=0.75,
            voxel_size=VOXEL_SIZE,
            min_pcl_per_cube=3,
        )

        # Aggregate collision objects
        for idx, box_center in enumerate(collision_boxes_center):  # data
            collision_object = CollisionObject()
            collision_object.header = Header()
            collision_object.header.frame_id = "world"
            collision_object.id = f"box_{idx}"

            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = list([VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE])

            pose = Pose()
            pose.position.x = float(box_center[0])
            pose.position.y = float(box_center[1])
            pose.position.z = float(box_center[2])
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0

            collision_object.primitives.append(box)
            collision_object.primitive_poses.append(pose)
            collision_object.operation = CollisionObject.ADD

            planning_scene.world.collision_objects.append(collision_object)

        # Publish all collision objects as one message
        self.publisher_.publish(planning_scene)
        self.get_logger().info(
            f"Published {len(planning_scene.world.collision_objects)} CollisionObjects in one PlanningScene message"
        )


def main():
    rclpy.init()
    node = CollisionMakerService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
