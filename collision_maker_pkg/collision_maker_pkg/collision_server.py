from collections import defaultdict

import numpy as np
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import PointCloud2, PointField
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header

from custom_srv_pkg.srv import PointCloudSend


class CollisionMakerService(Node):
    def __init__(self):
        super().__init__("Collision_Maker_service")
        self.publisher_ = self.create_publisher(
            PlanningScene, "/collision_object_scene_topic", 10
        )
        self.srv = self.create_service(
            PointCloudSend, "CollisionMaker", self.collision_maker_callback
        )


        self.VOXEL_SIZE = 0.01
        self.MAX_DISTANCE=0.7
        self.MIN_PCL_PER_CUBE=15

        self.get_logger().info("CollisionMaker service is ready.")

    def collision_maker_callback(self, request, response):
        self.get_logger().info(
            f"Received point cloud with width: {request.pointcloud.width} and height: {request.pointcloud.height}"
        )

        pcd = request.pointcloud
        transformed_pointcloud = self.transform_pointcloud(msg=pcd)
        self.publish_collision_objects_from_pcd(transformed_pointcloud)
        response.send_status = True

        return response

    def transform_pointcloud(self, msg: PointCloud2) -> PointCloud2:
        # Read points (x, y, z, rgb)
        points_msg = pc2.read_points_numpy(
            msg, field_names=("x", "y", "z", "rgb"), skip_nans=False
        )

        # Extract xyz and rgb
        transformed_xyz = points_msg[:, :3]
        transformed_xyz_no_nan = transformed_xyz[~np.isnan(transformed_xyz).any(axis=1)]

        # rgb = points_msg[:, -1]

        return transformed_xyz_no_nan

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

        if not filtered_voxels:
            self.get_logger().warn("No voxels passed filtering. Skipping collision creation.")
            return []

        # Group by (x, y), sort z, then merge contiguous z's
        xy_groups = defaultdict(list)
        for voxel in filtered_voxels:
            x, y, z = voxel
            xy_groups[(x, y)].append(z)

        merged_cubes = []

        for (x, y), z_list in xy_groups.items():
            z_list.sort()
            start_z = z_list[0]
            prev_z = z_list[0]

            for z in z_list[1:] + [None]:  # sentinel to trigger last group
                if z is None or z != prev_z + 1:
                    height = (prev_z - start_z + 1) * voxel_size
                    center_z = (start_z + prev_z + 1) / 2 * voxel_size
                    center_x = min_bound[0] + (x + 0.5) * voxel_size
                    center_y = min_bound[1] + (y + 0.5) * voxel_size
                    center_z += min_bound[2]

                    # if height == voxel_size:
                    #     continue
                    merged_cubes.append((center_x, center_y, center_z, height))
                    start_z = z
                prev_z = z

        return merged_cubes

    def publish_collision_objects_from_pcd(self, pcd):
        # collision_objects_data = self.process_pointcloud_to_boxes(pcd)

        # Create a PlanningScene message and mark it as a diff
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        collision_boxes = self.pointcloud_xyz_to_simple_collision(
            pcd,
            max_distance=self.MAX_DISTANCE,
            voxel_size=self.VOXEL_SIZE,
            min_pcl_per_cube=self.MIN_PCL_PER_CUBE,
        )

        # Aggregate collision objects
        for idx, (x, y, z, height) in enumerate(collision_boxes):
            collision_object = CollisionObject()
            collision_object.header = Header()
            collision_object.header.frame_id = "world"
            collision_object.id = f"box_{idx}"

            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [self.VOXEL_SIZE, self.VOXEL_SIZE, height]

            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = float(z)
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0

            collision_object.primitives.append(box)
            collision_object.primitive_poses.append(pose)
            collision_object.operation = CollisionObject.ADD

            planning_scene.world.collision_objects.append(collision_object)

        self.publisher_.publish(planning_scene)
        self.get_logger().info(
            f"Published {len(planning_scene.world.collision_objects)} merged CollisionObjects."
        )


def main():
    rclpy.init()
    node = CollisionMakerService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
