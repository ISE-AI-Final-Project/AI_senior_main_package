<<<<<<< Updated upstream
=======
from collections import defaultdict
import numpy as np
>>>>>>> Stashed changes
import rclpy
from rclpy.node import Node
<<<<<<< Updated upstream

=======
from sensor_msgs.msg import PointCloud2
>>>>>>> Stashed changes
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
<<<<<<< Updated upstream
from scipy.spatial.transform import Rotation as R

=======
>>>>>>> Stashed changes
from custom_srv_pkg.srv import PointCloudSend
from sensor_msgs.msg import PointCloud2,PointField
from moveit_msgs.msg import CollisionObject, PlanningScene

import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from collections import defaultdict

class CollisionMakerService(Node):
    def __init__(self):
        super().__init__('Collision_Maker_service')
        self.publisher_ = self.create_publisher(PlanningScene, '/collision_object_scene_topic', 10)
        self.srv = self.create_service(PointCloudSend, 'CollisionMaker', self.collision_maker_callback)
        self.get_logger().info("CollisionMaker service is ready.")

    def collision_maker_callback(self, request, response):
        self.get_logger().info(
            f"Received point cloud with width: {request.pointcloud.width} and height: {request.pointcloud.height}"
        )
  
        pcd = request.pointcloud  # Modify this based on actual request format
        transformed_pointcloud = self.transform_pointcloud(msg=pcd)
        self.publish_collision_objects_from_pcd(transformed_pointcloud)
        response.send_status = True

        return response

<<<<<<< Updated upstream

    def transform_pointcloud(msg: PointCloud2) -> PointCloud2:
        # Read points (x, y, z, rgb)
=======
    def transform_pointcloud(self, msg: PointCloud2) -> np.ndarray:
>>>>>>> Stashed changes
        points_msg = pc2.read_points_numpy(
            msg, field_names=("x", "y", "z", "rgb"), skip_nans=True
        )
        transformed_xyz = points_msg[:, :3]
<<<<<<< Updated upstream
        rgb = points_msg[:, -1] 


        return  transformed_xyz


    def pointcloud_xyz_to_simple_collision(
        points_xyz: np.array, voxel_size=0.1, max_distance=1, min_pcl_per_cube=5):
        """
        Convert pointcloud xyz to cube collision object

        output:
            cloud_out: PointCloud2
            transformed_xyz: np.array
        """

        # Get bounds and compute voxel indices
=======
        transformed_xyz_no_nan = transformed_xyz[~np.isnan(transformed_xyz).any(axis=1)]
        return transformed_xyz_no_nan

    def pointcloud_xyz_to_simple_collision(
        self,
        points_xyz,
        voxel_size,
        max_distance,
        min_pcl_per_cube,
        base_link=np.array([0.4, 0.53, 0.8]),
    ):
>>>>>>> Stashed changes
        min_bound = np.floor(points_xyz.min(axis=0) / voxel_size) * voxel_size
        voxel_indices = np.floor((points_xyz - min_bound) / voxel_size).astype(int)

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

        filtered_voxels = {
            k: voxel_distances[k]
            for k in voxel_counts
            if voxel_counts[k] >= min_pcl_per_cube
        }

<<<<<<< Updated upstream
        # Normalize distances for color mapping
        distances = np.array(list(filtered_voxels.values()))
        min_dist = distances.min()
        max_dist = distances.max()
        norm_dists = (distances - min_dist) / (max_dist - min_dist + 1e-6)
=======
        if not filtered_voxels:
            self.get_logger().warn("No voxels passed filtering. Skipping collision creation.")
            return []
>>>>>>> Stashed changes

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
                    merged_cubes.append((center_x, center_y, center_z, height))
                    start_z = z
                prev_z = z

<<<<<<< Updated upstream
    
    def publish_collision_objects_from_pcd(self, pcd):
        # collision_objects_data = self.process_pointcloud_to_boxes(pcd)

        # Create a PlanningScene message and mark it as a diff
=======
        return merged_cubes

    def publish_collision_objects_from_pcd(self, pcd: np.ndarray):
        VOXEL_SIZE = 0.04
>>>>>>> Stashed changes
        planning_scene = PlanningScene()
        planning_scene.is_diff = True

        collision_boxes = self.pointcloud_xyz_to_simple_collision(
            pcd,
            max_distance=1.5,
            voxel_size=0.05,
            min_pcl_per_cube=1,
        )

<<<<<<< Updated upstream
        # Aggregate collision objects
        for idx, box_center in enumerate(collision_boxes_center): #data 
=======
        for idx, (x, y, z, height) in enumerate(collision_boxes):
>>>>>>> Stashed changes
            collision_object = CollisionObject()
            collision_object.header = Header()
            collision_object.header.frame_id = "panda_link0"
            collision_object.id = f"box_{idx}"

            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
<<<<<<< Updated upstream
            box.dimensions = list(box_center["size"])

            rot = R.from_matrix(box_center["rotation"])
            quat = rot.as_quat()

            pose = Pose()
            pose.position.x = float(box_center[0])
            pose.position.y = float(box_center[1])
            pose.position.z = float(box_center[2])
            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]
=======
            box.dimensions = [VOXEL_SIZE, VOXEL_SIZE, height]

            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = float(z)
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
>>>>>>> Stashed changes

            collision_object.primitives.append(box)
            collision_object.primitive_poses.append(pose)
            collision_object.operation = CollisionObject.ADD

            planning_scene.world.collision_objects.append(collision_object)

        self.publisher_.publish(planning_scene)
<<<<<<< Updated upstream
        self.get_logger().info(f'Published {len(planning_scene.world.collision_objects)} CollisionObjects in one PlanningScene message')
=======
        self.get_logger().info(
            f"Published {len(planning_scene.world.collision_objects)} merged CollisionObjects."
        )

>>>>>>> Stashed changes

def main():
    rclpy.init()
    node = CollisionMakerService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


