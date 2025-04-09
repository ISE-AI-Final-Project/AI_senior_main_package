import rclpy
from rclpy.node import Node

from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from scipy.spatial.transform import Rotation as R

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


    def transform_pointcloud(msg: PointCloud2) -> PointCloud2:
        # Read points (x, y, z, rgb)
        points_msg = pc2.read_points_numpy(
            msg, field_names=("x", "y", "z", "rgb"), skip_nans=True
        )

        # Extract xyz and rgb
        transformed_xyz = points_msg[:, :3]
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

    
    def publish_collision_objects_from_pcd(self, pcd):
        # collision_objects_data = self.process_pointcloud_to_boxes(pcd)

        # Create a PlanningScene message and mark it as a diff
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        collision_boxes_center = self.pointcloud_xyz_to_simple_collision(
            pcd,
            max_distance=1.5,
            voxel_size=0.05,
            min_pcl_per_cube=1,
        )

        # Aggregate collision objects
        for idx, box_center in enumerate(collision_boxes_center): #data 
            collision_object = CollisionObject()
            collision_object.header = Header()
            collision_object.header.frame_id = "panda_link0"
            collision_object.id = f"box_{idx}"

            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
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

            collision_object.primitives.append(box)
            collision_object.primitive_poses.append(pose)
            collision_object.operation = CollisionObject.ADD

            planning_scene.world.collision_objects.append(collision_object)

        # Publish all collision objects as one message
        self.publisher_.publish(planning_scene)
        self.get_logger().info(f'Published {len(planning_scene.world.collision_objects)} CollisionObjects in one PlanningScene message')

def main():
    rclpy.init()
    node = CollisionMakerService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


