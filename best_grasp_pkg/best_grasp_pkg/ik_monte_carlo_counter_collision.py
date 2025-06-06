#!/usr/bin/env python3
import asyncio
import itertools
import math
import os
from datetime import datetime
from threading import Thread

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from matplotlib.patches import Wedge
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from mpl_toolkits.mplot3d import Axes3D
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2

from custom_srv_pkg.srv import IKPassCount, JointStateCollisionBool
from main_pkg.utils import utils


def random_pose(
    x_range: tuple[float, float],
    y_range: tuple[float, float],
    z_range: tuple[float, float],
) -> Pose:
    """Generate a random Pose (position in bounds, random orientation)."""
    p = Pose()
    p.position.x = np.random.uniform(*x_range)
    p.position.y = np.random.uniform(*y_range)
    p.position.z = np.random.uniform(*z_range)
    q = np.random.normal(size=4)
    q /= np.linalg.norm(q)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
    return p


class MonteCarloRandomIK(Node):
    def __init__(self):
        self.loop = asyncio.new_event_loop()
        thread = Thread(target=self.loop.run_forever)
        thread.daemon = True
        thread.start()
        super().__init__("ik_monte_carlo_counter")

        # 1) service client
        self.cli = self.create_client(IKPassCount, "ik_pass_count")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for ik_pass_count…")

        self.client_joint_state_collision_bool = self.create_client(
            JointStateCollisionBool, "joint_state_collision_check_bool"
        )

        self.client_get_planning_scene = self.create_client(
            GetPlanningScene, "/get_planning_scene"
        )

        # 2) MC parameters
        self.num_samples = 1
        self.poses_per_sample = 10000
        self.x_bounds = (0, 0)
        self.y_bounds = (0, 0.8)
        self.z_bounds = (-0.5, 1.0)

        # 3) run after a short delay
        # self.create_timer(1.0, self._run_once)

        asyncio.run_coroutine_threadsafe(self._run_once(), self.loop)

        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
            "robotiq_hande_left_finger_joint",
            "robotiq_hande_right_finger_joint",
        ]

        # UR3e standard DH parameters (a, alpha, d, theta_offset)
        self.dh_params = [
            (0.0, np.pi / 2, 0.15185, 0.0),  # Base to Shoulder (shoulder_pan_joint) 0
            (-0.24355, 0.0, 0.0, 0.0),  # Shoulder to Elbow (shoulder_lift_joint) 1
            (-0.2132, 0.0, 0.0, 0.0),  # Elbow to Wrist1 (elbow_joint) 2
            (0.0, np.pi / 2, 0.13105, 0.0),  # Wrist1 to Wrist2 (wrist_1_joint) 3
            (0.0, -np.pi / 2, 0.08535, 0.0),  # Wrist2 to Wrist3 (wrist_2_joint) 4
            (0.0, 0.0, 0.0921, 0.0),  # Wrist3 to tool0 (wrist_3_joint) 5
            (0.0, 0.0, 0.15695, -np.pi / 2),  # tool0 to end effector!!! 6
        ]

        # radius‐binning parameters
        self.z0 = 0.15185       # center height for radius computation
        self.bin_width = 0.05    # radius increment per bin (m)


    def dh_matrix(self, a, alpha, d, theta):
        """Build the individual DH transform matrix."""
        return np.array(
            [
                [
                    np.cos(theta),
                    -np.sin(theta) * np.cos(alpha),
                    np.sin(theta) * np.sin(alpha),
                    a * np.cos(theta),
                ],
                [
                    np.sin(theta),
                    np.cos(theta) * np.cos(alpha),
                    -np.cos(theta) * np.sin(alpha),
                    a * np.sin(theta),
                ],
                [0, np.sin(alpha), np.cos(alpha), d],
                [0, 0, 0, 1],
            ]
        )

    def dh_matrix_joint(self, joint=0, joint_angle=0):
        a, alpha, d, theta_offset = self.dh_params[joint]
        return self.dh_matrix(a, alpha, d, joint_angle + theta_offset)

    def ik(self, pose_wrt_base: PoseStamped, combination):
        try:

            # Note T_a_b = Frame a wrt to b, T_a = Frame a wrt 0
            T_e = utils.pose_to_ht(pose_wrt_base)

            # Step1: Find T_6, O_5 (don't care about orientation)
            T_e_6 = self.dh_matrix_joint(6)

            T_6 = T_e @ utils.inverse_ht(T_e_6)

            T_6_5_zero_joint = self.dh_matrix_joint(5)
            O_5 = T_6 @ utils.inverse_ht(T_6_5_zero_joint)

            R = T_e[:3, :3]

            # Step2: Find theta0
            x5, y5, z5 = O_5[0, 3], O_5[1, 3], O_5[2, 3]
            p5 = np.array([x5, y5, z5])
            D3 = np.abs(self.dh_params[3][2])

            cos0 = D3 / math.sqrt(x5**2 + y5**2)
            sin0 = math.sqrt(1 - cos0**2)
            theta0 = math.atan2(y5, x5) - math.atan2(combination[0] * sin0, cos0)

            # Step3: Find p4
            D4 = np.abs(self.dh_params[4][2])
            pe = np.array([T_e[0, 3], T_e[1, 3], T_e[2, 3]])
            n_p5 = (pe - p5) / np.linalg.norm(pe - p5)

            n_p1_prime = np.array([math.cos(theta0), math.sin(theta0), 0])

            n_line = np.cross(n_p5, n_p1_prime) / np.linalg.norm(
                np.cross(n_p5, n_p1_prime)
            )

            p4 = p5 + combination[1] * D4 * n_line

            # Step4: Find theta4
            n_x4 = n_p1_prime
            n_z4 = (p5 - p4) / np.linalg.norm(p5 - p4)
            n_y4 = np.cross(n_z4, n_p1_prime) / np.linalg.norm(
                np.cross(n_z4, n_p1_prime)
            )

            theta4 = math.atan2(np.dot(n_y4, (pe - p5)), np.dot(n_x4, (pe - p5)))

            # Step5: Find theta2
            A1 = np.abs(self.dh_params[1][0])
            A2 = np.abs(self.dh_params[2][0])
            D0 = np.abs(self.dh_params[0][2])

            p1 = np.array([0, 0, D0])
            p1_prime = p1 + D3 * n_p1_prime

            # print("p4: ", p4)

            cos2_180 = (A1**2 + A2**2 - np.linalg.norm(p1_prime - p4) ** 2) / (
                2 * A1 * A2
            )
            sin2_180 = combination[2] * math.sqrt(1 - cos2_180**2)

            theta2 = np.pi - math.atan2(sin2_180, cos2_180)

            n_p1_prime_y = np.array([0, 0, 1])

            angle0_to_4 = math.atan2(
                np.dot((p4 - p1_prime), n_p1_prime_y),
                np.dot(
                    (p4 - p1_prime),
                    np.cross(n_p1_prime_y, n_p1_prime)
                    / np.linalg.norm(np.cross(n_p1_prime_y, n_p1_prime)),
                ),
            )

            theta1 = angle0_to_4 - math.atan2(
                A2 * math.sin(theta2), A1 + A2 * math.cos(theta2)
            )

            p2_prime = (
                A1
                * math.cos(theta1)
                * np.cross(n_p1_prime_y, n_p1_prime)
                / np.linalg.norm(np.cross(n_p1_prime_y, n_p1_prime))
                + A1 * math.sin(theta1) * n_p1_prime_y
                + p1_prime
            )

            n_y3 = (p2_prime - p4) / np.linalg.norm(p2_prime - p4)
            n_x3 = np.cross(n_y3, n_p1_prime) / np.linalg.norm(
                np.cross(n_y3, n_p1_prime)
            )
            theta3 = math.atan2(np.dot(n_y3, p5 - p4), np.dot(n_x3, p5 - p4))

            # Finally: Find theta5
            joint_list = [
                theta0 + np.pi / 2,
                theta1 + np.pi,
                theta2,
                theta3,
                theta4,
                0.0,
                0.0,
                0.0,
            ]
            T_ik = np.eye(4)
            for i in range(7):
                a, alpha, d, theta_offset = self.dh_params[i]
                theta = joint_list[i] + theta_offset
                T_i = self.dh_matrix(a, alpha, d, theta)
                T_ik = T_ik @ T_i

            R_ik = T_ik[:3, :3]

            nx_ik = R_ik[:, 0] / np.linalg.norm(R_ik[:, 0])
            ny_ik = R_ik[:, 1] / np.linalg.norm(R_ik[:, 1])
            nx_e = R[:, 0] / np.linalg.norm(R[:, 0])
            # ny_e = R[:, 1]

            # Calculate theta5
            theta5 = np.arctan2(np.dot(ny_ik, nx_e), np.dot(nx_ik, nx_e))

            return [
                theta0 + np.pi / 2,
                theta1 + np.pi,
                theta2,
                theta3,
                theta4,
                theta5,
                0.025,  # robotiq_hande_left_finger_joint
                0.025,  # robotiq_hande_right_finger_joint
            ]

        except Exception as e:
            # self.get_logger().info(f"Combination: {combination} Fail due to {e}")
            return [
                0.0,
                0.0,
                np.pi,  # Collide Joint State
                0.0,
                0.0,
                0.0,
                0.025,
                0.025,
            ]

    async def call_service(self, client, request):
        try:
            future = client.call_async(request)
            await future
            return future.result()
        except Exception as e:
            self.elog(f"Service call failed: {e}")
            return None

    async def _run_once(self):

        # Step 1: Get Planning Scene
        planning_scene_request = GetPlanningScene.Request()
        planning_scene_request.components.components = (
            PlanningSceneComponents.SCENE_SETTINGS
            | PlanningSceneComponents.ROBOT_STATE
            | PlanningSceneComponents.WORLD_OBJECT_NAMES
            | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        )
        planning_scene_response = await self.call_service(
            self.client_get_planning_scene, planning_scene_request
        )
        if not planning_scene_response:
            return
        data_planning_scene = planning_scene_response.scene
        self.get_logger().info("Planning scene retrieved successfully.")

        # pass_cnt = 0
        # x_pass, y_pass, z_pass = [], [], []
        # x_fail, y_fail, z_fail = [], [], []
        # for _ in range(self.num_samples):
        pa = PoseArray()
        pa.header.frame_id = "base"
        pa.poses = [
            random_pose(self.x_bounds, self.y_bounds, self.z_bounds)
            for __ in range(self.poses_per_sample)
        ]
        p = pa.poses[0]
        # self.get_logger().info(f"random pose = {pa.poses[0]}")

        collision_request = JointStateCollisionBool.Request()
        collision_request.planning_scene = data_planning_scene

        # response_list = []

        for i, pose_wrt_base in enumerate(pa.poses):
            self.get_logger().info(f"pose {i}")
            # self.get_logger().info(f"pose = {pose_wrt_base}")

            total_pass = 0
            combinations = list(itertools.product([1, -1], repeat=3))
            for combination in combinations:
                # print(f"Combination: {combination}")

                # self.get_logger().info(f"Combination: {combination}")

                joint_state_by_ik = self.ik(
                    pose_wrt_base=pose_wrt_base, combination=combination
                )

                msg = JointState()
                msg.name = self.joint_names
                msg.position = joint_state_by_ik

                # self.get_logger().info(f"{msg}")

                collision_request.joint_state.append(msg)

            # response_list.append(total_pass)

        # response.pass_count = response_list

        collision_response = await self.call_service(
            self.client_joint_state_collision_bool, collision_request
        )
        if not collision_response:
            self.get_logger().warn("Error in Collision Check.")
            return

        self.get_logger().info(str(collision_response.pass_list))

        pass_list_array = np.array(collision_response.pass_list)

        pass_list_array = pass_list_array.reshape((-1, 8))

        self.get_logger().info(
            str(pass_list_array.shape) + str(np.sum(pass_list_array, axis=1))
        )

        # pass_list_array = pass_list_array.reshape((-1, 8))
        # pass_counts[i] is how many of the 8 IK combinations for pose i passed
        pass_counts = np.sum(pass_list_array, axis=1)

        # ––– visualize pass/fail in Y–Z space –––
        # 1) pull out the Y and Z coordinates of each sampled pose
        y_coords = np.array([pose.position.y for pose in pa.poses])
        z_coords = np.array([pose.position.z for pose in pa.poses])

        # 2) mask them by pass_counts
        # passed_mask = pass_counts > 0    # any valid IK → “pass”
        # failed_mask = pass_counts == 0    # zero valid IK → “fail”

        # # 3) scatter‐plot
        # plt.figure()
        # plt.scatter(y_coords[failed_mask], z_coords[failed_mask],
        #             marker='x', label='Failed poses')
        # plt.scatter(y_coords[passed_mask], z_coords[passed_mask],
        #             marker='o', label='Passed poses')
        # plt.xlabel('Y (m)')
        # plt.ylabel('Z (m)')
        # plt.title('IK Collision‐Check: Passed vs Failed (Y–Z plane)')
        # plt.legend()
        # plt.grid(True)
        # plt.show()
        # #–––––––––––––––––––––––––––––––––––––––

        # after computing pass_counts and extracting y_coords, z_coords:

        # mask out zeros
        valid_mask = (pass_counts >= 1) & (pass_counts <= 8)

        # normalize counts to [0,1] for the colormap
        #   1 → 0.0 (red), 8 → 1.0 (green)
        norm_counts = (pass_counts[valid_mask] - 1) / (8 - 1)

        # # choose a red→green gradient
        # cmap = plt.get_cmap("RdYlGn")

        # plt.figure()
        # sc = plt.scatter(
        #     y_coords[valid_mask],
        #     z_coords[valid_mask],
        #     c=norm_counts,
        #     cmap=cmap,
        #     marker="o",
        #     s=10,
        #     # edgecolors="k",
        # )
        # plt.xlabel("Radial Distance (m)")
        # plt.ylabel("Z (m)")

        # plt.ylim((-1, 1))
        # plt.xlim(0, 1)
        # plt.title("IK Collision Free Solution Count given a Pose")
        # plt.grid(True)
        # # add a colorbar with ticks labeled 1…8
        # cbar = plt.colorbar(sc, ticks=[i / 7 for i in range(8)])
        # cbar.set_ticklabels([str(i) for i in range(1, 9)])
        # cbar.set_label("Number of IK solutions passed")
        # plt.show()


        # 7) Compute radii around (0, z0)
        radii = np.sqrt((y_coords - 0.0)**2 + (z_coords - self.z0)**2)

        # 8) Bin and average pass counts
        bins = np.arange(0, radii.max() + self.bin_width, self.bin_width)
        centers = bins[:-1] + self.bin_width/2
        inds = np.digitize(radii, bins) - 1
        avg_pass = []
        for i in range(len(centers)):
            vals = pass_counts[inds == i]
            avg_pass.append(vals.mean() if vals.size > 0 else np.nan)

        # 9) Plot bar graph
        plt.figure()
        plt.bar(centers, avg_pass, width=self.bin_width)
        plt.xlabel(f"Radius around (0, {self.z0}) [m]")
        plt.ylabel("Average IK pass count")
        plt.title("Average IK Pass Count vs. Radius")
        plt.grid(True)
        plt.show()


        fig, ax = plt.subplots()
        # normalize avg_pass for colormap
        norm = plt.Normalize(vmin=np.nanmin(avg_pass), vmax=np.nanmax(avg_pass))
        cmap = plt.get_cmap('RdYlGn')
        # draw each annular bin as a filled wedge (full circle)
        for i in range(len(centers)):
            r_inner = bins[i]
            r_outer = bins[i+1]
            # skip empty bins
            if np.isnan(avg_pass[i]):
                continue
            color = cmap(norm(avg_pass[i]))
            ring = Wedge(
                (0, self.z0), r_outer,
                theta1=0, theta2=360,
                width=(r_outer - r_inner),
                facecolor=color,
                edgecolor='none'
            )
            ax.add_patch(ring)
        # formatting
        # ax.set_aspect('equal', 'box')
        ax.set_xlim(0, 0.8)
        ax.set_ylim(0, 1.0)
        ax.set_xlabel('Radial Distance (m)')
        ax.set_ylabel('Z (m)')
        ax.set_title('Average IK pass count by Radial Distance')
        # colorbar
        sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
        sm.set_array([])
        plt.colorbar(sm, label='Avg IK pass count', ax=ax)
        plt.grid(True)
        plt.show()



def main(args=None):
    rclpy.init(args=args)
    node = MonteCarloRandomIK()
    rclpy.spin(node)
    # rclpy.shutdown() is already called in the timer


if __name__ == "__main__":
    main()
