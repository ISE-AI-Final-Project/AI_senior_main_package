#!/usr/bin/env python3
import asyncio
import os
from datetime import datetime
from threading import Thread

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from mpl_toolkits.mplot3d import Axes3D
from rclpy.node import Node

from custom_srv_pkg.srv import IKPassCount


def random_pose(
    x_range: tuple[float, float],
    y_range: tuple[float, float],
    z_range: tuple[float, float]
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
        super().__init__('ik_monte_carlo_counter')

        # 1) service client
        self.cli = self.create_client(IKPassCount, 'ik_pass_count')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for ik_pass_count…')

        # 2) MC parameters
        self.num_samples = 10000
        self.poses_per_sample = 1
        self.x_bounds = (-0.5, 0.5)
        self.y_bounds = (-0.5, 0.5)
        self.z_bounds = ( 0.0, 1.0)

        # 3) run after a short delay
        # self.create_timer(1.0, self._run_once)

        asyncio.run_coroutine_threadsafe(self._run_once(), self.loop)


    async def _run_once(self):
        pass_cnt = 0
        x_pass, y_pass, z_pass = [], [], []
        x_fail, y_fail, z_fail = [], [], []
        for _ in range(self.num_samples):
            pa = PoseArray()
            pa.header.frame_id = 'world'
            pa.poses = [
                random_pose(self.x_bounds, self.y_bounds, self.z_bounds)
                for __ in range(self.poses_per_sample)
            ]
            p = pa.poses[0]
            self.get_logger().info(f"random pose = {pa.poses[0]}")

            req = IKPassCount.Request()
            req.pose_array = pa

            future = self.cli.call_async(req)
            await future

            #stuck here
            # rclpy.spin_until_future_complete(self, future)
            res = future.result()

            if res and res.pass_count and res.pass_count[0] > 0:
                pass_cnt += 1
                x_pass.append(p.position.x)
                y_pass.append(p.position.y)
                z_pass.append(p.position.z)
            else:
                x_fail.append(p.position.x)
                y_fail.append(p.position.y)
                z_fail.append(p.position.z)
            
            # res.pass_count not working
            self.get_logger().info(f"result = {res.pass_count}")
            pose_status = "fail"
            if res.pass_count[0] != 0:
                pose_status = "pass"
                pass_cnt += 1
            self.get_logger().info(f"pose status = {pose_status}")

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(x_pass, y_pass, z_pass, label='pass')  # default color #1
        ax.scatter(x_fail, y_fail, z_fail, label='fail')  # default color #2
        ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
        ax.legend()
        plt.show()
        ratio = pass_cnt / float(self.num_samples)
        self.get_logger().info(
            f'Random IK MC: {pass_cnt}/{self.num_samples} full passes → {ratio:.2f}'
        )
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = MonteCarloRandomIK()
    rclpy.spin(node)
    # rclpy.shutdown() is already called in the timer

if __name__ == '__main__':
    main()