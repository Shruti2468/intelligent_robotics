#!/usr/bin/env python3
"""
warehouse_nav_env.py
--------------------
Gymnasium environment for training the SAC local navigation policy.
Wraps Gazebo via ROS 2 — no changes to the world SDF needed.

Install deps first:
  pip install gymnasium stable-baselines3[extra] torch --break-system-packages
"""

import math
import time
import random
import numpy as np
import gymnasium as gym
from gymnasium import spaces

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty

# ── constants matching your world ────────────────────────────────────────────
LIDAR_RAYS      = 360
MAX_RANGE       = 10.0          # normalisation divisor (metres)
MAX_LIN_VEL     = 0.8
MAX_ANG_VEL     = 1.5
ARRIVE_DIST     = 0.30          # matches WP_TOLERANCE in controller
COLLISION_DIST  = 0.35          # slightly less than OBSTACLE_STOP_DIST
MAX_STEPS       = 600           # ~60 s at 10 Hz

# Random start poses scattered in the aisle regions of your world
START_POSES = [
    ( 0.0,  0.0, 0.0), (-6.0,  3.0, 0.0), ( 6.0,  3.0, 0.0),
    (-6.0, -3.0, 0.0), ( 6.0, -3.0, 0.0), ( 0.0,  9.0, 0.0),
]

# Shelf approach positions from WAREHOUSE_MAP in robot_controller.py
GOAL_POSES = [
    (-9.0,  4.8), (-3.0,  4.8), ( 3.0,  4.8), ( 9.0,  4.8),
    (-9.0, -1.2), (-3.0, -1.2), ( 3.0, -1.2), ( 9.0, -1.2),
    (-9.0, -4.8), (-3.0, -4.8), ( 3.0, -4.8), ( 9.0, -4.8),
    (10.0,-10.0), (-10.0,-10.0),
]


class _ROSBridge(Node):
    """Minimal ROS 2 node — just publishes cmd_vel and reads sensors."""

    def __init__(self):
        super().__init__("rl_env_bridge")
        self.scan   = np.full(LIDAR_RAYS, MAX_RANGE)
        self.x      = 0.0
        self.y      = 0.0
        self.yaw    = 0.0
        self.lin_v  = 0.0
        self.ang_v  = 0.0

        self.cmd_pub   = self.create_publisher(Twist, "/cmd_vel", 1)
        self.pose_pub  = self.create_publisher(PoseStamped, "/initialpose", 1)

        self.create_subscription(LaserScan, "/scan", self._scan_cb,  1)
        self.create_subscription(Odometry,  "/odom", self._odom_cb,  1)

        self._reset_cli = self.create_client(Empty, "/reset_simulation")

    def _scan_cb(self, msg):
        r = np.array(msg.ranges, dtype=np.float32)
        r = np.where(np.isfinite(r), r, MAX_RANGE)
        self.scan = np.clip(r, 0.0, MAX_RANGE)

    def _odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.lin_v = msg.twist.twist.linear.x
        self.ang_v = msg.twist.twist.angular.z

    def publish_cmd(self, lin, ang):
        t = Twist()
        t.linear.x  = float(np.clip(lin, 0.0, MAX_LIN_VEL))
        t.angular.z = float(np.clip(ang, -MAX_ANG_VEL, MAX_ANG_VEL))
        self.cmd_pub.publish(t)

    def stop(self):
        self.cmd_pub.publish(Twist())

    def reset_sim(self):
        if self._reset_cli.wait_for_service(timeout_sec=2.0):
            self._reset_cli.call_async(Empty.Request())
            time.sleep(0.5)

    def spin_once(self):
        rclpy.spin_once(self, timeout_sec=0.0)


class WarehouseNavEnv(gym.Env):
    """
    Gymnasium env for SAC local controller training.

    Observation  (363,):
        [0:360]   normalised lidar (0=wall touching, 1=max range)
        [360]     carrot dx in robot frame  / MAX_RANGE
        [361]     carrot dy in robot frame  / MAX_RANGE
        [362]     distance to carrot        / MAX_RANGE

    Action  (2,):
        [0]  linear velocity   in [0, MAX_LIN_VEL]
        [1]  angular velocity  in [-MAX_ANG_VEL, MAX_ANG_VEL]
    """

    metadata = {"render_modes": []}

    def __init__(self):
        super().__init__()

        if not rclpy.ok():
            rclpy.init()
        self._ros = _ROSBridge()

        obs_dim = LIDAR_RAYS + 3
        self.observation_space = spaces.Box(
            low=-1.0, high=1.0, shape=(obs_dim,), dtype=np.float32)
        self.action_space = spaces.Box(
            low=np.array([0.0, -1.0], dtype=np.float32),
            high=np.array([1.0,  1.0], dtype=np.float32))

        self._goal        = (0.0, 0.0)
        self._step_count  = 0
        self._prev_dist   = 0.0
        self._prev_lin_v  = 0.0

    # ── gym API ──────────────────────────────────────────────────────────────

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self._ros.stop()
        self._ros.reset_sim()

        # Spin until we have fresh sensor data
        for _ in range(20):
            self._ros.spin_once()
            time.sleep(0.05)

        self._goal       = random.choice(GOAL_POSES)
        self._step_count = 0
        self._prev_dist  = self._dist_to_goal()
        self._prev_lin_v = 0.0

        return self._obs(), {}

    def step(self, action):
        lin = float(action[0]) * MAX_LIN_VEL
        ang = float(action[1]) * MAX_ANG_VEL
        self._ros.publish_cmd(lin, ang)

        # Step at ~10 Hz
        time.sleep(0.1)
        self._ros.spin_once()

        obs     = self._obs()
        reward  = self._reward(lin, ang)
        dist    = self._dist_to_goal()
        collide = float(np.min(self._ros.scan)) < COLLISION_DIST

        terminated = dist < ARRIVE_DIST or collide
        truncated  = self._step_count >= MAX_STEPS

        self._step_count += 1
        self._prev_dist   = dist
        self._prev_lin_v  = lin

        info = {"dist": dist, "collision": collide, "arrived": dist < ARRIVE_DIST}
        return obs, reward, terminated, truncated, info

    def close(self):
        self._ros.stop()
        self._ros.destroy_node()

    # ── internals ────────────────────────────────────────────────────────────

    def _dist_to_goal(self):
        return math.hypot(
            self._goal[0] - self._ros.x,
            self._goal[1] - self._ros.y)

    def _obs(self):
        # Carrot = goal expressed in robot frame
        gx, gy  = self._goal
        dx      = gx - self._ros.x
        dy      = gy - self._ros.y
        c       = math.cos(-self._ros.yaw)
        s       = math.sin(-self._ros.yaw)
        cdx     = dx * c - dy * s
        cdy     = dx * s + dy * c
        dist    = math.hypot(dx, dy)

        lidar_norm = self._ros.scan / MAX_RANGE   # [0, 1]

        return np.concatenate([
            lidar_norm.astype(np.float32),
            np.array([cdx / MAX_RANGE,
                      cdy / MAX_RANGE,
                      dist / MAX_RANGE], dtype=np.float32),
        ])

    def _reward(self, lin_v, ang_v):
        dist    = self._dist_to_goal()
        min_r   = float(np.min(self._ros.scan))
        collide = min_r < COLLISION_DIST

        if collide:
            return -20.0

        if dist < ARRIVE_DIST:
            return +20.0

        r = 0.0
        r += 1.5  * (self._prev_dist - dist)          # progress toward goal
        r -= 0.6  * abs(ang_v)                         # penalise spinning
        r -= 1.5  * max(0.0, (1.0 / max(min_r, 0.1)) - 1.0)  # soft wall repulsion
        r -= 0.2  * abs(lin_v - self._prev_lin_v)      # smoothness
        r -= 0.01                                       # step cost (stay efficient)
        return float(r)