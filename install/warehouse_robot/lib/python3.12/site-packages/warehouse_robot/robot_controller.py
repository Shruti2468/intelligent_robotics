#!/usr/bin/env python3
"""
robot_controller.py  —  UPGRADED (drop-in replacement)
=======================================================
Keeps 100% of the original path-following / obstacle-avoidance logic
and adds the warehouse FSM on top:

  IDLE  →  NAVIGATE_TO_SHELF  →  WAIT_LOAD (3 s)
        →  NAVIGATE_TO_DROP   →  WAIT_UNLOAD (3 s)
        →  NAVIGATE_TO_DOCK   →  IDLE

How it talks to your existing path_planner
──────────────────────────────────────────
  Publishes   /goal_pose   (geometry_msgs/PoseStamped)
              ↑ your planner already listens here for new goals

  Subscribes  /planned_path  (nav_msgs/Path)
              ↑ planner sends the A* path back here

The controller sets a goal on /goal_pose, waits for the planner to
send /planned_path, follows it, and when it detects arrival it
advances the FSM.

Topics
──────
  Subscribed:
    /planned_path          nav_msgs/Path          (from path_planner)
    /odom                  nav_msgs/Odometry
    /scan                  sensor_msgs/LaserScan
    /task_assignment       std_msgs/String  JSON  (from task_manager)

  Published:
    /cmd_vel               geometry_msgs/Twist
    /goal_pose             geometry_msgs/PoseStamped  (to path_planner)
    /replan_request        geometry_msgs/PoseStamped  (obstacle replan)
    /robot_status          std_msgs/String  JSON
    /parcel_state          std_msgs/String  "LOADED:<id>" | "UNLOADED:<id>"
"""

import json
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String

# ── Tuning ────────────────────────────────────────────────────────────────────
OBSTACLE_STOP_DIST = 0.35   # hard stop only this close (was 0.45)
WP_TOLERANCE       = 0.30
LOOKAHEAD_DIST     = 1.2
MAX_RANGE          = 10.0

# ── Warehouse world coordinates ───────────────────────────────────────────────
WAREHOUSE_MAP = {
    "shelves": {
        # Row 1  y=6  → approach from south, face north (yaw=90)
        "shelf_1_1": {"x": -9.0, "y":  5.1, "yaw":  90.0, "parcel": "parcel_1_1"},
        "shelf_1_2": {"x": -3.0, "y":  5.1, "yaw":  90.0, "parcel": "parcel_1_2"},
        "shelf_1_3": {"x":  3.0, "y":  5.1, "yaw":  90.0, "parcel": "parcel_1_3"},
        "shelf_1_4": {"x":  9.0, "y":  5.1, "yaw":  90.0, "parcel": "parcel_1_4"},
        # Row 2  y=0  → approach from south, face north (yaw=90)
        "shelf_2_1": {"x": -9.0, "y": -0.9, "yaw":  90.0, "parcel": "parcel_2_1"},
        "shelf_2_2": {"x": -3.0, "y": -0.9, "yaw":  90.0, "parcel": "parcel_2_2"},
        "shelf_2_3": {"x":  3.0, "y": -0.9, "yaw":  90.0, "parcel": "parcel_2_3"},
        "shelf_2_4": {"x":  9.0, "y": -0.9, "yaw":  90.0, "parcel": "parcel_2_4"},
        # Row 3  y=-6  → approach from north, face south (yaw=-90)
        "shelf_3_1": {"x": -9.0, "y": -5.1, "yaw": -90.0, "parcel": "parcel_3_1"},
        "shelf_3_2": {"x": -3.0, "y": -5.1, "yaw": -90.0, "parcel": "parcel_3_2"},
        "shelf_3_3": {"x":  3.0, "y": -5.1, "yaw": -90.0, "parcel": "parcel_3_3"},
        "shelf_3_4": {"x":  9.0, "y": -5.1, "yaw": -90.0, "parcel": "parcel_3_4"},
    },
    # Stop just north of each platform (platform center is at y=-10, size 2m deep)
    "drop_point":      {"x":  10.0, "y": -9.0, "yaw": 0.0},
    "docking_station": {"x": -10.0, "y": -9.0, "yaw": 0.0},
    "load_wait_sec":   3.0,
    "unload_wait_sec": 3.0,
}
# ── FSM state names ───────────────────────────────────────────────────────────
class S:
    IDLE             = "IDLE"
    SEND_GOAL        = "SEND_GOAL"
    NAVIGATING       = "NAVIGATING"
    WAITING_AT_SHELF = "WAITING_AT_SHELF"
    LOADING          = "LOADING"
    WAITING_AT_DROP  = "WAITING_AT_DROP"
    UNLOADING        = "UNLOADING"
    DOCKING          = "DOCKING"


def _yaw_to_quat(yaw_deg: float):
    y = math.radians(yaw_deg)
    return (0.0, 0.0, math.sin(y / 2.0), math.cos(y / 2.0))


def _make_goal(x, y, yaw_deg, node) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = "map"
    ps.header.stamp = node.get_clock().now().to_msg()
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.position.z = 0.0
    qx, qy, qz, qw = _yaw_to_quat(yaw_deg)
    ps.pose.orientation.x = qx
    ps.pose.orientation.y = qy
    ps.pose.orientation.z = qz
    ps.pose.orientation.w = qw
    return ps


# ═════════════════════════════════════════════════════════════════════════════
class RobotController(Node):

    def __init__(self):
        super().__init__("robot_controller")

        self.declare_parameter("robot_id", "robot_1")
        self.robot_id: str = self.get_parameter("robot_id").value

        self.wmap = WAREHOUSE_MAP

        # ── Path-following state ─────────────────────────────────────────
        self.waypoints      = []
        self.wp_index       = 0
        self.robot_x        = 0.0
        self.robot_y        = 0.0
        self.robot_yaw      = 0.0
        self.obstacle_ahead = False
        self.slow_factor    = 1.0       # velocity scale from scan_callback
        self.blocked_ticks  = 0
        self.REPLAN_TICKS   = 25        # increased from 10 — less jittery replanning
        self.current_goal   = None

        # ── FSM state ────────────────────────────────────────────────────
        self.fsm_state     = S.IDLE
        self.active_task   = None
        self.parcel_loaded = False
        self._wait_ticks   = 0
        self._wait_target  = 0
        self._pending_goal = None

        # ── Subscribers ──────────────────────────────────────────────────
        self.create_subscription(Path,      "/planned_path",    self.path_callback,  10)
        self.create_subscription(Odometry,  "/odom",            self.odom_callback,  10)
        self.create_subscription(LaserScan, "/scan",            self.scan_callback,  10)
        self.create_subscription(String,    "/task_assignment", self.task_callback,  10)

        # ── Publishers ───────────────────────────────────────────────────
        self.cmd_pub    = self.create_publisher(Twist,       "/cmd_vel",        10)
        self.goal_pub   = self.create_publisher(PoseStamped, "/goal_pose",      10)
        self.replan_pub = self.create_publisher(PoseStamped, "/replan_request", 10)
        self.status_pub = self.create_publisher(String,      "/robot_status",   10)
        self.parcel_pub = self.create_publisher(String,      "/parcel_state",   10)

        # ── Timers ───────────────────────────────────────────────────────
        self.create_timer(0.1, self.control_loop)
        self.create_timer(1.0, self.publish_status)

        self.get_logger().info(
            f"[{self.robot_id}] Warehouse controller ready. "
            f"Shelves known: {list(self.wmap['shelves'].keys())}"
        )

    # ═══════════════════════════════════════════════════════════════════
    # Sensor callbacks
    # ═══════════════════════════════════════════════════════════════════

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def scan_callback(self, msg):
        n   = len(msg.ranges)
        arc = max(1, int(n * 25 / 360))
        mid = n // 2
        front = msg.ranges[mid - arc : mid + arc]
        valid = [r for r in front if msg.range_min < r < msg.range_max]
        if not valid:
            self.obstacle_ahead = False
            self.slow_factor    = 1.0
            return
        min_r = min(valid)
        # Hard stop only when very close; gradual slowdown from 1.2 m down to 0.35 m
        self.obstacle_ahead = min_r < OBSTACLE_STOP_DIST
        self.slow_factor    = max(0.15, (min_r - OBSTACLE_STOP_DIST) / (1.2 - OBSTACLE_STOP_DIST))

    def path_callback(self, msg):
        self.waypoints = [
            (p.pose.position.x, p.pose.position.y) for p in msg.poses
        ]
        self.wp_index      = 0
        self.blocked_ticks = 0
        if self.waypoints:
            self.current_goal = self.waypoints[-1]
        # Accept DOCKING state too so _navigate_to_dock doesn't deadlock
        if self.fsm_state in (S.SEND_GOAL, S.DOCKING):
            self.fsm_state = S.NAVIGATING
        self.get_logger().info(
            f"[{self.robot_id}] New path: {len(self.waypoints)} waypoints. "
            f"FSM → {self.fsm_state}"
        )

    # ═══════════════════════════════════════════════════════════════════
    # Task ingestion
    # ═══════════════════════════════════════════════════════════════════

    def task_callback(self, msg: String):
        try:
            task = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("Bad JSON on /task_assignment — ignored.")
            return

        if task.get("robot_id", self.robot_id) != self.robot_id:
            return

        if self.fsm_state != S.IDLE:
            self.get_logger().warn(
                f"[{self.robot_id}] Busy ({self.fsm_state}) — "
                f"rejecting task {task.get('task_id','?')}"
            )
            return

        shelf_id = task.get("shelf_id", "")
        if shelf_id not in self.wmap["shelves"]:
            self.get_logger().error(f"Unknown shelf '{shelf_id}' — task dropped.")
            return

        self.active_task = task
        self.get_logger().info(
            f"[{self.robot_id}] Task {task['task_id']}: "
            f"pick {task.get('parcel_id','?')} from {shelf_id}"
        )
        self._navigate_to_shelf(shelf_id)

    # ═══════════════════════════════════════════════════════════════════
    # FSM navigation steps
    # ═══════════════════════════════════════════════════════════════════

    def _send_nav_goal(self, x, y, yaw_deg, after_state: str):
        self._pending_goal = (x, y, yaw_deg, after_state)
        self.fsm_state     = S.SEND_GOAL
        self.goal_pub.publish(_make_goal(x, y, yaw_deg, self))
        self.get_logger().info(
            f"[{self.robot_id}] Goal sent ({x:.1f}, {y:.1f}) → after={after_state}"
        )

    def _navigate_to_shelf(self, shelf_id: str):
        s = self.wmap["shelves"][shelf_id]
        self._send_nav_goal(s["x"], s["y"], s["yaw"], S.WAITING_AT_SHELF)

    def _navigate_to_drop(self):
        d = self.wmap["drop_point"]
        self._send_nav_goal(d["x"], d["y"], d["yaw"], S.WAITING_AT_DROP)

    def _navigate_to_dock(self):
        d = self.wmap["docking_station"]
        self._send_nav_goal(d["x"], d["y"], d["yaw"], S.IDLE)
        self.fsm_state = S.DOCKING  # display label only; path_callback promotes to NAVIGATING

    # ═══════════════════════════════════════════════════════════════════
    # Path-following helpers
    # ═══════════════════════════════════════════════════════════════════

    def request_replan(self):
        if self.current_goal is None:
            return
        msg = PoseStamped()
        msg.header.frame_id    = "map"
        msg.header.stamp       = self.get_clock().now().to_msg()
        msg.pose.position.x    = self.current_goal[0]
        msg.pose.position.y    = self.current_goal[1]
        msg.pose.orientation.w = 1.0
        self.replan_pub.publish(msg)
        self.blocked_ticks = 0
        self.get_logger().info("Replanning around obstacle...")

    def get_carrot(self):
        while self.wp_index < len(self.waypoints) - 1:
            tx, ty = self.waypoints[self.wp_index]
            if math.hypot(tx - self.robot_x, ty - self.robot_y) < WP_TOLERANCE:
                self.wp_index += 1
            else:
                break

        if self.wp_index >= len(self.waypoints):
            return None

        acc = 0.0
        px, py = self.robot_x, self.robot_y
        for i in range(self.wp_index, len(self.waypoints)):
            wx, wy = self.waypoints[i]
            seg = math.hypot(wx - px, wy - py)
            if acc + seg >= LOOKAHEAD_DIST:
                ratio = (LOOKAHEAD_DIST - acc) / seg if seg > 0 else 0.0
                return (px + ratio * (wx - px), py + ratio * (wy - py))
            acc += seg
            px, py = wx, wy

        return self.waypoints[-1]

    def _drive_toward_carrot(self):
        """Pure path-following step — returns True if goal reached."""
        if not self.waypoints or self.wp_index >= len(self.waypoints):
            self.cmd_pub.publish(Twist())
            return False

        gx, gy = self.waypoints[-1]
        if math.hypot(gx - self.robot_x, gy - self.robot_y) < WP_TOLERANCE:
            self.cmd_pub.publish(Twist())
            self.waypoints = []
            self.wp_index  = 0
            return True

        carrot = self.get_carrot()
        if carrot is None:
            self.cmd_pub.publish(Twist())
            return False

        cx, cy = carrot
        dx = cx - self.robot_x
        dy = cy - self.robot_y
        dist_to_goal = math.hypot(gx - self.robot_x, gy - self.robot_y)

        desired_yaw = math.atan2(dy, dx)
        angle_err   = desired_yaw - self.robot_yaw
        angle_err   = (angle_err + math.pi) % (2.0 * math.pi) - math.pi

        twist = Twist()
        twist.angular.z = max(-1.5, min(1.5, 1.2 * angle_err))

        # slow_factor scales velocity down near obstacles (1.0 = full speed)
        if abs(angle_err) < 0.3:
            twist.linear.x = min(0.8,  0.9 * dist_to_goal) * self.slow_factor
        elif abs(angle_err) < 0.7:
            twist.linear.x = min(0.45, 0.6 * dist_to_goal) * self.slow_factor
        elif abs(angle_err) < 1.2:
            twist.linear.x = min(0.15, 0.3 * dist_to_goal) * self.slow_factor

        self.cmd_pub.publish(twist)
        return False

    # ═══════════════════════════════════════════════════════════════════
    # Main 10 Hz control loop
    # ═══════════════════════════════════════════════════════════════════

    def control_loop(self):
        state = self.fsm_state

        if state in (S.IDLE, S.SEND_GOAL):
            self.cmd_pub.publish(Twist())
            return

        if state == S.NAVIGATING:
            if self.obstacle_ahead:
                self.cmd_pub.publish(Twist())
                self.blocked_ticks += 1
                if self.blocked_ticks >= self.REPLAN_TICKS:
                    self.request_replan()
                return

            self.blocked_ticks = 0
            arrived = self._drive_toward_carrot()

            if arrived:
                dest_state = self._pending_goal[3] if self._pending_goal else S.IDLE
                self.get_logger().info(
                    f"[{self.robot_id}] Arrived. Transitioning -> {dest_state}"
                )
                self.fsm_state = dest_state

                if dest_state == S.WAITING_AT_SHELF:
                    self._start_wait(self.wmap["load_wait_sec"])
                elif dest_state == S.WAITING_AT_DROP:
                    self._start_wait(self.wmap["unload_wait_sec"])
                elif dest_state == S.IDLE:
                    self.get_logger().info(
                        f"[{self.robot_id}] Docked. Ready for next task."
                    )
                    self.active_task   = None
                    self._pending_goal = None
            return

        if state == S.WAITING_AT_SHELF:
            self._wait_ticks += 1
            self._log_wait("Loading parcel")
            if self._wait_ticks >= self._wait_target:
                self._do_load()
            return

        if state == S.WAITING_AT_DROP:
            self._wait_ticks += 1
            self._log_wait("Unloading parcel")
            if self._wait_ticks >= self._wait_target:
                self._do_unload()
            return

    # ═══════════════════════════════════════════════════════════════════
    # FSM action helpers
    # ═══════════════════════════════════════════════════════════════════

    def _start_wait(self, seconds: float):
        self._wait_ticks  = 0
        self._wait_target = int(seconds / 0.1)

    def _log_wait(self, label: str):
        remaining = (self._wait_target - self._wait_ticks) * 0.1
        if self._wait_ticks % 10 == 0:
            self.get_logger().info(
                f"[{self.robot_id}] {label}... {remaining:.0f}s remaining"
            )

    def _do_load(self):
        self.parcel_loaded = True
        parcel_id = (self.active_task or {}).get("parcel_id", "unknown")
        self.get_logger().info(f"[{self.robot_id}] Parcel '{parcel_id}' LOADED.")
        self.parcel_pub.publish(String(data=f"LOADED:{parcel_id}"))
        self._navigate_to_drop()

    def _do_unload(self):
        self.parcel_loaded = False
        parcel_id = (self.active_task or {}).get("parcel_id", "unknown")
        self.get_logger().info(f"[{self.robot_id}] Parcel '{parcel_id}' UNLOADED.")
        self.parcel_pub.publish(String(data=f"UNLOADED:{parcel_id}"))
        self._navigate_to_dock()

    # ═══════════════════════════════════════════════════════════════════
    # Status publisher
    # ═══════════════════════════════════════════════════════════════════

    def publish_status(self):
        status = {
            "robot_id":      self.robot_id,
            "state":         self.fsm_state,
            "parcel_loaded": self.parcel_loaded,
            "task_id":       (self.active_task or {}).get("task_id"),
            "shelf_id":      (self.active_task or {}).get("shelf_id"),
            "pos_x":         round(self.robot_x, 2),
            "pos_y":         round(self.robot_y, 2),
        }
        self.status_pub.publish(String(data=json.dumps(status)))


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()