# dwa_controller.py  — drop into warehouse_robot/warehouse_robot/
import math, numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from warehouse_robot.robot_controller import RobotController, WP_TOLERANCE

# ── DWA tuning ────────────────────────────────────────────────────────────────
V_MIN, V_MAX       = 0.0,  0.8      # linear vel range  (m/s)
W_MIN, W_MAX       = -1.5, 1.5      # angular vel range (rad/s)
V_SAMPLES          = 10             # how many linear velocities to try
W_SAMPLES          = 20             # how many angular velocities to try
SIM_TIME           = 1.5            # seconds to simulate each candidate
SIM_STEPS          = 15             # time steps per simulation
ROBOT_RADIUS       = 0.25           # metres — conservative clearance
SCORE_GOAL         = 1.0            # weight: progress toward goal
SCORE_CLEARANCE    = 0.5            # weight: distance from nearest obstacle
SCORE_VELOCITY     = 0.2            # weight: prefer faster forward motion


class DWARobotController(RobotController):

    def __init__(self):
        super().__init__()
        self._scan_ranges  = []
        self._scan_angles  = []

    def scan_callback(self, msg: LaserScan):
        angles = np.arange(len(msg.ranges)) * msg.angle_increment + msg.angle_min
        ranges = np.array(msg.ranges, dtype=np.float32)
        valid  = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
        self._scan_ranges = ranges[valid]
        self._scan_angles = angles[valid]
        # no obstacle_ahead flag — DWA handles avoidance itself

    def _drive_toward_carrot(self) -> bool:
        if not self.waypoints:
            self.cmd_pub.publish(Twist())
            return False

        gx, gy = self.waypoints[-1]
        dist_to_goal = math.hypot(gx - self.robot_x, gy - self.robot_y)

        if dist_to_goal < WP_TOLERANCE:
            self.cmd_pub.publish(Twist())
            self.waypoints = []
            self.wp_index  = 0
            return True

        carrot = self.get_carrot() or (gx, gy)
        best_score = -math.inf
        best_v, best_w = 0.0, 0.0

        for v in np.linspace(V_MIN, V_MAX, V_SAMPLES):
            for w in np.linspace(W_MIN, W_MAX, W_SAMPLES):
                score, safe = self._score_trajectory(v, w, carrot)
                if safe and score > best_score:
                    best_score = score
                    best_v, best_w = v, w

        twist = Twist()
        twist.linear.x  = best_v
        twist.angular.z = best_w
        self.cmd_pub.publish(twist)
        return False

    def _score_trajectory(self, v, w, carrot):
        """Simulate (v,w) for SIM_TIME, return (score, is_safe)."""
        x, y, yaw = self.robot_x, self.robot_y, self.robot_yaw
        dt = SIM_TIME / SIM_STEPS

        for _ in range(SIM_STEPS):
            x   += v * math.cos(yaw) * dt
            y   += v * math.sin(yaw) * dt
            yaw += w * dt
            # Collision check against lidar hits
            if len(self._scan_ranges) > 0:
                obs_x = self.robot_x + self._scan_ranges * np.cos(
                    self.robot_yaw + self._scan_angles)
                obs_y = self.robot_y + self._scan_ranges * np.sin(
                    self.robot_yaw + self._scan_angles)
                dists = np.hypot(obs_x - x, obs_y - y)
                if np.min(dists) < ROBOT_RADIUS:
                    return 0.0, False   # unsafe trajectory

        # Score end state
        goal_dist    = math.hypot(carrot[0] - x, carrot[1] - y)
        goal_score   = 1.0 / (goal_dist + 0.01)

        clearance = (np.min(np.hypot(
            self.robot_x + self._scan_ranges * np.cos(self.robot_yaw + self._scan_angles) - x,
            self.robot_y + self._scan_ranges * np.sin(self.robot_yaw + self._scan_angles) - y,
        )) if len(self._scan_ranges) > 0 else 1.0)

        score = (SCORE_GOAL      * goal_score +
                 SCORE_CLEARANCE * min(clearance, 2.0) +
                 SCORE_VELOCITY  * v)
        return score, True

    # control_loop: remove blocked_ticks / replan, keep rest of FSM
    def control_loop(self):
        from warehouse_robot.robot_controller import S
        state = self.fsm_state

        if state in (S.IDLE, S.SEND_GOAL):
            self.cmd_pub.publish(Twist())
            return

        if state == S.NAVIGATING:
            arrived = self._drive_toward_carrot()
            if arrived:
                dest = self._pending_goal[3] if self._pending_goal else S.IDLE
                self.fsm_state = dest
                if dest == S.WAITING_AT_SHELF:
                    self._start_wait(self.wmap["load_wait_sec"])
                elif dest == S.WAITING_AT_DROP:
                    self._start_wait(self.wmap["unload_wait_sec"])
                elif dest == S.IDLE:
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


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = DWARobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()