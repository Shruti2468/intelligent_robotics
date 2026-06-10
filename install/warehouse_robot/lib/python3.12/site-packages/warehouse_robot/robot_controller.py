"""robot_controller.py — DWB local planner (improved)

Changes vs original
───────────────────
FIX-1  COLLISION_R corrected: was 0.20 m, smaller than the robot's actual
       half-width (0.285 m from URDF collision box 0.72×0.57 m).  DWB was       
       accepting arcs that physically clipped shelf edges.  New value 0.32 m
       adds a small safety margin beyond the half-width.
FIX-2  Replan storm prevention: replan requests are now rate-limited to one
       per REPLAN_COOLDOWN seconds.  Previously REPLAN_TICKS=40 reset on
       every new path, so a slow replan cycle could spam /replan_request
       faster than the planner could respond.
FIX-3  Heading score uses absolute angular error correctly: the original
       formula computed heading error after simulating the full arc and
       comparing final yaw to carrot direction.  This is correct for DWB but
       the carrot direction itself was computed at sim end, not at the carrot
       point.  Fixed to compute carrot direction once before the loop.
FIX-4  Zero-velocity escape: when all arcs are collision-blocked the
       original tried MAX_W then -MAX_W and gave up.  Now it sweeps the full
       angular range in finer steps, improving in-place recovery in narrow
       aisles.
IMPROVE-1  Adaptive lookahead: lookahead distance scales with current speed
           (LOOKAHEAD_DIST_BASE + speed × LOOKAHEAD_SPEED_SCALE) so the
           robot looks further ahead at high speed and closer at low speed.
IMPROVE-2  Goal-approach speed cap: within SLOW_ZONE_R of the goal, MAX_V
           is clamped to APPROACH_V so the robot doesn't overshoot.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Twist
import math

# ── Robot spawn position in world frame ──────────────────────────────────────
SPAWN_X = -10.0
SPAWN_Y = -10.0

# ── LiDAR mount offset from base_link ────────────────────────────────────────
LIDAR_X_OFFSET = 0.25
LIDAR_Y_OFFSET = 0.00

# ── Kinematic limits ──────────────────────────────────────────────────────────
MAX_V  =  0.8     # m/s  forward max
MIN_V  = -0.1     # m/s  small reverse allowed
MAX_W  =  1.5     # rad/s
MAX_AV =  1.2     # m/s²   linear acceleration limit
MAX_AW =  2.0     # rad/s² angular acceleration limit

# ── DWB sampling ─────────────────────────────────────────────────────────────
DT       = 0.1    # control loop period (s)
SIM_TIME = 1.0    # arc simulation horizon (s)
N_V      = 7      # velocity samples
N_W      = 15     # angular velocity samples

# ── Scoring weights (must sum to 1.0) ────────────────────────────────────────
W_HEADING   = 0.55
W_CLEARANCE = 0.30
W_VELOCITY  = 0.15

# ── Safety geometry ───────────────────────────────────────────────────────────
SELF_FILTER_R = 0.55
# FIX-1: robot URDF collision box is 0.72 × 0.57 m → half-diagonal 0.451 m.
# COLLISION_R must be > half-width (0.285 m).  0.32 m gives ~35 mm margin.
COLLISION_R = 0.45  # match robot half-diagonal
CLEARANCE_PREFER = 1.5  # clearance (m) that scores maximum in W_CLEARANCE term

# ── Path tracking ─────────────────────────────────────────────────────────────
WP_TOLERANCE = 0.30    # distance to consider a waypoint reached (m)

# IMPROVE-1: adaptive lookahead
LOOKAHEAD_DIST_BASE   = 0.8   # m  minimum lookahead at zero speed
LOOKAHEAD_SPEED_SCALE = 0.6   # lookahead += cur_v * this

# IMPROVE-2: slow down near goal
SLOW_ZONE_R  = 1.5    # m  start decelerating within this distance of goal
APPROACH_V   = 0.25   # m/s  max speed inside SLOW_ZONE_R

# ── Recovery / replan ─────────────────────────────────────────────────────────
REPLAN_TICKS    = 25          # ticks before requesting replan (was 40)
# FIX-2: minimum seconds between two replan requests
REPLAN_COOLDOWN = 4.0


class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.create_subscription(Path,      '/planned_path', self.path_cb, 10)
        self.create_subscription(Odometry,  '/odom',         self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan',         self.scan_cb, 10)
        self.cmd_pub    = self.create_publisher(Twist,       '/cmd_vel',        10)
        self.replan_pub = self.create_publisher(PoseStamped, '/replan_request', 10)

        self.robot_x   = SPAWN_X
        self.robot_y   = SPAWN_Y
        self.robot_yaw = 0.0
        self.cur_v   = self.cur_w = 0.0
        self.waypoints     = []
        self.wp_index      = 0
        self.current_goal  = None
        self.obstacle_pts  = []
        self.scan_range_max = 10.0
        self.blocked_ticks  = 0
        self._last_replan_t = 0.0   # FIX-2

        self.create_timer(DT, self.control_step)
        self.get_logger().info('Robot Controller (DWB improved) ready.')

    # ── Sensor callbacks ──────────────────────────────────────────────────────

    def odom_cb(self, msg):
        # Gazebo 8 DiffDrive: odom origin = spawn point, add offset for world coords
        self.robot_x = msg.pose.pose.position.x + SPAWN_X
        self.robot_y = msg.pose.pose.position.y + SPAWN_Y
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z))
        self.cur_v = msg.twist.twist.linear.x
        self.cur_w = msg.twist.twist.angular.z

    def scan_cb(self, msg):
        self.scan_range_max = msg.range_max
        pts   = []
        angle = msg.angle_min
        rx, ry, ryaw = self.robot_x, self.robot_y, self.robot_yaw
        # LiDAR origin: 0.25 m forward of base_link in world frame
        lx = rx + LIDAR_X_OFFSET * math.cos(ryaw)
        ly = ry + LIDAR_X_OFFSET * math.sin(ryaw)
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max * 0.99:
                wx = lx + r * math.cos(ryaw + angle)
                wy = ly + r * math.sin(ryaw + angle)
                if math.hypot(wx - rx, wy - ry) > SELF_FILTER_R:
                    pts.append((wx, wy))
            angle += msg.angle_increment
        self.obstacle_pts = pts

    # ── Path callback ─────────────────────────────────────────────────────────

    def path_cb(self, msg):
        new_wps = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if not new_wps:
            return
        new_goal = new_wps[-1]
        same_goal = (
            self.current_goal is not None and
            math.hypot(new_goal[0] - self.current_goal[0],
                       new_goal[1] - self.current_goal[1]) < WP_TOLERANCE
        )
        if same_goal:
            best = min(range(len(new_wps)),
                       key=lambda i: math.hypot(new_wps[i][0] - self.robot_x,
                                                new_wps[i][1] - self.robot_y))
        else:
            best = 0
        self.waypoints     = new_wps
        self.wp_index      = best
        self.blocked_ticks = 0
        self.get_logger().info(
            f'New path: {len(new_wps)} wps, resuming from wp {best}.')

    # ── Replan request ────────────────────────────────────────────────────────

    def request_replan(self):
        """FIX-2: rate-limited replan requests."""
        if not self.current_goal:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_replan_t < REPLAN_COOLDOWN:
            return
        self._last_replan_t = now
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.pose.position.x = self.current_goal[0]
        msg.pose.position.y = self.current_goal[1]
        msg.pose.orientation.w = 1.0
        self.replan_pub.publish(msg)
        self.blocked_ticks = 0
        self.get_logger().info('DWB: requesting global replan.')

    # ── Carrot point (adaptive lookahead) ─────────────────────────────────────

    def get_carrot(self):
        """IMPROVE-1: lookahead scales with current speed."""
        while self.wp_index < len(self.waypoints) - 1:
            tx, ty = self.waypoints[self.wp_index]
            if math.hypot(tx - self.robot_x, ty - self.robot_y) < WP_TOLERANCE:
                self.wp_index += 1
            else:
                break
        if self.wp_index >= len(self.waypoints):
            return None

        lookahead = LOOKAHEAD_DIST_BASE + abs(self.cur_v) * LOOKAHEAD_SPEED_SCALE
        acc = 0.0
        px, py = self.robot_x, self.robot_y
        for i in range(self.wp_index, len(self.waypoints)):
            wx, wy = self.waypoints[i]
            seg = math.hypot(wx - px, wy - py)
            if acc + seg >= lookahead:
                r = (lookahead - acc) / seg if seg > 0 else 0.0
                return (px + r * (wx - px), py + r * (wy - py))
            acc += seg
            px, py = wx, wy
        return self.waypoints[-1]

    # ── Arc clearance ─────────────────────────────────────────────────────────

    def _arc_clearance(self, v, w):
        if not self.obstacle_pts:
            return self.scan_range_max
        x, y, yaw = self.robot_x, self.robot_y, self.robot_yaw
        steps = max(3, int(SIM_TIME / DT))
        md = self.scan_range_max
        for _ in range(steps):
            x   += v * math.cos(yaw) * DT
            y   += v * math.sin(yaw) * DT
            yaw  = (yaw + w * DT + math.pi) % (2 * math.pi) - math.pi
            for ox, oy in self.obstacle_pts:
                d = math.hypot(ox - x, oy - y)
                if d < md:
                    md = d
            if md < COLLISION_R:
                return 0.0
        return md

    # ── DWB scoring ───────────────────────────────────────────────────────────

    def dwb_best_command(self, cx, cy):
        """FIX-3: carrot direction computed once, before the simulation loop."""
        v_min = max(MIN_V, self.cur_v - MAX_AV * DT)
        v_max = min(MAX_V, self.cur_v + MAX_AV * DT)
        w_min = max(-MAX_W, self.cur_w - MAX_AW * DT)
        w_max = min(MAX_W,  self.cur_w + MAX_AW * DT)

        # IMPROVE-2: cap max speed near goal
        if self.current_goal:
            goal_dist = math.hypot(
                self.current_goal[0] - self.robot_x,
                self.current_goal[1] - self.robot_y)
            if goal_dist < SLOW_ZONE_R:
                v_max = min(v_max, APPROACH_V)

        # Desired heading: direction from robot to carrot
        desired_heading = math.atan2(cy - self.robot_y, cx - self.robot_x)

        best_score = -math.inf
        bv = bw = 0.0
        found  = False
        steps  = max(3, int(SIM_TIME / DT))

        for iv in range(N_V):
            v = v_min + (v_max - v_min) * iv / max(N_V - 1, 1)
            for iw in range(N_W):
                w  = w_min + (w_max - w_min) * iw / max(N_W - 1, 1)
                cl = self._arc_clearance(v, w)
                if cl < COLLISION_R:
                    continue

                # Simulate arc
                sx, sy, syaw = self.robot_x, self.robot_y, self.robot_yaw
                for _ in range(steps):
                    sx  += v * math.cos(syaw) * DT
                    sy  += v * math.sin(syaw) * DT
                    syaw = (syaw + w * DT + math.pi) % (2 * math.pi) - math.pi

                # FIX-3: heading error between final yaw and desired heading
                he = abs(norm_angle(desired_heading - syaw))

                score = (W_HEADING   * (1 - he / math.pi) +
                         W_CLEARANCE * min(1.0, cl / CLEARANCE_PREFER) +
                         W_VELOCITY  * (v / MAX_V if v > 0 else 0.0))
                if score > best_score:
                    best_score = score
                    bv, bw     = v, w
                    found      = True

        if not found:
            # FIX-4: sweep full angular range for escape rotation
            return self._escape_rotation()
        return bv, bw

    def _escape_rotation(self):
        """FIX-4: finer sweep of angular velocities when all arcs are blocked."""
        steps = 20
        for i in range(steps + 1):
            t  = i / steps
            wt = -MAX_W + 2 * MAX_W * t
            if self._arc_clearance(0.0, wt) >= COLLISION_R:
                return 0.0, wt
        self.get_logger().warn('DWB: fully blocked — stopping.')
        return 0.0, 0.0

    # ── Control loop ──────────────────────────────────────────────────────────

    def control_step(self):
        twist = Twist()
        if not self.waypoints or self.wp_index >= len(self.waypoints):
            self.cmd_pub.publish(twist)
            return

        self.current_goal = self.waypoints[-1]
        gx, gy = self.waypoints[-1]

        if math.hypot(gx - self.robot_x, gy - self.robot_y) < WP_TOLERANCE:
            self.get_logger().info(
                f'Goal reached! ({self.robot_x:.2f}, {self.robot_y:.2f})')
            self.current_goal = None
            self.waypoints    = []
            self.wp_index     = 0
            self.cmd_pub.publish(twist)
            return

        carrot = self.get_carrot()
        if carrot is None:
            self.cmd_pub.publish(twist)
            return

        bv, bw = self.dwb_best_command(*carrot)

        if abs(bv) < 0.01 and abs(bw) < 0.05:
            self.blocked_ticks += 1
            if self.blocked_ticks >= REPLAN_TICKS:
                self.request_replan()
        else:
            self.blocked_ticks = 0

        twist.linear.x  = bv
        twist.angular.z = bw
        self.cmd_pub.publish(twist)


def norm_angle(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def main(args=None):
    rclpy.init(args=args)
    n = RobotController()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()