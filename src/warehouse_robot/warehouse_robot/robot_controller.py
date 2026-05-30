import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Twist
import math

OBSTACLE_STOP_DIST = 0.45
WP_TOLERANCE       = 0.30   # tighter: 1.5 cells at 0.2 m/cell
LOOKAHEAD_DIST     = 1.2    # carrot-on-a-stick look-ahead in metres


class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        self.path_sub = self.create_subscription(
            Path,      '/planned_path', self.path_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry,  '/odom',         self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan',         self.scan_callback, 10)

        self.cmd_pub    = self.create_publisher(Twist,       '/cmd_vel',        10)
        # Use a dedicated replan topic so the planner can distinguish
        # "obstacle replan" from "brand new goal" and won't reset goal_satisfied.
        self.replan_pub = self.create_publisher(PoseStamped, '/replan_request',  10)

        self.waypoints       = []
        self.wp_index        = 0
        self.robot_x         = 0.0
        self.robot_y         = 0.0
        self.robot_yaw       = 0.0
        self.obstacle_ahead  = False
        self.current_goal    = None

        self.blocked_ticks   = 0
        self.REPLAN_TICKS    = 10

        self.create_timer(0.1, self.follow_path_step)
        self.get_logger().info('Robot Controller ready.')

    # ── Odometry ──────────────────────────────────────────────────────
    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    # ── LiDAR ─────────────────────────────────────────────────────────
    def scan_callback(self, msg):
        n   = len(msg.ranges)
        arc = max(1, int(n * 25 / 360))
        mid = n // 2
        front = msg.ranges[mid - arc : mid + arc]
        valid = [r for r in front if msg.range_min < r < msg.range_max]
        blocked = bool(valid) and min(valid) < OBSTACLE_STOP_DIST
        if blocked and not self.obstacle_ahead:
            self.get_logger().warn(f'Obstacle {min(valid):.2f} m — stopping.')
        elif not blocked and self.obstacle_ahead:
            self.get_logger().info('Clear — resuming.')
        self.obstacle_ahead = blocked

    # ── Path ──────────────────────────────────────────────────────────
    def path_callback(self, msg):
        self.waypoints = [
            (p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.wp_index    = 0
        self.blocked_ticks = 0
        self.get_logger().info(
            f'New path: {len(self.waypoints)} waypoints.')

    # ── Replan ────────────────────────────────────────────────────────
    def request_replan(self):
        if self.current_goal is None:
            return
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.pose.position.x = self.current_goal[0]
        msg.pose.position.y = self.current_goal[1]
        msg.pose.orientation.w = 1.0
        self.replan_pub.publish(msg)   # ← /replan_request, not /goal_pose
        self.blocked_ticks = 0
        self.get_logger().info('Replanning around obstacle…')

    # ── Lookahead: find the carrot point ─────────────────────────────
    # FIX: Instead of steering toward the immediate next waypoint (which
    # causes jerky turns at every grid cell), project a point LOOKAHEAD_DIST
    # ahead along the path.  The robot steers smoothly toward that carrot,
    # and also advances wp_index whenever a waypoint falls behind the robot —
    # eliminating the "robot circles a waypoint it overshot" problem.
    def get_carrot(self):
        # First advance past any waypoints already behind the robot
        while self.wp_index < len(self.waypoints) - 1:
            tx, ty = self.waypoints[self.wp_index]
            if math.hypot(tx - self.robot_x, ty - self.robot_y) < WP_TOLERANCE:
                self.wp_index += 1
            else:
                break

        if self.wp_index >= len(self.waypoints):
            return None

        # Walk forward along the remaining path until we reach LOOKAHEAD_DIST
        acc = 0.0
        px, py = self.robot_x, self.robot_y
        for i in range(self.wp_index, len(self.waypoints)):
            wx, wy = self.waypoints[i]
            seg = math.hypot(wx - px, wy - py)
            if acc + seg >= LOOKAHEAD_DIST:
                # Interpolate within this segment
                ratio = (LOOKAHEAD_DIST - acc) / seg if seg > 0 else 0.0
                return (px + ratio * (wx - px), py + ratio * (wy - py))
            acc += seg
            px, py = wx, wy

        # Lookahead overshoots the end — just use the final waypoint
        return self.waypoints[-1]

    # ── Control loop 10 Hz ────────────────────────────────────────────
    def follow_path_step(self):
        if not self.waypoints or self.wp_index >= len(self.waypoints):
            self.cmd_pub.publish(Twist())
            return

        if self.obstacle_ahead:
            self.cmd_pub.publish(Twist())
            self.blocked_ticks += 1
            if self.blocked_ticks >= self.REPLAN_TICKS:
                self.request_replan()
            return

        self.blocked_ticks = 0

        # Remember the final waypoint as the current goal for replanning
        self.current_goal = self.waypoints[-1]

        # Check goal reached
        gx, gy = self.waypoints[-1]
        if math.hypot(gx - self.robot_x, gy - self.robot_y) < WP_TOLERANCE:
            self.get_logger().info(
                f'Goal reached! ({self.robot_x:.2f}, {self.robot_y:.2f})')
            self.current_goal = None
            self.waypoints = []
            self.wp_index = 0
            self.cmd_pub.publish(Twist())
            return

        carrot = self.get_carrot()
        if carrot is None:
            self.cmd_pub.publish(Twist())
            return

        cx, cy = carrot
        dx = cx - self.robot_x
        dy = cy - self.robot_y
        dist_to_carrot = math.hypot(dx, dy)
        dist_to_goal   = math.hypot(gx - self.robot_x, gy - self.robot_y)

        desired_yaw = math.atan2(dy, dx)
        angle_err   = desired_yaw - self.robot_yaw
        angle_err   = (angle_err + math.pi) % (2.0 * math.pi) - math.pi

        twist = Twist()

        # FIX: Lower angular gain (1.2 vs 2.0) to reduce oscillation on
        # straights.  Speed now scales with goal distance, not carrot distance,
        # so the robot doesn't slow down just because the carrot is close.
        twist.angular.z = max(-1.5, min(1.5, 1.2 * angle_err))

        if abs(angle_err) < 0.3:
            twist.linear.x = min(0.8, 0.9 * dist_to_goal)   # fast when aligned
        elif abs(angle_err) < 0.7:
            twist.linear.x = min(0.45, 0.6 * dist_to_goal)  # moderate
        elif abs(angle_err) < 1.2:
            twist.linear.x = min(0.15, 0.3 * dist_to_goal)  # creep
        # else: rotate in place — no forward motion

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()