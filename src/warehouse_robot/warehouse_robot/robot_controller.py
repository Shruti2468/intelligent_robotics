import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Twist
import math

OBSTACLE_STOP_DIST = 0.45
WP_TOLERANCE       = 0.35


class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        self.path_sub = self.create_subscription(
            Path,      '/planned_path', self.path_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry,  '/odom',         self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan',         self.scan_callback, 10)

        self.cmd_pub  = self.create_publisher(Twist,       '/cmd_vel',   10)
        # Publish replan requests back to path_planner
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.waypoints       = []
        self.wp_index        = 0
        self.robot_x         = 0.0
        self.robot_y         = 0.0
        self.robot_yaw       = 0.0
        self.obstacle_ahead  = False
        self.current_goal    = None   # remember goal so we can replan

        # How long we've been blocked — triggers replan after 1 s
        self.blocked_ticks   = 0
        self.REPLAN_TICKS    = 10    # 10 × 0.1 s = 1 s

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
        mid = n // 2                        # index 180 = straight ahead (0°)
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

    # ── Replan by republishing the same goal ──────────────────────────
    def request_replan(self):
        if self.current_goal is None:
            return
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.pose.position.x = self.current_goal[0]
        msg.pose.position.y = self.current_goal[1]
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)
        self.blocked_ticks = 0
        self.get_logger().info('Replanning around obstacle…')

    # ── Control loop 10 Hz ────────────────────────────────────────────
    def follow_path_step(self):
        if not self.waypoints or self.wp_index >= len(self.waypoints):
            self.cmd_pub.publish(Twist())
            return

        # Blocked — count ticks, replan after 1 s
        if self.obstacle_ahead:
            self.cmd_pub.publish(Twist())
            self.blocked_ticks += 1
            if self.blocked_ticks >= self.REPLAN_TICKS:
                self.request_replan()
            return

        self.blocked_ticks = 0

        tx, ty = self.waypoints[self.wp_index]
        dx     = tx - self.robot_x
        dy     = ty - self.robot_y
        dist   = math.hypot(dx, dy)

        # Remember last waypoint as current goal
        if self.wp_index == len(self.waypoints) - 1:
            self.current_goal = (tx, ty)

        if dist < WP_TOLERANCE:
            self.wp_index += 1
            if self.wp_index >= len(self.waypoints):
                self.get_logger().info(f'Goal reached! ({self.robot_x:.2f}, {self.robot_y:.2f})')
                self.current_goal = None
                self.waypoints = []
                self.wp_index = 0
                self.cmd_pub.publish(Twist())
            return

        # Heading error [-π, π]
        desired_yaw = math.atan2(dy, dx)
        angle_err   = desired_yaw - self.robot_yaw
        angle_err   = (angle_err + math.pi) % (2.0 * math.pi) - math.pi

        twist = Twist()
        twist.angular.z = max(-1.5, min(1.5, 2.0 * angle_err))

        # Speed: fast when aligned, still moves while correcting small errors
        if abs(angle_err) < 0.25:
            twist.linear.x = min(0.6, 0.9 * dist)   # full speed
        elif abs(angle_err) < 0.6:
            twist.linear.x = min(0.35, 0.5 * dist)  # moderate while correcting
        elif abs(angle_err) < 1.0:
            twist.linear.x = min(0.1, 0.2 * dist)   # creep
        # else: rotate only

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()