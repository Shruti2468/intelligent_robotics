import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math


# How close an obstacle has to be (metres) before we stop
OBSTACLE_STOP_DIST = 0.4
# Waypoint reached tolerance (metres)
WP_TOLERANCE = 0.15


class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        self.path_sub = self.create_subscription(
            Path, '/planned_path', self.path_callback, 10)

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.waypoints  = []
        self.wp_index   = 0
        self.robot_x    = 0.0
        self.robot_y    = 0.0
        self.robot_yaw  = 0.0
        self.obstacle_ahead = False

        self.create_timer(0.1, self.follow_path_step)

        self.get_logger().info('Robot Controller ready.')

    # ── Odometry ───────────────────────────────────────────────────────
    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)

    # ── LiDAR — check the forward arc for obstacles ────────────────────
    def scan_callback(self, msg):
        # Look at the front 60° arc (-30° to +30°)
        n = len(msg.ranges)
        arc = int(n * 30 / 360)      # 30° worth of indices
        front = msg.ranges[:arc] + msg.ranges[n - arc:]
        valid = [r for r in front if msg.range_min < r < msg.range_max]
        if valid and min(valid) < OBSTACLE_STOP_DIST:
            if not self.obstacle_ahead:
                self.get_logger().warn(
                    f'Obstacle detected at {min(valid):.2f} m — stopping.')
            self.obstacle_ahead = True
        else:
            if self.obstacle_ahead:
                self.get_logger().info('Path clear — resuming.')
            self.obstacle_ahead = False

    # ── Path ───────────────────────────────────────────────────────────
    def path_callback(self, msg):
        self.waypoints = [
            (p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.wp_index = 0
        self.get_logger().info(
            f'New path: {len(self.waypoints)} waypoints received.')

    # ── Control loop at 10 Hz ──────────────────────────────────────────
    def follow_path_step(self):
        if not self.waypoints or self.wp_index >= len(self.waypoints):
            self.cmd_pub.publish(Twist())
            return

        if self.obstacle_ahead:
            self.cmd_pub.publish(Twist())
            return

        tx, ty = self.waypoints[self.wp_index]
        dx = tx - self.robot_x
        dy = ty - self.robot_y
        dist = math.hypot(dx, dy)

        # Skip intermediate waypoints aggressively
        if dist < 0.3:
            self.wp_index += 1
            if self.wp_index >= len(self.waypoints):
                self.get_logger().info(
                    f'Goal reached! Final pos: ({self.robot_x:.2f}, {self.robot_y:.2f})')
                self.cmd_pub.publish(Twist())
            return

        desired_yaw = math.atan2(dy, dx)
        angle_err   = desired_yaw - self.robot_yaw
        angle_err   = (angle_err + math.pi) % (2.0 * math.pi) - math.pi

        twist = Twist()
        twist.angular.z = max(-1.5, min(1.5, 2.0 * angle_err))

        if abs(angle_err) < 0.4:
            twist.linear.x = min(0.5, 0.8 * dist)   # faster
        elif abs(angle_err) < 0.8:
            twist.linear.x = min(0.2, 0.3 * dist)   # slow while turning
        # else: just rotate, don't drive

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()