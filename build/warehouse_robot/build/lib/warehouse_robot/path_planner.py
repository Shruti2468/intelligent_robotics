import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
import math


class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')

        # Current robot position from odometry (no map needed)
        self.robot_x = 0.0
        self.robot_y = 0.0

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

        self.path_pub = self.create_publisher(Path, '/planned_path', 10)

        self.get_logger().info(
            'Path Planner ready (no map needed).\n'
            '  • Set goal: RViz → "2D Nav Goal"')

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def goal_callback(self, msg):
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        self.get_logger().info(
            f'Goal set: ({gx:.2f}, {gy:.2f})  '
            f'Robot at: ({self.robot_x:.2f}, {self.robot_y:.2f})')
        self.plan_and_publish(gx, gy)

    def plan_and_publish(self, gx, gy):
        """
        Straight-line path with N intermediate waypoints.
        No map or obstacle data needed — robot_controller handles
        stopping if the LiDAR detects something close.
        """
        sx, sy = self.robot_x, self.robot_y
        dist = math.hypot(gx - sx, gy - sy)

        if dist < 0.1:
            self.get_logger().warn('Goal is too close to current position.')
            return

        # One waypoint every ~0.2 m
        n_steps = max(2, int(dist / 0.2))
        pts = [
            (
                sx + (gx - sx) * i / n_steps,
                sy + (gy - sy) * i / n_steps,
            )
            for i in range(1, n_steps + 1)
        ]

        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        for wx, wy in pts:
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x = wx
            p.pose.position.y = wy
            p.pose.orientation.w = 1.0
            msg.poses.append(p)

        self.path_pub.publish(msg)
        self.get_logger().info(
            f'Path published: {len(pts)} waypoints, '
            f'distance {dist:.2f} m')


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()