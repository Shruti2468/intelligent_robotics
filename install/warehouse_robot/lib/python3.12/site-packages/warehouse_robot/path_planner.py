import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from collections import deque
import math
import heapq

# ── Costmap parameters ────────────────────────────────────────────────
RESOLUTION   = 0.2      # metres per cell
MAP_SIZE     = 200      # cells  →  40 m × 40 m arena
INFLATE_R    = 2        # inflation radius in cells (~0.6 m buffer)
OBSTACLE_VAL = 255
FREE_VAL     = 0


def world_to_grid(wx, wy, origin_x, origin_y):
    return (int((wx - origin_x) / RESOLUTION),
            int((wy - origin_y) / RESOLUTION))


def grid_to_world(gx, gy, origin_x, origin_y):
    return (gx * RESOLUTION + origin_x + RESOLUTION / 2.0,
            gy * RESOLUTION + origin_y + RESOLUTION / 2.0)


def in_bounds(gx, gy):
    return 0 <= gx < MAP_SIZE and 0 <= gy < MAP_SIZE


# 8-connected neighbours with costs
DIRS = [
    ( 1,  0, 1.0),
    (-1,  0, 1.0),
    ( 0,  1, 1.0),
    ( 0, -1, 1.0),
    ( 1,  1, 1.414),
    ( 1, -1, 1.414),
    (-1,  1, 1.414),
    (-1, -1, 1.414),
]


class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')

        self.robot_x   = 0.0
        self.robot_y   = 0.0
        self.robot_yaw = 0.0
        self.goal      = None

        # Costmap centred on world origin
        self.origin_x = -(MAP_SIZE * RESOLUTION / 2.0)
        self.origin_y = -(MAP_SIZE * RESOLUTION / 2.0)
        self.costmap  = bytearray(MAP_SIZE * MAP_SIZE)   # fast zero array

        self.odom_sub = self.create_subscription(
            Odometry,   '/odom',      self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan,  '/scan',      self.scan_callback, 5)
        self.goal_sub = self.create_subscription(
            PoseStamped,'/goal_pose', self.goal_callback, 10)

        self.path_pub = self.create_publisher(Path, '/planned_path', 10)

        self.get_logger().info(
            'Path Planner (A* + LiDAR costmap) ready.\n'
            '  Set goal: RViz → "2D Nav Goal"')

    # ── Odometry ──────────────────────────────────────────────────────
    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    # ── LiDAR → costmap ───────────────────────────────────────────────
    def scan_callback(self, msg):
        costmap = bytearray(MAP_SIZE * MAP_SIZE)   # fresh map each scan

        angle = msg.angle_min
        for r in msg.ranges:
            angle += msg.angle_increment
            if not (msg.range_min < r < msg.range_max * 0.99):
                continue

            ox = self.robot_x + r * math.cos(self.robot_yaw + angle)
            oy = self.robot_y + r * math.sin(self.robot_yaw + angle)
            gx, gy = world_to_grid(ox, oy, self.origin_x, self.origin_y)
            if not in_bounds(gx, gy):
                continue

            # Inflate obstacle
            for dx in range(-INFLATE_R, INFLATE_R + 1):
                for dy in range(-INFLATE_R, INFLATE_R + 1):
                    nx, ny = gx + dx, gy + dy
                    if in_bounds(nx, ny):
                        costmap[ny * MAP_SIZE + nx] = OBSTACLE_VAL

        self.costmap = costmap

        # Re-plan automatically whenever scan updates and we have a goal
        if self.goal:
            dist_to_goal = math.hypot(
                self.goal[0] - self.robot_x,
                self.goal[1] - self.robot_y)
            if dist_to_goal > 0.2:
                self.plan_and_publish(*self.goal)

    # ── Goal ──────────────────────────────────────────────────────────
    def goal_callback(self, msg):
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        self.goal = (gx, gy)
        self.get_logger().info(
            f'Goal: ({gx:.2f}, {gy:.2f})  '
            f'Robot: ({self.robot_x:.2f}, {self.robot_y:.2f})')
        self.plan_and_publish(gx, gy)

    # ── Helpers ───────────────────────────────────────────────────────
    def is_free(self, gx, gy):
        return in_bounds(gx, gy) and self.costmap[gy * MAP_SIZE + gx] == FREE_VAL

    def nearest_free(self, start):
        """BFS from start until a free cell is found."""
        q = deque([start])
        visited = {start}
        while q:
            cur = q.popleft()
            if self.is_free(*cur):
                return cur
            for dx, dy, _ in DIRS:
                nb = (cur[0] + dx, cur[1] + dy)
                if in_bounds(*nb) and nb not in visited:
                    visited.add(nb)
                    q.append(nb)
        return None

    # ── A* ────────────────────────────────────────────────────────────
    def astar(self, start_w, goal_w):
        s = world_to_grid(*start_w, self.origin_x, self.origin_y)
        g = world_to_grid(*goal_w,  self.origin_x, self.origin_y)

        if not in_bounds(*s):
            self.get_logger().warn('Start outside costmap!')
            return None
        if not in_bounds(*g):
            self.get_logger().warn('Goal outside costmap!')
            return None

        # Snap start/goal to nearest free cell if occupied
        if not self.is_free(*s):
            s = self.nearest_free(s)
        if not self.is_free(*g):
            g = self.nearest_free(g)
        if s is None or g is None:
            return None

        heap      = [(0.0, s)]
        came_from = {}
        g_score   = {s: 0.0}

        while heap:
            _, cur = heapq.heappop(heap)
            if cur == g:
                # Reconstruct path
                path = []
                while cur in came_from:
                    path.append(grid_to_world(*cur, self.origin_x, self.origin_y))
                    cur = came_from[cur]
                path.reverse()
                return path

            for dx, dy, cost in DIRS:
                nb = (cur[0] + dx, cur[1] + dy)
                if not self.is_free(*nb):
                    continue
                tg = g_score[cur] + cost
                if tg < g_score.get(nb, float('inf')):
                    came_from[nb] = cur
                    g_score[nb]   = tg
                    h = math.hypot(nb[0] - g[0], nb[1] - g[1])
                    heapq.heappush(heap, (tg + h, nb))

        return None

    # ── Plan & publish ────────────────────────────────────────────────
    def plan_and_publish(self, gx, gy):
        dist = math.hypot(gx - self.robot_x, gy - self.robot_y)
        if dist < 0.1:
            self.get_logger().warn('Goal too close to robot.')
            return

        pts = self.astar((self.robot_x, self.robot_y), (gx, gy))
        if not pts:
            self.get_logger().warn('A*: no path found!')
            return

        pts.append((gx, gy))   # exact goal at end

        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp    = self.get_clock().now().to_msg()
        for wx, wy in pts:
            p                    = PoseStamped()
            p.header             = msg.header
            p.pose.position.x    = wx
            p.pose.position.y    = wy
            p.pose.orientation.w = 1.0
            msg.poses.append(p)

        self.path_pub.publish(msg)
        self.get_logger().info(
            f'A* path: {len(pts)} waypoints, {dist:.2f} m to goal')


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()