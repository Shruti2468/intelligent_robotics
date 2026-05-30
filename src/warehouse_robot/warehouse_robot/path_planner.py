import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from collections import deque
import math
import heapq

# ── Costmap parameters ────────────────────────────────────────────────
RESOLUTION   = 0.2
# Warehouse world: 80×80 m → 400×400 cells, covers -40 m … +40 m
MAP_SIZE     = 400
INFLATE_R    = 3
OBSTACLE_VAL = 255
FREE_VAL     = 0

GOAL_TOLERANCE  = 0.50   # metres — must be > GOAL_TOLERANCE in controller
REPLAN_INTERVAL = 1.0    # seconds — minimum time between replans while moving


def world_to_grid(wx, wy, origin_x, origin_y):
    return int((wx - origin_x) / RESOLUTION), int((wy - origin_y) / RESOLUTION)


def grid_to_world(gx, gy, origin_x, origin_y):
    return (gx * RESOLUTION + origin_x + RESOLUTION / 2.0,
            gy * RESOLUTION + origin_y + RESOLUTION / 2.0)


def in_bounds(gx, gy):
    return 0 <= gx < MAP_SIZE and 0 <= gy < MAP_SIZE


DIRS = [
    ( 1,  0, 1.0), (-1,  0, 1.0), ( 0,  1, 1.0), ( 0, -1, 1.0),
    ( 1,  1, 1.414), ( 1, -1, 1.414), (-1,  1, 1.414), (-1, -1, 1.414),
]


class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')

        self.robot_x        = 0.0
        self.robot_y        = 0.0
        self.robot_yaw      = 0.0
        self.goal           = None
        self.goal_satisfied = False
        self.last_replan_t  = 0.0   # ROS time of the last published plan

        self.origin_x = -(MAP_SIZE * RESOLUTION / 2.0)
        self.origin_y = -(MAP_SIZE * RESOLUTION / 2.0)
        self.costmap  = bytearray(MAP_SIZE * MAP_SIZE)

        self.odom_sub   = self.create_subscription(
            Odometry,    '/odom',           self.odom_callback,   10)
        self.scan_sub   = self.create_subscription(
            LaserScan,   '/scan',           self.scan_callback,    5)
        self.goal_sub   = self.create_subscription(
            PoseStamped, '/goal_pose',      self.goal_callback,   10)
        self.replan_sub = self.create_subscription(
            PoseStamped, '/replan_request', self.replan_callback, 10)

        self.path_pub = self.create_publisher(Path, '/planned_path', 10)

        self.get_logger().info(
            f'Path Planner ready. '
            f'Costmap: {MAP_SIZE}×{MAP_SIZE} cells = '
            f'{MAP_SIZE*RESOLUTION:.0f}×{MAP_SIZE*RESOLUTION:.0f} m '
            f'({self.origin_x:.0f}…{-self.origin_x:.0f} m)')

    # ── Odometry ──────────────────────────────────────────────────────
    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(
            2.0*(q.w*q.z + q.x*q.y),
            1.0 - 2.0*(q.y*q.y + q.z*q.z))

    # ── LiDAR → costmap ───────────────────────────────────────────────
    def scan_callback(self, msg):
        # Build costmap from this scan
        costmap = bytearray(MAP_SIZE * MAP_SIZE)
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max * 0.99:
                ox = self.robot_x + r * math.cos(self.robot_yaw + angle)
                oy = self.robot_y + r * math.sin(self.robot_yaw + angle)
                gx, gy = world_to_grid(ox, oy, self.origin_x, self.origin_y)
                if in_bounds(gx, gy):
                    for dx in range(-INFLATE_R, INFLATE_R + 1):
                        for dy in range(-INFLATE_R, INFLATE_R + 1):
                            nx, ny = gx + dx, gy + dy
                            if in_bounds(nx, ny):
                                costmap[ny * MAP_SIZE + nx] = OBSTACLE_VAL
            angle += msg.angle_increment
        self.costmap = costmap

        # Only replan if:
        #  1. We have an active, unsatisfied goal
        #  2. Enough time has passed since the last replan
        if not self.goal or self.goal_satisfied:
            return

        dist_to_goal = math.hypot(
            self.goal[0] - self.robot_x,
            self.goal[1] - self.robot_y)
        if dist_to_goal <= GOAL_TOLERANCE:
            self.goal_satisfied = True
            self.get_logger().info('Planner: goal satisfied, halting replans.')
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_replan_t < REPLAN_INTERVAL:
            return   # ← rate-limit: skip this scan's replan

        self.plan_and_publish(*self.goal)

    # ── New goal from RViz ─────────────────────────────────────────────
    def goal_callback(self, msg):
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        self.goal           = (gx, gy)
        self.goal_satisfied = False
        self.last_replan_t  = 0.0   # allow immediate replan for new goal
        self.get_logger().info(
            f'New goal: ({gx:.2f}, {gy:.2f})  '
            f'robot at ({self.robot_x:.2f}, {self.robot_y:.2f})')
        self.plan_and_publish(gx, gy)

    # ── Obstacle replan (does NOT reset goal_satisfied) ───────────────
    def replan_callback(self, msg):
        if not self.goal or self.goal_satisfied:
            return
        self.last_replan_t = 0.0   # allow immediate replan for obstacle
        self.plan_and_publish(*self.goal)

    # ── Helpers ───────────────────────────────────────────────────────
    def is_free(self, gx, gy):
        return in_bounds(gx, gy) and self.costmap[gy * MAP_SIZE + gx] == FREE_VAL

    def nearest_free(self, start):
        q, visited = deque([start]), {start}
        while q:
            cur = q.popleft()
            if self.is_free(*cur):
                return cur
            for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
                nb = (cur[0]+dx, cur[1]+dy)
                if in_bounds(*nb) and nb not in visited:
                    visited.add(nb); q.append(nb)
        return None

    # ── A* ────────────────────────────────────────────────────────────
    def astar(self, start_w, goal_w):
        s = world_to_grid(*start_w, self.origin_x, self.origin_y)
        g = world_to_grid(*goal_w,  self.origin_x, self.origin_y)

        if not in_bounds(*s):
            self.get_logger().warn('Start outside costmap!'); return None
        if not in_bounds(*g):
            self.get_logger().warn('Goal outside costmap!');  return None

        if not self.is_free(*s): s = self.nearest_free(s)
        if not self.is_free(*g): g = self.nearest_free(g)
        if s is None or g is None: return None

        heap      = [(0.0, s)]
        came_from = {}
        g_score   = {s: 0.0}

        while heap:
            _, cur = heapq.heappop(heap)
            if cur == g:
                path, node = [], cur
                while node in came_from:
                    path.append(grid_to_world(*node, self.origin_x, self.origin_y))
                    node = came_from[node]
                path.append(grid_to_world(*s, self.origin_x, self.origin_y))
                path.reverse()
                return path

            for dx, dy, cost in DIRS:
                nb = (cur[0]+dx, cur[1]+dy)
                if not self.is_free(*nb): continue
                tg = g_score[cur] + cost
                if tg < g_score.get(nb, float('inf')):
                    came_from[nb] = cur
                    g_score[nb]   = tg
                    h = math.hypot(nb[0]-g[0], nb[1]-g[1])
                    heapq.heappush(heap, (tg+h, nb))
        return None

    # ── Plan & publish ────────────────────────────────────────────────
    def plan_and_publish(self, gx, gy):
        dist = math.hypot(gx - self.robot_x, gy - self.robot_y)
        if dist < GOAL_TOLERANCE:
            self.goal_satisfied = True
            return

        pts = self.astar((self.robot_x, self.robot_y), (gx, gy))
        if not pts:
            self.get_logger().warn('A*: no path found!')
            return

        pts.append((gx, gy))

        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp    = self.get_clock().now().to_msg()
        for wx, wy in pts:
            p = PoseStamped()
            p.header             = msg.header
            p.pose.position.x    = wx
            p.pose.position.y    = wy
            p.pose.orientation.w = 1.0
            msg.poses.append(p)

        self.path_pub.publish(msg)
        self.last_replan_t = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info(f'A* path: {len(pts)} waypoints, {dist:.2f} m to goal')


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()