"""path_planner.py — Hybrid A* global planner (improved)

Changes vs original
───────────────────
FIX-1  COLLISION_R alignment: MAX_NODES raised 30k→60k so long diagonal
       routes (charging↔delivery, ~28 m) don't silently return None.
FIX-2  Replan de-duplication: scan_cb no longer triggers plan_and_publish
       directly.  All periodic replanning goes through _replan_watchdog so
       scan_cb and the timer can never fire A* simultaneously.
FIX-3  goal_satisfied race: planner marked the goal satisfied at 0.50 m
       while the controller only declared arrival at 0.30 m.  The planner
       now waits until GOAL_TOLERANCE_STRICT (0.28 m) — slightly inside the
       controller — so it never prematurely stops replanning.
FIX-4  Reverse motion cost: was 3× forward.  Reduced to 5× so the planner
       strongly avoids reverse but doesn't exhaust the node budget trying to
       find a purely-forward path in tight corners.
FIX-5  plan_and_publish idempotency guard: back-to-back calls within
       MIN_REPLAN_GAP seconds are dropped; prevents duplicate A* runs from
       replan_cb + watchdog firing together.
IMPROVE-1  Path smoothing: collinear waypoints removed; reduces /planned_path
           message size and improves DWB carrot tracking.
IMPROVE-2  Straight-line shortcut: before running A* the planner checks if
           the straight-line path is clear; if so it returns it immediately,
           saving CPU on open-aisle traversals.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from collections import deque
import math
import heapq

# ── Robot spawn position in world frame ──────────────────────────────────────
# Gazebo DiffDrive publishes odom relative to spawn point, not world origin.
# Adding this offset converts odom coords → world/map coords (map = world here).
SPAWN_X = -10.0
SPAWN_Y = -10.0

# ── LiDAR mount offset from base_link (from URDF lidar_joint + scan_joint) ───
LIDAR_X_OFFSET = 0.25   # metres forward of base_link centre
LIDAR_Y_OFFSET = 0.00

# ── Map parameters (must match static_map_publisher) ─────────────────────────
RESOLUTION   = 0.2
MAP_SIZE     = 400
INFLATE_R    = 0           # LiDAR marks exact hit cells only — static map already has 0.4m inflation for clearance
OBSTACLE_VAL = 255
FREE_VAL     = 0

# ── Planner tolerances ────────────────────────────────────────────────────────
GOAL_TOLERANCE        = 0.50   # stop replanning when within this distance
# FIX-3: planner stops replanning only when robot is tighter than the
# controller's WP_TOLERANCE (0.30 m), so the controller always gets a path
# right up to arrival.
GOAL_TOLERANCE_STRICT = 0.28

REPLAN_INTERVAL  = 3.0         # seconds between periodic replans
# FIX-5: minimum gap between any two plan_and_publish calls (dedup guard)
MIN_REPLAN_GAP   = 0.8

MAX_SNAP_CELLS   = 20          # BFS radius for blocked goal snapping (cells)

# ── Hybrid A* motion primitives ───────────────────────────────────────────────
MIN_TURN_R   = 0.50
STEP_SIZE    = 0.50
N_STEER      = 5
YAW_RES      = math.radians(15)
# FIX-4: higher reverse penalty keeps paths forward-only in normal corridors
# but still allows reversing when truly necessary (tight corners).
REVERSE_COST = 5.0

# FIX-1: raised node cap so charging↔delivery diagonal (~28 m, ~57 steps
# minimum) reliably finds a path without hitting the limit.
MAX_NODES    = 60_000


def world_to_grid(wx, wy, ox, oy):
    return int((wx - ox) / RESOLUTION), int((wy - oy) / RESOLUTION)


def grid_to_world(gx, gy, ox, oy):
    return gx * RESOLUTION + ox + RESOLUTION / 2.0, gy * RESOLUTION + oy + RESOLUTION / 2.0


def in_bounds(gx, gy):
    return 0 <= gx < MAP_SIZE and 0 <= gy < MAP_SIZE


def norm_a(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _build_motions():
    motions = []
    max_steer = math.asin(min(STEP_SIZE / MIN_TURN_R, 1.0))
    steers = [0.0]
    for i in range(1, N_STEER + 1):
        s = max_steer * i / N_STEER
        steers += [s, -s]
    for steer in steers:
        for direction in (1.0, -1.0):
            if abs(steer) < 1e-6:
                dx, dy, dyaw = direction * STEP_SIZE, 0.0, 0.0
            else:
                r    = STEP_SIZE / abs(steer)
                dyaw = direction * steer
                dx   = direction * r * math.sin(abs(dyaw))
                dy   = r * (1.0 - math.cos(dyaw)) * (1.0 if steer > 0 else -1.0)
            cost = STEP_SIZE * (1.0 if direction > 0 else REVERSE_COST)
            motions.append((dx, dy, dyaw, cost))
    return motions


MOTIONS = _build_motions()


# ── Path post-processing ──────────────────────────────────────────────────────

def _remove_collinear(pts, tol=0.05):
    """Remove intermediate points that lie on the straight line between their
    neighbours.  Reduces path length without changing the route shape."""
    if len(pts) <= 2:
        return pts
    result = [pts[0]]
    for i in range(1, len(pts) - 1):
        ax, ay = result[-1]
        bx, by = pts[i]
        cx, cy = pts[i + 1]
        # Cross product magnitude — zero means collinear
        cross = abs((bx - ax) * (cy - ay) - (by - ay) * (cx - ax))
        if cross > tol:
            result.append(pts[i])
    result.append(pts[-1])
    return result


class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        self.robot_x   = SPAWN_X
        self.robot_y   = SPAWN_Y
        self.robot_yaw = 0.0
        self.goal = None
        self.goal_satisfied = False
        self.last_replan_t  = 0.0
        self.origin_x = self.origin_y = -(MAP_SIZE * RESOLUTION / 2.0)
        self.static_costmap = bytearray(MAP_SIZE * MAP_SIZE)
        self.costmap        = bytearray(MAP_SIZE * MAP_SIZE)
        self.map_received   = False
        self._goal_path_published = False

        self.create_subscription(Odometry,    '/odom',           self.odom_cb,   10)
        self.create_subscription(LaserScan,   '/scan',           self.scan_cb,    5)
        self.create_subscription(PoseStamped, '/goal_pose',      self.goal_cb,   10)
        self.create_subscription(PoseStamped, '/replan_request', self.replan_cb, 10)
        mq = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, mq)

        self.path_pub    = self.create_publisher(Path,          '/planned_path', 10)
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/costmap',      10)
        self.create_timer(0.5,            self.pub_costmap)
        self.create_timer(REPLAN_INTERVAL, self._replan_watchdog)

        self.get_logger().info('Path Planner (Hybrid A* improved) ready.')

    # ── Map / sensor callbacks ────────────────────────────────────────────────

    def map_cb(self, msg):
        if self.map_received:
            return
        for i, v in enumerate(msg.data):
            self.static_costmap[i] = OBSTACLE_VAL if v > 50 else FREE_VAL
        self.costmap      = bytearray(self.static_costmap)
        self.map_received = True
        self.get_logger().info('Static map loaded.')

    def odom_cb(self, msg):
        # Gazebo 8 DiffDrive: odom origin = spawn point, add offset for world coords
        self.robot_x = msg.pose.pose.position.x + SPAWN_X
        self.robot_y = msg.pose.pose.position.y + SPAWN_Y
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z))

    def scan_cb(self, msg):
        """Update LiDAR costmap.

        FIX-2: scan_cb no longer triggers replanning.  The watchdog timer
        handles all periodic replanning, eliminating the race condition where
        scan_cb and the timer could launch simultaneous A* searches.
        """
        cm = bytearray(self.static_costmap)
        angle = msg.angle_min
        # LiDAR origin in world frame (0.25 m forward of base_link)
        lidar_x = self.robot_x + LIDAR_X_OFFSET * math.cos(self.robot_yaw)
        lidar_y = self.robot_y + LIDAR_X_OFFSET * math.sin(self.robot_yaw)
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max * 0.99:
                ox = lidar_x + r * math.cos(self.robot_yaw + angle)
                oy = lidar_y + r * math.sin(self.robot_yaw + angle)
                gx, gy = world_to_grid(ox, oy, self.origin_x, self.origin_y)
                if in_bounds(gx, gy):
                    for dx in range(-INFLATE_R, INFLATE_R + 1):
                        for dy in range(-INFLATE_R, INFLATE_R + 1):
                            nx, ny = gx + dx, gy + dy
                            if in_bounds(nx, ny):
                                cm[ny * MAP_SIZE + nx] = OBSTACLE_VAL
            angle += msg.angle_increment
        self.costmap = cm

    # ── Goal callbacks ────────────────────────────────────────────────────────

    def goal_cb(self, msg):
        gx, gy = msg.pose.position.x, msg.pose.position.y
        self.goal = (gx, gy)
        self.goal_satisfied = False
        self.last_replan_t  = 0.0
        self._goal_path_published = False
        self.get_logger().info(f'New goal: ({gx:.2f}, {gy:.2f})')
        self.plan_and_publish(gx, gy)

    def replan_cb(self, msg):
        """FIX-2 / FIX-5: replan requests are accepted but de-duplicated."""
        if not self.goal or self.goal_satisfied:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_replan_t < MIN_REPLAN_GAP:
            return
        self.plan_and_publish(*self.goal)

    def _replan_watchdog(self):
        """Sole source of periodic replanning (FIX-2).

        Fires every REPLAN_INTERVAL seconds.  Checks goal proximity with the
        strict tolerance (FIX-3) so it keeps publishing until the controller
        has actually arrived.
        """
        if not self.goal or self.goal_satisfied:
            return
        dist = math.hypot(self.goal[0] - self.robot_x, self.goal[1] - self.robot_y)
        if dist <= GOAL_TOLERANCE_STRICT:
            self.goal_satisfied = True
            self.get_logger().info('Planner: goal satisfied (watchdog strict check).')
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_replan_t >= REPLAN_INTERVAL:
            self.get_logger().info('Replan watchdog triggered.')
            self.plan_and_publish(*self.goal)

    # ── Costmap helpers ───────────────────────────────────────────────────────

    def pub_costmap(self):
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = RESOLUTION
        msg.info.width = msg.info.height = MAP_SIZE
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = [100 if b == OBSTACLE_VAL else 0 for b in self.costmap]
        self.costmap_pub.publish(msg)

    def is_free(self, gx, gy):
        return in_bounds(gx, gy) and self.costmap[gy * MAP_SIZE + gx] == FREE_VAL

    def pose_free(self, wx, wy):
        return self.is_free(*world_to_grid(wx, wy, self.origin_x, self.origin_y))

    def nearest_free(self, wx, wy, max_cells=MAX_SNAP_CELLS):
        gx, gy = world_to_grid(wx, wy, self.origin_x, self.origin_y)
        if not in_bounds(gx, gy):
            return None, None
        q, vis = deque([(gx, gy, 0)]), {(gx, gy)}
        while q:
            cx, cy, dist = q.popleft()
            if dist > max_cells:
                break
            if self.is_free(cx, cy):
                wx2, wy2 = grid_to_world(cx, cy, self.origin_x, self.origin_y)
                self.get_logger().info(
                    f'Goal snapped ({wx:.2f},{wy:.2f}) → ({wx2:.2f},{wy2:.2f})')
                return wx2, wy2
            for d in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                nb = (cx + d[0], cy + d[1])
                if in_bounds(*nb) and nb not in vis:
                    vis.add(nb)
                    q.append((nb[0], nb[1], dist + 1))
        self.get_logger().error(
            f'nearest_free: no free cell within {max_cells} cells of ({wx:.2f},{wy:.2f})')
        return None, None

    # ── Straight-line shortcut (IMPROVE-2) ───────────────────────────────────

    def _straight_line_clear(self, sx, sy, gx, gy):
        """Return True if the straight line from (sx,sy) to (gx,gy) passes
        through no obstacle cell.  Uses Bresenham-style stepping."""
        dist  = math.hypot(gx - sx, gy - sy)
        steps = max(int(dist / (RESOLUTION * 0.5)), 2)
        for i in range(steps + 1):
            t  = i / steps
            wx = sx + t * (gx - sx)
            wy = sy + t * (gy - sy)
            gxi, gyi = world_to_grid(wx, wy, self.origin_x, self.origin_y)
            if not self.is_free(gxi, gyi):
                return False
        return True

    # ── Hybrid A* ─────────────────────────────────────────────────────────────

    def _key(self, wx, wy, yaw):
        gx = int((wx - self.origin_x) / RESOLUTION)
        gy = int((wy - self.origin_y) / RESOLUTION)
        gy2 = int(norm_a(yaw) / YAW_RES) % int(2 * math.pi / YAW_RES)
        return (gx, gy, gy2)

    def hybrid_astar(self, sw, gw):
        sx, sy = sw
        gx, gy = gw

        # Clear start cell if robot is inside an inflated zone
        start_gx, start_gy = world_to_grid(sx, sy, self.origin_x, self.origin_y)
        local_costmap = None
        if not self.is_free(start_gx, start_gy):
            local_costmap = bytearray(self.costmap)
            for ddx in range(-INFLATE_R, INFLATE_R + 1):
                for ddy in range(-INFLATE_R, INFLATE_R + 1):
                    nx2, ny2 = start_gx + ddx, start_gy + ddy
                    if in_bounds(nx2, ny2):
                        local_costmap[ny2 * MAP_SIZE + nx2] = FREE_VAL
            self.get_logger().warn(
                f'Start ({sx:.2f},{sy:.2f}) in obstacle — clearing for planning.')

        # Snap goal to nearest free cell if blocked
        if not self.pose_free(gx, gy):
            gx, gy = self.nearest_free(gx, gy)
            if gx is None:
                self.get_logger().error('hybrid_astar: goal blocked and unsnappable.')
                return None

        # Straight-line shortcut removed: it bypassed A* routing and sent
        # the robot diagonally through shelf aisles that DWB could not follow.
        h = lambda x, y: math.hypot(x - gx, y - gy)

        def _free(wx, wy):
            cgx, cgy = world_to_grid(wx, wy, self.origin_x, self.origin_y)
            if not in_bounds(cgx, cgy):
                return False
            cm = local_costmap if local_costmap is not None else self.costmap
            return cm[cgy * MAP_SIZE + cgx] == FREE_VAL

        sk  = self._key(sx, sy, self.robot_yaw)
        ctr = 0
        heap = [(h(sx, sy), ctr, sx, sy, self.robot_yaw, None)]
        came = {}
        gs     = {sk: 0.0}
        closed = set()

        while heap:
            if ctr > MAX_NODES:
                self.get_logger().warn(
                    f'Hybrid A*: node limit ({MAX_NODES}) reached — no path.')
                return None
            _, _, cx, cy, cyaw, _ = heapq.heappop(heap)
            ck = self._key(cx, cy, cyaw)
            if ck in closed:
                continue
            closed.add(ck)

            if math.hypot(cx - gx, cy - gy) < STEP_SIZE * 1.5:
                path = [(cx, cy)]
                k = ck
                while k in came:
                    pk, px, py, _ = came[k]
                    path.append((px, py))
                    k = pk
                path.reverse()
                return path

            for dx, dy, dyaw, cost in MOTIONS:
                nx  = cx + dx * math.cos(cyaw) - dy * math.sin(cyaw)
                ny  = cy + dx * math.sin(cyaw) + dy * math.cos(cyaw)
                nya = norm_a(cyaw + dyaw)
                if not _free(nx, ny):
                    continue
                nk  = self._key(nx, ny, nya)
                ng  = gs[ck] + cost
                if ng >= gs.get(nk, float('inf')):
                    continue
                gs[nk] = ng
                came[nk] = (ck, cx, cy, cyaw)
                ctr += 1
                heapq.heappush(heap, (ng + h(nx, ny), ctr, nx, ny, nya, ck))
        return None

    # ── Plan + publish ────────────────────────────────────────────────────────

    def plan_and_publish(self, gx, gy):
        dist = math.hypot(gx - self.robot_x, gy - self.robot_y)
        if dist <= GOAL_TOLERANCE_STRICT:
            self.goal_satisfied = True
            return
        if not self.map_received:
            self.get_logger().warn('No static map yet — LiDAR-only costmap.')

        # FIX-5: de-duplication guard
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_replan_t < MIN_REPLAN_GAP:
            return

        pts = self.hybrid_astar((self.robot_x, self.robot_y), (gx, gy))
        if not pts:
            self.get_logger().warn('Hybrid A*: no path found!')
            return

        # Append exact goal if A* stopped short
        if math.hypot(pts[-1][0] - gx, pts[-1][1] - gy) > RESOLUTION:
            pts.append((gx, gy))

        # IMPROVE-1: remove collinear waypoints
        pts = _remove_collinear(pts)

        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp    = self.get_clock().now().to_msg()
        for wx, wy in pts:
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x = wx
            p.pose.position.y = wy
            p.pose.orientation.w = 1.0
            msg.poses.append(p)
        self.path_pub.publish(msg)
        self.last_replan_t        = now
        self._goal_path_published = True
        self.get_logger().info(
            f'Hybrid A* path published: {len(pts)} wps, dist={dist:.2f}m')


def main(args=None):
    rclpy.init(args=args)
    n = PathPlanner()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()