import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

RESOLUTION = 0.2
MAP_SIZE   = 400   # 80×80 m, -40…+40 m

ORIGIN_X = -(MAP_SIZE * RESOLUTION / 2.0)
ORIGIN_Y = -(MAP_SIZE * RESOLUTION / 2.0)

# ── FIX 1: Inflate static obstacles by this many cells so A* keeps
#           the robot clear of shelves before LiDAR even sees them.
#           4 cells × 0.2 m = 0.8 m clearance on every side.
INFLATE_R = 3 

def world_to_grid(wx, wy):
    return int((wx - ORIGIN_X) / RESOLUTION), int((wy - ORIGIN_Y) / RESOLUTION)


def in_bounds(gx, gy):
    return 0 <= gx < MAP_SIZE and 0 <= gy < MAP_SIZE


def fill_box(grid, cx, cy, half_x, half_y):
    """Mark a solid box obstacle (world coords, half-sizes in metres)."""
    x0, y0 = world_to_grid(cx - half_x, cy - half_y)
    x1, y1 = world_to_grid(cx + half_x, cy + half_y)
    for gx in range(x0, x1 + 1):
        for gy in range(y0, y1 + 1):
            if in_bounds(gx, gy):
                grid[gy * MAP_SIZE + gx] = 100


def inflate(grid, radius):
    """Expand every marked cell outward by radius cells."""
    original = bytearray(grid)          # snapshot so we don't inflate inflations
    for gy in range(MAP_SIZE):
        for gx in range(MAP_SIZE):
            if original[gy * MAP_SIZE + gx] == 100:
                for dx in range(-radius, radius + 1):
                    for dy in range(-radius, radius + 1):
                        nx, ny = gx + dx, gy + dy
                        if in_bounds(nx, ny):
                            grid[ny * MAP_SIZE + nx] = 100


class StaticMapPublisher(Node):
    def __init__(self):
        super().__init__('static_map_publisher')

        # ── FIX 2: TRANSIENT_LOCAL ("latched") so path_planner receives
        #           the map even if it starts after this node does. ──────
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)

        self.pub  = self.create_publisher(OccupancyGrid, '/map', map_qos)
        self.grid = bytearray(MAP_SIZE * MAP_SIZE)   # all free

        self._build_map()

        # Publish once immediately, then keep at 1 Hz for RViz
        self.publish_map()
        self.create_timer(1.0, self.publish_map)
        self.get_logger().info('Static map publisher ready.')

    def _build_map(self):
        g = self.grid

        # ── WALLS ────────────────────────────────────────────────────
        fill_box(g,   0,  12, 15.0, 0.15)   # north wall
        fill_box(g,   0, -12, 15.0, 0.15)   # south wall
        fill_box(g,  15,   0,  0.15, 12.0)  # east wall
        fill_box(g, -15,   0,  0.15, 12.0)  # west wall

        # ── SHELF FRAMES ─────────────────────────────────────────────
        # SDF frame:  2.0 m × 0.08 m
        # SDF boards: 2.0 m × 0.85 m  ← actual physical footprint
        # FIX 3: Use board depth (0.85/2 = 0.425) not frame depth (0.04)
        shelf_positions = [
            (-9, 6), (-3, 6), (3, 6), (9, 6),    # Row 1  y= 6
            (-9, 0), (-3, 0), (3, 0), (9, 0),    # Row 2  y= 0
            (-9,-6), (-3,-6), (3,-6), (9,-6),    # Row 3  y=-6
        ]
        for sx, sy in shelf_positions:
            fill_box(g, sx, sy, 1.0, 0.50)   # 2.0 m × 0.9 m footprint

        # ── SUPPORT PILLARS ───────────────────────────────────────────
        for px, py in [(-13, 10), (13, 10), (-13, -10), (13, -10)]:
            fill_box(g, px, py, 0.2, 0.2)

        # ── CHARGING STATION (-10, -10) ───────────────────────────────
        # North border removed: robot approaches from north (y > -10).
        # After inflation the north border at y=-8.8 becomes y=-9.35,
        # blocking the only approach path to the goal at y=-10.
        fill_box(g, -10, -11.2, 1.5, 0.15)   # south edge only
        fill_box(g, -11.7,-10,  0.15, 1.0)   # west edge
        fill_box(g,  -8.3,-10,  0.15, 1.0)   # east edge

        # ── DELIVERY STATION (10, -10) ────────────────────────────────
        # Synced with Gazebo: delivery_station_base at (10,-10), size 3.0x2.0
        # North + west borders removed so robot can approach from north aisle.
        # East edge at x=11.5 (pad half-width=1.5 from centre x=10).
        fill_box(g,  10, -11.2, 1.5, 0.15)   # south edge (y=-12+0.8=pad south)
        fill_box(g,  11.5,-10,  0.15, 1.0)   # east edge (pad right edge)

        # ── DRONE PAD — open area, not an obstacle ────────────────────

        # ── FIX 5: Inflate all obstacles after marking ────────────────
        inflate(g, INFLATE_R)

        self.get_logger().info('Static warehouse map built and inflated.')

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = RESOLUTION
        msg.info.width  = MAP_SIZE
        msg.info.height = MAP_SIZE
        msg.info.origin.position.x = ORIGIN_X
        msg.info.origin.position.y = ORIGIN_Y
        msg.info.origin.orientation.w = 1.0
        msg.data = list(self.grid)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StaticMapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()