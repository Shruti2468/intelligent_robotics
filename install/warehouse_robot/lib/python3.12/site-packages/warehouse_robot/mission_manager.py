import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
import math
import enum


# ── Mission waypoints (world coordinates) ────────────────────────────────────
CHARGING_STATION  = (-10.0, -10.0)   # start / home
# Move waypoints into clear aisle centres
SHELF_1_APPROACH   = (-8.0,  9.0)   # north aisle, x=-8 clear of shelf inflation
DELIVERY_EAST_WP   = (11.0,  9.0)   # east corridor
DELIVERY_STATION   = (10.0, -10.0)
CHARGE_APPROACH_WP = (-5.0, -10.0)
# Gazebo odom frame origin = robot spawn point in world
SPAWN_X = -10.0
SPAWN_Y = -10.0

GOAL_TOLERANCE    = 0.40             # metres — matches planner GOAL_TOLERANCE
REPLAN_TIMEOUT    = 15.0             # seconds before we warn + trigger replan
# BUG FIX #4: only warn once per leg, then retry every REPLAN_RETRY_INTERVAL.
REPLAN_RETRY_INTERVAL = 5.0          # seconds between forced replan attempts after timeout


class MissionState(enum.Enum):
    IDLE                = "IDLE"
    GO_TO_SHELF         = "GO_TO_SHELF"
    WAIT_AT_SHELF       = "WAIT_AT_SHELF"
    GO_TO_DELIVERY_WP   = "GO_TO_DELIVERY_WP"   # intermediate east waypoint
    GO_TO_DELIVERY      = "GO_TO_DELIVERY"
    WAIT_AT_DELIVERY    = "WAIT_AT_DELIVERY"
    GO_TO_CHARGE_WP     = "GO_TO_CHARGE_WP"     # intermediate south waypoint
    RETURN_TO_CHARGE    = "RETURN_TO_CHARGE"
    DONE                = "DONE"


PICK_DWELL   = 2.0
DROP_DWELL   = 2.0


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')

        self.goal_pub    = self.create_publisher(PoseStamped, '/goal_pose',      10)
        # BUG FIX #3 (mission side): publish on /replan_request to nudge the
        # planner when the timeout fires instead of just logging a warning.
        self.replan_pub  = self.create_publisher(PoseStamped, '/replan_request', 10)

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.path_sub = self.create_subscription(
            Path, '/planned_path', self.path_callback, 10)

        self.robot_x   = 0.0
        self.robot_y   = 0.0
        self.dwell_end = None
        self.leg_sent_t     = None
        self.path_received  = False

        # BUG FIX #4: per-leg state to avoid repeating the timeout warning and
        # to space out replan retries.
        self._timeout_warned  = False
        self._last_retry_t    = None

        # Current goal coordinates — needed for replan retries.
        self._current_goal    = None

        self.state = MissionState.IDLE
        self.create_timer(0.1, self.tick)

        self._start_timer = None
        self.get_logger().info('Mission Manager ready. Starting mission in 1 s…')
        self._start_timer = self.create_timer(1.0, self._start_mission)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def odom_callback(self, msg):
        # Gazebo 8 DiffDrive: odom origin = spawn point, add offset for world coords
        self.robot_x = msg.pose.pose.position.x + SPAWN_X
        self.robot_y = msg.pose.pose.position.y + SPAWN_Y

    def path_callback(self, _msg):
        self.path_received = True

    # ── One-shot mission start ────────────────────────────────────────────────

    def _start_mission(self):
        self._start_timer.cancel()
        if self.state != MissionState.IDLE:
            return
        self.get_logger().info('=== Mission START ===')
        self.state = MissionState.GO_TO_SHELF
        self._send_goal(SHELF_1_APPROACH)

    # ── Goal publisher ────────────────────────────────────────────────────────

    def _send_goal(self, xy):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.pose.position.x = xy[0]
        msg.pose.position.y = xy[1]
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)

        now = self.get_clock().now().nanoseconds * 1e-9
        self.leg_sent_t       = now
        self.path_received    = False
        self._timeout_warned  = False   # reset for new leg
        self._last_retry_t    = None
        self._current_goal    = xy
        self.get_logger().info(
            f'[{self.state.value}] Goal sent → ({xy[0]:.2f}, {xy[1]:.2f})')

    # ── Distance helper ───────────────────────────────────────────────────────

    def _at(self, xy):
        return math.hypot(xy[0] - self.robot_x, xy[1] - self.robot_y) < GOAL_TOLERANCE

    # ── Timeout watchdog ──────────────────────────────────────────────────────

    def _check_timeout(self, label):
        """Warn once when path hasn't arrived after REPLAN_TIMEOUT, then
        publish a /replan_request every REPLAN_RETRY_INTERVAL seconds."""
        if self.leg_sent_t is None or self.path_received:
            return

        now     = self.get_clock().now().nanoseconds * 1e-9
        elapsed = now - self.leg_sent_t

        if elapsed < REPLAN_TIMEOUT:
            return

        # BUG FIX #4: warn exactly once per leg instead of every 0.1 s tick.
        if not self._timeout_warned:
            self.get_logger().warn(
                f'[{label}] No path received after {REPLAN_TIMEOUT:.0f} s — '
                f'requesting replan.')
            self._timeout_warned = True
            self._last_retry_t   = now

        # BUG FIX #3: actively request a replan instead of just warning.
        if self._last_retry_t is not None and \
                (now - self._last_retry_t) >= REPLAN_RETRY_INTERVAL and \
                self._current_goal is not None:
            self.get_logger().info(f'[{label}] Sending replan request…')
            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.pose.position.x = self._current_goal[0]
            msg.pose.position.y = self._current_goal[1]
            msg.pose.orientation.w = 1.0
            self.replan_pub.publish(msg)
            self._last_retry_t = now

    # ── State machine ─────────────────────────────────────────────────────────

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        if self.state == MissionState.IDLE:
            pass

        elif self.state == MissionState.GO_TO_SHELF:
            self._check_timeout('GO_TO_SHELF')
            if self._at(SHELF_1_APPROACH):
                self.get_logger().info('Reached Shelf 1 — picking up item…')
                self.dwell_end = now + PICK_DWELL
                self.state = MissionState.WAIT_AT_SHELF

        elif self.state == MissionState.WAIT_AT_SHELF:
            if now >= self.dwell_end:
                self.get_logger().info('Pick-up complete — heading to east waypoint first.')
                self.state = MissionState.GO_TO_DELIVERY_WP
                self._send_goal(DELIVERY_EAST_WP)

        elif self.state == MissionState.GO_TO_DELIVERY_WP:
            self._check_timeout('GO_TO_DELIVERY_WP')
            if self._at(DELIVERY_EAST_WP):
                self.get_logger().info('East waypoint reached — heading to delivery station.')
                self.state = MissionState.GO_TO_DELIVERY
                self._send_goal(DELIVERY_STATION)

        elif self.state == MissionState.GO_TO_DELIVERY:
            self._check_timeout('GO_TO_DELIVERY')
            if self._at(DELIVERY_STATION):
                self.get_logger().info('Reached delivery station — dropping off item…')
                self.dwell_end = now + DROP_DWELL
                self.state = MissionState.WAIT_AT_DELIVERY

        elif self.state == MissionState.WAIT_AT_DELIVERY:
            if now >= self.dwell_end:
                self.get_logger().info('Drop-off complete — heading to south corridor first.')
                self.state = MissionState.GO_TO_CHARGE_WP
                self._send_goal(CHARGE_APPROACH_WP)

        elif self.state == MissionState.GO_TO_CHARGE_WP:
            self._check_timeout('GO_TO_CHARGE_WP')
            if self._at(CHARGE_APPROACH_WP):
                self.get_logger().info('South corridor reached — returning to charging station.')
                self.state = MissionState.RETURN_TO_CHARGE
                self._send_goal(CHARGING_STATION)

        elif self.state == MissionState.RETURN_TO_CHARGE:
            self._check_timeout('RETURN_TO_CHARGE')
            if self._at(CHARGING_STATION):
                self.get_logger().info('=== Mission COMPLETE — robot at charging station. ===')
                self.state = MissionState.DONE

        elif self.state == MissionState.DONE:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()