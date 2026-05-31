#!/usr/bin/env python3
"""
task_manager.py
---------------
Drop into  warehouse_robot/warehouse_robot/  alongside robot_controller.py

Central dispatcher for the multi-robot warehouse.

  • Maintains a FIFO queue of pick jobs.
  • Watches each robot's state on  /<robot_id>/robot_status
  • Assigns the next task to the first IDLE robot.
  • Accepts new jobs on  /request_task  (std_msgs/String JSON).

Topic API
─────────
  Subscribe:
    /request_task         std_msgs/String  JSON
      {"shelf_id": "shelf_1_1"}
      {"shelf_id": "shelf_2_3", "parcel_id": "p99", "robot_id": "robot_1"}

    /robot_1/robot_status  std_msgs/String  JSON  (one per managed robot)
    /robot_2/robot_status  …

  Publish:
    /task_assignment      std_msgs/String  JSON
"""

import json
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Default parcel IDs that match the shelf contents shown in the world file
SHELF_DEFAULT_PARCELS = {
    "shelf_1_1": "parcel_1_1", "shelf_1_2": "parcel_1_2",
    "shelf_1_3": "parcel_1_3", "shelf_1_4": "parcel_1_4",
    "shelf_2_1": "parcel_2_1", "shelf_2_2": "parcel_2_2",
    "shelf_2_3": "parcel_2_3", "shelf_2_4": "parcel_2_4",
    "shelf_3_1": "parcel_3_1", "shelf_3_2": "parcel_3_2",
    "shelf_3_3": "parcel_3_3", "shelf_3_4": "parcel_3_4",
}

VALID_SHELVES = set(SHELF_DEFAULT_PARCELS.keys())


class TaskManager(Node):

    def __init__(self):
        super().__init__("task_manager")

        self.declare_parameter("robot_ids", ["robot_1"])
        self.robot_ids: list = self.get_parameter("robot_ids").value

        # Track each robot's latest reported FSM state
        self.robot_states: dict = {r: "IDLE" for r in self.robot_ids}

        # FIFO task queue
        self.task_queue: list = []

        # ── Publishers ───────────────────────────────────────────────
        self._assign_pub = self.create_publisher(String, "/task_assignment", 10)

        # ── Subscribers ──────────────────────────────────────────────
        self.create_subscription(String, "/request_task",
                                 self._on_request_task, 10)

        for rid in self.robot_ids:
            self.create_subscription(
                String, f"/{rid}/robot_status",
                lambda msg, r=rid: self._on_robot_status(msg, r),
                10,
            )

        # 1 Hz dispatch loop
        self.create_timer(1.0, self._dispatch_loop)

        self.get_logger().info(
            f"TaskManager ready. Managing: {self.robot_ids}"
        )

    # ─────────────────────────────────────────────────────────────────
    # Incoming task requests
    # ─────────────────────────────────────────────────────────────────

    def _on_request_task(self, msg: String):
        try:
            req = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("Malformed JSON on /request_task — ignored.")
            return

        shelf_id = req.get("shelf_id", "")
        if shelf_id not in VALID_SHELVES:
            self.get_logger().error(
                f"Unknown shelf '{shelf_id}'. "
                f"Valid: {sorted(VALID_SHELVES)}"
            )
            return

        task = {
            "task_id":  str(uuid.uuid4())[:8],
            "shelf_id": shelf_id,
            "parcel_id": req.get(
                "parcel_id", SHELF_DEFAULT_PARCELS.get(shelf_id, "unknown")
            ),
            "robot_id": req.get("robot_id", None),  # None = auto-assign
        }
        self.task_queue.append(task)
        self.get_logger().info(
            f"Task queued: {task['task_id']} | "
            f"shelf={shelf_id} parcel={task['parcel_id']} "
            f"(queue depth={len(self.task_queue)})"
        )

    # ─────────────────────────────────────────────────────────────────
    # Robot status tracking
    # ─────────────────────────────────────────────────────────────────

    def _on_robot_status(self, msg: String, robot_id: str):
        try:
            s = json.loads(msg.data)
            self.robot_states[robot_id] = s.get("state", "UNKNOWN")
        except json.JSONDecodeError:
            pass

    # ─────────────────────────────────────────────────────────────────
    # Dispatch loop
    # ─────────────────────────────────────────────────────────────────

    def _dispatch_loop(self):
        if not self.task_queue:
            return

        for task in list(self.task_queue):
            preferred = task.get("robot_id")

            if preferred:
                if self.robot_states.get(preferred) == "IDLE":
                    self._assign(task, preferred)
                    self.task_queue.remove(task)
                    return
                # preferred robot not idle yet — leave in queue
            else:
                idle_robots = [
                    r for r, s in self.robot_states.items() if s == "IDLE"
                ]
                if idle_robots:
                    chosen = idle_robots[0]
                    task["robot_id"] = chosen
                    self._assign(task, chosen)
                    self.task_queue.remove(task)
                    return

    def _assign(self, task: dict, robot_id: str):
        # Optimistic state update — avoids double-assigning same robot
        self.robot_states[robot_id] = "NAVIGATING"
        self.get_logger().info(
            f"▶ Assigning {task['task_id']} → {robot_id} "
            f"(shelf={task['shelf_id']} parcel={task['parcel_id']})"
        )
        self._assign_pub.publish(String(data=json.dumps(task)))


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = TaskManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
