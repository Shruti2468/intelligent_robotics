#!/usr/bin/env python3
"""
send_task.py  —  CLI helper to dispatch a pick task

Usage:
  python3 send_task.py <shelf_id> [parcel_id] [robot_id]

Examples:
  python3 send_task.py shelf_1_1
  python3 send_task.py shelf_2_3 my_parcel_99
  python3 send_task.py shelf_3_4 box_42 robot_1

Valid shelf IDs (from your world file):
  Row 1 (y=6):  shelf_1_1  shelf_1_2  shelf_1_3  shelf_1_4
  Row 2 (y=0):  shelf_2_1  shelf_2_2  shelf_2_3  shelf_2_4
  Row 3 (y=-6): shelf_3_1  shelf_3_2  shelf_3_3  shelf_3_4
"""

import sys, json, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def main():
    rclpy.init()
    node = Node('task_sender_cli')
    pub  = node.create_publisher(String, '/request_task', 10)
    time.sleep(0.6)   # let publisher connect

    shelf_id  = sys.argv[1] if len(sys.argv) > 1 else 'shelf_1_1'
    parcel_id = sys.argv[2] if len(sys.argv) > 2 else None
    robot_id  = sys.argv[3] if len(sys.argv) > 3 else None

    payload = {'shelf_id': shelf_id}
    if parcel_id: payload['parcel_id'] = parcel_id
    if robot_id:  payload['robot_id']  = robot_id

    pub.publish(String(data=json.dumps(payload)))
    node.get_logger().info(f'Published task: {payload}')
    time.sleep(0.3)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
