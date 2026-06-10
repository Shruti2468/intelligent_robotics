"""odom_tf_publisher.py
Reads /odom and publishes the odom→base_footprint TF transform.
Needed because Gazebo 8's /tf bridge publishes transforms in world
frame rather than relative to the odom frame origin.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

SPAWN_X = -10.0
SPAWN_Y = -10.0


class OdomTFPublisher(Node):
    def __init__(self):
        super().__init__('odom_tf_publisher')
        self.br = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.get_logger().info('Odom TF publisher ready.')

    def odom_cb(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        # /odom gives spawn-relative position (0,0 at spawn)
        # odom frame origin = spawn point, so this is correct as-is
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()