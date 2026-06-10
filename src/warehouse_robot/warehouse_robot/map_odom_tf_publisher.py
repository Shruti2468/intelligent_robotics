"""map_odom_tf_publisher.py
Publishes a static map→odom transform so RViz/TF knows that the
odom frame origin is at the robot's spawn point (-10, -10) in
world/map frame. Without this, the robot appears at (0,0) on the
map instead of its actual world position.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

SPAWN_X = -10.0
SPAWN_Y = -10.0


class MapOdomTFPublisher(Node):
    def __init__(self):
        super().__init__('map_odom_tf_publisher')
        self.br = StaticTransformBroadcaster(self)
        self._publish_static_tf()
        self.get_logger().info('Static map→odom TF published.')

    def _publish_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        # odom frame origin = spawn point in world/map frame
        t.transform.translation.x = SPAWN_X
        t.transform.translation.y = SPAWN_Y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = MapOdomTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()