from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_path = get_package_share_directory('warehouse_robot')
    robot_file = os.path.join(pkg_path, 'urdf', 'warehouse_amr.urdf')

    with open(robot_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([

        # 1. Static transform: map → odom (needed for RViz fixed frame)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
             name='map_to_odom',
                arguments=[
        '--x', '0', '--y', '0', '--z', '0',
        '--roll', '0', '--pitch', '0', '--yaw', '0',
        '--frame-id', 'map',
        '--child-frame-id', '0dom'
                    ]
            ),

        # 2. Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        # 3. Spawn in Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'warehouse_amr',
                '-file', robot_file,
                '-x', '0',
                '-y', '0',
                '-z', '0.12',
                '-Y', '0.0',
            ],
            output='screen'
        ),

        # 4. Bridge Gazebo <-> ROS 2
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            ],
            output='screen'
        ),

    ])