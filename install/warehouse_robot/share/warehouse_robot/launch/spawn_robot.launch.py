from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('warehouse_robot')
    robot_file = os.path.join(pkg_path, 'urdf', 'warehouse_amr.urdf')

    # Read URDF as string for robot_state_publisher
    with open(robot_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([

        # 1. Robot state publisher — correct way
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],  # ← parameter not argument
            output='screen'
        ),

        # 2. Spawn in Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'warehouse_amr',
                '-file', robot_file,
                '-x', '0', '-y', '0', '-z', '0.12'
            ],
            output='screen'
        ),

        # 3. Bridge Gazebo <-> ROS 2
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',

            arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
            output='screen'
        ),

    ])