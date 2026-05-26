from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    pkg_path = get_package_share_directory('warehouse_robot')

    robot_file = os.path.join(
        pkg_path,
        'urdf',
        'warehouse_amr.urdf'
    )

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            arguments=[robot_file],
            output='screen'
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'warehouse_amr',
                '-file', robot_file,
                '-x', '0',
                '-y', '0',
                '-z', '0.12'
            ],
            output='screen'
        )

    ])