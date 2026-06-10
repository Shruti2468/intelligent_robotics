from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('warehouse_robot')
    urdf = os.path.join(pkg, 'urdf', 'warehouse_amr.urdf')
    with open(urdf, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([

        # ── TF ──────────────────────────────────────────────────────
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['--x','-10.0','--y','-10.0','--z','0',
                       '--roll','0','--pitch','0','--yaw','0',
                       '--frame-id','map','--child-frame-id','odom']
        ),
        # base_scan offset matches URDF: lidar_joint (0.25,0,0.115) + scan_joint (0,0,0.04)


        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='base_lidar_mount_tf',
            arguments=['--x','0.25','--y','0','--z','0.115',
                    '--roll','0','--pitch','0','--yaw','0',
                    '--frame-id','base_link','--child-frame-id','lidar_mount']
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='lidar_mount_scan_tf',
            arguments=['--x','0','--y','0','--z','0.04',
                    '--roll','0','--pitch','0','--yaw','0',
                    '--frame-id','lidar_mount','--child-frame-id','base_scan']
        ),

        # ── Robot state publisher ────────────────────────────────────
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        # ── Gazebo spawn ─────────────────────────────────────────────
        Node(
            package='ros_gz_sim', executable='create',
            arguments=['-name','warehouse_amr','-file', urdf,
                       '-x','-10.0','-y','-10.0','-z','0.12','-Y','0.0'],
            output='screen'
        ),

        # ── Gazebo ↔ ROS 2 bridge ─────────────────────────────────────
        # joint_states bridge added so robot_state_publisher gets wheel angles
        # and RViz stops dropping base_scan TF frames
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                # /tf bridge removed: Gazebo 8 publishes TF in world frame,
                # not odom-relative. Our odom_tf_publisher node handles this correctly.
                '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            ],
            output='screen'
        ),

        # ── Our nodes ────────────────────────────────────────────────
        Node(package='warehouse_robot', executable='odom_tf_publisher',    output='screen'),
        Node(package='warehouse_robot', executable='static_map_publisher', output='screen'),
        Node(package='warehouse_robot', executable='path_planner',         output='screen'),
        Node(package='warehouse_robot', executable='robot_controller',     output='screen'),
        Node(package='warehouse_robot', executable='mission_manager',      output='screen'),
    ])