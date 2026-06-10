Multi-Robot Warehouse Automation With Priority-Based Scheduling


source /opt/ros/jazzy/setup.bash

cd ~/intelligent_robotics
source install/setup.bash
colcon build --symlink-install



colcon build --packages-select warehouse_robot



source install/setup.bash
colcon build --symlink-install
# Terminal 1
ros2 launch warehouse_world world.launch.py

# Terminal 2
ros2 launch warehouse_robot spawn_robot.launch.py

# Terminal 3
ros2 run warehouse_robot path_planner

# Terminal 4
ros2 run warehouse_robot robot_controller

# Terminal 5

ros2 run warehouse_robot mission_manager

# Terminal 6

ros2 run warehouse_robot static_map_publisher

# Terminal 7
rviz2 -d ~/intelligent_robotics/src/warehouse_robot/config/warehouse.rviz

