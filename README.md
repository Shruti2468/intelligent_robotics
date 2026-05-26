Multi-Robot Warehouse Automation With Priority-Based Scheduling



# Terminal 1
ros2 launch warehouse_world world.launch.py

# Terminal 2
ros2 launch warehouse_robot spawn_robot.launch.py

# Terminal 3
ros2 run warehouse_robot path_planner

# Terminal 4
ros2 run warehouse_robot robot_controller

# Terminal 5
rviz2   # click "2D Nav Goal" on the map → robot moves