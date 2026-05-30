Multi-Robot Warehouse Automation With Priority-Based Scheduling


source /opt/ros/jazzy/setup.bash

cd ~/intelligent_robotics
source install/setup.bash
colcon build --packages-select warehouse_robot

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




# Terminal 1 — world
cd ~/intelligent_robotics && source install/setup.bash
ros2 launch warehouse_world world.launch.py

# Terminal 2 — robot
cd ~/intelligent_robotics && source install/setup.bash
ros2 launch warehouse_robot spawn_robot.launch.py

# Terminal 3 — path planner
cd ~/intelligent_robotics && source install/setup.bash
ros2 run warehouse_robot path_planner

# Terminal 4 — robot controller
cd ~/intelligent_robotics && source install/setup.bash
ros2 run warehouse_robot robot_controller

# Terminal 5 — RViz
cd ~/intelligent_robotics && source install/setup.bash
rviz2
# Click "2D Nav Goal" → robot moves → logs "Goal reached!"