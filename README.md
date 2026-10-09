**Summary**

This project implements exploration and target search for the TurtleBot3 Waffle Pi using ROS2 and Gazebo. The robot explores an unknown environment by finding frontier cells and navigating towards unexplored aeras using Nav2. While exploring, the robot uses its front-facing camera to detect ArUco markers. The environment is mapped using LiDAR-based SLAM.

The repository contains four packages:
- exploration: Detects frontiers, groups them as clusters and finds waypoint for best cluster.

- mapping_pkg: Camera processing and detects ArUco markers.

- mapping_interfaces: ROS 2 interfaces.

- metr4202_test_worlds: Different Gazebo worlds for testing.


**Running the System:**

1. Build the workspace
```
source /opt/ros/humble/setup.bash
cd ~/turtlebot3_ws/src/4202_turtlebot
colcon build --symlink-install
source install/setup.bash
```
2. Launch the full system
```
ros2 launch team10_bringup full_system.launch.py
```

This launch file starts the system components configured in full_system.launch.py.
