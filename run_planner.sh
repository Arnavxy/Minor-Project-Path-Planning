#!/bin/bash
set -e

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source ~/px4_ws/install/setup.bash

# Build the package
echo "--- Building path_planner_pkg ---"
colcon build

# Source the local workspace
source install/setup.bash

# Launch the path planner
echo "--- Launching path_planner ---"
ros2 launch path_planner_pkg path_planner.launch.py