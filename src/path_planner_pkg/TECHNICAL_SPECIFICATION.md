# Technical Specification for Path Planning Node

This document outlines the technical specification for the ROS 2 path planning node.

## 1. Node Components

### Subscribers
-   **/local_position/pose**: `geometry_msgs/PoseStamped` - To get the current position and orientation of the vehicle.
-   **/mission/waypoints**: `custom_interfaces/Path` - To receive the mission path to be followed.

### Publishers
-   **/offboard_control_mode**: `px4_msgs/msg/OffboardControlMode` - To set the offboard control mode for the vehicle.
-   **/trajectory_setpoint**: `px4_msgs/msg/TrajectorySetpoint` - To send velocity commands to the vehicle.
-   **/vehicle_command**: `px4_msgs/msg/VehicleCommand` - To send commands like arm, disarm, and land to the vehicle.

### Timers
-   **Control Loop Timer**: A timer that runs at a high frequency (e.g., 10Hz) to execute the main control logic, check the vehicle's state, and publish velocity commands.

## 2. Data Structures

-   **Path**: A list or vector of waypoints. Each waypoint will be a data structure containing:
    -   `x`: float
    -   `y`: float
    -   `z`: float
-   **Vehicle State**: A structure to hold the current state of the vehicle, including:
    -   `position`: A structure with `x`, `y`, and `z` coordinates.
    -   `orientation`: A structure representing the vehicle's orientation (e.g., quaternion).
    -   `is_armed`: boolean
    -   `is_home_set`: boolean
    -   `home_position`: A structure with `x`, `y`, and `z` coordinates for the home position.

## 3. Core Logic

### Path Planning Logic
1.  The node receives a path (a series of waypoints) from the `/mission/waypoints` topic.
2.  The node arms the vehicle and takes off if it is not already in the air.
3.  The node iterates through the waypoints in the path.
4.  For each waypoint, the node calculates the required velocity vector to move the vehicle from its current position to the target waypoint.
5.  The node publishes the velocity commands to the `/trajectory_setpoint` topic.
6.  The node continuously checks if the vehicle has reached the current target waypoint within a certain tolerance.
7.  Once the current waypoint is reached, the node moves to the next waypoint in the path.
8.  After the last waypoint is reached, the node initiates the return-to-home logic.

### Return-to-Home (RTH) Logic
1.  The home position is recorded when the vehicle is first armed.
2.  Once the primary mission path is complete, the RTH logic is triggered.
3.  The node sets the home position as the final waypoint.
4.  The node calculates the velocity vector to fly back to the home position.
5.  Upon reaching the home position, the node commands the vehicle to land.