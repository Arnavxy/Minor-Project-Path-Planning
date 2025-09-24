# Autonomous 3D Path Planning and Adaptive Return Navigation for UAVs

This ROS2 package provides a sophisticated high-level 3D path planning and navigation system for Unmanned Aerial Vehicles (UAVs). It is engineered to function reliably using pose data from external localization modules, enabling robust autonomous operation, particularly in environments where GPS is unavailable or unreliable.

## 📖 Table of Contents
- [Autonomous 3D Path Planning and Adaptive Return Navigation for UAVs](#autonomous-3d-path-planning-and-adaptive-return-navigation-for-uavs)
  - [📖 Table of Contents](#-table-of-contents)
  - [📜 Project Abstract](#-project-abstract)
  - [✨ Key Features](#-key-features)
  - [🤖 System Architecture](#-system-architecture)
    - [Architectural Overview](#architectural-overview)
    - [Node Descriptions](#node-descriptions)
    - [Core Logic and Algorithms](#core-logic-and-algorithms)
  - [📈 Current Status and Accomplishments](#-current-status-and-accomplishments)
  - [🚀 Future Work and Development Roadmap](#-future-work-and-development-roadmap)
  - [⚙️ Setup and Installation](#️-setup-and-installation)
  - [🎮 How to Run the Simulation](#-how-to-run-the-simulation)
  - [📊 Build and Execution Logs](#-build-and-execution-logs)
  - [📊 Visualization and Analysis](#-visualization-and-analysis)
    - [Start Point](#start-point)
    - [Goal Point](#goal-point)
    - [Obstacles](#obstacles)
    - [Path](#path)
    - [RViz Visualization](#rviz-visualization)

## 📜 Project Abstract

Effective autonomous operation for a UAV requires not only knowing its position but also making intelligent decisions about its movement. This project focuses on the design and implementation of a high-level 3D path planning and navigation system that operates reliably using pose data from localization modules. The system employs a hybrid planning approach, utilizing a global planner like A* on a pre-existing map to generate an initial, optimal route to a target. During execution, this global plan will be augmented by a local planner that uses the dynamically generated SLAM map for real-time obstacle avoidance and dynamic replanning. A key innovation of this project is the development of an adaptive return-to-home logic that reuses the map and path memory from the outbound journey to ensure a safe and efficient return trip. The system will also incorporate critical fail-safe strategies, including low-battery protocols and obstacle-induced hovering. The final deliverable is a comprehensive ROS2 navigation node that consumes pose estimates and produces velocity commands to safely and effectively guide the UAV through its mission.

## ✨ Key Features

*   **Hybrid Path Planning**: A combination of a global planner (A*) for long-range route optimization and a local planner for real-time adjustments and obstacle avoidance.
*   **Global Path Optimization**: Employs the A* search algorithm to compute the shortest, most efficient path on a 3D grid map, considering static obstacles.
*   **Dynamic Local Navigation**: The local planner is responsible for generating feasible velocity commands (`cmd_vel`) that adhere to the global path while providing a framework for future real-time obstacle avoidance.
*   **Adaptive Return-to-Home**: A crucial safety feature where the UAV continuously records its trajectory. This allows it to autonomously retrace its steps and return to the launch point.
*   **Robust Fail-Safe Mechanisms**: The system includes a low-battery protocol that automatically suspends the current mission and triggers the return-to-home functionality, ensuring the vehicle's safe recovery.
*   **Advanced Visualization**: Publishes critical data, including the global path, start/goal markers, obstacles, and a 3D scoring grid to RViz for real-time monitoring and debugging.
*   **Modular and Testable**: The system is built with modular components, including a `mock_pose_publisher` that allows for thorough testing of the planning logic in a controlled simulation environment without requiring physical hardware.

## 🤖 System Architecture

### Architectural Overview

The system is designed as a collection of interconnected ROS2 nodes, each with a specific responsibility. This modular architecture promotes separation of concerns and makes the system easier to develop, test, and maintain. The core of the system is the `path_planner_node`, which acts as the brain, coordinating the planning and navigation tasks.

### Node Descriptions

*   **`path_planner_node`**: This is the central node of the navigation system.
    *   **Subscriptions**:
        *   `/amcl_pose` (`PoseWithCovarianceStamped`): Listens for the UAV's estimated pose from a localization system (e.g., AMCL). In the current prototype, this is provided by the `mock_pose_publisher`.
        *   `/goal_pose` (`PoseStamped`): Receives the target destination for the UAV.
        *   `/battery_state` (`BatteryState`): Monitors the battery level to trigger fail-safe behaviors.
    *   **Publishers**:
        *   `/cmd_vel` (`Twist`): Publishes velocity commands to control the UAV's motors.
        *   `/path` (`Path`): Publishes the calculated global path for visualization in RViz.
        *   `/visualization_marker` (`Marker`): Publishes markers to visualize the start and goal positions.
        *   `/obstacles` (`MarkerArray`): Publishes markers to visualize the static obstacles.
        *   `/scoring_grid` (`MarkerArray`): Publishes markers to visualize the 3D scoring grid.
        *   PX4-related topics (`/offboard_control_mode`, `/trajectory_setpoint`, `/vehicle_command`): These are placeholders for future integration with the PX4 flight controller.

*   **`mock_pose_publisher`**: A development tool used to simulate the UAV's movement and provide necessary inputs for testing the `path_planner_node`.
    *   It simulates a UAV moving along a predefined 3D trajectory.
    *   After a short delay, it publishes a goal pose to `/goal_pose` to initiate the path planning process. This simulates a mission command.

*   **`static_transform_publisher`**: A standard ROS2 utility that publishes static coordinate frame transformations. This is essential for ensuring all components share a consistent understanding of the spatial relationships between different frames, such as `map`, `odom`, and `base_link`.

### Core Logic and Algorithms

1.  **Global Planning (A* Algorithm)**:
    *   The `a_star_planner.py` file contains the implementation of the A* search algorithm.
    *   A* is a widely-used pathfinding algorithm known for its completeness, optimality, and efficiency. It explores a graph by combining the cost to reach a node (`g(n)`) with a heuristic estimate of the cost to the goal from that node (`h(n)`).
    *   The implementation uses a `Grid` class to represent the 3D environment and a `Node` class to represent states in the search space.

2.  **Local Planning**:
    *   The `local_planner.py` file implements the local planning logic.
    *   Its primary role is to translate the high-level global path into low-level velocity commands.
    *   It evaluates a set of possible linear and angular velocities and scores them based on criteria like path following and obstacle avoidance. The velocity pair with the best score is chosen.
    *   Currently, the obstacle avoidance is based on the static map, but this component is designed to be extended with real-time sensor data.

3.  **Path Recording and Return-to-Home**:
    *   The `path_planner_node` maintains a list of poses, `recorded_path`.
    *   When the `activate_return_to_home` function is called (e.g., due to low battery), the `global_path` is replaced with the reversed `recorded_path`, guiding the UAV back to its starting point.

## 📈 Current Status and Accomplishments

The project has successfully implemented the foundational components of the 3D path planning system. The current prototype robustly demonstrates:

*   **End-to-End 3D Global Path Generation**: The system can successfully receive a 3D goal, trigger the A* planner, and generate a valid, obstacle-free 3D path from a start to a goal position.
*   **Functional Local Planner**: The local planner is capable of interpreting the global path and generating appropriate velocity commands to follow it in a simulated environment.
*   **3D Scoring Grid**: The system generates and visualizes a 3D scoring grid around the drone, providing a rich representation of the local environment for decision-making.
*   **Path Memory Implementation**: The system correctly records the 3D trajectory of the UAV, which is the basis for the return-to-home feature.
*   **Implemented Low-Battery Failsafe**: The logic to monitor battery status and trigger the return-to-home functionality is in place and functional.
*   **Complete Simulation and Visualization Environment**: The project can be launched with a single command, and the results can be clearly visualized in RViz, which is crucial for debugging and presentations.

## 🚀 Future Work and Development Roadmap

The following features represent the next steps in the development of this project:

*   **Dynamic Obstacle Avoidance**: This is the highest priority. The plan is to integrate real-time sensor data from a LiDAR or depth camera. The local planner will be upgraded to use this data to perform dynamic replanning and avoid unforeseen obstacles.
*   **Full PX4 Integration**: Transition from publishing `/cmd_vel` to publishing `TrajectorySetpoint` messages for direct control of a PX4-based UAV in offboard mode. This will involve a more sophisticated state machine to handle arming, taking off, and landing.
*   **Live SLAM Integration**: The current static map will be replaced with a map that is generated in real-time by a SLAM (Simultaneous Localization and Mapping) algorithm, such as ORB-SLAM3 or Cartographer. This will enable the UAV to navigate in completely unknown environments.
*   **Enhanced Local Planner**: The local planner will be improved to generate smoother, more kinematically realistic trajectories using techniques like Dynamic Window Approach (DWA) or Model Predictive Control (MPC).
*   **Refined Adaptive Return Navigation**: The return-to-home feature will be enhanced to reuse the map data gathered during the outbound journey, allowing it to find safer and more efficient return paths if the environment has changed.
*   **Advanced Failsafe Behaviors**: Implement more sophisticated failsafe strategies, such as hovering and waiting for instructions if an obstacle completely blocks the path, or landing safely if the return path is also blocked.

## ⚙️ Setup and Installation

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/Arnavxy/Minor-Project-Path-Planning.git
    ```
2.  **Navigate to the ROS2 workspace directory:**
    ```bash
    cd Minor-Project-Path-Planning/drone_ws
    ```
3.  **Build the specific package:**
    ```bash
    colcon build --packages-select path_planner_pkg
    ```
4.  **Source the workspace's setup file:**
    ```bash
    source install/setup.bash
    ```

## 🎮 How to Run the Simulation

To launch the entire simulation, including the path planner and the mock pose publisher, execute the following command in your terminal:

```bash
ros2 launch path_planner_pkg path_planner.launch.py
```

To visualize the output, open RViz2 in a new terminal:
```bash
rviz2
```
In RViz2, add the following topics to view the planner's output:
*   Set the **Fixed Frame** to `map`.
*   Add a `Pose` display and subscribe to the `/amcl_pose` topic.
*   Add a `Path` display and subscribe to the `/path` topic.
*   Add a `MarkerArray` display and subscribe to the `/obstacles` topic.
*   Add a `MarkerArray` display and subscribe to the `/scoring_grid` topic.
*   Add a `Marker` display and subscribe to the `/visualization_marker` topic.

## 📊 Build and Execution Logs

The following logs demonstrate the successful build and execution of the path planning system.

### Build Log

```
arnav@Arnavs-Laptop:~/drone_ws$ colcon build
Starting >>> px4_msgs
Finished <<< px4_msgs [2.50s]                    
Starting >>> path_planner_pkg
Finished <<< path_planner_pkg [1.73s]          

Summary: 2 packages finished [4.39s]
```
This log shows that the `colcon build` command successfully compiled both the `px4_msgs` and `path_planner_pkg` packages.

### Execution Log

```
arnav@Arnavs-Laptop:~/drone_ws$ . install/setup.bash && ros2 launch path_planner_pkg path_planner.launch.py
[INFO] [launch]: All log files can be found below /home/arnav/.ros/log/2025-09-24-14-26-15-588004-Arnavs-Laptop-12109
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [path_planner_node-1]: process started with pid [12113]
[INFO] [static_transform_publisher-2]: process started with pid [12114]
[INFO] [static_transform_publisher-3]: process started with pid [12115]
[static_transform_publisher-2] [WARN] [1758704175.742244832] []: Old-style arguments are deprecated; see --help for new-style arguments
[static_transform_publisher-3] [WARN] [1758704175.742261854] []: Old-style arguments are deprecated; see --help for new-style arguments
[static_transform_publisher-2] [INFO] [1758704175.787729309] [static_transform_publisher]: Spinning until stopped - publishing transform
[static_transform_publisher-2] translation: ('0.000000', '0.000000', '0.000000')
[static_transform_publisher-2] rotation: ('0.000000', '0.000000', '0.000000', '1.000000')
[static_transform_publisher-2] from 'map' to 'base_link'
[static_transform_publisher-3] [INFO] [1758704175.787729409] [static_transform_publisher_map_to_odom]: Spinning until stopped - publishing transform
[static_transform_publisher-3] translation: ('0.000000', '0.000000', '0.000000')
[static_transform_publisher-3] rotation: ('0.000000', '0.000000', '0.000000', '1.000000')
[static_transform_publisher-3] from 'map' to 'odom'
[path_planner_node-1] [INFO] [1758704176.564729061] [path_planner_node]: Path Planner Node has been started.
[INFO] [mock_pose_publisher-4]: process started with pid [12179]
[path_planner_node-1] [INFO] [1758704180.304247492] [path_planner_node]: Goal callback triggered.
[path_planner_node-1] [INFO] [1758704180.304863808] [path_planner_node]: Planning from (0, 0, 0) to (40, 40, 30)
[path_planner_node-1] [INFO] [1758704180.305735827] [path_planner_node]: Published start and end points.
[path_planner_node-1] [INFO] [1758704180.307682729] [path_planner_node]: Global path found: [(0, 0, 0), (1, 1, 1), (2, 2, 2), (3, 3, 3), (4, 4, 4), (5, 5, 5), (6, 6, 6), (7, 7, 7), (8, 8, 8), (9, 9, 9), (10, 10, 10), (11, 11, 11), (12, 12, 12), (13, 13, 13), (14, 14, 14), (15, 15, 15), (16, 16, 16), (17, 17, 17), (18, 18, 18), (19, 19, 19), (20, 20, 20), (21, 21, 21), (22, 22, 22), (23, 23, 23), (24, 24, 24), (25, 25, 25), (26, 26, 26), (27, 27, 27), (28, 28, 28), (29, 29, 29), (30, 30, 30), (31, 31, 30), (32, 32, 30), (33, 33, 30), (34, 34, 30), (35, 35, 30), (36, 36, 30), (37, 37, 30), (38, 38, 30), (39, 39, 30), (40, 40, 30)]
[mock_pose_publisher-4] [INFO] [1758704180.332588202] [mock_pose_publisher]: Published goal pose.
```
This log shows the output of launching the system. Key events include:
- The `path_planner_node`, `mock_pose_publisher`, and `static_transform_publisher` nodes are all started successfully.
- The `path_planner_node` receives a goal, plans a path from (0, 0, 0) to (40, 40, 30), and publishes the resulting path.
- The `mock_pose_publisher` successfully publishes the goal pose.

## 📊 Visualization and Analysis

### Start Point
![Start Point](start.png)

### Goal Point
![Goal Point](goal.png)

### Obstacles
![Obstacles](obstacle.png)

### Path
![Path](path.png)

### RViz Visualization
![RViz Visualization](rviz.png)
![RViz Visualization 2](rviz2.png)