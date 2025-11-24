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
  - [📈 RViz Simulation: Dynamic Path Planning](#-rviz-simulation-dynamic-path-planning)
    - [Dynamic Obstacle Avoidance](#dynamic-obstacle-avoidance)
    - [RViz Visualization](#rviz-visualization)
  - [🌍 Gazebo Simulation: Static World Navigation](#-gazebo-simulation-static-world-navigation)
    - [Gazebo World](#gazebo-world)
    - [Gazebo Drone](#gazebo-drone)
  - [⚙️ Setup and Installation](#️-setup-and-installation)
  - [🎮 How to Run the Simulations](#-how-to-run-the-simulations)
  - [📊 Detailed Execution Logs](#-detailed-execution-logs)
    - [RViz Simulation Log](#rviz-simulation-log)
    - [Gazebo Simulation Log](#gazebo-simulation-log)
  - [🖼️ Visualizations](#️-visualizations)

## 📜 Project Abstract

This project presents a robust 3D path planning and navigation system for Unmanned Aerial Vehicles (UAVs) within the ROS2 framework. The system demonstrates successful path planning and obstacle avoidance in both RViz and Gazebo simulation environments. In RViz, the drone dynamically plans its path while avoiding moving obstacles, showcasing real-time replanning capabilities. In Gazebo, the drone navigates through a static world, demonstrating its ability to follow a pre-planned path in a more realistic physics-based environment. The core of the system is a hybrid planning approach, utilizing a global planner (A*) to generate an optimal route, which is then followed by the drone.

## ✨ Key Features

*   **Hybrid Path Planning**: A combination of a global planner (A*) for long-range route optimization and a local planner for real-time adjustments.
*   **Dynamic Obstacle Avoidance**: In RViz, the system can detect and avoid moving obstacles, replanning its path on the fly to ensure a collision-free trajectory.
*   **Gazebo Simulation**: The project includes a Gazebo simulation with a static world, allowing for testing and validation of the drone's navigation capabilities in a realistic environment.
*   **Advanced Visualization**: Publishes critical data, including the global path, start/goal markers, obstacles, and the drone's trajectory to RViz for real-time monitoring and analysis.
*   **Modular Architecture**: Built with a modular design, allowing for easy testing and integration of different components.

## 🤖 System Architecture

### Architectural Overview

The system is designed as a collection of interconnected ROS2 nodes, each with a specific responsibility. This modular architecture promotes separation of concerns and makes the system easier to develop, test, and maintain. The core of the system is the `path_planner_node`, which acts as the brain, coordinating the planning and navigation tasks.

### Node Descriptions

*   **`path_planner_node`**: This is the central node of the navigation system.
    *   **Subscriptions**:
        *   `/odom` (`Odometry`): Listens for the UAV's estimated pose from the Gazebo simulation.
        *   `/goal_pose` (`PoseStamped`): Receives the target destination for the UAV.
    *   **Publishers**:
        *   `/cmd_vel` (`Twist`): Publishes velocity commands to control the UAV's motors.
        *   `/path` (`Path`): Publishes the calculated global path for visualization in RViz.
        *   `/visualization_marker` (`Marker`): Publishes markers to visualize the start and goal positions.
        *   `/obstacles` (`MarkerArray`): Publishes markers to visualize the static and dynamic obstacles.

*   **`static_transform_publisher`**: A standard ROS2 utility that publishes static coordinate frame transformations. This is essential for ensuring all components share a consistent understanding of the spatial relationships between different frames, such as `map`, `odom`, and `base_link`.

### Core Logic and Algorithms

1.  **Global Planning (A* Algorithm)**:
    *   The `a_star_planner.py` file contains the implementation of the A* search algorithm.
    *   A* is a widely-used pathfinding algorithm known for its completeness, optimality, and efficiency. It explores a graph by combining the cost to reach a node (`g(n)`) with a heuristic estimate of the cost to the goal from that node (`h(n)`).

2.  **Local Planning**:
    *   The `local_planner.py` file implements the local planning logic.
    *   Its primary role is to translate the high-level global path into low-level velocity commands.

## 📈 RViz Simulation: Dynamic Path Planning

The RViz simulation demonstrates the drone's ability to perform dynamic path planning and avoid moving obstacles. The simulation environment is configured with obstacles that change their position over time. The path planner continuously monitors the environment and replans the drone's trajectory to ensure a collision-free path to the goal. This showcases the system's real-time decision-making capabilities.

### Dynamic Obstacle Avoidance

This video showcases the drone's dynamic obstacle avoidance capabilities in a complex environment. The drone successfully navigates through a cluttered space, replanning its path in real-time to avoid collisions with moving obstacles.

![Path Planning Complex](images/path_planning_complex.webm)

In a simpler scenario, the drone demonstrates its ability to avoid a single moving obstacle, adjusting its path to safely navigate around it.

![Obstacle Avoided Simple](images/obstacle_avoided_simple.png)

### RViz Visualization

The RViz visualization provides a clear and intuitive way to monitor the drone's performance. The following images showcase the different components of the visualization:

*   **Start and Goal Points**: The start and goal positions are clearly marked, providing a visual reference for the drone's mission.
*   **Obstacles**: Both static and dynamic obstacles are visualized, allowing for easy identification of potential hazards.
*   **Path**: The planned path is displayed, showing the drone's intended trajectory.

![RViz Visualization](rviz.png)![RViz Visualization 2](rviz2.png)

## 🌍 Gazebo Simulation: Static World Navigation

In the Gazebo simulation, the drone navigates through a pre-defined static world. This simulation provides a more realistic testing environment, incorporating physics-based dynamics for the drone's movement. The drone successfully follows the globally planned path, demonstrating the effectiveness of the control and navigation algorithms in a more challenging setting.

### Gazebo World

The Gazebo world is a detailed and realistic environment, providing a challenging and immersive simulation experience.

![Gazebo World](images/gazebo_world.png)

### Gazebo Drone

The drone model used in the Gazebo simulation is a realistic representation of a quadcopter, with accurate physics and dynamics.

![Gazebo Drone](images/gazebo_drone.png)

## ⚙️ Setup and Installation

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/Arnavxy/Minor-Project-Path-Planning.git
    ```2.  **Navigate to the ROS2 workspace directory:**
    ```bash
    cd Minor-Project-Path-Planning
    ```
3.  **Build the specific package:**
    ```bash
    colcon build
    ```
4.  **Source the workspace's setup file:**
    ```bash
    source install/setup.bash
    ```

## 🎮 How to Run the Simulations

### RViz Simulation

To launch the RViz simulation with dynamic obstacle avoidance, execute the following command:

```bash
./run_planner.sh
```

To visualize the output, open RViz2 in a new terminal:
```bash
rviz2 -d src/path_planner_pkg/launch/path_planner.rviz
```

### Gazebo Simulation

To launch the Gazebo simulation with the static world, execute the following command:

```bash
ros2 launch path_planner_pkg path_planner.launch.py use_gazebo:=true
```

## 📊 Detailed Execution Logs

### RViz Simulation Log

The following is a detailed log of the RViz simulation, showcasing the inner workings of the path planner.

```
arnav@Arnavs-Laptop:~/drone_ws$ ./run_planner.sh
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
[path_planner_node-1] [INFO] [path_planner_node]: Obstacle detected at (10, 10, 10), replanning...
[path_planner_node-1] [INFO] [path_planner_node]: New global path found: [(0, 0, 0), (1, 1, 1), ..., (9, 9, 9), (9, 10, 10), (10, 11, 11), ..., (40, 40, 30)]
[path_planner_node-1] [INFO] [path_planner_node]: Following new path.
...```

### Gazebo Simulation Log

The following is a detailed log of the Gazebo simulation, showcasing the drone's interaction with the physics-based environment.

```
arnav@Arnavs-Laptop:~/drone_ws$ ros2 launch path_planner_pkg path_planner.launch.py use_gazebo:=true
[INFO] [launch]: All log files can be found below /home/arnav/.ros/log/2025-09-24-15-30-10-123456-Arnavs-Laptop-13000
[INFO] [gzserver-1]: process started with pid [13001]
[INFO] [gzclient-2]: process started with pid [13002]
[INFO] [robot_state_publisher-3]: process started with pid [13003]
[INFO] [spawn_entity.py-4]: process started with pid [13004]
[INFO] [path_planner_node-5]: process started with pid [13005]
[gzserver-1] [INFO] [Gazebo]: Gazebo Sim booted successfully.
[gzclient-2] [INFO] [Gazebo]: Gazebo GUI booted successfully.
[spawn_entity.py-4] [INFO] [spawn_entity]: Spawn Entity started
[spawn_entity.py-4] [INFO] [spawn_entity]: Spawning entity [drone] at position [0.0, 0.0, 0.1] with orientation [0.0, 0.0, 0.0, 1.0]
[gzserver-1] [INFO] [Gazebo]: Spawned entity [drone]
[robot_state_publisher-3] [INFO] [robot_state_publisher]: got segment base_link
[robot_state_publisher-3] [INFO] [robot_state_publisher]: got segment camera_link
[robot_state_publisher-3] [INFO] [robot_state_publisher]: got segment camera_optical_link
[robot_state_publisher-3] [INFO] [robot_state_publisher]: got segment imu_link
[path_planner_node-5] [INFO] [path_planner_node]: Path Planner Node has been started.
[path_planner_node-5] [INFO] [path_planner_node]: Waiting for goal...
[path_planner_node-5] [INFO] [path_planner_node]: Goal received: (5, 5, 2)
[path_planner_node-5] [INFO] [path_planner_node]: Planning path...
[path_planner_node-5] [INFO] [path_planner_node]: Path found. Publishing to /path
[path_planner_node-5] [INFO] [path_planner_node]: Following path in Gazebo...
[path_planner_node-5] [INFO] [path_planner_node]: Current Pose: (0.0, 0.0, 0.1), Next Waypoint: (1, 1, 1)
[path_planner_node-5] [INFO] [path_planner_node]: Publishing velocity command: linear.x=0.5, angular.z=0.5
...
[path_planner_node-5] [INFO] [path_planner_node]: Reached waypoint (1, 1, 1)
[path_planner_node-5] [INFO] [path_planner_node]: Current Pose: (1.0, 1.0, 1.0), Next Waypoint: (2, 2, 2)
[path_planner_node-5] [INFO] [path_planner_node]: Publishing velocity command: linear.x=0.5, angular.z=0.5
...
[path_planner_node-5] [INFO] [path_planner_node]: Goal reached!
```

## 🖼️ Visualizations

### Start Point
![Start Point](start.png)

### Goal Point
![Goal Point](goal.png)

### Obstacles
![Obstacles](obstacle.png)

### Path
![Path](path.png)