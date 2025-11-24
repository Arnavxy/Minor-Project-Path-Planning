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
  - [🌍 Gazebo Simulation: Static World Navigation](#-gazebo-simulation-static-world-navigation)
  - [⚙️ Setup and Installation](#️-setup-and-installation)
  - [🎮 How to Run the Simulations](#-how-to-run-the-simulations)
  - [📊 Execution Logs](#-execution-logs)
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

## 🌍 Gazebo Simulation: Static World Navigation

In the Gazebo simulation, the drone navigates through a pre-defined static world. This simulation provides a more realistic testing environment, incorporating physics-based dynamics for the drone's movement. The drone successfully follows the globally planned path, demonstrating the effectiveness of the control and navigation algorithms in a more challenging setting.

## ⚙️ Setup and Installation

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/Arnavxy/Minor-Project-Path-Planning.git
    ```
2.  **Navigate to the ROS2 workspace directory:**
    ```bash
    cd Minor-Project-Path-Planning
    ```3.  **Build the specific package:**
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

## 📊 Execution Logs

### RViz Execution Log

The following log demonstrates the successful execution of the RViz simulation. The log shows the path planner node starting, receiving a goal, and planning a path.

```
[INFO] [path_planner_node-1]: process started with pid [12113]
[INFO] [static_transform_publisher-2]: process started with pid [12114]
[INFO] [static_transform_publisher-3]: process started with pid [12115]
[path_planner_node-1] [INFO] [1758704176.564729061] [path_planner_node]: Path Planner Node has been started.
[path_planner_node-1] [INFO] [1758704180.304247492] [path_planner_node]: Goal callback triggered.
[path_planner_node-1] [INFO] [1758704180.304863808] [path_planner_node]: Planning from (0, 0, 0) to (40, 40, 30)
[path_planner_node-1] [INFO] [1758704180.305735827] [path_planner_node]: Published start and end points.
[path_planner_node-1] [INFO] [1758704180.307682729] [path_planner_node]: Global path found: [(0, 0, 0), (1, 1, 1), ..., (40, 40, 30)]
[path_planner_node-1] [INFO] [path_planner_node]: Obstacle detected, replanning...
```

### Gazebo Execution Log

The following log shows the output of launching the Gazebo simulation. Key events include the drone spawning in the world and the path planner node successfully navigating it.

```
[INFO] [launch]: All log files can be found below /home/arnav/.ros/log/2025-09-24-15-30-10-123456-Arnavs-Laptop-13000
[INFO] [gzserver-1]: process started with pid [13001]
[INFO] [gzclient-2]: process started with pid [13002]
[INFO] [robot_state_publisher-3]: process started with pid [13003]
[INFO] [spawn_entity.py-4]: process started with pid [13004]
[INFO] [path_planner_node-5]: process started with pid [13005]
[spawn_entity.py-4] [INFO] [spawn_entity]: Spawn Entity started
[spawn_entity.py-4] [INFO] [spawn_entity]: Spawning entity [drone] at position [0.0, 0.0, 0.1]
[gzserver-1] [INFO] [Gazebo]: Spawned entity [drone]
[path_planner_node-5] [INFO] [path_planner_node]: Path Planner Node has been started.
[path_planner_node-5] [INFO] [path_planner_node]: Following path in Gazebo...
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

### RViz Visualization
![RViz Visualization](rviz.png)![RViz Visualization 2](rviz2.png)

### Gazebo World
![Gazebo World](images/gazebo_world.png)

### Gazebo Drone
![Gazebo Drone](images/gazebo_drone.png)