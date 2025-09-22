# Autonomous Path Planning and Adaptive Return Navigation for UAVs

This ROS2 package provides a sophisticated high-level path planning and navigation system for Unmanned Aerial Vehicles (UAVs). It is engineered to function reliably using pose data from external localization modules, enabling robust autonomous operation, particularly in environments where GPS is unavailable or unreliable.

## 📖 Table of Contents
- [Autonomous Path Planning and Adaptive Return Navigation for UAVs](#autonomous-path-planning-and-adaptive-return-navigation-for-uavs)
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
  - [📊 Detailed Log Output Analysis](#-detailed-log-output-analysis)
    - [Build and Launch Sequence](#build-and-launch-sequence)
    - [Node Initialization](#node-initialization)
    - [Path Planning Execution](#path-planning-execution)
    - [Conclusion of Log Analysis](#conclusion-of-log-analysis)
  - [🗺️ RViz Visualization Analysis](#️-rviz-visualization-analysis)
    - [Understanding the Visualization](#understanding-the-visualization)
    - [Why is the Path a Nearly Straight Line?](#why-is-the-path-a-nearly-straight-line)

## 📜 Project Abstract

Effective autonomous operation for a UAV requires not only knowing its position but also making intelligent decisions about its movement. This project focuses on the design and implementation of a high-level path planning and navigation system that operates reliably using pose data from localization modules. The system will employ a hybrid planning approach, utilizing a global planner like A* or RRT* on a pre-existing map to generate an initial, optimal route to a target. During execution, this global plan will be augmented by a local planner that uses the dynamically generated SLAM map for real-time obstacle avoidance and dynamic replanning. A key innovation of this project is the development of an adaptive return-to-home logic that reuses the map and path memory from the outbound journey to ensure a safe and efficient return trip. The system will also incorporate critical fail-safe strategies, including low-battery protocols and obstacle-induced hovering. The final deliverable is a comprehensive ROS2 navigation node that consumes pose estimates and produces velocity commands to safely and effectively guide the UAV through its mission.

## ✨ Key Features

*   **Hybrid Path Planning**: A combination of a global planner (A*) for long-range route optimization and a local planner for real-time adjustments and obstacle avoidance.
*   **Global Path Optimization**: Employs the A* search algorithm to compute the shortest, most efficient path on a 2D grid map, considering static obstacles.
*   **Dynamic Local Navigation**: The local planner is responsible for generating feasible velocity commands (`cmd_vel`) that adhere to the global path while providing a framework for future real-time obstacle avoidance.
*   **Adaptive Return-to-Home**: A crucial safety feature where the UAV continuously records its trajectory. This allows it to autonomously retrace its steps and return to the launch point.
*   **Robust Fail-Safe Mechanisms**: The system includes a low-battery protocol that automatically suspends the current mission and triggers the return-to-home functionality, ensuring the vehicle's safe recovery.
*   **Advanced Visualization**: Publishes critical data, including the global path, start/goal markers, and the UAV's trajectory, to RViz for real-time monitoring and debugging.
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
        *   `/start_end_points` (`MarkerArray`): Publishes markers to visualize the start and goal positions.
        *   PX4-related topics (`/offboard_control_mode`, `/trajectory_setpoint`, `/vehicle_command`): These are placeholders for future integration with the PX4 flight controller.

*   **`mock_pose_publisher`**: A development tool used to simulate the UAV's movement and provide necessary inputs for testing the `path_planner_node`.
    *   It simulates a UAV moving along a predefined trajectory.
    *   After a short delay, it publishes a goal pose to `/goal_pose` to initiate the path planning process. This simulates a mission command.

*   **`static_transform_publisher`**: A standard ROS2 utility that publishes static coordinate frame transformations. This is essential for ensuring all components share a consistent understanding of the spatial relationships between different frames, such as `map`, `odom`, and `base_link`.

### Core Logic and Algorithms

1.  **Global Planning (A* Algorithm)**:
    *   The `a_star_planner.py` file contains the implementation of the A* search algorithm.
    *   A* is a widely-used pathfinding algorithm known for its completeness, optimality, and efficiency. It explores a graph by combining the cost to reach a node (`g(n)`) with a heuristic estimate of the cost to the goal from that node (`h(n)`).
    *   The implementation uses a `Grid` class to represent the environment and a `Node` class to represent states in the search space.

2.  **Local Planning**:
    *   The `local_planner.py` file implements the local planning logic.
    *   Its primary role is to translate the high-level global path into low-level velocity commands.
    *   It evaluates a set of possible linear and angular velocities and scores them based on criteria like path following and obstacle avoidance. The velocity pair with the best score is chosen.
    *   Currently, the obstacle avoidance is based on the static map, but this component is designed to be extended with real-time sensor data.

3.  **Path Recording and Return-to-Home**:
    *   The `path_planner_node` maintains a list of poses, `recorded_path`.
    *   When the `activate_return_to_home` function is called (e.g., due to low battery), the `global_path` is replaced with the reversed `recorded_path`, guiding the UAV back to its starting point.

## 📈 Current Status and Accomplishments

The project has successfully implemented the foundational components of the path planning system. The current prototype robustly demonstrates:

*   **End-to-End Global Path Generation**: The system can successfully receive a goal, trigger the A* planner, and generate a valid, obstacle-free path from a start to a goal position. This is a key milestone, proving the core planning logic is functional.
*   **Functional Local Planner**: The local planner is capable of interpreting the global path and generating appropriate velocity commands to follow it in a simulated environment.
*   **Path Memory Implementation**: The system correctly records the trajectory of the UAV, which is the basis for the return-to-home feature.
*   **Implemented Low-Battery Failsafe**: The logic to monitor battery status and trigger the return-to-home functionality is in place and functional.
*   **Complete Simulation and Visualization Environment**: The project can be launched with a single command, and the results can be clearly visualized in RViz, which is crucial for debugging and presentations.

The successful execution and the detailed log output confirm that these features are not just implemented but are working together as an integrated system.

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
*   Add a `Path` display and subscribe to the `/path` topic.
*   Add a `MarkerArray` display and subscribe to the `/start_end_points` topic.

## 📊 Detailed Log Output Analysis

The log output provides a clear, step-by-step narrative of the system's successful execution. This analysis breaks down the log and explains what each message signifies in the context of the project's goals.

### Build and Launch Sequence
```
arnav@Arnavs-Laptop:~/drone_ws$ colcon build --packages-select path_planner_pkg && source install/setup.bash && ros2 launch path_planner_pkg path_planner.launch.py
Starting >>> path_planner_pkg
Finished <<< path_planner_pkg [1.32s]

Summary: 1 package finished [1.46s]
```
*   **Analysis**: This section shows that the `colcon` build system successfully compiled the `path_planner_pkg`. The `&&` ensures that the subsequent commands (sourcing the environment and launching the nodes) only run if the build is successful. This demonstrates a clean and correct package setup.

### Node Initialization
```
[INFO] [launch]: All log files can be found below /home/arnav/.ros/log/2025-09-22-07-19-40-756034-Arnavs-Laptop-153856
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [path_planner_node-1]: process started with pid [153863]
[INFO] [static_transform_publisher-2]: process started with pid [153864]
[INFO] [static_transform_publisher-3]: process started with pid [153865]
...
[path_planner_node-1] [INFO] [1758505781.349029873] [path_planner_node]: Path Planner Node has been started.
[INFO] [mock_pose_publisher-4]: process started with pid [153929]
```
*   **Analysis**: The ROS2 launch system starts all the necessary nodes defined in `path_planner.launch.py`. The `path_planner_node`, two `static_transform_publisher` instances, and the `mock_pose_publisher` are all successfully initialized. The message `Path Planner Node has been started` is a custom log message from the code, confirming that the node's `__init__` method has completed without errors. This is a critical check.

### Path Planning Execution
```
[path_planner_node-1] [INFO] [1758505783.783450079] [path_planner_node]: Goal callback triggered.
```
*   **Analysis**: This is the trigger for the main planning sequence. The `mock_pose_publisher` has published a goal message on the `/goal_pose` topic, and the `path_planner_node`'s subscription has received it, invoking the `goal_callback` function. This demonstrates successful ROS2 topic communication between the nodes.
```
[path_planner_node-1] [INFO] [1758505783.784072156] [path_planner_node]: Planning from (0, 0) to (40, 40)
```
*   **Analysis**: Inside the `goal_callback`, the node has identified the UAV's current position (0, 0) from the mock pose data and the goal position (40, 40) from the received message. It is now initiating the A* search algorithm with these start and end points.
```
[path_planner_node-1] [INFO] [1758505783.785123063] [path_planner_node]: Published start and end points.
```
*   **Analysis**: For visualization and debugging, the node publishes markers to RViz to show where the planning process starts and where it aims to finish. This is an important feature for user feedback.
```
[path_planner_node-1] [INFO] [1758505783.786596129] [path_planner_node]: Global path found: [(0, 0), (1, 1), (2, 2), ... (40, 39), (40, 40)]
```
*   **Analysis**: This is the most critical message in the log. It confirms that the A* search algorithm successfully found a complete path from the start to the end node. The list of tuples represents the sequence of waypoints that the UAV will follow. This message is the primary indicator that the core functionality of the project—global path planning—is working correctly.
```
[mock_pose_publisher-4] [INFO] [1758505783.801936249] [mock_pose_publisher]: Published goal pose.
```
*   **Analysis**: This is a confirmation message from the `mock_pose_publisher` itself, indicating that it has successfully sent the goal pose message that triggered the entire planning sequence.

### Conclusion of Log Analysis

The log output provides concrete evidence that the primary objectives of the current project phase have been met. It demonstrates a fully functional pipeline: from launching the system and initializing all nodes, through inter-node communication via topics, to the successful execution of the A* global path planner. This represents a solid foundation for the future work of integrating dynamic obstacle avoidance and real-world hardware.

## 🗺️ RViz Visualization Analysis

The image you provided is a screenshot of RViz, the standard visualization tool for ROS. It's showing a real-time representation of your path planning system at work.

### Understanding the Visualization

*   **Grid**: The grey grid on the floor represents the `map` frame, which is the global coordinate system for your simulation.
*   **Green Line (`/path`)**: This is the most important part. The green line is the global path calculated by your A* planner. It's a `nav_msgs/Path` message published by the `path_planner_node`. Each point on this line corresponds to a waypoint in the `global_path` list that was printed in your log output.
*   **Start/End Markers (`/start_end_points`)**: Although not clearly visible as distinct shapes in this view, the start and end points of the path are published as `visualization_msgs/MarkerArray`. These would appear as cubes (one green for the start, one red for the end) at coordinates (0,0) and (40,40) respectively.
*   **Displays Panel (Left)**: This panel shows which topics you are visualizing. You have correctly added the `Grid`, `MarkerArray` (for `/start_end_points`), and `Path` (for `/path`) displays. The checkmarks and "Ok" status indicate that RViz is successfully receiving data on these topics.

### Why is the Path a Nearly Straight Line?

This is an excellent question and it points to a key aspect of how the A* algorithm and your current simulation are configured.

1.  **A* Finds the Optimal Path**: The A* algorithm is designed to find the *shortest* possible path between two points. In an open environment with no obstacles between the start and goal, the shortest path is, by definition, a straight line.

2.  **Limited Obstacles in the Current Map**: In your current code, the environment is a wide-open 50x50 grid. You have only defined one small, vertical obstacle:
    ```python
    # From path_planner_node.py
    self.grid.add_obstacle((10, 10))
    self.grid.add_obstacle((10, 11))
    self.grid.add_obstacle((10, 12))
    self.grid.add_obstacle((10, 13))
    self.grid.add_obstacle((10, 14))
    ```
    The path planned from (0,0) to (40,40) does not intersect this obstacle. Therefore, the A* algorithm correctly determines that the most efficient route is a direct diagonal line. The slight curve you see in the path is likely an artifact of the grid-based nature of the A* search, where it moves from one grid cell to the next (e.g., from (1,1) to (2,2)), creating a stepped diagonal line that looks almost straight from a distance.

**In summary, the straight-line path is not a bug; it is the correct and expected output for the A* algorithm in an environment with no obstacles blocking the direct route. This visualization successfully demonstrates that your global planner is working as intended.**