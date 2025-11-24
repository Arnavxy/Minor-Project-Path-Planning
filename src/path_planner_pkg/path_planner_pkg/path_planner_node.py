import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Point, PoseWithCovarianceStamped
from sensor_msgs.msg import BatteryState
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Path, Odometry
from .a_star_planner import Grid, a_star_search
from .local_planner import LocalPlanner
from nav_msgs.msg import OccupancyGrid
from interactive_markers import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from enum import Enum

class DroneState(Enum):
    IDLE = 1
    PLANNING = 2
    EXECUTING = 3

class PathPlannerNode(Node):
    """
    Path planning node for the drone.
    """
    def __init__(self):
        super().__init__('path_planner_node')
 
        # Create a grid
        self.grid = Grid(50, 50, 50)
        self.costmap = None
 
        self.global_path = None
        self.current_pose = None
        self.local_planner = LocalPlanner()
        self.recorded_path = []
        self.return_to_home_active = False
        self.battery_level = 100.0
        self.goal_pose = None
        self.state = DroneState.IDLE
        self.offboard_setpoint_counter = 0
  
        # Subscribers
        self.battery_subscriber = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            10)
        self.pose_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.pose_callback,
            10)
        self.goal_subscriber = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10)
        self.costmap_subscriber = self.create_subscription(
            OccupancyGrid,
            '/costmap',
            self.costmap_callback,
            10)

        # Publishers
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.traversed_path_publisher = self.create_publisher(Path, '/traversed_path', 10)
        self.marker_publisher = self.create_publisher(Marker, '/visualization_marker', 10)
        self.obstacle_publisher = self.create_publisher(MarkerArray, '/obstacles', 10)
        self.path_publisher = self.create_publisher(Path, '/path', 10)
        self.scoring_grid_publisher = self.create_publisher(MarkerArray, '/scoring_grid', 10)
 
        # Timers
        self.timer = self.create_timer(0.1, self.timer_callback) # 10Hz
        self.get_logger().info("Path Planner Node has been started.")
        self.publish_obstacles()
        self.interactive_marker_server = InteractiveMarkerServer(self, 'path_planner_markers')
        self.setup_interactive_markers()

        # Set an initial goal to kickstart the process
        initial_goal = PoseStamped()
        initial_goal.header.frame_id = "map"
        initial_goal.header.stamp = self.get_clock().now().to_msg()
        initial_goal.pose.position.x = 15.0
        initial_goal.pose.position.y = -15.0
        initial_goal.pose.position.z = 5.0
        self.goal_callback(initial_goal)

    def costmap_callback(self, msg):
        self.get_logger().info("Received costmap.")
        self.costmap = msg
        self.grid = Grid(msg.info.width, msg.info.height, 50) # Assuming a fixed depth for now
        for i in range(msg.info.height):
            for j in range(msg.info.width):
                if msg.data[i * msg.info.width + j] > 50: # Obstacle threshold
                    for k in range(self.grid.depth): # Add obstacle at all depths
                        self.grid.add_obstacle((j, i, k))
        self.publish_obstacles()
 
    def setup_interactive_markers(self):
        # Create an interactive marker for the start pose
        start_marker = InteractiveMarker()
        start_marker.header.frame_id = "map"
        start_marker.name = "start_pose"
        start_marker.description = "Start Pose"
        
        # Create a box marker for the start pose
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.scale.x = 1.0
        box_marker.scale.y = 1.0
        box_marker.scale.z = 1.0
        box_marker.color.r = 0.0
        box_marker.color.g = 1.0
        box_marker.color.b = 0.0
        box_marker.color.a = 1.0

        # Create a control for the start marker
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(box_marker)
        start_marker.controls.append(control)

        # Add motion controls
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        control.name = "move_plane"
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        start_marker.controls.append(control)

        self.interactive_marker_server.insert(start_marker)

        # Create an interactive marker for the goal pose
        goal_marker = InteractiveMarker()
        goal_marker.header.frame_id = "map"
        goal_marker.name = "goal_pose"
        goal_marker.description = "Goal Pose"

        # Create a box marker for the goal pose
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.scale.x = 1.0
        box_marker.scale.y = 1.0
        box_marker.scale.z = 1.0
        box_marker.color.r = 1.0
        box_marker.color.g = 0.0
        box_marker.color.b = 0.0
        box_marker.color.a = 1.0

        # Create a control for the goal marker
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(box_marker)
        goal_marker.controls.append(control)

        # Add motion controls
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        control.name = "move_plane"
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        goal_marker.controls.append(control)

        self.interactive_marker_server.insert(goal_marker)
        self.interactive_marker_server.applyChanges()

    def start_pose_callback(self, feedback):
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose = feedback.pose
        self.pose_callback(pose)

    def goal_pose_callback(self, feedback):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose = feedback.pose
        self.goal_callback(pose)

    def pose_callback(self, msg):
        """
        Callback for the vehicle's pose.
        """
        # Convert Odometry to PoseWithCovarianceStamped structure for compatibility
        self.current_pose = PoseWithCovarianceStamped()
        self.current_pose.header = msg.header
        self.current_pose.pose = msg.pose

        if not self.return_to_home_active:
            if not self.recorded_path or \
               self.dist(self.current_pose.pose.pose.position, self.recorded_path[-1].pose.position) > 0.5:
                pose_stamped = PoseStamped()
                pose_stamped.header = self.current_pose.header
                pose_stamped.pose = self.current_pose.pose.pose
                self.recorded_path.append(pose_stamped)

                # Publish the traversed path
                traversed_path_msg = Path()
                traversed_path_msg.header.frame_id = "map"
                traversed_path_msg.header.stamp = self.get_clock().now().to_msg()
                traversed_path_msg.poses = self.recorded_path
                self.traversed_path_publisher.publish(traversed_path_msg)

    def dist(self, p1, p2):
        """
        Calculate the distance between two points.
        """
        return ((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)**0.5

    def goal_callback(self, msg):
        """
        Callback for the goal pose.
        """
        if self.state == DroneState.IDLE or self.state == DroneState.EXECUTING:
            self.goal_pose = msg
            self.state = DroneState.PLANNING

    def activate_return_to_home(self):
        """
        Activate the return-to-home functionality.
        """
        self.get_logger().info("Activating return to home.")
        self.return_to_home_active = True
        self.global_path = [(int(p.position.x), int(p.position.y), int(p.position.z)) for p in self.recorded_path[::-1]]
        self.recorded_path = []
 
    def battery_callback(self, msg):
        """
        Callback for the battery state.
        """
        self.battery_level = msg.percentage
        if self.battery_level < 0.2:
            self.activate_return_to_home()
 
    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info("Arm command sent")

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info("Disarm command sent")

    def publish_vehicle_command(self, command, **params):
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self):
        """
        Main control loop for the path planning logic.
        """
        self.publish_obstacles()
        self.publish_scoring_grid()
        self.publish_markers()

        if self.current_pose is None:
            self.get_logger().info("Waiting for drone pose...")
            return

        if self.state == DroneState.IDLE:
            pass

        elif self.state == DroneState.PLANNING:
            if self.current_pose is not None and self.goal_pose is not None:
                start = (int(self.current_pose.pose.pose.position.x), int(self.current_pose.pose.pose.position.y), int(self.current_pose.pose.pose.position.z))
                end = (int(self.goal_pose.pose.position.x), int(self.goal_pose.pose.position.y), int(self.goal_pose.pose.position.z))
                
                self.get_logger().info(f"Attempting to plan path from start: {start} to end: {end}")
                if start in self.grid.obstacles:
                    self.get_logger().error(f"Start point {start} is inside an obstacle!")
                if end in self.grid.obstacles:
                    self.get_logger().error(f"End point {end} is inside an obstacle!")

                self.global_path = a_star_search(self.grid, start, end)
                
                if self.global_path:
                    self.get_logger().info(f"SUCCESS: Global path found with {len(self.global_path)} points.")
                    self.state = DroneState.EXECUTING
                else:
                    self.get_logger().error("FAILURE: No global path found. Drone will remain idle.")
                    self.state = DroneState.IDLE

        elif self.state == DroneState.EXECUTING:
            if self.global_path is None or self.current_pose is None:
                return

            if self.current_pose and self.goal_pose and self.dist(self.current_pose.pose.pose.position, self.goal_pose.pose.position) < 1.0:
                self.get_logger().info("Goal reached.")
                self.state = DroneState.IDLE
                self.global_path = None
                # Stop the drone
                vel_msg = Twist()
                self.velocity_publisher.publish(vel_msg)
                return

            if len(self.global_path) > 0:
                next_waypoint = self.global_path
                next_point = Point()
                next_point.x = float(next_waypoint)
                next_point.y = float(next_waypoint)
                next_point.z = float(next_waypoint)

                # Proportional controller for velocity
                vel_msg = Twist()
                P = 0.5
                vel_msg.linear.x = P * (next_point.x - self.current_pose.pose.pose.position.x)
                vel_msg.linear.y = P * (next_point.y - self.current_pose.pose.pose.position.y)
                vel_msg.linear.z = P * (next_point.z - self.current_pose.pose.pose.position.z)
                
                # Normalize and scale
                norm = (vel_msg.linear.x**2 + vel_msg.linear.y**2 + vel_msg.linear.z**2)**0.5
                if norm > 1.0:
                    vel_msg.linear.x /= norm
                    vel_msg.linear.y /= norm
                    vel_msg.linear.z /= norm
                
                self.velocity_publisher.publish(vel_msg)

                if self.dist(self.current_pose.pose.pose.position, next_point) < 0.5:
                    self.global_path.pop(0)
 
    def publish_scoring_grid(self):
        """
        Publish the scoring grid as markers.
        """
        if self.current_pose is None:
            return

        marker_array = MarkerArray()
        grid_size = 5
        resolution = 1.0

        for i in range(-grid_size, grid_size):
            for j in range(-grid_size, grid_size):
                for k in range(-grid_size, grid_size):
                    x = self.current_pose.pose.pose.position.x + i * resolution
                    y = self.current_pose.pose.pose.position.y + j * resolution
                    z = self.current_pose.pose.pose.position.z + k * resolution
                    
                    score = self.calculate_score(x, y, z)

                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = "scoring_grid"
                    marker.id = (i + grid_size) * (2 * grid_size) * (2 * grid_size) + (j + grid_size) * (2 * grid_size) + (k + grid_size) + 1
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD
                    marker.pose.position.x = x
                    marker.pose.position.y = y
                    marker.pose.position.z = z
                    marker.scale.x = resolution
                    marker.scale.y = resolution
                    marker.scale.z = resolution
                    marker.color.a = 0.3
                    marker.color.r = 1.0 - score
                    marker.color.g = score
                    marker.color.b = 0.0
                    marker_array.markers.append(marker)
        
        self.scoring_grid_publisher.publish(marker_array)

    def calculate_score(self, x, y, z):
        """
        Calculate the score for a given point in 3D.
        """
        # Score based on distance to nearest obstacle
        min_dist_obstacle = float('inf')
        for obs_x, obs_y, obs_z in self.grid.obstacles:
            dist = ((x - obs_x)**2 + (y - obs_y)**2 + (z - obs_z)**2)**0.5
            if dist < min_dist_obstacle:
                min_dist_obstacle = dist
        
        obstacle_score = 0.0
        if min_dist_obstacle < 2.0: # Penalize points close to obstacles
            obstacle_score = 1.0 - (min_dist_obstacle / 2.0)

        # Score based on distance to global path
        min_dist_path = float('inf')
        if self.global_path:
            for point in self.global_path:
                path_x = float(point)
                path_y = float(point)
                path_z = float(point)
                dist = ((x - path_x)**2 + (y - path_y)**2 + (z - path_z)**2)**0.5
                if dist < min_dist_path:
                    min_dist_path = dist

        path_score = 0.0
        if min_dist_path > 1.0: # Penalize points far from the path
            path_score = min(1.0, (min_dist_path - 1.0) / 5.0)

        # Combine scores (higher is worse)
        total_score = obstacle_score + path_score
        return min(1.0, total_score)

    def publish_markers(self):
        """
        Publish visualization markers for the global path.
        """
        if self.global_path:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "global_path"
            marker.id = 0
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.2  # Line width
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            for point in self.global_path:
                p = Point()
                p.x = float(point)
                p.y = float(point)
                p.z = float(point)
                marker.points.append(p)
            self.marker_publisher.publish(marker)

    def publish_obstacles(self):
        """
        Publish the obstacles as markers.
        """
        marker_array = MarkerArray()
        for i, obs in enumerate(self.grid.obstacles):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "obstacles"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(obs)
            marker.pose.position.y = float(obs)
            marker.pose.position.z = float(obs)
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)
        self.obstacle_publisher.publish(marker_array)
 
def main(args=None):
    rclpy.init(args=args)
    path_planner_node = PathPlannerNode()
    rclpy.spin(path_planner_node)
    path_planner_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()