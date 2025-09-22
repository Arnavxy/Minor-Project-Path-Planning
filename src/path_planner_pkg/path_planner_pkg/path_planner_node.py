import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Point, PoseWithCovarianceStamped
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
from sensor_msgs.msg import BatteryState
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Path
from .a_star_planner import Grid, a_star_search
from .local_planner import LocalPlanner

class PathPlannerNode(Node):
    """
    Path planning node for the drone.
    """
    def __init__(self):
        super().__init__('path_planner_node')

        # Create a static grid with obstacles
        self.grid = Grid(50, 50)
        self.grid.add_obstacle((10, 10))
        self.grid.add_obstacle((10, 11))
        self.grid.add_obstacle((10, 12))
        self.grid.add_obstacle((10, 13))
        self.grid.add_obstacle((10, 14))

        self.global_path = None
        self.current_pose = None
        self.local_planner = LocalPlanner()
        self.recorded_path = []
        self.return_to_home_active = False
        self.battery_level = 100.0
 
        # Subscribers
        self.battery_subscriber = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            10)
        self.pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10)
        self.goal_subscriber = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10)
 
        # Publishers
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode,
            '/offboard_control_mode',
            10)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint,
            '/trajectory_setpoint',
            10)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand,
            '/vehicle_command',
            10)
        self.marker_publisher = self.create_publisher(Marker, '/visualization_marker', 10)
        self.path_publisher = self.create_publisher(Path, '/path', 10)
        self.start_end_publisher = self.create_publisher(MarkerArray, '/start_end_points', 10)

        # Timers
        self.timer = self.create_timer(0.1, self.timer_callback) # 10Hz
        self.get_logger().info("Path Planner Node has been started.")

    def pose_callback(self, msg):
        """
        Callback for the vehicle's pose.
        """
        self.current_pose = msg.pose.pose
        if not self.return_to_home_active:
            if not self.recorded_path or \
               self.dist(self.current_pose.position, self.recorded_path[-1].position) > 0.5:
                self.recorded_path.append(self.current_pose)

    def dist(self, p1, p2):
        """
        Calculate the distance between two points.
        """
        return ((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)**0.5

    def goal_callback(self, msg):
        """
        Callback for the goal pose.
        """
        self.get_logger().info("Goal callback triggered.")
        if self.current_pose is None:
            self.get_logger().info("No current pose yet, cannot plan path.")
            return

        start = (int(self.current_pose.position.x), int(self.current_pose.position.y))
        end = (int(msg.pose.position.x), int(msg.pose.position.y))
        
        self.get_logger().info(f"Planning from {start} to {end}")
        self.publish_start_end_points(start, end)
        self.global_path = a_star_search(self.grid, start, end)
        
        if self.global_path:
            self.get_logger().info(f"Global path found: {self.global_path}")
        else:
            self.get_logger().info("No global path found.")

    def publish_start_end_points(self, start, end):
        """
        Publish the start and end points as markers.
        """
        marker_array = MarkerArray()

        # Start marker
        start_marker = Marker()
        start_marker.header.frame_id = "map"
        start_marker.header.stamp = self.get_clock().now().to_msg()
        start_marker.ns = "start_end"
        start_marker.id = 0
        start_marker.type = Marker.CUBE
        start_marker.action = Marker.ADD
        start_marker.pose.position.x = float(start[0])
        start_marker.pose.position.y = float(start[1])
        start_marker.pose.position.z = 0.0
        start_marker.scale.x = 1.0
        start_marker.scale.y = 1.0
        start_marker.scale.z = 1.0
        start_marker.color.a = 1.0
        start_marker.color.r = 0.0
        start_marker.color.g = 1.0
        start_marker.color.b = 0.0
        marker_array.markers.append(start_marker)

        # End marker
        end_marker = Marker()
        end_marker.header.frame_id = "map"
        end_marker.header.stamp = self.get_clock().now().to_msg()
        end_marker.ns = "start_end"
        end_marker.id = 1
        end_marker.type = Marker.CUBE
        end_marker.action = Marker.ADD
        end_marker.pose.position.x = float(end[0])
        end_marker.pose.position.y = float(end[1])
        end_marker.pose.position.z = 0.0
        end_marker.scale.x = 1.0
        end_marker.scale.y = 1.0
        end_marker.scale.z = 1.0
        end_marker.color.a = 1.0
        end_marker.color.r = 1.0
        end_marker.color.g = 0.0
        end_marker.color.b = 0.0
        marker_array.markers.append(end_marker)

        self.start_end_publisher.publish(marker_array)
        self.get_logger().info("Published start and end points.")

    def activate_return_to_home(self):
        """
        Activate the return-to-home functionality.
        """
        self.get_logger().info("Activating return to home.")
        self.return_to_home_active = True
        self.global_path = [(int(p.position.x), int(p.position.y)) for p in self.recorded_path[::-1]]
        self.recorded_path = []
 
    def battery_callback(self, msg):
        """
        Callback for the battery state.
        """
        self.battery_level = msg.percentage
        if self.battery_level < 0.2:
            self.activate_return_to_home()
 
    def timer_callback(self):
        """
        Main control loop for the path planning logic.
        """
        if self.global_path is None:
            return
 
        if self.current_pose is None:
            return

        current_pose = self.current_pose
        current_velocity = (0.0, 0.0)
        path_segment = self.global_path[:10]
        obstacles = []

        velocities = self.local_planner.get_best_velocity(
            current_pose, current_velocity, path_segment, obstacles
        )
 
        twist_msg = Twist()
        if velocities:
            linear_velocity, angular_velocity = velocities
            twist_msg.linear.x = linear_velocity
            twist_msg.angular.z = angular_velocity
        else:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
        self.velocity_publisher.publish(twist_msg)

        self.publish_markers()

    def publish_markers(self):
        """
        Publish visualization markers for the global path.
        """
        if self.global_path:
            path_msg = Path()
            path_msg.header.frame_id = "map"
            path_msg.header.stamp = self.get_clock().now().to_msg()
            for point in self.global_path:
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = float(point[0])
                pose.pose.position.y = float(point[1])
                pose.pose.position.z = 0.0
                path_msg.poses.append(pose)
            self.path_publisher.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    path_planner_node = PathPlannerNode()
    rclpy.spin(path_planner_node)
    path_planner_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()