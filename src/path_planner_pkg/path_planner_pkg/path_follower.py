import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
import math

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        # Configure QoS profile for MAVROS topics
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.path_subscriber = self.create_subscription(
            Path,
            '/path',
            self.path_callback,
            10)
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            qos_profile)

        # Publishers
        self.local_pos_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10)

        # Service Clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # State variables
        self.path = []
        self.current_path_index = 0
        self.current_pose = None
        self.current_state = State()
        self.target_pose = PoseStamped()

        # Wait for MAVROS services to be available
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Arming service not available, waiting again...')
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Set mode service not available, waiting again...')

        # Main control loop timer
        self.timer = self.create_timer(0.1, self.timer_callback)

    def state_callback(self, msg):
        self.current_state = msg

    def path_callback(self, msg):
        self.get_logger().info('Received a new path. Following...')
        self.path = msg.poses
        self.current_path_index = 0

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def timer_callback(self):
        if not self.path or not self.current_pose:
            return

        # Offboard and Arming Logic
        if self.current_state.mode != 'OFFBOARD':
            self.set_offboard_mode()
        elif not self.current_state.armed:
            self.arm_drone()

        # Path Following Logic
        if self.current_path_index < len(self.path):
            self.target_pose = self.path[self.current_path_index]
            
            dx = self.target_pose.pose.position.x - self.current_pose.position.x
            dy = self.target_pose.pose.position.y - self.current_pose.position.y
            dz = self.target_pose.pose.position.z - self.current_pose.position.z
            
            distance = math.sqrt(dx**2 + dy**2 + dz**2)
            
            if distance < 0.5:  # Waypoint reached threshold
                self.current_path_index += 1
                if self.current_path_index < len(self.path):
                    self.get_logger().info(f'Waypoint reached, moving to next waypoint ({self.current_path_index})')
                else:
                    self.get_logger().info('Final waypoint reached. Mission complete.')
        
        # Publish the target pose continuously
        self.local_pos_pub.publish(self.target_pose)

    def set_offboard_mode(self):
        req = SetMode.Request()
        req.custom_mode = 'OFFBOARD'
        future = self.set_mode_client.call_async(req)
        future.add_done_callback(self.offboard_response_callback)

    def offboard_response_callback(self, future):
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info('Offboard mode enabled')
            else:
                self.get_logger().warn('Failed to enable Offboard mode')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def arm_drone(self):
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        future.add_done_callback(self.arming_response_callback)

    def arming_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Drone armed')
            else:
                self.get_logger().warn('Failed to arm drone')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    path_follower = PathFollower()
    rclpy.spin(path_follower)
    path_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()