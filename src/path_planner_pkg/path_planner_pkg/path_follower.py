import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped
import math

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
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
        # self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path = []
        self.current_path_index = 0
        self.current_pose = None
        # self.timer = self.create_timer(0.1, self.timer_callback)

    def path_callback(self, msg):
        self.get_logger().info('Received a new path. Following...')
        self.path = msg.poses
        self.current_path_index = 0

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    # def timer_callback(self):
    #     if self.path and self.current_pose:
    #         if self.current_path_index < len(self.path):
    #             target_pose = self.path[self.current_path_index].pose
                
    #             dx = target_pose.position.x - self.current_pose.position.x
    #             dy = target_pose.position.y - self.current_pose.position.y
    #             dz = target_pose.position.z - self.current_pose.position.z
                
    #             distance = math.sqrt(dx**2 + dy**2 + dz**2)
                
    #             if distance < 0.5:
    #                 self.current_path_index += 1
                
    #             twist = Twist()
    #             twist.linear.x = dx * 0.5  # Proportional control
    #             twist.linear.y = dy * 0.5
    #             twist.linear.z = dz * 0.5
                
    #             self.cmd_vel_publisher.publish(twist)
    #         else:
    #             # Stop if path is finished
    #             self.cmd_vel_publisher.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    path_follower = PathFollower()
    rclpy.spin(path_follower)
    path_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()