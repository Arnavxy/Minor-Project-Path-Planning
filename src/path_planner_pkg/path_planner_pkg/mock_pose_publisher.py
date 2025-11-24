import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion, PoseStamped
from std_msgs.msg import Header
from nav_msgs.msg import Path
import time

class MockPosePublisher(Node):
    """
    A node to publish mock pose data for testing the path planner.
    It simulates a robot moving along a planned path.
    """
    def __init__(self):
        super().__init__('mock_pose_publisher')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 10)
        self.goal_publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        self.path_subscription = self.create_subscription(
            Path,
            '/path',
            self.path_callback,
            10)
        
        self.pose_timer = self.create_timer(0.1, self.publish_pose)
        self.goal_timer = self.create_timer(2.0, self.publish_goal)

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.goal_published = False
        self.path = None
        self.path_index = 0

    def path_callback(self, msg):
        """
        Callback for receiving the global path.
        """
        self.get_logger().info("Received a new path.")
        self.path = msg.poses
        self.path_index = 0

    def publish_pose(self):
        """
        Publishes a mock pose message, following the received path.
        """
        if self.path and self.path_index < len(self.path):
            next_pose = self.path[self.path_index].pose.position
            self.x = next_pose.x
            self.y = next_pose.y
            self.z = next_pose.z
            self.path_index += 1
        elif self.path and self.path_index >= len(self.path):
            # Loop the path for continuous testing
            self.path_index = 0
            if self.path:
                 next_pose = self.path[self.path_index].pose.position
                 self.x = next_pose.x
                 self.y = next_pose.y
                 self.z = next_pose.z
                 self.path_index += 1

        msg = PoseWithCovarianceStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.pose.pose = Pose()
        msg.pose.pose.position = Point()
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = self.z
        
        msg.pose.pose.orientation = Quaternion()
        msg.pose.pose.orientation.w = 1.0
 
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published pose: {self.x}, {self.y}, {self.z}")

    def publish_goal(self):
        """
        Publishes the goal pose once.
        """
        goal_msg = PoseStamped()
        goal_msg.header = Header()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = 40.0
        goal_msg.pose.position.y = 40.0
        goal_msg.pose.position.z = 30.0
        self.goal_publisher_.publish(goal_msg)
        self.get_logger().info("Published goal pose.")

def main(args=None):
    rclpy.init(args=args)
    mock_pose_publisher = MockPosePublisher()
    rclpy.spin(mock_pose_publisher)
    mock_pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()