import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion, PoseStamped
from std_msgs.msg import Header
import time

class MockPosePublisher(Node):
    """
    A node to publish mock pose data for testing the path planner.
    """
    def __init__(self):
        super().__init__('mock_pose_publisher')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 10)
        self.goal_publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.timer = self.create_timer(0.1, self.publish_pose)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.pose_publish_count = 0
        self.goal_published = False
        self.has_pose = False
        self.pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10)
 
    def pose_callback(self, msg):
        self.has_pose = True

    def publish_pose(self):
        """
        Publishes a mock pose message.
        """
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
        
        # Simulate movement
        self.x += 0.1
        self.y += 0.1
        self.z += 0.05
        self.pose_publish_count += 1
 
        if not self.goal_published and self.has_pose:
            time.sleep(2.0) # Wait for the path planner to initialize
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            goal_msg = PoseStamped()
            goal_msg.header = Header()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = 'map'
            goal_msg.pose.position.x = 40.0
            goal_msg.pose.position.y = 40.0
            goal_msg.pose.position.z = 30.0
            self.goal_publisher_.publish(goal_msg)
            self.get_logger().info("Published goal pose.")
            self.goal_published = True

def main(args=None):
    rclpy.init(args=args)
    mock_pose_publisher = MockPosePublisher()
    rclpy.spin(mock_pose_publisher)
    mock_pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()