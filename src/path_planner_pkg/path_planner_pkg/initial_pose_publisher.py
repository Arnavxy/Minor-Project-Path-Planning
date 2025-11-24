import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        # Allow time for the publisher to be ready
        time.sleep(1.0)
        self.publish_initial_pose()

    def publish_initial_pose(self):
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = "map"
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.pose.position.x = 0.0
        initial_pose.pose.pose.position.y = 0.0
        initial_pose.pose.pose.position.z = 0.0
        initial_pose.pose.pose.orientation.w = 1.0
        self.publisher.publish(initial_pose)
        self.get_logger().info('Published initial pose to kickstart the simulation.')
        self.get_logger().info('Published initial pose to kickstart the simulation.')

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    # The node will shutdown on its own after publishing
    # We spin just to keep the process alive long enough for the publish to happen
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()