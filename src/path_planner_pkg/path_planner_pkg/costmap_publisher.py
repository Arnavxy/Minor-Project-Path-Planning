import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

class CostmapPublisher(Node):
    def __init__(self):
        super().__init__('costmap_publisher')
        self.publisher_ = self.create_publisher(OccupancyGrid, '/map', 10)
        self.timer = self.create_timer(1.0, self.publish_costmap)
        self.get_logger().info("Costmap Publisher has been started.")

    def publish_costmap(self):
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'map'
        
        grid.info.resolution = 1.0
        grid.info.width = 50
        grid.info.height = 50
        grid.info.origin.position.x = 0.0
        grid.info.origin.position.y = 0.0
        grid.info.origin.position.z = 0.0

        # Create a more complex obstacle layout
        costmap = np.zeros((grid.info.height, grid.info.width), dtype=np.int8)
        
        # Outer walls
        costmap[0, :] = 100
        costmap[-1, :] = 100
        costmap[:, 0] = 100
        costmap[:, -1] = 100

        # Maze-like structure
        costmap[10:40, 10] = 100
        costmap[10, 10:40] = 100
        costmap[40, 10:41] = 100
        costmap[10:41, 40] = 100
        
        costmap[20:30, 20:30] = 100
        costmap[25, 10:20] = 100
        costmap[35, 30:40] = 100


        grid.data = costmap.flatten().tolist()
        self.publisher_.publish(grid)
        self.get_logger().info('Publishing costmap.')

def main(args=None):
    rclpy.init(args=args)
    costmap_publisher = CostmapPublisher()
    rclpy.spin(costmap_publisher)
    costmap_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()