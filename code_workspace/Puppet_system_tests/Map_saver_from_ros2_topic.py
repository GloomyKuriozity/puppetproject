import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import csv

class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/global_costmap',
            self.listener_callback,
            10)
        self.received = False

    def listener_callback(self, msg):
        if self.received:
            return
        self.received = True

        # Save raw values
        with open('map_data.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            width = msg.info.width
            data = msg.data
            # reshape data to 2D map
            for i in range(0, len(data), width):
                writer.writerow(data[i:i + width])

        self.get_logger().info(f"Map saved as map_data.csv with width {width}, height {msg.info.height}")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = MapSaver()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
