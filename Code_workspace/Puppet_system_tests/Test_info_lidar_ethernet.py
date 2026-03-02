from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

scan = create_subscription(LaserScan, "/scan", self.laser_callback, 10)

def laser_callback(msg):
    laser_ranges = np.array(msg.ranges)
    angle_min = msg.angle_min 
    increment = msg.angle_increment

    angle_max = angle_min + len(laser_ranges) * increment

    print("Nombre de lasers:", len(laser_ranges))
    print("Angle minimal:", angle_min)
    print("Angle maximal:", angle_max)
    print("Incrementation:", increment)
    print("Centered laser value:", laser_ranges[np.round(len(laser_ranges)/2)])

if __name__ == '__main__':
    while True:
        print("Loop!")