import math
import sys

from geometry_msgs.msg import TransformStamped

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class StaticFramePublisher(Node):
    def __init__(self):
        super().__init__('static_tf2_broadcaster')
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transform()

    def publish_static_transform(self):
        static_transform_stamped = TransformStamped()

        # Set the timestamp to the current time
        static_transform_stamped.header.stamp = self.get_clock().now().to_msg()
        static_transform_stamped.header.frame_id = "base_link"
        static_transform_stamped.child_frame_id = "base_scan"

        static_transform_stamped.transform.translation.x = -0.310
        static_transform_stamped.transform.translation.y = 0.0
        static_transform_stamped.transform.translation.z = 0.33

        # Laser faces backward
        yaw = math.pi
        static_transform_stamped.transform.rotation.x = 0.0
        static_transform_stamped.transform.rotation.y = 0.0
        static_transform_stamped.transform.rotation.z = math.sin(yaw / 2.0)
        static_transform_stamped.transform.rotation.w = math.cos(yaw / 2.0)

        # Publish the static transform
        self.static_broadcaster.sendTransform(static_transform_stamped)
        self.get_logger().info("Static transform from 'base_link' to 'basic_scan' published.")

def main():
    rclpy.init()
    node = StaticFramePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
