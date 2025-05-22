#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy
class ROIFilter(Node):
    def __init__(self):
        super().__init__('lidar_roi')

        self.declare_parameter('min_angle', -math.radians(60))
        self.declare_parameter('max_angle', math.radians(60))

        self.min_angle = self.get_parameter('min_angle').value
        self.max_angle = self.get_parameter('max_angle').value
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos
        )

        self.pub = self.create_publisher(LaserScan, '/scan_roi', 10)

    def scan_callback(self, msg):
        new_msg = LaserScan()
        new_msg.header = msg.header
        new_msg.angle_min = msg.angle_min
        new_msg.angle_max = msg.angle_max
        new_msg.angle_increment = msg.angle_increment
        new_msg.time_increment = msg.time_increment
        new_msg.scan_time = msg.scan_time
        new_msg.range_min = msg.range_min
        new_msg.range_max = msg.range_max
        new_msg.ranges = list(msg.ranges)

        for i, angle in enumerate(self._angle_range(msg)):
            if angle < self.min_angle or angle > self.max_angle:
                new_msg.ranges[i] = float('inf')

        self.pub.publish(new_msg)

    def _angle_range(self, msg):
        return [msg.angle_min + i * msg.angle_increment for i in range(len(msg.ranges))]

def main(args=None):
    rclpy.init(args=args)
    node = ROIFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()