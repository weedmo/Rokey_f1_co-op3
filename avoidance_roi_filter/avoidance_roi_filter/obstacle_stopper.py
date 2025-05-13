import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class ObstacleStopper(Node):
    def __init__(self):
        super().__init__('obstacle_stopper_node')

        self.cmd_input_sub = self.create_subscription(
            Twist,
            '/cmd_vel_input',
            self.cmd_input_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan_roi',
            self.scan_callback,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.latest_cmd = Twist()
        self.obstacle_detected = False
        self.stop_distance = 1

    def cmd_input_callback(self, msg):
        self.latest_cmd = msg
        self.publish_command()

    def scan_callback(self, msg):
        valid_ranges = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r)]
        self.obstacle_detected = (valid_ranges and min(valid_ranges) < self.stop_distance)
        self.publish_command()

    def publish_command(self):
        cmd = Twist()
        if self.obstacle_detected and self.latest_cmd.linear.x > 0:
            self.get_logger().info('STOP!!!')
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            cmd = self.latest_cmd

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleStopper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 