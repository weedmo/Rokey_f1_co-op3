#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class SobelCurveDirection(Node):
    def __init__(self):
        super().__init__('sobel_curve_direction')
        self.bridge = CvBridge()

        # Declare parameters with default values
        self.declare_parameter('score_threshold', 1500000)
        self.declare_parameter('max_ang', 0.25)
        self.declare_parameter('base_lin', 0.1)
        self.declare_parameter('gain', 0.00003)
        self.declare_parameter('lost_rotate_gain', 1.0)

        self.score_threshold = self.get_parameter('score_threshold').value
        self.max_ang = self.get_parameter('max_ang').value
        self.base_lin = self.get_parameter('base_lin').value
        self.gain = self.get_parameter('gain').value
        self.lost_rotate_gain = self.get_parameter('lost_rotate_gain').value

        self.subscription = self.create_subscription(Image, '/pi_camera/birds_eye_view', self.image_callback, 10)
        self.direction_pub = self.create_publisher(String, '/lane_direction', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("✅ SobelCurveDirection + cmd_vel 제어 시작됨")

        self.kernels = {
            'vertical': np.array([[-1, 0, 1],
                                  [-2, 0, 2],
                                  [-1, 0, 1]], dtype=np.float32),
            'diag_left': np.array([[0, 1, 2],
                                   [-1, 0, 1],
                                   [-2, -1, 0]], dtype=np.float32),
            'diag_right': np.array([[2, 1, 0],
                                    [1, 0, -1],
                                    [0, -1, -2]], dtype=np.float32),
        }
        self.lost_count = 0
        
    def image_callback(self, msg):
        gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        scores = {}

        for name, kernel in self.kernels.items():
            filtered = cv2.filter2D(gray, -1, kernel)
            scores[name] = np.sum(np.abs(filtered))

        total_score = sum(scores.values())
        cmd = Twist()

        if total_score < self.score_threshold:
            direction = '정지'
            self.lost_count += 1
            rotate_dir = 1 if self.lost_count % 2 == 0 else -1
            cmd.linear.x = 0.0
            cmd.angular.z = rotate_dir * self.lost_rotate_gain
        else:
            self.lost_count = 0
            best = max(scores, key=scores.get)
            delta = abs(scores['diag_left'] - scores['diag_right'])

            if best == 'vertical':
                direction = '직진'
                cmd.linear.x = self.base_lin
                cmd.angular.z = 0.0
            elif best == 'diag_left':
                direction = '우회전'
                cmd.linear.x = self.base_lin
                cmd.angular.z = -min(delta * self.gain, self.max_ang)
            elif best == 'diag_right':
                direction = '좌회전'
                cmd.linear.x = self.base_lin
                cmd.angular.z = min(delta * self.gain, self.max_ang)
            else:
                direction = '정지'
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0

        self.direction_pub.publish(String(data=direction))
        self.cmd_pub.publish(cmd)
        self.get_logger().info(f"[{direction}] lin: {cmd.linear.x:.2f}, ang: {cmd.angular.z:.2f} | total_score: {total_score:.0f}, lost: {self.lost_count}")

def main(args=None):
    rclpy.init(args=args)
    node = SobelCurveDirection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("종료됨.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()