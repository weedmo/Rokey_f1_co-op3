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

        # 구독 및 퍼블리셔
        self.subscription = self.create_subscription(Image, '/pi_camera/birds_eye_view', self.image_callback, 10)
        self.direction_pub = self.create_publisher(String, '/lane_direction', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("✅ SobelCurveDirection + cmd_vel 제어 시작됨")

        # Sobel 필터
        self.kernels = {
            'vertical': np.array([[-1, 0, 1],
                                  [-2, 0, 2],
                                  [-1, 0, 1]], dtype=np.float32),  # 직진
            'diag_left': np.array([[0, 1, 2],
                                   [-1, 0, 1],
                                   [-2, -1, 0]], dtype=np.float32),  # ↙ 우회전
            'diag_right': np.array([[2, 1, 0],
                                    [1, 0, -1],
                                    [0, -1, -2]], dtype=np.float32),  # ↘ 좌회전
        }

    def image_callback(self, msg):
        gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        scores = {}

        for name, kernel in self.kernels.items():
            filtered = cv2.filter2D(gray, -1, kernel)
            score = np.sum(np.abs(filtered))
            scores[name] = score

        best = max(scores, key=scores.get)
        delta = abs(scores['diag_left'] - scores['diag_right'])

        max_ang = 0.25
        base_lin = 0.1

        if best == 'vertical':
            direction = '직진'
            lin, ang = base_lin, 0.0
        elif best == 'diag_left':
            direction = '우회전'
            lin = base_lin
            ang = -min(delta * 0.00005, max_ang)
        elif best == 'diag_right':
            direction = '좌회전'
            lin = base_lin
            ang = +min(delta * 0.00005, max_ang)
        else:
            direction = '정지'
            lin, ang = 0.0, 0.0

        # direction 퍼블리시
        dir_msg = String()
        dir_msg.data = direction
        self.direction_pub.publish(dir_msg)

        # cmd_vel 퍼블리시
        cmd = Twist()
        cmd.linear.x = lin
        cmd.angular.z = ang
        self.cmd_pub.publish(cmd)

        self.get_logger().info(f"[{direction}] lin: {lin:.2f}, ang: {ang:.2f}")

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
