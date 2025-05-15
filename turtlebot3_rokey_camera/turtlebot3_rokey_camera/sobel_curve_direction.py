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
        self.lost_count = 0  # ✅ 차선을 못 본 횟수 누적
        
    def image_callback(self, msg):
        gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        scores = {}

        for name, kernel in self.kernels.items():
            filtered = cv2.filter2D(gray, -1, kernel)
            score = np.sum(np.abs(filtered))
            scores[name] = score

        total_score = sum(scores.values())
        score_threshold = 1500000

        cmd = Twist()

        if total_score < score_threshold:
            direction = '정지'
            self.lost_count += 1  # ✅ 차선 못 보면 증가

            # ✅ 각도 회전 이벤트 (좌우 번갈아)
            rotate_dir = 1 if self.lost_count % 2 == 0 else -1
            cmd.linear.x = 0.0
            cmd.angular.z = rotate_dir * 1.0  # 일정 각도 회전

        else:
            self.lost_count = 0  # ✅ 차선이 보이면 초기화

            best = max(scores, key=scores.get)
            delta = abs(scores['diag_left'] - scores['diag_right'])
            max_ang = 0.25
            base_lin = 0.1

            if best == 'vertical':
                direction = '직진'
                cmd.linear.x = base_lin
                cmd.angular.z = 0.0
            elif best == 'diag_left':
                direction = '우회전'
                cmd.linear.x = base_lin
                cmd.angular.z = -min(delta * 0.00003, max_ang)
            elif best == 'diag_right':
                direction = '좌회전'
                cmd.linear.x = base_lin
                cmd.angular.z = +min(delta * 0.00003, max_ang)
            else:
                direction = '정지'
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0

        # 퍼블리시
        dir_msg = String()
        dir_msg.data = direction
        self.direction_pub.publish(dir_msg)
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