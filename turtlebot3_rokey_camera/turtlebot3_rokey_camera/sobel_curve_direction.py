#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import deque

class SobelCurveDirection(Node):
    def __init__(self):
        super().__init__('sobel_curve_direction')

        # parameters
        self.declare_parameter('history_depth', 10)  # 🔹 히스토리 깊이
        self.history_depth = self.get_parameter('history_depth').value
        self.cmd_history = deque(maxlen=self.history_depth)  # 🔹 cmd_vel angular.z 이력
        self.declare_parameter('score_threshold', 1500000)
        self.declare_parameter('max_ang', 0.25)
        self.declare_parameter('base_lin', 0.1)
        self.declare_parameter('gain', 0.00003)
        self.declare_parameter('lost_rotate_gain', 1.0)
        self.declare_parameter('obstacle_dist_threshold', 0.3) # 새 파라미터

        self.score_threshold = self.get_parameter('score_threshold').value
        self.max_ang = self.get_parameter('max_ang').value
        self.base_lin = self.get_parameter('base_lin').value
        self.gain = self.get_parameter('gain').value
        self.lost_rotate_gain = self.get_parameter('lost_rotate_gain').value
        self.obstacle_dist_threshold = self.get_parameter('obstacle_dist_threshold').value

        self.bridge = CvBridge()
        self.lost_count = 0
        self.obstacle_close = False

        # 구독자
        self.create_subscription(Image, '/pi_camera/birds_eye_view', self.image_callback, 10)
        self.create_subscription(LaserScan, '/scan_roi', self.scan_callback, 10)
        # 퍼블리셔
        self.direction_pub = self.create_publisher(String, '/lane_direction', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Sobel 커널
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

        self.get_logger().info("✅ SobelCurveDirection + cmd_vel 제어 시작됨")

    def scan_callback(self, scan: LaserScan):
        # scan.ranges 중 유효한 값들만 추려 최소값 계산
        valid = [d for d in scan.ranges if scan.range_min < d < scan.range_max]
        if valid and min(valid) < self.obstacle_dist_threshold:
            self.obstacle_close = True
        else:
            self.obstacle_close = False

    def image_callback(self, msg: Image):
        cmd = Twist()
        direction = '정지'

        # 🛑 수동 조작 감지
        if self.count_publishers('/cmd_vel') >= 2:
            self.get_logger().info("❗️수동 조작 중 — 자동주행 차단")
            return

        # 🛑 장애물 감지 시 우선 정지
        if hasattr(self, 'obstacle_close') and self.obstacle_close:
            direction = '장애물 감지: 멈춤'
            self.publish_and_log(direction, cmd)
            return

        # 📷 이미지 처리
        gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        scores = {name: np.sum(np.abs(cv2.filter2D(gray, -1, kernel)))
                for name, kernel in self.kernels.items()}
        total_score = sum(scores.values())

        # 🔸 라인 상실 시 평균 회전
        if total_score < self.score_threshold:
            direction = '라인 상실: 평균 회전'
            self.lost_count += 1
            avg_ang = np.mean(self.cmd_history) if self.cmd_history else 0.0
            cmd.linear.x = 0.05
            cmd.angular.z = avg_ang
            self.publish_and_log(direction, cmd, total_score)
            return

        # ✅ 정상 인식
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

        # 🔹 현재 angular.z 저장
        self.cmd_history.append(cmd.angular.z)
        self.publish_and_log(direction, cmd, total_score)



    def publish_and_log(self, direction, cmd: Twist, total_score=None):
        self.direction_pub.publish(String(data=direction))
        self.cmd_pub.publish(cmd)
        if total_score is not None:
            self.get_logger().info(
                f"[{direction}] lin: {cmd.linear.x:.2f}, ang: {cmd.angular.z:.2f} | "
                f"total_score: {total_score:.0f}, lost: {self.lost_count}"
            )
        else:
            self.get_logger().info(f"[{direction}] cmd zeroed due to obstacle")

def main(args=None):
    rclpy.init(args=args)
    node = SobelCurveDirection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
