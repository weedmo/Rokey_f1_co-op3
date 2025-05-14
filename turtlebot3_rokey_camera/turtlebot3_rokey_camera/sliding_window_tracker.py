#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class SlidingWindowController(Node):
    def __init__(self):
        super().__init__('sliding_window_controller')
        self.bridge = CvBridge()
        self.direction = "직진"  # 기본 방향
        self.image_sub = self.create_subscription(Image, '/pi_camera/birds_eye_view', self.image_callback, 10)
        self.direction_sub = self.create_subscription(String, '/lane_direction', self.direction_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("✅ SlidingWindowController 시작됨")

    def direction_callback(self, msg):
        self.direction = msg.data

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

        height, width = img.shape
        crop_height = int(height * 0.3)
        binary = img[-crop_height:, :]  # 하단 30%만 사용

        histogram = np.sum(binary, axis=0)
        midpoint = width // 2
        left_base = np.argmax(histogram[:midpoint])
        right_base = np.argmax(histogram[midpoint:]) + midpoint

        nonzero = binary.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        margin = 60
        left_inds = ((nonzerox >= left_base - margin) & (nonzerox < left_base + margin)).nonzero()[0]
        right_inds = ((nonzerox >= right_base - margin) & (nonzerox < right_base + margin)).nonzero()[0]

        leftx = nonzerox[left_inds]
        lefty = nonzeroy[left_inds]
        rightx = nonzerox[right_inds]
        righty = nonzeroy[right_inds]

        if len(leftx) > 0 and len(rightx) > 0:
            # 평균 중심 계산
            left_fit = np.polyfit(lefty, leftx, 2)
            right_fit = np.polyfit(righty, rightx, 2)

            y_eval = crop_height - 1
            left_x = left_fit[0]*y_eval**2 + left_fit[1]*y_eval + left_fit[2]
            right_x = right_fit[0]*y_eval**2 + right_fit[1]*y_eval + right_fit[2]
            lane_center = (left_x + right_x) / 2
            robot_center = width / 2
            center_offset = lane_center - robot_center

            # 제어값 생성
            twist = Twist()
            twist.linear.x = 0.08  # 일정 속도
            twist.angular.z = -float(center_offset) * 0.003  # offset → 회전
            self.cmd_pub.publish(twist)

            self.get_logger().info(f"[{self.direction}] offset: {center_offset:.2f}, ang.z: {twist.angular.z:.3f}")
        else:
            self.get_logger().warn("차선 감지 실패 → 정지")
            self.cmd_pub.publish(Twist())  # 정지

def main(args=None):
    rclpy.init(args=args)
    node = SlidingWindowController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("종료됨.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
