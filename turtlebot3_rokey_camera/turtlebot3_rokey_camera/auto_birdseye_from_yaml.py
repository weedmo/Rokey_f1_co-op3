#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
import os

# ✅ YAML 경로 수동 지정
# CONFIG_FILE = "~/team_f_01_ws/src/turtlebot3_rokey_camera/config/birds_eye_points.yaml"

class BirdsEyeAuto(Node):
    def __init__(self):
        super().__init__('birds_eye_auto')
        self.bridge = CvBridge()

        # ✅ launch에서 전달받는 파라미터 읽기
        config_path = self.declare_parameter('config_file_path', '').get_parameter_value().string_value
        config_path = os.path.expanduser(config_path)  # ~ 해석

        if not os.path.exists(config_path):
            self.get_logger().error(f"⚠ YAML 파일이 존재하지 않습니다: {config_path}")
            exit(1)

        self.src_points = self.load_points_from_yaml(config_path)
        self.dst_points = np.float32([
            [0, 300],
            [640, 300],
            [640, 0],
            [0, 0]
        ])

        # 2. 구독 및 퍼블리셔 설정
        self.subscription = self.create_subscription(
            Image, '/pi_camera/preprocessed', self.image_callback, 10)
        self.birds_eye_pub = self.create_publisher(
            Image, '/pi_camera/birds_eye_view', 10)

    def load_points_from_yaml(self, file_path):
        with open(file_path, 'r') as f:
            data = yaml.safe_load(f)
        return np.float32([tuple(p) for p in data['src_points']])

    def image_callback(self, msg):
        # 1. 이미지 수신 및 변환
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 2. Bird's Eye View 변환
        matrix = cv2.getPerspectiveTransform(self.src_points, self.dst_points)
        birds_eye = cv2.warpPerspective(cv_image, matrix, (640, 300))

        # 3. HSV로 변환 후 흰색 마스크 생성
        birds_eye_hsv = cv2.cvtColor(birds_eye, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 30, 255])
        white_mask = cv2.inRange(birds_eye_hsv, lower_white, upper_white)

        # 4. 컨투어 추출 및 마킹
        contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        marked_birds_eye = birds_eye.copy()
        cv2.drawContours(marked_birds_eye, contours, -1, (0, 0, 255), 2)  # 빨간색 윤곽선

        # ✅ 5. 이미지 90도 왼쪽 회전
        rotated_image = cv2.rotate(marked_birds_eye, cv2.ROTATE_90_COUNTERCLOCKWISE)

        # 6. 퍼블리시
        marked_msg = self.bridge.cv2_to_imgmsg(rotated_image, encoding='bgr8')
        marked_msg.header.stamp = msg.header.stamp
        marked_msg.header.frame_id = msg.header.frame_id
        self.birds_eye_pub.publish(marked_msg)

        # 7. 디버깅 출력
        cv2.imshow("Marked & Rotated Bird's Eye View", rotated_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = BirdsEyeAuto()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("종료됨.")
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()