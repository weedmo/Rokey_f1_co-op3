#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImagePreprocessor(Node):
    def __init__(self):
        super().__init__('image_preprocessor')
        self.bridge = CvBridge()

        # ✅ 이미지 구독 및 결과 퍼블리셔 설정
        self.subscription = self.create_subscription(
            Image, '/pi_camera/image_raw', self.image_callback, 10)
        
        self.publisher = self.create_publisher(
            Image, '/pi_camera/preprocessed', 10)

        self.get_logger().info("✅ image_preprocessor 노드 시작됨!")

    def image_callback(self, msg):
        # 1. 이미지 수신 및 OpenCV 이미지로 변환
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 2. 흑백 변환
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # 3. 조명 제거
        blur = cv2.GaussianBlur(gray, (21, 21), 0)
        norm = cv2.divide(gray, blur, scale=255)

        # 4. 이진 변환
        binary = cv2.adaptiveThreshold(
            norm, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            11, 0.1
        )

        # 5. 중위수 필터
        median = cv2.medianBlur(binary, 3)

        # ✅ 6. 연결된 선 또는 곡선이 일정 길이 이상인 것만 추출
        contours, _ = cv2.findContours(median, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        min_length = 5
        min_points = 1  # 곡선 점 개수 기준 추가
        mask = np.zeros_like(median)

        for contour in contours:
            arc_len = cv2.arcLength(contour, False)
            if arc_len > min_length and len(contour) > min_points:
                cv2.drawContours(mask, [contour], -1, 255, thickness=cv2.FILLED)


        # 7. 퍼블리시
        result_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
        result_msg.header.stamp = msg.header.stamp
        result_msg.header.frame_id = msg.header.frame_id
        self.publisher.publish(result_msg)

        # 디버깅용 OpenCV 뷰
        cv2.imshow("Preprocessed Long-Line Mask", mask)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ImagePreprocessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("종료됨.")
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()