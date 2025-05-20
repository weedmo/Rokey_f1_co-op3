#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class YellowEdgeNode(Node):
    def __init__(self):
        super().__init__('yellow_edge_node')
        self.bridge = CvBridge()
        # 구독: 원본 카메라 이미지
        self.subscription = self.create_subscription(
            Image,
            '/pi_camera/image_raw',
            self.image_callback,
            10)
        # 퍼블리시: 전처리된 이진 이미지
        self.publisher = self.create_publisher(
            Image,
            '/pi_camera/preprocessed',
            10)
        self.get_logger().info('🟢 YellowEdgeNode 시작됨')

    def image_callback(self, msg):
        # 1. ROS Image → OpenCV BGR
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 2. HSV로 변환 후 어두운 노란색 차선 마스크
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # 더 어두운 노란색을 포함하도록 S, V 임계치 낮춤
        lower_yellow = np.array([15,  50,  50])
        upper_yellow = np.array([35, 255, 255])
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # 3. 노이즈 제거 (Morphological Opening + Closing)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask_clean = cv2.morphologyEx(mask_yellow, cv2.MORPH_OPEN, kernel, iterations=1)
        mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, kernel, iterations=2)

        # 4. 에지 검출 (Canny)
        edges = cv2.Canny(mask_clean, 50, 150)

        # ✅ 에지를 굵게 만들기 위해 팽창(Dilate) 추가
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        edges_dilated = cv2.dilate(edges, kernel, iterations=1)

        # 5. 배경은 흰색, 에지 부분은 검은색 이진 이미지 생성
        height, width = edges.shape
        binary = 255 * np.ones((height, width), dtype=np.uint8)
        binary[edges_dilated > 0] = 0

        # 6. ROS Image로 변환 및 퍼블리시
        out_msg = self.bridge.cv2_to_imgmsg(binary, encoding='mono8')
        out_msg.header = msg.header
        self.publisher.publish(out_msg)

        # 7. 디버그 창 출력
        cv2.imshow('Dark Yellow Edge Preprocessing', binary)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = YellowEdgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 종료 요청 받음, 노드 중지')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()