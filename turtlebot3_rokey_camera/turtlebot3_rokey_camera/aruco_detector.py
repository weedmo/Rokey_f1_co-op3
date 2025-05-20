#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import CompressedImage  # ✅ 메시지 타입 변경
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.bridge = CvBridge()

        # ✅ CompressedImage 메시지 구독
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/pi_camera/image_raw/compressed',
            self.image_callback,
            10
        )

        self.id_pub = self.create_publisher(Int32, '/detected_marker_id', 10)

        self.get_logger().info("✅ ArUco Detector Node Started")

    def image_callback(self, msg):
        # ✅ Compressed 이미지 처리
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # ArUco 딕셔너리 및 파라미터 설정
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters()

        # 마커 감지
        detector = aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, _ = detector.detectMarkers(frame)

        if ids is not None:
            for marker_id in ids.flatten():
                self.get_logger().info(f"📸 Detected Marker ID: {marker_id}")
                id_msg = Int32()
                id_msg.data = int(marker_id) 
                self.id_pub.publish(id_msg)

        # 디버깅용 시각화 (옵션)
        # aruco.drawDetectedMarkers(frame, corners, ids)
        # cv2.imshow("ArUco Detection", frame)
        # cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
