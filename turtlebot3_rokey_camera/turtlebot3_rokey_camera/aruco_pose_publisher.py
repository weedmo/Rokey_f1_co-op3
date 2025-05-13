#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage
from geometry_msgs.msg import PoseStamped

import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge
import numpy as np


class ArucoPosePublisher(Node):
    def __init__(self):
        super().__init__('aruco_pose_publisher')
        self.bridge = CvBridge()

        # 카메라 파라미터
        self.camera_matrix = None
        self.dist_coeffs = None

        # 구독자
        self.create_subscription(CameraInfo, '/pi_camera/camera_info', self.camera_info_callback, 10)
        self.create_subscription(CompressedImage, '/pi_camera/image_raw/compressed', self.image_callback, 10)

        # 퍼블리셔
        self.pose_pub = self.create_publisher(PoseStamped, 'aruco_marker_pose', 10)

        # ArUco 설정
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
        self.detector_params = aruco.DetectorParameters()

        # 추적할 마커 ID 및 실제 길이
        self.target_id = 2
        self.marker_length = 0.05  # meter

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
        self.get_logger().info('Camera calibration parameters received.')

    def image_callback(self, msg):
        if self.camera_matrix is None or self.dist_coeffs is None:
            return

        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.detector_params)

        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs)

            for i, marker_id in enumerate(ids.flatten()):
                if marker_id != self.target_id:
                    continue

                tvec = tvecs[i][0]  # marker 위치 (optical frame 기준)
                rvec = rvecs[i][0]

                # 메시지 생성 및 발행
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = 'camera_rgb_optical_frame'  # 카메라 기준

                pose_msg.pose.position.x = tvec[0]
                pose_msg.pose.position.y = tvec[1]
                pose_msg.pose.position.z = tvec[2]

                # yaw 추정 (회전벡터는 사용 안 함)
                yaw = np.arctan2(tvec[0], tvec[2])
                pose_msg.pose.orientation.z = np.sin(yaw / 2.0)
                pose_msg.pose.orientation.w = np.cos(yaw / 2.0)

                self.pose_pub.publish(pose_msg)
                self.get_logger().info(
                    f"[ID {marker_id}] Marker (camera 기준): {tvec}, 거리={np.linalg.norm(tvec):.3f}m"
                )


def main(args=None):
    rclpy.init(args=args)
    node = ArucoPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
