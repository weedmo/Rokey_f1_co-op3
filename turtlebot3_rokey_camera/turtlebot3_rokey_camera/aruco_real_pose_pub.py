#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np

class ArucoPoseDetector(Node):
    def __init__(self):
        super().__init__('aruco_real_pose_pub')
        self.bridge = CvBridge()

        # ✅ 카메라 캘리브레이션 파라미터 (자신의 값으로 교체할 것)
        self.camera_matrix = np.array([
            [640.0,   0.0, 320.0],
            [  0.0, 640.0, 240.0],
            [  0.0,   0.0,   1.0]
        ])
        self.dist_coeffs = np.zeros((5, 1))  # 왜곡 없는 기본값

        # ✅ 마커 한 변 길이 (단위: m)
        self.marker_length = 0.05

        self.image_sub = self.create_subscription(
            CompressedImage,
            '/pi_camera/image_raw/compressed',
            self.image_callback,
            10
        )

        self.id_pub = self.create_publisher(Int32, '/detected_marker_id', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/detected_marker_pose', 10)

        self.get_logger().info("✅ ArUco Pose Detector Node Started")

    def image_callback(self, msg):
        # ✅ 이미지 디코딩
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # ✅ 마커 감지
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, _ = detector.detectMarkers(frame)

        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                self.get_logger().info(f"📸 ID: {marker_id}")

                # ✅ 3D 마커 좌표 (왼위부터 시계 방향)
                half_len = self.marker_length / 2.0
                object_points = np.array([
                    [-half_len,  half_len, 0],
                    [ half_len,  half_len, 0],
                    [ half_len, -half_len, 0],
                    [-half_len, -half_len, 0]
                ], dtype=np.float32)

                # ✅ solvePnP로 Pose 추정
                success, rvec, tvec = cv2.solvePnP(
                    object_points,
                    corners[i][0],
                    self.camera_matrix,
                    self.dist_coeffs
                )

                if not success:
                    self.get_logger().warn(f"⚠ solvePnP 실패 for ID {marker_id}")
                    continue

                # ✅ ID 퍼블리시
                id_msg = Int32()
                id_msg.data = int(marker_id)
                self.id_pub.publish(id_msg)

                # ✅ PoseStamped 퍼블리시
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "camera_link"

                pose_msg.pose.position.x = float(tvec[0])
                pose_msg.pose.position.y = float(tvec[1])
                pose_msg.pose.position.z = float(tvec[2])

                # ✅ 회전벡터 → 회전행렬 → 쿼터니언
                R, _ = cv2.Rodrigues(rvec)
                q = self.rotation_matrix_to_quaternion(R)

                pose_msg.pose.orientation.x = q[0]
                pose_msg.pose.orientation.y = q[1]
                pose_msg.pose.orientation.z = q[2]
                pose_msg.pose.orientation.w = q[3]

                self.pose_pub.publish(pose_msg)

    def rotation_matrix_to_quaternion(self, R):
        """3x3 회전 행렬 → [x, y, z, w] 쿼터니언"""
        q = np.empty((4, ))
        trace = np.trace(R)
        if trace > 0.0:
            s = np.sqrt(trace + 1.0) * 2
            q[3] = 0.25 * s
            q[0] = (R[2, 1] - R[1, 2]) / s
            q[1] = (R[0, 2] - R[2, 0]) / s
            q[2] = (R[1, 0] - R[0, 1]) / s
        else:
            i = np.argmax(np.diagonal(R))
            if i == 0:
                s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
                q[0] = 0.25 * s
                q[1] = (R[0, 1] + R[1, 0]) / s
                q[2] = (R[0, 2] + R[2, 0]) / s
                q[3] = (R[2, 1] - R[1, 2]) / s
            elif i == 1:
                s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
                q[0] = (R[0, 1] + R[1, 0]) / s
                q[1] = 0.25 * s
                q[2] = (R[1, 2] + R[2, 1]) / s
                q[3] = (R[0, 2] - R[2, 0]) / s
            else:
                s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
                q[0] = (R[0, 2] + R[2, 0]) / s
                q[1] = (R[1, 2] + R[2, 1]) / s
                q[2] = 0.25 * s
                q[3] = (R[1, 0] - R[0, 1]) / s
        return q


def main(args=None):
    rclpy.init(args=args)
    node = ArucoPoseDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()