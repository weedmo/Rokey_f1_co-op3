#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
from cv2 import aruco
import numpy as np

def quaternion_from_euler(roll: float, pitch: float, yaw: float):
    """Roll-Pitch-Yaw 순서로 quaternion 생성"""
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    y = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    return np.array([x, y, z, w], dtype=np.float64)

def quaternion_matrix(q: np.ndarray):
    """quaternion → 4×4 homogeneous transform matrix"""
    x, y, z, w = q
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z

    M = np.array([
        [1 - 2*(yy+zz),     2*(xy - wz),     2*(xz + wy), 0],
        [    2*(xy + wz), 1 - 2*(xx+zz),     2*(yz - wx), 0],
        [    2*(xz - wy),     2*(yz + wx), 1 - 2*(xx+yy), 0],
        [              0,               0,               0, 1],
    ], dtype=np.float64)
    return M

def matrix_quaternion(M: np.ndarray):
    """4×4 homogeneous transform matrix → quaternion"""
    m = M[:3, :3]
    trace = m[0,0] + m[1,1] + m[2,2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m[2,1] - m[1,2]) * s
        y = (m[0,2] - m[2,0]) * s
        z = (m[1,0] - m[0,1]) * s
    else:
        if m[0,0] > m[1,1] and m[0,0] > m[2,2]:
            s = 2.0 * np.sqrt(1.0 + m[0,0] - m[1,1] - m[2,2])
            w = (m[2,1] - m[1,2]) / s
            x = 0.25 * s
            y = (m[0,1] + m[1,0]) / s
            z = (m[0,2] + m[2,0]) / s
        elif m[1,1] > m[2,2]:
            s = 2.0 * np.sqrt(1.0 + m[1,1] - m[0,0] - m[2,2])
            w = (m[0,2] - m[2,0]) / s
            x = (m[0,1] + m[1,0]) / s
            y = 0.25 * s
            z = (m[1,2] + m[2,1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + m[2,2] - m[0,0] - m[1,1])
            w = (m[1,0] - m[0,1]) / s
            x = (m[0,2] + m[2,0]) / s
            y = (m[1,2] + m[2,1]) / s
            z = 0.25 * s
    return np.array([x, y, z, w], dtype=np.float64)


class ArucoPosePublisher(Node):
    def __init__(self):
        super().__init__('aruco_pose_publisher')
        self.bridge = CvBridge()

        # 카메라 파라미터
        self.camera_matrix = None
        self.dist_coeffs = None

        # subscribers
        self.create_subscription(CameraInfo,
                                 '/pi_camera/camera_info',
                                 self.camera_info_callback, 10)
        self.create_subscription(CompressedImage,
                                 '/pi_camera/image_raw/compressed',
                                 self.image_callback, 10)

        # publisher
        self.base_pose_pub = self.create_publisher(
            PoseStamped, '/base_pose_from_marker', 10)

        # ArUco 세팅
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
        self.detector_params = aruco.DetectorParameters_create()
        self.target_id = 2
        self.marker_length = 0.05  # meter

        # 1) base_link ← camera_link 고정 변환
        q1 = quaternion_from_euler(0.0, 0.7, 0.0)        # RPY(0,0.7,0)
        T1 = quaternion_matrix(q1)
        T1[0:3, 3] = [0.0, 0.0, 0.2]                     # translate

        # 2) camera_link ← camera_rgb_optical_frame
        q2 = quaternion_from_euler(-1.57, 0.0, -1.57)    # RPY(-1.57,0,-1.57)
        T2 = quaternion_matrix(q2)
        # no additional translation

        # 전체: base_link ← optical_frame
        self.T_base_optical = T1 @ T2

        self.get_logger().info("✅ ArucoPosePublisher (pure NumPy) 시작")

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
        self.get_logger().info("CameraInfo 수신됨")

    def image_callback(self, msg: CompressedImage):
        if self.camera_matrix is None:
            return

        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.detector_params)
        if ids is None:
            return

        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners, self.marker_length,
            self.camera_matrix, self.dist_coeffs)

        for idx, m_id in enumerate(ids.flatten()):
            tvec = tvecs[idx][0]
            rvec = rvecs[idx][0]

            # 마커 기준 카메라(광학) 좌표계 → 4×4 행렬
            R_mc, _ = cv2.Rodrigues(rvec)
            T_mc = np.eye(4)
            T_mc[:3, :3] = R_mc
            T_mc[:3, 3] = tvec

            # 마커 기준 base_link pose
            T_mb = T_mc @ self.T_base_optical

            # PoseStamped 생성 (frame_id = aruco_marker_<id>)
            ps = PoseStamped()
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.header.frame_id = f'aruco_marker_{m_id}'

            trans = T_mb[:3, 3]
            quat = matrix_quaternion(T_mb)

            ps.pose.position.x = float(trans[0])
            ps.pose.position.y = float(trans[1])
            ps.pose.position.z = float(trans[2])
            ps.pose.orientation.x = float(quat[0])
            ps.pose.orientation.y = float(quat[1])
            ps.pose.orientation.z = float(quat[2])
            ps.pose.orientation.w = float(quat[3])

            self.base_pose_pub.publish(ps)
            self.get_logger().info(
                f"[ID {m_id}] base_link in marker-frame: {trans}, dist={np.linalg.norm(trans):.3f}m"
            )



def main(args=None):
    rclpy.init(args=args)
    node = ArucoPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
