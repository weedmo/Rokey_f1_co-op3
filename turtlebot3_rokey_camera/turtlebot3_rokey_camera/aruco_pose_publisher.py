#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Int32
from aruco_msgs.msg import Marker
from cv_bridge import CvBridge
import yaml
import os


def load_camera_parameters(yaml_path):
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
        camera_matrix = np.array(data["camera_matrix"]["data"], dtype=np.float32).reshape(3, 3)
        dist_coeffs = np.array(data["distortion_coefficients"]["data"], dtype=np.float32)
    return camera_matrix, dist_coeffs


def rotation_matrix_to_euler(R):
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    singular = sy < 1e-6
    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0
    return np.degrees([x, y, z])


class ArucoPosePublisher(Node):
    def __init__(self):
        super().__init__('optimized_aruco_pose_publisher')

        # Subscribers & Publishers
        self.subscription = self.create_subscription(
            CompressedImage,
            '/pi_camera/image_raw/compressed',
            self.image_callback,
            10
        )
        self.pose_pub = self.create_publisher(Marker, '/id_client_pose', 10)
        self.image_pub = self.create_publisher(Image, '/aruco/annotated_image', 10)

        # Bridge & parameters
        self.bridge = CvBridge()
        self.marker_size = 0.04  # meters
        yaml_path = os.path.expanduser(
            '~/rokeyracing_ws/src/Rokey_f1_co-op3/turtlebot3_rokey_camera/config/calibration_params.yaml'
        )
        self.camera_matrix, self.dist_coeffs = load_camera_parameters(yaml_path)

        # Precompute object points for a square marker
        half = self.marker_size / 2
        self.obj_points = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0]
        ], dtype=np.float32)

        # ArUco detection setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
        self.det_params = cv2.aruco.DetectorParameters()
        # Improve detection robustness
        self.det_params.adaptiveThreshWinSizeMin = 7
        self.det_params.adaptiveThreshWinSizeMax = 23
        self.det_params.adaptiveThreshWinSizeStep = 10
        self.det_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.det_params.minMarkerPerimeterRate = 0.03
        self.det_params.maxMarkerPerimeterRate = 4.0

        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.det_params)
        self.get_logger().info("✅ Optimized ArUco Pose Publisher Started")

    def image_callback(self, msg):
        # Decode image
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is not None:
            # Refine and draw markers
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            for idx, c in enumerate(corners):
                marker_id = int(ids[idx][0])
                # Solve for pose
                ok, rvec, tvec = cv2.solvePnP(
                    self.obj_points, c.reshape(-1, 2),
                    self.camera_matrix, self.dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )
                if not ok:
                    self.get_logger().warn(f"solvePnP failed for ID {marker_id}")
                    continue

                # Convert and draw axis
                rot_mat, _ = cv2.Rodrigues(rvec)
                yaw, pitch, roll = rotation_matrix_to_euler(rot_mat)
                cv2.drawFrameAxes(
                    frame, self.camera_matrix, self.dist_coeffs,
                    rvec, tvec, self.marker_size * 0.5
                )

                # Publish pose
                pos = tvec.flatten()
                marker_msg = Marker()
                marker_msg.id = marker_id
                marker_msg.pose.pose.position.x = float(pos[0])
                marker_msg.pose.pose.position.y = float(pos[1])
                marker_msg.pose.pose.position.z = float(pos[2])
                marker_msg.pose.pose.orientation.x = float(roll)
                marker_msg.pose.pose.orientation.y = float(pitch)
                marker_msg.pose.pose.orientation.z = float(yaw)
                self.pose_pub.publish(marker_msg)

                self.get_logger().info(
                    f"ID:{marker_id} Pos:{pos.round(3)} YPR:({yaw:.1f},{pitch:.1f},{roll:.1f})"
                )

        # Publish annotated image
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.image_pub.publish(img_msg)

    def destroy_node(self):
        # cleanup
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()