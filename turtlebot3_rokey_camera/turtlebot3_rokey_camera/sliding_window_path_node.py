#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import numpy as np
import cv2

class SlidingWindowPathNode(Node):
    def __init__(self):
        super().__init__('sliding_window_path_node')
        self.bridge = CvBridge()

        self.create_subscription(Image, '/pi_camera/preprocessed', self.image_callback, 1)
        self.path_pub = self.create_publisher(Path, '/lane_center_path', 1)
        self.debug_pub = self.create_publisher(Image, '/lane_center/debug', 1)

        # 예시 Homography (픽셀 → map 좌표)
        self.homography = np.array([[0.05, 0.0, -16.0],
                                    [0.0, 0.05, -12.0],
                                    [0.0, 0.0,   1.0]])

        self.get_logger().info("✅ SlidingWindowPathNode started")

    def image_callback(self, msg: Image):
        # mono8 이미지 수신
        mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        h, w = mask.shape
        debug = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        # 슬라이딩 윈도우 파라미터
        n_windows = 15
        window_height = int(h * 0.3 / n_windows)
        window_width = int(w * 0.2)
        margin = window_width // 2
        minpix = 50

        # 초기 중심: 하단 10% 평균
        bottom = mask[int(h*0.9):, :]
        hist = np.sum(bottom, axis=0)
        base_x = np.argmax(hist)

        center_points = []

        for window in range(n_windows):
            win_y_low = h - (window + 1) * window_height
            win_y_high = h - window * window_height
            win_x_low = base_x - margin
            win_x_high = base_x + margin

            # 이미지 경계 확인
            win_x_low = max(0, win_x_low)
            win_x_high = min(w, win_x_high)

            window_img = mask[win_y_low:win_y_high, win_x_low:win_x_high]
            nonzero = np.column_stack(np.nonzero(window_img))

            if len(nonzero) > minpix:
                mean_x = int(np.mean(nonzero[:, 1])) + win_x_low
                mean_y = int(np.mean(nonzero[:, 0])) + win_y_low
                base_x = mean_x
                center_points.append((mean_x, mean_y))

                cv2.rectangle(debug, (win_x_low, win_y_low), (win_x_high, win_y_high), (0,255,0), 2)
                cv2.circle(debug, (mean_x, mean_y), 4, (255,0,0), -1)

        if len(center_points) < 3:
            self.get_logger().warn("❌ 중심점을 충분히 찾지 못했습니다")
            return

        # Path 생성
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "odom"

        for u, v in center_points:
            pix = np.array([u, v, 1.0])
            wc = self.homography @ pix
            x, y = wc[0]/wc[2], wc[1]/wc[2]

            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)

        self.path_pub.publish(path)
        self.get_logger().info(f"📤 Published path with {len(path.poses)} points")

        debug_msg = self.bridge.cv2_to_imgmsg(debug, 'bgr8')
        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SlidingWindowPathNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
