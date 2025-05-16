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

        # Declare parameters with defaults
        self.declare_parameter('min_length', 5.0)
        self.declare_parameter('min_points', 1)
        self.declare_parameter('blur_ksize', 21)
        self.declare_parameter('adaptive_blocksize', 11)
        self.declare_parameter('adaptive_C', 0.1)

        # Load parameters
        self.min_length = self.get_parameter('min_length').value
        self.min_points = self.get_parameter('min_points').value
        self.blur_ksize = self.get_parameter('blur_ksize').value
        self.adaptive_blocksize = self.get_parameter('adaptive_blocksize').value
        self.adaptive_C = self.get_parameter('adaptive_C').value

        self.subscription = self.create_subscription(
            Image, '/pi_camera/image_raw', self.image_callback, 10)
        
        self.publisher = self.create_publisher(
            Image, '/pi_camera/preprocessed', 10)

        self.get_logger().info("✅ image_preprocessor 노드 시작됨!")

    def image_callback(self, msg):
        # Convert to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Illumination normalization
        blur = cv2.GaussianBlur(gray, (self.blur_ksize, self.blur_ksize), 0)
        norm = cv2.divide(gray, blur, scale=255)

        # Adaptive thresholding
        binary = cv2.adaptiveThreshold(
            norm, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            self.adaptive_blocksize, self.adaptive_C
        )

        # Median filter
        median = cv2.medianBlur(binary, 3)

        # Contour filtering
        contours, _ = cv2.findContours(median, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        mask = np.zeros_like(median)

        for contour in contours:
            arc_len = cv2.arcLength(contour, False)
            if arc_len > self.min_length and len(contour) > self.min_points:
                cv2.drawContours(mask, [contour], -1, 255, thickness=cv2.FILLED)

        # Publish result
        result_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
        result_msg.header.stamp = msg.header.stamp
        result_msg.header.frame_id = msg.header.frame_id
        self.publisher.publish(result_msg)

        # Debug view
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