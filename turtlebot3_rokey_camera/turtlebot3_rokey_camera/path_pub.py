#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

class LaneCenterPathNode(Node):
    def __init__(self):
        super().__init__('lane_center_path_node')
        self.bridge = CvBridge()

        # Subscribe to preprocessed lane mask
        self.create_subscription(Image, '/pi_camera/preprocessed', self.image_callback, 1)

        # Publishers
        self.path_pub = self.create_publisher(Path, '/lane_center_path', 1)
        self.debug_image_pub = self.create_publisher(Image, '/lane_center/debug', 1)

        # Example homography matrix (modify as needed)
        self.homography = np.array([[0.05, 0.0, -16.0],
                                    [0.0, 0.05, -12.0],
                                    [0.0, 0.0,   1.0]])

        self.get_logger().info("✅ LaneCenterPathNode started")

    def image_callback(self, msg: Image):
        # ROS image → OpenCV image (mono)
        gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        color_debug = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        # Skeleton
        skeleton = self.thinning(gray)

        # Extract skeleton points
        points = cv2.findNonZero(skeleton)
        if points is None:
            self.get_logger().warn("❌ No skeleton points found")
            return

        points = points.reshape(-1, 2)
        points = points[points[:, 1].argsort()[::-1]]  # sort by y (descending)

        # Path message
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"

        for (u, v) in points:
            pixel_coord = np.array([u, v, 1.0])
            world_coord = self.homography @ pixel_coord
            x, y = world_coord[0] / world_coord[2], world_coord[1] / world_coord[2]

            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

            # For visualization (blue circle)
            cv2.circle(color_debug, (u, v), 2, (255, 0, 0), -1)

        self.path_pub.publish(path_msg)

        # Publish debug image
        debug_msg = self.bridge.cv2_to_imgmsg(color_debug, encoding='bgr8')
        debug_msg.header = msg.header
        self.debug_image_pub.publish(debug_msg)

        self.get_logger().info(f"📤 Published path with {len(path_msg.poses)} poses")

    def thinning(self, img):
        """Morphological thinning to extract skeleton."""
        skel = np.zeros_like(img)
        kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
        while True:
            eroded = cv2.erode(img, kernel)
            temp = cv2.dilate(eroded, kernel)
            temp = cv2.subtract(img, temp)
            skel = cv2.bitwise_or(skel, temp)
            img = eroded.copy()
            if cv2.countNonZero(img) == 0:
                break
        return skel


def main(args=None):
    rclpy.init(args=args)
    node = LaneCenterPathNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
