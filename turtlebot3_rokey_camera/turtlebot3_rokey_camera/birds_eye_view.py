import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
import os

CONFIG_PATH = '/home/weed/team_f_01_ws/src/turtlebot3_rokey_camera/config/birds_eye_points.yaml'
clicked_points = []
done = False

def mouse_callback(event, x, y, flags, param):
    global clicked_points, done
    if event == cv2.EVENT_LBUTTONDOWN and len(clicked_points) < 4:
        clicked_points.append((x, y))
        print(f"Point {len(clicked_points)}: ({x}, {y})")
        if len(clicked_points) == 4:
            done = True

class BirdsEyeView(Node):
    def __init__(self):
        super().__init__('birds_eye_view')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, '/pi_camera/preprocessed', self.image_callback, 10)
        self.image_received = False
        self.latest_image = None

    def image_callback(self, msg):
        if not self.image_received:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.image_received = True

def save_points_to_yaml(points, path=CONFIG_PATH):
    data = {'src_points': [list(p) for p in points]}
    with open(path, 'w') as f:
        yaml.dump(data, f)
    print(f"✔ Saved to {path}")

def main(args=None):
    global clicked_points, done

    rclpy.init(args=args)
    node = BirdsEyeView()

    print("📷 Waiting for image...")
    while not node.image_received and rclpy.ok():
        rclpy.spin_once(node)

    image = node.latest_image.copy()
    clone = image.copy()

    print("🖱️ Click 4 points in clockwise order (starting from bottom-left)")
    cv2.namedWindow("Select 4 Points")
    cv2.setMouseCallback("Select 4 Points", mouse_callback)

    while not done:
        for pt in clicked_points:
            cv2.circle(clone, pt, 5, (0, 0, 255), -1)
        cv2.imshow("Select 4 Points", clone)
        if cv2.waitKey(10) == 27:  # ESC to cancel
            print("❌ Cancelled")
            cv2.destroyAllWindows()
            node.destroy_node()
            rclpy.shutdown()
            return

    cv2.destroyAllWindows()
    save_points_to_yaml(clicked_points)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
