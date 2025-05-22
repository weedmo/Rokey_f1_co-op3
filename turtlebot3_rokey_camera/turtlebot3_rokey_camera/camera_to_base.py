#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from aruco_msgs.msg import Marker
import numpy as np
from tf_transformations import quaternion_matrix, quaternion_from_matrix

class ArucoPoseToBaseLink(Node):
    def __init__(self):
        super().__init__('camera_to_base')

        self.subscription = self.create_subscription(
            Marker,
            '/id_client_pose',
            self.aruco_callback,
            10
        )

        self.pub = self.create_publisher(Pose, '/camera_to_base', 10)

        # 고정된 카메라 → base_link 변환
        self.T_base_camera = self.get_base_camera_transform()

        self.get_logger().info("✅ ArUco pose transformer started (Pose-only publisher)")

    def get_base_camera_transform(self):
        # rotation quaternion [x, y, z, w]
        q = [0.000, 0.555, -0.001, 0.832]
        t = [0.004, 0.0, 0.392]

        T = quaternion_matrix(q)  # 4x4
        T[0:3, 3] = t
        return T

    def pose_to_matrix(self, pose: Pose):
        q = [pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w]
        t = [pose.position.x, pose.position.y, pose.position.z]
        T = quaternion_matrix(q)
        T[0:3, 3] = t
        return T

    def matrix_to_pose(self, T: np.ndarray) -> Pose:
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = T[0:3, 3]
        q = quaternion_from_matrix(T)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
        return pose

    def aruco_callback(self, msg: Marker):
        # T_camera_marker
        T_cam_marker = self.pose_to_matrix(msg.pose.pose)

        # T_base_marker = T_base_camera × T_camera_marker
        T_base_marker = self.T_base_camera @ T_cam_marker

        pose_in_base = self.matrix_to_pose(T_base_marker)

        self.pub.publish(pose_in_base)
        self.get_logger().info("📤 Published geometry_msgs::Pose (base_link 기준)")

def main(args=None):
    rclpy.init(args=args)
    node = ArucoPoseToBaseLink()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
