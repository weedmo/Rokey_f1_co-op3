#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import deque
import subprocess
from aruco_msgs.msg import Marker

class SobelCurveDirection(Node):
    def __init__(self):
        super().__init__('sobel_curve_direction')

        # 파라미터 선언 및 초기화
        self.declare_parameter('history_depth', 10)
        self.declare_parameter('score_threshold', 1500000)
        self.declare_parameter('max_ang', 0.25)
        self.declare_parameter('base_lin', 0.1)
        self.declare_parameter('gain', 0.00003)
        self.declare_parameter('lost_rotate_gain', 1.0)
        self.declare_parameter('obstacle_dist_threshold', 0.8)

        self.history_depth           = self.get_parameter('history_depth').value
        self.score_threshold         = self.get_parameter('score_threshold').value
        self.max_ang                 = self.get_parameter('max_ang').value
        self.base_lin                = self.get_parameter('base_lin').value
        self.gain                    = self.get_parameter('gain').value
        self.lost_rotate_gain        = self.get_parameter('lost_rotate_gain').value
        self.obstacle_dist_threshold = self.get_parameter('obstacle_dist_threshold').value

        # 내부 변수
        self.bridge        = CvBridge()
        self.cmd_history   = deque(maxlen=self.history_depth)
        self.lost_count    = 0
        self.obstacle_close = False
        self.multi_launched = False
        self.active         = True   # 자동주행 활성/비활성 플래그

        # Sobel 커널 정의
        self.kernels = {
            'vertical':   np.array([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]], dtype=np.float32),
            'diag_left':  np.array([[ 0, 1, 2], [-1, 0, 1], [-2,-1, 0]], dtype=np.float32),
            'diag_right': np.array([[ 2, 1, 0], [ 1, 0,-1], [ 0,-1,-2]], dtype=np.float32),
        }

        # 구독자: BEV, 라이다, ArUco pose
        self.create_subscription(Image,      '/pi_camera/birds_eye_view', self.image_callback, 10)
        self.create_subscription(LaserScan, '/scan_roi',               self.scan_callback,  10)
        self.create_subscription(Marker, '/id_client_pose',      self.aruco_callback, 10)
        self.aruco_sub = self.create_subscription(Marker, '/id_client_pose', self.aruco_callback, 10)
        # 퍼블리셔: 차선 방향, cmd_vel
        self.direction_pub = self.create_publisher(String, '/lane_direction', 10)
        self.cmd_pub       = self.create_publisher(Twist,  '/cmd_vel',       10)

        self.get_logger().info("✅ SobelCurveDirection 노드 시작됨")

    def scan_callback(self, scan: LaserScan):
        valid = [d for d in scan.ranges if scan.range_min < d < scan.range_max]
        self.obstacle_close = bool(valid and min(valid) < self.obstacle_dist_threshold)

    def image_callback(self, msg: Image):
        # 자동주행이 비활성화된 상태면 아무 것도 하지 않음
        if not self.active:
            return

        cmd = Twist()
        direction = '정지'

        # 1) 수동 조작 감지
        if self.count_publishers('/cmd_vel') >= 2:
            self.get_logger().info("❗️ 수동 조작 중 — 자동주행 차단")
            return

        # 2) 장애물 감지 시 정지
        if self.obstacle_close:
            direction = '장애물 감지: 멈춤'
            self.publish_and_log(direction, cmd)
            return

        # 3) 이미지 처리 & 스코어 계산
        gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        scores = {name: np.sum(np.abs(cv2.filter2D(gray, -1, kernel)))
                  for name, kernel in self.kernels.items()}
        total_score = sum(scores.values())

        # 4) 라인 상실 시 평균 회전
        if total_score < self.score_threshold:
            direction = '라인 상실: 평균 회전'
            self.lost_count += 1
            avg_ang = np.mean(self.cmd_history) if self.cmd_history else 0.0
            cmd.linear.x  = 0.05
            cmd.angular.z = avg_ang * self.lost_rotate_gain
            self.publish_and_log(direction, cmd, total_score)
            return

        # 5) 정상 추종
        self.lost_count = 0
        best  = max(scores, key=scores.get)
        delta = abs(scores['diag_left'] - scores['diag_right'])

        if best == 'vertical':
            direction     = '직진'
            cmd.linear.x  = self.base_lin
            cmd.angular.z = 0.0
        elif best == 'diag_left':
            direction     = '우회전'
            cmd.linear.x  = self.base_lin
            cmd.angular.z = -min(delta * self.gain, self.max_ang)
        else:  # diag_right
            direction     = '좌회전'
            cmd.linear.x  = self.base_lin
            cmd.angular.z =  min(delta * self.gain, self.max_ang)

        self.cmd_history.append(cmd.angular.z)
        self.publish_and_log(direction, cmd, total_score)

    def aruco_callback(self, pose: PoseStamped):
        if self.active:
            self.get_logger().info("📍 ArUco 감지 — 자동주행 중단, 후진 → mission_pose 실행")
            self.active = False  # 자동주행 일시 정지

            # 방향 메시지
            self.direction_pub.publish(String(data="ArUco 감지: 후진 중"))

            # 1. 1초간 뒤로 이동
            back_cmd = Twist()
            back_cmd.linear.x = -0.1  # 후진 속도 (m/s)
            self.cmd_pub.publish(back_cmd)

            # 1초 대기
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1.0))

            # 2. 정지a
            self.cmd_pub.publish(Twist())  # 멈춤
            self.direction_pub.publish(String(data="후진 완료 — mission_pose 실행"))

            # 3. mission_pose 실행
            try:
                subprocess.run([
                    "bash", "-c",
                    "source ~/rokeyracing_ws/install/setup.bash && ros2 run turtlebot3_manipulation_bringup mission_pose"
                ], check=True)
                self.get_logger().info("✅ mission_pose 종료됨 — 자동주행 재개")
            except subprocess.CalledProcessError:
                self.get_logger().error("❌ mission_pose 실행 중 오류 발생")
            self.destroy_subscription(self.aruco_sub)
            self.get_logger().info("🛑 더 이상 ArUco 콜백 안 받을 거예요")
            # 4. 자동주행 재개
            self.active = True



    def publish_and_log(self, direction: str, cmd: Twist, total_score: float = None):
        self.direction_pub.publish(String(data=direction))
        self.cmd_pub.publish(cmd)
        if total_score is not None:
            self.get_logger().info(
                f"[{direction}] lin: {cmd.linear.x:.2f}, ang: {cmd.angular.z:.2f} | "
                f"score: {total_score:.0f}, lost: {self.lost_count}"
            )
        else:
            self.get_logger().info(f"[{direction}] 명령 발행 (정지)")

def main(args=None):
    rclpy.init(args=args)
    node = SobelCurveDirection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
