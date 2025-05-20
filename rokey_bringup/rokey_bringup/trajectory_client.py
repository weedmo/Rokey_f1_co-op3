#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class ArmAndGripperClient(Node):
    def __init__(self):
        super().__init__('arm_and_gripper_client')

        # 액션 클라이언트 초기화
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')

        self.arm_sent = False
        self.gripper_sent = False

        # 1초마다 goal 시도
        self.timer = self.create_timer(1.0, self.send_goals_once)

    def send_goals_once(self):
        if not self.arm_sent and self.arm_client.wait_for_server(timeout_sec=1.0):
            self.arm_sent = True
            self.send_arm_goal()

        if not self.gripper_sent and self.gripper_client.wait_for_server(timeout_sec=1.0):
            self.gripper_sent = True
            self.send_gripper_goal(open_gripper=False)  # True면 열기, False면 닫기

    def send_arm_goal(self):
        traj = JointTrajectory()
        traj.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        point = JointTrajectoryPoint()
        point.positions = [1.57, 1.57, 0.0, 0.0]
        point.time_from_start = Duration(sec=2)

        traj.points.append(point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        self.get_logger().info("🦾 Sending arm goal...")
        self.arm_client.send_goal_async(goal).add_done_callback(self.arm_goal_response_callback)

    def arm_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Arm goal rejected.')
            return
        self.get_logger().info('✅ Arm goal accepted.')
        goal_handle.get_result_async().add_done_callback(self.arm_result_callback)

    def arm_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('🎯 Arm movement done.')

    def send_gripper_goal(self, open_gripper=True):
        goal = GripperCommand.Goal()
        if open_gripper:
            goal.command.position = 1.57  # 열린 상태
        else:
            goal.command.position = 0.0   # 닫힌 상태

        goal.command.max_effort = 1.0  # 필요한 경우 설정

        self.get_logger().info("✊ Sending gripper goal...")
        self.gripper_client.send_goal_async(goal).add_done_callback(self.gripper_goal_response_callback)

    def gripper_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Gripper goal rejected.')
            return
        self.get_logger().info('✅ Gripper goal accepted.')
        goal_handle.get_result_async().add_done_callback(self.gripper_result_callback)

    def gripper_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('🤝 Gripper movement done.')

def main(args=None):
    rclpy.init(args=args)
    node = ArmAndGripperClient()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
