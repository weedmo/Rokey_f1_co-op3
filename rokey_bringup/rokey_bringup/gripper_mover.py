#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState

class GripperOpenNode(Node):
    def __init__(self):
        super().__init__('gripper_open_node')
        self.get_logger().info("🚀 Initializing MoveItPy for OpenManipulator-X Gripper")
        self.moveit = MoveItPy(node_name='moveit_py')
        self.gripper = self.moveit.get_planning_component('gripper')

    def open_gripper(self):
        # 현재 로봇 상태로부터 시작
        self.gripper.set_start_state_to_current_state()

        # 목표 상태 정의
        robot_model = self.moveit.get_robot_model()
        target_state = RobotState(robot_model)

        # 여기서 gripper_joint 는 SRDF에서 정의된 이름이어야 함
        # 예: OpenManipulator-X의 경우 보통 하나의 gripper_joint
        target_state.set_joint_group_positions('gripper', [0.01])  # 그리퍼 열기

        self.gripper.set_goal_state(robot_state=target_state)

        # 계획 수립
        plan_result = self.gripper.plan()
        if plan_result:
            self.get_logger().info("✅ Plan success. Executing gripper open trajectory")
            self.moveit.execute(plan_result.trajectory, controllers=['gripper_controller'])
        else:
            self.get_logger().error("❌ Planning failed. Cannot open gripper.")

def main(args=None):
    rclpy.init(args=args)
    node = GripperOpenNode()
    node.open_gripper()
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
