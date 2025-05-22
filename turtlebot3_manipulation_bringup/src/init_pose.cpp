#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <memory>
#include <vector>

void move_robot(const std::shared_ptr<rclcpp::Node>& node)
{
  // ✅ 1. arm MoveGroupInterface 생성
  auto arm_move_group = moveit::planning_interface::MoveGroupInterface(node, "arm");

  // ✅ 2. 목표 joint 값 설정
  std::vector<double> arm_joint_goal = {
    0.0,       // joint1
    -0.1571,   // joint2
    -0.7330,   // joint3
    2.0420     // joint4
  };

  bool within_bounds = arm_move_group.setJointValueTarget(arm_joint_goal);

  if (!within_bounds) {
    RCLCPP_WARN(node->get_logger(),
                "❗ 목표 joint 값이 제한 범위를 초과하였습니다. 클램핑 처리 예정.");
    return;
  }

  // ✅ 3. 경로 계획 및 실행
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool plan_success = (arm_move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (plan_success) {
    RCLCPP_INFO(node->get_logger(), "✅ 경로 생성 성공! 로봇 암 이동 시작.");
    arm_move_group.move();
  } else {
    RCLCPP_ERROR(node->get_logger(), "❌ 경로 계획 실패!");
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("init_pose");

  move_robot(node);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
