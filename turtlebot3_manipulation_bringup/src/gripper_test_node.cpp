#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <memory>
#include <vector>
#include <thread>
#include <chrono>

int main(int argc, char **argv)
{
  // ROS 2 초기화
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("gripper_test_node");

  // Gripper용 MoveGroupInterface 생성
  moveit::planning_interface::MoveGroupInterface gripper_group(node, "gripper");

  // 현재 상태 모니터 시작
  gripper_group.startStateMonitor();

  // joint_states 수신 대기
  RCLCPP_INFO(node->get_logger(), "⌛ joint_states 수신 대기 중...");
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // 현재 상태를 시작 상태로 설정 (안전하고 간단한 방식)
  gripper_group.setStartStateToCurrentState();

  // 목표 조인트 위치 설정 (그리퍼 열기)
  std::vector<double> gripper_open = {0.010};
  gripper_group.setJointValueTarget(gripper_open);

  // 계획 및 실행
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (gripper_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_INFO(node->get_logger(), "✅ 계획 성공. 실행 중...");
    gripper_group.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "❌ 계획 실패");
  }

  rclcpp::shutdown();
  return 0;
}
