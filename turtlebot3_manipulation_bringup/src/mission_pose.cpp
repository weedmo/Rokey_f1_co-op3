#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <memory>
#include <vector>
#include <chrono>

using namespace std::chrono_literals;

// 그리퍼 제어 함수
bool control_gripper(
  const std::shared_ptr<rclcpp::Node>& node,
  const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& gripper_group,
  const std::string& named_target)
{
  if (gripper_group->setNamedTarget(named_target)) {
    RCLCPP_INFO(node->get_logger(), "🔧 그리퍼 '%s' 상태로 이동 중...", named_target.c_str());
    gripper_group->move();
    return true;
  } else {
    RCLCPP_ERROR(node->get_logger(), "❌ '%s' 상태 설정 실패. SRDF 확인 필요!", named_target.c_str());
    return false;
  }
}

// 팔 이동 함수
bool move_to_joint_goal(
  const std::shared_ptr<rclcpp::Node>& node,
  const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& arm_group,
  const std::vector<double>& joint_goal)
{
  if (!arm_group->setJointValueTarget(joint_goal)) {
    RCLCPP_WARN(node->get_logger(), "⚠️ 목표 joint 값이 제한 범위를 초과했습니다.");
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (arm_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success) {
    RCLCPP_INFO(node->get_logger(), "✅ 경로 계획 성공! 실행 중...");
    arm_group->execute(plan);
    return true;
  } else {
    RCLCPP_ERROR(node->get_logger(), "❌ 경로 계획 실패!");
    return false;
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("mission_pose");

  auto gripper_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "gripper");
  auto arm_group     = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "arm");

  gripper_group->setPlanningTime(2.0);
  arm_group->setPlanningTime(2.0);
  gripper_group->setMaxVelocityScalingFactor(1.0);
  arm_group->setMaxVelocityScalingFactor(1.0);
  gripper_group->setMaxAccelerationScalingFactor(1.0);
  arm_group->setMaxAccelerationScalingFactor(1.0);

  // 1. 그리퍼 open
  control_gripper(node, gripper_group, "open");
  rclcpp::sleep_for(2s);

  // 2. 첫 번째 자세로 이동
  std::vector<double> joint_goal_1 = {0.0, 1.3265, -0.9425, -0.4189};
  move_to_joint_goal(node, arm_group, joint_goal_1);
  rclcpp::sleep_for(2s);

  // 3. 그리퍼 close
  control_gripper(node, gripper_group, "close");
  rclcpp::sleep_for(2s);

  // 4. 두 번째 자세로 이동
  std::vector<double> joint_goal_2 = {0.0, -0.1571, -0.7330, 2.0420};
  move_to_joint_goal(node, arm_group, joint_goal_2);
  rclcpp::sleep_for(2s);

  std::vector<double> joint_goal_3 = {1.0472, 1.3265, -0.9425, -0.4189};
  move_to_joint_goal(node, arm_group, joint_goal_3);
  rclcpp::sleep_for(2s);

  control_gripper(node, gripper_group, "open");
  rclcpp::sleep_for(2s);

  std::vector<double> joint_goal_4 = {0.0, -0.1571, -0.7330, 2.0420};
  move_to_joint_goal(node, arm_group, joint_goal_4);
  rclcpp::sleep_for(2s);

  control_gripper(node, gripper_group, "close");
  rclcpp::sleep_for(2s);

  RCLCPP_INFO(node->get_logger(), "🎉 전체 시퀀스 완료!");

  rclcpp::shutdown();
  return 0;
}
