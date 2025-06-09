#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

class MoveItTestNode : public rclcpp::Node
{
public:
  MoveItTestNode()
  : Node("moveit_test_node"),
    move_group_(std::shared_ptr<rclcpp::Node>(this), "arm")
  {
    RCLCPP_INFO(this->get_logger(), "MoveIt Test Node Started");

    move_group_.setPlanningTime(5.0);
    move_group_.setMaxVelocityScalingFactor(0.3);
    move_group_.setMaxAccelerationScalingFactor(0.3);

    // 1. 고정된 테스트 target pose (base_link 기준)
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.02;
    target_pose.position.y = -0.02;
    target_pose.position.z = 0.0;

    // 단순 orientation
    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 1.0;

    move_group_.setPoseTarget(target_pose);

    // 2. 계획 수립
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
      RCLCPP_INFO(this->get_logger(), "✅ Plan success. Executing...");
      move_group_.execute(plan);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "❌ Planning failed.");
    }
  }

private:
  moveit::planning_interface::MoveGroupInterface move_group_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveItTestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
