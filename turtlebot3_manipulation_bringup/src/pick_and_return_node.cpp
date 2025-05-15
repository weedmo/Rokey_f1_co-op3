#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <chrono>

using namespace std::chrono_literals;

class PickAndReturnNode : public rclcpp::Node
{
public:
  PickAndReturnNode()
  : Node("pick_and_return_node"),
    move_group_(std::shared_ptr<rclcpp::Node>(this), "arm")
  {
    RCLCPP_INFO(this->get_logger(), "✅ Aruco Follower Node Started");

    move_group_.setPlanningTime(5.0);
    move_group_.setMaxVelocityScalingFactor(0.3);
    move_group_.setMaxAccelerationScalingFactor(0.3);

    // 현재 pose 저장 (복귀용)
    home_pose_ = move_group_.getCurrentPose().pose;
    RCLCPP_INFO(this->get_logger(), "🏠 Home pose saved.");

    // 구독자 설정
    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/aruco_goal_pose", 10,
      std::bind(&PickAndReturnNode::goalPoseCallback, this, std::placeholders::_1));
  }

private:
  void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "🎯 Received ArUco goal pose.");

    // 받은 목표 pose로 이동
    move_group_.setPoseTarget(msg->pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan_to_goal;

    if (move_group_.plan(plan_to_goal) == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "✅ Plan to ArUco pose success. Executing...");
      move_group_.execute(plan_to_goal);
      rclcpp::sleep_for(1s);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "❌ Failed to plan to ArUco pose.");
      return;
    }

    // 원래 위치로 복귀
    move_group_.setPoseTarget(home_pose_);
    moveit::planning_interface::MoveGroupInterface::Plan plan_to_home;

    if (move_group_.plan(plan_to_home) == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "↩️ Plan to home pose success. Executing...");
      move_group_.execute(plan_to_home);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "❌ Failed to plan to home pose.");
    }
  }

  moveit::planning_interface::MoveGroupInterface move_group_;
  geometry_msgs::msg::Pose home_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickAndReturnNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
