#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class ArucoMoveitExecutor : public rclcpp::Node
{
public:
  ArucoMoveitExecutor()
  : Node("pick_node"),
    move_group_(std::make_shared<rclcpp::Node>("move_group_interface_node"), "arm")  // 🔁 그룹 이름은 본인 설정에 맞게 수정
  {
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/base_pose_from_marker", 10,
      std::bind(&ArucoMoveitExecutor::pose_callback, this, std::placeholders::_1));

    trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/execute_pose_trigger", 10,
      std::bind(&ArucoMoveitExecutor::trigger_callback, this, std::placeholders::_1));

    // 옵션: plan 안정성을 위한 설정
    move_group_.setPlanningTime(5.0);
    move_group_.setGoalPositionTolerance(0.01);
    move_group_.setGoalOrientationTolerance(0.05);

    RCLCPP_INFO(this->get_logger(), "✅ Aruco MoveIt Executor 노드 시작됨");
  }

private:
  moveit::planning_interface::MoveGroupInterface move_group_;
  geometry_msgs::msg::Pose latest_pose_;
  bool pose_received_ = false;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_sub_;

  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    latest_pose_ = msg->pose;
    pose_received_ = true;
    RCLCPP_INFO(this->get_logger(), "📥 Pose 수신됨 (x=%.3f, y=%.3f, z=%.3f)",
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  }

  void trigger_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!pose_received_) {
      RCLCPP_WARN(this->get_logger(), "❌ 아직 pose 수신 전입니다.");
      return;
    }

    if (!msg->data) {
      RCLCPP_INFO(this->get_logger(), "⚠️ Trigger 메시지가 false입니다. 무시합니다.");
      return;
    }

    move_group_.clearPoseTargets();  // 이전 목표 제거
    move_group_.setPoseTarget(latest_pose_);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      RCLCPP_INFO(this->get_logger(), "✅ Plan 성공, 실행 시작!");
      move_group_.execute(plan);
    } else {
      RCLCPP_ERROR(this->get_logger(), "❌ 경로 계획 실패!");
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArucoMoveitExecutor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
