#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

#include <memory>
#include <thread>
#include <iostream>

class RobotMover : public rclcpp::Node
{
public:
  RobotMover(const rclcpp::NodeOptions &options)
  : Node("robot_mover_main", options),
    // MoveIt 인터페이스용 Node에도 동일한 options 적용
    moveit_node_(std::make_shared<rclcpp::Node>("move_group_interface_node", options)),
    move_group_(moveit_node_, "arm"),
    executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
  {
    // MoveIt 전용 노드를 executor 에 등록
    executor_->add_node(moveit_node_);
    executor_thread_ = std::thread([this]() {
      RCLCPP_INFO(this->get_logger(), "▶ MoveGroupInterface executor thread 시작");
      executor_->spin();
    });

    RCLCPP_INFO(this->get_logger(), "✅ RobotMover 생성 완료");
  }

  ~RobotMover()
  {
    executor_->cancel();
    if (executor_thread_.joinable())
      executor_thread_.join();
  }

  void moveToPose()
  {
    // 목표 pose (base_link 기준)
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.25;
    target_pose.position.y = 0.0;
    target_pose.position.z = 0.15;
    target_pose.orientation.w = 1.0;

    move_group_.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "✅ 경로 계획 성공. 실행 중...");
      move_group_.move();
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "❌ 경로 계획 실패");
    }
  }

private:
  rclcpp::Node::SharedPtr moveit_node_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // ─── 여기서 use_sim_time 파라미터 선언 및 적용 ─────────────────────────────
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.append_parameter_override("use_sim_time", true);
  // ────────────────────────────────────────────────────────────────────────

  auto robot_node = std::make_shared<RobotMover>(node_options);

  // joint_states가 sim_time 기준으로 업데이트될 시간을 잠시 기다립니다.
  rclcpp::sleep_for(std::chrono::seconds(2));

  robot_node->moveToPose();

  rclcpp::shutdown();
  return 0;
}
