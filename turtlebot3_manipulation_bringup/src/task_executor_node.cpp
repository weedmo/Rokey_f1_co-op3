#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <control_msgs/action/gripper_command.hpp>

#include <memory>
#include <vector>
#include <thread>
#include <chrono>

using GripperCommand = control_msgs::action::GripperCommand;

class TaskExecutor : public rclcpp::Node,
                     public std::enable_shared_from_this<TaskExecutor>
{
public:
  TaskExecutor()
  : Node("task_executor")
  {
    RCLCPP_INFO(this->get_logger(), "🟢 TaskExecutor Node Started");

    // 그리퍼 액션 클라이언트 준비
    gripper_client_ = rclcpp_action::create_client<GripperCommand>(
      this, "gripper_controller/gripper_cmd");

    // 별도 스레드에서 MoveGroup 초기화 후 시퀀스 실행
    std::thread(&TaskExecutor::initializeAndRun, this).detach();
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;

  void initializeAndRun()
  {
    // node가 shared_from_this 사용 가능할 때까지 대기
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // MoveGroupInterface 생성 (arm planning group)
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
  std::enable_shared_from_this<TaskExecutor>::shared_from_this(), "arm");

    // 시퀀스 실행
    runSequence();
  }

  void runSequence()
  {
    const std::vector<double> pose2 = {0.0, -0.1571, -0.7330, 2.0420};
    const std::vector<double> pose3 = {0.0,  1.3265, -0.9425, -0.4189};

    // 1) 그리퍼 열기
    if (!sendGripperAndWait(0.010)) return;

    // 2) 위치2로 이동
    moveToJointPosition(pose2);

    // 3) 그리퍼 닫기
    if (!sendGripperAndWait(0.018)) return;

    // 4) 위치3으로 이동
    moveToJointPosition(pose3);

    RCLCPP_INFO(this->get_logger(), "🎉 모든 작업 완료!");
  }

  bool sendGripperAndWait(double position)
  {
    // 액션 서버 대기
    if (!gripper_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "❌ Gripper action server 사용 불가");
      return false;
    }

    // goal 생성
    auto goal_msg = GripperCommand::Goal();
    goal_msg.command.position = position;
    goal_msg.command.max_effort = 5.0;

    RCLCPP_INFO(this->get_logger(), "📨 그리퍼 목표 전송: %.3f", position);

    // goal 전송
    auto goal_handle_future = gripper_client_->async_send_goal(goal_msg);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future)
        != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "❌ 그리퍼 목표 전송 실패");
      return false;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "❌ 유효하지 않은 goal handle");
      return false;
    }

    // 결과 대기
    auto result_future = gripper_client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future)
        != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "❌ 그리퍼 결과 수신 실패");
      return false;
    }

    auto result = result_future.get();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_WARN(this->get_logger(), "⚠️ 그리퍼 동작 실패");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "✅ 그리퍼 위치 %.3f 도달", position);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return true;
  }

  void moveToJointPosition(const std::vector<double>& joint_values)
  {
    move_group_->setStartState(*move_group_->getCurrentState());
    move_group_->setJointValueTarget(joint_values);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "✅ 팔 이동 계획 성공. 실행 중...");
      move_group_->execute(plan);
    } else {
      RCLCPP_ERROR(this->get_logger(), "❌ 팔 경로 계획 실패");
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TaskExecutor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
