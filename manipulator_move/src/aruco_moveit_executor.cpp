#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

class TargetPoseExecutor : public rclcpp::Node
{
public:
  TargetPoseExecutor(const rclcpp::NodeOptions& opt = rclcpp::NodeOptions())
  : Node("aruco_moveit_executor", opt),
    moveit_node_(std::make_shared<rclcpp::Node>("move_group_interface_node", opt)),
    move_group_(moveit_node_, "arm"),
    exec_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
  {
    /* ✅ MoveIt용 스레드 실행 */
    exec_->add_node(moveit_node_);
    exec_thread_ = std::thread([this] { exec_->spin(); });

    /* 🔧 플래너 파라미터 튜닝 (실패율 ↓) */
    move_group_.setPlanningTime(5.0);                 // default 5 s 이상 권장
    move_group_.setNumPlanningAttempts(10);           // 여러 번 시도
    move_group_.setGoalPositionTolerance(0.005);      // 5 mm
    move_group_.setGoalOrientationTolerance(0.02);    // ~1.15°

    /* 📨 구독자 */
    create_subscription<geometry_msgs::msg::PoseStamped>(
      "/base_pose_from_marker", 10,
      std::bind(&TargetPoseExecutor::poseCallback, this, std::placeholders::_1));
  }

  ~TargetPoseExecutor() override { exec_->cancel(); exec_thread_.join(); }

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped &msg)
  {
    /* 1️⃣ 현재 상태를 시작 상태로 고정 */
    move_group_.setStartStateToCurrentState();

    /* 2️⃣ Pose 타깃 설정 */
    move_group_.setPoseTarget(msg);

    /* 3️⃣ 플랜 */
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto ok = static_cast<bool>(move_group_.plan(plan));

    if (!ok)
    {
      RCLCPP_WARN(get_logger(), "❌ Planning failed – target unreachable?");
      return;
    }

    /* 4️⃣ 실행 */
    auto exec_ok = static_cast<bool>(move_group_.execute(plan));
    if (exec_ok)
      RCLCPP_INFO(get_logger(), "✅ Motion completed");
    else
      RCLCPP_WARN(get_logger(), "⚠️ Execution failed");
  }

  /* 멤버 */
  rclcpp::Node::SharedPtr moveit_node_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr exec_;
  std::thread exec_thread_;
};
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetPoseExecutor>());
  rclcpp::shutdown();
  return 0;
}
