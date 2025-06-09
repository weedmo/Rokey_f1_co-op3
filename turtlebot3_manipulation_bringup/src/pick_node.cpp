#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>

class ArucoPickNode : public rclcpp::Node
{
public:
  ArucoPickNode()
  : Node("pick_node"),
    move_group_(std::shared_ptr<rclcpp::Node>(this), "arm"),
    initial_pose_set_(false)
  {
    aruco_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/aruco_marker_pose_base", 10,
      std::bind(&ArucoPickNode::arucoCallback, this, std::placeholders::_1));

    move_group_.setPlanningTime(5.0);
    move_group_.setMaxVelocityScalingFactor(0.3);
    move_group_.setMaxAccelerationScalingFactor(0.3);

    RCLCPP_INFO(this->get_logger(), "Aruco Pick Node Started.");
  }

private:
  void arucoCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!initial_pose_set_)
    {
      initial_pose_ = move_group_.getCurrentPose().pose;
      initial_pose_set_ = true;
      RCLCPP_INFO(this->get_logger(), "Initial pose saved.");
    }

    RCLCPP_INFO(this->get_logger(), "Aruco Marker Pose Received");

    geometry_msgs::msg::PoseStamped marker_pose = *msg;

    // STEP 1: 접근 pose (z 위에서 접근)
    geometry_msgs::msg::Pose approach_pose = marker_pose.pose;
    approach_pose.position.z += 0.10;

    if (!planAndExecute(approach_pose, "Approach"))
      return;

    // STEP 2: 실제 집기 위치 (내려감)
    geometry_msgs::msg::Pose grasp_pose = marker_pose.pose;
    grasp_pose.position.z += 0.02;

    if (!planAndExecute(grasp_pose, "Grasp"))
      return;

    // STEP 3: 다시 접근 위치로 올라감
    if (!planAndExecute(approach_pose, "Lift"))
      return;

    // STEP 4: 초기 위치로 복귀
    if (!planAndExecute(initial_pose_, "Return to initial"))
      return;

    RCLCPP_INFO(this->get_logger(), "Pick and place completed.");
  }

  bool planAndExecute(const geometry_msgs::msg::Pose& pose, const std::string& label)
  {
    move_group_.setPoseTarget(pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    if (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "[%s] Plan successful. Executing...", label.c_str());
      move_group_.execute(plan);
      return true;
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "[%s] Plan failed.", label.c_str());
      return false;
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_sub_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  geometry_msgs::msg::Pose initial_pose_;
  bool initial_pose_set_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArucoPickNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
