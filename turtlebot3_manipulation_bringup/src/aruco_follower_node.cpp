#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <aruco_msgs/msg/marker.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class ArucoFollowerNode : public rclcpp::Node
{
public:
  ArucoFollowerNode()
  : Node("aruco_follower_node"),
    move_group_(std::shared_ptr<rclcpp::Node>(this), "arm"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    RCLCPP_INFO(this->get_logger(), "✅ ArUco Follower Node Started");

    move_group_.setPlanningTime(5.0);
    move_group_.setMaxVelocityScalingFactor(0.3);
    move_group_.setMaxAccelerationScalingFactor(0.3);

    // ArUco pose 구독
    marker_sub_ = this->create_subscription<aruco_msgs::msg::Marker>(
      "/id_client_pose", 10,
      std::bind(&ArucoFollowerNode::arucoCallback, this, std::placeholders::_1));
  }

private:
  void arucoCallback(const aruco_msgs::msg::Marker::SharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped marker_pose;
    marker_pose.header.frame_id = "camera_link";  // 카메라 기준 좌표계
    marker_pose.pose = msg->pose.pose;

    geometry_msgs::msg::PoseStamped base_pose;

    try
    {
      // ✅ camera_link → base_link로 변환
      base_pose = tf_buffer_.transform(marker_pose, "base_link", tf2::durationFromSec(0.5));
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "❌ TF transform failed: %s", ex.what());
      return;
    }

    move_group_.setPoseTarget(base_pose.pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
      RCLCPP_INFO(this->get_logger(), "📦 Moving to ArUco pose...");
      move_group_.execute(plan);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "❌ Failed to plan to ArUco pose.");
    }
  }

  moveit::planning_interface::MoveGroupInterface move_group_;
  rclcpp::Subscription<aruco_msgs::msg::Marker>::SharedPtr marker_sub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArucoFollowerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
