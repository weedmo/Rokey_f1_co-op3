#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class ArucoPoseListener : public rclcpp::Node
{
public:
  ArucoPoseListener()
  : Node("aruco_pose_listener"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // 구독자: ArUco 마커 pose (camera 기준)
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/aruco_marker_pose",
      10,
      std::bind(&ArucoPoseListener::pose_callback, this, std::placeholders::_1));

    // 퍼블리셔: base_link 기준으로 변환된 pose
    base_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/aruco_marker_pose_base", 10);

    RCLCPP_INFO(this->get_logger(), "ArucoPoseListener started. Transforming to base_link...");
  }

private:
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped transformed_pose;

    try
    {
      // transform camera_rgb_optical_frame → base_link
      tf_buffer_.transform(*msg, transformed_pose, "base_link", tf2::durationFromSec(0.5));

      RCLCPP_INFO(this->get_logger(),
        "Transformed Marker Pose (base_link): [%.3f, %.3f, %.3f]",
        transformed_pose.pose.position.x,
        transformed_pose.pose.position.y,
        transformed_pose.pose.position.z);

      base_pose_pub_->publish(transformed_pose);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr base_pose_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArucoPoseListener>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
