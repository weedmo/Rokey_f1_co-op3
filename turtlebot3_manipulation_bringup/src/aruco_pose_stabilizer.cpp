#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>
#include <deque>

class ArucoPoseStabilizer : public rclcpp::Node
{
public:
  ArucoPoseStabilizer()
  : Node("aruco_pose_stabilizer"),
    stable_(false)
  {
    sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/aruco_marker_pose_base", 10,
      std::bind(&ArucoPoseStabilizer::poseCallback, this, std::placeholders::_1));

    pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/aruco_goal_pose", 10);

    RCLCPP_INFO(this->get_logger(), "ArucoPoseStabilizer node started.");
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (stable_)
      return;

    const auto &p = msg->pose.position;

    // 최신 위치를 큐에 저장 (최대 5개)
    history_.push_back({p.x, p.y, p.z});
    if (history_.size() > 5)
      history_.pop_front();

    // 최소 5개 누적 후 평균 변화량 체크
    if (history_.size() == 5)
    {
      double std_x = stddev([](auto pt) { return pt[0]; });
      double std_y = stddev([](auto pt) { return pt[1]; });
      double std_z = stddev([](auto pt) { return pt[2]; });

      if (std_x < 0.01 && std_y < 0.01 && std_z < 0.01)  // 1cm 이하 변화면 안정화 판단
      {
        RCLCPP_INFO(this->get_logger(), "Pose stabilized. Publishing target pose...");

        pub_->publish(*msg);
        stable_ = true;
      }
    }
  }

  template<typename F>
  double stddev(F f)
  {
    double mean = 0.0;
    for (const auto &v : history_)
      mean += f(v);
    mean /= history_.size();

    double sum_sq = 0.0;
    for (const auto &v : history_)
      sum_sq += std::pow(f(v) - mean, 2);

    return std::sqrt(sum_sq / history_.size());
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
  std::deque<std::array<double, 3>> history_;
  bool stable_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArucoPoseStabilizer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
