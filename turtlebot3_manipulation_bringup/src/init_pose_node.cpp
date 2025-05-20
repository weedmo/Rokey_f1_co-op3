#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <memory>
#include <vector>

void move_robot(const std::shared_ptr<rclcpp::Node> node)
{
    // ✅ 1. arm MoveGroupInterface 생성 (planning group 이름은 "arm" 또는 사용 중인 그룹 이름으로 교체)
    auto arm_move_group = moveit::planning_interface::MoveGroupInterface(node, "arm");

    // ✅ 2. 목표 joint 값 설정 (단위: 라디안)
    // 예: joint1 = 0°, joint2 = -34°, joint3 = -12°, joint4 = 117°
    std::vector<double> arm_joint_goal = {
        0.0,        // joint1
        -0.1571,    //-0.2893,     // joint2
        -0.7330,     //-0.9425,     // joint3
        2.0420      // joint4
    };

    bool within_bounds = arm_move_group.setJointValueTarget(arm_joint_goal);

    if (!within_bounds)
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                    "❗ 목표 joint 값이 제한 범위를 초과하였습니다. 클램핑 처리 예정.");
        return;
    }

    // ✅ 3. 경로 계획 및 실행
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool plan_success = (arm_move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (plan_success)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "✅ 경로 생성 성공! 로봇 암 이동 시작.");
        arm_move_group.move();
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "❌ 경로 계획 실패!");
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("init_pose_node");
    move_robot(node);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
