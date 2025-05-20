################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(turtlebot3_manipulation_bringup)

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(aruco_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
###############################################################################
add_executable(aruco_follower_node src/aruco_follower_node.cpp)
ament_target_dependencies(aruco_follower_node
  rclcpp
  geometry_msgs
  aruco_msgs
  moveit_ros_planning_interface
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# init_pose_node
add_executable(init_pose_node src/init_pose_node.cpp)
ament_target_dependencies(init_pose_node
  rclcpp
  moveit_ros_planning_interface
)
add_executable(task_executor_node src/task_executor_node.cpp)
ament_target_dependencies(task_executor_node
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  control_msgs
  trajectory_msgs
)
add_executable(gripper_test_node src/gripper_test_node.cpp)

# 실행 파일에 필요한 라이브러리 연결
ament_target_dependencies(gripper_test_node
  rclcpp
  moveit_ros_planning_interface
)

# 설치 경로 지정
install(TARGETS
  gripper_test_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS task_executor_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS
  aruco_follower_node
  init_pose_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
