################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(turtlebot3_manipulation_bringup)

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_common REQUIRED)
find_package(moveit_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)


add_executable(mission_pose src/mission_pose.cpp)
ament_target_dependencies(mission_pose
  rclcpp
  moveit_ros_planning_interface
  moveit_common
  moveit_core
  geometry_msgs
  tf2_geometry_msgs
  tf2_ros
)

add_executable(init_pose src/init_pose.cpp)
ament_target_dependencies(init_pose
  rclcpp
  moveit_ros_planning_interface
  moveit_common
  moveit_core
  geometry_msgs
  tf2_geometry_msgs
  tf2_ros
)

################################################################################
# Install
################################################################################
# install
install(TARGETS
  mission_pose
  init_pose
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch config rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_package()
