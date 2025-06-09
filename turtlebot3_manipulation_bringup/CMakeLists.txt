################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(turtlebot3_manipulation_bringup)

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

###############################################################################
add_executable(pick_node src/pick_node.cpp)
ament_target_dependencies(pick_node
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)

add_executable(aruco_pose_listener src/aruco_pose_listener.cpp)
ament_target_dependencies(aruco_pose_listener
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(moveit_test_node src/moveit_test_node.cpp)
ament_target_dependencies(moveit_test_node
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)

add_executable(aruco_pose_stabilizer src/aruco_pose_stabilizer.cpp)
ament_target_dependencies(aruco_pose_stabilizer
  rclcpp
  geometry_msgs
)
add_executable(pick_and_return_node src/pick_and_return_node.cpp)
ament_target_dependencies(pick_and_return_node rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  tf2_geometry_msgs
)

install(TARGETS
  pick_and_return_node
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Install
################################################################################
install(TARGETS
  aruco_pose_listener
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config rviz worlds
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS
  pick_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  moveit_test_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  aruco_pose_stabilizer
  DESTINATION lib/${PROJECT_NAME}
)
################################################################################
# Macro for ament package
################################################################################
ament_package()
