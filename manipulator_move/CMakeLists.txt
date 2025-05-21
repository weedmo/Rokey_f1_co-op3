cmake_minimum_required(VERSION 3.8)
project(manipulator_move)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 필요한 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
# 실행 파일 추가
add_executable(aruco_moveit_executor_node src/aruco_moveit_executor.cpp)

# 링크 대상
ament_target_dependencies(aruco_moveit_executor_node
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  tf2_ros
)

# 설치 경로 설정
install(TARGETS
  aruco_moveit_executor_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
