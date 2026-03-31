cmake_minimum_required(VERSION 3.14)
project(z1_task)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(control_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

add_executable(pick_cube_task src/pick_cube_task.cpp)
ament_target_dependencies(pick_cube_task
  rclcpp
  rclcpp_action
  moveit_task_constructor_core
  control_msgs
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  geometry_msgs
  moveit_msgs
)

install(TARGETS pick_cube_task
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
