cmake_minimum_required(VERSION 3.14)
project(z1_task)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(control_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

# ── Non-MTC version (MoveGroupInterface + Cartesian + TOTG) ─────────────
add_executable(pick_cube_task src/pick_cube_task.cpp)
ament_target_dependencies(pick_cube_task
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_core
  control_msgs
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  geometry_msgs
)

# ── MTC version (kept for reference / fallback) ─────────────────────────
add_executable(pick_cube_task_MTC src/pick_cube_task_MTC.cpp)
ament_target_dependencies(pick_cube_task_MTC
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

# ── MTC simple version (flat pipeline, direct controller execution) ──────
add_executable(pick_cube_task_MTC_simple src/pick_cube_task_MTC_simple.cpp)
ament_target_dependencies(pick_cube_task_MTC_simple
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

install(TARGETS pick_cube_task pick_cube_task_MTC pick_cube_task_MTC_simple
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/task_sequencer.py
  scripts/pick_cube_pinocchio.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
