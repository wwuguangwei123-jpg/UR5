cmake_minimum_required(VERSION 3.8)
project(ur5_motion_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

set(COMMON_DEPS
  rclcpp
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  shape_msgs
  std_msgs
  tf2_geometry_msgs
  trajectory_msgs
  visualization_msgs
)

foreach(target
  move_group_demo_node
  planning_scene_node
  multi_target_planner_node
  cartesian_path_node
)
  add_executable(${target} src/${target}.cpp)
  target_compile_features(${target} PUBLIC cxx_std_17)
  ament_target_dependencies(${target} ${COMMON_DEPS})
endforeach()

install(TARGETS
  move_group_demo_node
  planning_scene_node
  multi_target_planner_node
  cartesian_path_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/fake_trajectory_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
