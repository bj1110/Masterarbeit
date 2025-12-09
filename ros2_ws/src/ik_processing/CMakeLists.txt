cmake_minimum_required(VERSION 3.8)
project(ik_processing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_visual_tools REQUIRED)

add_executable(solver src/solver.cpp)

ament_target_dependencies(solver
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2_ros
  moveit_ros_planning_interface
  moveit_core
  moveit_visual_tools
)

install(TARGETS solver
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
