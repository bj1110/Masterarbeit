cmake_minimum_required(VERSION 3.8)
project(nullspace_solver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_library(solver SHARED
  src/solver.cpp
)

ament_target_dependencies(solver
  rclcpp
  Eigen3
  pinocchio
  moveit_ros_planning_interface
  moveit_core
)

install(
  TARGETS solver
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/
)

ament_package()