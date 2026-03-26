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

find_package(ik_processing REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_library(nullspace_solver_lib SHARED
  src/solver.cpp
  src/input_trajectory.cpp
)

target_include_directories(nullspace_solver_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(nullspace_solver_lib
  rclcpp
  Eigen3
  pinocchio
  moveit_ros_planning_interface
  moveit_core
  ik_processing
)


install(
  TARGETS nullspace_solver_lib
  EXPORT export_nullspace_solver
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/
)


ament_export_targets(export_nullspace_solver HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  Eigen3
  pinocchio
  moveit_ros_planning_interface
  moveit_core
  ik_processing
)

ament_package()