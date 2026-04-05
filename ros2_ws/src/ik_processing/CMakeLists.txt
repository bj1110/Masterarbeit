cmake_minimum_required(VERSION 3.8)
project(ik_processing)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------
# Dependencies
# -----------------------------
find_package(ament_cmake REQUIRED)

set(COMMON_DEPS
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2_ros
  moveit_ros_planning_interface
  moveit_core
  nullspace_solver
  yaml-cpp
)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(RBDL REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nullspace_solver REQUIRED)
find_package(yaml-cpp REQUIRED)

# -----------------------------
# External: nlohmann/json
# -----------------------------
include(FetchContent)
FetchContent_Declare(
  json
  URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz
)
FetchContent_MakeAvailable(json)

# -----------------------------
# Libraries
# -----------------------------
add_library(ik_helpers src/helpers.cpp)
add_library(ik_evaluator src/evaluator.cpp)
add_library(rba src/rigid_body_algorithm.cpp)

set(PROJECT_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

foreach(lib ik_helpers ik_evaluator rba)
  target_include_directories(${lib}
    PUBLIC
      $<BUILD_INTERFACE:${PROJECT_INCLUDE_DIR}>
      $<INSTALL_INTERFACE:include># استاندارد
  )
endforeach()

# Dependencies
ament_target_dependencies(ik_helpers ${COMMON_DEPS})
ament_target_dependencies(ik_evaluator ${COMMON_DEPS})
ament_target_dependencies(rba rclcpp Eigen3)

# Link libraries
target_link_libraries(ik_helpers nlohmann_json::nlohmann_json)
target_link_libraries(ik_evaluator nlohmann_json::nlohmann_json)
target_link_libraries(rba rbdl rbdl_urdfreader)

# -----------------------------
# Executables
# -----------------------------
add_executable(solver src/solver.cpp)
add_executable(fullsim src/fullsim.cpp)
add_executable(parser src/parser.cpp)

ament_target_dependencies(solver ${COMMON_DEPS})
ament_target_dependencies(fullsim ${COMMON_DEPS})
ament_target_dependencies(parser rclcpp ament_index_cpp)

# Linking
target_link_libraries(fullsim
  ik_helpers
  ik_evaluator
  rba
  nlohmann_json::nlohmann_json
  yaml-cpp
)

# -----------------------------
# ROS Interfaces
# -----------------------------
set(msg_files
  "msg/Datapoint.msg"
  "msg/Header.msg"
)

set(srv_files
  "srv/Data.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
)

ament_export_dependencies(rosidl_default_runtime)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# Link typesupport
foreach(target
    ik_helpers ik_evaluator rba
    parser fullsim solver)
  target_link_libraries(${target} "${cpp_typesupport_target}")
endforeach()

# -----------------------------
# Install
# -----------------------------
install(TARGETS
  solver fullsim parser
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ik_helpers ik_evaluator rba
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch data config
  DESTINATION share/${PROJECT_NAME}
)

# -----------------------------
# Final
# -----------------------------
ament_package()