cmake_minimum_required(VERSION 3.8)
project(ik_processing)

set(CMAKE_CXX_STANDARD 20)

include(FetchContent)
FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz)
FetchContent_MakeAvailable(json)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_visual_tools REQUIRED)

find_package(rosidl_default_generators REQUIRED)

add_executable(solver src/solver.cpp)
add_executable(fullsim src/fullsim.cpp)
add_executable(parser src/parser.cpp)

add_library(ik_helpers
  src/helpers.cpp
)

target_include_directories(ik_helpers
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(solver
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2_ros
  moveit_ros_planning_interface
  moveit_core
  moveit_visual_tools
)

ament_target_dependencies(fullsim
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2_ros
  moveit_ros_planning_interface
  moveit_core
  moveit_visual_tools
)

ament_target_dependencies(parser
  rclcpp
  ament_index_cpp
  # geometry_msgs
  # sensor_msgs
  # tf2_ros
  # moveit_ros_planning_interface
  # moveit_core
  # moveit_visual_tools
)

ament_target_dependencies(ik_helpers
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2_ros
  moveit_ros_planning_interface
  moveit_core
  moveit_visual_tools
)

target_link_libraries(fullsim
  ik_helpers
  nlohmann_json::nlohmann_json
)

target_link_libraries(ik_helpers
  nlohmann_json::nlohmann_json
)

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

target_link_libraries(ik_helpers "${cpp_typesupport_target}")

target_link_libraries(parser "${cpp_typesupport_target}")
target_link_libraries(fullsim "${cpp_typesupport_target}")


install(DIRECTORY
  data
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS solver
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS fullsim
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS parser
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS ik_helpers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
