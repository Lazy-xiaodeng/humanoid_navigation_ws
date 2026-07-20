cmake_minimum_required(VERSION 3.8)
project(humanoid_route_runtime)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

option(BUILD_RUNTIME_PROBES "Build developer probe executables for humanoid_route_runtime" OFF)

add_executable(navigation_state_manager_cpp
  src/navigation_state_manager.cpp
  src/nav2_action_controller.cpp
  src/navigation_gatekeeper.cpp
  src/obstacle_wait_manager.cpp
  src/route_task_protocol.cpp
  src/route_task_state_machine.cpp
)
target_include_directories(navigation_state_manager_cpp PRIVATE
  include
)
ament_target_dependencies(navigation_state_manager_cpp
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  nav2_msgs
  lifecycle_msgs
  action_msgs
  tf2
  tf2_ros
)

if(BUILD_RUNTIME_PROBES)
  add_executable(
    obstacle_confirmation_probe
    test/probes/obstacle_confirmation_probe.cpp
    src/obstacle_wait_manager.cpp
  )
  target_include_directories(obstacle_confirmation_probe PRIVATE include)
  install(TARGETS obstacle_confirmation_probe DESTINATION lib/${PROJECT_NAME})
endif()

install(TARGETS
  navigation_state_manager_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
