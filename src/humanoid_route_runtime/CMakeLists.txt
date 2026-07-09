cmake_minimum_required(VERSION 3.16)
project(humanoid_route_runtime)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(navigation_state_manager_cpp
  src/navigation_state_manager.cpp
)

target_compile_features(navigation_state_manager_cpp PRIVATE cxx_std_17)

ament_target_dependencies(navigation_state_manager_cpp
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  nav_msgs
  nav2_msgs
  action_msgs
  tf2
  tf2_ros
)

install(TARGETS navigation_state_manager_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
