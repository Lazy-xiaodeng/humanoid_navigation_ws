cmake_minimum_required(VERSION 3.8)
project(humanoid_control_runtime)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(dynamic_waypoints_manager_cpp
  src/dynamic_waypoints_manager.cpp
)
ament_target_dependencies(dynamic_waypoints_manager_cpp
  rclcpp
  std_msgs
)

add_executable(map_context_manager_cpp
  src/map_context_manager.cpp
)
ament_target_dependencies(map_context_manager_cpp
  rclcpp
  std_msgs
  geometry_msgs
)

install(TARGETS
  dynamic_waypoints_manager_cpp
  map_context_manager_cpp
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
