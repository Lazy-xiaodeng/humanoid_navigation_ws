cmake_minimum_required(VERSION 3.8)
project(straight_first_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_theta_star_planner REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(straight_first_planner SHARED
  src/straight_first_planner.cpp
)

target_include_directories(straight_first_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(straight_first_planner
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_theta_star_planner
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS straight_first_planner
  EXPORT export_straight_first_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  FILES straight_first_planner.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(nav2_core straight_first_planner.xml)

ament_export_include_directories(include)
ament_export_libraries(straight_first_planner)
ament_export_targets(export_straight_first_planner HAS_LIBRARY_TARGET)

ament_package()
