cmake_minimum_required(VERSION 3.8)
project(humanoid_navigation2_cpp_nodes)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(COMMON_DEPS
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

add_executable(dynamic_odom_ground_publisher src/dynamic_odom_ground_publisher.cpp)
ament_target_dependencies(dynamic_odom_ground_publisher ${COMMON_DEPS})

add_executable(robot_realpose_publisher src/robot_realpose_publisher.cpp)
ament_target_dependencies(robot_realpose_publisher ${COMMON_DEPS})

add_executable(rviz_initialpose_adapter src/rviz_initialpose_adapter.cpp)
ament_target_dependencies(rviz_initialpose_adapter ${COMMON_DEPS})

add_executable(periodic_clearing_3d_publisher src/periodic_clearing_3d_publisher.cpp)
ament_target_dependencies(periodic_clearing_3d_publisher ${COMMON_DEPS})

add_executable(prior_map_odom_bridge_cpp src/prior_map_odom_bridge_cpp.cpp)
target_link_libraries(prior_map_odom_bridge_cpp Eigen3::Eigen)
ament_target_dependencies(prior_map_odom_bridge_cpp ${COMMON_DEPS})

add_executable(roi_obstacle_detector_cpp src/roi_obstacle_detector_cpp.cpp)
target_link_libraries(roi_obstacle_detector_cpp Eigen3::Eigen)
ament_target_dependencies(roi_obstacle_detector_cpp ${COMMON_DEPS})

install(TARGETS
  dynamic_odom_ground_publisher
  robot_realpose_publisher
  rviz_initialpose_adapter
  periodic_clearing_3d_publisher
  prior_map_odom_bridge_cpp
  roi_obstacle_detector_cpp
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
