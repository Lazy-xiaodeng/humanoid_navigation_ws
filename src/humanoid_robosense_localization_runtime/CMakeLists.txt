cmake_minimum_required(VERSION 3.10)
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()
project(humanoid_robosense_localization_runtime)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(ROS_VERSION 2)
add_definitions(-DROS2)
add_definitions(-DUSE_EIGEN_OPTIMIZATION)
add_definitions(-DPROJECT_DIR="${PROJECT_SOURCE_DIR}")
add_definitions(-DPACKAGE_PATH="${CMAKE_CURRENT_LIST_DIR}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_subdirectory(lidar_matcher)
add_subdirectory(lidar_localization)
add_subdirectory(node)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
