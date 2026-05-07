cmake_minimum_required(VERSION 3.10)
project(humanoid_global_localization)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable OpenMP for grid search parallelism
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# Find ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

# Find PCL
find_package(PCL 1.10 REQUIRED COMPONENTS common io filters registration)
include_directories(${PCL_INCLUDE_DIRS})

# Build the node
add_executable(global_localization_node
  src/global_localization_node.cpp
)

target_include_directories(global_localization_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(global_localization_node
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  pcl_conversions
)

target_link_libraries(global_localization_node
  ${PCL_LIBRARIES}
)
if(OpenMP_CXX_FOUND)
  target_link_libraries(global_localization_node OpenMP::OpenMP_CXX)
endif()

# Install
install(TARGETS global_localization_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
