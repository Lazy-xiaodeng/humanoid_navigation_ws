cmake_minimum_required(VERSION 3.8)
project(humanoid_open3d_adapter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(fastlio_open3d_axis_adapter
  src/fastlio_open3d_axis_adapter.cpp
)
target_compile_features(fastlio_open3d_axis_adapter PUBLIC cxx_std_17)
target_link_libraries(fastlio_open3d_axis_adapter Eigen3::Eigen)
ament_target_dependencies(fastlio_open3d_axis_adapter
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
)

install(TARGETS fastlio_open3d_axis_adapter
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
