cmake_minimum_required(VERSION 3.8)
project(humanoid_nav2_bt_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
if("$ENV{ROS_DISTRO}" STREQUAL "humble")
  find_package(behaviortree_cpp_v3 REQUIRED)
  set(BTCPP_DEP behaviortree_cpp_v3)
  set(BTCPP_IS_V3 TRUE)
else()
  find_package(behaviortree_cpp QUIET)
  if(behaviortree_cpp_FOUND)
    set(BTCPP_DEP behaviortree_cpp)
  else()
    find_package(behaviortree_cpp_v3 REQUIRED)
    set(BTCPP_DEP behaviortree_cpp_v3)
    set(BTCPP_IS_V3 TRUE)
  endif()
endif()
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/pose_angle_nodes.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_definitions(${PROJECT_NAME} PRIVATE BT_PLUGIN_EXPORT)
if(BTCPP_IS_V3)
  target_compile_definitions(${PROJECT_NAME} PRIVATE HUMANOID_BT_FORCE_V3)
endif()

ament_target_dependencies(${PROJECT_NAME}
  angles
  ${BTCPP_DEP}
  geometry_msgs
  nav2_behavior_tree
  nav2_msgs
  rclcpp
  rclcpp_action
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  angles
  ${BTCPP_DEP}
  geometry_msgs
  nav2_behavior_tree
  nav2_msgs
  rclcpp
  rclcpp_action
  tf2
  tf2_geometry_msgs
  tf2_ros
)

ament_package()
