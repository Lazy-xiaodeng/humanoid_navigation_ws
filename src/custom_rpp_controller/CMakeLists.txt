cmake_minimum_required(VERSION 3.8)
project(custom_rpp_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_regulated_pure_pursuit_controller REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)

# 编译控制器库
add_library(custom_rpp_controller SHARED
  src/regulated_pure_pursuit_with_min_angular.cpp
)

target_include_directories(custom_rpp_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(custom_rpp_controller
  nav2_core
  nav2_regulated_pure_pursuit_controller
  rclcpp
  geometry_msgs
  pluginlib
  tf2_ros
  nav2_costmap_2d
)

# 安装库文件
install(
  TARGETS custom_rpp_controller
  EXPORT export_custom_rpp_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装插件描述文件
install(
  FILES custom_rpp_controller.xml
  DESTINATION share/${PROJECT_NAME}
)

# 导出插件
pluginlib_export_plugin_description_file(nav2_core custom_rpp_controller.xml)

ament_export_include_directories(include)
ament_export_libraries(custom_rpp_controller)
ament_export_targets(export_custom_rpp_controller HAS_LIBRARY_TARGET)

ament_package()
