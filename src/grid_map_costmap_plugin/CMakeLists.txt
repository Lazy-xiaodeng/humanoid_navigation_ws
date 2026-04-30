cmake_minimum_required(VERSION 3.5)
project(grid_map_costmap_plugin)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  nav2_costmap_2d
  pluginlib
  grid_map_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# 编译插件库
add_library(grid_map_costmap_layer SHARED
  plugins/grid_map_layer.cpp
)

target_include_directories(grid_map_costmap_layer
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_target_dependencies(grid_map_costmap_layer ${dependencies})

# 安装
install(TARGETS grid_map_costmap_layer
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(FILES grid_map_layer_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# 注册插件
pluginlib_export_plugin_description_file(nav2_costmap_2d grid_map_layer_plugin.xml)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})
ament_package()