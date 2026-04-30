cmake_minimum_required(VERSION 3.8)
project(humanoid_terrain_analyzer)

# ============================================================
# 编译标准
# ============================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================
# 依赖查找
# ============================================================
find_package(ament_cmake        REQUIRED)
find_package(rclcpp             REQUIRED)
find_package(grid_map_msgs      REQUIRED)
find_package(sensor_msgs        REQUIRED)
find_package(std_msgs           REQUIRED)
find_package(humanoid_interfaces REQUIRED)
find_package(Eigen3             REQUIRED)
find_package(OpenCL             REQUIRED)
find_package(ament_index_cpp    REQUIRED)
find_package(geometry_msgs      REQUIRED)
find_package(tf2_geometry_msgs  REQUIRED)

# ============================================================
# 可执行目标
# ============================================================
add_executable(terrain_analyzer_node
  src/terrain_analyzer_node.cpp
  src/terrain_analyzer_core.cpp
)

# 头文件搜索路径
target_include_directories(terrain_analyzer_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
)

# ROS2 依赖
ament_target_dependencies(terrain_analyzer_node
  rclcpp
  grid_map_msgs
  sensor_msgs
  std_msgs
  humanoid_interfaces
  ament_index_cpp
  geometry_msgs
  tf2_geometry_msgs
)

# 非 ROS 库链接
target_link_libraries(terrain_analyzer_node
  OpenCL::OpenCL
)


# ============================================================
# 安装规则
# ============================================================
install(TARGETS terrain_analyzer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# 把 kernels / config / launch 安装到 share 目录
# ament_index_cpp 会从这里找 kernels/terrain_analyzer.cl
install(DIRECTORY
  kernels
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()