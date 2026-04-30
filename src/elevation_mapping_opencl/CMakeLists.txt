cmake_minimum_required(VERSION 3.14)
project(elevation_mapping_opencl)

# ===== C++ 标准 =====
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# ===== 查找 ROS2 依赖 =====
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ===== 查找 PCL =====
find_package(PCL REQUIRED COMPONENTS common io)

# ===== 查找 Eigen3 =====
find_package(Eigen3 REQUIRED)

# ===== 查找 OpenCL =====
find_package(OpenCL REQUIRED)
if(NOT OpenCL_FOUND)
  message(FATAL_ERROR "❌ 未找到 OpenCL！请安装 Intel OpenCL 驱动。")
endif()
message(STATUS "✅ 找到 OpenCL: ${OpenCL_VERSION_STRING}")
message(STATUS "   头文件: ${OpenCL_INCLUDE_DIRS}")
message(STATUS "   库文件: ${OpenCL_LIBRARIES}")

# ===== 构建可执行文件 =====
add_executable(elevation_mapping_opencl_node
  src/elevation_mapping_node.cpp
  src/elevation_map_opencl.cpp
)

target_include_directories(elevation_mapping_opencl_node PRIVATE
  ${cv_bridge_INCLUDE_DIRS}
  /opt/ros/jazzy/include
  include
  ${OpenCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(elevation_mapping_opencl_node
  ${OpenCL_LIBRARIES}
  ${PCL_LIBRARIES}
)

ament_target_dependencies(elevation_mapping_opencl_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  grid_map_core
  grid_map_ros
  grid_map_msgs
  tf2_ros
  tf2_eigen
  pcl_conversions
  ament_index_cpp
)

# ===== 安装 =====
install(TARGETS elevation_mapping_opencl_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# 安装 kernel 文件（必须！节点运行时需要读取）
install(DIRECTORY src/kernels/
  DESTINATION share/${PROJECT_NAME}/kernels
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()