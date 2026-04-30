cmake_minimum_required(VERSION 3.8)
project(humanoid_point_cloud_filter)

# ============================================================
# 编译标准
# ============================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================
# 设置 Intel SYCL 编译器
# ============================================================
# 添加 Intel oneAPI CMake 模块路径
list(APPEND CMAKE_PREFIX_PATH "/opt/intel/oneapi/compiler/2025.3/lib/cmake")
list(APPEND CMAKE_MODULE_PATH "/opt/intel/oneapi/compiler/2025.3/lib/cmake/IntelSYCL")

# 设置 SYCL 编译器
set(CMAKE_CXX_COMPILER "icpx")
set(CMAKE_C_COMPILER "icx")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================
# 依赖查找
# ============================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters kdtree)
find_package(ament_index_cpp REQUIRED)

# SYCL / OpenMP 二选一
find_package(IntelSYCL QUIET)

if(IntelSYCL_FOUND)
  message(STATUS "✅ 找到 Intel SYCL，启用 GPU 加速")
  set(USE_SYCL ON)
else()
  message(WARNING "⚠️ 未找到 Intel SYCL，使用 CPU 多线程版本")
  set(USE_SYCL OFF)
  find_package(OpenMP REQUIRED)
endif()

# ============================================================
# 头文件路径
# ============================================================
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# ============================================================
# 源文件
# ============================================================
set(FILTER_SOURCES
  src/point_cloud_filter_node.cpp
  src/point_cloud_filter_core.cpp
)

if(USE_SYCL)
  list(APPEND FILTER_SOURCES src/sycl_kernels.cpp)
endif()

# ============================================================
# 可执行目标
# ============================================================
add_executable(point_cloud_filter_node ${FILTER_SOURCES})

target_include_directories(point_cloud_filter_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(point_cloud_filter_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_eigen
  pcl_ros
  pcl_conversions
  ament_index_cpp
)

target_link_libraries(point_cloud_filter_node
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

if(USE_SYCL)
  target_compile_definitions(point_cloud_filter_node PRIVATE USE_SYCL)

  target_compile_options(point_cloud_filter_node PRIVATE
    -fsycl
    -fsycl-targets=spir64
  )
  target_link_options(point_cloud_filter_node PRIVATE
    -fsycl
    -fsycl-targets=spir64
  )
else()
  target_link_libraries(point_cloud_filter_node OpenMP::OpenMP_CXX)
endif()

# ============================================================
# 安装
# ============================================================
install(TARGETS
  point_cloud_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  config
  launch
  kernels
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_eigen
  pcl_ros
  pcl_conversions
)

ament_package()