cmake_minimum_required(VERSION 3.8)
project(fast_lio_robosense)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# --- 编译选项优化 ---
add_compile_options(-std=c++17)
add_definitions(-DROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

message("Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")

# --- CPU 核心数自动检测与并行处理优化 (针对16核CPU优化) ---
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processor number: ${N}")
  
  # 针对多核CPU优化：充分利用CPU核心，但预留部分核心给系统和其他节点
  if(N GREATER 12)
    # 16核CPU使用10个线程，预留6个核心给系统和其他节点
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=6)
    message("Fast-LIO: Using 6 threads for high-core-count CPU (${N} cores detected)")
  elseif(N GREATER 8)
    # 12核CPU使用8个线程
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=4)
    message("Fast-LIO: Using 4 threads for medium-core-count CPU (${N} cores detected)")
  elseif(N GREATER 4)
    # 8核CPU使用6个线程
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=4)
    message("Fast-LIO: Using 4 threads for low-core-count CPU (${N} cores detected)")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

# --- 查找依赖 ---
find_package(OpenMP QUIET)
find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

# ROS 基础依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 第三方库
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

set(dependencies
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  visualization_msgs
  tf2_ros
  tf2_eigen
  pcl_conversions
)

# --- 生成自定义消息 (Pose6D) ---
set(msg_files
  "msg/Pose6D.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)
ament_export_dependencies(rosidl_default_runtime)

# --- 生成可执行文件 ---
add_executable(fastlio_mapping 
  src/laserMapping.cpp 
  include/ikd-Tree/ikd_Tree.cpp 
  src/preprocess.cpp
)

target_include_directories(fastlio_mapping PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# 链接库
target_link_libraries(fastlio_mapping 
  ${PCL_LIBRARIES} 
  ${PYTHON_LIBRARIES} 
  Eigen3::Eigen
  OpenMP::OpenMP_CXX
)

# 映射自定义接口依赖
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(fastlio_mapping ${cpp_typesupport_target})

ament_target_dependencies(fastlio_mapping ${dependencies})

# --- 安装规则 (针对你的精简需求修复) ---
install(TARGETS fastlio_mapping
  DESTINATION lib/${PROJECT_NAME}
)

# 只安装你保留的文件夹
install(
  DIRECTORY 
  config    # 现在里面只剩 robosenseAiry.yaml
  launch    # 现在里面只剩专属 launch
  rviz      # 可视化配置
  DESTINATION share/${PROJECT_NAME}
)

ament_package()