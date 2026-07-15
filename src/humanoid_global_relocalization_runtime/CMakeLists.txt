cmake_minimum_required(VERSION 3.16)
project(humanoid_global_relocalization_runtime)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration)
find_package(yaml-cpp REQUIRED)
find_package(OpenMP)

set(BUILD_CUDA OFF CACHE BOOL "Disable 3D-BBS CUDA backend for the CPU runtime build" FORCE)
add_subdirectory(third_party/3d_bbs)

set(COMMON_DEPS
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  pcl_conversions
  pcl_ros
  rosbag2_cpp
  rosbag2_storage
)

add_library(global_relocalization_core
  src/bbs2d_search.cpp
  src/config_loader.cpp
  src/point_cloud_adapter.cpp
  src/refiner.cpp
  src/resource_monitor.cpp
  src/scan_context.cpp
  src/solid.cpp
  src/simple_bbs3d.cpp
  src/temporal_consistency.cpp
  src/evaluator.cpp
)

target_include_directories(global_relocalization_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/3d_bbs/bbs3d/include
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(global_relocalization_core
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  yaml-cpp
  cpu_bbs3d
)

if(OpenMP_CXX_FOUND)
  target_link_libraries(global_relocalization_core OpenMP::OpenMP_CXX)
endif()

ament_target_dependencies(global_relocalization_core ${COMMON_DEPS})

add_executable(global_relocalization_offline_eval
  src/offline_eval_main.cpp
)
target_link_libraries(global_relocalization_offline_eval global_relocalization_core)
ament_target_dependencies(global_relocalization_offline_eval ${COMMON_DEPS})

add_executable(global_relocalization_node
  src/global_relocalization_node.cpp
)
target_link_libraries(global_relocalization_node global_relocalization_core)
ament_target_dependencies(global_relocalization_node ${COMMON_DEPS})

add_executable(solid_database_builder
  src/solid_database_builder_main.cpp
)
target_link_libraries(solid_database_builder global_relocalization_core)
ament_target_dependencies(solid_database_builder ${COMMON_DEPS})

install(TARGETS
  global_relocalization_core
  global_relocalization_offline_eval
  global_relocalization_node
  solid_database_builder
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES
  config/relocalization_runtime.yaml
  config/relocalization_validation.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
