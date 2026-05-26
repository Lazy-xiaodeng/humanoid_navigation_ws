cmake_minimum_required(VERSION 3.8)
project(humanoid_scancontext_global_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io registration filters)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(scancontext_global_core SHARED
  src/scancontext_global_core.cpp
)
target_include_directories(scancontext_global_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(scancontext_global_core
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

add_executable(scancontext_global_localizer_node
  src/scancontext_global_localizer_node.cpp
)
target_link_libraries(scancontext_global_localizer_node
  scancontext_global_core
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)
ament_target_dependencies(scancontext_global_localizer_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  pcl_conversions
)

install(TARGETS
  scancontext_global_core
  scancontext_global_localizer_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/build_scancontext_database.py
  scripts/offline_bag_nav_validation.py
  scripts/record_scancontext_validation.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
