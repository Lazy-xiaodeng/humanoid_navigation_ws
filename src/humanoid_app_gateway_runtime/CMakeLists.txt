#[[
  文件用途：
  1. 构建 humanoid_app_gateway_runtime 的 C++ 库和两个节点可执行文件。
  2. 上游：colcon/ament_cmake 构建系统。
  3. 下游：安装后的 app_gateway_node、data_integration_node、config、launch 和 README。
]]
cmake_minimum_required(VERSION 3.8)
project(humanoid_app_gateway_runtime)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

option(BUILD_RUNTIME_PROBES "Build developer probe executables for humanoid_app_gateway_runtime" OFF)

add_library(app_gateway_runtime_core
  src/app_gateway_config.cpp
  src/app_protocol.cpp
  src/business_command_router.cpp
  src/client_registry.cpp
  src/data_store.cpp
  src/integration_forwarder.cpp
  src/map_waypoint_adapter.cpp
  src/navigation_status_adapter.cpp
  src/path_metrics.cpp
  src/pose_speed_adapter.cpp
  src/protocol_builder.cpp
  src/robot_status_adapter.cpp
  src/subscription_manager.cpp
)
target_include_directories(app_gateway_runtime_core PUBLIC include)
target_link_libraries(app_gateway_runtime_core Boost::system)
ament_target_dependencies(app_gateway_runtime_core
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  ament_index_cpp
)

add_executable(app_gateway_node src/app_gateway_node.cpp)
target_link_libraries(app_gateway_node app_gateway_runtime_core Boost::system)
ament_target_dependencies(app_gateway_node rclcpp std_msgs geometry_msgs)

add_executable(data_integration_node src/data_integration_node.cpp)
target_link_libraries(data_integration_node app_gateway_runtime_core)
ament_target_dependencies(data_integration_node rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs ament_index_cpp)

if(BUILD_RUNTIME_PROBES)
  add_executable(robot_status_adapter_probe test/probes/robot_status_adapter_probe.cpp)
  target_link_libraries(robot_status_adapter_probe app_gateway_runtime_core)
  ament_target_dependencies(robot_status_adapter_probe rclcpp)

  add_executable(protocol_builder_probe test/probes/protocol_builder_probe.cpp)
  target_link_libraries(protocol_builder_probe app_gateway_runtime_core)
  ament_target_dependencies(protocol_builder_probe rclcpp)

  add_executable(data_store_probe test/probes/data_store_probe.cpp)
  target_link_libraries(data_store_probe app_gateway_runtime_core)
  ament_target_dependencies(data_store_probe rclcpp)

  add_executable(subscription_manager_probe test/probes/subscription_manager_probe.cpp)
  target_link_libraries(subscription_manager_probe app_gateway_runtime_core)
  ament_target_dependencies(subscription_manager_probe rclcpp)

  add_executable(app_protocol_probe test/probes/app_protocol_probe.cpp)
  target_link_libraries(app_protocol_probe app_gateway_runtime_core)
  ament_target_dependencies(app_protocol_probe rclcpp)

  add_executable(path_metrics_probe test/probes/path_metrics_probe.cpp)
  target_link_libraries(path_metrics_probe app_gateway_runtime_core)
  ament_target_dependencies(path_metrics_probe rclcpp)

  add_executable(pose_speed_adapter_probe test/probes/pose_speed_adapter_probe.cpp)
  target_link_libraries(pose_speed_adapter_probe app_gateway_runtime_core)
  ament_target_dependencies(pose_speed_adapter_probe rclcpp)

  add_executable(navigation_status_adapter_probe test/probes/navigation_status_adapter_probe.cpp)
  target_link_libraries(navigation_status_adapter_probe app_gateway_runtime_core)
  ament_target_dependencies(navigation_status_adapter_probe rclcpp)

  add_executable(map_waypoint_adapter_probe test/probes/map_waypoint_adapter_probe.cpp)
  target_link_libraries(map_waypoint_adapter_probe app_gateway_runtime_core)
  ament_target_dependencies(map_waypoint_adapter_probe rclcpp)

  add_executable(client_registry_probe test/probes/client_registry_probe.cpp)
  target_link_libraries(client_registry_probe app_gateway_runtime_core)
  ament_target_dependencies(client_registry_probe rclcpp)

  add_executable(integration_forwarder_probe test/probes/integration_forwarder_probe.cpp)
  target_link_libraries(integration_forwarder_probe app_gateway_runtime_core)
  ament_target_dependencies(integration_forwarder_probe rclcpp)

  add_executable(business_command_router_probe test/probes/business_command_router_probe.cpp)
  target_link_libraries(business_command_router_probe app_gateway_runtime_core)
  ament_target_dependencies(business_command_router_probe rclcpp)

  install(TARGETS
    robot_status_adapter_probe
    protocol_builder_probe
    data_store_probe
    subscription_manager_probe
    app_protocol_probe
    path_metrics_probe
    pose_speed_adapter_probe
    navigation_status_adapter_probe
    map_waypoint_adapter_probe
    client_registry_probe
    integration_forwarder_probe
    business_command_router_probe
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(TARGETS
  app_gateway_runtime_core
  app_gateway_node
  data_integration_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
