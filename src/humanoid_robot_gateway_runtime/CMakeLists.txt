#[[
  文件用途：
  1. 构建 humanoid_robot_gateway_runtime 的 C++ 库和 robot_gateway_node 可执行文件。
  2. 上游：colcon/ament_cmake 构建系统。
  3. 下游：安装后的 robot_gateway_node、config、launch 和 README。
]]
cmake_minimum_required(VERSION 3.8)
project(humanoid_robot_gateway_runtime)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

option(BUILD_RUNTIME_PROBES "Build developer probe executables for humanoid_robot_gateway_runtime" OFF)

add_library(robot_gateway_runtime_core
  src/action_result_builder.cpp
  src/gesture_sync.cpp
  src/motion_controller.cpp
  src/response_waiter.cpp
  src/robot_gateway_config.cpp
  src/robot_protocol.cpp
  src/robot_status_parser.cpp
  src/robot_ws_client.cpp
  src/walk_velocity_controller.cpp
)
target_include_directories(robot_gateway_runtime_core PUBLIC include)
target_link_libraries(robot_gateway_runtime_core yaml-cpp Boost::system)
ament_target_dependencies(robot_gateway_runtime_core
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  ament_index_cpp
)

add_executable(robot_gateway_node src/robot_gateway_node.cpp)
target_link_libraries(robot_gateway_node robot_gateway_runtime_core)
ament_target_dependencies(robot_gateway_node rclcpp std_msgs std_srvs geometry_msgs sensor_msgs ament_index_cpp)

if(BUILD_RUNTIME_PROBES)
  add_executable(robot_protocol_probe test/probes/robot_protocol_probe.cpp)
  target_link_libraries(robot_protocol_probe robot_gateway_runtime_core)
  ament_target_dependencies(robot_protocol_probe rclcpp)

  add_executable(response_waiter_probe test/probes/response_waiter_probe.cpp)
  target_link_libraries(response_waiter_probe robot_gateway_runtime_core)
  ament_target_dependencies(response_waiter_probe rclcpp)

  add_executable(robot_status_parser_probe test/probes/robot_status_parser_probe.cpp)
  target_link_libraries(robot_status_parser_probe robot_gateway_runtime_core)
  ament_target_dependencies(robot_status_parser_probe rclcpp)

  add_executable(action_result_builder_probe test/probes/action_result_builder_probe.cpp)
  target_link_libraries(action_result_builder_probe robot_gateway_runtime_core)
  ament_target_dependencies(action_result_builder_probe rclcpp)

  add_executable(walk_velocity_controller_probe test/probes/walk_velocity_controller_probe.cpp)
  target_link_libraries(walk_velocity_controller_probe robot_gateway_runtime_core)
  ament_target_dependencies(walk_velocity_controller_probe rclcpp)

  add_executable(gesture_sync_probe test/probes/gesture_sync_probe.cpp)
  target_link_libraries(gesture_sync_probe robot_gateway_runtime_core)
  ament_target_dependencies(gesture_sync_probe rclcpp)

  add_executable(motion_controller_probe test/probes/motion_controller_probe.cpp)
  target_link_libraries(motion_controller_probe robot_gateway_runtime_core)
  ament_target_dependencies(motion_controller_probe rclcpp)

  add_executable(robot_ws_client_probe test/probes/robot_ws_client_probe.cpp)
  target_link_libraries(robot_ws_client_probe robot_gateway_runtime_core)
  ament_target_dependencies(robot_ws_client_probe rclcpp)

  install(TARGETS
    robot_protocol_probe
    response_waiter_probe
    robot_status_parser_probe
    action_result_builder_probe
    walk_velocity_controller_probe
    gesture_sync_probe
    motion_controller_probe
    robot_ws_client_probe
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(TARGETS
  robot_gateway_runtime_core
  robot_gateway_node
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
