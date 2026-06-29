cmake_minimum_required(VERSION 3.8)
project(humanoid_broadcast_runtime)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(humanoid_interfaces REQUIRED)

add_executable(broadcast_service_node_cpp
  src/broadcast_service_node.cpp
)

add_executable(call_broadcast_service_cpp
  src/call_broadcast_service.cpp
)

ament_target_dependencies(broadcast_service_node_cpp
  rclcpp
  humanoid_interfaces
)

ament_target_dependencies(call_broadcast_service_cpp
  rclcpp
  humanoid_interfaces
)

install(TARGETS
  broadcast_service_node_cpp
  call_broadcast_service_cpp
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
