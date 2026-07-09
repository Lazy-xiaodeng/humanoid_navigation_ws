cmake_minimum_required(VERSION 3.8)
project(humanoid_expression_runtime)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(facial_driver_cpp
  src/facial_driver.cpp
)
ament_target_dependencies(facial_driver_cpp
  ament_index_cpp
  rclcpp
  std_msgs
)
target_link_libraries(facial_driver_cpp
  yaml-cpp
)

install(TARGETS
  facial_driver_cpp
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
