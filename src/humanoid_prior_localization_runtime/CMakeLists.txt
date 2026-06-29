cmake_minimum_required(VERSION 3.8)
project(humanoid_prior_localization_runtime)

# 先验地图定位运行层：
# - global_localization_node 负责 Open3D 点云全局定位。
# - fastlio_open3d_axis_adapter 负责把 Fast-LIO 原始 odom/点云转换为 Open3D 使用的标准轴输入。
if(NOT CMAKE_CXX_STANDARD)
        set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

set(CMAKE_BUILD_TYPE "Release")

# 查找 ROS、PCL、OpenCV、Open3D 等依赖。
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(urdf REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)

# Set Open3D path.
# 优先使用 colcon/cmake 命令行传入的 -DOpen3D_DIR=...；
# 如果没有传入，再尝试读取环境变量 Open3D_DIR。
# 这样不会把外部作者机器上的 /home/sax/open3d141 写死到本工作区。
if(NOT Open3D_DIR AND DEFINED ENV{Open3D_DIR})
        set(Open3D_DIR $ENV{Open3D_DIR})
endif()

# set(Open3D_DIR "/home/unitree/open3d141_arm/open3d141/lib/cmake/Open3D")
find_package(Open3D REQUIRED)

# 修正 Open3D 导出的部分历史库路径。
# Ubuntu 22.04/24.04 上 librt、libc++ 等库路径可能和 Open3D 预编译包记录不一致，
# 这里过滤掉不存在或不需要的链接项，避免构建阶段误报。
foreach(target Open3D::Open3D Open3D::3rdparty_filament Open3D::3rdparty_glfw Open3D::3rdparty_rply Open3D::3rdparty_glew)
        if(TARGET ${target})
                get_target_property(LINK_LIBS ${target} INTERFACE_LINK_LIBRARIES)

                if(LINK_LIBS)
                        # Create a new list with problematic libraries removed
                        set(FILTERED_LIBS "")

                        foreach(lib ${LINK_LIBS})
                                # Skip problematic libraries
                                if(NOT "${lib}" MATCHES ".*librt\\.so.*" AND
                                        NOT "${lib}" MATCHES ".*libc\\+\\+\\.so.*" AND
                                        NOT "${lib}" MATCHES ".*libc\\+\\+abi\\.so.*")
                                        list(APPEND FILTERED_LIBS ${lib})
                                endif()
                        endforeach()

                        # Set the filtered list back
                        set_target_properties(${target} PROPERTIES INTERFACE_LINK_LIBRARIES "${FILTERED_LIBS}")
                endif()
        endif()
endforeach()

# 同步过滤 Open3D_LIBRARIES 变量，保证 target 和变量两种链接方式都安全。
if(Open3D_LIBRARIES)
        set(FILTERED_O3D_LIBS "")

        foreach(lib ${Open3D_LIBRARIES})
                if(NOT "${lib}" MATCHES ".*librt\\.so.*" AND
                        NOT "${lib}" MATCHES ".*libc\\+\\+\\.so.*" AND
                        NOT "${lib}" MATCHES ".*libc\\+\\+abi\\.so.*")
                        list(APPEND FILTERED_O3D_LIBS ${lib})
                endif()
        endforeach()

        set(Open3D_LIBRARIES ${FILTERED_O3D_LIBS})
endif()

include_directories(
        include
        ${Open3D_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${YAMLCPP_INCLUDE_DIRS}
)

# Open3D 配准算法库：封装粗配准、多尺度 ICP 等核心配准能力。
add_library(open3d_registration src/open3d_registration/open3d_registration.cpp)
target_link_libraries(open3d_registration ${Open3D_LIBRARIES})
ament_target_dependencies(open3d_registration
        rclcpp
        sensor_msgs
        geometry_msgs
)

# ROS/Open3D 点云转换库：负责 PointCloud2 和 Open3D 点云之间的数据转换。
add_library(open3d_conversions src/open3d_conversions/open3d_conversions.cpp)
target_link_libraries(open3d_conversions ${Open3D_LIBRARIES})
ament_target_dependencies(open3d_conversions
        rclcpp
        sensor_msgs
        geometry_msgs
        std_msgs
        tf2
        tf2_ros
        tf2_geometry_msgs
        tf2_eigen
        urdf
)

# 全局定位节点：订阅标准轴 odom/点云，输出先验地图定位结果和置信度。
add_executable(global_localization_node src/global_localization.cpp)
target_link_libraries(global_localization_node
        ${Open3D_LIBRARIES}
        open3d_registration
        open3d_conversions
)
ament_target_dependencies(global_localization_node
        rclcpp
        std_msgs
        geometry_msgs
        nav_msgs
        sensor_msgs
        tf2
        tf2_ros
        tf2_geometry_msgs
        tf2_eigen
        pcl_ros
        pcl_conversions
)

# 坐标轴适配节点：把 Fast-LIO 原始输出变成 Open3D 定位使用的标准轴输入。
add_executable(fastlio_open3d_axis_adapter src/fastlio_open3d_axis_adapter.cpp)
target_compile_features(fastlio_open3d_axis_adapter PUBLIC cxx_std_17)
target_link_libraries(fastlio_open3d_axis_adapter Eigen3::Eigen)
ament_target_dependencies(fastlio_open3d_axis_adapter
        geometry_msgs
        nav_msgs
        rclcpp
        sensor_msgs
        tf2
        tf2_ros
)

# 安装库和可执行文件。
install(TARGETS
        open3d_registration
        open3d_conversions
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(TARGETS
        global_localization_node
        fastlio_open3d_axis_adapter

        DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch、配置和 RViz 资源。
install(DIRECTORY
        launch
        config
        rviz_cfg
        DESTINATION share/${PROJECT_NAME}/
)

# 安装公共头文件。
install(DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        ament_lint_auto_find_test_dependencies()
endif()

ament_package()
