# humanoid_point_cloud_filter

## 这个包是做什么的

`humanoid_point_cloud_filter` 是导航点云滤波运行包，负责把雷达/定位链路中的点云处理成更适合 Nav2、ROI 障碍检测和可视化使用的过滤点云。

它是当前导航资源消耗较高的核心 C++ 节点之一，主要计算量来自点云降采样、离群点过滤、高度过滤、密度过滤、坐标变换和调试输出。

## 当前状态

- 默认由 `humanoid_navigation2` launch 间接启动。
- 主要输入通常是 Fast-LIO 或雷达相关点云。
- 主要输出是 `/airy_points_filtered` 等过滤后的点云。
- 当前参数已经包含多项性能优化开关，例如 pre-voxel、合并 KDTree、关闭 elevation 输出和高度连续性过滤等。
- SYCL/GPU 路径保留在代码中，但当前默认仍以 CPU/OpenMP 路径为主。

## 主要文件说明

- `src/point_cloud_filter_node.cpp`：ROS2 节点入口，负责参数、订阅、发布、TF 和处理流程调度。
- `src/point_cloud_filter_core.cpp`：核心滤波算法实现。
- `src/sycl_kernels.cpp`：SYCL/GPU 相关实验内核。
- `include/humanoid_point_cloud_filter/point_cloud_filter_node.hpp`：节点类定义。
- `include/humanoid_point_cloud_filter/point_cloud_filter_core.hpp`：核心算法接口。
- `include/humanoid_point_cloud_filter/sycl_kernels.hpp`：SYCL 内核接口。
- `config/point_cloud_filter_config.yaml`：滤波参数配置，包含性能和质量相关开关。
- `launch/point_cloud_filter.launch.py`：单独启动点云滤波节点的 launch 文件。

## 上下游链路

上游：

- `rslidar_sdk` 原始点云。
- `fast_lio_robosense` 注册点云。
- TF 系统提供点云坐标变换。

下游：

- Nav2 costmap 点云层。
- `humanoid_obstacle_runtime` ROI 障碍检测。
- RViz 或调试脚本查看过滤效果。
- 可选动态障碍层 `humanoid_decay_obstacle_layer`。

## 使用方式

正式导航中由导航 launch 启动。单独调试：

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
ros2 launch humanoid_point_cloud_filter point_cloud_filter.launch.py
```

查看输出频率：

```bash
ros2 topic hz /airy_points_filtered
```

查看点云大小：

```bash
ros2 topic echo /airy_points_filtered --once
```

## 维护注意事项

- 这是点云链路核心包，参数优化需要同时看 CPU、延迟、帧率和点云质量。
- 降采样越激进，CPU 越低，但可能影响 ROI 障碍检测和 costmap 质量。
- 关闭某一路输出前，要确认下游没有订阅该话题。
- GPU/SYCL 路径需要硬件、驱动、oneAPI 和编译环境共同满足，不能只改开关。
