# fast_lio_robosense

## 这个包是做什么的

`fast_lio_robosense` 是当前机器人使用的 Fast-LIO 雷达惯性里程计包，已经适配 RoboSense Airy 雷达。

它负责融合雷达点云和 IMU，输出局部里程计、注册点云和建图结果。它是 RO/RoboSense 定位链路和 OP/Open3D 定位链路共同依赖的局部运动基础。

## 当前状态

- 当前导航和建图都会启动本包的 `fastlio_mapping` 可执行文件。
- 包内包含自定义消息 `Pose6D`，因此包名不建议轻易修改。
- 当前配置文件为 `config/robosenseAiry.yaml`。
- 默认使用 RoboSense Airy 96 线雷达配置。
- 已根据 CPU 核心数配置 OpenMP 线程数，避免 Fast-LIO 独占全部 CPU。

## 主要输出

- `/odom`：Fast-LIO 局部里程计。
- `/fast_lio/cloud_registered`：Fast-LIO 注册点云。
- `/cloud_registered`：兼容历史工具的点云输出。
- `/map_save`：保存当前 Fast-LIO PCD 地图的服务。
- `odom -> camera_init -> body` 等局部 TF 链路。

## 主要文件说明

- `src/laserMapping.cpp`：Fast-LIO 主流程，负责里程计估计、地图维护、发布点云和 TF。
- `src/preprocess.cpp`：雷达点云预处理。
- `include/`：Fast-LIO 算法头文件、ikd-tree、消息和工具定义。
- `msg/Pose6D.msg`：Fast-LIO 自定义消息。
- `config/robosenseAiry.yaml`：RoboSense Airy 实机参数。
- `launch/mapping_robosense_airy.launch.py`：单独启动 Fast-LIO 的 launch 文件。

## 上下游链路

上游：

- `rslidar_sdk` 发布 `/rslidar_points` 和 `/rslidar_imu_data`。

下游：

- `humanoid_robosense_localization_runtime` 使用 `/odom` 和 `/fast_lio/cloud_registered` 做 RO 全局定位。
- `humanoid_prior_localization_runtime` 使用 Fast-LIO 输出做 OP/Open3D 先验地图定位。
- `humanoid_point_cloud_filter` 使用点云做过滤和 costmap 输入。
- 建图脚本调用 `/map_save` 保存 PCD 地图。

## 使用方式

正式导航通常由一键脚本启动：

```bash
./start_navigation.sh
```

单独启动 Fast-LIO：

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
ros2 launch fast_lio_robosense mapping_robosense_airy.launch.py
```

保存地图：

```bash
ros2 service call /map_save std_srvs/srv/Trigger '{}'
```

## 维护注意事项

- 本包含有自定义消息，改包名会牵扯消息命名空间和所有下游引用，风险高。
- 修改坐标系、外参、IMU 参数或时间同步参数后，必须重新验证定位、点云滤波和 Nav2。
- Fast-LIO 是 CPU 消耗核心节点之一，性能优化要同时关注帧率、漂移、点云质量和定位稳定性。
