# humanoid_robosense_localization_runtime

## 这个包是做什么的

`humanoid_robosense_localization_runtime` 是 RO/RoboSense 先验地图定位运行包。

它订阅 Fast-LIO 的局部里程计和注册点云，将当前点云与先验 PCD 地图进行匹配，输出全局定位候选位姿。最终的 `map -> odom` TF 不由本包直接发布，而是交给 `humanoid_localization_runtime` 中的 `prior_map_odom_bridge_cpp` 统一维护。

## 当前状态

- 当前默认导航链路使用 RO/RoboSense 定位模式。
- 包名已经统一为 `humanoid_robosense_localization_runtime`。
- 可执行文件名仍保留为 `robosense_lidar_localization_node`，便于日志识别和最小化运行时变化。
- 配置文件名仍保留为 `robosense_lidar_localization.yaml`，由 `start_navigation_stack.sh` 按地图生成 runtime 配置。

## 主要文件说明

- `node/lidar_localization_node.cpp`：ROS2 节点入口，负责参数读取、话题订阅发布、手动初始位姿、odom cache 和定位输出。
- `lidar_localization/`：定位核心逻辑，负责地图加载、点云匹配和位姿估计。
- `lidar_matcher/`：点云匹配模块和优化求解相关代码。
- `common/`：通用数据结构、坐标变换、配置读取和工具函数。
- `config/robosense_lidar_localization.yaml`：实机导航主配置。
- `config/robosense_lidar_localization_bag.yaml`：bag 回放或隔离验证配置。
- `launch/robosense_lidar_localization.launch.py`：单独启动本定位节点的 launch 文件。

## 上下游链路

上游：

- `fast_lio_robosense`：发布 `/odom` 和 `/fast_lio/cloud_registered`。
- `humanoid_localization_runtime/rviz_initialpose_adapter`：发布 `/prior_localization/manual_initialpose`，用于手动重定位。
- 地图配置：先验 PCD 地图路径由 runtime 配置传入。

下游：

- `/prior_localization/robosense_odom`：输出全局定位候选。
- `/prior_localization/robosense_input_odom`：输出给 bridge 按时间戳插值使用的 odom cache。
- `prior_map_odom_bridge_cpp`：融合定位候选和局部 odom，发布 `map -> odom`。

## 使用方式

正式导航通常由一键脚本启动：

```bash
./start_navigation.sh
```

单独启动本包：

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
ros2 launch humanoid_robosense_localization_runtime robosense_lidar_localization.launch.py
```

查看输出：

```bash
ros2 topic hz /prior_localization/robosense_odom
ros2 topic echo /prior_localization/robosense_odom --once
```

## 维护注意事项

- 本包只负责输出全局定位候选，不应直接抢发 `map -> odom`。
- 修改外参、坐标系或点云转换参数前，必须同步验证 `prior_map_odom_bridge_cpp`、Nav2 和 RViz 中的机器人位姿。
- 可执行文件名和节点名目前保持 `robosense_lidar_localization_node`，不要误认为这是旧包名。
- 如果后续改配置文件名，也要同步 `start_navigation_stack.sh` 和多地图 runtime 配置生成逻辑。
