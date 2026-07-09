# humanoid_prior_localization_runtime

`humanoid_prior_localization_runtime` 是机器人 OP/Open3D 先验地图定位运行层功能包。它负责把 Fast-LIO 的局部里程计和注册点云接入 Open3D 点云配准定位链路，并输出全局定位候选给 `prior_map_odom_bridge_cpp`，最终由桥接节点统一维护 `map->odom`。

这个包只服务可选的 `relocalization_engine:=op` / `prior` 定位链路。当前默认 RO/RoboSense 定位链路不会启动本包节点。

## 功能边界

- `fastlio_open3d_axis_adapter`：坐标轴适配节点。订阅 Fast-LIO 原始 `/odom` 和 `/fast_lio/cloud_registered`，转换为 Open3D 定位需要的标准轴 odom/点云。
- `global_localization_node`：Open3D 先验地图定位节点。订阅标准轴 odom/点云，将实时点云和先验 PCD 地图配准，输出全局定位候选和置信度。
- `open3d_registration`：Open3D 配准算法库，封装粗配准、多尺度 ICP 等能力。
- `open3d_conversions`：ROS PointCloud2 与 Open3D 点云之间的数据转换库。

## 主要文件

- `src/fastlio_open3d_axis_adapter.cpp`：Fast-LIO 到 Open3D 定位链路的输入适配节点。
- `src/global_localization.cpp`：Open3D 全局定位主节点。
- `src/open3d_registration/open3d_registration.cpp`：点云配准算法实现。
- `src/open3d_conversions/open3d_conversions.cpp`：ROS/Open3D 点云格式转换实现。
- `config/loc_param_g1.yaml`：Open3D 定位参数，包括初始位姿、Kalman 参数、配准阈值和 TF 发布开关。
- `launch/prior_localization_runtime.launch.py`：本包独立调试 launch。正式导航通常由 `humanoid_navigation2/launch/navigation2.launch.py` 编排。

## 数据流

```text
fast_lio_node
  -> /odom
  -> /fast_lio/cloud_registered
  -> fastlio_open3d_axis_adapter
  -> /prior_localization/open3d_input_odom
  -> /prior_localization/open3d_input_cloud
  -> global_localization_node
  -> /prior_localization/odom
  -> /prior_localization/confidence
  -> prior_map_odom_bridge_cpp
  -> map->odom
```

## 常用启动

正式导航中不建议单独启动本包，而是通过导航层选择 OP 链路：

```bash
ros2 launch humanoid_bringup robot_navigation_stack.launch.py relocalization_engine:=op
```

单独调试本包时可以运行：

```bash
ros2 launch humanoid_prior_localization_runtime prior_localization_runtime.launch.py
```

## 关键注意事项

- `global_localization_node` 不直接发布 `map->odom`，避免和 `prior_map_odom_bridge_cpp` 抢占全局 TF。
- `fastlio_open3d_axis_adapter` 的 `initial_body_to_base_translation_raw` 必须和 Fast-LIO/body 到导航 base 的外参保持一致。
- 先验地图必须是和本链路坐标轴一致的 grounded PCD 地图，否则会出现定位偏移或置信度异常。
- 本包默认支持按 CPU 架构选择包内 Open3D：`third_party/open3d/x86_64/open3d-0.18.0` 用于 PC/x86_64，`third_party/open3d/aarch64/open3d-0.18.0` 用于 Jetson/ARM64。
- 当前仓库已放入 x86_64 版本；如果要在 Jetson Orin NX 上离线编译，需要把 ARM64 编译版 Open3D 放到 `third_party/open3d/aarch64/open3d-0.18.0`。
- ARM64 机器上可以运行 `tools/install_open3d_aarch64.sh` 自动从 Open3D v0.18.0 源码构建 C++ SDK 并安装到上述目录。
- 如果需要临时切换 Open3D 版本，可以在 `colcon build` 时传入 `-DOpen3D_DIR=/path/to/Open3D`，或设置环境变量 `Open3D_DIR`。
