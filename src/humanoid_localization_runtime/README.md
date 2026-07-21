# humanoid_localization_runtime

`humanoid_localization_runtime` 是机器人导航中的定位、TF 和真实位姿运行层 C++ 功能包。它把原先集中在综合包中的定位相关节点独立出来，方便后续维护、参数调试和资源评估。

## 包作用

- 维护 `map -> odom` 定位桥接 TF。
- 动态发布 `map_ground` 和 `odom_ground` 地面坐标系。
- 根据 TF 输出 `/robot_realpose`。
- 将 RViz `/initialpose` 转换给定位桥和 RoboSense 定位初始化链路。
- 在 Nav2 启动前等待定位 TF 就绪，避免导航核心节点过早启动。

## 文件说明

- `src/prior_map_odom_bridge_cpp.cpp`：定位桥主节点，融合外部定位和 odom，发布 `map -> odom` 与 `/localization/prior_map_odom_bridge_status`。
- `src/dynamic_odom_ground_publisher.cpp`：动态地面 TF 发布节点，根据机器人当前高度发布 `map_ground` 或 `odom_ground`。
- `src/robot_realpose_publisher.cpp`：机器人真实位姿发布节点，从 TF 查询全局位姿并发布 `/robot_realpose`。
- `src/rviz_initialpose_adapter.cpp`：RViz 初始位姿适配节点，把 `/initialpose` 同步给定位桥和 RoboSense。
- `src/wait_for_tf.cpp`：TF 就绪门控节点，等待指定 TF 连续稳定后退出，供 launch 继续启动 Nav2。
- `config/localization_runtime.yaml`：定位运行层正式参数，统一维护 bridge、可信度监督、恢复协调器和辅助节点配置。
- `launch/localization_runtime.launch.py`：独立调试 launch，便于单独启动定位辅助节点。
- `docs/全局重定位灰度集成修改记录.md`：`off/shadow/enforce` 权限、链路和验证记录。

## 上下游链路

上游：

- `fast_lio_node`：提供 `/odom` 和底盘相关 TF。
- `humanoid_robosense_localization_runtime/robosense_lidar_localization_node`：提供 `/prior_localization/odom` 等全局定位输入。
- RViz 或 APP：通过 `/initialpose` 设置初始位姿。

下游：

- Nav2：使用 `map_ground`、`odom_ground` 和 `map -> odom`。
- 路线运行层：订阅 `/localization/prior_map_odom_bridge_status` 判断定位健康。
- APP/数据整合层：订阅 `/robot_realpose` 展示机器人实时位置。
- `wait_for_tf` 启动门控：在 TF 稳定后退出，触发正式 launch 启动 Nav2 核心节点。

## 使用说明

正式导航通常由 `humanoid_navigation2` 的 launch 统一启动。单独调试时可使用：

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
ros2 launch humanoid_localization_runtime localization_runtime.launch.py
```

完整导航默认读取 `config/localization_runtime.yaml`，其中 `integration_mode: shadow`
只计算和记录结果，不写正式 RO、bridge、TF 或路线控制链路。灰度切换时应修改或选择
另一份定位运行层 YAML，并保证 `localization_trust_supervisor` 与
`global_relocalization_coordinator` 的 `integration_mode` 同时设为 `enforce`。

正式 launch 只保留节点启停、配置文件路径和 `use_sim_time` 等编排参数。例如：

```bash
ros2 launch humanoid_bringup robot_real.launch.py \
  localization_runtime_config_file:=/path/to/localization_runtime.yaml
```

无论采用何种模式，`prior_map_odom_bridge` 都是唯一的 `map -> odom` 发布者；RO 和全局
重定位节点只能提交候选，不能直接广播该 TF。

## 维护说明

定位相关 C++ 节点已经从旧综合包中独立到本包。后续如果定位桥、地面 TF、真实位姿或 RViz 初始位姿适配需要调整，优先在本包内维护。
