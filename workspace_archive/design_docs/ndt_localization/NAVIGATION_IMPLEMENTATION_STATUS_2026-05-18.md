# 机器人导航功能当前实现情况总结

更新时间：2026-05-18  
工作空间：`/home/ubuntu/humanoid_ws`  
依据：当前源码、launch/config、安装空间可执行节点、最近修改说明和现有文档。本文只做分析总结，不代表已完成实机全流程验收。

## 总体结论

当前导航系统已经具备完整的实机导航链路：

```text
雷达/IMU -> Fast-LIO -> 点云滤波/清障 -> HDL bootstrap 全局重定位
         -> /initialpose -> lidar_localization_ros2 持续 NDT 定位
         -> map->odom TF -> Nav2 分阶段导航 -> /cmd_vel -> WebSocket 底层行走
         -> APP 点位/导航状态/失败恢复交互
```

实现重点已经从“能启动 Nav2”推进到“定位初始化、定位防跳、代价地图残影清除、目标朝向分阶段处理、APP 可恢复失败交互”这一层。当前更像是实机联调阶段，而不是最终收敛版本。高优先级风险主要集中在坐标系一致性、定位 bootstrap 失败时的启动卡住、costmap 高度参数、APP 状态管理使用 `/odom` 位姿、以及多个备用/历史文件与当前主入口不同步。

## 当前实际启动入口

当前主入口是：

```bash
/home/ubuntu/humanoid_ws/start_navigation.sh
```

该脚本会 source ROS 2 Jazzy 和工作空间 `install/setup.bash`，检查 Python 包元数据，然后启动：

```bash
ros2 launch humanoid_bringup robot_real.launch.py use_sim_time:=false
```

`robot_real.launch.py` 当前编排如下：

- 立即启动 `humanoid_description/display.launch.py`，但不启动 RViz。
- 延迟 6 秒启动 `humanoid_navigation2/launch/navigation_stack.launch.py`。
- 延迟 9 秒启动应用层：
  - `humanoid_navigation/launch/navigation.launch.py`
  - `humanoid_websocket/launch/websocket_server.launch.py`
  - `humanoid_locomotion/launch/locomotion.launch.py`
- 延迟 10 秒启动 RViz，默认启用。

需要注意：`navigation_stack.launch.py` 是当前真实入口；`navigation2.launch.py` 仍存在，但其 `odom_ground` 等实现与主入口不完全一致，应视为备用或历史入口，后续需要统一。

## 已实现的导航底层链路

### 传感器与 Fast-LIO

`navigation_stack.launch.py` 启动：

- `rslidar_sdk_node`
- `fast_lio_robosense/fastlio_mapping`

Fast-LIO remap：

- `/CloudPoints` -> `/airy_points`
- `/Imu` -> `/airy_imu`
- `/Odometry` -> `/odom`
- `/cloud_registered` -> `/fast_lio/cloud_registered`

`fast_lio_robosense/src/laserMapping.cpp` 已改回使用雷达扫描时间戳发布 `/odom` 和 `camera_init->body` TF，而不是系统 wall time。这个修改是正确方向，因为点云消费者需要同一帧点云对应同一时间的 Fast-LIO TF。

### TF 架构

主栈当前使用的关键 TF：

```text
map -> map_ground
map -> odom
odom -> odom_ground
odom -> camera_init -> body -> base_footprint
base_footprint -> clearing_lidar
```

其中：

- `map->map_ground` 是静态 TF，当前 z 为 `-1.215`。
- `odom->odom_ground` 由 `dynamic_odom_ground_publisher` 动态发布，根据 `odom->base_footprint` 高度让 local costmap 地面基准贴近 `base_footprint`。
- `body->base_footprint` 使用静态外参。
- `base_footprint->clearing_lidar` 静态发布给清障射线使用。

Nav2 使用：

- 全局层：`map_ground`
- 局部层：`odom_ground`
- 机器人基准：`base_footprint`

### 定位初始化与持续定位

当前主线不是单一定位器，而是组合方案：

1. `hdl_global_localization` + `hdl_localization` 作为 bootstrap，只做启动阶段全局重定位。
2. bootstrap 输出 `/hdl_bootstrap/odom`，不发布 `map->odom` TF。
3. `hdl_bootstrap_to_initialpose` 等待 HDL 输出稳定样本，把 `map->base` 和当前 `odom->base` 换算为 `map->odom`，向 `/initialpose` 连续发布 8 次。
4. `lidar_localization_ros2/lidar_localization_node` 接收 `/initialpose` 后，持续 NDT 跟踪并独占发布 `map->odom`。
5. `wait_for_tf` 等待 `map_ground->base_footprint` 连续可查后，才启动 Nav2 core nodes。
6. bootstrap helper 退出后，会 SIGINT 停掉 bootstrap container 和 bootstrap global localization。

这条链路的优点是：启动时可在非原点位置依赖 HDL/FPFH/RANSAC 找初始位姿，稳定后交给更简单的 lidar_localization 持续跟踪。

### lidar_localization_ros2 改动

`lidar_localization/src/lidar_localization_component.cpp` 已加入：

- `/initialpose` frame 不一致时自动 TF 转换到 `global_frame_id`。
- `force_2d_pose`、`force_2d_fixed_z`、`force_2d_z` 参数。
- NDT 输出发布前投影为平面 yaw-only 姿态。
- 保存 `last_scan_ptr_`，手动或自动 initialpose 后可立即用最新点云重定位。

当前参数：

- `global_frame_id: map`
- `odom_frame_id: odom`
- `base_frame_id: odom`
- `map_path: /home/ubuntu/humanoid_ws/src/humanoid_navigation2/pcd/hall.pcd`
- `set_initial_pose` 在参数文件中为 `true`，但主 launch 覆盖为 `false`
- `force_2d_pose: true`
- `force_2d_fixed_z: false`

### HDL/NDT 防跳变能力

`hdl_localization` 已实现较完整的防错误匹配逻辑：

- 扫描匹配可按 `fitness`、`inlier_fraction`、单帧平移/yaw 跳变量拒绝。
- 支持 2D 投影约束。
- 支持点云 TF 短等待，避免点云先于同时间戳 TF 到达导致丢帧。
- 拒绝后可短时使用 odom prediction，也可在超出预算后冻结最后可信位姿。
- 全局定位候选会做 XY 边界、连续一致性、post-accept validation。

当前主入口里 bootstrap 的关键参数包括：

- `global_localization_required_consistent_results: 3`
- `global_localization_query_accumulation_frames: 8`
- `global_localization_post_accept_validation_frames: 5`
- `global_localization_enforce_xy_bounds: true`
- `max_scan_matching_fitness_score: 0.20`
- `min_scan_matching_inlier_fraction: 0.78`
- `max_scan_matching_correction_translation: 0.80`
- `max_scan_matching_correction_yaw: 0.45`
- `publish_odom_prediction_on_rejection: false`

注意：现有说明文档中有些阈值和当前主 launch 不完全一致，应以后续实机验证值为准。

## 已实现的 Nav2 导航策略

### 分阶段 XY/Yaw 行为树

当前默认参数文件：

```text
src/humanoid_navigation2/config/nav2_params_xy_yaw.yaml
```

当前默认行为树：

```text
src/humanoid_navigation2/config/behavior_tree/navigate_xy_then_yaw.xml
```

自定义 BT 插件包：

```text
src/humanoid_nav2_bt_nodes
```

已确认安装空间存在：

```text
install/humanoid_nav2_bt_nodes/lib/libhumanoid_nav2_bt_nodes.so
```

行为树主流程：

```text
SpinToPose(mode=goal_position)
-> MakePoseTowardGoal
-> ComputePathToPose(planner_id=GridBased)
-> FollowPath(goal_checker_id=xy_goal_checker)
-> SpinToPose(mode=goal_yaw)
```

设计目标是避免 Hybrid/Dubins 为了满足最终 yaw 生成绕圈路径。现在路径规划只负责到目标 XY，最终朝向由 `behavior_server` 的 Spin action 单独完成。

### 控制器与规划器

当前 Nav2 核心配置：

- 控制器：`nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`
- 控制频率：`8 Hz`
- 期望线速度：`0.30 m/s`
- `allow_reversing: false`
- `use_rotate_to_heading: false`
- 规划器：`nav2_smac_planner::SmacPlanner2D`
- 全局 unknown：`allow_unknown: false`
- XY-only goal checker：`PositionGoalChecker`
- 当前 XY 容差：`0.40 m`
- 当前最终 yaw 容差：`0.45 rad`
- 行为插件实际加载：`spin`、`wait`

当前 `backup` 参数块仍保留在 yaml 中，但 `behavior_plugins` 没有加载 `backup`，当前行为树也没有 BackUp 分支。这符合“后方无传感器，避免倒车”的安全策略，但配置注释需要同步清理。

## 已实现的代价地图与清障

### 点云滤波

`humanoid_point_cloud_filter` 当前按点云时间戳查询 TF：

- 输入点云先转到 `body`
- 再转到 `base_footprint`
- 输出仍使用原点云时间戳

这能降低点云在 Fast-LIO 动态 TF 前后错位的问题。

### Local costmap

当前 local costmap：

- `global_frame: odom_ground`
- `robot_base_frame: base_footprint`
- `plugins: static_layer, voxel_layer, inflation_layer`
- marking 点云：`/airy_points_filtered`
- clearing 点云：`/clearing_cloud_3d`

`periodic_clearing_3d_publisher` 实际输出 `/clearing_cloud_3d`，与配置匹配。它以 `/fast_lio/cloud_registered` 作为 heartbeat，按 2 Hz 生成 360 度、多距离、多高度的清障 endpoint，只做 clearing 不做 marking。

### Global costmap

当前 global costmap：

- `global_frame: map_ground`
- `plugins: static_layer, obstacle_layer, inflation_layer`
- pointcloud observation source 存在，但 `marking: False`、`clearing: False`
- 主要动态清除依靠 `/clearing_scan`

这意味着动态障碍主要由 local costmap 处理，全局规划基本依赖静态地图。该策略可减少全局残影，但需要验证远处动态障碍不会导致路径反复局部失败。

## 已实现的 APP/任务层能力

### 点位管理

`dynamic_waypoints_manager` 负责：

- APP 点位增删改查
- 点位数据持久化到 `/home/ubuntu/humanoid_ws/data/dynamic_waypoints.json`
- 启动后多次初始发布已有点位给状态管理器
- 将单点、多点、展台导航请求转发给状态管理器

### 导航状态管理

当前实际启动的是：

```text
humanoid_navigation.navigation_state_manager_recoverable
```

已实现：

- 单点导航
- 多点导航
- 展台导航
- 暂停、恢复、停止
- Nav2 action feedback 进度转发
- 到点仅信 Nav2 action success，不再用本地距离直接判定到点
- 非 Walk 或动作执行中时缓存导航请求，等待机器人回到 Walk 再执行
- Nav2 失败进入 `recoverable_failed`，支持：
  - `retry_failed_waypoint`
  - `skip_failed_waypoint`
  - `stop_navigation`
- 障碍物阻塞检测：
  - 导航执行中速度低于阈值开始计时
  - Nav2 Spin/Recovery 阶段和接近目标阶段会抑制误报
  - 超时后上报 `navigation_obstacle_blocked`，但不主动取消 Nav2

### WebSocket 与数据集成

`websocket_server.launch.py` 当前启动：

- `websocket_server_node`
- `data_integration_node_recoverable`
- `websocket_client_node`
- `message_bridge_node`

`data_integration_node_recoverable` 已实现：

- `/robot_realpose` 作为 APP 主定位数据源
- `/odom` 只用于速度数据，且将 Fast-LIO 非标准轴转换为标准机器人速度
- 导航状态增强，包括当前 pose、path、速度、估算剩余时间
- 对关键离散导航事件立即推送，避免到达/失败/跳过事件被周期推送漏掉
- 保留 recoverable 失败上下文和 available actions 透传到 APP

`websocket_client` 已实现：

- `/cmd_vel` 到机器人 `request_set_walk_vel_sync`
- 只在 Walk 状态且没有动作执行时下发速度
- 上半身动作执行时切 Menu，动作结束后切回 Walk
- 动作库 OTA 拉取并更新 gestures.yaml

## 当前未完成或需要收敛的点

### 1. 主入口与备用入口需要统一

`navigation_stack.launch.py` 是当前主入口；`navigation2.launch.py` 仍存在且部分实现不同，例如 `odom_ground` 处理方式与主入口不一致。后续需要明确：

- 保留哪个作为唯一实机入口。
- 另一个删除、改名为 legacy，或同步到完全一致。

### 2. recoverable 与非 recoverable 文件存在重复维护风险

当前实际启动：

- `navigation_state_manager_recoverable.py`
- `data_integration_node_recoverable.py`

但仓库中仍有：

- `navigation_state_manager.py`
- `data_integration_node.py`

两套文件内容相近但不完全一致。后续建议合并成单一实现，通过参数开关控制失败策略，避免修一个漏一个。

### 3. 状态管理器仍用 `/odom` 位姿计算距离和进度

这是高风险点。

`navigation_state_manager_recoverable` 在 `odom_callback()` 中把 `/odom` 的 pose 当成 `current_pose`，并用于：

- `distance_to_goal`
- 进度百分比
- 接近目标时抑制障碍物阻塞误报

但 `/odom` 来自 Fast-LIO，header frame 是 `camera_init`，不是全局 `map` 或 `map_ground`。APP 位姿数据已经改用 `/robot_realpose`，但状态管理器自身还没有切换。

建议后续改为：

- 订阅 `/robot_realpose` 用于全局 pose、距离和进度。
- `/odom` 只保留速度来源。
- 或直接通过 TF 查询 `map/map_ground -> base_footprint`。

### 4. 地图边界和 HDL 全局定位边界需要重算

当前 `hall.yaml`：

```text
resolution: 0.050
origin: [-6.648, -13.571, 0]
hall.pgm size: 673 x 799
```

按当前地图计算，XY 约为：

```text
x: -6.648 到 27.002
y: -13.571 到 26.379
```

但 `navigation_stack.launch.py` 里 HDL 全局定位候选边界仍是：

```text
x: -6.675 到 26.325
y: -12.819 到 26.231
```

这会让地图边缘约 0.6 到 0.8 m 区域可能被候选边界拒绝。地图更新后应同步重算这些边界。

### 5. VoxelLayer Z 覆盖范围需要复核

当前 local VoxelLayer：

```yaml
origin_z: -1.0
z_resolution: 0.15
z_voxels: 16
max_obstacle_height: 1.8
```

理论覆盖高度约为 `-1.0` 到 `1.4 m`。但注释和 clearing 高度层都在覆盖到 `1.7/1.8 m`。需要实测 `/local_costmap/voxel_grid` 或 RViz，确认高障碍物是否被正确标记/清除。

### 6. 启动等待定位没有超时策略

`wait_for_tf` 当前 `timeout_sec: 0.0`，即无限等待 `map_ground->base_footprint`。如果 HDL bootstrap 或 lidar_localization 初始化失败，Nav2 会一直不启动。

建议至少增加：

- 日志提示明确显示当前卡在定位 ready。
- APP/状态层可见的“定位初始化中/定位失败”状态。
- 可选 timeout 后进入可恢复错误状态。

### 7. 部分 launch 参数声明但没有真正控制节点

例如：

- `enable_periodic_clearing`
- `enable_fastdds_shm`

当前主入口中这些参数被声明，但 clearing 节点和 FastDDS 环境变量实际总是启用。需要改成 `IfCondition` 或删除开关，避免误以为可关闭。

### 8. 硬编码路径较多

当前存在多个绝对路径：

- PCD 地图路径
- BT XML 路径
- `/home/ubuntu/humanoid_ws/...`

在当前机器上可用，但对换机器、重新安装、打包部署不友好。建议统一使用 `get_package_share_directory()`、launch 参数和安装空间路径。

### 9. `humanoid_navigation2/package.xml` 存在疑似无效依赖

当前工作区实际包名是 `lidar_localization_ros2`，但 `humanoid_navigation2/package.xml` 同时声明了：

```xml
<depend>lidar_localization</depend>
<depend>lidar_localization_ros2</depend>
```

`colcon list` 和 `ros2 pkg list` 都没有 `lidar_localization` 这个包名。建议删除无效依赖，避免 rosdep 或干净环境构建失败。

### 10. 文档和注释已有部分过期

例如现有分阶段导航文档里提到的部分容差、BackUp 插件加载方式，与当前 `nav2_params_xy_yaw.yaml` 已不完全一致。建议后续把旧文档更新或标注为历史版本。

## 需要重点测试的风险点

### P0：启动闭环

目标：确认从 `start_navigation.sh` 到 APP 可用的全链路能稳定启动。

测试项：

- `rslidar_sdk_node`、`fast_lio_node` 正常出数据。
- `/fast_lio/cloud_registered` 有稳定频率。
- `/airy_points_filtered` 有稳定频率。
- HDL bootstrap 能调用 `/relocalize` 并输出稳定 `/hdl_bootstrap/odom`。
- `/initialpose` 发布后 `lidar_localization` 开始发布 `map->odom`。
- `wait_for_tf` 退出，Nav2 core nodes 被启动并激活。
- `/navigate_to_pose` action server 可用。
- APP 能收到 `robot_pose`、`navigation_status`、`system_status`。

### P0：TF 与坐标系一致性

测试项：

- RViz Fixed Frame 分别设为 `map` 和 `map_ground`，确认机器人位置不跳、不双重偏移。
- 检查 `map->map_ground->odom->odom_ground->camera_init->body->base_footprint` 链路。
- 发送 `frame_id=map` 和 `frame_id=map_ground` 的目标，确认 Nav2 都能正确规划。
- 验证 APP 保存的点位 frame 是否与 Nav2 全局 frame 一致。

### P0：状态管理器距离/进度准确性

测试项：

- 对比 `/navigation/status` 中 `distance_to_goal` 与 RViz 中机器人到目标的真实距离。
- 对比 `/odom` pose、`/robot_realpose` pose 和 TF `map->base_footprint`。
- 机器人被 NDT 修正 `map->odom` 后，确认 APP 进度不会因为 `/odom` 局部坐标而明显错误。

如果此项不通过，应优先修状态管理器的位姿来源。

### P0：分阶段导航行为

测试项：

- 目标在正前方，确认正常规划和到点。
- 目标在侧方，确认先 Spin 到目标方向，再前进。
- 目标在身后，确认不倒车，先转向再前进。
- 目标最终 yaw 与路径方向相差 90/180 度，确认到 XY 后才执行最终 Spin，不生成绕圈路径。
- 最终 Spin 中机器人发生少量 XY 漂移时，行为树不反复回到 XY 阶段。

### P0：定位防跳变

测试项：

- 走廊弯角、相似门框、相似墙面区域反复行走。
- 记录 `fitness`、`inlier_fraction`、`jump_rejected`、`inlier_rejected`。
- 验证坏匹配不会把机器人瞬间拉到墙边。
- 验证阈值不会过严导致长期只靠 Fast-LIO 漂移。

### P1：local costmap 与清障

测试项：

- 动态障碍进入和离开后，local costmap 残影是否在 1 到 3 秒内清掉。
- 低矮障碍、人体腿部、箱子、桌腿是否能被 `/airy_points_filtered` 标记。
- 1.4 m 以上障碍是否被 VoxelLayer 忽略。
- `/clearing_cloud_3d` 是否因为 TF 或 QoS 被 MessageFilter 丢弃。
- 清障是否过强导致真实动态障碍闪烁或被过早清掉。

### P1：全局规划与动态障碍

测试项：

- 远处动态障碍不写入 global costmap 时，global planner 是否仍给出穿越动态障碍的路径。
- local costmap 是否能可靠阻挡并触发等待或重规划。
- 长时间动态障碍停在路径上时，APP 是否收到 `navigation_obstacle_blocked`。

### P1：失败可恢复交互

测试项：

- 单点导航失败后进入 `recoverable_failed`。
- APP 收到 `available_actions`。
- `retry_failed_waypoint` 能重新发送同一目标。
- `skip_failed_waypoint` 能跳过当前点并继续后续点。
- 最后一个点失败后选择 skip，完成状态和 skipped list 是否正确。
- `stop_navigation` 能清空 recoverable 上下文。

### P1：动作与导航互锁

测试项：

- 机器人在 Menu 执行动作时发起导航，请求应进入 pending。
- 动作结束并回到 Walk 后，pending 导航自动启动。
- 动作执行中 `/cmd_vel` 不应继续下发到底层。
- 切回 Walk 失败时，导航不应误启动。

### P1：地图更新一致性

测试项：

- 更新 `hall.pgm/hall.yaml/hall.pcd/hall_localization_grounded.pcd` 后，确认：
  - HDL 全局定位边界同步更新。
  - 2D 栅格和 3D PCD 坐标一致。
  - APP 点位仍落在新地图可行区域。
  - 旧点位不会因为 origin 改动整体偏移。

### P2：部署与干净环境构建

测试项：

- 从干净 build/install 重新 `colcon build`。
- 不依赖源码绝对路径启动。
- `rosdep` 不因 `lidar_localization` 无效依赖失败。
- `~/.config/fastdds_shm.xml` 不存在时，系统有明确报错或 fallback。

## 建议的下一步顺序

1. 先修或验证状态管理器位姿来源，避免 APP 距离/进度/近目标抑制使用 `/odom` 局部坐标。
2. 重算 HDL 全局定位 XY bounds，跟当前 `hall.yaml` 保持一致。
3. 实测 local VoxelLayer Z 覆盖，必要时调整 `origin_z/z_resolution/z_voxels`。
4. 把 `navigation_stack.launch.py` 固化为唯一主入口，同步或废弃 `navigation2.launch.py`。
5. 合并 recoverable 与非 recoverable 两套 Python 节点。
6. 增加定位初始化失败的用户可见状态和 timeout 策略。
7. 跑一轮实机测试矩阵：启动、定位、单点、多点、绕障、失败重试/跳过、动作互锁。

## 本次静态检查结果

已执行并确认：

- `colcon list` 能看到当前工作区导航相关包。
- `ros2 pkg executables` 能看到 `humanoid_navigation2`、`humanoid_navigation`、`humanoid_websocket`、`lidar_localization_ros2` 的关键可执行节点。
- `navigate_xy_then_yaw.xml` 通过 `xmllint --noout`。
- `humanoid_nav2_bt_nodes/package.xml` 通过 `xmllint --noout`。
- 自定义 BT 插件库 `install/humanoid_nav2_bt_nodes/lib/libhumanoid_nav2_bt_nodes.so` 存在。
- `hall.pcd` 和 `hall_localization_grounded.pcd` 在源码目录存在，安装空间中为指向 build/src 的符号链接。

未在本次执行：

- 未启动完整 `start_navigation.sh`。
- 未做实机导航动作测试。
- 未跑完整 `colcon build`。
- 未验证 rosbag 回放或 RViz costmap 可视化结果。
