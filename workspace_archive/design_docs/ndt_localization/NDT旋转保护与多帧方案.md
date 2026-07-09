# NDT 旋转保护与多帧匹配改造方案

## 背景结论

点位5的问题主要发生在 `SpinToPose` 或点位切换后的短距离回补阶段。日志显示 NDT 在原地旋转期间给出了很低的 fitness，例如 `0.004`，但相对最后可信位姿产生约 `1.8m - 2.2m` 的平移跳变。也就是说问题不是“匹配质量差”，而是“错误位置也能匹配得很好”。

这类问题通常来自单帧点云在重复走廊、墙面、开阔厅等局部几何退化环境中的歧义。单帧 NDT 只看当前一帧，旋转时视角变化大、平移小，容易落入相邻局部极小值。

## 方案1：旋转保护

### 目标

在机器人原地旋转、点位最终朝向调整、倒走回补前后的短窗口内，不让 NDT 单帧大平移 correction 直接触发定位恢复。策略是“先冻结/降权大平移跳变，等旋转结束并静稳后再重新判断”。

### 回答关键疑问

如果机器人转弯后停两秒，再转向另一个方向，NDT 还能匹配上吗？

可以。旋转保护不是关闭 NDT，而是只在旋转窗口内禁止把大平移跳变当作真实定位更新，也避免把它立刻升级成 recovery。NDT 仍然每帧运行，仍然发布诊断。旋转结束并稳定后，如果 NDT 回到正常小 correction，就继续接受；如果静稳后仍然高 fitness 或持续跳变，再进入真正 recovery。

停留期间会不会直接报 jump？

设计上不应该直接报 `pose_jump` 给状态管理器。停留/settle 阶段应该发布新的诊断原因，例如 `rotation_guard_hold` 或 `rotation_guard_settle`，并重发 last good TF。只有超过保护窗口，或者静稳后连续多帧仍然跳变，才发布 `pose_jump`。

当前到达目标点和前往下一个目标点之间只有 `1s - 2s`，够 NDT 重新稳定吗？

一般够做“连续健康帧确认”，但不够做完整全局重定位。当前雷达/NDT 频率约 `10Hz` 量级，`1s` 能看到约 10 帧，`2s` 能看到约 20 帧。若只是要求 NDT 在静稳后连续 `3 - 5` 帧 `accepted/ok`，理论上够；如果已经进入 ScanContext/HDL 全局重定位，`1s - 2s` 通常不够。

因此目标点切换前不应只靠固定 sleep，而应加一个“定位稳定门槛”：

- 到点最终旋转完成后，进入 `settle`。
- 在 `settle` 内要求 NDT 连续 `3 - 5` 帧正常，且 correction 小。
- 如果 `settle_timeout_sec` 到期仍不稳定，则不要直接发下一个点位，先进入 recovery 或保持暂停。

推荐初值：

- `arrival_localization_settle_min_sec: 0.5`
- `arrival_localization_settle_max_sec: 2.0`
- `arrival_required_healthy_frames: 5`
- `arrival_max_correction_translation: 0.20m`
- `arrival_max_correction_yaw: 0.10rad`

### 触发条件

推荐优先用状态管理器已有行为状态：

- `navigation_state_manager_fusion.py` 已能识别 `SpinToPose`、`Spin`、`BackUp`。
- 这些状态可作为旋转保护的强信号。

定位节点侧可再加一个 `/cmd_vel` 订阅作为兜底：

- `abs(angular.z) > 0.2 rad/s`
- `hypot(linear.x, linear.y) < 0.05 m/s`
- 连续满足 `0.2s` 后进入 rotating。

退出条件：

- `abs(angular.z) < 0.05 rad/s` 且线速度很小，持续 `1.0s - 2.0s`。
- 或行为树不再处于 `SpinToPose/Spin/BackUp`。

更精确的 guard 时机建议：

1. 前置转向 guard
   - 从 BT 进入 `SpinToPose` 或 `/cmd_vel.angular.z` 超阈值开始。
   - 到旋转速度归零后进入 settle。

2. 目标点最终朝向 guard
   - Nav2 直线段到达后，开始 `goal_yaw` 的 `SpinToPose` 时进入。
   - 不能在 controller `Reached the goal!` 后立刻结束，因为后面还有最终朝向旋转。
   - 应在 `SpinToPose` 完成、Nav2 `Goal succeeded` 后继续 settle `0.5s - 2.0s`。

3. 下一个点位启动前 guard
   - 如果 APP/上层在到达后很快发下一个点位，状态管理器应先检查 localization settle 是否完成。
   - 未完成时延迟发下一个 Nav2 goal，而不是立即开始下一个点位。

也就是说，旋转保护不是简单按“到点”开始/结束，而是覆盖：`SpinToPose 开始 -> SpinToPose 完成 -> NDT 连续健康 settle`。

### 保护期间行为

对 NDT 结果分三类处理：

1. `fitness > score_threshold`
   - 不接受结果。
   - 重发 last good TF，stamp 使用当前时间。
   - 发布诊断 `rejected/rotation_guard_high_fitness`，不触发 recovery。

2. `fitness <= score_threshold` 且平移 correction 超过阈值
   - 不接受平移跳变。
   - 重发 last good TF。
   - 发布诊断 `confirming/rotation_guard_pose_jump_candidate` 或 `rejected/rotation_guard_hold`。
   - 不向状态管理器发布普通 `pose_jump`。

3. `fitness <= score_threshold` 且 correction 很小
   - 正常接受。
   - 刷新 last good TF。

### 超时与风险控制

保护窗口不能无限长。建议：

- `rotation_guard_max_duration_sec: 8.0`
- `rotation_guard_settle_sec: 2.0`
- `rotation_guard_max_odom_displacement: 0.25m`

如果超过最大时间或 odom/Fast-LIO 显示实际移动太多，则退出保护并允许 recovery。这样避免机器人在真实丢定位后一直冻结。

### 修改点

定位节点 `src/lidar_localization`：

- 增加参数：
  - `rotation_guard_enabled`
  - `rotation_guard_angular_threshold`
  - `rotation_guard_linear_threshold`
  - `rotation_guard_settle_sec`
  - `rotation_guard_max_duration_sec`
  - `rotation_guard_publish_status`
- 订阅 `/cmd_vel` 或新增 `/navigation/active_behavior` 状态输入。
- 在 pose jump 判断前加入 rotation guard 分支。
- `publishLastGoodTransformIfFresh()` 需要确保重发 TF 时使用当前 stamp，避免旧 stamp 再次污染 TF 查询。

状态管理器 `src/humanoid_navigation/humanoid_navigation/navigation_state_manager_fusion.py`：

- 将 `rotation_guard_*` 诊断原因从 recovery 触发集合中排除。
- 在 UI/事件层显示“旋转保护中”，但不报定位恢复。

### 复杂度

低到中等。主要是状态机和诊断原因改造，不需要改点云算法。预计 1 天内可完成初版，半天到 1 天 bag 回放验证。

### 验证标准

- 点位5最终转向期间不再触发 `navigation_auto_paused`。
- NDT 仍然运行，但旋转期间大跳变被记录为 `rotation_guard_*`。
- 旋转结束静稳 `1s - 2s` 后，若 NDT 正常，继续导航。
- 若静稳后仍连续跳变，才进入 recovery。

## 方案2：多帧 NDT / 局部 scan 匹配

### 原理

单帧 NDT 使用当前一帧点云作为 source cloud。多帧 NDT 将最近若干帧点云用 odom/Fast-LIO/IMU 的相对运动补偿到同一个局部坐标系，再合成一个更密、更完整的 source cloud，然后拿这个局部 scan 去匹配全局 PCD 地图。

直观理解：

- 单帧：只看这一瞬间的墙面和柱子。
- 多帧：看过去 `0.5s - 1.0s` 内扫过的一小段局部结构。

这样可以降低单帧视角变化、遮挡、局部重复结构造成的歧义。

### 坐标系注意事项

当前 `/fast_lio/cloud_registered` 和 `/odom` 来自 Fast-LIO 内部坐标系，坐标轴不是 ROS 标准导航坐标系。日志和代码里已经存在从 Fast-LIO/相机系到 ROS 标准系的固定旋转处理，地图点云也被转换成 ROS 标准 Z-up 后再用于 NDT。

因此多帧 NDT 不能直接把 `/fast_lio/cloud_registered` 当作 ROS 标准 source cloud 使用，必须明确坐标系转换：

- 所有参与 NDT 的 source cloud 和 target map 必须处于同一坐标约定。
- 如果 target map 已经在 `lidar_localization` 内转换为 ROS 标准系，则多帧累积后的 source cloud 也必须转换到同一 ROS 标准系。
- 用 `/odom` 做历史帧运动补偿时，也必须使用同一坐标约定下的相对位姿，不能把 Fast-LIO 非标准轴的相对位姿直接混到 ROS 标准点云里。

推荐实现方式：

1. 在 Fast-LIO 原生坐标系内完成历史帧相对补偿和累积。
2. 累积完成后，对整个局部 scan 统一应用与现有 NDT 输入一致的固定轴变换。
3. 再送入 NDT，与已转换到 ROS 标准系的 PCD map 匹配。

或者：

1. 每帧进入 buffer 前先转换到 ROS 标准系。
2. 同时把用于补偿的 odom pose 也转换到 ROS 标准系。
3. 全流程只使用 ROS 标准系。

两种方式都可以，但不能混用。为了降低风险，初版建议沿用现有 `lidar_localization` 对单帧输入/地图的坐标转换路径，只在该路径之后增加 buffer；这样最不容易引入轴向不一致。

### 对当前环境的适配性

对点位5这类“旋转/短距离移动时飘”的场景，多帧有价值：

- 原地旋转时，多帧可以形成更完整的环视局部 scan。
- 从点位5去下一个点位时，多帧包含进入/离开点位的连续结构，比单帧稳定。
- 如果走廊两侧几何高度重复，多帧能增加局部上下文，但不能完全消除全局对称性。

前提是运动补偿可靠。当前系统已有 Fast-LIO `/odom` 和 `/fast_lio/cloud_registered`，适合做短窗口补偿。

### 局部 scan 多大

这里的“局部 scan”指累积后的 source cloud，不是裁剪后的全局地图。建议初始参数：

- 时间窗口：`0.6s`
- 帧数上限：`6 - 8` 帧
- 输入距离：保留 `1m - 25m` 或先沿用当前 `scan_min_range/scan_max_range`
- 累积后 voxel：`0.15m - 0.25m`
- 点数上限：`20000 - 40000`
- 最大累积位移：`0.8m`
- 最大累积 yaw：`60deg - 120deg`

如果专门解决原地旋转，可以允许较大 yaw 窗口；如果用于正常行走，要限制窗口，避免累积点云拖影。

### 实现路线

定位节点内新增 scan buffer：

1. 每帧点云到达后，转换为 filtered cloud。
2. 记录该帧对应的 odom/Fast-LIO pose。
3. 保留最近 `N` 帧或最近 `T` 秒。
4. 匹配前，将历史帧通过相对位姿变换到当前帧坐标系或固定局部坐标系。
5. 合并并 voxel downsample。
6. 用合并后的 cloud 替代当前单帧 cloud 调用 `registration_->setInputSource()`。

建议保留参数开关：

- `multi_frame_matching_enabled`
- `multi_frame_window_sec`
- `multi_frame_max_frames`
- `multi_frame_voxel_leaf_size`
- `multi_frame_max_points`
- `multi_frame_max_odom_translation`
- `multi_frame_max_odom_yaw`
- `multi_frame_use_only_when_rotating`

初版建议 `multi_frame_use_only_when_rotating=true`，先只解决点位5旋转问题，降低正常行走副作用。

### 和旋转保护的关系

推荐先做方案1，再做方案2。方案2即使改善匹配，也不能替代保护机制，因为任何局部优化算法都可能偶发跳变。最终结构应是：

1. 多帧 scan 提高匹配输入质量。
2. rotation guard 阻止旋转期间单帧/多帧异常直接触发 recovery。
3. recovery 只在静稳后仍失败时启动。

### 复杂度

中等。涉及点云缓存、TF/odom 查询、运动补偿、内存与耗时控制。预计 2 到 4 天完成初版，另需 1 到 2 天 bag 回放与现场验证。

### 风险

- 运动补偿不准会把多帧点云拼糊，反而降低 fitness。
- 点数过多会增加 NDT 耗时。
- 窗口太长会引入动态物体或旧视角拖影。
- 如果地图本身与现场局部结构差异大，多帧也可能持续 high fitness。

### 验证标准

- 在 nav_drift_test6 bag 上，点位5旋转期间不再出现 `~2m` 级 pose jump。
- NDT 平均耗时仍满足实时性。
- 点位1-4正常段不引入额外抖动。
- 点位5旋转结束后能够重新接受稳定位姿。

## 推荐实施顺序

1. 修复旧 stamp/TF 查询问题，避免恢复失败后死锁。
2. 实施方案1旋转保护，只改状态机和 pose jump gate。
3. 用 nav_drift_test6 bag 回放验证点位5。
4. 若仍有旋转后 high fitness 或无法恢复，再实施方案2多帧 NDT。
5. 最后再做 `GICP_OMP` A/B 对照，不建议作为第一优先级。

## 最小可落地版本

第一阶段只做这些：

- 识别 `SpinToPose` 或 `/cmd_vel` 原地旋转。
- 旋转和 settle 期间把 `pose_jump` 改成 `rotation_guard_hold`。
- 重发 last good TF，使用当前 stamp。
- settle 后连续 3 帧正常才恢复接受；连续 3 帧异常才触发 recovery。

这个版本改动最小，能最快验证“点位5是不是被旋转触发的单帧 NDT 跳变”。

## 本次实施内容

已在 `src/lidar_localization` 落地方案1和方案2的第一版。

### 已实现：方案1旋转保护

定位节点新增参数：

- `rotation_guard_enabled`
- `rotation_guard_use_cmd_vel_fallback`
- `rotation_guard_navigation_status_topic`
- `rotation_guard_angular_threshold`
- `rotation_guard_linear_threshold`
- `rotation_guard_settle_sec`
- `rotation_guard_max_duration_sec`

实现方式：

- `lidar_localization` 订阅 `/navigation/status`，优先用状态管理器发布的 `detailed_state=TURNING` 进入旋转保护。
- `TURNING` 来自状态管理器对 Nav2 `/behavior_tree_log` 的解析：`SpinToPose` 为 `RUNNING` 时进入，`SpinToPose` 结束后退出。
- 退出 `TURNING` 后不立刻放开 NDT 跳变，而是进入 `rotation_guard_settle_sec` 的短 settle 窗口。
- `/cmd_vel` 速度阈值只保留为 fallback，默认 `rotation_guard_use_cmd_vel_fallback=false`，避免路径跟踪角速度、减速抖动、带线速度转弯误触发。
- 保护窗口超过 `rotation_guard_max_duration_sec` 后自动退出，避免真实丢定位时无限冻结。
- 在保护窗口内，如果 NDT 收敛且 fitness 合格但平移/航向 correction 超过 pose jump 阈值，不再发布普通 `pose_jump`，而是发布：
  - `confirming/rotation_guard_hold`
  - 或 settle 阶段的 `confirming/rotation_guard_settle`
- 保护期间重发 last good TF，且 stamp 使用当前时间。
- `/localization/ndt_status` 追加诊断字段：
  - `rotation_guard_active`
  - `rotation_guard_settle`
  - `rotation_guard_age_sec`
  - `rotation_guard_source`
  - `rotation_guard_hold_count`

这样状态管理器不会因为点位5最终旋转期间的一两帧低 fitness 大跳变直接进入 recovery。

### 已实现：方案2多帧匹配初版

定位节点新增参数：

- `multi_frame_matching_enabled`
- `multi_frame_use_only_when_rotating`
- `multi_frame_window_sec`
- `multi_frame_max_frames`
- `multi_frame_voxel_leaf_size`
- `multi_frame_max_points`

实现方式：

- 多帧 buffer 放在现有固定轴转换和距离过滤之后。
- 因此参与 buffer 的点云已经走过当前单帧 NDT 相同的坐标转换路径，和已转换到 ROS Z-up 的 map 保持同一坐标约定。
- 初版没有重新引入额外 `/odom` 手工轴变换，避免把 Fast-LIO 非标准轴和 ROS 标准轴混用。
- 默认配置为 `multi_frame_use_only_when_rotating=true`，只在旋转保护窗口内用多帧 source cloud，降低正常行走段副作用。
- buffer 保留最近 `multi_frame_window_sec` 或最多 `multi_frame_max_frames` 帧，合并后再按 `multi_frame_voxel_leaf_size` 降采样，并用 `multi_frame_max_points` 限制点数。
- `/localization/ndt_status` 追加诊断字段：
  - `multi_frame_source_frames`
  - `multi_frame_source_points`

这版多帧利用的是 `/fast_lio/cloud_registered` 已经在 Fast-LIO 世界系内配准好的特性：历史帧本身已经在同一局部世界坐标中，不再做二次运动补偿。后续如果改用原始 body scan，再需要加入基于 `/odom` 或 TF 的显式运动补偿。

### 已配置的实机入口

`src/humanoid_navigation2/launch/navigation2_fusion_sc_v2.launch.py` 已启用：

- `rotation_guard_enabled: True`
- `rotation_guard_use_cmd_vel_fallback: False`
- `rotation_guard_navigation_status_topic: /navigation/status`
- `rotation_guard_settle_sec: 1.2`
- `rotation_guard_max_duration_sec: 8.0`
- `multi_frame_matching_enabled: True`
- `multi_frame_use_only_when_rotating: True`
- `multi_frame_window_sec: 0.6`
- `multi_frame_max_frames: 8`
- `multi_frame_voxel_leaf_size: 0.20`
- `multi_frame_max_points: 40000`

`src/lidar_localization/param/localization.yaml` 中也补了同名参数，默认仍关闭，避免其他 launch 未明确选择时行为突变。

### 顺手修复

`/initialpose` 转换为内部 `map->odom` guess 后，发布到 `/pcl_pose` 前会把 header stamp 刷新为当前时间。这样可以降低恢复失败后下游持续拿旧 initialpose stamp 查询 TF、最终出现“extrapolation into the past”的概率。

### 已验证

已执行：

```bash
colcon build --packages-select lidar_localization_ros2 --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

结果：`lidar_localization_ros2` 编译通过。编译输出中仍有 PCL/CMake dev warning 和既有 IMU double-to-float warning，不影响本次改动。

### 2026-05-28 离线回放结果

新增批量回放脚本：

- `tools/replay_ndt_rotation_guard_batch.py`

回放方法：

- 从 bag 内较早的 `/pcl_pose` 初始化 NDT，而不是只截取点位5附近。
- 回放时只播放 `/fast_lio/cloud_registered`、`/tf` 和用于保护状态的辅助话题，不播放原始 `/localization/ndt_status`。
- `nav_drift_test6` 使用 `/behavior_tree_log` 合成 `detailed_state=TURNING`，和实机状态管理路径一致。
- `mapping-test5/7/8` 没有 `/behavior_tree_log`，离线评估中临时用 `/cmd_vel` 合成 TURNING；这只用于旧 bag 复盘，不改变生产配置，生产配置仍是 `rotation_guard_use_cmd_vel_fallback=false`。

回放摘要：

| bag | 回放窗口 | 状态来源 | 新逻辑结果 | 原始记录对照 |
| --- | --- | --- | --- | --- |
| `nav_drift_test6` | 1779966702.992..1779967086.562，383.6s | BT `SpinToPose` | accepted 1816，rejected 612，普通 `pose_jump` 0，最大 correction 0.467m | accepted 1957，rejected 1103，confirming 7，`pose_jump` 14，candidate 7，最大 correction 2.211m |
| `mapping-test5` | 1779947157.744..1779947312.744，155.0s | 离线 `/cmd_vel` | accepted 998/998，普通 `pose_jump` 0，最大 correction 0.484m | accepted 995，rejected 212，`pose_jump` 208，`high_fitness` 4，最大 correction 1.608m |
| `mapping-test8` | 1779956090.739..1779956240.739，150.0s | 离线 `/cmd_vel` | accepted 992/992，普通 `pose_jump` 0，最大 correction 0.490m | accepted 960，rejected 310，`pose_jump` 86，`high_fitness` 224，最大 correction 1.696m |
| `mapping-test7` | 1779954023.799..1779954183.799，160.0s | 离线 `/cmd_vel` | accepted 991/991，普通 `pose_jump` 0，最大 correction 0.028m | accepted 1600/1600，普通 `pose_jump` 0，最大 correction 0.121m |

结论：

- 从点位1附近提前回放 `nav_drift_test6` 后，点位5附近的普通 `pose_jump` 被消掉；原始记录中的 `pose_jump/candidate` 没再出现。
- `mapping-test5` 和 `mapping-test8` 的失败窗口也被覆盖，新逻辑没有再触发普通 `pose_jump`。
- `mapping-test7` 正常窗口中保护没有启动，多帧也没有启动，说明当前 `multi_frame_use_only_when_rotating=true` 没有改变正常直线行走段行为。
- `nav_drift_test6` 从早期开始仍有一段 `high_fitness`，这部分更像离线初始化/起始匹配质量问题；但它没有演变成点位5那种大 correction `pose_jump`。

### 2026-05-28 单帧/多帧对照

为了确认不是“保护把异常藏起来”，又补跑了相同窗口下的“旋转保护开启、但多帧关闭”对照：

| bag | 配置 | accepted | confirming | rejected | 普通 `pose_jump` | correction p95 / max | fitness p95 / max |
| --- | --- | ---: | ---: | ---: | ---: | --- | --- |
| `mapping-test5` | 保护 + 单帧 | 722 | 19 | 265 | 265 | 1.182m / 2.607m | 0.228 / 0.274 |
| `mapping-test5` | 保护 + 多帧 | 998 | 0 | 0 | 0 | 0.175m / 0.484m | 0.088 / 0.281 |
| `mapping-test8` | 保护 + 单帧 | 648 | 13 | 334 | 334 | 1.777m / 2.072m | 0.006 / 0.114 |
| `mapping-test8` | 保护 + 多帧 | 992 | 0 | 0 | 0 | 0.143m / 0.490m | 0.006 / 0.115 |

这个对照说明：

- 单靠旋转保护不够。保护期间可以 hold 一部分跳变，但单帧 NDT 在窗口外或保护结束后仍会继续产生普通 `pose_jump`。
- 多帧 NDT 明显降低了 correction 的长尾，`mapping-test5/8` 两个失败包都从 1.1m 以上 p95 correction 降到 0.18m 以内。
- `mapping-test8` 的 fitness 本来就不高，失败主要不是 fitness gate，而是单帧几何匹配解出现了大平移跳变；多帧后 correction 收敛很多。

### 导航完成度验证边界

当前离线回放已经验证的是“定位链路是否还会触发导航恢复条件”，不是完整闭环导航是否物理走完全程：

- 原始 `mapping-test5` 在 1779947256.894 发布 `/localization/recovery_requests`，reason=`pose_jump`，之后 `/localization/recovery_status` 持续等待静止和重定位。
- 原始 `mapping-test8` 在 1779956187.477 发布 `/localization/recovery_requests`，reason=`pose_jump`，之后进入 runtime recovery。
- 新回放只启动 NDT 和状态桥，没有启动完整 Nav2 action server / 控制器 / 状态管理器，所以不能直接声称“导航 action 成功到达终点”。
- 但新回放没有普通 `pose_jump`，也没有 rejected/confirming（多帧版本），因此不会再触发同一类 `localization_failure_navigation_context_recovery_request`。

要验证“能不能成功走完全程”，还需要补一轮完整导航栈回放或实机复测：

1. 启动完整 `navigation2_fusion_sc_v2.launch.py`，使用新 NDT 参数。
2. bag 只播放传感器和 TF 输入，不播放旧 `/cmd_vel`、旧 `/pcl_pose`、旧 `/localization/ndt_status`。
3. 按原始点位序列下发导航目标，记录 `/navigation/status`、Nav2 action result、`/cmd_vel`、`/localization/recovery_requests`、`/localization/ndt_status`。
4. 成功标准：
   - 所有点位 action result 为 succeeded；
   - 全程没有 `localization_failure_navigation_context_recovery_request`；
   - `/localization/ndt_status` 无普通 `pose_jump`；
   - 旋转保护只覆盖 `TURNING/settle` 窗口，正常直线段不持续 active；
   - 多帧只在保护窗口内启用，正常段 `multi_frame_source_frames<=1`。

注意：只用录好的 bag 不能证明真实闭环导航“走完全程”。原因是 Nav2 新输出的 `/cmd_vel` 不会改变 bag 内已经录好的点云、里程计和 TF 轨迹。bag 可以验证定位链路、恢复触发条件和保护退出稳定性；完整行走成功需要实机复测，或者一个能根据 `/cmd_vel` 重新生成传感器/里程计的闭环仿真。

### 2026-05-28 全程多帧对照

为了判断多帧是全程开启还是只在旋转保护期启用，又补跑了 `--multi-frame-always`：

| bag | 多帧策略 | accepted | confirming | rejected | 普通 `pose_jump` | correction p95 / max | fitness p95 / max |
| --- | --- | ---: | ---: | ---: | ---: | --- | --- |
| `mapping-test5` | 只旋转期 | 998 | 0 | 0 | 0 | 0.175m / 0.484m | 0.088 / 0.281 |
| `mapping-test5` | 全程开启 | 991 | 2 | 0 | 0 | 0.116m / 0.574m | 0.011 / 0.200 |
| `mapping-test8` | 只旋转期 | 992 | 0 | 0 | 0 | 0.143m / 0.490m | 0.006 / 0.115 |
| `mapping-test8` | 全程开启 | 933 | 7 | 0 | 0 | 0.088m / 0.720m | 0.010 / 0.209 |
| `mapping-test7` | 只旋转期 | 991 | 0 | 0 | 0 | 0.009m / 0.028m | 0.002 / 0.011 |
| `mapping-test7` | 全程开启 | 1033 | 0 | 0 | 0 | 0.009m / 0.013m | 0.004 / 0.018 |

判断：

- 全程多帧在正常窗口 `mapping-test7` 没有明显副作用。
- 但在失败包 `mapping-test5/8` 中，全程多帧没有比“只旋转期多帧”更稳，反而出现了少量 `pose_jump_candidate` / `rotation_guard_settle`，最大 correction 也更大。
- 当前更合理的默认策略仍是 `multi_frame_use_only_when_rotating=true`。
- 如果之前地图红圈区域单帧也会飘，建议不要直接改成全程多帧，而是增加第二类触发条件：
  - 地图风险区 geofence：进入红圈区域后开启多帧，离开后延迟关闭；
  - 或 NDT 质量退化触发：连续几帧 correction/fitness 接近阈值时开启多帧 2-3 秒；
  - 或导航状态触发：原地旋转、窄通道低速调整、局部重规划频繁时开启。

### 保护退出后 2 秒稳定性

对“保护退出后 1-2 秒内能不能重新找回位置”做了窗口统计：每次 `rotation_guard_active=true -> false` 后，检查后续 2 秒内是否有非 `accepted/ok`。

| bag / 策略 | 保护退出后 2 秒结果 |
| --- | --- |
| `nav_drift_test6` 只旋转期多帧 | 多个退出窗口均为 `accepted/ok`，无 bad frame |
| `mapping-test5` 只旋转期多帧 | 多个退出窗口均为 `accepted/ok`，无 bad frame |
| `mapping-test8` 只旋转期多帧 | 多个退出窗口均为 `accepted/ok`，无 bad frame |
| `mapping-test5` 全程多帧 | 有 1 个退出窗口出现 2 帧 `pose_jump_candidate` |
| `mapping-test8` 全程多帧 | 有 1 个退出窗口出现 2 帧 `pose_jump_candidate` |

因此，以当前参数看，“只旋转期多帧 + 1.2s settle”在这些 bag 上能在保护退出后的 2 秒内保持稳定，满足导航点之间 1-2 秒停顿的风险要求；全程多帧反而在退出附近更容易出现 candidate。
