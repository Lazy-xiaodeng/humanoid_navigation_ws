# NDT/Fusion/HDL 重定位改造方案（test3 复盘后）

日期：2026-05-27  
背景 bag：`/home/ubuntu/nav_drift_test3/nav_drift_test3_0.mcap`  
相关日志：
- `/home/ubuntu/humanoid_ws/debug_logs/ndt_fusion_monitor_20260526_221359.jsonl`
- `/home/ubuntu/humanoid_ws/debug_logs/start_navigation_20260526_221145.log`

## 1. 结论

这次 test3 不是单纯 NDT 漂移，也不是单纯 HDL recovery 不稳定，而是三类逻辑叠加：

1. `map->odom` 存在多发布者竞争。
   - NDT/lidar_localization 发布或重发 `map->odom`。
   - `localization_odom_fusion` 在 DEGRADED 时也发布冻结的 `map->odom`。
   - bag 中同一条 `map->odom` 在固定冻结值和 NDT 漂移值之间高速切换，下游 TF 查询会随机读到当前最新一帧，导致机器人在地图中来回跳。

2. fusion/rusion 接管后继续让导航走得太久。
   - fusion 的价值应该是短时抗抖、保证 TF 连续、安全停车。
   - 这次 DEGRADED 后仍有非零 `/cmd_vel`，机器人继续运动，导致 NDT 在错误先验附近继续匹配，漂移被放大。

3. HDL recovery 需要静止，但系统没有真正静止。
   - recovery 状态一直报 `robot still moving before HDL relocalize`。
   - buffer 清理、全局搜索、initialpose 发布流程被运动状态和错误 TF 污染。

最终表现是：NDT 漂移 -> fusion 冻结 -> NDT 和 fusion 抢 TF -> 导航继续走 -> recovery 等不到可靠静止 -> NDT 继续发散 -> costmap/robot pose 跳到地图外。

## 2. 设计目标

目标不是回退到“只用 NDT + HDL”，而是形成三层定位保护：

1. NDT 正常时：使用 NDT。
2. NDT 短暂异常时：fusion 短时接管，保持 TF 连续，允许低速短距离过渡。
3. NDT 明确失稳时：停止导航，冻结 TF，静止后执行 HDL/SC recovery，恢复稳定后继续当前点。

核心原则：

- 全系统只能有一个节点发布 `map->odom`。
- fusion/rusion 可以接管，但必须有时间和距离边界。
- recovery 只能在机器人确实静止后执行。
- 恢复导航必须经过连续稳定验收，不能收到一帧 `localization_recovered` 就继续跑。

## 3. 推荐架构

```text
Fast-LIO
  publishes: odom -> camera_init/body/base

lidar_localization / NDT
  subscribes: filtered pointcloud, odom
  publishes: /pcl_pose
             /localization/ndt_status
  does NOT publish: map -> odom

localization_odom_fusion
  subscribes: /pcl_pose
             /localization/ndt_status
             /localization/recovery_status
             /navigation_status
  publishes: map -> odom  (唯一发布者)
             /localization/fusion_status
             /localization/recovery_requests

navigation_state_manager
  subscribes: /localization/fusion_status
             /localization/recovery_status
  owns: pause/cancel/resume/zero-cmd policy

hdl_bootstrap_to_initialpose / SC recovery
  runs only after navigation is paused and robot is stationary
  publishes: /initialpose
             /localization/recovery_status
```

## 4. 状态机

### 4.1 Fusion 状态

```text
INITIALIZING
  -> HEALTHY
     条件：NDT 已有稳定 /pcl_pose

HEALTHY
  -> BRIDGING
     条件：短暂 NDT 异常，但还未超过硬边界

BRIDGING
  -> HEALTHY
     条件：NDT 连续稳定恢复
  -> LOCALIZATION_RECOVERY
     条件：超过接管时间/距离，或 NDT 明确失稳

LOCALIZATION_RECOVERY
  -> RECOVERY_VALIDATING
     条件：HDL/SC 发布 initialpose，NDT 重新输出

RECOVERY_VALIDATING
  -> HEALTHY
     条件：NDT、TF、costmap 连续稳定
  -> LOCALIZATION_RECOVERY
     条件：恢复验收失败，继续等待或重试 recovery
```

### 4.2 状态含义

`HEALTHY`：
- fusion 使用 `/pcl_pose` 更新并发布 `map->odom`。
- Nav2 正常执行。

`BRIDGING`：
- fusion 冻结进入 BRIDGING 前最后可信 `map->odom`。
- 允许 odom 驱动机器人在地图中连续运动。
- 限速并设置硬边界。
- NDT 仍可继续计算 `/pcl_pose`，但不能直接影响 TF。

`LOCALIZATION_RECOVERY`：
- cancel 当前 Nav2 goal。
- 持续发布零速度。
- fusion 继续发布冻结 `map->odom`，保持 TF 树不断。
- 等待 stationary 后触发 HDL/SC recovery。

`RECOVERY_VALIDATING`：
- 收到 initialpose 或 recovery candidate 后，不立即恢复导航。
- 观察 NDT 连续帧、TF 跳变、costmap bounds。
- 验收通过才恢复当前未完成路点。

## 5. 关键改造点

### 5.1 单 TF 发布源

必须保证运行时只有 `localization_odom_fusion` 发布 `map->odom`。

建议改动：

1. `lidar_localization` 增加参数：

```yaml
publish_tf: false
republish_last_good_tf_on_failure: false
```

2. `lidar_localization` 保留：

```text
/pcl_pose
/localization/ndt_status
```

3. fusion 在所有状态下统一发布 `map->odom`：
- HEALTHY：转发 NDT `/pcl_pose`。
- BRIDGING/LOCALIZATION_RECOVERY：发布冻结值。
- RECOVERY_VALIDATING：根据验收策略决定是否平滑切换。

相关代码位置：
- `src/lidar_localization/src/lidar_localization_component.cpp`
- `src/lidar_localization/param/localization.yaml`
- `src/humanoid_navigation2/humanoid_navigation2/localization_odom_fusion.py`

### 5.2 BRIDGING 接管边界

fusion 接管不能无限持续。建议初始参数：

```yaml
bridging_enabled: true
bridging_max_duration_sec: 2.0
bridging_max_distance_m: 0.8
bridging_max_yaw_rad: 0.35
bridging_cmd_vel_scale: 0.4
bridging_max_linear_speed: 0.15
bridging_max_angular_speed: 0.25
```

触发 BRIDGING 的条件：

```text
NDT status == rejected
或 reason in ["pose_jump", "pose_jump_candidate", "high_fitness"]
或 correction_translation > 0.4m
或 /pcl_pose 帧间跳变 > 0.5m
或 /pcl_pose stale > 0.5s
```

BRIDGING 转 RECOVERY 的条件：

```text
持续时间 > bridging_max_duration_sec
或 odom 位移 > bridging_max_distance_m
或 yaw 累计变化 > bridging_max_yaw_rad
或 NDT 连续 rejected > 1.0s
或 NDT fitness > 0.5
或 costmap 报 robot/sensor out of bounds
```

BRIDGING 转 HEALTHY 的条件：

```text
NDT 连续稳定 10 帧以上
fitness < 0.05
correction_translation < 0.2m
/pcl_pose 与冻结 TF 推算的当前位姿差 < 0.3m
```

### 5.3 明确失稳后强制停车

一旦进入 `LOCALIZATION_RECOVERY`：

1. navigation_state_manager 立即 cancel 当前 Nav2 goal。
2. 发布零 `/cmd_vel`，持续 hold。
3. 禁止手动 resume。
4. 禁止自动切下一个 waypoint。
5. 保留当前 waypoint，上一次 waypoint、当前路径段、当前冻结 pose，供 recovery 做先验。

相关代码位置：
- `src/humanoid_navigation/humanoid_navigation/navigation_state_manager_recoverable.py`

当前代码里已经有 `handle_localization_recovery_started()`、`begin_localization_stop_hold()`、`enforce_localization_stop()` 这类能力，但需要确保触发源从 fusion 的明确失稳状态进入，而不是等 HDL recovery 自己开始后才暂停。

### 5.4 Recovery 触发时机

HDL/SC recovery 不应该在机器人还在走时运行。

建议条件：

```text
current_state == LOCALIZATION_RECOVERY
cmd_vel 连续为 0
odom/body 在 1.0s 内 xy_delta < 0.03m
yaw_delta < 0.03rad
点云 buffer 已清理并重新采集 stationary scans
```

流程：

```text
1. 进入 LOCALIZATION_RECOVERY
2. cancel goal + zero cmd hold
3. 等静止 1.0s
4. clear HDL relocalize buffer
5. 等新 stationary scans 填充 1.0-1.5s
6. 执行局部先验搜索
7. 局部失败后再执行全局搜索
8. 候选多帧一致后发布 /initialpose
9. 进入 RECOVERY_VALIDATING
```

### 5.5 Recovery 搜索策略

不要只做纯全局搜索。建议分层：

第一层：路径上下文先验
- 当前 waypoint 附近。
- 上一个 waypoint 附近。
- 当前路径段投影附近。
- 冻结 pose 附近。

第二层：扩大局部搜索
- 半径 2m、5m、10m 逐步扩大。
- 每层要求候选多帧一致。

第三层：全局搜索
- 只有局部失败时才执行。
- 如果候选 fitness 接近、margin 太小，认为歧义，不发布 initialpose。

候选验收：

```text
candidate_fitness < 0.07
top1-top2 margin > 0.03
连续 3 次候选 xy 差 < 0.8m
连续 3 次 yaw 差 < 0.35rad
candidate 在地图 bounds 内
candidate 与导航上下文不明显冲突
```

### 5.6 恢复后验收

收到 `/initialpose` 或 `localization_recovered` 后，不立刻 resume。

建议进入 `RECOVERY_VALIDATING`，持续 1-2 秒：

```text
NDT accepted 连续 >= 15 帧
fitness < 0.05
correction_translation < 0.25m
correction_yaw < 0.15rad
/pcl_pose 帧间跳变 < 0.1m
map->odom 由 fusion 单源发布，且无跳变
robot pose 在地图 bounds 内
global/local costmap 无 out of bounds
```

验收通过：
- fusion 平滑切回 HEALTHY。
- navigation_state_manager 等待 costmap 稳定 1-2 秒。
- resume 当前未完成 waypoint。

验收失败：
- 不 resume。
- 保持冻结 TF。
- 清 buffer 后重新 recovery，或进入人工重定位等待。

## 6. 参数建议

第一版保守参数：

```yaml
localization_odom_fusion:
  bridging_enabled: true
  bridging_max_duration_sec: 2.0
  bridging_max_distance_m: 0.8
  bridging_max_yaw_rad: 0.35
  degraded_correction_threshold_m: 0.4
  degraded_pcl_jump_threshold_m: 0.5
  degraded_rejected_duration_sec: 1.0
  recovery_required_stationary_sec: 1.0
  recovery_stationary_xy_m: 0.03
  recovery_stationary_yaw_rad: 0.03
  validation_duration_sec: 2.0
  validation_required_ndt_frames: 15
  validation_max_fitness: 0.05
  validation_max_correction_m: 0.25
  validation_max_pcl_jump_m: 0.1

lidar_localization:
  publish_tf: false
  republish_last_good_tf_on_failure: false
```

如果现场希望尽量不中断导航，可以稍微放宽：

```yaml
bridging_max_duration_sec: 3.0
bridging_max_distance_m: 1.2
```

但不建议超过这个范围。长时间 odom 接管会把问题从“定位失败”变成“错误定位下继续导航”，风险更高。

## 7. 分阶段实施计划

### P0：止血，解决 TF 抢占

目标：消除 `map->odom` 多发布者。

改动：

1. `lidar_localization` 增加 `publish_tf` 参数。
2. fusion 模式 launch 中设置 `publish_tf: false`。
3. 禁用 NDT 的 `republish_last_good_tf_on_failure`。
4. fusion 成为唯一 `map->odom` 发布者。

验收：

```bash
ros2 topic echo /tf
```

确认 `map->odom` 不再在固定值和 NDT 值之间交替跳。

bag 验证指标：
- `map->odom` 发布频率稳定。
- 相邻帧跳变 < 0.1m，除非处于受控 recovery snap。
- `/robot_realpose` 不再出现一帧 NDT、一帧冻结位姿的交替。

### P1：短时接管边界

目标：保留 fusion 抗抖能力，但防止无限接管。

改动：

1. 增加 BRIDGING 状态。
2. 添加时间/距离/yaw 边界。
3. BRIDGING 超界后发布 recovery request。
4. 向 navigation_state_manager 发布明确的 localization recovery start。

验收：
- NDT 短暂抖动 < 2s 时，导航不中断。
- NDT 持续失稳时，2s 或 0.8m 内强制停车。

### P2：导航暂停前置

目标：不要等 HDL recovery 已开始后才停车。

改动：

1. navigation_state_manager 订阅 fusion 状态。
2. fusion 进入 `LOCALIZATION_RECOVERY` 即 pause/cancel/zero cmd。
3. 禁止 recovery 期间手动 resume。
4. recovery 期间持续 zero cmd hold。

验收：
- recovery 日志中不再长时间出现 `robot still moving before HDL relocalize`。
- `/cmd_vel` 在 recovery 期间保持 0。

### P3：Recovery 分层搜索和验收

目标：降低“几分钟无法重定位”和“错误重定位”的概率。

改动：

1. HDL/SC 先用导航上下文局部先验。
2. 局部失败再全局搜索。
3. 全局候选需要多帧一致和 margin 检查。
4. initialpose 后进入 RECOVERY_VALIDATING。
5. 验收通过后才 resume。

验收：
- 长廊歧义场景不会单帧错误恢复。
- 正常场景恢复时间控制在 5-15 秒内。
- 恢复失败时系统停车等待，不继续错误导航。

## 8. 测试方案

### 8.1 离线 bag 回放

使用 test3 bag 验证：

```bash
ros2 bag play /home/ubuntu/nav_drift_test3/nav_drift_test3_0.mcap --clock
```

关注：
- `/tf`
- `/pcl_pose`
- `/robot_realpose`
- `/localization/ndt_status`
- `/localization/fusion_status`
- `/localization/recovery_status`
- `/cmd_vel`

验收脚本应检查：

```text
1. map->odom 是否单源连续
2. map->odom 相邻跳变最大值
3. robot_realpose 相邻跳变最大值
4. DEGRADED/BRIDGING 持续时间
5. BRIDGING 位移
6. RECOVERY 期间 cmd_vel 是否为 0
7. recovery 是否等到 stationary 后才 relocalize
```

### 8.2 现场测试

建议按风险递增：

1. 原地启动，确认 TF 单源。
2. 短路径 1-3 点，确认正常导航。
3. 人为遮挡/弱特征触发短暂 NDT 异常，确认 BRIDGING 不抖。
4. 长廊点位 8-9，确认超过边界后停车，不继续飞。
5. recovery 成功后 resume 当前点，不跳下一个点。

## 9. 风险和取舍

### 为什么不能继续无限 rusion 接管？

因为 odom 接管只能保证短期连续，不能保证全局正确。长时间继续导航会导致：

- 机器人实际越走越偏。
- NDT 在错误先验附近越匹配越差。
- recovery 等不到静止。
- costmap 使用错误 pose，障碍物和边界全部失真。

### 为什么不能完全回到旧的 NDT+HDL？

旧方案的问题是 NDT 一抖就停车，完全依赖 HDL 全局恢复，容易因为点云歧义、运动 buffer、长廊重复结构导致恢复慢或失败。

新方案保留 fusion 的短时抗抖能力：

- 短异常不打断导航。
- 长异常停车恢复。
- recovery 前强制静止并清 buffer。
- 恢复后连续验收再 resume。

### 为什么单 TF 发布源是硬要求？

TF 没有“谁更可信”的语义。下游只会读当前最新 TF。如果两个节点同时发 `map->odom`，就会出现：

```text
t0: fusion 发布冻结位姿
t1: NDT 发布漂移位姿
t2: fusion 又发布冻结位姿
t3: NDT 又发布漂移位姿
```

Nav2、costmap、状态管理器读到的位姿会来回跳。这个问题无法靠调阈值解决，只能靠架构上保证单发布源。

## 10. 最终推荐

采用“短时 fusion bridge + 超界停车 recovery + 单 TF 发布源”的方案：

```text
NDT 负责感知定位结果，不拥有 TF。
fusion 负责唯一 TF 和短时连续性。
navigation manager 负责停车和恢复导航。
HDL/SC 负责静止后的重定位。
```

这不是回退到旧方案，而是在旧方案前面增加一个受控缓冲层，并修掉 test3 中暴露的 TF 抢占和边走边恢复问题。

