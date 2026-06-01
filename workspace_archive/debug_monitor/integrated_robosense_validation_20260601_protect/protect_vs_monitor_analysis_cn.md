# RoboSense 集成定位 protect/monitor 回放分析

分析时间：2026-06-01  
工作区：`/home/ubuntu/humanoid_ws`  

## 1. 总体结论

本次没有改正式导航 launch 的大跳保护策略。正式文件 `navigation2_robosense_lidar.launch.py` 仍保持：

```text
jump_protection_mode = monitor
enable_spin_to_pose_guard = true
```

本次只在 bag isolated 验证链路中把 bridge 改成 `protect`，重放 `nav_drift_test23/24/25`。

结果：

- `protect` 对 test25 的 `map->odom` 大跳抑制非常明显，最大 xy 从 `1.73m` 降到 `0.19m`。
- test23 有改善但不彻底，最大 xy 从 `1.23m` 降到 `1.08m`，仍存在 >1m 级修正。
- test24 不建议直接按当前 protect 参数上实机：虽然部分跳变被 hold，但出现大量 `HOLD/DEGRADED`，说明定位候选长期不满足自动恢复条件，状态管理器实机上很可能需要介入暂停/等待/人工处理。
- 90 度级 ro prior yaw 变化仍主要发生在 SpinToPose/settle 阶段；旋转保护能冻结 TF 更新，但 protect 还需要解决“旋转结束后重新接入时 xy 回正过大”的问题。

## 2. PCD 地图降采样确认

当前相关 PCD：

| 文件 | 点数 | 大小 | 作用 |
|---|---:|---:|---|
| `src/humanoid_navigation2/pcd/hall.pcd` | 3,706,195 | 125MB | Fast-LIO 原始三维地图 |
| `src/humanoid_navigation2/pcd/hall_open3d_grounded.pcd` | 2,250,994 | 35MB | 当前 Open3D 与 RoboSense 共用定位地图 |
| `src/humanoid_navigation2/pcd/hall_localization_grounded.pcd` | 288,500 | 4.5MB | 旧 HDL/NDT 地图 |

`hall.pcd` 来源是 Fast-LIO 保存的 ikd-tree 地图。当前 `robosenseAiry.yaml` 没显式配置 `filter_size_map`，源码默认值是 `0.5`，因此 Fast-LIO 建图内部 map 更新按默认 `filter_size_map=0.5m` 做过 ikd-tree 层面的点管理/降采样，但保存出的 `hall.pcd` 不是后处理 voxel 地图。

`hall_open3d_grounded.pcd` 的生成记录来自 `debug_monitor/prior_map_open3d_nav_drift_test14/report.md`：

```text
hall.pcd 轴系转换、去地面以下点、0.05m voxel 后生成。
原始 hall.pcd 约 125MB / 3,706,195 点；
新定位地图约 35MB / 2,250,994 点。
```

两个定位节点使用同一张 `hall_open3d_grounded.pcd`，但运行时内部还会再次降采样：

| 节点 | 输入定位地图 | 运行时地图降采样 |
|---|---|---|
| Open3D `open3d_loc` | `hall_open3d_grounded.pcd` | `voxelsize_coarse=0.01m`，`voxelsize_fine=0.20m`，target 最多 `250000` 点 |
| RoboSense `lidar_localization` | `hall_open3d_grounded.pcd` | 加载后固定 `VoxelGrid leaf=0.30m` 建 KD-tree |

RoboSense 实时 scan 侧还有：

```text
semantic_cloud voxel leaf = 0.20m
lidar_matcher leaf_size = 0.10m
```

所以两者虽然共用同一张 PCD 文件，但实际匹配用的地图分辨率并不相同。

## 3. 本次数据来源和限制

本次 protect 回放目录：

```text
debug_monitor/integrated_robosense_validation_20260601_protect
```

上一次 monitor 回放目录：

```text
debug_monitor/integrated_robosense_validation_20260601_monitor
```

Open3D 基线目录：

```text
debug_monitor/robosense_open3d_validation_20260531_full
```

限制：

- 这是 ro + bridge 的 isolated 在线回放，不是完整 Nav2 闭环重跑。
- 没有播放旧 `/tf`，`map->odom` 由实验 bridge 重新发布。
- test25 protect 中 bridge 长时间处于 `SPIN_GUARD`，prior 采样不足，因此“跳变变小”不能单独解释为 ro 定位质量变好。
- 如果要判断真实机器人是否会停、是否能继续完成导航，还需要完整 Nav2 闭环或 shadow 分析结合 `/cmd_vel`、action 状态。

## 4. monitor/protect/Open3D 对比

### test23

| 方法 | map->odom 最大 xy | map->odom 最大 yaw | >=0.25m 次数 | >=0.50m 次数 | 状态摘要 |
|---|---:|---:|---:|---:|---|
| ro monitor | 1.232m | 4.13deg | 25 | 16 | ACCEPTED 780, REJECTED 151, PENDING 6, SPIN_GUARD 4 |
| ro protect | 1.081m | 3.75deg | 22 | 14 | ACCEPTED 665, REJECTED 142, HOLD 21, DEGRADED 105 |
| op baseline | 1.093m | 2.87deg | 22 | 19 | ACCEPTED 785, REJECTED 133, SPIN_GUARD 17 |

判断：protect 略有改善，但 test23 仍有 >1m 级 map->odom 修正。当前参数不能认为已经彻底解决。

### test24

| 方法 | map->odom 最大 xy | map->odom 最大 yaw | >=0.25m 次数 | >=0.50m 次数 | 状态摘要 |
|---|---:|---:|---:|---:|---|
| ro monitor | 1.707m | 3.95deg | 22 | 10 | ACCEPTED 671, SPIN_GUARD 66, REJECTED 101 |
| ro protect | 1.237m | 3.11deg | 6 | 103 | DEGRADED 212, ACCEPTED 418, REJECTED 160, HOLD 33 |
| op baseline | 0.415m | 7.33deg | 3 | 0 | ACCEPTED 109, SPIN_GUARD 714 |

判断：test24 是最不理想的包。protect 会把 ro 候选长期判为 hard reject/large jump hold，并触发大量 DEGRADED。虽然部分接受次数下降，但系统会长时间处于退化状态；实机上这更像“需要状态管理器停车等待或人工恢复”，不是可以直接放行的健康定位。

### test25

| 方法 | map->odom 最大 xy | map->odom 最大 yaw | >=0.25m 次数 | >=0.50m 次数 | 状态摘要 |
|---|---:|---:|---:|---:|---|
| ro monitor | 1.730m | 8.45deg | 32 | 20 | ACCEPTED 812, REJECTED 169, SPIN_GUARD 15 |
| ro protect | 0.191m | 0.37deg | 0 | 0 | ACCEPTED 175, SPIN_GUARD 837 |
| op baseline | 2.381m | 6.40deg | 29 | 20 | ACCEPTED 804, REJECTED 152, PENDING 52 |

判断：test25 中 protect 对 TF 跳变抑制非常明显。但状态中 `SPIN_GUARD` 占比过高，需要进一步确认是否因为 bag 的 `/navigation/status` 让 guard 维持太久。如果实机也这样，导航可能变成“长时间只靠 last good TF + odom”，短期稳定，长期会累积 odom 漂移风险。

## 5. SpinToPose 与 90 度 yaw 跳变

上一轮 monitor 分析已经定位到 90 度级 ro prior yaw 跳变：

| bag | ro prior 最大 yaw 跳变 | 阶段 | 最近点位 |
|---|---:|---|---|
| test23 | 94.1deg | SpinToPose settle | 点位18 附近，距约 0.84m |
| test24 | 77.3deg | SpinToPose settle | 点位16 附近，距约 0.55m |
| test25 | 92.6deg | SpinToPose settle | 点位17 附近，距约 0.69m |

这说明 ro 原始 prior yaw 的大变化主要是到点附近原地旋转导致的 `map->base_footprint` 朝向快速变化。bridge 的旋转保护在这些窗口内确实会冻结外部定位更新，不会直接把 90 度写进 `map->odom`。

真正需要关注的是旋转结束后的重新接入：如果 ro 候选位置也发生较大 xy 修正，monitor 会继续接受；protect 会 hold 或 degraded。

## 6. 是否变好

按“Nav2 看到的 `map->odom` 是否少跳”这个指标：

- test25：明显变好。
- test23：轻微变好，但仍不合格。
- test24：不算变好，只是从“接受大修正”变成“长时间退化/冻结”。

按“能否直接上实机”这个标准：

- 不建议直接把正式 launch 从 monitor 改为 protect。
- 建议继续保持正式 monitor，先把 protect 作为 shadow/isolated 验证。
- 下一步应增加“旋转结束后恢复窗口”的更细策略，而不是简单全局 protect。

## 7. 建议下一版保护策略

建议把保护拆成三个阶段：

1. 正常导航中：
   - `<=0.25m` 小修正接受。
   - `0.25m~0.50m` 连续稳定 5 帧接受。
   - `>0.50m` hold。

2. SpinToPose/settle 中：
   - 完全冻结外部定位更新。
   - 只重发 last good `map->odom`。

3. SpinToPose 刚结束后的恢复窗口，例如 5 秒：
   - 比普通空闲更严格。
   - xy 修正建议先限制到 `<=0.30m`。
   - yaw 修正建议 `<=0.10rad`。
   - 超过后不立刻 DEGRADED，先继续 hold 观察连续稳定帧。

test24 的大量 DEGRADED 说明当前 `hard_reject_translation=1.00m` 与恢复逻辑还不够细，需要区分“候选本身不可信”和“机器人刚完成旋转，odom/点云还在快速稳定”的情况。

## 8. 产物

protect 报告：

```text
debug_monitor/integrated_robosense_validation_20260601_protect/integrated_ro_vs_open3d_report.md
```

protect 轨迹图：

```text
debug_monitor/integrated_robosense_validation_20260601_protect/plots
```

monitor 报告：

```text
debug_monitor/integrated_robosense_validation_20260601_monitor/integrated_ro_vs_open3d_report.md
```
