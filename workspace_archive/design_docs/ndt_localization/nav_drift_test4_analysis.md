# nav_drift_test4 导航漂移与重定位失败分析

## 结论

这次问题不是点位 6 本身导航失败。日志显示点位 6 已经成功到达，真正的问题发生在点位 7 导航过程中。

核心链路是：

1. 点位 7 导航过程中，Nav2/BT 持续使用旧时间戳查询 TF，触发 `Lookup would require extrapolation into the past`。
2. Nav2 因 TF 查询失败 abort，状态管理器进入可恢复失败状态并请求重定位。
3. 重定位阶段 NDT 匹配 fitness 一开始并不差，但连续修正量约 0.9 到 1.7m，超过 `pose_jump` 门限 0.4m，被拒绝。
4. 后续 `/initialpose` 注入不稳定，其中一次 `map_ground` 位姿注入后 NDT fitness 飙到约 145，定位 TF 停止更新，系统无法恢复。

## 数据来源

- 日志：`/home/ubuntu/humanoid_ws/debug_output.txt`
- bag 元数据：`/home/ubuntu/nav_drift_test4/metadata.yaml`
- bag 文件：`/home/ubuntu/nav_drift_test4/nav_drift_test4_0.mcap`

bag 记录范围：

- 开始时间：`1779874357.688`
- 结束时间：`1779874829.947`
- 时长：`472.259s`
- 消息数：`163301`

## 关键时间线

| 时间戳 | 事件 |
| --- | --- |
| `1779874604.798` | 开始导航到点位 6 |
| `1779874615.963` | Nav2 确认到达点位 6 |
| `1779874617.983` | 开始导航到点位 7 |
| `1779874623.985` | 后续 TF 报错反复引用的旧请求时间戳 |
| `1779874632.026` | 状态管理器收到 `navigation_obstacle_blocked` |
| `1779874636.182` | 第一次明显的 `SpinToPose TF failure` |
| `1779874647.202` | 再次 `SpinToPose TF failure` |
| `1779874658.197` | BT navigator abort，Nav2 goal failed |
| `1779874658.199` | 状态管理器请求 HDL 按需重定位，目标为点位 7 |
| `1779874693.282` | NDT 收到 `/initialpose`：`map (12.338, -6.879, yaw=40deg)` |
| `1779874697.945` | 数据整合节点推送 `navigation_localization_recovered` |
| `1779874716.057` | 用户 stop_navigation 成功 |
| `1779874732.245` | NDT 收到 `/initialpose`：`map_ground (16.292, 17.950, yaw=0.6deg)` |
| `1779874732.259` 之后 | NDT fitness 持续高于 0.3，全部跳过 |

## 点位 6 与点位 7 证据

日志中点位 6 已经成功：

```text
1779874604.798 开始导航到路点: 点位6
1779874615.963 Nav2确认到达路点: 点位6
```

点位 7 才是失败点：

```text
1779874617.983 开始导航到路点: 点位7
1779874658.197 Goal failed
1779874658.199 导航失败后请求 HDL 按需重定位: ... 目标=点位7
```

## TF 失败现象

Nav2/BT 报错反复使用同一个旧时间戳：

```text
Requested time 1779874623.985943
earliest data is at time 1779874647.983390
```

后续 planner、controller、robot_realpose_publisher、navigation_state_manager 都出现同类问题。这说明某个导航行为或目标消息的 stamp 没有刷新，TF cache 前移后继续拿旧 stamp 查 transform，必然失败。

bag 中关键 TF span：

| TF | bag 相对时间 | TF stamp 范围 | 说明 |
| --- | --- | --- | --- |
| `map->odom` | `0.076..374.497s` | `1779874357.764..1779874732.185` | 定位 TF 在后半段停止 |
| `map->map_ground` | `0.021..384.619s` | `1779874357.706..1779874742.306` | 依赖 map 位姿，稍后也失效 |
| `odom->odom_ground` | `0.268..472.257s` | `1779874357.810..1779874829.944` | 里程计地面 TF 仍在更新 |
| `camera_init->body` | `0.007..472.231s` | `1779874357.581..1779874829.785` | fast_lio/body TF 仍在更新 |

这说明传感器/里程计链路还在跑，主要断点是全局定位 `map->odom`。

## NDT 状态统计

从 `/localization/ndt_status` 统计：

| 状态 | 数量 | 首次 | 末次 | fitness 范围 | 最大修正量 |
| --- | ---: | ---: | ---: | --- | ---: |
| `accepted / ok` | 2998 | `0.076s` | `374.497s` | `0.0008 / 0.0014 / 0.0477` | `1.196m` |
| `rejected / pose_jump` | 609 | `222.529s` | `345.344s` | `0.0021 / 0.0040 / 0.0409` | `1.707m` |
| `confirming / pose_jump_candidate` | 27 | `193.474s` | `268.696s` | `0.0014 / 0.0030 / 0.0074` | `0.714m` |
| `accepted / confirmed_pose_jump` | 18 | `222.922s` | `264.373s` | `0.0021 / 0.0033 / 0.0067` | `0.799m` |
| `rejected / high_fitness` | 977 | `374.572s` | `472.142s` | `143.8020 / 145.2405 / 146.7472` | `0.000m` |

重点：

- 导航失败前，NDT fitness 很低，说明扫描匹配质量本身并不差。
- 但修正量经常超过 `pose_jump` 限制，被拒绝。
- 最后一次 `/initialpose` 后，fitness 直接变成约 145，已经不是小幅漂移，而是明显错误先验或坐标系/位置不一致。

## 重定位失败原因

失败后系统做了两类恢复：

1. ScanContext recovery
2. HDL runtime recovery

日志显示 ScanContext conservative trigger 先失败：

```text
trigger #1 FAILED: Scan Context candidates rejected by odom consistency gate
```

HDL recovery 收到的先验是：

```text
frame=map pose=(15.422, 18.353, yaw=-0.011), preferred_search_radius=10.0m
```

但之后 NDT 持续：

```text
Rejecting NDT pose jump: translation≈0.9~1.7 limit=0.400
```

也就是说：恢复候选可能接近正确，但当前 NDT jump gate 太严，导致恢复期间不能接受大于 0.4m 的纠偏。

随后 `1779874732.245` 又收到一个 `map_ground` frame 的 `/initialpose`：

```text
initialpose input: frame=map_ground x=16.292 y=17.950 yaw=0.6deg
Transformed initialpose from map_ground to map: x=16.292 y=17.950 z=-1.221 yaw=0.6deg
```

这个位姿注入后：

```text
The fitness score is over 0.300000, skip this result.
```

bag 中对应 `high_fitness` 的 fitness 约 145，说明该初始位姿与当前点云/地图严重不匹配。此后 `map->odom` 停止更新，重定位无法完成。

## 直接根因判断

### 1. Nav2 失败的直接原因

点位 7 过程中，BT/SpinToPose 或相关自定义节点使用了过期 PoseStamped stamp。长时间运行后 TF buffer 已经没有该时间点的数据，于是 Nav2 查询 TF 失败并 abort。

这个问题表现为所有报错都引用旧的 requested time：

```text
Requested time 1779874623.985943
```

### 2. 定位无法恢复的直接原因

恢复阶段 NDT 有两个门槛问题：

- 前期 fitness 很低但 correction translation 超过 0.4m，被 `pose_jump` 拒绝。
- 后期错误或未验证的 `/initialpose` 注入导致 fitness 约 145，NDT 全部拒绝。

### 3. 系统设计上的放大因素

`/initialpose` 会触发 HDL standby 和 manual override 抑制：

```text
manual initial pose received ... auto HDL recovery suppressed for 300.0s
HDL localization entered standby
```

如果这个 initialpose 没有经过验证，反而会关闭一条可恢复路径。

## 建议修改

1. 修复 BT/SpinToPose 的 TF 查询时间戳

长时间行为里不要一直使用启动时的 goal stamp。查询当前机器人位姿时应使用最新时间，或者在内部刷新 `PoseStamped.header.stamp`。

2. 恢复期间放宽 NDT pose jump gate

在 `/initialpose`、HDL recovery、ScanContext recovery 后，允许若干帧更大的 translation jump，例如 1.5 到 2.0m，并用连续 accepted 帧收敛后再恢复 0.4m 正常门限。

3. `/initialpose` 必须做坐标系和质量校验

建议只接受 `map` frame 的重定位结果，或统一转换后再验证。`map_ground` 输入要特别谨慎，避免 z/ground frame 转换带来错误先验。

4. manual override 不应直接抑制自动恢复 300s

只有 NDT 或 HDL 验证通过后，才应该进入 HDL standby 或 suppress 自动恢复。未验证 initialpose 应该保持 recovery active。

5. Nav2 failure 后清理 stale goal 和 costmap filter 消息

Nav2 abort 后应取消旧 goal、清理 costmap/behavior 的旧 stamp 数据，避免旧 requested time 继续污染后续恢复流程。

## 可复查命令

```bash
sed -n '1048,1135p' debug_output.txt
sed -n '2240,2260p' debug_output.txt
sed -n '2850,3035p' debug_output.txt
```

如需复查 bag，重点 topic：

```text
/localization/ndt_status
/localization/recovery_status
/localization/recovery_requests
/initialpose
/robot_realpose
/pcl_pose
/tf
/cmd_vel
```
