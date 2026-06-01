# nav_drift_test28 bag 导航状态分析

分析对象：

- bag: `/home/ubuntu/nav_drift_test/nav_drift_test28/nav_drift_test28_0.mcap`
- metadata: `/home/ubuntu/nav_drift_test/nav_drift_test28/metadata.yaml`
- debug log: `/home/ubuntu/humanoid_ws/debug_output.txt`
- 录制时间：2026-06-01 15:56:28 - 16:08:11 CST
- bag 时长：725.75s，消息数：301807，大小约 24GB

## 1. 数据完整性

关键话题存在：

- `/tf`: 79296
- `/tf_static`: 4
- `/rosout`: 1996
- `/navigation/status`: 19361
- `/localization/prior_map_odom_bridge_status`: 7213
- `/odom`: 7200
- `/robot_realpose`: 7214
- `/robot_speed`: 未录制
- `/cmd_vel`: 1271
- `/behavior_tree_log`: 129
- `/plan`: 52
- `/app/navigation_command`: 17
- `/navigation/acknowledgments`: 31

本包缺少分析文档中要求的若干 prior/action 话题：

- `/prior_localization/odom`
- `/prior_localization/confidence`
- `/prior_localization/open3d_input_odom`
- `/navigate_to_pose/_action/status`
- `/navigate_to_pose/_action/feedback`
- `/follow_path/_action/status`
- `/follow_path/_action/feedback`
- `/spin/_action/status`
- `/spin/_action/feedback`

因此本次可以可靠分析 Nav 状态、BT SpinToPose、`map->odom`、bridge 输出字符串、`cmd_vel` 和 odom；但无法直接从 prior 原始候选话题复核 confidence 和候选 odom，只能根据 bridge status 中的候选修正量判断。

## 2. 导航完成情况

本次共收到 14 个单点导航命令，waypoint id 为 `1107` 到 `1120`，对应点位 1 到点位 14。点位 1-14 均有 `navigation_completed` ack。

点位 2 中途用户暂停一次：

- 15:56:59.813 点位 2 开始。
- 15:57:08.164 收到 `pause_navigation`，状态进入 `paused`。
- 15:57:23.068 收到 `resume_navigation`，恢复执行。
- 15:57:37.552 点位 2 完成。

16:07:31.820 收到一次 `stop_navigation`，但 ack 为 error：`当前没有在执行导航`。此时点位 14 已在 16:06:43.664 完成，所以不是导航失败。

| 点位 | 执行时间 CST | 结果 | 估计耗时 | 最大 map->odom 单步 | 备注 |
| --- | --- | --- | ---: | ---: | --- |
| 1 | 15:56:47.157-15:56:47.220 | 完成 | 0.1s | 0.000m | 目标很近 |
| 2 | 15:56:59.824-15:57:37.552 | 暂停后完成 | 14.5s 有效段 | 0.066m | 15:57:08 暂停，15:57:23 恢复 |
| 3 | 15:58:09.324-15:58:21.385 | 完成 | 12.1s | 0.136m | 多次小修正 |
| 4 | 15:58:46.953-15:58:58.274 | 完成 | 11.3s | 0.092m | 轻微修正 |
| 5 | 15:59:48.323-15:59:58.354 | 完成 | 10.0s | 0.099m | 多次小修正 |
| 6 | 16:00:15.820-16:00:26.704 | 完成 | 10.9s | 0.088m | 轻微修正 |
| 7 | 16:01:03.787-16:01:17.399 | 完成 | 13.6s | 0.067m | 轻微修正 |
| 8 | 16:01:40.127-16:02:00.899 | 完成 | 20.8s | 0.055m | 轻微修正 |
| 9 | 16:02:22.124-16:02:39.686 | 完成 | 17.6s | 0.737m | 出现 confirmed large correction |
| 10 | 16:02:57.959-16:03:12.692 | 完成 | 14.7s | 0.097m | 轻微修正 |
| 11 | 16:03:56.453-16:04:09.436 | 完成 | 13.0s | 0.129m | 多次小修正 |
| 12 | 16:04:11.456-16:04:20.787 | 完成 | 9.3s | 0.166m | 多次小修正 |
| 13 | 16:04:22.821-16:04:35.757 | 完成 | 12.9s | 0.211m | 接近中等修正 |
| 14 | 16:06:33.821-16:06:43.664 | 完成 | 9.8s | 1.217m | 本包最大漂移/大修正 |

## 3. map->odom 与定位漂移

总体判断：

- 点位 1-8、10-13：没有超过 0.25m 的单次 `map->odom` 跳变，主要是 0.05-0.21m 的小修正。
- 点位 9：出现 `0.385 -> 0.737m` 的连续大候选，最终被 bridge 以 `confirmed_large_correction` 接受。
- 点位 14：出现 `0.297 -> 1.217m` 的连续大候选，最终被 bridge 以 `confirmed_large_correction` 接受。随后 `/robot_realpose` 有一次 0.214m/0.1s 的跳变告警。

重点事件：

| 时间 CST | 点位 | bridge 事件 | 候选修正 | 结果 |
| --- | --- | --- | ---: | --- |
| 16:02:37.206 | 9 | `WOULD_PENDING nav_medium_correction` | 0.385m / 0.030rad | 进入 pending |
| 16:02:37.309 | 9 | `WOULD_HOLD nav_large_correction` | 0.545m / 0.042rad | pending 第 2 帧 |
| 16:02:37.396 | 9 | `WOULD_HOLD nav_large_correction` | 0.633m / 0.046rad | pending 第 3 帧 |
| 16:02:37.501 | 9 | `WOULD_HOLD nav_large_correction` | 0.677m / 0.047rad | pending 第 4 帧 |
| 16:02:37.601 | 9 | `WOULD_HOLD nav_large_correction` | 0.737m / 0.051rad | 第 5 帧确认 |
| 16:02:37.606 | 9 | `ACCEPTED confirmed_large_correction` | map_odom_norm 0.165m / yaw -0.165 | 已接受 |
| 16:06:36.105 | 14 | `WOULD_PENDING nav_medium_correction` | 0.297m / 0.015rad | 进入 pending |
| 16:06:36.213 | 14 | `WOULD_HOLD nav_large_correction` | 0.576m / 0.028rad | spread 过大，pending reset |
| 16:06:36.310 | 14 | `WOULD_HOLD nav_large_correction` | 0.836m / 0.041rad | spread 过大，pending reset |
| 16:06:36.411 | 14 | `WOULD_HOLD nav_large_correction` | 1.096m / 0.053rad | spread 过大，pending reset |
| 16:06:36.508 | 14 | `WOULD_HOLD nav_large_correction` | 1.136m / 0.054rad | pending 第 2 帧 |
| 16:06:36.604 | 14 | `WOULD_HOLD nav_large_correction` | 1.105m / 0.052rad | pending 第 3 帧 |
| 16:06:36.704 | 14 | `WOULD_HOLD nav_large_correction` | 1.083m / 0.052rad | pending 第 4 帧 |
| 16:06:36.812 | 14 | `WOULD_HOLD nav_large_correction` | 1.217m / 0.058rad | 第 5 帧确认 |
| 16:06:36.815 | 14 | `ACCEPTED confirmed_large_correction` | map_odom_norm 0.297m / yaw -0.179 | 已接受 |
| 16:06:37.030 | 14 | data integration 跳变告警 | realpose 0.214m / 0.1s | 速度帧被丢弃 |

## 4. bridge 状态

bridge status 统计：

- `ACCEPTED small_correction`: 7187
- `WOULD_HOLD nav_large_correction`: 11
- `PENDING large_correction`: 8
- `PENDING reset_large_candidate`: 3
- `WOULD_PENDING nav_medium_correction`: 2
- `ACCEPTED confirmed_large_correction`: 2

没有看到 `REJECTED`、实际 `HOLD`、`SPIN_GUARD`、`DEGRADED`。也就是说当前逻辑对大候选采用 pending/确认机制，但没有真正冻结 `map->odom` 更新。点位 9 和 14 的大候选都在连续确认后被接受。

## 5. odom 兜底判断

本包没有出现“bridge 冻结后靠 odom 继续推进导航”的典型场景：

- 未记录实际 `HOLD` 或 `SPIN_GUARD` 状态。
- 未记录 `REJECTED`。
- 大候选在约 0.4-0.7s 内被 pending 后接受。
- `/odom` 和 `/cmd_vel` 在导航中持续存在，Nav2 没有因 TF 断裂失败。

因此这里不是“冻结兜底成功/失败”，而是“没有冻结，最终接受了大修正”。

## 6. SpinToPose 阶段

BT log 和 debug log 均显示每个点位都涉及 `SpinToPose`，包括：

- 到点前 `goal_position` 对齐。
- 到点后的 `goal_yaw` 对齐。

两个重点大修正都发生在 SpinToPose 附近：

- 点位 9：16:02:36.045 开始 goal_yaw SpinToPose；16:02:37.206-16:02:37.606 bridge pending 并接受大修正；16:02:39.686 点位完成。
- 点位 14：16:06:33.821 开始 goal_position SpinToPose；16:06:35.851 结束该段；16:06:36.105-16:06:36.815 bridge pending 并接受大修正；16:06:41.021 开始 goal_yaw SpinToPose；16:06:43.664 点位完成。

这说明旋转阶段确实应纳入保护判断。当前日志中有 `WOULD_HOLD`，但没有真正 `HOLD/SPIN_GUARD`，所以旋转保护没有实际阻止点位 9/14 的大修正进入 `map->odom`。

## 7. 结论

1. 导航状态本身正常：点位 1-14 全部完成；点位 2 的暂停/恢复是用户命令触发，不是定位或 Nav2 失败。
2. 本包核心风险是定位修正接受策略：点位 9 接受了 0.737m 候选，点位 14 接受了 1.217m 候选。
3. 点位 14 是最严重事件，接受大修正后 0.215s 内出现 `/robot_realpose` 跳变告警。
4. 当前 bridge 只有 pending/confirmed，没有真正 HOLD/REJECT/SPIN_GUARD；所以没有形成“冻结 map->odom，用 odom 兜底”的保护链路。
5. 大修正与 SpinToPose 时间高度重叠，建议下一步优先把旋转阶段 `WOULD_HOLD` 改为真实 hold/freeze，并在离线 shadow 中验证点位 9、14 是否被拦截。

