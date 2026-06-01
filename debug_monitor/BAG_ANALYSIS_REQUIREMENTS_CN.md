# 导航 bag 定位漂移分析要求

这个文档记录后续分析导航 bag 时的固定要求。用户只需要提供：

- bag 路径，例如 `/home/ubuntu/nav_drift_test/nav_drift_test20/nav_drift_test20_0.mcap`
- 日志路径，例如 `/home/ubuntu/humanoid_ws/debug_output.txt`
- 是否需要额外模拟某个保护策略

后续默认按本文档输出分析，不需要每次重新说明。

## 分析目标

每次分析都要回答这些问题：

1. 本次导航完成了哪些点位，哪些点位失败、暂停或没有完成。
2. 每个点位导航过程中定位是否漂移，漂移发生在什么时间、什么位置附近。
3. `map->odom` 有没有明显跳变，跳变前后位姿是多少，跳变量是多少。
4. prior-map localization 是否输出了错误候选，bridge 是否接受、拒绝、pending、冻结。
5. 是否出现“定位更新被冻结后，靠 odom 继续推进导航”的情况。
6. 是否出现 TF 断裂、导航暂停、Nav2 失败、状态管理器拦截、点位缓存。
7. 到点后的 SpinToPose 原地旋转阶段是否被纳入判断，旋转冻结是否正常生效。
8. 如果模拟打开保护策略，要说明保护后会拦哪些跳变、冻结多久、是否靠 odom 兜底、是否可能暂停。

## 输入数据要求

优先使用下面这些话题。如果 bag 缺少某些话题，报告中要明确说明影响。

### 必须话题

- `/tf`
- `/tf_static`
- `/rosout`
- `/navigation/status`
- `/localization/prior_map_odom_bridge_status`
- `/prior_localization/odom`
- `/prior_localization/confidence`
- `/prior_localization/open3d_input_odom`
- `/odom`
- `/robot_realpose`
- `/robot_speed`
- `/cmd_vel`
- `/navigate_to_pose/_action/status`
- `/navigate_to_pose/_action/feedback`
- `/follow_path/_action/status`
- `/follow_path/_action/feedback`
- `/spin/_action/status`
- `/spin/_action/feedback`
- `/behavior_tree_log`
- `/plan`

### 推荐话题

- `/goal_pose`
- `/initialpose`
- `/app/navigation_command`
- `/app/waypoint_command`
- `/navigation/waypoints_data`
- `/navigation/acknowledgments`
- `/localization/recovery_status`
- `/prior_localization/pose`
- `/prior_localization/pose_with_covariance`
- `/prior_localization/open3d_odom2map`
- `/prior_localization/open3d_odom2map_kalman`
- `/prior_localization/open3d_baselink2map_kalman`
- `/prior_localization/open3d_motionlink2map`
- `/prior_localization/open3d_scan`
- `/prior_localization/open3d_submap`
- `/prior_localization/open3d_scan2map`

### 可选大数据话题

这些话题主要用于点云细查、建图/匹配质量分析，文件会很大：

- `/prior_localization/open3d_input_cloud`
- `/fast_lio/cloud_registered`
- `/airy_points_filtered`
- `/airy_points`
- `/airy_imu`
- `/local_costmap/costmap`
- `/global_costmap/costmap`
- `/local_costmap/costmap_updates`
- `/global_costmap/costmap_updates`
- `/local_costmap/published_footprint`
- `/global_costmap/published_footprint`

## 默认分析流程

### 1. 基础完整性检查

先确认：

- bag 文件是否存在，大小和 metadata 是否正常。
- debug log 是否对应本次 bag。
- bag 中实际包含哪些话题。
- 是否包含 `/navigation/status`、bridge status、prior odom、open3d input odom、`/cmd_vel`、`/odom`。

如果日志和 bag 时间明显对不上，要先说明，不要强行下结论。

### 2. 导航状态分析

从 `debug_output.txt` 和 `/navigation/status` 提取：

- 每个点位的开始时间。
- 每个点位的完成时间。
- 点位编号、点位名称、目标坐标，如果可用。
- 是否出现 `Goal failed`、`paused`、`Nav2确认到达路点`。
- 是否出现点位缓存、状态管理器拦截、定位不健康提示。
- 是否出现主动转向/后退行为，尤其是 `SpinToPose`。

输出每个点位：

- 点位
- 开始时间
- 完成/失败时间
- 耗时
- 导航结果
- 是否出现 SpinToPose
- 是否出现暂停/失败/恢复运动

### 3. map->odom 漂移和跳变分析

从 `/tf` 中提取 `map->odom`，计算：

- 每个点位内 `map->odom` 数值变化次数。
- 每个点位内最大单次平移跳变。
- 每个点位内最大 yaw 跳变。
- `>=0.05m` 或 `>=0.03rad` 的变化次数。
- `>=0.25m` 中等修正。
- `>=0.50m` 大跳。
- 跳变前 `map->odom` 位姿。
- 跳变后 `map->odom` 位姿。
- 跳变量 `dx/dy/dxy/dyaw`。

判断标准默认：

- `<0.25m`：轻微修正。
- `0.25m~0.50m`：中等修正，需要关注但不一定是漂移。
- `>0.50m`：明显漂移/大修正，需要列入重点事件。
- `>1.00m`：严重漂移/错误局部最优高风险。

### 4. prior-map localization 与 bridge 状态分析

结合：

- `/prior_localization/odom`
- `/prior_localization/confidence`
- `/prior_localization/open3d_input_odom`
- `/localization/prior_map_odom_bridge_status`

分析：

- prior-map localization 输出候选是否跳变。
- bridge 是 `ACCEPTED`、`REJECTED`、`PENDING`、`HOLD`、`DEGRADED` 还是 `SPIN_GUARD`。
- 被接受的大修正发生在哪个点位、哪个时间。
- 被拒绝/冻结的候选有多少帧。
- confidence 是否长期为 0 或过低。
- 是否出现 `wait_odom_cache`、`missing_odom_cache`、`stale_pose`、`stale_confidence`。

每个重点事件要尽量列：

- 时间
- 点位
- 事件类型
- 候选修正量
- 是否接受
- 是否拒绝
- 是否冻结
- 是否后来恢复

### 5. odom 兜底判断

“靠 odom 兜底”不是 TF 断裂，而是：

- bridge 没有接受新的 `map->odom`；
- 仍持续发布 last good `map->odom`；
- `/odom` 还在连续变化；
- `/cmd_vel` 仍有速度；
- 导航还在推进。

判断时要结合：

- bridge 是否处于 `HOLD`、`SPIN_GUARD`、长时间 `REJECTED`。
- 同一时间窗口 `/odom` 走了多少米。
- 同一时间窗口 `/cmd_vel` 是否还在动。
- `/navigation/status` 是否仍是 executing/running/active。

输出：

- 是否存在 odom 兜底。
- 发生在哪些点位。
- 开始时间、结束时间、持续多久。
- 期间 odom 位移。
- 期间最大线速度、角速度。
- 是否导致导航失败或仍然完成。

### 6. SpinToPose 旋转阶段分析

必须把到点后的原地旋转纳入分析。

需要区分：

- 到点前朝向预对齐的 SpinToPose。
- 到点后的 goal yaw SpinToPose。
- SpinToPose 期间 bridge 的旋转冻结。
- 旋转结束后的 settle 冻结。

判断：

- SpinToPose 冻结期间是否持续发布 last good TF。
- 是否错误地把旋转保护当成定位失败。
- 旋转阶段是否引发 prior-map localization 视野变化和大跳。
- 旋转结束后是否能恢复定位更新。

### 7. 保护策略 shadow 模拟

如果用户要求“模拟打开保护”，不要修改真实 launch，不要改生产参数。只用离线脚本 shadow。

默认模拟第一版保护策略：

- 小修正 `<=0.25m` 直接接受。
- 导航中 `0.25m~0.50m` 连续 5 帧稳定后接受。
- 导航中 `>0.50m` 不接受，冻结 `map->odom` 更新，持续发布 last good TF。
- 空闲/讲解/到点后 `<=1.00m` 连续 5 帧稳定后接受回正。
- 超过 `1.00m` 不自动接受。
- 冻结超过 `3.0s` 认为 degraded。

模拟报告必须包含：

- 实际记录最大跳变。
- 当前逻辑 shadow 最大跳变。
- 保护后 shadow 最大跳变。
- 保护拦截了哪些候选。
- hold 次数和聚类段数。
- 每段 hold 的开始、结束、持续时间、最大 dx。
- hold 期间 odom 位移。
- hold 期间 `/cmd_vel` 是否仍在动。
- 是否触发 degraded。
- 是否可能需要状态管理器暂停。
- 空闲阶段接受了哪些回正。

注意说明：

- 这是离线 shadow，不是真正 Nav2 闭环重跑。
- 不能直接证明真实机器人一定不会停或一定能恢复。
- 如果要闭环验证，需要过滤 bag 中旧 `map->odom`，让实验 bridge 重新发布 TF，并重新跑 Nav2。

## 输出文档要求

每次最终都生成一个中文 md 报告，建议路径：

```text
debug_monitor/<bag_name>_analysis/<bag_name>_protection_analysis_cn.md
```

报告结构默认如下：

1. 总体结论
2. 数据来源和限制
3. 真实导航状态
4. 每个点位路线定位漂移情况
5. 重点漂移/跳变事件
6. bridge 接受/拒绝/冻结情况
7. odom 兜底和导航暂停判断
8. SpinToPose 旋转保护分析
9. 保护策略 shadow 模拟效果
10. 风险判断
11. 后续建议
12. 生成的中间文件

## 每点位表格字段

每个点位至少输出：

- 点位
- 时间段
- 耗时
- 导航结果
- 是否 SpinToPose
- 实际导航中最大 `map->odom` 跳变
- 实际全段最大 `map->odom` 跳变
- odom 位移
- prior 位移，如果可用
- robot_realpose 位移，如果可用
- accepted 数量
- rejected 数量
- pending/hold 数量
- spin 冻结拒绝数量
- 是否漂移
- 是否纠正回来
- 是否 odom 兜底
- 是否暂停

## 重点事件表格字段

重点事件至少输出：

- 点位
- 时间
- 类型：accepted / rejected / pending / hold / degraded / spin_guard
- 跳变前位姿
- 跳变后位姿
- dx
- dy
- dxy
- dyaw
- confidence
- 是否接受
- 是否影响导航
- 后续是否恢复

## 判断口径

### 是否漂移

满足任一条件就标记为漂移或大修正：

- 导航中 `map->odom` 单次跳变 `>0.50m`。
- prior-map 候选相对 last good TF 偏差 `>0.50m`。
- `robot_realpose` 或 Nav2 当前位姿相对目标出现明显不连续。
- 发生大修正后局部规划失败、目标失败、机器人明显走偏。

### 是否纠正回来

满足任一条件可以认为“纠正回来”：

- 后续接受小修正后 `map->odom` 回到稳定范围。
- 空闲窗口接受稳定回正。
- 后续点位继续完成，并且没有持续扩大误差。
- prior 候选重新回到 last good 附近。

如果只是 TF 冻住然后导航完成，不能简单说定位纠正回来，要说明“靠 odom 连续性完成”。

### 是否 odom 兜底

满足以下组合时判断为 odom 兜底：

- bridge 未接受新全局修正或处于 hold/spin freeze。
- last good `map->odom` 仍持续发布。
- `/odom` 有明显位移。
- `/cmd_vel` 有运动命令。
- 导航状态仍 active/executing。

### 是否暂停

需要从以下信息判断：

- `/navigation/status` 是否 paused。
- 日志是否有 paused、暂停、缓存点位、等待定位恢复。
- `/cmd_vel` 是否长期为 0。
- Nav2 action 是否进入失败、取消或等待。

不能仅因为 bridge hold 就说导航暂停。

## 最终回答要求

最终回复用户时要简洁说明：

- 报告文件路径。
- 这次是否完成所有点位。
- 最大漂移/跳变在哪里。
- 是否出现 odom 兜底。
- 是否暂停或失败。
- 模拟保护后效果是正向还是有风险。

如果没有做真正闭环重跑，必须明确说明“这是离线 shadow 模拟，不是闭环 Nav2 回放”。

