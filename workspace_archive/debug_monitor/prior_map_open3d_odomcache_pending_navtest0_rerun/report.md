# navtest0 方案 4 odom cache + pending 回放验证

## 回放结论

这轮是完整回放，monitor 正常落盘。

- bag：`/home/ubuntu/nav_drift_test/navtest0`
- 输出目录：`debug_monitor/prior_map_open3d_odomcache_pending_navtest0_rerun`
- 采样数：`1368`
- 采样覆盖时长：`683.45s`
- bridge 当前方案：优先使用 `/prior_localization/open3d_input_odom` 的 odom cache，并在 prior pose 时间戳略超前时挂起候选，等同时间戳 odom 到达后再计算 `map->odom`

## 对 0.1s TF 问题的改善

旧方案主要问题：

- ROS 日志里有大量 `REJECTED missing_tf`
- 典型原因是 prior pose 的 stamp 比 `odom->prior_open3d_base` TF 最新 stamp 晚约 `0.1s`
- 上一轮日志中 `REJECTED missing_tf` 约 `89` 次

本轮方案 4 结果：

- 日志中未看到 `REJECTED missing_tf`
- 采样状态中未出现 `odom_cache_future`、`odom_cache_gap`、`odom_cache_pending_timeout`
- 初始化早期仍有少量 `REJECTED no_confidence`，这是正常的 confidence 门控

说明：方案 4 已经解决“pose 先到、odom/TF 后到”导致的 0.1s 时间戳拒绝问题。

## bridge 状态

summary 中事件计数：

- `WAITING`: `140`
- `REJECTED`: `28`
- `PENDING`: `1597`
- `ACCEPTED`: `6701`

采样状态：

- `WAITING`: `7`
- `REJECTED`: `1`
- `PENDING`: `4`
- `ACCEPTED`: `1356`

解释：

- summary 里的大量 `PENDING` 主要来自 `wait_odom_cache`，表示 prior pose 先到，bridge 等待同 stamp odom 补齐。
- 这类 pending 是短暂同步等待，不等于 TF 冻结或定位失败。
- 采样中几乎看不到长期 pending，说明等待很快被 odom cache 补齐并转为 accepted。

## 定位一致性

prior pose 与 `/bag/robot_realpose` 对比：

- 平均 XY 差：`0.079m`
- 95 分位 XY 差：`0.127m`
- 最大 XY 差：`0.316m`

这和上一轮基本一致，说明方案 4 没有破坏定位坐标轴或位姿精度。

## 置信度

- 有置信度采样：`1361`
- 最小：`0.8519`
- 平均：`0.9623`
- 中位数：`0.9796`
- 低于 `0.5`：`0`
- 低于 `0.9`：`146`
- 低于 `0.95`：`412`

置信度整体正常，没有长期低置信度段。

## `map->odom` 修正情况

- `map->odom` 有效采样：`1360`
- 发生变化次数：`680`
- 单步最大 XY 变化：`1.113m`
- 单步最大 yaw 变化：`0.0426rad`
- XY 单步大于 `0.10m`：`153` 次
- XY 单步大于 `0.25m`：`51` 次
- XY 单步大于 `0.50m`：`15` 次
- XY 单步大于 `1.00m`：`2` 次
- 最大更新间隔：`1.50s`

这说明方案 4 解决的是时间同步拒绝问题，不会自动解决外部定位大修正偏多的问题。大修正仍然需要后续增加：

- 大修正分段平滑注入；
- 低速/停止/到点窗口才允许大修正；
- 对 x `17m~26m`、y `11m~15m` 风险区域做更严格一致性判断；
- 结合速度话题或 odom 速度做动态阈值。

## 当前判断

方案 4 可以保留：它修复了 `missing_tf`，没有引入明显坐标轴错误，也没有让 prior pose 与 `/robot_realpose` 的误差变坏。

但它不是最终稳定方案。后续要继续优化的是“大修正怎么进入 `map->odom`”，而不是再纠结 0.1s TF 查询失败。
