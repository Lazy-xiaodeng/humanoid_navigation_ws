# navtest0 当前配置回放验证报告

## 测试输入

- bag：`/home/ubuntu/nav_drift_test/navtest0`
- 回放时长：约 `684.6s`
- 回放方式：隔离回放，bag 内 `/odom`、`/fast_lio/cloud_registered`、`/robot_realpose`、`/navigation/status` 分别重映射到 `/bag/...`
- 测试 launch：`humanoid_navigation2 prior_map_bag_test_isolated.launch.py`
- 监控输出：`debug_monitor/prior_map_open3d_spintopose_guard_3s_navtest0_isolated/samples.csv`

## 这个 bag 能验证什么

- 能验证 prior-map localization 是否能持续接收 Fast-LIO odom 和点云。
- 能验证当前 bridge 是否持续维护 `map->odom`。
- 能验证定位输出和 `/robot_realpose` 的相对一致性。
- 能验证 bridge 的接受、拒绝、等待、大修正情况。
- 能验证当前坐标轴适配链路是否能跑通。

## 这个 bag 不能验证什么

- 不能验证真实导航点是否成功到达，因为 bag 内没有 `/cmd_vel`。
- 不能验证 SpinToPose 旋转保护，因为 `/navigation/status` 全程基本是 `idle`，没有进入 SpinToPose/TURNING 状态。
- 不能用 `/robot_speed` 做速度判定，因为这个 bag 没有 `/robot_speed` 话题。
- 不能直接评估 Nav2 控制器遇到 TF 大修正时是否会停车，只能从 `map->odom` 跳变量推断风险。

## 基本运行结果

- open3d 定位成功初始化。
- 定位配准次数：`684`
- 单次定位耗时：
  - 最小：`92.565ms`
  - 平均：`172.026ms`
  - 95 分位：`240.341ms`
  - 最大：`448.498ms`
- submap 更新次数：`29`
- submap 更新时间：
  - 平均：`5.154ms`
  - 最大：`28.553ms`

## bridge 状态统计

来自 monitor 的事件计数：

- `WAITING`：`158`
- `REJECTED`：`144`
- `PENDING`：`96`
- `ACCEPTED`：`6591`

从 ROS 日志能看到的主要拒绝原因：

- `REJECTED no_confidence`：初始化早期出现。
- `REJECTED missing_tf`：主要拒绝原因，表现为 bridge 查 `odom->prior_open3d_base` 时，prior pose 时间戳比 TF 最新时间晚约 `0.1s`，触发 TF future extrapolation。

这类 `missing_tf` 不会让 TF 树断掉，bridge 会继续发布 last good `map->odom`；但它会降低有效全局校正频率。

## 置信度统计

- 有置信度采样：`1360`
- 最小：`0.0`，只在初始化早期出现 1 个采样点
- 平均：`0.9617`
- 中位数：`0.9798`
- 最大：`1.0`
- 低于 `0.5`：`1` 个采样点
- 低于 `0.7`：`1` 个采样点
- 低于 `0.9`：`147` 个采样点
- 低于 `0.95`：`410` 个采样点

置信度低于 `0.9` 的主要连续区间在 `1780122782.52s ~ 1780122847.02s`，持续约 `65s`，最低 `0.8535`。这一段 bridge 仍然主要接受小修正，没有表现为长期定位失效。

## `/robot_realpose` 覆盖情况

monitor 以 `0.5s` 采样，得到：

- 采样点数：`1365`
- 采样轨迹长度：约 `142.79m`
- x 范围：`[-6.35, 26.10]`
- y 范围：`[-11.67, 18.00]`

因为 monitor 是低频采样，轨迹长度会低于 bag 原始 10Hz 轨迹长度。

## prior 定位和 `/robot_realpose` 的一致性

在同时有 prior pose 和 `/robot_realpose` 的采样点上：

- 平均 XY 差：`0.076m`
- 95 分位 XY 差：`0.113m`
- 最大 XY 差：`0.322m`

这说明这次 bag 里，外部 prior-map localization 输出和 `/robot_realpose` 基本一致，没有看到长期跑飞。

## `map->odom` 修正情况

`map->odom` 有效采样：`1359`

- 发生变化次数：`679`
- 单步最大 XY 变化：`1.111m`
- 单步最大 yaw 变化：`0.0368rad`
- XY 单步大于 `0.10m`：`152` 次
- XY 单步大于 `0.25m`：`48` 次
- XY 单步大于 `0.50m`：`19` 次
- XY 单步大于 `1.00m`：`1` 次

大于 `0.25m` 的变化被聚类后有 `16` 个区间，最大风险区间集中在 sim time `1780123181s ~ 1780123258s`，机器人位置大约在 x `17m ~ 26m`、y `11m ~ 15m` 一带。

最大单步修正：

- sim time：`1780123198.522s`
- 单步 XY：`1.111m`
- 单步 yaw：`0.0368rad`
- robot pose：`x=26.101, y=13.485`
- confidence：`0.9856`
- 状态采样：`ACCEPTED small_correction`

注意：采样状态显示为 `small_correction`，但 ROS 日志显示实际有 `ACCEPTED confirmed_large_correction`。这是因为 monitor 只按 `0.5s` 采样，可能错过大修正刚发生的那一瞬间，下一帧已经变成小修正状态。

## 结论

这次 bag 对 prior-map localization 本体是正向结果：定位能初始化、能持续跑完整包、置信度整体较高，而且 prior pose 和 `/robot_realpose` 的平均差只有约 `7.6cm`。

但当前 bridge 参数对导航仍然偏激进：这包里接受了 `48` 次超过普通阈值的大修正，最大单步 `map->odom` 跳变达到 `1.11m`。如果真实 Nav2 正在闭环控制，这类瞬时大修正有概率造成机器人位姿在 map 下跳变、局部规划突变或控制器停车。

下一步优先优化方向：

- 给 bridge 增加 TF 查询容忍或使用最近 TF，先减少 `missing_tf` 拒绝。
- 大修正不要直接瞬时发布，改成分段平滑注入，或只在低速/暂停/到点后窗口允许。
- 对大修正增加空间分区统计，重点检查 x `17m ~ 26m`、y `11m ~ 15m` 区域是否地图结构重复或局部点云质量变化。
- 当前包不能验证 SpinToPose 保护，需要后续录到真实 `/navigation/status` 或 `/cmd_vel`/`robot_speed` 的导航 bag 再验证。
