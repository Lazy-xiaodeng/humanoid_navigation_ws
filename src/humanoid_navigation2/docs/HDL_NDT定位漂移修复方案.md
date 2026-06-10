# HDL/NDT 定位跳变保护说明

## 背景

导航过程中机器人在弯角附近出现定位漂移：实际机器人仍在通道中间，但 RViz 中的定位跳到右侧贴墙位置。监控数据表明，问题发生时 Nav2 只是依据错误 TF 做控制和碰撞判断，根因不是路径跟踪，而是 HDL/NDT 在走廊弯角处接受了错误的局部最优匹配，并立即发布新的 `map -> odom`。

本次修复目标：

- 连续定位时同时检查 `fitness`、`inlier_fraction` 和单帧位姿跳变量。
- 低置信度匹配不写入 UKF，不更新为 NDT 错误位姿。
- 被拒绝时短时间只信 Fast-LIO odom 预测，保持 `map -> odom` 平滑，避免定位瞬间贴墙。
- 匹配质量恢复后继续使用 NDT 校正，限制 Fast-LIO 长时间累计漂移。

## 修改文件

- `src/hdl_localization/include/hdl_localization/pose_estimator.hpp`
- `src/hdl_localization/src/hdl_localization/pose_estimator.cpp`
- `src/hdl_localization/apps/hdl_localization_nodelet.cpp`
- `src/humanoid_navigation2/launch/navigation2.launch.py`

## 定位节点修改

### 1. 扩展 NDT 校正返回信息

`PoseEstimator::correct()` 增加了以下输入和输出：

- `max_correction_translation`
- `max_correction_yaw`
- `correction_rejected_by_jump`
- `min_inlier_fraction`
- `max_inlier_distance`
- `inlier_fraction`
- `correction_rejected_by_inlier`

这样调用方可以知道本帧 NDT 是因为未收敛、`fitness` 超阈值、`inlier_fraction` 过低，还是单帧跳变过大而被拒绝。

### 2. 增加 inlier_fraction 检查

新增 `compute_inlier_fraction()`，对 NDT 对齐后的点云逐点查找地图最近邻：

- 最近邻距离小于 `scan_matching_inlier_max_correspondence_distance` 的点计为 inlier。
- inlier 数量除以对齐点云数量得到 `inlier_fraction`。
- 若 `inlier_fraction < min_scan_matching_inlier_fraction`，本帧 NDT 结果被拒绝。

该检查放在 UKF `correct()` 之前，因此坏匹配不会写入滤波器。

### 3. 增加单帧位姿跳变量检查

NDT 输出位姿会和预测位姿比较：

- 平面位移差超过 `max_scan_matching_correction_translation` 时拒绝。
- yaw 差超过 `max_scan_matching_correction_yaw` 时拒绝。

预测位姿优先使用 Fast-LIO odom 增量预测后的 `odom_matrix()`；没有 odom 预测时退回当前初始化预测。

### 4. 拒绝低置信度 NDT 后使用 odom 预测发布

当 `correction_accepted == false` 且已有 odom 预测时，节点发布 `pose_estimator->odom_matrix()`，而不是 NDT 错误结果。

效果上：

- NDT 错误匹配不会立即拖动 `map -> odom`。
- `map -> odom` 会保持平滑，机器人位姿短时间沿 Fast-LIO odom 增量前进。
- 当 NDT 匹配恢复到阈值内，定位继续用 NDT 校正。

### 5. 日志增强

拒绝日志现在包含更多信息：

```text
scan matching rejected: converged=... fitness=... threshold=... inlier=... min_inlier=... jump_rejected=... inlier_rejected=...
```

调试时重点看：

- `jump_rejected=1`：NDT 候选位姿相对 odom 预测跳变过大。
- `inlier_rejected=1`：点云和地图重合比例过低。
- `fitness` 高于阈值：整体匹配误差过大。

## launch 修改

`navigation2.launch.py` 中 HDL 定位节点新增并启用了以下参数：

```python
'max_scan_matching_fitness_score': 0.20,
'min_scan_matching_inlier_fraction': 0.85,
'scan_matching_inlier_max_correspondence_distance': 0.5,
'max_scan_matching_correction_translation': 0.25,
'max_scan_matching_correction_yaw': 0.12,
'publish_odom_prediction_on_rejection': True,
```

这些参数来自本次现场监控数据：

- 正常定位时 `fitness` 约 `0.05`，`inlier_fraction` 约 `0.95`。
- 弯角漂移时 `fitness` 升到约 `0.30`，`inlier_fraction` 降到约 `0.76`。

因此当前阈值用于拦截弯角处的错误局部最优，同时保留正常 NDT 校正。

## Fast-LIO 坐标系处理

Fast-LIO 原始 odom 坐标轴不是 ROS 标准轴：

- Fast-LIO：`x` 朝左，`y` 朝下，`z` 朝后。
- ROS 标准：`x` 朝前，`y` 朝左，`z` 朝上。

本次修复没有直接使用 Fast-LIO `/odom` 消息作为标准 odom，而是沿用已有 TF 链：

```text
odom -> camera_init -> body -> base_footprint
```

HDL 定位节点通过 TF 查询的是 `base_footprint` 在 `odom` 下的前后帧增量，因此使用的是转换后的标准机器人坐标，不是 Fast-LIO 原始轴向。

## 实现效果

本次修改实现的是定位层的防跳变保护：

1. NDT 正常时，继续用 NDT 修正全局位姿。
2. NDT 匹配质量变差时，拒绝该帧结果，不让错误局部最优进入 UKF。
3. NDT 候选位姿相对 Fast-LIO odom 预测突然跳变时，拒绝该帧结果。
4. 拒绝期间短时间使用 Fast-LIO odom 增量维持连续运动。
5. NDT 恢复可信后重新接受校正，避免长期只靠 odom 漂移。

这能解决的典型现象：

- 走廊弯角相似墙面导致 NDT 错配。
- 定位瞬间跳到墙边。
- Nav2 因错误 TF 误判机器人贴墙或前方碰撞。

## 可靠性评估

这个实现是可靠的工程防护方案，但不是绝对定位保证。

可靠的原因：

- 拒绝条件覆盖了 NDT 错配的三个主要信号：`fitness` 变差、`inlier_fraction` 降低、位姿相对 odom 预测突跳。
- 拒绝发生在 UKF 校正之前，坏结果不会污染滤波状态。
- 拒绝后使用的是转换到 ROS 标准坐标系后的 TF odom 增量，不直接使用 Fast-LIO 原始轴向。
- Fast-LIO 在短时间内通常局部连续性较好，适合作为 NDT 失信时的短时兜底。

需要注意的边界：

- 如果 Fast-LIO 本身短时间剧烈漂移，兜底位姿也会受影响。
- 如果长时间没有可靠 NDT 校正，odom 仍会累计漂移。
- 阈值过严会导致正常帧也被拒绝，表现为定位过度依赖 odom。
- 阈值过松会让弯角错误匹配继续漏过。
- 该方案避免错误匹配被接受，但不能替代地图质量、点云时间同步、外参和 TF 树正确性。

## 调参建议

当前建议先使用：

```text
max_scan_matching_fitness_score = 0.20
min_scan_matching_inlier_fraction = 0.85
scan_matching_inlier_max_correspondence_distance = 0.5
max_scan_matching_correction_translation = 0.25
max_scan_matching_correction_yaw = 0.12
```

如果正常行走时频繁出现 `inlier_rejected=1`：

- 将 `min_scan_matching_inlier_fraction` 降到 `0.80`。
- 或将 `scan_matching_inlier_max_correspondence_distance` 调到 `0.6`。

如果弯角仍然会跳墙：

- 将 `max_scan_matching_correction_translation` 收紧到 `0.18`。
- 将 `max_scan_matching_correction_yaw` 收紧到 `0.08`。
- 将 `max_scan_matching_fitness_score` 收紧到 `0.15`。

如果定位短时间平滑但慢慢偏：

- 说明 NDT 被拒绝太多或 Fast-LIO odom 有累计误差。
- 需要适当放宽阈值，让可信 NDT 更早恢复校正。

## 验证方法

启动前重新 source：

```bash
source install/setup.bash
```

启动导航后，在弯角复测并观察日志：

```text
scan matching rejected ... jump_rejected=1
scan matching rejected ... inlier_rejected=1
```

预期结果：

- 弯角处即使 NDT 出现错误候选，也不会立即把定位拉到墙边。
- RViz 中机器人位姿应沿实际路线连续移动。
- Nav2 不再因为定位跳墙而误报贴墙碰撞。

## 构建状态

已执行：

```bash
colcon build --packages-select hdl_localization humanoid_navigation2
```

构建通过。仅有 PCL deprecated header 警告，不影响运行。
