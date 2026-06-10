# Prior-map localization 维护 map->odom 优化方案

本文档记录 FAST-LIO prior-map localization 接入后，`prior_map_odom_bridge` 维护 `map -> odom` 的优化方案。

目标不是一次性把所有策略都接入，而是按单方案逐步修改、逐步跑 bag 验证，最后筛选出适合当前机器人、地图和现实环境的参数组合。

## 当前结论

根据 `nav_drift_test13` 和 `nav_drift_test14` 的 prior-map 回放结果，当前问题可以分成两类：

1. prior-map localization 本体没有明显低置信度崩溃。
   - 两个 bag 的导航目标段内，基本没有 `low_confidence` / `no_confidence` 导致的长期冻结。
   - 多数拒绝来自主动的 `SpinToPose` 冻结保护。

2. bridge 接受策略仍会吃进偏大的 `map -> odom` 修正。
   - bag14 中后半段点位 13-17 出现多次 `0.3m ~ 1.1m` 级别的 accepted 修正。
   - bag13 中点位 10/11 附近出现 `0.9m`、`1.3m` 级别 accepted 修正。
   - 这些修正不是 TF 断裂，也不是低置信度拒绝，而是 bridge 认为候选可接受后写入了 `map -> odom`。

因此后续优化重点应放在 bridge 层：对外部定位结果做更严格、更平滑、更可解释的门控。

## TF 职责边界

系统中应保持如下职责：

```text
map -> odom       prior_map_odom_bridge 维护
odom -> base      Fast-LIO / 里程计维护
base -> sensors   静态 TF 或机器人 URDF 维护
```

bridge 的职责不是替代 odom，也不是直接控制机器人，而是把外部 prior-map 定位结果转换成稳定的 `map -> odom` 校正。

冻结外部定位更新时，bridge 仍然必须持续发布 last good `map -> odom`。这样 TF 链不断，Nav2 可以短时间依赖 `odom -> base` 连续运动。

## 已验证的有效策略

### SpinToPose 保护

当前 3s 版本比 1.2s 更好：

```yaml
enable_spin_to_pose_guard: true
spin_to_pose_guard_settle_sec: 3.0
spin_to_pose_guard_max_duration_sec: 8.0
```

该策略只在 `/navigation/status` 进入到点后的 `TURNING` / `SpinToPose` 阶段时冻结外部定位候选。冻结期间不接受 prior-map 候选，只重发 last good `map -> odom`。

结论：

- 保留 3s settle。
- 不建议先盲目加到 5s 或 8s。
- 后续重点应放在解冻后的候选门控。

## 需要新增的核心策略

### 1. 真实 map->odom 单步硬门控

当前普通阈值 `max_small_correction_translation=0.25` 的判断应基于：

```text
candidate_map_to_odom 相对 accepted_map_to_odom 的 XY 平移范数
candidate_map_to_odom 相对 accepted_map_to_odom 的 yaw 差
```

也就是：

```python
dx = candidate_map_odom.x - accepted_map_odom.x
dy = candidate_map_odom.y - accepted_map_odom.y
dxy = sqrt(dx * dx + dy * dy)
dyaw = abs(normalize(candidate_yaw - accepted_yaw))
```

必须明确：该阈值不是单独 x 差，也不是导航到点容差，而是写入 `map -> odom` 前的 TF 修正量。

新增硬门控参数：

```yaml
max_direct_map_odom_step_translation: 0.25
max_direct_map_odom_step_yaw: 0.08
```

规则：

```text
如果候选相对当前 last good map->odom 的实际变化超过该阈值：
  不允许直接 ACCEPTED small_correction
  必须进入 PENDING 或 REJECTED
```

预期效果：

- 挡住 bag13 中 `0.9m`、`1.3m` 级别 accepted small correction。
- 挡住 bag14 中点位 13-17 的 `0.6m ~ 1.1m` 级别 accepted correction。

第一轮优先接入该策略。

### 2. SpinToPose 解冻缓启动

SpinToPose 结束后，外部定位往往还处于视野变化、点云重叠变化、匹配结果未稳定阶段。因此不能立即恢复普通阈值。

新增参数：

```yaml
post_spin_strict_duration_sec: 3.0
post_spin_max_translation: 0.10
post_spin_max_yaw: 0.05
post_spin_pending_instead_of_reject: true
```

规则：

```text
SpinToPose 保护结束后的 3s 内：
  只允许 <= 0.10m 且 yaw <= 0.05rad 的 map->odom 修正直接接受
  超过该范围的候选进入 PENDING
```

预期效果：

- 针对 bag14 中 `1780069486` 附近的解冻后大修正。
- 针对到点旋转结束后立即出现的 accepted small correction。

### 3. PENDING 大修正确认收紧

当前 3 帧一致即可确认大修正，对当前场景偏宽。

建议参数：

```yaml
required_consistent_frames: 5
consistency_translation_tolerance: 0.12
consistency_yaw_tolerance: 0.05
max_large_correction_translation: 1.5
max_large_correction_yaw: 0.6
```

新增确认条件：

```text
大修正只有在机器人低速或近似停止时才能确认。
```

低速判断可按可用信号分级：

1. 优先使用 `/robot_speed`，如果该话题表示实际机身速度。
2. 有 Nav2 时可参考 `/cmd_vel`。
3. 没有速度话题时，可从 `/odom` 相邻帧差分估计速度。

建议阈值：

```yaml
large_correction_max_linear_speed: 0.08
large_correction_max_angular_speed: 0.15
```

含义：

```text
机器人还在明显移动或旋转时，不接受大修正，只继续 PENDING。
```

### 4. map->odom 输出限速

即使某个候选已经通过确认，也不一定要一帧跳过去。可以把目标 `map -> odom` 和当前发布的 `map -> odom` 分开：

```text
accepted_target_map_to_odom: 通过门控后的目标 TF
published_map_to_odom: 当前实际广播出去的 TF
```

新增参数：

```yaml
enable_map_odom_rate_limit: true
max_publish_step_translation: 0.05
max_publish_step_yaw: 0.02
```

规则：

```text
每次 publish_timer 最多让 published_map_to_odom 朝 target 移动 5cm
yaw 每次最多变化 0.02rad
```

预期效果：

- 即使接受了 0.3m 修正，Nav2 看到的是连续变化，不是一帧跳变。
- 减少 global pose 瞬间跳动导致的重规划、到点判断抖动、碰撞误判。

注意：

- 该方案会引入 map 校正滞后。
- 应放在第一轮门控验证之后再接入。

### 5. 滑动窗口累计修正预算

很多小修正连续接受，也可能累计成大漂移。

新增参数：

```yaml
enable_correction_budget: true
correction_budget_window_sec: 5.0
max_correction_budget_translation: 0.35
max_correction_budget_yaw: 0.12
```

规则：

```text
统计最近 5s 内实际写入 map->odom 的累计修正量。
如果累计超过预算：
  新候选不直接接受
  进入 PENDING 或短暂冻结
```

预期效果：

- 针对 bag14 后半段连续 small correction 逐步拉动 `map -> odom` 的情况。

### 6. 手动遥控 bag 的旋转保护信号

如果后续用遥控器手动走完整路径，bag 里可能没有 Nav2 的 `/navigation/status` 或没有 `SpinToPose` 状态。

这种情况下，当前 SpinToPose guard 不会自动触发。

建议增加可选输入：

```yaml
enable_motion_rotation_guard: false
robot_speed_topic: /robot_speed
robot_speed_timeout_sec: 0.5
rotation_guard_angular_speed: 0.35
rotation_guard_linear_speed_max: 0.08
rotation_guard_settle_sec: 2.0
```

规则：

```text
当机器人实际角速度较大，且线速度较小：
  认为处于原地旋转或近似原地旋转
  冻结 prior-map 候选
  旋转结束后等待 settle_sec
```

优先级建议：

1. 有 `/navigation/status` 时，以 SpinToPose guard 为准。
2. 没有导航状态时，才启用 `/robot_speed` 或 odom 差分旋转保护。

## 实验落地顺序

### 实验 0：基线

使用当前代码和参数：

```yaml
spin_to_pose_guard_settle_sec: 3.0
```

验证 bag：

- `nav_drift_test13`
- `nav_drift_test14`
- 后续新录制完整遥控 bag

记录：

```text
真实 map->odom 跳变次数
ACCEPTED / REJECTED / PENDING 数量
accepted small correction 中 >0.3m 的次数
confirmed large correction 次数
每个目标点最终距离
是否 completed / paused / failed / cancelled
最长无 ACCEPT 时间
low_confidence / no_confidence 次数
```

### 实验 1：单步硬门控 + 解冻缓启动

接入：

```text
真实 map->odom 单步硬门控
SpinToPose 解冻后 3s 严格阈值
```

建议参数：

```yaml
max_direct_map_odom_step_translation: 0.25
max_direct_map_odom_step_yaw: 0.08
post_spin_strict_duration_sec: 3.0
post_spin_max_translation: 0.10
post_spin_max_yaw: 0.05
```

通过标准：

```text
bag13 中 0.9m / 1.3m accepted correction 不再直接接受
bag14 中点位 13-17 的 0.6m+ accepted correction 明显减少
TF 不断
前面 completed 点位不明显变差
low_confidence 不应增加，因为这是 bridge 门控，不是定位本体退化
```

### 实验 2：收紧 PENDING 确认

接入：

```text
required_consistent_frames 从 3 增加到 5
一致性阈值收紧
大修正只允许低速确认
```

建议参数：

```yaml
required_consistent_frames: 5
consistency_translation_tolerance: 0.12
consistency_yaw_tolerance: 0.05
large_correction_max_linear_speed: 0.08
large_correction_max_angular_speed: 0.15
```

通过标准：

```text
confirmed_large_correction 误接受减少
不会导致长时间完全无全局校正
目标点最终误差不显著变大
```

### 实验 3：输出限速

接入：

```text
target map->odom 和 published map->odom 分离
发布端限速逼近 target
```

建议参数：

```yaml
enable_map_odom_rate_limit: true
max_publish_step_translation: 0.05
max_publish_step_yaw: 0.02
```

通过标准：

```text
监控中的相邻 published map->odom 跳变显著降低
Nav2 不再看到一帧级大跳
不会因为校正滞后导致目标点误差明显增加
```

### 实验 4：累计修正预算

接入：

```text
5s 滑动窗口累计修正预算
```

建议参数：

```yaml
correction_budget_window_sec: 5.0
max_correction_budget_translation: 0.35
max_correction_budget_yaw: 0.12
```

通过标准：

```text
连续 small correction 拉动 map->odom 的情况减少
不会长期阻止正常小漂移校正
```

### 实验 5：手动遥控 bag 旋转保护

接入：

```text
/robot_speed 或 odom 差分旋转保护
```

只在手动 bag 或没有 `/navigation/status` 的场景启用。

通过标准：

```text
手动原地旋转期间 map->odom 不被污染
旋转结束后能恢复正常校正
不会在正常曲线行走中频繁误冻结
```

## 新录制遥控 bag 建议

由于当前定位不稳定，无法完全依赖 Nav2 自动跑完整路径，可以用遥控器按导航点位走一圈。

建议录制话题：

```text
/odom
/fast_lio/cloud_registered
/tf
/tf_static
/robot_realpose
/initialpose
/robot_speed
/cmd_vel                如果有
/navigation/status      如果当时仍有导航状态机
/plan                   如果有
/localization/ndt_status
/localization/recovery_status
```

录制动作要求：

```text
每个目标点附近都走完整：
  接近目标点
  到达容差范围内停顿
  原地旋转或调整朝向
  旋转后停顿 2-3s
  再去下一个点

最后一个点也要完整录到：
  完成停顿
  或明确失败/暂停状态
```

如果是纯手动遥控，没有 `/navigation/status`，则这包主要验证 prior-map localization 和 bridge 门控；SpinToPose guard 需要用 `/robot_speed` 旋转保护替代验证。

## 每轮验证报告格式

每次改完一个方案，跑 bag 后都按下面格式记录：

```text
bag 名称:
方案:
参数:

整体统计:
  samples:
  unique goals:
  ACCEPTED:
  REJECTED:
  PENDING:
  low_confidence:
  no_confidence:
  spin_to_pose_freeze_tf:
  confirmed_large_correction:

map->odom 跳变:
  总次数:
  accepted 跳变次数:
  pending 跳变次数:
  最大 accepted dxy:
  最大 pending dxy:

点位统计:
  点位 - 是否 completed/paused/failed - 最终距离 - accepted dxy 总量 - 最大 accepted dxy - pending dxy 总量

结论:
  是否比基线更稳:
  是否误伤导航:
  是否建议保留:
  下一轮需要改什么:
```

## 当前推荐第一步

下一次改代码时，建议只做实验 1：

```text
真实 map->odom 单步硬门控
SpinToPose 解冻缓启动
```

先不要同时接入输出限速、累计预算、robot_speed 旋转保护。

原因：

- bag13 / bag14 中最大问题是偏大的 accepted correction。
- 实验 1 可以最直接验证能否挡住这些已知问题。
- 单项改动便于判断效果，避免多个策略叠加后无法定位收益来源。

## open3d_loc 本体参数和算法细节

本节记录当前 prior-map localization 节点本体的关键参数、源码位置、当前值和建议调整方向。

相关文件：

```text
src/FAST_LIO_LOCALIZATION_HUMANOID/open3d_loc/src/global_localization.cpp
src/FAST_LIO_LOCALIZATION_HUMANOID/open3d_loc/src/open3d_registration/open3d_registration.cpp
src/humanoid_navigation2/launch/navigation2.launch.py
/tmp/prior_map_bag_test.launch.py
```

### 1. 该节点是不是全局重定位

当前 open3d_loc 更接近“有初值的局部 scan-to-map tracking”，不是严格意义上的全局重定位。

它的流程是：

```text
使用当前 odom 和上一次 odom2map 作为初值
裁剪当前位置附近的 prior map 子图
累计最近若干帧 cloud_registered 作为 source
用 ICP 把 source 匹配到 target map 子图
如果 fitness 超过阈值，更新 odom2map
```

它不会在整张地图上做多候选全局搜索，也不会像粒子滤波那样维护多个假设。

因此：

```text
初始位姿需要大致正确。
启动位置偏差太大时，不应期待它从地图任意位置自动收敛。
```

### 2. 当前地图范围

当前 bridge/open3d_loc 使用的是：

```text
hall_open3d_grounded.pcd
```

解析结果：

```text
点数: 2,250,994
x范围: -14.55 ~ 32.78，长度约 47.33m
y范围: -27.47 ~ 24.88，长度约 52.35m
z范围: 0.00 ~ 7.97，长度约 7.97m
```

所以当前地图整体 XY 尺寸本身不到 60m。源码里 `60 x 60 x 40` 的 OBB 裁剪会覆盖大部分甚至整张地图，局部定位会退化成“拿很大范围地图参与 ICP”。

这会带来两个问题：

```text
重复走廊/相似墙面更容易参与匹配。
fitness 可能被大量相似结构抬高，不能很好区分正确/错误匹配。
```

### 3. 地图和 scan 裁剪 OBB

源码位置：

```cpp
// global_localization.cpp
OBB_map->extent_ = Eigen::Vector3d(60, 60, 40);
OBB_scan->extent_ = Eigen::Vector3d(60, 60, 40);
```

当前值：

```yaml
map_crop_extent: [60, 60, 40]   # 硬编码，当前不是 launch 参数
scan_crop_extent: [60, 60, 40]  # 硬编码，当前不是 launch 参数
```

作用：

```text
OBB_map: 从 prior PCD 中裁当前位置附近的 target 子图。
OBB_scan: 从累计当前点云中裁 source 子图。
```

建议：

```yaml
map_crop_extent: [20.0, 20.0, 8.0]
scan_crop_extent: [20.0, 20.0, 8.0]
```

或者先保守：

```yaml
map_crop_extent: [25.0, 25.0, 8.0]
scan_crop_extent: [25.0, 25.0, 8.0]
```

说明：

```text
20m 是长宽总长度，不是半径；等价于机器人前后左右约 10m。
如果担心走廊长距离墙面特征不足，可以先试 25m。
```

建议把该硬编码改成 launch 参数，后续单独跑 bag 验证。

### 4. 定位频率 loc_frequence

源码位置：

```cpp
this->declare_parameter<double>("loc_frequence", 2.0);
...
if (time_diff_loc < loc_frequence_) sleep(...)
```

当前 launch 值：

```yaml
loc_frequence: 1.0
```

注意：该参数实际是“定位间隔秒数”，不是 Hz。当前等价于约 1Hz。

bag 日志中的定位耗时大致为：

```text
约 120ms ~ 190ms，初始化时约 400ms+
```

分析：

```text
1Hz 会让每次校正间隔较长，机器人每次定位之间走得更远。
如果匹配结果有偏差，map->odom 单次修正量更容易变大。
```

建议：

```yaml
loc_frequence: 0.5   # 约 2Hz，第一轮建议
```

暂不建议直接上 10Hz：

```text
当前 ICP 单次耗时约 120~190ms，10Hz 基本没有余量。
除非先大幅缩小 crop、降低点数、优化滤波，否则 10Hz 不现实。
```

### 5. 点云队列 pcd_queue_maxsize

源码位置：

```cpp
this->declare_parameter<int>("pcd_queue_maxsize", 5);
...
if (que_pcd_scan_.size() >= queue_maxsize_) {
    pcd_scan_cur_->Clear();
    while (!que_pcd_scan_.empty()) {
        *pcd_scan_cur_ += que_pcd_scan_.front();
        ...
    }
    que_pcd_scan_.pop();
}
que_pcd_scan_.push(pcd_recieved);
```

当前 launch 值：

```yaml
pcd_queue_maxsize: 10
```

作用：

```text
累计最近约 10 帧点云作为当前 source。
bag 中 cloud_registered 约 10Hz，因此 10 帧约等于 1 秒局部点云。
```

分析：

```text
队列太小：局部点云稀疏，匹配不稳定。
队列太大：引入更长时间跨度，运动时可能产生拖影/滞后，计算更重。
```

建议：

```yaml
pcd_queue_maxsize: 15
```

第二轮可试：

```yaml
pcd_queue_maxsize: 20
```

不建议第一步直接 30：

```text
30 帧约 3 秒点云。机器人行走或旋转时，累计点云可能更密，但也可能把过旧视角混进去。
如果 cloud_registered 已经在同一 odom/global 坐标中累积，30 帧可能可用；
如果时间同步/姿态有延迟，30 帧会放大拖影。
```

### 6. voxel size 和地图分辨率

源码位置：

```cpp
this->declare_parameter<double>("voxelsize_coarse", 0.2);
this->declare_parameter<double>("voxelsize_fine", 0.05);

pcd_map_coarse_ = pcd_map_ori_->VoxelDownSample(voxelsize_coarse_);
pcd_map_fine_ = pcd_map_ori_->VoxelDownSample(voxelsize_fine_);

source = source->VoxelDownSample(voxelsize_fine_);
```

当前 launch 值：

```yaml
voxelsize_coarse: 0.01
voxelsize_fine: 0.20
```

持续定位实际主要使用：

```yaml
voxelsize_fine: 0.20
```

所以当前用于 ICP 的 source 和 target 都会按 0.20m 降采样：

```text
source scan: VoxelDownSample(0.20)
target map: pcd_map_fine_ = map VoxelDownSample(0.20)
```

结论：

```text
当前匹配时 source 和 map 的 voxel 是一致的，都是 0.20m。
不是 source 0.20、map 其他分辨率导致的直接不匹配。
```

但 `voxelsize_coarse=0.01` 命名和用途不合理：

```text
coarse 反而比 fine 更细。
coarse map 主要用于可视化发布，对当前持续定位影响不大。
```

建议：

```yaml
voxelsize_fine: 0.15
voxelsize_coarse: 0.20
```

先不要直接改到 0.05：

```text
0.05 会显著增加点数和 ICP 计算量。
当前问题更像错误匹配被接受，不一定靠更细 voxel 解决。
```

### 7. ICP 方法、阈值和迭代次数

源码位置：

```cpp
auto reg_result2 = pcd_tools::RegistrationIcp(
    source, target, voxelsize_fine_ * 2, reg_matrix, 1);
```

当前值：

```yaml
icp_method: 1               # point-to-plane，硬编码
icp_iteration: 30           # RegistrationIcp 默认值
icp_distance_threshold: 0.4 # voxelsize_fine 0.20 * 2
```

初始化时 multiscale ICP：

```cpp
RegistrationMultiScaleIcp(source, target, voxelsize_fine_, 1, {1, 2, 3})
```

对应阈值：

```text
0.20 * 1 * 1.3 = 0.26m
0.20 * 2 * 1.3 = 0.52m
0.20 * 3 * 1.3 = 0.78m
```

建议：

```yaml
icp_distance_threshold_ratio: 1.5  # 新增参数，持续定位可从 2.0 收紧到 1.5
icp_iteration: 30                  # 先保持
```

如果 `voxelsize_fine=0.15`，持续 ICP 阈值建议：

```text
0.15 * 1.5 = 0.225m
```

比当前 0.4m 更不容易把错误对应点纳入 ICP。

### 8. fitness / confidence

源码位置：

```cpp
auto eva_result2 = open3d::pipelines::registration::EvaluateRegistration(
    *source, *target, voxelsize_fine_ * 4, reg_matrix);
loc_fitness_ = eva_result2.fitness_;

if (loc_fitness_ > threshold_fitness_) {
    mat_odom2map_ = reg_matrix;
}
```

当前值：

```yaml
threshold_fitness: 0.50
threshold_fitness_init: 0.50
confidence_loc_th: 0.7
```

实际参与持续定位更新的是：

```yaml
threshold_fitness: 0.50
```

`confidence_loc_th` 当前主要在 initialpose callback 中打印/判断使用，不是持续定位的核心门控。

当前 EvaluateRegistration 距离阈值：

```text
voxelsize_fine * 4 = 0.80m
```

分析：

```text
0.80m 的评估对应距离偏宽。
错误局部最优也可能得到较高 fitness。
```

建议：

```yaml
threshold_fitness: 0.65
threshold_fitness_init: 0.65
fitness_eval_distance_ratio: 2.0  # 新增参数，替代当前写死的 *4
```

如果 `voxelsize_fine=0.15`：

```text
评估距离 = 0.15 * 2.0 = 0.30m
```

这会让 confidence 更严格，更能反映真正贴合程度。

### 9. maxpoints_source / maxpoints_target

当前 launch 值：

```yaml
maxpoints_source: 60000
maxpoints_target: 250000
```

作用：

```text
source 或 target 点数超过阈值时，随机降采样到该上限。
```

分析：

```text
如果 crop 从 60m 缩到 20m，target 点数会明显下降。
如果队列从 10 增到 15/20，source 点数会上升。
```

建议第一轮先保持：

```yaml
maxpoints_source: 60000
maxpoints_target: 250000
```

等记录 source/target 实际点数后再决定。

建议新增日志或 topic：

```text
source_raw_points
source_downsampled_points
target_raw_points
target_sampled_points
fitness
inlier_rmse
inlier_count
```

没有这些观测前，不建议盲目把 maxpoints 调大。

### 10. dis_updatemap

源码位置：

```cpp
this->declare_parameter<double>("dis_updatemap", 1);
...
if (dis_motion > dis_updatemap_) {
    *map_fine_crop = *pcd_map_fine_->Crop(*OBB_map);
}
```

当前 launch 值：

```yaml
dis_updatemap: 3.5
```

作用：

```text
机器人相对上次地图裁剪中心移动超过 3.5m，才重新裁剪 target 子图。
```

分析：

```text
在 60m crop 下，3.5m 不明显。
如果 crop 改到 20m，3.5m 偏大，target 子图中心可能滞后。
```

建议：

```yaml
dis_updatemap: 1.0
```

如果定位频率提高到 2Hz，crop 变小，地图更新也应更及时。

### 11. Kalman / filter_odom2map 参数

当前 launch 值：

```yaml
filter_odom2map: false
kf_baselink2map/x: [0.001, 0.002]
kf_baselink2map/y: [0.001, 0.005]
kf_baselink2map/z: [0.00001, 0.04]
kalman_processVar2: 0.001
kalman_estimatedMeasVar2: 0.02
```

作用说明：

```text
kf_baselink2map/x/y/z:
  用于发布 baselink2map_kalman 相关输出。
  当前 bridge 主输入不是 kalman 输出，因此它们不能防止 map->odom 跳变。

filter_odom2map:
  如果为 true，源码只对 mat_odom2map 的 z 方向做了 Kalman。
  没有平滑 x/y/yaw。

kalman_processVar2 / kalman_estimatedMeasVar2:
  只在 filter_odom2map=true 时参与 z 方向滤波。
```

结论：

```text
当前这些 Kalman 参数对 x/y/yaw 的 1m 级跳变没有保护作用。
不建议靠打开 filter_odom2map 来解决当前问题。
应该在 bridge 层做 x/y/yaw 硬门控和输出限速。
```

### 12. 地面点、近距离点、最大距离、机器人自身点云

当前 open3d_loc 本体没有明确实现以下处理：

```text
地面点剔除
墙面点选择
近距离点剔除
最大距离过滤
机器人自身点云切除
```

当前有的处理主要是：

```text
RemoveNonFinitePoints
OBB Crop
VoxelDownSample
EstimateNormals
Point-to-plane ICP
```

建议新增预处理，优先放在 `fastlio_open3d_axis_adapter` 或 open3d_loc 入队前：

```yaml
enable_range_filter: true
min_range: 0.6
max_range: 18.0

enable_self_filter: true
self_filter_min_x: -0.6
self_filter_max_x: 0.8
self_filter_min_y: -0.5
self_filter_max_y: 0.5
self_filter_min_z: -0.2
self_filter_max_z: 1.5

enable_height_filter: true
min_z: 0.15
max_z: 2.5
```

地面点建议先用简单 z 阈值剔除：

```text
去掉 z < 0.15m 的点，减少地面平面对 ICP fitness 的干扰。
```

最大距离建议先从 18m 开始：

```text
走廊远端点可能重复且易误配，过远点不一定有利。
```

机器人自身点云必须考虑：

```text
如果当前 scan 包含机身、腿部、外壳或雷达支架点，这些点在 prior map 中不存在，
会污染 ICP 或降低真实对应比例。
```

### 13. 基于当前环境的建议实验顺序

不要一次性改所有 open3d_loc 参数。建议分层测试：

#### A. 先改 bridge，验证能否挡住坏候选

```text
真实 map->odom 单步硬门控
SpinToPose 解冻缓启动
```

这是当前最直接、风险最低的修改。

#### B. 再改 open3d_loc 的局部匹配范围和频率

建议第一轮 open3d_loc 参数：

```yaml
map_crop_extent: [25.0, 25.0, 8.0]
scan_crop_extent: [25.0, 25.0, 8.0]
loc_frequence: 0.5
pcd_queue_maxsize: 15
dis_updatemap: 1.0
voxelsize_fine: 0.20
threshold_fitness: 0.60
fitness_eval_distance_ratio: 3.0
```

说明：

```text
先不动 voxel，避免点数和计算量同时变化。
先把 60m crop 缩小，增加频率，地图更新更及时。
```

#### C. 再测试点云预处理

第二轮再加：

```yaml
min_range: 0.6
max_range: 18.0
min_z: 0.15
max_z: 2.5
self_filter_box: enabled
```

验证：

```text
fitness 是否下降过多
accepted 大修正是否减少
目标点完成情况是否变差
```

#### D. 最后再考虑更细 voxel

如果 CPU 允许，再试：

```yaml
voxelsize_fine: 0.15
icp_distance_threshold_ratio: 1.5
fitness_eval_distance_ratio: 2.0
```

验证重点：

```text
定位耗时是否仍小于 loc_frequence
错误 accepted correction 是否减少
置信度是否更能区分好坏匹配
```
