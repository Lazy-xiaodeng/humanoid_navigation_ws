# NDT 漂移根治技术修改方案

日期：2026-05-27  
背景 bag：`/home/ubuntu/nav_drift_test3/nav_drift_test3_0.mcap`  
相关现象：大厅原点区域 NDT 稳定；进入上方实验室房间后，尤其最里面两个拐角容易漂移，点位 8-9 附近出现严重跳变。

## 1. 目标

当前 fusion/rusion 接管、停车、HDL/SC recovery 属于“定位漂移后的补救”。本方案目标是降低 NDT 漂移发生概率，并防止 NDT 在弱几何场景下错误接受漂移位姿。

目标不是让 NDT 在所有场景永不失败，而是：

```text
1. 正常场景下 NDT 更稳定。
2. 实验室拐角/长廊弱几何场景下，NDT 不轻易被错误局部最优吸走。
3. 如果 NDT 不可观测，宁可拒绝，也不要错误 accepted。
4. 如果必须恢复，必须通过受控 recovery，而不是运行期自动 confirmed_pose_jump。
```

## 2. 当前问题总结

### 2.1 Fast-LIO 输入不是普通局部 scan

当前 NDT 输入：

```text
/fast_lio/cloud_registered
```

该话题不是原始 LiDAR 局部 scan，而是 Fast-LIO 已经配准到其世界系后的点云。

当前 NDT 代码对 PCD 和实时点云都做了相同旋转：

```cpp
Eigen::Quaternionf q_cam_to_ros(0.5, -0.5, -0.5, 0.5);
pcl::transformPointCloud(*cloud_ptr, *cloud_ptr, t_cam_to_ros, q_cam_to_ros);
```

所以 source 和 target 的坐标轴方向是一致的，但它们的语义不是“局部 scan 点云”。

### 2.2 `scan_max_range` 当前语义不正确

当前距离过滤：

```cpp
r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
```

这个对局部 scan 是合理的，因为 `p.x/p.y` 是点相对雷达的位置。

但对 `/fast_lio/cloud_registered` world cloud 来说，这变成了：

```text
点到 map/odom 原点的距离
```

而不是：

```text
点到机器人当前位姿的距离
```

因此不能直接把：

```yaml
scan_max_range: 15.0
```

用于当前代码，否则机器人远离原点后会误删有效点。

### 2.3 Fast-LIO odom 不能直接用于 `use_odom`

Fast-LIO 输出坐标系是非标准的：

```text
x 朝左
y 朝下
z 朝后
```

当前 NDT 的 `odomReceived()` 按 ROS 标准 twist 积分：

```cpp
msg->twist.twist.linear.x
msg->twist.twist.linear.y
msg->twist.twist.angular.z
```

如果直接设置：

```yaml
use_odom: true
```

会把非标准 odom 当标准坐标使用，导致 init guess 方向错误。

### 2.4 当前是单分辨率 NDT

当前代码只初始化一个 `registration_`：

```text
registration_ = ndt_omp
```

只有一个：

```yaml
ndt_resolution
```

没有 coarse-to-fine 多分辨率流程。实验室拐角/长廊这种多局部最优场景，单分辨率 NDT 更容易落入错误局部最优。

### 2.5 `pose_jump_reacquire` 会确认低 fitness 错跳

test3 参数扫测发现：

```text
大跳变 accepted 大多来自 confirmed_pose_jump。
这些 confirmed_pose_jump 的 fitness 很低，通常 0.002 ~ 0.013。
```

这说明：

```text
fitness 低不一定代表位置正确。
在长廊/拐角重复结构中，错误位置也可能有很低 fitness。
```

因此，运行期不能只靠低 fitness 自动确认 pose jump。

## 3. 参数扫测结论

回放窗口：

```text
bag: /home/ubuntu/nav_drift_test3/nav_drift_test3_0.mcap
start-offset: 135s
duration: 95s
topics: /fast_lio/cloud_registered
initial pose: x=5.756, y=10.765, yaw≈75.6deg
```

关键结果：

| 参数组 | accepted | rejected | pose_jump | high_fitness | 最大 accepted 跳变 | 结论 |
|---|---:|---:|---:|---:|---:|---|
| current_like | 262 | 101 | 71 | 30 | 0.691m | 当前近似配置仍会确认大跳变 |
| res04 | 222 | 119 | 87 | 32 | 0.797m | 0.4m 更细反而更差 |
| res06 | 343 | 40 | 31 | 9 | 0.783m | 匹配稳定性改善，但仍有 confirmed jump |
| score015 | 366 | 105 | 46 | 59 | 0.785m | 降低 fitness 阈值不能解决低 fitness 错跳 |
| strict_jump | 303 | 200 | 150 | 50 | 0.785m | 只收紧普通 jump gate 不够 |
| res06_no_reacquire | 148 | 109 | 109 | 0 | 0.389m | 最安全，但 accepted 太少 |
| res06_reacquire_strict | 239 | 188 | 161 | 27 | 0.397m | 推荐短期折中 |

短期推荐参数：

```yaml
ndt_resolution: 0.6
voxel_leaf_size: 0.18
ndt_step_size: 0.08
ndt_max_iterations: 60
score_threshold: 0.3
scan_min_range: 0.0
scan_max_range: 100.0

reject_pose_jump: true
max_pose_jump_translation: 0.4
max_pose_jump_yaw: 0.3

pose_jump_reacquire_enabled: true
pose_jump_reacquire_max_translation: 0.5
pose_jump_reacquire_max_yaw: 0.2
pose_jump_reacquire_max_fitness: 0.05
pose_jump_reacquire_required_frames: 5
pose_jump_reacquire_xy_tolerance: 0.3
pose_jump_reacquire_yaw_tolerance: 0.15
```

该参数不是根治，只是短期降低错误 accepted 风险。

## 4. 根治方案总览

推荐按以下顺序实施：

```text
P0: 上线短期安全参数，减少 confirmed_pose_jump 错误放行。
P1: 修正 range filter 语义，让 scan range 按机器人当前位置裁剪。
P2: 增加 Fast-LIO delta guess，替代 use_odom。
P3: 修改 pose_jump_reacquire 策略，只允许 recovery/initialpose 后 reacquire。
P4: 实现多分辨率 NDT coarse-to-fine。
P5: 补扫/重建实验室最里面两个拐角。
P6: 增加 SC/HDL/GICP 候选验证，处理相似拐角和长廊歧义。
```

## 5. P0：短期安全参数

### 5.1 修改目标

先不改代码，仅修改 launch 覆盖参数，使 NDT 不轻易确认低 fitness 大跳变。

### 5.2 修改位置

优先修改当前实际使用的 launch：

```text
src/humanoid_navigation2/launch/navigation2_fusion_sc.launch.py
```

如果使用 v2：

```text
src/humanoid_navigation2/launch/navigation2_fusion_sc_v2.launch.py
```

### 5.3 推荐参数

```python
{
    'score_threshold': 0.3,
    'ndt_resolution': 0.6,
    'voxel_leaf_size': 0.18,
    'ndt_step_size': 0.08,
    'ndt_max_iterations': 60,

    'scan_min_range': 0.0,
    'scan_max_range': 100.0,

    'reject_pose_jump': True,
    'max_pose_jump_translation': 0.40,
    'max_pose_jump_yaw': 0.30,

    'pose_jump_reacquire_enabled': True,
    'pose_jump_reacquire_max_translation': 0.50,
    'pose_jump_reacquire_max_yaw': 0.20,
    'pose_jump_reacquire_max_fitness': 0.05,
    'pose_jump_reacquire_required_frames': 5,
    'pose_jump_reacquire_xy_tolerance': 0.30,
    'pose_jump_reacquire_yaw_tolerance': 0.15,
}
```

### 5.4 风险

```text
rejected 增多。
短时间内 /pcl_pose 可能少发。
必须依赖 fusion/rusion 做 TF 连续和停车保护。
```

### 5.5 验收

同样 test3 回放窗口下：

```text
accepted correction > 0.4m 数量应接近 0。
pose frame jump > 0.4m 数量应接近 0。
confirmed_pose_jump 不应频繁出现。
```

## 6. P1：修正 range filter 语义

### 6.1 修改目标

让 `scan_min_range/scan_max_range` 表示：

```text
点到机器人当前位姿的距离
```

而不是：

```text
点到 map/odom 原点的距离
```

### 6.2 当前代码

文件：

```text
src/lidar_localization/src/lidar_localization_component.cpp
```

当前逻辑：

```cpp
for (const auto & p : filtered_cloud_ptr->points) {
  r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
  if (scan_min_range_ < r && r < scan_max_range_) {
    tmp.push_back(p);
  }
}
```

### 6.3 建议改造

新增参数：

```yaml
range_filter_mode: "origin"            # 兼容旧逻辑
range_filter_mode: "relative_to_pose"  # 新逻辑
```

代码逻辑：

```cpp
double origin_x = 0.0;
double origin_y = 0.0;

if (range_filter_mode_ == "relative_to_pose") {
  origin_x = corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x;
  origin_y = corrent_pose_with_cov_stamped_ptr_->pose.pose.position.y;
}

for (const auto & p : filtered_cloud_ptr->points) {
  const double dx = static_cast<double>(p.x) - origin_x;
  const double dy = static_cast<double>(p.y) - origin_y;
  const double r = std::hypot(dx, dy);
  if (scan_min_range_ < r && r < scan_max_range_) {
    tmp.push_back(p);
  }
}
```

### 6.4 推荐参数

修正后：

```yaml
range_filter_mode: "relative_to_pose"
scan_min_range: 0.5
scan_max_range: 20.0
min_scan_points: 80
```

测试组：

```text
A: scan_max_range = 15.0
B: scan_max_range = 20.0
C: scan_max_range = 25.0
```

### 6.5 风险

```text
如果 current pose 已经漂移，用漂移 pose 做 range filter 可能裁错局部点云。
```

缓解：

```text
只在 HEALTHY/正常 tracking 时使用 relative_to_pose。
在 recovery/reacquire 时可临时放大 scan_max_range 或回退 origin 模式。
```

### 6.6 验收

记录 `filtered_points`：

```text
实验室拐角 filtered_points 不应低于 min_scan_points。
filtered_points 不应在远离原点后异常掉到很低。
```

同时观察：

```text
pose_jump 减少。
high_fitness 不显著增加。
correction_translation p90/p99 下降。
```

## 7. P2：Fast-LIO delta guess

### 7.1 修改目标

用 Fast-LIO 短时相对运动给 NDT 初值，但不直接使用非标准 odom twist。

禁止直接：

```yaml
use_odom: true
```

改为新增：

```yaml
use_fastlio_delta_guess: true
```

### 7.2 数据来源

从 TF 查询：

```text
camera_init -> body
```

这是 Fast-LIO 短时里程计姿态。

### 7.3 基本流程

```text
1. 每帧点云到达时，查询当前 T_camera_init_body。
2. 保存上一帧 T_camera_init_body。
3. 计算 T_delta = inverse(T_prev) * T_now。
4. 将 T_delta 转换到 ROS 标准平面坐标。
5. 把 delta 应用到当前 NDT pose，作为 init_guess。
6. 再执行 NDT align。
```

### 7.4 伪代码

```cpp
if (use_fastlio_delta_guess_) {
  auto T_now = lookupTransform(fastlio_odom_parent_frame_, fastlio_odom_child_frame_);

  if (has_prev_fastlio_pose_) {
    Eigen::Matrix4f T_delta = prev_T_now.inverse() * T_now;
    Eigen::Matrix4f T_delta_ros = fastlioDeltaToRosPlanar(T_delta);

    Eigen::Matrix4f pose_guess = current_pose_matrix * T_delta_ros;
    init_guess = pose_guess;
  }

  prev_fastlio_pose_ = T_now;
  has_prev_fastlio_pose_ = true;
}
```

### 7.5 参数

```yaml
use_fastlio_delta_guess: true
fastlio_odom_parent_frame: "camera_init"
fastlio_odom_child_frame: "body"
fastlio_delta_max_translation: 0.5
fastlio_delta_max_yaw: 0.35
fastlio_delta_timeout_sec: 0.2
```

### 7.6 坐标转换注意

Fast-LIO 坐标轴：

```text
x 左
y 下
z 后
```

ROS 标准平面：

```text
x 前
y 左
z 上
```

当前代码使用的固定旋转：

```cpp
Eigen::Quaternionf q_cam_to_ros(0.5, -0.5, -0.5, 0.5);
```

建议复用同一个旋转，把 Fast-LIO delta 转到 ROS 标准方向。

### 7.7 风险

```text
如果 TF 查询时间和点云时间不同步，delta 可能有时间误差。
如果 Fast-LIO 在局部也退化，delta guess 会带偏 NDT。
```

缓解：

```text
设置 delta 最大平移/最大 yaw。
超过阈值丢弃 delta。
NDT rejected 时不更新 prev delta 或按状态机控制。
```

### 7.8 验收

对比 P1 与 P2：

```text
correction_translation p90/p99 应下降。
pose_jump_candidate 应减少。
NDT align iteration/time 不应明显增加。
实验室拐角不应出现更大的错误 accepted。
```

## 8. P3：修改 pose_jump_reacquire 策略

### 8.1 问题

当前运行期 `pose_jump_reacquire` 会把连续低 fitness 的跳变确认为：

```text
confirmed_pose_jump
```

test3 中大跳变就是这样被放行的。

### 8.2 修改目标

运行期不允许 NDT 自己确认大跳变并接管位姿。只有以下情况允许 reacquire：

```text
1. 收到 /initialpose 后的 reacquire window。
2. fusion/rusion 明确处于 RECOVERY_VALIDATING。
3. recovery_arbiter 发布了候选并等待 NDT 验证。
```

### 8.3 新增参数

```yaml
pose_jump_reacquire_mode: "recovery_only"
```

可选值：

```text
always          当前行为，运行期也允许确认 pose jump
disabled        完全关闭
recovery_only   只在 /initialpose 或 recovery 验证期间开启
```

### 8.4 推荐逻辑

```cpp
bool reacquire_allowed = false;

if (pose_jump_reacquire_mode_ == "always") {
  reacquire_allowed = true;
} else if (pose_jump_reacquire_mode_ == "recovery_only") {
  reacquire_allowed = initialPoseReacquireActive() || recovery_validation_active_;
} else {
  reacquire_allowed = false;
}

if (!reacquire_allowed) {
  reject pose jump;
}
```

### 8.5 推荐参数

```yaml
pose_jump_reacquire_mode: "recovery_only"
pose_jump_reacquire_max_translation: 0.5
pose_jump_reacquire_max_yaw: 0.2
pose_jump_reacquire_max_fitness: 0.05
pose_jump_reacquire_required_frames: 5
pose_jump_reacquire_xy_tolerance: 0.3
pose_jump_reacquire_yaw_tolerance: 0.15
```

### 8.6 验收

运行期：

```text
confirmed_pose_jump 应接近 0。
大 correction 不应 accepted。
```

recovery 后：

```text
initialpose 后 NDT 仍可重新捕获。
连续 accepted 后 fusion 才恢复 HEALTHY。
```

## 9. P4：多分辨率 NDT

### 9.1 修改目标

用 coarse-to-fine 降低局部最优风险。

当前：

```text
single NDT resolution = 0.5 或 0.6
```

建议：

```text
coarse NDT resolution = 1.0
fine NDT resolution = 0.6
```

### 9.2 新增成员

```cpp
pcl::Registration<PointT, PointT>::Ptr coarse_registration_;
pcl::Registration<PointT, PointT>::Ptr fine_registration_;
```

或专门类型：

```cpp
pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr coarse_ndt_;
pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr fine_ndt_;
```

### 9.3 流程

```text
1. 对输入点云做一次预处理。
2. coarse source 使用较大 voxel。
3. coarse target 使用较大 resolution。
4. coarse align(init_guess)。
5. coarse 结果通过基本 gate 后，作为 fine init。
6. fine align(coarse_result)。
7. fine 结果走最终 NDT gates。
```

### 9.4 参数

```yaml
multi_resolution_ndt: true

coarse_ndt_resolution: 1.0
coarse_voxel_leaf_size: 0.25
coarse_ndt_max_iterations: 25
coarse_score_threshold: 0.8

fine_ndt_resolution: 0.6
fine_voxel_leaf_size: 0.18
fine_ndt_max_iterations: 50
fine_score_threshold: 0.3

multi_ndt_max_total_align_time_ms: 80
```

### 9.5 风险

```text
CPU 占用上升。
定位频率可能下降。
如果 coarse 错了，fine 会跟着错。
```

缓解：

```text
coarse 结果只作为候选，不无条件接受。
对 coarse correction 设置上限。
记录 align time。
超时则回退 single fine NDT 或拒帧。
```

### 9.6 验收

```text
align time p90 < 80ms。
pose_jump_candidate 减少。
实验室拐角不出现低 fitness 大跳变。
```

## 10. P5：实验室拐角补扫与地图质量修复

### 10.1 为什么需要

如果局部 PCD 本身缺结构、重影、动态物体污染，NDT 参数和代码只能降低错误概率，不能让错误地图变正确。

### 10.2 检查方法

对最里面两个易漂拐角：

```text
1. 在 CloudCompare/RViz 打开 hall.pcd。
2. 标记拐角中心坐标。
3. 半径 2m/3m 统计点数和墙面连续性。
4. 检查是否有双墙、重影、缺角。
5. 把 test3 漂移前后的 cloud_registered 叠到 PCD。
```

### 10.3 重建建议

```text
1. 每个拐角原地慢速转一圈。
2. 从两个方向进出拐角。
3. 保持门、桌椅、设备状态和导航时一致。
4. 尽量减少人员走动。
5. 不只沿走廊中心线扫一遍。
```

### 10.4 验收

```text
重新建图后，同一路线连续跑 3 次。
实验室拐角 correction_translation 不应突然超过 0.4m。
不应出现 confirmed_pose_jump。
```

## 11. P6：候选验证与全局辅助

### 11.1 目标

处理长廊/相似拐角中的低 fitness 错误候选。

### 11.2 推荐流程

```text
NDT tracking 异常
  -> fusion/rusion blocked
  -> SC top-K candidates
  -> HDL/GICP 几何验证
  -> NDT initialpose 验证
  -> fusion 恢复
```

### 11.3 为什么不是只靠 NDT

NDT 是局部优化，依赖初值。长廊/拐角重复结构中，它可能在错误局部最优处得到低 fitness。

SC/HDL/GICP 的作用：

```text
SC: 快速召回候选。
HDL/GICP: 几何验证候选。
NDT: 最终 tracking 验收。
```

## 12. 分阶段实施计划

### Phase A：参数先行

改动：

```text
ndt_resolution 0.5 -> 0.6
voxel_leaf_size 0.15 -> 0.18
step_size 0.1 -> 0.08
iterations 50 -> 60
reacquire 参数收紧
```

目标：

```text
降低 confirmed_pose_jump 错误放行。
```

### Phase B：range filter 修正

改动：

```text
新增 range_filter_mode。
实现 relative_to_pose。
```

目标：

```text
让 scan_max_range 真正可调。
减少远处/门外/重复结构干扰。
```

### Phase C：Fast-LIO delta guess

改动：

```text
新增 use_fastlio_delta_guess。
用 camera_init->body TF 计算标准平面 delta。
```

目标：

```text
让 NDT 初值更接近真实位姿。
降低局部最优吸附。
```

### Phase D：reacquire 状态机重构

改动：

```text
pose_jump_reacquire_mode: recovery_only
运行期 pose jump 不自动确认。
```

目标：

```text
杜绝低 fitness 错跳在运行期被 accepted。
```

### Phase E：多分辨率 NDT

改动：

```text
coarse + fine NDT。
```

目标：

```text
进一步增强弱几何场景鲁棒性。
```

### Phase F：地图补扫与验证

改动：

```text
补扫实验室最里面两个拐角。
替换或生成新的 hall.pcd。
```

目标：

```text
从地图质量上减少错误匹配。
```

## 13. 验证指标

每次修改都用同一个 test3 bag 窗口和现场路线验证。

### 13.1 离线 bag 指标

```text
accepted count
rejected count
confirmed_pose_jump count
pose_jump_candidate count
high_fitness count
fitness p50/p90/p99/max
correction_translation p50/p90/p99/max
pose frame jump p90/p99/max
filtered_points min/p50
align time p50/p90/p99
```

### 13.2 通过条件

```text
confirmed_pose_jump ≈ 0 或只在 recovery 后出现。
accepted correction > 0.4m 接近 0。
pose frame jump > 0.4m 接近 0。
实验室拐角不飞出地图。
Nav2 不因定位跳变失败。
```

### 13.3 现场验证

路线：

```text
大厅原点 -> 实验室入口 -> 最里面两个拐角 -> 点位 8-9
```

每组至少跑 3 次。

通过：

```text
三次均不出现定位飞出地图。
拐角处 NDT 不确认大跳变。
fusion/rusion 不进入长时间 recovery。
```

## 14. 风险与边界

| 风险 | 说明 | 缓解 |
|------|------|------|
| 参数收紧导致拒帧增多 | NDT 宁可不发也不接受错误 | fusion/rusion 负责短时 TF 连续和停车 |
| relative_to_pose 用漂移 pose 裁点 | pose 已错时 range filter 可能错 | 仅 HEALTHY 使用；recovery 放大范围 |
| Fast-LIO delta 带偏 NDT | Fast-LIO 局部也可能退化 | delta 最大值 gate，异常丢弃 |
| 多分辨率 CPU 增加 | 两次 align 更耗时 | 记录 align time，必要时降低频率 |
| 地图补扫成本高 | 需要重新采集现场 | 先只补扫实验室拐角局部 |
| SC/HDL 增加系统复杂度 | 节点和状态更多 | 由 recovery_arbiter/SC bridge 统一仲裁 |

## 15. 最终推荐

短期先落地：

```text
P0 参数：
  ndt_resolution=0.6
  voxel_leaf_size=0.18
  strict reacquire

fusion/rusion:
  非健康立即 blocked + zero cmd
```

中期必须做：

```text
range_filter_mode=relative_to_pose
use_fastlio_delta_guess=true
pose_jump_reacquire_mode=recovery_only
```

长期增强：

```text
multi_resolution_ndt
实验室拐角补扫
SC top-K + HDL/GICP 验证
```

真正根治的判断标准：

```text
不是 NDT 永远不 rejected，
而是 NDT 在弱几何场景不会错误 accepted；
系统能稳定拒绝、停车、恢复，而不是飞出地图。
```
