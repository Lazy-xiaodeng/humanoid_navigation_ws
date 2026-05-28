# NDT Fast-LIO Delta Guess 问题复盘与重构方案

日期：2026-05-28  
工作区：`/home/ubuntu/humanoid_ws`  
状态：当前 Plan B 已被 bag A/B 测试证实会破坏正常定位；建议先关闭，再按本文方案重构为实验项

## 1. 结论摘要

当前实现的核心问题不是“有没有使用 `camera_init -> body` 的变换量”，而是**把这个 body 运动增量加错了对象**。

当前代码计算的是：

```text
camera_init -> body 的相邻帧运动 delta
```

但实际写回的是：

```text
corrent_pose / NDT init_guess / 最终发布的 map -> odom
```

在当前系统配置下：

```text
global_frame_id = map
base_frame_id   = odom
```

所以 `corrent_pose_with_cov_stamped_ptr_` 的语义是：

```text
map -> odom
```

`camera_init -> body` 表示机器人在 Fast-LIO 局部世界里的运动，机器人前进 1m 时它应当变化 1m；但 `map -> odom` 是地图系和里程计系之间的校正关系，不应该跟着机器人自身运动每帧前进。

因此当前逻辑等价于：

```text
map->odom += body_delta
```

这是语义错误，会把机器人运动混入全局校正项，导致 NDT 初始值被持续带偏。

## 2. 已验证现象

### 2.1 test1 旧地图 A/B

为了排除“旧 bag 配新地图”的影响，使用录制 test1 时对应的旧地图重新回放。

旧地图路径：

```text
/home/ubuntu/地图文件存放处/5-10天津展厅图/pcd/hall.pcd
```

同一 bag、同一旧地图、同一前 180 秒窗口，仅切换 `use_fastlio_delta_guess`：

| 条件 | accepted | rejected | accept rate | first_bad |
|---|---:|---:|---:|---|
| 旧地图 + delta off | 1232 | 0 | 100.0% | 无 |
| 旧地图 + delta on | 485 | 581 | 45.5% | 51.2s high_fitness |

delta on 主要失败项：

```text
high_fitness: 577
pose_jump: 4
max consecutive rejected: 576
fastlio_delta_applied: 496
dead_reckon_timeout: 567
first_bad: t=51.2s, high_fitness, fitness=0.331334, fastlio_delta_applied=true
```

判断：

```text
旧地图 + delta off 可正常匹配
旧地图 + delta on 快速失败
```

因此 test1 前段失败不能归因于地图不一致，当前 delta guess 逻辑本身会破坏 NDT。

### 2.2 test4 回放

当前 Plan B 在 test4 上表现为：

```text
pose_jump 数量下降，但 high_fitness 明显增加
accept rate 从约 65.1% 降到约 51.4%
```

判断：

```text
当前方案不是稳定提升定位率，而是把一部分 pose_jump 问题转化成 high_fitness 问题。
```

## 3. 当前实现的具体问题位置

主要问题位置：

```text
src/lidar_localization/src/lidar_localization_component.cpp
```

当前 Plan B 逻辑位于 `cloudReceived()` 内，约在：

```text
Plan B: Fast-LIO delta guess
```

关键错误逻辑是：

```cpp
corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x += dx_ros;
corrent_pose_with_cov_stamped_ptr_->pose.pose.position.y += dy_ros;

double current_yaw = tf2::getYaw(
  corrent_pose_with_cov_stamped_ptr_->pose.pose.orientation);
double new_yaw = current_yaw + dyaw;
```

这段代码把 `camera_init -> body` 的相邻帧运动增量直接叠加到了 `map -> odom`。

## 4. 设计边界重新定义

Fast-LIO delta 不能被定义为“修正 NDT 的定位结果”。

它最多只能作为：

```text
短时间 NDT init_guess 预测项
```

不能作为：

```text
最终定位真值
全局 correction
长期 dead reckoning
直接发布的 TF
```

如果 NDT 已经漂移、Fast-LIO 已经跳变、地图不匹配或处于长期退化区域，继续叠加预测会形成错误闭环：

```text
错误 Fast-LIO delta
→ 错误 init_guess
→ NDT 错误收敛或 high_fitness
→ 错误结果被接受或连续拒绝
→ 下一帧继续基于错误状态预测
```

因此新方案必须保持保守边界：

```text
只在上一次 NDT accepted 后短窗口内使用
只作为 init_guess
NDT accepted 后必须由 NDT 结果覆盖预测
NDT rejected 后不能发布新结果
超时或异常后立即停止推进并等待 recovery
```

## 5. 短期修复方案：关闭当前 delta guess

### 5.1 修改内容

在导航 launch 中关闭当前错误实现：

```python
'use_fastlio_delta_guess': False,
```

对应位置：

```text
src/humanoid_navigation2/launch/navigation2_fusion_sc_v2.launch.py
```

保留以下机制：

```python
'republish_last_good_tf_on_failure': True,
'max_last_good_tf_age_sec': 5.0,
'pose_jump_reacquire_enabled': True,
```

### 5.2 预期效果

优点：

```text
立即消除当前 Plan B 对正常 NDT 匹配的破坏
避免 test1 中 51s high_fitness 的早期失败
风险低，改动小，可快速回归稳定基线
```

缺点：

```text
不能解决长时间 NDT reject 后 init_guess 冻结的问题
只能依赖 last_good_tf、pose_jump reacquire、ScanContext/HDL recovery 恢复
```

### 5.3 适用场景

短期上车、需要先恢复导航稳定性时，推荐使用该方案。

## 6. 中期重构方案：map_body_to_map_odom 模式

### 6.1 核心原则

不要再做：

```text
map->odom += body_delta
```

应改为：

```text
用 Fast-LIO delta 预测 map->body
再结合当前 odom->body 反推 map->odom guess
```

公式：

```text
T_map_body_prev  = T_map_odom_prev * T_odom_body_prev
T_map_body_guess = T_map_body_prev * T_delta_fastlio
T_map_odom_guess = T_map_body_guess * inverse(T_odom_body_curr)
```

其中：

```text
T_map_odom_prev   = 当前 corrent_pose
T_odom_body_prev  = 上一帧 odom -> body
T_odom_body_curr  = 当前帧 odom -> body
T_delta_fastlio   = Fast-LIO 相邻帧 body 运动 delta
```

最终写回的是：

```text
T_map_odom_guess
```

它只用于 NDT `init_guess`。

### 6.2 为什么这个公式更合理

当前 TF 链：

```text
map -> odom -> camera_init -> body
```

如果想获得机器人在 map 下的预测位姿，需要先构造：

```text
map -> body
```

而不是直接改变：

```text
map -> odom
```

预测完成后，NDT 节点仍然需要输出 `map -> odom`，所以再用当前 `odom -> body` 反推：

```text
map->odom = map->body * inverse(odom->body)
```

这样 body 自身运动不会直接污染 map/odom 的全局校正项。

## 7. 具体代码修改方案

### 7.1 新增参数

建议不要继续复用旧语义，新增模式参数：

```cpp
declare_parameter("fastlio_delta_guess_mode", "disabled");
```

可选值：

```text
disabled
map_body_to_map_odom
```

或保留原布尔参数但改变含义：

```cpp
use_fastlio_delta_guess = false
```

建议最终使用显式 mode，避免以后无法区分旧逻辑和新逻辑。

### 7.2 头文件新增状态

文件：

```text
src/lidar_localization/include/lidar_localization/lidar_localization_component.hpp
```

建议在 Fast-LIO delta 相关成员附近新增：

```cpp
bool has_prev_fastlio_body_ros_{false};
bool has_prev_odom_body_pose_{false};

Eigen::Affine3d prev_T_fastlio_body_ros_{Eigen::Affine3d::Identity()};
Eigen::Affine3d prev_T_odom_body_{Eigen::Affine3d::Identity()};
```

如果不希望在头文件里增加 Eigen 状态，也可以存 `geometry_msgs::msg::TransformStamped`，但矩阵形式更不容易写错乘法。

### 7.3 抽出函数

建议把当前 `cloudReceived()` 里的 Plan B 逻辑抽成函数：

```cpp
bool PCLLocalization::applyFastlioDeltaGuess(const rclcpp::Time & cloud_stamp);
```

职责：

```text
读取 Fast-LIO TF
读取 odom->body TF
计算 delta
检查异常
生成 map->odom init_guess
写回 corrent_pose
维护 debug 字段
维护上一帧基准
```

`cloudReceived()` 主流程只保留：

```cpp
resetFastlioDebugFields();

if (use_fastlio_delta_guess_ && corrent_pose_with_cov_stamped_ptr_) {
  applyFastlioDeltaGuess(rclcpp::Time(msg->header.stamp));
}

tf2::fromMsg(corrent_pose_with_cov_stamped_ptr_->pose.pose, affine);
Eigen::Matrix4f init_guess = affine.matrix().cast<float>();
```

### 7.4 Fast-LIO delta 计算方式

不要手写：

```cpp
dx_ros = -dz_cam;
dy_ros = dx_cam;
```

建议把完整位姿先转换到 ROS 标准系，再做相对变换：

```cpp
Eigen::Affine3d T_fastlio_body_curr_ros = convertFastlioBodyToRos(tf_body_in_cam);
Eigen::Affine3d T_delta_fastlio =
  prev_T_fastlio_body_ros_.inverse() * T_fastlio_body_curr_ros;
```

这样平移和旋转使用同一套坐标转换，避免只转换平移/yaw 而忽略完整旋转关系。

`convertFastlioBodyToRos()` 需要使用当前系统已经验证过的相同旋转：

```text
q_cam_to_ros = (w=0.5, x=-0.5, y=-0.5, z=0.5)
```

### 7.5 odom->body 查询

在每帧 delta 计算时查询：

```cpp
tf_odom_body_curr = tfbuffer_.lookupTransform(
  odom_frame_id_, fastlio_body_frame_, cloud_stamp);
```

如果按 `cloud_stamp` 查不到，可以降级：

```cpp
tfbuffer_.lookupTransform(
  odom_frame_id_, fastlio_body_frame_, tf2::TimePointZero);
```

但必须检查：

```text
abs(tf_stamp - cloud_stamp) <= tf_max_stamp_mismatch_sec
```

并捕获 clock type mismatch。

### 7.6 map->odom guess 生成

核心伪代码：

```cpp
Eigen::Affine3d T_map_odom_prev;
tf2::fromMsg(corrent_pose_with_cov_stamped_ptr_->pose.pose, T_map_odom_prev);

Eigen::Affine3d T_map_body_prev =
  T_map_odom_prev * prev_T_odom_body_;

Eigen::Affine3d T_map_body_guess =
  T_map_body_prev * T_delta_fastlio;

Eigen::Affine3d T_map_odom_guess =
  T_map_body_guess * T_odom_body_curr.inverse();

corrent_pose_with_cov_stamped_ptr_->pose.pose =
  tf2::toMsg(T_map_odom_guess);

applyPlanarPoseConstraint(corrent_pose_with_cov_stamped_ptr_->pose.pose);
```

### 7.7 accepted 后处理

NDT accepted 后仍然必须用 NDT 结果覆盖 `corrent_pose`：

```cpp
corrent_pose = navigation_transformation;
last_accept_time_ = this->now();
```

不要把 Fast-LIO 预测结果当成最终结果发布。

### 7.8 rejected 后处理

NDT rejected 后：

```text
不发布 NDT result
不更新 last_good_transform
不认为定位恢复
```

是否继续短时间推进 init_guess，取决于：

```text
this->now() - last_accept_time_ <= fastlio_max_dead_reckon_sec
```

超过窗口：

```text
停止推进
等待 recovery / initialpose / 全局重定位
```

### 7.9 重置点

以下位置必须同时重置：

```cpp
has_prev_fastlio_body_ros_ = false;
has_prev_odom_body_pose_ = false;
```

重置场景：

```text
/initialpose
mapReceived
Fast-LIO TF lookup failed
odom->body TF lookup failed
tf_stamp_mismatch
tf_clock_mismatch
max_delta_exceeded
dead_reckon_timeout
参数变更
生命周期 deactivate/cleanup
```

已有位置：

```text
src/lidar_localization/src/lidar_localization_component.cpp
initialPoseReceived()
mapReceived()
```

还需要在异常分支补齐。

## 8. 安全门控

### 8.1 时间窗口

保留：

```text
fastlio_max_dead_reckon_sec = 2.0
```

含义：

```text
距离上一次 NDT accepted 超过 2 秒后，不再使用 Fast-LIO delta 推进 init_guess。
```

### 8.2 单帧异常检测

建议继续保留：

```text
fastlio_max_delta_translation = 0.20m
fastlio_max_delta_yaw = 0.25rad
```

并增加 dt 检查：

```text
0.01s <= dt <= 0.30s
```

如果 dt 异常：

```text
重置基准，不推进
```

### 8.3 NDT 验收门控不能放宽

不要因为使用了 Fast-LIO 预测就放宽：

```text
score_threshold
pose_jump gate
pose_jump reacquire
```

Fast-LIO delta 只负责给初值，不负责证明结果可信。

## 9. 诊断字段建议

现有字段：

```text
fastlio_delta_applied
fastlio_delta_reject_reason
fastlio_dead_reckon_age_sec
fastlio_delta_translation
fastlio_delta_yaw
```

建议新增：

```text
fastlio_delta_guess_mode
fastlio_odom_body_used
fastlio_delta_dt_sec
fastlio_map_odom_guess_shift
```

用途：

```text
确认当前跑的是新模式，不是旧 map->odom += body_delta
确认 odom->body 参与了反推
确认 delta 时间间隔正常
确认 map->odom guess 的变化量是否异常
```

## 10. 风险评估

### 10.1 主要风险

| 风险 | 说明 | 缓解 |
|---|---|---|
| 矩阵乘法方向写反 | 会立即导致 init_guess 偏离 | 用 test1 旧地图前 180s 做硬验收 |
| Fast-LIO 已漂移 | 错误 delta 会带偏 NDT 初值 | 只允许 2s 短窗口，异常立即断开 |
| odom->body 时间戳不匹配 | 反推 map->odom 使用了不同时间的 body pose | cloud_stamp 优先，latest 降级必须检查 0.2s |
| NDT 错误 accept | 错误初值可能让 NDT 收敛到错误局部最优 | 不放宽 fitness 和 pose_jump gate |
| 长时间 reject 后错误累计 | 预测可能越来越差 | dead_reckon_timeout 后停止推进并重置 |
| 地图变化混淆测试结论 | 新旧地图差异会影响 fitness | 每次测试固定 bag 和地图，做 delta off/on A/B |

### 10.2 当前方案不可解决的问题

该重构方案不能解决：

```text
Fast-LIO 本身全局漂移
NDT 地图严重不匹配
长期退化区域无法匹配
全局绑架到错误走廊
```

这些情况应交给：

```text
ScanContext / HDL recovery
/initialpose
人工重定位
地图重建或地图质量修复
```

## 11. 可行性评估

可行，但只能作为中期实验项，不建议直接替代短期回退。

原因：

```text
当前 TF 树中已有 odom -> camera_init -> body
odom->body 可以由 TF buffer 组合得到
NDT 仍然输出 map->odom，符合 Nav2 需要的 TF 链
Fast-LIO delta 只作为 init_guess，不改变最终验收逻辑
```

但实现复杂度高于当前 Plan B：

```text
需要维护两套上一帧基准
需要严格处理 TF 时间戳
需要用矩阵统一处理坐标转换
需要完整 bag A/B 验收
```

## 12. 验收标准

### 12.1 必须先过 test1 旧地图前 180 秒

基线：

```text
旧地图 + delta off:
accepted 1232 / 1232
accept rate 100%
first_bad 无
```

新模式硬要求：

```text
accept rate 接近 100%
不能在 51s 左右 high_fitness
max consecutive rejected 接近 0
high_fitness 不得显著增加
```

如果这一项不过，不能继续上车。

### 12.2 test1 旧地图前 520 秒

硬要求：

```text
不得比 delta off 更早进入连续 rejected
accept rate 不得明显低于 delta off
fitness p90 不得明显恶化
```

### 12.3 test4 全程

目标：

```text
pose_jump 数量下降
high_fitness 不增加
accept rate 不低于原始基线
max consecutive rejected 缩短
```

如果只是：

```text
pose_jump 下降但 high_fitness 增加
```

则仍判定失败。

### 12.4 实机验证

分三阶段：

```text
1. 原地小幅转向 + 短直线
2. 单走廊往返
3. 多点导航
```

重点观察：

```text
map->odom 是否平滑
NDT correction 是否下降
fitness 是否稳定
是否提前 high_fitness
是否出现长时间 dead_reckon_timeout
```

## 13. 推荐执行顺序

### Phase 0：立即止损

关闭当前错误 Plan B：

```python
'use_fastlio_delta_guess': False
```

并用旧地图 test1 前 180 秒确认恢复：

```text
accept rate = 100%
first_bad = null
```

### Phase 1：实现新模式但默认关闭

新增：

```text
fastlio_delta_guess_mode = disabled
fastlio_delta_guess_mode = map_body_to_map_odom
```

默认：

```text
disabled
```

### Phase 2：离线 bag A/B

同一 bag、同一地图：

```text
mode=disabled
mode=map_body_to_map_odom
```

对比：

```text
accept rate
high_fitness
pose_jump
max consecutive rejected
first_bad
fitness/correction 分位数
```

### Phase 3：实机小范围灰度

仅当 Phase 2 通过后再上车。

## 14. 最终建议

当前版本：

```text
不建议继续调阈值
不建议继续使用当前 use_fastlio_delta_guess=true
```

短期：

```text
关闭 Fast-LIO delta guess，恢复稳定基线
```

中期：

```text
把 Fast-LIO delta 重构为 map_body_to_map_odom init_guess 预测模式
默认关闭，只作为实验项逐步验证
```

判断标准：

```text
任何新方案都必须先证明不会破坏 test1 旧地图正常段。
如果正常段都被破坏，即使某些 bag 上 pose_jump 下降，也不能上线。
```

