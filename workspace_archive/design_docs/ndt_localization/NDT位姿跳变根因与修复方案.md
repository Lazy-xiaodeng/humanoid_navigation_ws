# NDT 98% 帧被拒根因分析与 B+C 修复方案

日期：2026-05-28 (更新：修正参数基线、重构方案 C)
工作区：`/home/ubuntu/humanoid_ws`
状态：方案评估完成，已通过最终审查，待实施 Phase 1

## 1. 问题摘要

2026-05-27 14:27 的 23 分钟导航测试中，NDT 定位暴露出严重问题：

```
NDT 总帧数:    1501
Accepted:        27 ( 1.8%)   ← 几乎不工作
Rejected:      1473 (98.1%)   ← 几乎全被 pose_jump gate 拦截
Confirming:       1 ( 0.1%)
Max 连续 reject: 269 帧
```

**NDT 匹配质量本身极好**（fitness_score p50 = 0.0013，95.5% 在 [0.00, 0.01)），但结果被 pose_jump gate 系统性丢弃。

**实际影响**（当前架构 commit `a4ffca5`：NDT 自监控 + nav_state_manager 直接监听 ndt_status）：

```
NDT 连续 reject → ndt_status: state=rejected → nav_state_manager → DEGRADED → 停车
→ 触发 SC/HDL recovery → /initialpose → NDT 短暂恢复 (1-2 帧)
→ 几帧后又 reject → 再次停车 → 再次 recovery → ...
→ 23 分钟内 34 次 recovery，导航反复中断，无法连续运行
```

不是 NDT 漂移导致位姿错误，而是 NDT 被误杀导致**导航无法持续**。

## 2. 数据来源

| 日志 | 大小 | 内容 |
|------|------|------|
| `debug_logs/ndt_fusion_monitor_20260527_142701.jsonl` | 5.9MB, 12382 事件 | NDT 逐帧状态 + fusion 状态 + 位姿快照 |
| `logs/ndt_drift_20260527_114942.jsonl` | 244KB, 582 事件 | NDT 漂移诊断：方向一致性、关联距离 |

分析脚本：`ndt_baseline_analysis.py`

### 2.1 NDT 真实帧率

全域平均 1.08 Hz，但这**被长间隙严重拉低**。帧间隔分布：

```
活跃期间:
  p10: 0.100s → 10 Hz  (最快)
  p50: 0.200s →  5 Hz  (中位)
  p90: 12.60s          (长间隙——DEGRADED/停车/恢复期间)

极值: 259.3s 超长间隙 (可能是完整 recovery 周期)
```

**结论**：NDT 活跃时运行在 5-10 Hz，帧间隔 0.1-0.2s。长间隙 (p90=12.6s) 才是 init_guess 冻结累积大偏移的原因，不是门限本身太紧。

### 2.2 机器人运动参数

```
前进速度: 0.5 m/s
旋转速度: 0.6 rad/s (≈34°/s)
```

5-10Hz 下每帧物理上限：

| 帧率 | dt | 平移上限 | yaw 上限 |
|------|-----|---------|----------|
| 10Hz | 0.10s | 0.050m | 0.060rad (3.4°) |
| 5Hz | 0.20s | 0.100m | 0.120rad (6.9°) |

## 3. Root Cause: pose_jump 门控正反馈环

### 3.1 死亡螺旋机制

代码位置：`src/lidar_localization/src/lidar_localization_component.cpp`

NDT 活跃时运行在 5-10Hz，帧间位移仅 0.05-0.1m，0.4m 门限在此条件下绰绰有余。真正的问题是**长间隙累计**：

```
活跃期 (5-10Hz):
  帧 N:   机器人位置 A, NDT ACCEPT → corrent_pose = A
  帧 N+1: 机器人移动 0.05m 到 A', init_guess=A, NDT 找到 A' → corr=0.05m → ACCEPT
  ...正常运作...

长间隙期 (DEGRADED/停车/恢复, 可达 12+ 秒):
  NDT 连续 reject → nav_state_manager 检测 DEGRADED → 停车
  SC/HDL recovery 被触发 → 等待 /initialpose → 等待 NDT 恢复
  期间 NDT 停止处理 → init_guess 冻结在最后 accepted 位置

恢复期:
  /initialpose 到达 → has_last_good_transform_=false → 第一帧无条件通过
  → corrent_pose 更新为新位置 → has_last_good_transform_=true
  → 若此时实际位置与 init_guess 差异仍大 (恢复前累积的位移)
  → 下一帧 corr 超门限 → REJECT → init_guess 再次冻结
  → 又进入 DEGRADED → 再次停车 → 再次 recovery
  → "恢复—冻死—恢复" 循环, 34 次/23 分钟
```

```
导航中                  长间隙 (DEGRADED + recovery)      恢复后
  ✓✓✓✓✓  (N帧)         ✗✗✗✗✗✗✗ (12s+)                ✗✗✗ (又冻死)
  corr<0.1m              机器人停车                         corr 可能大
                          等待 recovery                        → REJECT
  ACCEPT                  init_guess 冻结                   → 再次停车
```

### 3.2 四个关键代码点合谋

| 行号 | 代码 | 后果 |
|------|------|------|
| 880 | `if (!use_odom_) {return;}` | 长间隙期间 odom 不更新 init_guess，完全冻结 |
| 1069-1070 | `init_guess = corrent_pose_with_cov_stamped_ptr_->pose.pose` | init_guess 总是上次 accepted 的值（可能已过期多秒） |
| 1153-1156 | `pose_jump_exceeded = ... correction_translation > max_pose_jump_translation_` | 长间隙后的修正量远超门限 → 拒绝 |
| 1227-1240 | reject 分支直接 `return` | 不更新 `corrent_pose_with_cov_stamped_ptr_` → 永久锁定 |
| 504-510 | `max_last_good_tf_age_sec_ = 0.5` + `republish = false` | 0.5 秒后 TF 停发 → Nav2 无法获取位姿 → 进一步确认失锁 |

### 3.3 reacquire 为何救不了

```cpp
// 行1158-1163: reacquire 三重硬限制
correction_translation <= pose_jump_reacquire_max_translation_  // 0.8m
fitness_score <= pose_jump_reacquire_max_fitness_               // 0.08
correction_yaw <= pose_jump_reacquire_max_yaw_                  // 0.3rad
```

reacquire 入口门槛 0.8m。如果在恢复前机器人已累积了 >0.8m 的位置偏移（导航中被中断在远离上次 accepted 帧的位置），连 reacquire 通道一起关闭。系统进入无法自愈态，只能等下一次 SC/HDL 重发 `/initialpose`——但即使恢复后也只在 4 秒 initialpose 窗口（2.0m 放宽上限）内有效，窗口一过又冻死。

### 3.4 数据验证：渐进恶化，不是一开始就全丢

**各区域 accept 率**（按 NDT 帧出现顺序）：

```
区域 (4, 8):    4/5   = 80%     起点附近, NDT 基本正常
区域 (8, 0):    4/7   = 57%     开始出现交替 accept/reject
区域 (8, 16):   3/3   = 100%    正常
区域 (-8,16):   2/2   = 100%    正常
区域 (-4, 8):   2/3   = 67%     边缘
区域 (0, 8):    1/41  = 2.4%    严重退化 (fitness 飙到 2.517)
区域 (28,24):   8/1437= 0.56%   死亡螺旋全面爆发
```

**转折点在 (9.03, 2.20) —— 区域 (8, 0)**：

```
位置 (5.8, 11.0): corr=0.001m, fitness=0.001  ← 正常
    ↓ 导航到 (9.0, 2.2)
位置 (9.0, 2.2):  fitness 升高到 0.11-0.14  ← 几何开始退化
                  交替: reject(0.41m) → accept(0.03m) → reject(0.41m) → accept(0.03m)
                  ↑ 几次 reject 后 init_guess 冻结
    ↓
位置 (0, 8):      fitness 飙到 2.517 ← 严重退化 + init_guess 已冻结
    ↓
位置 (28, 24):    1437 帧中仅 8 帧 accept ← 死亡螺旋全面爆发
```

**关键 insight**：(9.03, 2.20) 的几何特征让 NDT 匹配质量下降（fitness ~0.12，虽低于 0.3 阈值但偏高），导致 correction 偶发超过 0.4m 门限。几次拒绝后 init_guess 冻结，后续即使回到几何更好的区域也无法自愈。前几个导航点正常是因为还没到这个几何退化区。

**若当时有方案 B**：delta guess 会在 (9.0, 2.2) 持续推进 init_guess，NDT correction 保持在 ~0.02m（而非 0.41m），不会触发拒绝链，系统从 (9.0, 2.2) 平滑过渡到后续区域。

PCL 位姿轨迹全程约 28m：`(5.84,10.95) → (9.03,2.20) → (30.36,28.43) → (-6.89,17.03) → (31.43,25.22)`。

### 3.5 日志中 34 次 recovery_request 印证了循环

```
HEALTHY → 短暂(通常<10秒) → DEGRADED → LOST → recovery → /initialpose → HEALTHY → ...
```

每次 recovery 通过 `/initialpose` 将 `has_last_good_transform_` 重置为 false（行 811），短暂"呼吸"一帧（行 1153-1154 条件不满足），然后再次冻死。

## 4. 当前参数快照

> **注意**：以下为当前 HEAD (commit `a4ffca5`) 的实际生效参数。测试当天 (commit `0349f58`) 的值为 `max_pose_jump_translation=0.40, max_pose_jump_yaw=0.30, republish=false`。commit `f302565` 已将门限放宽到 0.80/0.60 并开启 republish，但**尚未用新参数跑过实机测试**——当前 0.80m 门限下的 accept rate 未知。

```
score_threshold:              0.3
max_pose_jump_translation:    0.80m    ← f302565 已从 0.40 放宽, 98%拒收时的 2x
max_pose_jump_yaw:            0.60rad  ← f302565 已从 0.30 放宽
pose_jump_reacquire_enabled:  true
pose_jump_reacquire_max_translation: 0.80m
pose_jump_reacquire_max_yaw:         0.30rad
pose_jump_reacquire_max_fitness:     0.08
pose_jump_reacquire_required_frames: 2
pose_jump_reacquire_xy_tolerance:    0.50m
pose_jump_reacquire_yaw_tolerance:   0.25rad
initialpose_relax_duration_sec:      4.0s
initialpose_max_pose_jump_translation: 2.00m
initialpose_max_pose_jump_yaw:        3.00rad  ← a4ffca5 已从 1.20 放宽
republish_last_good_tf_on_failure:   true     ← yaml 默认 true, launch 之前 override 为 false, 现已修复
max_last_good_tf_age_sec:            0.5s
ndt_outlier_ratio:            0.30
ndt_max_corr_dist:            2.0
ndt_rotation_prior_weight:    10.0
use_odom:                     false    ← init_guess 冻结的直接原因
```

**关键推论**：门限已从 0.40→0.80 放宽但仍未实机验证。即使 0.80m 门限改善了 accept rate，根因（init_guess 在长间隙期间冻结）仍未解决——只要间隙足够长（>8 帧, ~1.6s @ 5Hz），累积位移仍会超过 0.80m。方案 B (delta guess) 从源头消除冻结，才是治本之策。

## 5. B+C 方案设计

### 5.1 方案概述

| 方案 | 定位 | 改什么 | 代码量 |
|------|------|--------|--------|
| **B** - Fast-LIO delta guess | 防漂（根本） | 每帧用 `camera_init→body` TF 的变化量推进 init_guess | ~80 行 C++ |
| **C** - TF 延长 + 门限收紧 | 容错 + 安全（辅助） | max_age 延长 + 方案 B 后收紧 gate | ~5 行 launch |

**B 是防，C 是收。合在一起：防得住 + 拦得住。**

### 5.2 方案 B：Fast-LIO delta guess

#### 原理

不用 `use_odom=true`（那是直接积分非标准轴 twist，方向错误）。而是从 TF 树中查询 `camera_init → body` 的**位姿差**（delta），通过固定的 120° 旋转矩阵 `R_cam_to_ros` 显式转换为 ROS 标准方向后再推进 `corrent_pose_with_cov_stamped_ptr_`。

#### 非标准轴转换 — 已验证

Fast-LIO 坐标约定：x=左, y=下, z=后。ROS 标准：x=前, y=左, z=上。

旋转矩阵 `R_cam_to_ros = R.from_quat([-0.5, -0.5, 0.5, 0.5])`：

```
    [ 0   0  -1 ]        ROS_x = -camera_init_z   (前进 = -后退)
R = [ 1   0   0 ]   →    ROS_y =  camera_init_x   (左 = 左)
    [ 0  -1   0 ]        ROS_z = -camera_init_y   (上 = -下)
```

数值验证通过：0.3m 前进 → ROS delta = (0.3, 0, 0)；10° 偏航 → ROS yaw delta = 10°。

#### 与 `use_odom=true` 的本质区别

| | `use_odom=true` (危险) | Plan B delta guess (安全) |
|---|---|---|
| 输入 | odom topic 的 twist (非标准轴) | TF `camera_init→body` 的位姿差 |
| 坐标转换 | 无，直接按 ROS 标准积分 → **方向错误** | 显式 `R_cam_to_ros` 转换 → **方向正确** |
| 误差模式 | 绝对位姿累积，不重置 | 累计 delta 从上次 accept 算起，accept 时归零 |
| 安全边界 | 无 | 单帧 max_delta (0.20m/0.25rad) + dead reckoning 时间上限 2.0s |

#### Delta guess 需要跟踪哪些分量

`camera_init → body` 的 TF 包含完整 6DOF 位姿（平移 + 四元数）。但 NDT 工作在 `force_2d_pose=true` 模式下，各分量需求如下：

| 分量 | 是否需要 | 原因 |
|------|----------|------|
| dx (ROS x, 前进) | **必须** | 机器人前后移动的核心分量 |
| dy (ROS y, 左侧) | **必须** | 机器人左右移动的核心分量 |
| dz (ROS z, 上方) | 不需要 | `force_2d_fixed_z=true` 强制 z=0，delta guess 算出来也被清零 |
| droll (绕 ROS x) | 不需要 | `force_2d_pose` 约束 roll=0 |
| dpitch (绕 ROS y) | 不需要 | `force_2d_pose` 约束 pitch=0 |
| **dyaw (绕 ROS z)** | **必须** | spintopose + 转弯场景关键——yaw 变化远大于 xy 变化 |

#### spintopose 场景：为什么 yaw 不可或缺

机器人到达导航点位后会执行原地旋转（spin-in-place）。这段时间内：

```
xy:   几乎不变 (0-0.1m/帧)
yaw:  每帧变化 20-40°  (1.1Hz 帧率下，转一圈 ~3-4 秒 → 每帧 ~30° = 0.5rad)
```

**如果 delta guess 只跟踪 xy 不跟踪 yaw**：

```
帧 N:   机器人 yaw=0°,   init_guess yaw=0°   → NDT 匹配正确
帧 N+1: 机器人 yaw=30°,  init_guess yaw=0°   → NDT 初值偏 30° → 搜索范围窄 → 高风险
帧 N+2: 机器人 yaw=60°,  init_guess yaw=0°   → NDT 初值偏 60° → 几乎必然收敛到错误局部最优
```

NDT 从偏 60° 的初值出发，可能把当前 scan 匹配到地图中另一个方向相似的走廊/拐角，导致 **NDT 漂移到完全错误的位置**。spintopose 是 NDT 漂移的**高危时刻**。

**所以 delta guess 必须包含 yaw，其重要性与 xy 等同。**

#### Yaw 从 TF 中提取的方式

`camera_init → body` 的 TF 包含完整旋转四元数。提取 yaw delta 的步骤：

```
1. 查 TF 得到 body 在 camera_init 中的四元数 q_body_in_cam
2. 转换为 ROS 系: q_body_in_ros = R_cam_to_ros * q_body_in_cam * R_cam_to_ros^(-1)
3. 从 q_body_in_ros 提取 yaw: yaw = atan2(2(qw·qz + qx·qy), 1 - 2(qy² + qz²))
4. delta_yaw = yaw_current - yaw_prev (规范化到 [-π, π])
```

> 注意：yaw 的单帧异常检测已在伪代码 step 4e 中通过 `max_delta_yaw_` (0.25rad) 实现——超限直接重置基准并跳过本帧，无需额外 clamp。

数值验证已确认正确：ROS 中 10° yaw 旋转 → 提取的 delta_yaw = 10°。

#### 核心设计：逐帧 delta 推进 corrent_pose

**关键决策**：直接修改 `corrent_pose_with_cov_stamped_ptr_`。这是防止 init_guess 冻结的机制。

> **命名提示**：启用 fastlio_delta_guess 后，`corrent_pose_with_cov_stamped_ptr_` 不再是"当前可信定位"。后续可考虑重命名为 `current_guess_pose_`。当前实现阶段先在成员变量旁加注释说明语义变更。

```
corrent_pose_with_cov_stamped_ptr_ 的语义变更 (启用 fastlio_delta_guess 后):

  旧语义: 始终 = 最后 NDT accepted 位姿
  新语义: 内部 init_guess rolling state (被 delta 逐帧推进)

  对外发布规则 (不变):
    - /pcl_pose: 仅在 NDT accepted 或 confirmed_pose_jump 时发布
    - last_good_transform_: 仅在 NDT accepted 时更新
    - path: 仅在 NDT accepted 时追加
    - accepted status (reason=ok/confirmed_pose_jump): 仅在 NDT accepted 时发布

  关键约束: delta 可以"污染"内部 rolling guess, 但绝不允许污染对外发布的定位结果。
  被推进的位姿只能作为下一帧 NDT 的 init_guess, 不能作为定位输出。
```

#### 伪代码流程

```
// === 成员变量 ===
bool has_prev_body_pose_ = false;
double prev_body_x_, prev_body_y_, prev_body_z_;    // 上帧 body 位置 (camera_init 系)
double prev_body_qx_, prev_body_qy_, prev_body_qz_, prev_body_qw_;  // 上帧 body 姿态
rclcpp::Time prev_cloud_stamp_;                      // 上帧 cloud 时间戳
rclcpp::Time last_accept_time_;                      // 上次 NDT accepted 的时间

cloudReceived():
  1. 按点云时间戳查 TF:
     cloud_stamp = msg->header.stamp
     try: tfbuffer_.lookupTransform("camera_init", "body", cloud_stamp)
     失败 -> 降级: lookupTransform("camera_init", "body", tf2::TimePointZero)
        -> 取到后检查 |tf_stamp - cloud_stamp|:
           注意: 用 rclcpp::Time 统一比较, 确保 clock type 一致
           若时间差 > tf_max_stamp_mismatch_sec_(0.2) -> 跳过本帧
     仍失败 -> 跳过本帧 (★ 不重置基准。TF 偶发失败只跳过本帧 delta，
       连续失败由 dead_reckon_sec 或 NDT rejected 触发 recovery)

  2. 若 has_prev_body_pose_ == false:
        存储当前 body 位姿 -> has_prev_body_pose_ = true -> 跳过 delta
        (第一帧只建立基准)

  3. dead reckoning 时间检查:
     dt_since_accept = (now() - last_accept_time_).seconds()
     若 dt_since_accept > max_dead_reckon_sec_ (2.0s):
        -> 停止应用 Fast-LIO delta (不推进 corrent_pose)
        -> 不回滚 corrent_pose 到 last accepted (回滚会重新引入大 correction)
        -> 不发布新 /pcl_pose 或 TF
        -> 持续发布 rejected status，等待 recovery 或 /initialpose
        (NDT 长时间未 accept, Fast-LIO 可能已累积漂移)

  4. 计算本帧 delta:
     a. delta_pos_cam = body_curr.pos - body_prev.pos
     b. delta_pos_ros = R_cam_to_ros * delta_pos_cam
        (map/world 全局位移, 非 body-local。
         前提: map 由同一 Fast-LIO PCD 经同一 R 变换得到, 平面轴对齐)

     c. yaw 处理 (★ 先分别转 ROS, 在 ROS 系内算相对旋转):
        q_prev_ros = R_cam_to_ros * q_prev_cam * R_cam_to_ros^(-1)
        q_curr_ros = R_cam_to_ros * q_curr_cam * R_cam_to_ros^(-1)
        delta_q_ros = q_prev_ros.inv() * q_curr_ros
        dyaw = getYaw(delta_q_ros)
        (全程 ROS 系操作, 不会把 camera_init 非标准轴量当 ROS 量)

     d. 提取 dx = delta_pos_ros.x, dy = delta_pos_ros.y

     e. 单帧异常检测 (Fast-LIO 跳变保护, 平移+yaw 同时检查):
        若 |dx| > max_delta_translation_ (0.20m) 或
           |dy| > max_delta_translation_ (0.20m) 或
           |dyaw| > max_delta_yaw_ (0.25rad):
           -> 重置 has_prev_body_pose_=false -> 跳过 delta
           (异常 delta 直接丢弃+重置基准, 不 clamp。
            两个分量必须同时检查——纯旋转跳变如果只检查平移会漏掉)

     f. 推进 corrent_pose:
        corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x += dx
        corrent_pose_with_cov_stamped_ptr_->pose.pose.position.y += dy
        对 corrent_pose 的 orientation 应用 dyaw 旋转
        applyPlanarPoseConstraint -> z=0, roll/pitch=0

  5. 存储本帧 body 位姿和 cloud_stamp 供下帧使用
  6. 继续原有 NDT 匹配流程 (init_guess 已接近真实位置)
  7. 若 NDT accepted (含 confirmed_pose_jump):
        -> last_accept_time_ = now()
        -> corrent_pose 已被 NDT 结果覆盖 (行1256-1259, Fast-LIO 累积漂移归零)
        -> 更新 last_good_transform_, has_last_good_transform_
        -> 发布 /pcl_pose 和 accepted status
        (confirmed_pose_jump 的 state='accepted' (行1263-1264), 与普通 accepted 语义一致)

initialPoseReceived():
  -> has_prev_body_pose_ = false  (外部重定位, 基准失效)

mapReceived():
  -> has_prev_body_pose_ = false  (新地图)

onParameterChange():  // use_fastlio_delta_guess 切换或参数重载
  -> has_prev_body_pose_ = false  (参数变更)
```

#### 新增参数

```yaml
use_fastlio_delta_guess: false                # 默认关闭, launch 显式打开。回滚安全, 离线/单元测试不受影响
fastlio_camera_frame: "camera_init"
fastlio_body_frame: "body"
tf_max_stamp_mismatch_sec: 0.2              # TF 与 cloud 时间戳最大容忍差

# 单帧 delta 硬上限 (拦截 Fast-LIO 异常跳变, 平移+yaw 同时独立检查)
fastlio_max_delta_translation: 0.20         # 单帧平移上限 (m) — 5Hz 下 = 2 帧物理上限
fastlio_max_delta_yaw: 0.25                 # 单帧偏航上限 (rad) — 5Hz 下 = 2 帧物理上限

# Dead reckoning 时间限制 (防止长期 reject 时 Fast-LIO 无限漂移)
fastlio_max_dead_reckon_sec: 2.0            # NDT 超过此时长未 accept -> 停止推进 delta
```

**参数设计依据**：
- `max_delta_translation=0.20m`: 5Hz 下物理上限 0.1m × 2 倍 = 0.20m。正常帧不受限，Fast-LIO 异常跳变被拦截
- `max_delta_yaw=0.25rad`: 5Hz 下物理上限 0.12rad × 2 倍 = 0.25rad。纯旋转跳变如果只看平移会漏掉，必须同时检查
- `max_dead_reckon_sec=2.0s`: 覆盖 5Hz 下 10 帧的正常间隙。超过 2 秒无 accept 说明 NDT 可能已进入 DEGRADED，应停止信任 delta，等待 recovery 通过 /initialpose 重新建立基准

#### 与 "累计 delta" 方案的对比

| 场景 | 逐帧 delta 推进 corrent_pose (本方案) | 累计 delta + 平移上限 (被否决) |
|------|--------------------------------------|-------------------------------|
| 正常导航, NDT 持续 accept | corrent_pose 每帧被推进+NDT覆盖, gap=0 ✓ | 同 ✓ |
| 短暂 reject (1-5 帧) | corrent_pose 推进跟踪, 恢复后 accept ✓ | 同 ✓ |
| 长距离 reject (机器人走 3m, NDT 持续不工作) | delta 逐帧推进, init_guess 始终跟踪真实位置。dead_reckon_sec 超时后停止 ✓ | 累计平移>1m 停止推进 -> init_guess 冻结 -> 死锁 ✗ |
| Fast-LIO 异常跳变 | max_delta_translation 拦截+重置 ✓ | max_instant_speed 拦截 ✓ |

#### 双非标准轴下的旋转公式验证

用户确认：`camera_init` 和 `body` **都在 Fast-LIO 的非标准坐标系中**（x=左, y=下, z=后）。

**公式仍然正确。** 核心逻辑是**坐标基变换**，与两个 frame 是否同轴无关：

```
1. camera_init→body TF: 描述 body 在 camera_init 系中的位姿
2. delta_cam = body_curr ⊖ body_prev: body 的物理运动，在 camera_init 轴中的表达
3. delta_ros = R_cam_to_ros · delta_cam · R_cam_to_ros^(-1): 同一运动，转译到 ROS 轴
```

**数值验证**（0.5m/s 前进 + 0.6rad/s 旋转, dt=0.909s 最坏情况）：

```
真实位移 (ROS):  dx=0.455m, dy=0.000m, dyaw=0.545rad (31.2°)

从 camera_init→body TF 提取:
  delta_pos_cam = [0, 0, -0.455]    ← 前进沿 camera_init 的 -z
  delta_pos_ros = [0.455, 0, 0]     ← 正确恢复为 ROS +x (前进) ✓
  delta_yaw_ros = 0.545rad = 31.2°  ← 正确恢复 ✓
```

**为什么两个非标准轴之间能正确转换？** TF 给出的位移向量的数值取决于用什么坐标系来描述它。camera_init 用自己的轴（非标准）"编码"位移，R 矩阵将其"转译"成 ROS 轴的编码。只要 `camera_init` 和 `body` **使用相同的轴约定**（它们确实都是 Fast-LIO 约定），delta 在它们之间正确，R 矩阵的转译就正确。

#### 依赖

- TF 基础设施已存在（`tfbuffer_` 在行 761, 949 已使用），无需新增 Buffer/Listener
- `R_cam_to_ros` 旋转矩阵与 `cloudReceived` (行 1017)、`mapReceived` (行 848)、静态 TF `odom→camera_init` (launch 行 133) 中使用的四元数一致

#### 实现验收项 (reject 分支不发布推进后的位姿)

以下状态**必须仅在 NDT accepted 或 confirmed_pose_jump 时**更新，**绝不能**在 reject 后因 delta 推进了 corrent_pose 而连带发布：

| 状态 | 更新时机 | 检查点 |
|------|----------|--------|
| `/pcl_pose` | accepted / confirmed_pose_jump | `pose_pub_->publish()` 仅在行 1262 调用 (accepted 路径) |
| `last_good_transform_` | accepted | 行 1278-1279, 仅在 accepted 路径 |
| `last_good_transform_time_` | accepted | 行 1279 |
| `has_last_good_transform_` | accepted | 行 1280 |
| `path` | accepted | 行 1293-1294 |
| accepted status (reason=ok/confirmed_pose_jump) | accepted / confirmed_pose_jump | 行 1263-1266 |

delta 只能推进 `corrent_pose_with_cov_stamped_ptr_` 的内部值（作为下帧 init_guess），绝不允许污染上述对外发布的定位结果。现有代码的 reject 分支（行 1227-1240）已经在 `return` 前不执行发布逻辑，但实现方案 B 时需确认 delta 推进代码插入位置在 init_guess 设置之前，且不绕过 reject 分支。

#### 调试字段 (加到 /localization/ndt_status)

实机调参时需区分"NDT 拒了"还是"delta 被熔断了"。在 `publishLocalizationStatus()` 的 JSON 中增加：

```json
{
  "fastlio_delta_applied": true/false,       // 本帧是否应用了 delta
  "fastlio_delta_reject_reason": "",          // 未应用的原因 (tf_lookup_failed / tf_stamp_mismatch / max_delta_exceeded / dead_reckon_timeout / no_prev_pose)
  "fastlio_dead_reckon_age_sec": 0.0,         // 距上次 NDT accept 的时间 (秒)
  "fastlio_delta_translation": 0.0,           // 本帧 delta 平移量 (m)
  "fastlio_delta_yaw": 0.0                    // 本帧 delta 偏航量 (rad)
}
```

这些字段仅用于调试，不参与 nav_state_manager 的 HEALTHY/DEGRADED 判断。

### 5.3 方案 C：TF 延长 + 门限收紧

#### 核心定位

commit `f302565` 已将门限从 0.40→0.80m、0.30→0.60rad 放宽，`republish_last_good_tf_on_failure` 也已改为 true。这些改动**尚未实机验证**——98% 拒收的基线数据是在旧门限 (0.40m) 下采集的。

方案 C 在当前基线 (0.80m/0.60rad, republish=true) 上的真正改动：

1. **TF 存活时间延长**：`max_last_good_tf_age_sec: 0.5→5.0`（覆盖典型 DEGRADED 窗口）
2. **方案 B 生效后收紧门限**：delta guess 将 correction 从 ~1-2m 压到 ~0.01-0.02m，宽门限 (0.80m) 不再必要，收紧后反而能更好拦截真正的 NDT 匹配错误

#### 修改参数

```yaml
# ═══ 当前基线 (f302565/a4ffca5, 已生效, 无需再改) ═══
max_pose_jump_translation: 0.80        # 已从 0.40 放宽
max_pose_jump_yaw: 0.60                # 已从 0.30 放宽
republish_last_good_tf_on_failure: true  # 已开启 (yaml 本就是 true)

# ═══ 方案 C 改动的参数 ═══
# TF 存活时间 (覆盖 DEGRADED 窗口, 给 recovery 响应时间)
max_last_good_tf_age_sec: 5.0          # 旧: 0.5  → 新: 5.0

# 门限收紧 (方案 B 生效后, correction 仅 ~0.01-0.02m, 宽门限不再必要)
max_pose_jump_translation: 0.50        # 当前: 0.80 → 新: 0.50
max_pose_jump_yaw: 0.40                # 当前: 0.60 → 新: 0.40

# Reacquire 通道收紧 (方案 B 防止了大跳变, reacquire 应是罕见路径)
pose_jump_reacquire_max_translation: 0.50  # 当前: 0.80 → 新: 0.50
# pose_jump_reacquire_max_fitness 保持 0.08
```

#### 每个参数的重新评估

##### max_pose_jump_translation：0.80 → 0.50m (收紧, 非放宽)

当前值 0.80m 是 f302565 中为应对 98% 拒收做的紧急放宽。方案 B 的 delta guess 每帧推进 init_guess 后，NDT correction 预计仅 ~0.01-0.02m（delta 误差级别）。即使用 5Hz 保守估计，5 帧累积误差也不超过 0.15m。0.50m 预留了充足余量（~25 帧的 delta 误差累积），同时比 0.80m 更早拦截真正的 NDT 匹配错误。

**门限收紧的逻辑**：
```
无方案 B: correction = 真实位移 + NDT误差 — 宽门限(0.80)是不得已
有方案 B: correction = delta误差 + NDT误差 ≈ 0.01-0.02m/帧
          → 0.50m 门限 = 正常帧通过 (0.02m << 0.50m) + 错误匹配被拦截
          → 收紧门限是安全收益, 不是风险
```

##### max_pose_jump_yaw：0.60 → 0.40rad (收紧)

方案 B delta guess 跟踪 yaw 后，NDT yaw correction 预计 ~0.02rad。0.40rad 覆盖 ~20 帧的 delta yaw 误差累积。**物理上限：0.6rad/s ÷ 5Hz = 0.12rad/帧 × 3 帧 = 0.36rad，0.40rad 覆盖有余。**

##### pose_jump_reacquire_max_translation：0.80 → 0.50m (收紧)

reacquire 通道用于处理 delta guess 失效后的大跳变。方案 B 生效后，delta guess 持续推进 init_guess，大跳变场景极少出现。收紧 reacquire 门限降低误通过风险。reacquire 仍需连续 2 帧 `max_fitness=0.08` 验证，不会因为门限降低就放过错误匹配。

##### max_last_good_tf_age_sec：0.5 → 5.0s

**覆盖典型 DEGRADED 窗口**（5-15 秒）。给 SC/HDL recovery 足够的响应时间。5 秒后如果仍未恢复，TF 静默表明定位确实丢失。

#### 为什么门限收紧是安全的

方案 B + 收紧门限的组合逻辑：

```
1. 方案 B delta guess 推进 init_guess → NDT 从接近真实位置出发
2. correction 微小 (~0.02m) → 正常帧轻松通过 0.50m 门限
3. 如果 NDT 匹配到错误位置 (真实错误):
   - 错误匹配的 correction 通常 >0.5m (偏离真实位置很远)
   - 且 fitness 通常很高 (>0.3)
   - → 被 0.50m 门限 + score_threshold=0.3 双重拦截
4. 如果 delta guess 失效 (Fast-LIO 漂移):
   - dead_reckon_sec=2.0s 停止 delta
   - init_guess 冻结 → correction 变大 → 0.50m 门限拦截
   - → 触发 DEGRADED → recovery (正确行为)
```

**核心 insight**：收紧门限不是限制正常帧，而是提高错误帧的拦截速度。0.80m 门限意味着 NDT 可以漂 0.79m 才被拦截；0.50m 门限在 0.51m 就拦截——更早发现问题。

#### 方案 C 全局风险分析

| 风险场景 | 概率 | 后果 | 防御机制 |
|----------|------|------|----------|
| 门限 0.50m 误杀正常帧 (方案 B 已生效) | 极低 | 偶发 reject | 方案 B delta guess 后 correction ~0.02m << 0.50m。即使偶发 TF 抖动，0.50m 是 25 帧的余量 |
| 门限收紧导致 false reject 增加 | 低 | accept rate 略降 | 可在实机验证阶段按需微调 (0.50→0.60)。优先安全 |
| 旧 TF 重发 5 秒导致陈旧位姿 | 低 | Nav2 短时用过期位姿 | nav_state_manager 在 DEGRADED 时停车；方案 B delta guess 推进位姿 |
| Reacquire 门限 0.50m 太紧 | 低 | reacquire 成功率下降 | reacquire 是罕见路径 (方案 B 防止了大跳变)；若实机显示需放宽可调回 0.80 |

**门限含义**：`max_pose_jump_translation=0.50m` 在方案 B 生效后不是"单帧物理上限"，而是**delta guess 失效的检测灵敏度**。值越小，NDT 匹配异常越早被发现。

#### 安全闸仍保留

| 闸门 | 作用 | 是否改动 |
|------|------|----------|
| `score_threshold=0.3` | 拒绝高 fitness 的错误匹配 | 不变 |
| `pose_jump_reacquire_max_fitness=0.08` | reacquire 要求极高质量（比 0.3 严格 3.75 倍） | 不变 |
| `has_converged` 检查 | 拒绝未收敛的匹配 | 不变 |
| `min_scan_points=50` | 拒绝点云不足的帧 | 不变 |
| `initialpose_relax_duration_sec=4.0` | 收到 /initialpose 后的窗口 | 不变 |

## 6. B+C 合在一起后的预期表现

### 6.1 正常导航 (大厅, 5-10Hz 活跃期)

```
旧: 长间隙后 init_guess 冻结 → 恢复后 corr=1-2m → REJECT → 死循环
新: delta guess 在活跃期推进 init_guess → corr=0.01-0.02m → ACCEPT → 持续健康
    即使短暂间隙 (1-2 帧), delta 误差累积 <0.3m → 仍在 0.5m 门限内
```

**预期**: accept rate 从 1.8% → 95%+

### 6.2 拐角转弯 (方案 B 的 yaw 跟踪关键)

```
旧: 转弯帧间 yaw 变化 0.12rad → 超过 0.3rad? 不会 (5Hz下仅 6.9°/帧)
    真正的问题: 转弯后可能出现长间隙 → init_guess 冻结 → 恢复后大偏移
新: delta guess 跟踪转弯 xy+yaw → 间隙后 init_guess 接近真实位置
```

转弯在 5-10Hz 帧率下不是问题——每帧 yaw 变化仅 3.4-6.9°，远在 0.4rad 门限内。

### 6.2b spintopose (点位原地旋转)

```
5Hz下每帧 yaw 变化: 0.6rad/s ÷ 5Hz = 0.12rad/帧 (6.9°/帧)
10Hz下每帧 yaw 变化: 0.6rad/s ÷ 10Hz = 0.06rad/帧 (3.4°/帧)

门限 0.4rad = 覆盖 3.3 帧 @ 5Hz → 正常 spin 不会超门限

旧: 核心风险不是单帧超门限, 而是 spin 后长间隙 → init_guess yaw 冻结在 spin 前
    恢复后 yaw 偏差 = 整个 spin 期间的累积旋转 (可达几十度)
新: delta guess 跟踪 yaw → 间隙后 init_guess yaw 接近真实方向
    NDT 从正确 yaw 初值出发 → 快速收敛 → ACCEPT
```

**这是 B+C 方案的关键价值场景**——spintopose 后长间隙是 NDT 最高危时刻，delta guess 消除了 yaw 冻结风险。

### 6.3 实验室拐角 (传统退化区)

```
旧: 几何弱 + init_guess 差 → NDT 可能收敛到错误局部最优
新: delta guess 提供可靠初值 → NDT 从正确位置附近搜索 → 大概率收敛到正确解
```

**注意**: B+C 改善的是收敛初值，不改变 NDT 匹配算法本身。若地图确实在某个区域缺失 3D 特征（z_std < 0.1m），NDT 仍可能在沿墙方向有轻微漂移，但 delta guess 限制了单帧误差累积。

### 6.4 导航连续性

```
旧: NDT 连续 reject → nav_state_manager → 停车 → recovery → 短暂恢复 → 又冻死
    34 次 recovery/23 分钟，导航反复中断

新: NDT 持续 ACCEPT → nav_state_manager 保持 HEALTHY
    → 导航连续运行，不需要频繁停车
    → recovery 仅在 NDT 真正匹配失败时触发（概率远低于当前）
```

## 7. 与救场全流程的关系

```
┌──────────────────────────────────────────────────────────────────┐
│                    定位救场全流程 (不变)                          │
│                                                                  │
│  HEALTHY ──→ DEGRADED ──→ LOST ──→ SC/HDL recovery ──→ /initialpose
│     ↑                                      │                     │
│     └────── NDT 恢复 ──────────────────────┘                     │
│                                                                  │
│  B+C 改善的环节:                                                  │
│  ① HEALTHY 态: delta guess 保持 init_guess 不冻结 → 不易退化      │
│  ② DEGRADED 态: republish TF + 放宽 gate → 有机会自愈             │
│  ③ /initialpose 后: delta guess 快速收敛 → 减少反复               │
│                                                                  │
│  B+C 不改动的环节:                                                │
│  - nav_state_manager 的 NDT 状态监听与停车决策                     │
│  - SC/HDL global recovery 触发逻辑                                │
│  - /initialpose → NDT 的位姿注入流程                              │
│  - 定位架构简化 (无 fusion 节点)                                   │
└──────────────────────────────────────────────────────────────────┘
```

**B+C 不破坏任何现有流程。** 恢复全流程完整保留。B+C 的作用是：
- **减少进入 DEGRADED 的频率**（消除因 init_guess 冻结导致的假性 DEGRADED）
- **让导航能连续运行**（不再反复停车→恢复→停车）

### 7.1 方案 B 在恢复流程各阶段的行为

| 阶段 | delta guess 在做什么 | 是否干扰恢复 |
|------|---------------------|-------------|
| 导航中 | 正常推进 init_guess | 不干扰，正是其职责 |
| DEGRADED 触发 | 继续推进但 robot 已停车, delta≈0 | 不干扰。`dead_reckon_sec=2.0s` 后自动停止 |
| Recovery 搜索 | rbt 静止, delta≈0; SC/HDL 独立搜索 | 不干扰。SC/HDL 不依赖 NDT 的 init_guess |
| /initialpose 注入 | `has_prev_body_pose_=false` → 重置基准 | **保护点 1**：旧 delta 基准清零, 新位姿不被污染 |
| 恢复后首帧 | 无 delta (基准刚建立); NDT 从 /initialpose 出发 | 不干扰。initialpose 窗口 (4s, 2.0m) 正常生效 |
| 恢复后正常 | delta 正常推进, 防止再次冻结 | 同 Phase 1 |

**两个关键保护点**：

1. **`/initialpose` 重置 delta 基准**：`initialPoseReceived()` 中设 `has_prev_body_pose_=false`。若不清，下一帧 delta 会用恢复前的旧 body pose 算位移，把刚纠正的位姿推偏。

2. **`dead_reckon_sec` 防止长时间 DEGRADED 中无限推进**：若 recovery 迟迟不触发（SC/HDL 超时），delta 可能把 init_guess 推漂（即使 robot 停了，Fast-LIO 静止漂移也会累积）。2.0s 后自动停止。

## 8. 与定位架构简化的兼容性

当前架构（commit `a4ffca5`）：

```
NDT 节点 → /localization/ndt_status → nav_state_manager → 停车/恢复决策
         → /pcl_pose (TF: map→odom) → Nav2
```

B+C 改动全部在 **NDT 节点内部**：
- delta guess: 在 `cloudReceived` 回调中，init_guess 设置之前
- 参数调整: 修改的是 NDT 节点已有的参数

**不改变**：
- `/localization/ndt_status` 的 topic 名、消息格式、发布频率
- `/pcl_pose` 的 frame_id、语义
- `nav_state_manager` 对 NDT 状态的判断逻辑
- SC/HDL 的触发逻辑和位姿发布

## 9. 风险矩阵

### 9.1 方案 B 风险

| 风险 | 严重度 | 概率 | 缓解措施 | 状态 |
|------|--------|------|----------|------|
| 坐标系错误：R_cam_to_ros 与实机 TF 不一致 | 高 | 低 | NDT 代码中已用同一 R 旋转 PCD 和实时点云（行 848, 1017），若 R 错误则 NDT 无法达到 fitness=0.001。建议实测时先用 "只 log 不发布" 验证 delta 方向正确后再启用 | 已覆盖 |
| 时间不同步：TF 查询用的时间戳与点云不对应 | 中 | 中 | **伪代码已更新**：优先按 `msg->header.stamp` 查 TF，失败降级到 `TimePointZero` | 已修复 |
| 长时间 reject 导致 delta 无限推进 corrent_pose | 中 | 中 | `max_dead_reckon_sec=2.0s`：NDT 超过此时长未 accept 停止推进 delta，等待 recovery | 已修复 |
| Fast-LIO 异常跳变：回环/重置/瞬移产生超大逐帧 delta | 中 | 低 | `max_delta_translation=0.20m` + `max_delta_yaw=0.25rad` 同时检查平移+yaw → 重置基准 (超限丢弃不 clamp)；纯旋转跳变不会因只看平移而漏掉 | 已强化 |
| TF 时间戳与 cloud stamp 的 clock type 不一致 (ROS2 use_sim_time/系统时间混用) | 中 | 低 | 统一用 `rclcpp::Time` 比较；`tf_max_stamp_mismatch_sec=0.2` 检查前先确保 clock type 一致，否则可能产生负 age 或超大 age 导致误判 | 已覆盖 |
| TF lookup 通信失败 | 低 | 低 | try-catch，跳过 delta update，回退到旧行为 | 已覆盖 |
| delta guess 在 DEGRADED 间隙不更新（cloudReceived 未调用） | 中 | 中 | `republish_last_good_tf_on_failure=true` + `max_age=5s` 维持 TF 不断链；nav_state_manager 已停车 | 已覆盖 |
| /initialpose 后未重置 delta 基准导致叠加偏差 | 中 | 中 | **伪代码已新增**：`initialPoseReceived()` 中设 `has_last_body_pose_=false` | 已修复 |
| NDT accepted 后未重置基准导致 delta 与绝对校正叠加 | 中 | 中 | **伪代码已新增**：accepted 后用当前 scan 对应的 body pose 重设基准 | 已修复 |
| map 平面轴与 ROS 化 camera_init 平面轴存在 yaw offset | 中 | 低 | 前提写死：map 由同一 Fast-LIO PCD 经同一 R 变换得到。实机验证阶段确认 | 前提明确 |

### 9.2 方案 C 风险

| 风险 | 严重度 | 概率 | 缓解措施 | 状态 |
|------|--------|------|----------|------|
| 门限收紧 (0.50m) 误杀正常帧 | 低 | 极低 | 方案 B delta guess 后 correction ~0.02m << 0.50m, 25 帧余量。若实机偶发误杀可微调至 0.60m | 需实机验证 |
| 门限收紧后 NDT 匹配错误仍能通过 (方案 B 失效时) | 中 | 低 | 0.50m 比 0.80m 更早拦截错误；`score_threshold=0.3` + `rotation_prior=10.0` + reacquire 一致性验证三重防御 | 已覆盖 |
| last-good TF 重发期间机器人未完全停止 (制动 0.5-1s) | 低 | 低 | 偏差 ~0.25-0.5m，局部规划器容限内；nav_state_manager 已触发停车；方案 B 从源头减少 DEGRADED | 可接受 |
| TF 重发 5 秒后仍无恢复，静默断链 | 低 | 低 | 5 秒为有意设置的上限——超过此时间说明确实需要 recovery，断链是正确行为 | 设计如此 |
| Reacquire 门限 0.50m 过紧导致恢复成功率下降 | 低 | 中 | 方案 B 防止了大跳变, reacquire 应是罕见路径；若实机显示需放宽, 可单独调回 0.80 | 需实机验证 |

## 10. 坐标系实机验证方案

方案 B 的核心风险是坐标系转换在实机上是否正确。建议分两阶段验证：

### 10.1 验证阶段：只 log 不推进

```
在 cloudReceived 中计算 delta (dx, dy, dyaw)，但不推进 corrent_pose:
- 日志输出: cloud_stamp, tf_stamp, dt, dx, dy, dyaw
- 同时记录 NDT 的 correction_translation 和 fitness_score

此阶段只能验证:
1. 直行时 dx 方向和累计量是否正确 (0.5m/s * 3s → 累积 ≈ 1.5m, dy ≈ 0)
2. 原地旋转时 dyaw 方向和累计量是否正确 (spin 90° → 累积 ≈ 1.57rad)
3. dt 和 delta 值是否在合理物理范围内
4. Fast-LIO 异常时 delta 是否被 max_delta/max_dt 正确拦截

此阶段不能验证 correction 变小 —— 因为未推进 init_guess。
```

### 10.2 启用阶段：推进但不改门限

```
- delta guess 推进 corrent_pose_with_cov_stamped_ptr_
- pose_jump gate 保持原值，NDT 原逻辑不受影响
- 观察: accept rate 是否上升，correction 是否下降
- 确认无异常后 → 全功能启用
```

### 10.3 为什么 R_cam_to_ros 大概率正确

NDT 代码中已有三处使用同一旋转矩阵：
- `mapReceived` (行 848)：PCD 地图从 camera_init 旋转到 ROS
- `cloudReceived` (行 1017)：实时点云从 camera_init 旋转到 ROS
- 静态 TF `odom→camera_init` (launch 行 133)：相同的四元数

这三个地方协同工作才让 NDT 达到 fitness=0.001。如果 R 矩阵错误，PCD 和实时点云的方向就不一致，NDT 不可能匹配成功。因此 delta guess 使用同一个 R 矩阵，在物理上是一致的。

## 11. 优缺点总结 (同前, 略)

### 优点

1. **根治冻结问题**：init_guess 不再冻结，accept rate 预期从 1.8% → 95%+
2. **非标准轴安全处理**：显式 `R_cam_to_ros` 转换，不用危险的 `use_odom=true`
3. **零新依赖**：复用已有 TF 基础设施（`tfbuffer_` 已存在）
4. **可独立开关**：B 和 C 各有独立参数，A/B 测试方便
5. **不改 NDT 算法**：不影响 fitness 语义，不破坏匹配质量判断
6. **硬上限保护**：delta guess 有 max 限制，不盲目信任
7. **向后兼容**：所有新参数有安全默认值，关闭后行为完全回退

### 缺点

1. 增加 ~80 行 C++，需编译（~2-3 分钟）
2. TF lookup 增加 ~0.1-0.5ms/帧（vs NDT 匹配 50-100ms，可忽略）
3. 需要在实际机器人上验证非标准轴转换在真实数据上的正确性
4. 参数调优可能需要 1-2 轮实机测试

## 12. 实施建议

> **当前基线** (commit `a4ffca5`)：门限已放宽至 0.80m/0.60rad，republish 已开启，但**尚未实机测试**。98% 拒收数据来自旧门限 (0.40m/0.30rad)。方案 C 的 gate 放宽部分已由 f302565 完成，剩余改动仅为 `max_last_good_tf_age_sec=5.0` 和方案 B 后的门限收紧。

```
Phase 1 (方案 B, C++ 改动 + 1 个 launch 参数):
  - 实施 C++ delta guess (方案 B 全部伪代码)
  - launch 改: max_last_good_tf_age_sec: 0.5→5.0 (方案 C 唯一未落地项)
  - 门限保持当前值 0.80/0.60/0.80 (不收紧, 先单独验证方案 B)
  - fastlio_max_delta_translation: 0.20, fastlio_max_delta_yaw: 0.25
  - fastlio_max_dead_reckon_sec: 2.0

Phase 2 (验证方案 B):
  - Step 1: "只 log 不推进" → 确认 delta 方向/量级正确
  - Step 2: 推进但不收紧门限 → 观察 accept rate 和 correction 分布
  - 验收标准:
    - accept rate > 90%
    - 平均 correction < 0.05m (方案 B 生效的标志)
    - max consecutive rejected < 10
    - recovery 触发次数减少 > 70%

Phase 3 (收紧门限, 方案 B 验证通过后):
  - launch 改:
    max_pose_jump_translation: 0.80→0.50
    max_pose_jump_yaw: 0.60→0.40
    pose_jump_reacquire_max_translation: 0.80→0.50
  - 同一路线重复 3 次
  - 对比收紧前后: accept rate, false positive (错误匹配被放行)
  - 若偶发误杀 → 微调至 0.60/0.45 (B 方案下仍有 30 帧余量)
```

## 13. 相关文件

| 文件 | 作用 |
|------|------|
| `src/lidar_localization/src/lidar_localization_component.cpp` | NDT 主逻辑，方案 B 改动位置 |
| `src/humanoid_navigation2/launch/navigation2_fusion_sc_v2.launch.py` | 方案 C 参数改动位置 |
| `ndt_baseline_analysis.py` | Baseline 统计分析脚本 |
| `debug_logs/ndt_fusion_monitor_20260527_142701.jsonl` | 本次分析的原始数据 |
| `NDT漂移根因与参数重设计.md` | 前端 NDT 参数分析和 Phase 0-5 改造计划 |

## A. 附录：Baseline 统计数据详情

### A.1 NDT 帧率与时长

```
时间范围: 14:27:02 → 14:50:15 (23.2 分钟)
平均帧率: 1.1 Hz
```

### A.2 NDT 状态分布

| 状态 | 帧数 | 占比 |
|------|------|------|
| accepted | 27 | 1.8% |
| rejected | 1473 | 98.1% |
| confirming | 1 | 0.1% |

### A.3 Fitness Score 分布

| 区间 | 帧数 | 占比 |
|------|------|------|
| [0.00, 0.01) | 1433 | 95.5% |
| [0.01, 0.05) | 9 | 0.6% |
| [0.05, 0.10) | 12 | 0.8% |
| [0.10, 0.15) | 43 | 2.9% |
| [0.15, 0.50) | 3 | 0.2% |
| [0.50, inf) | 1 | 0.1% |

### A.4 空间分析

| 区域 (4m grid) | 帧数 | avg_fitness | max_fitness | 评级 |
|----------------|------|-------------|-------------|------|
| (0, 8) | 16 | 0.29161 | 2.5168 | 严重 |
| (8, 0) | 4 | 0.12622 | 0.1436 | 警告 |
| (4, 8) | 8 | 0.04312 | 0.1237 | 正常 |
| (28, 24) | 14 | 0.00882 | 0.0486 | 优秀 |

### A.5 NDT 漂移诊断 (第二个 session)

| 诊断类型 | 次数 | 占比 | 特征 |
|----------|------|------|------|
| ODOM_DRIFT_SUSPECT | 542 | 94.9% | 方向一致性 0%，微小修正，全局漂移 20m+ |
| NDT_DRIFT_SUSPECT | 27 | 4.7% | 方向一致性 100%，修正集中单一方向 |
| NDT_DRIFT | 2 | 0.4% | 确认 NDT 漂移 |
