# 多帧 NDT 匹配优化方案

> 日期：2026-05-30 | 作者：ubuntu + Claude | 状态：方案设计

---

## 目录

1. [现状分析](#1-现状分析)
2. [核心问题：缺少运动补偿](#2-核心问题缺少运动补偿)
3. [方案一：运动补偿多帧累积（核心改造）](#3-方案一运动补偿多帧累积核心改造)
4. [方案二：关键帧选择策略](#4-方案二关键帧选择策略进阶优化)
5. [方案三：NDT 参数联动优化](#5-方案三ndt-参数联动优化)
6. [方案四：空间体素地图累积（远期架构升级）](#6-方案四空间体素地图累积远期架构升级)
7. [推荐实施路径](#7-推荐实施路径)
8. [完整改造代码指南](#8-完整改造代码指南)
9. [验证方案](#9-验证方案)

---

## 1. 现状分析

### 1.1 系统架构回顾

当前定位系统的完整数据流：

```
RoboSense Airy (96线)
    ↓ /airy_points
Fast-LIO (fast_lio_robosense)
    ↓ /fast_lio/cloud_registered  (已配准点云，camera_init坐标系)
    ↓ TF: camera_init → body       (Fast-LIO实时里程计)
lidar_localization (NDT匹配)
    ↓ 坐标系转换: camera_init → ROS标准 (固定四元数旋转)
    ↓ 体素滤波 (voxel_leaf_size=0.15m)
    ↓ 距离过滤 (1.0m ~ 100m)
    ↓ buildMultiFrameSource() → 多帧累积源点云
    ↓ NDT_OMP 与预建PCD地图配准
    ↓ Plan B: Fast-LIO delta guess (初始猜测推进)
    ↓ Pose jump guard → Candidate confirmation
    ↓ 发布 TF: map → odom
Nav2 (规划/控制)
```

### 1.2 当前多帧参数

| 参数 | 当前值 | 位置 |
|------|--------|------|
| `multi_frame_matching_enabled` | `true` | launch L575 |
| `multi_frame_use_only_when_rotating` | `false` | launch L576 |
| `multi_frame_window_sec` | **0.6s** | launch L577 |
| `multi_frame_max_frames` | **8** | launch L578 |
| `multi_frame_voxel_leaf_size` | **0.20m** | launch L579 |
| `multi_frame_max_points` | **40000** | launch L580 |

### 1.3 当前 NDT 匹配参数

| 参数 | 默认值(YAML) | v2 launch覆盖 | 说明 |
|------|-------------|---------------|------|
| `registration_method` | `NDT_OMP` | 继承 | pclomp OpenMP NDT |
| `ndt_resolution` | 0.5m | 继承 | NDT体素网格大小 |
| `ndt_step_size` | 0.1 | 继承 | 牛顿法线搜索步长 |
| `ndt_max_iterations` | 50 | 继承 | 最大迭代次数 |
| `ndt_num_threads` | 8 | 继承 | OpenMP线程数 |
| `transform_epsilon` | 0.02 | 继承 | 收敛阈值 |
| `voxel_leaf_size` | 0.15m | 继承 | 单帧预滤波 |
| `ndt_outlier_ratio` | 0.35 | **0.30** | 离群点比率（收紧） |
| `ndt_max_corr_dist` | 0.0(禁用) | **2.0m** | 最大关联距离 |
| `ndt_rotation_prior_enabled` | false | **true** | roll/pitch先验约束 |
| `ndt_rotation_prior_weight` | 0.0 | **10.0** | 先验权重 |
| `score_threshold` | 2.0 | **0.3** | fitness分数阈值（大幅收紧） |

### 1.4 LiDAR 特性

来自 `fast_lio_robosense/config/robosenseAiry.yaml`：

| 参数 | 值 |
|------|-----|
| 雷达型号 | RoboSense Airy |
| 扫描线数 | 96 |
| 水平FOV | 120° |
| 最大探测距离 | 60m |
| 盲区 | 0.2m |

典型帧率：10~20 Hz（取决于配置），按 15Hz 估算。

### 1.5 当前多帧累积的有效性分析

在 0.6s 窗口内：
- **静止时**：8帧完全重叠 → 点云密度提升 8x → NDT 匹配质量显著提升 ✓
- **低速运动时**（如 0.3m/s）：0.6s 内移动 ~0.18m → 轻微错位 → 密度提升但引入空间模糊 △
- **正常行走时**（如 1.0m/s）：0.6s 内移动 ~0.6m → 明显错位 → 可能反而降低精度 ✗
- **旋转时**：位移小但角度变化 → 远端点云错位严重 → 旋转保护介入 ✗

**结论**：当前 0.6s / 8帧的保守窗口是对"无运动补偿"这一短板的补偿策略。

---

## 2. 核心问题：缺少运动补偿

### 2.1 问题代码

[`lidar_localization_component.cpp:1380-1385`](../src/lidar_localization/src/lidar_localization_component.cpp#L1380-L1385)：

```cpp
// ★ 当前实现：直接原始拼接，没有任何坐标变换！
pcl::PointCloud<pcl::PointXYZI>::Ptr merged(new pcl::PointCloud<pcl::PointXYZI>());
for (const auto & buffered_frame : scan_frame_buffer_) {
    if (buffered_frame.cloud) {
        *merged += *buffered_frame.cloud;  // ← 所有历史帧在各自的原始坐标系中
    }
}
```

### 2.2 问题本质

```
时刻 t₀ (当前帧):  body 在 map 坐标系位姿 P₀
时刻 t₁ (历史帧):  body 在 map 坐标系位姿 P₁

cloud_t₀ 的坐标系原点 = body@t₀
cloud_t₁ 的坐标系原点 = body@t₁

直接拼接 = 假设 P₀ ≡ P₁（机器人没有移动）
```

当机器人移动时，`cloud_t₁` 的点在 `cloud_t₀` 坐标系中整体偏移了 `ΔP = P₀⁻¹ × P₁`，导致：

- **走廊场景**：墙壁点云产生"重影"，NDT 网格统计被污染
- **开放空间**：特征点空间分布发生畸变，NDT 梯度方向偏离
- **转弯时**：远端点的角误差最大，旋转保护不得不介入

### 2.3 运动补偿后的效果示意

```
补偿前（直接拼接）:
  cloud_t₀: ████████          ← 当前帧墙壁
  cloud_t₁:     ████████      ← 历史帧墙壁（已偏移0.5m）
  合并后:   ██████████████    ← 墙壁变"宽"了 → NDT网格方差增大 → 配准退化

补偿后（TF变换后再拼接）:
  cloud_t₀:       ████████
  cloud_t₁ (变换):████████     ← 已变换到当前坐标系
  合并后:         ████████    ← 墙壁精确重叠 → 点密度增加 → 配准精度提升
```

---

## 3. 方案一：运动补偿多帧累积（核心改造）

### 3.1 原理

利用 Fast-LIO 里程计提供的 `odom → body` TF，将每个历史帧从"该帧时刻的 body 坐标系"变换到"当前帧时刻的 body 坐标系"，再合并。

### 3.2 数学模型

```
对 buffer 中第 i 帧（时间戳 t_i ≠ t_curr）:
  1. 查 TF: T_odom_body_i = lookup("odom", "body", t_i)
  2. 查 TF: T_odom_body_curr = lookup("odom", "body", t_curr)
  3. 计算 delta: T_delta_i = T_odom_body_curr⁻¹ × T_odom_body_i
  4. 变换点云: cloud_i' = T_delta_i × cloud_i
  5. 拼入 merged cloud

最后对 merged cloud 做体素降采样（去重 + 均匀化）
```

### 3.3 为什么这个 T_delta 是可靠的低频信号

- Fast-LIO 在短时间窗口（1~2秒）内的相对漂移极小（<2cm平移，<0.5°旋转）
- `T_delta_i` 只关心相邻帧间的相对运动，不依赖全局定位精度
- 即使 Fast-LIO 长期漂移，帧间相对精度依然很高（这是 LIO 的核心优势）
- **我们已经在 Plan B（Fast-LIO delta guess）中使用了同一条 TF 链并验证了其可靠性**（[cpp:1581-1654](../src/lidar_localization/src/lidar_localization_component.cpp#L1581-L1654)）

### 3.4 数据结构改造

`ScanFrame` 结构体需要增加 TF 信息：

```cpp
// 当前结构体 (hpp:199-203)
struct ScanFrame {
    rclcpp::Time stamp;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
};

// 改造后
struct ScanFrame {
    rclcpp::Time stamp;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    Eigen::Affine3d T_odom_body;  // ★ 新增：该帧时刻的 odom→body 变换
    bool has_tf;                   // ★ 标记 TF 是否有效
};
```

### 3.5 参数建议

| 参数 | 当前值 | 建议值 | 理由 |
|------|--------|--------|------|
| `multi_frame_window_sec` | 0.6 | **1.5 ~ 2.0** | 运动补偿后无错位问题，可安全扩展 |
| `multi_frame_max_frames` | 8 | **20 ~ 30** | 15Hz下2秒=30帧，足够覆盖120°FOV旋转 |
| `multi_frame_voxel_leaf_size` | 0.20 | **0.15 ~ 0.20** | 更多点云可适当加密（可选） |
| `multi_frame_max_points` | 40000 | **80000** | 运动补偿后有效密度提升，值更多点 |

**保守档**（先验证稳定性）：
```
window = 1.0s, max_frames = 15, max_points = 60000
```

**激进档**（稳定性验证后）：
```
window = 2.0s, max_frames = 30, max_points = 100000
```

### 3.6 适用场景分析

| 场景 | 当前效果 | 运动补偿后效果 |
|------|---------|---------------|
| 静止 | ✓ 好（帧重叠） | ✓ 更好（更密且无冗余） |
| 直行 | △ 一般（错位模糊） | ✓ 好（壁面点云叠加增强） |
| 转弯 | ✗ 差（旋转保护触发） | ✓ 改善（历史帧可填补旋转后的FOV盲区） |
| 开阔空间 | △ 一般 | ✓ 好（累积更多特征点） |
| 退化场景（长走廊） | ✗ 差 | ✓✓ 显著改善（多帧累积增加横向约束） |

---

## 4. 方案二：关键帧选择策略（进阶优化）

### 4.1 原理

在方案一的基础上，不是把所有帧都加入 buffer，而是根据帧间运动量做关键帧筛选。避免静止时堆积大量完全重叠的冗余点云。

### 4.2 关键帧判定条件

```cpp
bool isKeyframe(const ScanFrame & last_keyframe, const ScanFrame & current) {
    // 计算帧间运动量
    Eigen::Affine3d T_delta = last_keyframe.T_odom_body.inverse() * current.T_odom_body;
    double dist = T_delta.translation().norm();
    double angle = Eigen::AngleAxisd(T_delta.rotation()).angle();

    // 任一超过阈值即为关键帧
    return (dist > keyframe_translation_threshold_) ||   // 建议 0.10m
           (angle > keyframe_rotation_threshold_);        // 建议 3° (0.052 rad)
}
```

### 4.3 参数建议

| 参数 | 建议值 | 说明 |
|------|--------|------|
| `keyframe_translation_threshold` | 0.10 ~ 0.15m | 平移超过此值才加入新帧 |
| `keyframe_rotation_threshold` | 0.05 ~ 0.10 rad (3°~6°) | 旋转超过此值才加入新帧 |

### 4.4 效果

- **静止时**：buffer 中只有 1~2 帧 → 计算量极小
- **直行 1m/s**：每 0.1m 一个关键帧 → 2s 窗口内约 20 帧（而非全量 30 帧）
- **转弯**：角度触发关键帧 → 确保转弯轨迹上的点云都被捕获
- **总体**：相同 buffer size 覆盖更长的空间距离，点云分布更均匀

### 4.5 与方案一的关系

方案二依赖方案一的 `T_odom_body` 字段来计算帧间运动量。如果实施周期紧张，**方案二可以暂缓，方案一单独落地已有显著收益**。

---

## 5. 方案三：NDT 参数联动优化

### 5.1 原理

运动补偿后源点云更密更准，NDT 匹配收敛条件改善，可以相应调整 NDT 参数以获得更好的精度。

### 5.2 参数建议

| 参数 | 当前v2值 | 配合方案一后建议 | 理由 |
|------|---------|-----------------|------|
| `ndt_resolution` | 0.5m | **0.3 ~ 0.4m** | 更密源点云支持更细的网格分辨率 |
| `ndt_outlier_ratio` | 0.30 | **0.25**（可选） | 匹配更准时可收紧离群容忍度 |
| `ndt_max_corr_dist` | 2.0m | 保持 | 当前值合理 |
| `ndt_max_iterations` | 50 | 保持 | 已足够充裕 |
| `ndt_rotation_prior_weight` | 10.0 | 保持或降至 **5.0** | 更密数据天然约束更好 |

### 5.3 注意

- NDT 参数调整应在方案一验证通过后单独进行，避免变量混杂
- 分辨率从 0.5m 降到 0.3m 会显著增加 NDT 网格数量和计算量（~4.6x），建议配合 `ndt_num_threads: 8` 保持实时性

---

## 6. 方案四：空间体素地图累积（远期架构升级）

### 6.1 原理

不按帧数/time window 管理，而是维护一个**空间体素网格结构**（类似 FAST-LIO2 的 ikd-Tree 的简化版），持续插入新点，旧的体素被覆盖或过期淘汰。

```
┌─────────────────────────────────┐
│  VoxelGridMap (0.2m resolution) │
│  ┌───┬───┬───┬───┬───┐         │
│  │ · │ · │   │ · │   │  每个体素存最近N个点
│  ├───┼───┼───┼───┼───┤  或仅存中心点
│  │   │ · │ · │ · │   │
│  ├───┼───┼───┼───┼───┤  新帧到达:
│  │ · │   │ · │ · │ · │  1. 变换到当前坐标
│  └───┴───┴───┴───┴───┘  2. 插入对应体素
│                          3. 淘汰过期体素
│  输出: 遍历所有体素 → 点云
└─────────────────────────────────┘
```

### 6.2 优势

- **内存恒定**：不受帧数/时间窗口影响
- **密度均匀**：每个体素点数可控，不会因停留而局部过密
- **天然去重**：同一空间位置只保留最新若干点
- **支持长期记忆**：可以设置较大的体素过期时间（如 5s），覆盖更广空间

### 6.3 劣势

- **实现复杂度高**：需要设计体素管理逻辑、过期策略、并发安全
- **需要大量点云插入操作**：每帧都要做 `transformPointCloud` + 体素查找 + 插入
- **调试难度**：可视化不如帧 buffer 直观

### 6.4 建议

方案四作为远期架构升级目标。短期内方案一+方案二性价比最高。

---

## 7. 推荐实施路径

### 阶段一：方案一落地（1-2天）

```
┌─ 改造 ScanFrame 结构体 ─────────────────────────────┐
│ 1. 新增 T_odom_body (Eigen::Affine3d) 字段          │
│ 2. 新增 has_tf 字段                                  │
└──────────────────────────────────────────────────────┘
                        ↓
┌─ 改造 buffer 填充逻辑 ───────────────────────────────┐
│ 3. push_back 前 lookup odom→body TF                  │
│ 4. TF lookup 失败 → 帧不加入 buffer（回退到单帧）    │
└──────────────────────────────────────────────────────┘
                        ↓
┌─ 改造 buildMultiFrameSource() 拼接循环 ──────────────┐
│ 5. 对每个历史帧做 transformPointCloud(T_delta_i)     │
│ 6. 当前帧（index=last）不需要变换                    │
│ 7. 保持后续的 voxel downsample + cap 逻辑不变        │
└──────────────────────────────────────────────────────┘
                        ↓
┌─ 调整参数 ────────────────────────────────────────────┐
│ 8. multi_frame_window_sec: 0.6 → 1.5                │
│ 9. multi_frame_max_frames: 8 → 25                    │
│ 10. multi_frame_max_points: 40000 → 80000            │
└──────────────────────────────────────────────────────┘
                        ↓
┌─ 验证 ────────────────────────────────────────────────┐
│ 11. bag回放测试，对比 fitness score 和稳定性         │
│ 12. 确认无 regression                                │
└──────────────────────────────────────────────────────┘
```

### 阶段二：方案二 + 方案三（1天）

- 方案二：加入关键帧筛选逻辑，进一步优化效率和点云分布
- 方案三：联动调整 NDT 分辨率等参数

### 阶段三（远期）：方案四

- 体素地图累积架构升级

---

## 8. 完整改造代码指南

> 以下为改造的关键代码段，标注了 `★ NEW` 的为新增/修改部分。

### 8.1 修改 `lidar_localization_component.hpp`

#### 8.1.1 扩展 ScanFrame 结构体 (hpp:199-203)

```cpp
// ★ 改造前
struct ScanFrame
{
    rclcpp::Time stamp;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
};

// ★ 改造后
struct ScanFrame
{
    rclcpp::Time stamp;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    Eigen::Affine3d T_odom_body;  // 该帧时刻的 odom→body 变换（用于运动补偿）
    bool has_tf;                   // TF 是否成功查得（失败则不参与多帧累积）
};
```

#### 8.1.2 新增成员变量 (在 multi_frame 参数区域，hpp:325-330 附近)

```cpp
// ★ NEW: 运动补偿相关参数
bool multi_frame_motion_compensation_enabled_{true};  // 是否启用运动补偿
double multi_frame_tf_timeout_sec_{0.1};               // 单帧TF lookup超时

// ★ NEW: 关键帧选择参数（方案二，可暂不加）
bool multi_frame_keyframe_filter_enabled_{false};
double multi_frame_keyframe_translation_threshold_{0.10};
double multi_frame_keyframe_rotation_threshold_{0.052}; // ~3°
```

### 8.2 修改 `lidar_localization_component.cpp`

#### 8.2.1 改造 `cloudReceived()` 中的 buffer 填充逻辑

在 `buildMultiFrameSource()` 被调用之前，需要先为当前帧查询 TF 并存入 buffer。

**位置**：在 `cloudReceived()` 中，`buildMultiFrameSource()` 调用之前（约 hpp:1487-1490 处）。

当前代码：
```cpp
const bool rotation_guard_active = rotationGuardActive();
const bool rotation_guard_settle = rotation_guard_settle_debug_;
pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud_ptr =
    buildMultiFrameSource(tmp_ptr, rclcpp::Time(msg->header.stamp), rotation_guard_active);
```

需要在调用 `buildMultiFrameSource()` 之前，为当前帧查询 TF 并 push 到 buffer：

```cpp
// ★ NEW: 在 buildMultiFrameSource 之前填充带 TF 信息的 ScanFrame
if (multi_frame_matching_enabled_) {
    ScanFrame frame;
    frame.stamp = rclcpp::Time(msg->header.stamp);
    frame.cloud.reset(new pcl::PointCloud<pcl::PointXYZI>(*tmp_ptr));
    frame.has_tf = false;

    // 查询当前帧的 odom→body TF（与 Plan B 使用同一条 TF 链）
    if (multi_frame_motion_compensation_enabled_) {
        geometry_msgs::msg::TransformStamped tf_odom_body;
        try {
            tf_odom_body = tfbuffer_.lookupTransform(
                odom_frame_id_, fastlio_body_frame_, frame.stamp);
            // 转换为 Eigen::Affine3d
            Eigen::Vector3d trans(
                tf_odom_body.transform.translation.x,
                tf_odom_body.transform.translation.y,
                tf_odom_body.transform.translation.z);
            Eigen::Quaterniond quat(
                tf_odom_body.transform.rotation.w,
                tf_odom_body.transform.rotation.x,
                tf_odom_body.transform.rotation.y,
                tf_odom_body.transform.rotation.z);
            frame.T_odom_body = Eigen::Affine3d::Identity();
            frame.T_odom_body.translate(trans);
            frame.T_odom_body.rotate(quat);
            frame.has_tf = true;
        } catch (const tf2::TransformException & ex) {
            // TF 查不到：帧仍加入 buffer，但不参与运动补偿
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Multi-frame: odom→body TF lookup failed for stamp=%.3f, "
                "frame added without motion compensation. (%s)",
                frame.stamp.seconds(), ex.what());
        }
    }

    scan_frame_buffer_.push_back(frame);
}
```

#### 8.2.2 改造 `buildMultiFrameSource()` 核心逻辑

**当前实现** ([cpp:1344-1411](../src/lidar_localization/src/lidar_localization_component.cpp#L1344-L1411))：

```cpp
pcl::PointCloud<pcl::PointXYZI>::Ptr PCLLocalization::buildMultiFrameSource(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr & current_cloud,
    const rclcpp::Time & stamp,
    bool rotation_guard_active)
{
    multi_frame_source_frames_debug_ = 1;
    multi_frame_source_points_debug_ = current_cloud ? static_cast<int>(current_cloud->size()) : 0;

    if (!current_cloud) {
        return current_cloud;
    }

    if (!multi_frame_matching_enabled_ ||
        (multi_frame_use_only_when_rotating_ && !rotation_guard_active)) {
        scan_frame_buffer_.clear();
        return current_cloud;
    }

    // ★ 注意：这里 push_back 和 prune 逻辑会被挪到外部
    // 改造后 buildMultiFrameSource 只负责：从 buffer 中取帧 → 运动补偿变换 → 合并 → 降采样

    ScanFrame frame;
    frame.stamp = stamp;
    frame.cloud.reset(new pcl::PointCloud<pcl::PointXYZI>(*current_cloud));
    scan_frame_buffer_.push_back(frame);
    // ... prune logic ...
    // ... merge + downsample + cap ...
}
```

**改造后**（prune + push 已移到 `cloudReceived()`，此函数聚焦于补偿+合并）：

```cpp
pcl::PointCloud<pcl::PointXYZI>::Ptr PCLLocalization::buildMultiFrameSource(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr & current_cloud,
    const rclcpp::Time & stamp,
    bool rotation_guard_active)
{
    multi_frame_source_frames_debug_ = 1;
    multi_frame_source_points_debug_ = current_cloud ? static_cast<int>(current_cloud->size()) : 0;

    if (!current_cloud) {
        return current_cloud;
    }

    if (!multi_frame_matching_enabled_ ||
        (multi_frame_use_only_when_rotating_ && !rotation_guard_active)) {
        scan_frame_buffer_.clear();
        return current_cloud;
    }

    // ★ 根据时间窗口和最大帧数裁剪 buffer
    while (!scan_frame_buffer_.empty()) {
        const double age = (stamp - scan_frame_buffer_.front().stamp).seconds();
        if (age <= multi_frame_window_sec_ &&
            static_cast<int>(scan_frame_buffer_.size()) <= multi_frame_max_frames_) {
            break;
        }
        scan_frame_buffer_.pop_front();
    }

    if (scan_frame_buffer_.size() <= 1) {
        return current_cloud;
    }

    // ==================== ★ NEW: 运动补偿 + 合并 ====================

    // 找到当前帧在 buffer 中的索引（最后插入的是当前帧）
    // 当前帧的 T_odom_body 作为参考坐标系
    const auto & ref_frame = scan_frame_buffer_.back();

    pcl::PointCloud<pcl::PointXYZI>::Ptr merged(new pcl::PointCloud<pcl::PointXYZI>());
    int valid_frame_count = 0;

    for (const auto & buffered_frame : scan_frame_buffer_) {
        if (!buffered_frame.cloud || buffered_frame.cloud->empty()) {
            continue;
        }

        // 判断是否需要变换：如果是最新帧（或 TF 无效），直接用原始点云
        bool is_current = (&buffered_frame == &scan_frame_buffer_.back());

        if (is_current || !multi_frame_motion_compensation_enabled_ ||
            !buffered_frame.has_tf || !ref_frame.has_tf) {
            // 无需变换：直接拼入
            *merged += *buffered_frame.cloud;
        } else {
            // ★ 核心：运动补偿变换
            // T_delta = T_odom_body_ref⁻¹ × T_odom_body_i
            // 含义：将历史帧从"历史时刻的body系"变换到"当前时刻的body系"
            Eigen::Affine3d T_delta = ref_frame.T_odom_body.inverse() * buffered_frame.T_odom_body;

            pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::transformPointCloud(*buffered_frame.cloud, *transformed, T_delta);
            *merged += *transformed;
        }
        valid_frame_count++;
    }

    // ==================== 降采样 + 点数上限（保持原逻辑）====================

    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::VoxelGrid<pcl::PointXYZI> multi_frame_filter;
    multi_frame_filter.setLeafSize(
        multi_frame_voxel_leaf_size_, multi_frame_voxel_leaf_size_, multi_frame_voxel_leaf_size_);
    multi_frame_filter.setInputCloud(merged);
    multi_frame_filter.filter(*downsampled);

    if (static_cast<int>(downsampled->size()) > multi_frame_max_points_) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr capped(new pcl::PointCloud<pcl::PointXYZI>());
        capped->reserve(multi_frame_max_points_);
        const double stride =
            static_cast<double>(downsampled->size()) / static_cast<double>(multi_frame_max_points_);
        for (int i = 0; i < multi_frame_max_points_; ++i) {
            const size_t index = std::min(
                downsampled->size() - 1,
                static_cast<size_t>(std::floor(static_cast<double>(i) * stride)));
            capped->push_back((*downsampled)[index]);
        }
        downsampled = capped;
    }

    multi_frame_source_frames_debug_ = valid_frame_count;
    multi_frame_source_points_debug_ = static_cast<int>(downsampled->size());
    return downsampled;
}
```

#### 8.2.3 改造策略说明

**Push/prune 职责分离**：

```
改造前: buildMultiFrameSource() 负责 push + prune + merge + downsample
改造后: cloudReceived() 负责 push (含TF查询)
        buildMultiFrameSource() 负责 prune + motion_compensate + merge + downsample
```

**为什么把 push 挪到 cloudReceived？**

因为 TF 查询需要 `tfbuffer_`（成员变量），在 `cloudReceived()` 中查询后存入 `ScanFrame.T_odom_body`，后续 `buildMultiFrameSource()` 直接用缓存的变换矩阵，避免重复查询 TF（也可能查不到最新时间戳的 TF）。

**第二种实现方式（更简洁，推荐）**：

如果不想改动太多调用逻辑，也可以保持 push 在 `buildMultiFrameSource()` 内，但在 push 之前查询 TF：

```cpp
// 在 buildMultiFrameSource() 内部，push_back 之前：
ScanFrame frame;
frame.stamp = stamp;
frame.cloud.reset(new pcl::PointCloud<pcl::PointXYZI>(*current_cloud));

// ★ NEW: 查询该帧的 odom→body TF
frame.has_tf = false;
if (multi_frame_motion_compensation_enabled_) {
    geometry_msgs::msg::TransformStamped tf_odom_body;
    try {
        tf_odom_body = tfbuffer_.lookupTransform(
            odom_frame_id_, fastlio_body_frame_, stamp);
        // ... 转换为 Eigen::Affine3d 存入 frame.T_odom_body ...
        frame.has_tf = true;
    } catch (...) { /* TF 不可用 */ }
}

scan_frame_buffer_.push_back(frame);
```

**两种方式的权衡**：

| 方式 | 优点 | 缺点 |
|------|------|------|
| push 在 cloudReceived | TF 上下文接近 Plan B 代码，逻辑清晰 | 需要修改 cloudReceived 调用链 |
| push 在 buildMultiFrameSource | 改动集中在一个函数内 | buildMultiFrameSource 增加了 TF 查询职责 |

**推荐采用第二种**（push 仍在 `buildMultiFrameSource` 内），改动最小、风险最低。

### 8.3 新增参数声明

在 `lidar_localization_component.hpp` 的 `on_configure` 参数声明区域添加：

```cpp
// ★ NEW: 运动补偿
this->declare_parameter<bool>("multi_frame_motion_compensation_enabled", true);
this->declare_parameter<double>("multi_frame_tf_timeout_sec", 0.1);

// ★ NEW: 关键帧过滤（方案二，可暂不加）
this->declare_parameter<bool>("multi_frame_keyframe_filter_enabled", false);
this->declare_parameter<double>("multi_frame_keyframe_translation_threshold", 0.10);
this->declare_parameter<double>("multi_frame_keyframe_rotation_threshold", 0.052);
```

### 8.4 启动文件参数更新

在 `navigation2_fusion_sc_v2.launch.py` L575-L580 区域：

```python
# ★ Scheme 2: multi-frame source cloud (运动补偿增强版)
'multi_frame_matching_enabled': True,
'multi_frame_use_only_when_rotating': False,
'multi_frame_window_sec': 1.5,           # 0.6 → 1.5 (运动补偿后可安全扩展)
'multi_frame_max_frames': 25,            # 8 → 25
'multi_frame_voxel_leaf_size': 0.20,
'multi_frame_max_points': 80000,         # 40000 → 80000
'multi_frame_motion_compensation_enabled': True,   # ★ 新增
'multi_frame_tf_timeout_sec': 0.1,                 # ★ 新增
# 方案二参数（暂不加）:
# 'multi_frame_keyframe_filter_enabled': False,
# 'multi_frame_keyframe_translation_threshold': 0.10,
# 'multi_frame_keyframe_rotation_threshold': 0.052,
```

---

## 9. 验证方案

### 9.1 离线 Bag 回放验证

使用已有的 `tools/replay_ndt_rotation_guard_batch.py` 和 `test_ndt_replay.launch.py`：

```bash
# 1. 准备测试 bag（选择包含直行、转弯、静止场景）
# 2. 运行改造前后的对比测试
# 3. 收集指标
```

### 9.2 关键观测指标

从 `/localization/ndt_status` JSON 中提取：

| 指标 | 含义 | 期望变化 |
|------|------|----------|
| `fitness_score` | NDT 匹配残差 | ↓ 下降（更好的匹配） |
| `has_converged` | 是否收敛 | 保持高比例 |
| `correction_translation` | 单帧修正量 | ↓ 下降（更稳定） |
| `correction_yaw` | 单帧修正量 | ↓ 下降 |
| `rejection_rate` | 被拒帧比例 | ↓ 下降 |
| `multi_frame_source_frames` | 实际使用的帧数 | ↑ 增加（窗口扩大） |
| `multi_frame_source_points` | 合并后的点数 | ↑ 增加 |
| `rotation_guard_hold_count` | 旋转保护触发次数 | ↓ 下降 |
| `ndt_mean_corr_dist` | NDT 平均关联距离 | ↓ 下降 |

### 9.3 A/B 对比测试矩阵

| 测试场景 | 当前配置 | 方案一保守档 | 方案一激进档 |
|---------|----------|-------------|-------------|
| 静止 30s | baseline | ← | ← |
| 直行 0.5 m/s | baseline | ← | ← |
| 直行 1.0 m/s | baseline | ← | ← |
| 原地旋转 90° | baseline | ← | ← |
| 行走 + 转弯 | baseline | ← | ← |
| 长走廊（退化） | baseline | ← | ← |
| 回环 | baseline | ← | ← |

### 9.4 回归检查清单

- [ ] NDT 收敛率不下降
- [ ] 平均 fitness score 不上升
- [ ] pose jump rejection 频率不增加
- [ ] 旋转保护触发频率不增加
- [ ] Fast-LIO delta guess 仍然正常工作
- [ ] 计算延迟在 100ms 以内（实时性）
- [ ] 内存使用在合理范围（<200MB 增量）
- [ ] 静止时不会因 buffer 溢出导致错误
- [ ] TF 树断裂时能优雅降级到单帧模式

---

## 附录 A：关键文件索引

| 文件 | 角色 |
|------|------|
| [lidar_localization_component.hpp](../src/lidar_localization/include/lidar_localization/lidar_localization_component.hpp) | 类定义、参数声明、ScanFrame 结构体 |
| [lidar_localization_component.cpp](../src/lidar_localization/src/lidar_localization_component.cpp) | 核心实现：buildMultiFrameSource (L1344)、cloudReceived (L1413)、Plan B (L1527) |
| [localization.yaml](../src/lidar_localization/param/localization.yaml) | 基础 NDT 参数 |
| [navigation2_fusion_sc_v2.launch.py](../src/humanoid_navigation2/launch/navigation2_fusion_sc_v2.launch.py) | v2 启动参数覆盖 (L575-L600) |
| [ndt_omp.h](../src/ndt_omp_ros2/include/pclomp/ndt_omp.h) | 自定义 NDT_OMP 实现 |
| [robosenseAiry.yaml](../src/fast_lio_robosense/config/robosenseAiry.yaml) | Fast-LIO / LiDAR 配置 |

## 附录 B：相关概念术语

| 术语 | 说明 |
|------|------|
| NDT (Normal Distributions Transform) | 将点云表示为高斯分布网格的配准算法 |
| Fast-LIO | 紧耦合 LiDAR-IMU 里程计，输出配准后点云 + 位姿 |
| Plan B (Fast-LIO delta guess) | 用 Fast-LIO 帧间运动推进 NDT 初始猜测 |
| Rotation guard | 检测到机器人旋转时冻结 NDT 修正 |
| Pose jump guard | 拒绝单帧过大的 NDT 修正 |
| Candidate confirmation | 对中等修正量要求连续帧一致性 |
| Voxel grid filter | 体素降采样，用于减少点云密度 |
| Fitness score | NDT 匹配残差，越低越好 |

---

> **下一步**：确认方案后，按"阶段一"路径开始编码实现。方案一的核心改动量约 50-80 行代码，风险可控。
