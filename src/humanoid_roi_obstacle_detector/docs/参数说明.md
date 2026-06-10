# 参数说明

配置文件：

```text
src/humanoid_roi_obstacle_detector/config/roi_obstacle_detector.yaml
```

所有参数都在 `roi_obstacle_detector.ros__parameters` 下。

## 输入和输出坐标

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `input_topic` | `/airy_points` | 输入点云 topic |
| `target_frame` | `base_footprint` | ROI 所在坐标系 |
| `source_frame_override` | `""` | 非空时覆盖 `PointCloud2.header.frame_id` |

`target_frame` 应该是机器人稳定坐标，推荐 `base_footprint`。不要直接在
`rslidar` 这种随身体晃动的 frame 下做 ROI。

## 点云变换

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `transform_mode` | `imu` | `tf`、`manual`、`imu`、`none` |
| `fallback_to_manual` | `false` | `tf` 查询失败时是否退回静态外参 |
| `tf_timeout_sec` | `0.05` | TF 查询超时 |
| `manual_translation_xyz` | `[0.07646, 0.00025, 1.21082]` | level source frame 到 `base_footprint` 的平移 |
| `manual_rotation_matrix` | 见 YAML | level source frame 到 `base_footprint` 的旋转矩阵 |
| `manual_rotation_rpy` | 见 YAML | 没有 `manual_rotation_matrix` 时使用 |

`transform_mode: imu` 的变换形式：

```text
p_base = manual_rotation_matrix * level_rotation * p_lidar + manual_translation_xyz
```

其中 `level_rotation` 来自 IMU 重力方向估计，只补偿 roll/pitch。

## IMU 姿态补偿

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `imu_topic` | `/airy_imu` | 输入 IMU topic |
| `imu_mode` | `accel` | `accel` 或 `orientation` |
| `imu_filter_mode` | `complementary` | `lowpass` 或 `complementary` |
| `source_to_imu_rotation_matrix` | 见 YAML | LiDAR 点云内部坐标到 IMU 内部坐标的旋转 |
| `imu_timeout_sec` | `0.3` | 点云时间戳匹配 IMU 样本的最大时间差 |
| `imu_accel_alpha` | `0.003` | 互补滤波中的加速度修正权重 |
| `imu_accel_is_up` | `true` | 静止时加速度是否指向局部 up 方向 |
| `imu_expected_accel_norm` | `9.80665` | 期望重力加速度模长 |
| `imu_accel_gate_sigma` | `3.0` | 加速度模长偏离重力时降低修正权重 |
| `imu_max_gyro_dt_sec` | `0.05` | 陀螺仪积分允许的最大 IMU 时间间隔 |
| `imu_gyro_bias_xyz` | `[0, 0, 0]` | 手动陀螺仪 bias |
| `estimate_gyro_bias` | `false` | 是否启动时估计陀螺仪 bias |
| `gyro_bias_estimation_sec` | `2.0` | bias 估计窗口 |
| `gyro_bias_min_samples` | `100` | bias 最少样本数 |
| `level_up_axis` | `[0, 0, 1]` | 目标坐标中的 up 方向 |

当前 Airy bag 里 IMU orientation 是固定 identity，所以默认不要改成
`imu_mode: orientation`，除非确认驱动真的输出了解算姿态。

调参建议：

- 点云姿态抖动明显：适当减小 `imu_accel_alpha`，让陀螺仪预测占比更高。
- 长时间慢漂：适当增大 `imu_accel_alpha`。
- 走动加速度冲击明显：增大 `imu_accel_gate_sigma` 会更相信加速度；减小则更严格。

## 地面 z 补偿

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `enable_ground_z_compensation` | `true` | 是否用点云估计地面高度并补偿上下起伏 |
| `ground_roi_min_x` | `0.25` | 地面估计区域 x 最小值 |
| `ground_roi_max_x` | `3.0` | 地面估计区域 x 最大值 |
| `ground_roi_min_y` | `-1.2` | 地面估计区域 y 最小值 |
| `ground_roi_max_y` | `1.2` | 地面估计区域 y 最大值 |
| `ground_roi_min_z` | `-0.45` | 地面候选 z 最小值 |
| `ground_roi_max_z` | `0.35` | 地面候选 z 最大值 |
| `ground_z_percentile` | `35.0` | 用候选点 z 的第几个百分位作为地面高度 |
| `ground_min_points` | `200` | 地面候选点最少数量 |
| `ground_filter_alpha` | `0.35` | ground_z 一阶滤波权重 |
| `ground_max_step` | `0.08` | 单帧 ground_z 最大变化 |
| `ground_z_offset` | `0.0` | 地面高度额外偏置 |

补偿形式：

```text
points[:, z] -= filtered_ground_z
```

如果前方没有地面点，节点会继续使用上一帧有效的 `filtered_ground_z`。

调参建议：

- 地面估计被障碍物拉高：降低 `ground_z_percentile`，或缩小 `ground_roi`。
- 地面估计跳变：降低 `ground_filter_alpha` 或 `ground_max_step`。
- 地面点不足：放大 `ground_roi_*`，或降低 `ground_min_points`。

## 前方 ROI

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `roi_min_x` | `0.45` | ROI 前向最小距离 |
| `roi_max_x` | `2.0` | ROI 前向最大距离 |
| `roi_min_y` | `-0.35` | ROI 左右范围最小值 |
| `roi_max_y` | `0.35` | ROI 左右范围最大值 |
| `roi_min_z` | `0.28` | ROI 离地高度最小值 |
| `roi_max_z` | `1.2` | ROI 离地高度最大值 |

坐标约定：

```text
x: forward
y: left
z: up
```

`roi_min_z` 是抑制地面误判的关键参数。启用 ground z 补偿后，它表示相对地面的高度。

## 聚类和投票

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `voxel_leaf_size` | `0.05` | ROI 点云体素降采样大小 |
| `min_points` | `10` | ROI 降采样后最少点数 |
| `use_clustering` | `true` | 是否计算最大连通 cluster |
| `cluster_min_size` | `20` | 最大 cluster 超过该值才认为当前帧检测到障碍 |
| `cluster_connectivity` | `26` | 体素连通域邻域，支持 `6` 或 `26` |
| `trigger_frames` | `3` | 连续多少帧检测到才置 `has_obstacle=true` |
| `clear_frames` | `5` | 连续多少帧未检测到才清除 |

调参建议：

- 误触发多：增大 `roi_min_z`、`cluster_min_size` 或 `trigger_frames`。
- 小障碍漏检：降低 `cluster_min_size`、`voxel_leaf_size` 或 `roi_min_z`。
- 状态闪烁：增大 `trigger_frames` 和 `clear_frames`。

## 发布和调试

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `has_obstacle_topic` | `/front_obstacle/has_obstacle` | Bool 输出 |
| `debug_topic` | `/front_obstacle/debug` | JSON 调试输出 |
| `roi_cloud_topic` | `/front_obstacle/roi_cloud` | ROI 调试点云输出 |
| `publish_roi_cloud` | `true` | 是否发布 ROI 点云 |
| `publish_debug` | `true` | 是否发布 debug JSON |
| `debug_publish_period_sec` | `0.5` | debug 发布周期 |
| `log_input_stats` | `true` | 是否周期性打印输入点云 bounds |

`/front_obstacle/debug` 常用字段：

| 字段 | 说明 |
| --- | --- |
| `raw_roi_count` | ROI 裁剪前的原始点数 |
| `voxel_count` | 体素降采样后的 ROI 点数 |
| `max_cluster_size` | 最大连通 cluster 点数 |
| `detected_now` | 当前帧是否检测到 |
| `has_obstacle` | 多帧投票后的稳定状态 |
| `hit_streak` | 当前连续命中帧数 |
| `clear_streak` | 当前连续清除帧数 |
| `ground_z` | 当前使用的地面 z 补偿量 |
| `ground_candidate_count` | 地面估计候选点数 |
