# Rosbag 验证和可视化

本项目附带三个离线脚本，用于在不启动 ROS 节点的情况下验证 ROI 避障效果。
这些脚本直接读取 ROS 2 bag，复用配置文件中的外参、IMU 滤波、ground_z 补偿和 ROI 参数。

## 数据要求

bag 至少需要包含：

| Topic | Type | 说明 |
| --- | --- | --- |
| `/airy_points` | `sensor_msgs/msg/PointCloud2` | 原始点云 |
| `/airy_imu` | `sensor_msgs/msg/Imu` | 雷达 IMU |

默认脚本使用 `clearing2`：

```text
clearing2/
```

如果 bag 名不同，用 `--bag` 指定路径。

## 生成当前效果动画

```bash
python3 tools/visualize_clearing_roi.py \
  --bag clearing2 \
  --output out/roi_visualization_latest \
  --gif \
  --gif-stride 8
```

输出：

| 文件 | 说明 |
| --- | --- |
| `timeline.png` | ROI 点数、最大 cluster、稳定 `has_obstacle` 时间线 |
| `clearing_roi_preview.gif` | 当前矫正链路下的 XY/XZ 动画 |
| `frame_*.png` | 指定时刻和峰值时刻的单帧图 |

单帧图四个面板：

| 面板 | 说明 |
| --- | --- |
| A | 原始 `rslidar` XY |
| B | 只套静态外参后的 `base_footprint` XY |
| C | IMU + ground_z 补偿后的 `base_footprint` XY |
| D | IMU + ground_z 补偿后的 `base_footprint` XZ |

红框是 ROI，红点是落入 ROI 的点。

## 生成矫正前后对比动画

```bash
python3 tools/visualize_roi_correction_compare.py \
  --bag clearing2 \
  --output out/roi_visualization_latest/no_correction_vs_imu_ground.gif \
  --stride 8
```

动画左侧是只套静态外参的点云，右侧是 IMU + ground_z 补偿后的点云。
它用于确认补偿方向是否合理，不用于评估最终避障状态。

## 生成量化诊断图

```bash
python3 tools/analyze_roi_correction_effect.py \
  --bag clearing2 \
  --output-dir out/roi_ground_compensation_latest
```

输出：

| 文件 | 说明 |
| --- | --- |
| `correction_effect_timeline.png` | tilt、点云位移、ROI cluster 和状态差异时间线 |
| `correction_delta_frame_*.png` | 矫正影响最大的单帧 overlay |

终端会打印类似：

```text
clouds=1315 used=1315
imu_orientation_unique_first1000=1
filtered_tilt: min 0.097 deg, med 1.544 deg, p95 3.137 deg, max 4.054 deg
ground_z: min -0.036 m, med 0.006 m, p95 0.036 m, max 0.060 m
stable_state_frames manual/corrected/different: 367/368/3
```

字段解释：

| 字段 | 说明 |
| --- | --- |
| `imu_orientation_unique_first1000` | 前 1000 个 IMU orientation 是否变化。`1` 表示 orientation 没有可用姿态 |
| `filtered_tilt` | 互补滤波估计出的动态 roll/pitch 补偿角 |
| `ground_z` | 点云地面高度补偿量 |
| `visible_shift_*` | 矫正前后前方可视点云的位移 |
| `stable_state_frames` | 未矫正/矫正后稳定触发帧数，以及两者不同的帧数 |

## 推荐验证流程

1. 先生成 `no_correction_vs_imu_ground.gif`，确认矫正后地面在 XZ 里更接近水平且接近 `z=0`。
2. 再看 `timeline.png`，确认 bag 开头无障碍时没有稳定误触发。
3. 看 `correction_effect_timeline.png`，确认 `ground_z` 没有异常跳变。
4. 如果误触发集中在低高度，优先调高 `roi_min_z`。
5. 如果误触发来自稀疏噪点，优先调大 `cluster_min_size` 或 `trigger_frames`。

## 常见异常

**脚本提示找不到 topic**

检查 bag 中 topic 名称：

```bash
ros2 bag info clearing2
```

然后改配置文件中的 `input_topic` 和 `imu_topic`。

**矫正后地面反而更歪**

优先检查：

- `source_to_imu_rotation_matrix` 是否是 LiDAR 到 IMU 的旋转。
- `imu_accel_is_up` 是否符合驱动定义。
- `manual_rotation_matrix` 是否是 level source 到 `base_footprint`。

**ground_z 跳变大**

优先检查地面估计窗口：

- 缩小 `ground_roi_min_y/max_y`，避免扫到墙或腿部。
- 降低 `ground_z_percentile`，让估计更偏向低处地面。
- 降低 `ground_filter_alpha` 或 `ground_max_step`。

**可视化和 ROS 节点结果不一致**

确认两边使用同一个配置文件。离线脚本默认读取：

```text
src/humanoid_roi_obstacle_detector/config/roi_obstacle_detector.yaml
```

ROS launch 默认使用安装后的 share 目录配置。修改源码配置后需要重新构建并 source：

```bash
colcon build --packages-select humanoid_roi_obstacle_detector
source install/setup.bash
```
