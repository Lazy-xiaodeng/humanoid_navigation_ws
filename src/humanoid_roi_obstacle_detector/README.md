# humanoid_roi_obstacle_detector

人形机器人前方 ROI 避障节点。它直接订阅 LiDAR 点云，不依赖
Fast-LIO 里程计，也不订阅 `/odom`。

核心目标是把晃动的雷达点云转换到稳定的机器人坐标下，再做前方
占用检测：

```text
/airy_points
  -> IMU 互补滤波补偿 roll/pitch
  -> 静态外参转换到 base_footprint
  -> 点云估计 ground_z 并补偿上下起伏
  -> front ROI
  -> voxel + cluster
  -> 连续帧投票
  -> /front_obstacle/has_obstacle
```

## 适用场景

- 人形机器人走动时雷达随身体俯仰、横滚、上下起伏。
- 现场没有稳定的 `rslidar -> base_footprint` TF。
- 只需要前方是否有障碍物，不需要目标跟踪或建图。
- 输入点云来自原始雷达，例如 `/airy_points`。

不适合直接解决 yaw 漂移和机器人前后/左右平移；这个节点只补偿
roll/pitch 和相对地面的 z 高度。

## 输入输出

默认输入：

| Topic | Type | 说明 |
| --- | --- | --- |
| `/airy_points` | `sensor_msgs/msg/PointCloud2` | 原始 LiDAR 点云 |
| `/airy_imu` | `sensor_msgs/msg/Imu` | 雷达 IMU。当前 bag 中 orientation 固定为 identity，因此默认使用 accel + gyro 互补滤波 |

默认输出：

| Topic | Type | 说明 |
| --- | --- | --- |
| `/front_obstacle/has_obstacle` | `std_msgs/msg/Bool` | 多帧投票后的稳定避障结果 |
| `/front_obstacle/roi_cloud` | `sensor_msgs/msg/PointCloud2` | ROI 内、体素降采样后的调试点云 |
| `/front_obstacle/debug` | `std_msgs/msg/String` | JSON 调试信息，包含 ROI 点数、cluster、ground_z、输入/输出 bounds |

## 快速运行

构建：

```bash
colcon build --packages-select humanoid_roi_obstacle_detector
source install/setup.bash
```

启动：

```bash
ros2 launch humanoid_roi_obstacle_detector roi_obstacle_detector.launch.py
```

常用覆盖参数：

```bash
ros2 launch humanoid_roi_obstacle_detector roi_obstacle_detector.launch.py \
  input_topic:=/airy_points \
  target_frame:=base_footprint
```

播放 bag 测试：

```bash
ros2 bag play clearing2
```

查看结果：

```bash
ros2 topic echo /front_obstacle/has_obstacle
ros2 topic echo /front_obstacle/debug
```

RViz 建议显示：

- `PointCloud2`: `/airy_points`
- `PointCloud2`: `/front_obstacle/roi_cloud`
- Fixed Frame: `base_footprint`

## 变换模式

`transform_mode` 有四种：

| 模式 | 用途 |
| --- | --- |
| `tf` | 使用 TF 查询点云 frame 到 `target_frame` |
| `manual` | 只使用配置里的静态外参 |
| `imu` | 使用 IMU 补 roll/pitch，再套静态外参。当前推荐 |
| `none` | 不变换，ROI 直接解释在输入点云 frame 下 |

当前配置默认：

```yaml
transform_mode: "imu"
input_topic: "/airy_points"
target_frame: "base_footprint"
imu_topic: "/airy_imu"
imu_filter_mode: "complementary"
enable_ground_z_compensation: true
```

静态外参来自 Airy RoboSense/Fast-LIO 外参和记录到的
`body -> base_footprint` 静态 TF。换机器人或换雷达安装位置时，
必须重新填写：

```yaml
manual_translation_xyz: [...]
manual_rotation_matrix: [...]
source_to_imu_rotation_matrix: [...]
```

## 当前 ROI 参数

这组参数按 `clearing2` 走动 bag 调过，目标是抑制地面和近身误触发：

```yaml
roi_min_x: 0.45
roi_max_x: 1.0
roi_min_y: -0.35
roi_max_y: 0.35
roi_min_z: 0.28
roi_max_z: 1.2
cluster_min_size: 20
trigger_frames: 3
clear_frames: 5
```

## 离线验证和可视化

生成当前效果动画：

```bash
python3 tools/visualize_clearing_roi.py \
  --bag clearing2 \
  --output out/roi_visualization_latest \
  --gif \
  --gif-stride 8
```

生成未矫正和矫正后的对比动画：

```bash
python3 tools/visualize_roi_correction_compare.py \
  --bag clearing2 \
  --output out/roi_visualization_latest/no_correction_vs_imu_ground.gif \
  --stride 8
```

生成量化诊断图：

```bash
python3 tools/analyze_roi_correction_effect.py \
  --bag clearing2 \
  --output-dir out/roi_ground_compensation_latest
```

更多验证流程见 [docs/rosbag_validation.md](docs/rosbag_validation.md)。

## 调参顺序

推荐按这个顺序调，不要一开始就改所有参数：

1. 确认输入点云 frame、IMU topic 和时间戳正常。
2. 确认 `manual_rotation_matrix` 和 `manual_translation_xyz` 让点云大体落到 `base_footprint`。
3. 打开 `transform_mode: imu`，观察地面在 XZ 图里是否接近水平。
4. 打开 `enable_ground_z_compensation`，确认地面 z 稳定在 0 附近。
5. 再调 `roi_*`，让 ROI 覆盖机器人真正需要避障的空间。
6. 最后调 `cluster_min_size`、`trigger_frames`、`clear_frames` 抑制闪烁。

完整参数说明见 [docs/parameters.md](docs/parameters.md)。

## 常见问题

**为什么 IMU orientation 没用？**

当前 bag 中 `/airy_imu.orientation` 一直是 `[0, 0, 0, 1]`，不是解算好的姿态。
所以默认使用加速度估计重力方向，再用陀螺仪做互补滤波。

**为什么矫正前后动画差别不大？**

`clearing2` 中补偿后的 roll/pitch 幅度大多只有几度，点云位移多数是厘米级。
矫正的价值主要是让 ROI 高度和地面关系更稳定，避免走路时地面/低矮点误进 ROI。

**能不能用 IMU 加速度积分补上下移动？**

不建议。加速度二次积分漂移很快。当前实现用点云估计 `ground_z`，再把整帧点云
沿 z 方向归一化到地面附近。

**为什么不直接使用 tracking 项目？**

当前任务只需要稳定坐标系下的前方占用检测。tracking 会引入更多状态和参数，
但不解决人形机器人雷达晃动这个核心问题。
