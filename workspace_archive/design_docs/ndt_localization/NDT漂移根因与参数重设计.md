# NDT 漂移根因与参数/代码改造评估方案

日期：2026-05-27  
工作区：`/home/ubuntu/humanoid_ws`  
背景：test3 导航中，原点大厅区域 NDT 较稳定；进入上方实验室房间后，尤其最里面两个拐角容易漂移。

## 1. 结论摘要

当前大逻辑改造（fusion/rusion 接管、停车、重定位）只能处理 NDT 已经漂移后的补救。真正要降低漂移概率，需要处理 NDT 输入语义和实验室局部地图质量问题。

本轮检查后的核心结论：

1. 不能直接打开 NDT 的 `use_odom: true`。
   - Fast-LIO 输出的 odom/点云处在非标准坐标系，坐标轴约定是 `x` 朝左、`y` 朝下、`z` 朝后。
   - 当前 NDT 的 `odomReceived()` 按 ROS 标准坐标系直接积分 twist。
   - 如果直接用 Fast-LIO odom，会把方向和角速度当成标准坐标处理，初值预测会错。

2. 当前 NDT 使用的是单分辨率 NDT。
   - 代码中只有一个 `registration_`。
   - 当前 launch/yaml 只配置一个 `ndt_resolution`。
   - 没有 coarse-to-fine 多分辨率搜索。

3. 当前 `/fast_lio/cloud_registered` 不是原始局部 scan，而是 Fast-LIO 世界系注册后的点云。
   - NDT 代码里对实时点云和 PCD 地图都做了同一个固定旋转到 ROS 标准方向。
   - source/target 坐标轴是一致的。
   - 但距离过滤 `scan_max_range` 当前按点到坐标原点的 XY 距离算，不是按点到机器人当前位姿的距离算。

4. 不能直接把 `scan_max_range` 从 100m 改成 15-20m。
   - 对局部雷达 scan，15-20m 是合理的。
   - 但对当前 world cloud，按原点裁剪会误删离原点较远区域的有效点。
   - 正确做法是先改距离过滤逻辑，用机器人当前位姿作为 scan origin，再设置真实传感器范围。

5. 实验室拐角漂移更像多因素叠加：
   - 实验室拐角几何可观测性弱。
   - 局部 PCD/栅格边界比大厅更毛、更碎。
   - 拐角/小房间结构重复，NDT 容易进入错误局部最优。
   - 单分辨率 NDT 缺少 coarse-to-fine 约束。
   - 当前 NDT 没有正确利用 Fast-LIO 短时 odom delta 做初值预测。

## 2. 当前地图与点云规模

### 2.1 静态栅格地图

文件：

```text
/home/ubuntu/humanoid_ws/src/humanoid_navigation2/maps/hall.yaml
/home/ubuntu/humanoid_ws/src/humanoid_navigation2/maps/hall.pgm
```

参数：

```yaml
resolution: 0.050
origin: [-6.648, -13.571, 0]
image size: 673 x 799 px
```

物理尺寸：

```text
width  = 673 * 0.05 = 33.65 m
height = 799 * 0.05 = 39.95 m
```

地图范围：

```text
x: -6.648 ~ 27.002
y: -13.571 ~ 26.379
```

占用统计：

```text
occupied cells: 20,157
free cells:     193,412
unknown cells:  324,158
```

观察：

- 大厅区域边界更连续、闭合。
- 上方实验室区域存在更多毛刺、未知区、射线痕迹和边界不闭合。
- 栅格未知区不直接影响 NDT，但说明当时扫描视角和建图质量在这些区域更弱。

### 2.2 3D PCD 地图

主 NDT 地图：

```text
/home/ubuntu/humanoid_ws/src/humanoid_navigation2/pcd/hall.pcd
```

原始 Fast-LIO 坐标范围：

```text
points: 4,068,088
x: -22.716 ~ 27.500
y: -7.911  ~ 16.242
z: -29.713 ~ 13.292
```

按当前 NDT 代码固定旋转到 ROS 标准方向后的近似范围：

```text
x: -13.292 ~ 29.713
y: -22.716 ~ 27.500
z: -16.242 ~ 7.911
```

其他 PCD：

```text
hall_standard.pcd:
  points: 3,267,778
  x: -8.418 ~ 29.914
  y: -25.990 ~ 27.082
  z: -12.104 ~ 14.217

hall_localization_grounded.pcd:
  points: 586,849
  x: -8.346 ~ 29.986
  y: -25.994 ~ 27.078
  z: 0.000 ~ 15.432
```

观察：

- PCD 覆盖范围大于 2D 栅格地图。
- 上方实验室区域并非完全缺 PCD。
- 但从 2D 栅格和 PCD 投影看，实验室边界更碎，内部结构更复杂，更容易让 NDT 局部匹配歧义。

## 3. 当前 NDT 数据链路

### 3.1 输入点云

launch 中：

```python
remappings=[('/cloud', '/fast_lio/cloud_registered')]
```

这说明 NDT 使用的是：

```text
/fast_lio/cloud_registered
```

该话题来自 Fast-LIO 内部配准后的世界系点云，不是原始 LiDAR 局部 scan。

### 3.2 PCD 与实时点云坐标转换

NDT 加载 PCD 时：

```cpp
Eigen::Quaternionf q_cam_to_ros(0.5, -0.5, -0.5, 0.5);
pcl::transformPointCloud(*map_cloud_ptr, *map_cloud_ptr, t_cam_to_ros, q_cam_to_ros);
```

NDT 接收实时点云时：

```cpp
Eigen::Quaternionf q_cam_to_ros(0.5, -0.5, -0.5, 0.5);
pcl::transformPointCloud(*cloud_ptr, *cloud_ptr, t_cam_to_ros, q_cam_to_ros);
```

因此：

```text
target PCD 和 source cloud_registered 都被同一旋转转换。
坐标轴方向上二者是一致的。
```

### 3.3 当前距离过滤的问题

当前代码：

```cpp
r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
if (scan_min_range_ < r && r < scan_max_range_) {
  tmp.push_back(p);
}
```

这个逻辑适用于“点云在雷达局部坐标系下”的情况，因为 `p.x/p.y` 是点相对雷达的坐标。

但现在输入是 Fast-LIO world cloud，`p.x/p.y` 更接近点在世界/地图中的坐标。此时：

```text
r = 点到地图/odom 原点的距离
```

而不是：

```text
r = 点到机器人当前雷达的距离
```

所以直接设置：

```yaml
scan_max_range: 20.0
```

可能会把机器人远离原点时看到的有效点误删。

当前 `scan_max_range: 100.0` 虽然不是真正的传感器范围，但能避免误删远离原点的区域点。

## 4. 当前 NDT 参数状态

基础 YAML：

```yaml
registration_method: "NDT_OMP"
ndt_resolution: 0.5
ndt_step_size: 0.1
ndt_max_iterations: 50
ndt_num_threads: 8
score_threshold: 1.0
voxel_leaf_size: 0.15
scan_max_range: 100.0
scan_min_range: 1.0
use_odom: false
use_imu: false
force_2d_pose: true
force_2d_fixed_z: true
force_2d_z: 0.0
```

`navigation2_fusion_sc.launch.py` 覆盖后，实际更接近：

```yaml
score_threshold: 0.3
reject_pose_jump: true
max_pose_jump_translation: 0.40
max_pose_jump_yaw: 0.30
initialpose_relax_duration_sec: 4.0
initialpose_max_pose_jump_translation: 2.00
initialpose_max_pose_jump_yaw: 1.20
pose_jump_reacquire_enabled: true
pose_jump_reacquire_max_translation: 0.80
pose_jump_reacquire_max_yaw: 0.30
pose_jump_reacquire_max_fitness: 0.08
pose_jump_reacquire_required_frames: 2
pose_jump_reacquire_xy_tolerance: 0.50
pose_jump_reacquire_yaw_tolerance: 0.25
min_scan_points: 50
republish_last_good_tf_on_failure: false
max_last_good_tf_age_sec: 0.5
```

结论：

```text
当前是单分辨率 NDT_OMP。
当前没有用 Fast-LIO odom 初值。
当前 score_threshold 已经很严格。
继续单纯收紧 score_threshold 不一定能解决低 fitness 错匹配。
```

## 5. 为什么大厅不飘，实验室拐角飘

### 5.1 大厅区域优势

大厅通常具备：

```text
空间大
远近墙面/曲面/柱子特征丰富
可见结构多
视野开阔
点云覆盖均匀
```

NDT 在这种场景下概率分布约束充分，不容易出现多个相似局部最优。

### 5.2 实验室拐角弱点

实验室最里面拐角通常具备：

```text
视野窄
只看到一两面墙
桌椅/柜子/门/玻璃等动态或半动态物体多
小房间结构重复
墙面长直且缺少垂直方向差异
角落扫描视角不足
```

NDT 在这些位置容易出现：

```text
沿墙方向滑移
跳到相似拐角
fitness 很低但位姿错误
correction_translation 突然变大
inlier_fraction 长期异常
```

### 5.3 建图缺漏的判断

从现有 PCD 投影和栅格图看：

```text
不是整块实验室没扫到。
更像是扫到了，但局部边界碎、角落视角不足、动态物体影响较多。
```

如果要确认最里面两个拐角是否真的缺漏，需要对指定拐角做局部统计：

```text
1. 在 RViz/CloudCompare 中看 hall.pcd。
2. 标出两个容易漂移的拐角坐标。
3. 对半径 2-3m 范围内统计 PCD 点密度、墙面连续性、是否有重影。
4. 把 test3 中漂移前后的实时 scan 叠到 PCD 上看是否一致。
```

## 6. 修改方案总览

建议按风险和收益分阶段，不要一次性改太多。

### Phase 0：确认当前坐标语义

目标：明确 NDT 里 source、target、pose、TF 的坐标关系。

检查项：

```text
1. /fast_lio/cloud_registered frame_id 是什么。
2. PCD hall.pcd 保存时的坐标系。
3. NDT 中 q_cam_to_ros 是否和 mapping launch 的 static TF 一致。
4. /pcl_pose 输出是否处于 map 标准坐标。
5. Nav2 使用的 map_ground/odom_ground 与 NDT map/odom 是否存在额外桥接。
```

验收：

```text
在 RViz 中显示:
  PCD map
  实时 cloud_registered 经 NDT 转换后的 source
  /pcl_pose
  robot model

机器人静止时，source 与 target 应在同一坐标系下重合。
```

### Phase 1：修正 scan range 语义

目标：让 `scan_min_range/scan_max_range` 表示点到机器人当前位姿的距离，而不是点到地图原点的距离。

当前错误语义：

```cpp
r = sqrt(p.x^2 + p.y^2);
```

建议改为：

```cpp
robot_x = current_pose.position.x;
robot_y = current_pose.position.y;

dx = p.x - robot_x;
dy = p.y - robot_y;
r = sqrt(dx * dx + dy * dy);
```

注意：

```text
这里的 p 和 current_pose 必须都在同一 ROS 标准 map/odom 坐标系下。
当前代码已经把 cloud 点云旋转到了 ROS 标准方向。
corrent_pose_with_cov_stamped_ptr_ 也用于 NDT init_guess，应保持同一坐标语义。
```

建议新增参数：

```yaml
range_filter_mode: "relative_to_pose"  # origin | relative_to_pose
scan_min_range: 0.5
scan_max_range: 20.0
```

兼容策略：

```text
默认保持 origin，避免破坏旧逻辑。
在新 launch 中显式设置 relative_to_pose。
```

推荐初始值：

```yaml
scan_min_range: 0.5
scan_max_range: 20.0
```

如果实验室小房间反射/远点影响仍大，可试：

```yaml
scan_max_range: 15.0
```

如果点数不足或门外远结构有帮助，可试：

```yaml
scan_max_range: 25.0
```

验收指标：

```text
filtered_points 不应低于 min_scan_points。
实验室拐角处 filtered_points 应稳定。
减少远处走廊/门外结构对局部匹配的影响。
```

### Phase 2：不要直接使用 Fast-LIO odom，改用标准化 delta 初值

目标：利用 Fast-LIO 短时运动约束，但不把非标准 odom 当 ROS 标准 odom。

禁止：

```yaml
use_odom: true
```

原因：

```text
当前 odomReceived() 按 ROS 标准 twist 积分。
Fast-LIO odom 坐标轴非标准。
直接启用会导致 delta 方向错误。
```

推荐新增逻辑：

```text
从 TF 查询 camera_init -> body。
保存上一帧 odom_body。
计算 odom_body_delta = inverse(prev_odom_body) * current_odom_body。
通过已有静态 TF/固定旋转把 delta 转为 ROS 标准平面 delta。
应用到 current pose，作为 NDT init_guess。
```

伪代码：

```cpp
if (use_fastlio_delta_guess_) {
  T_odom_body_now = lookupTransform("camera_init", "body");
  if (has_prev_odom_body) {
    T_delta = inverse(T_odom_body_prev) * T_odom_body_now;
    T_delta_ros = R_fastlio_to_ros * T_delta * R_fastlio_to_ros.inverse();
    current_pose_guess = current_pose_guess * planar(T_delta_ros);
  }
  T_odom_body_prev = T_odom_body_now;
}
```

建议参数：

```yaml
use_fastlio_delta_guess: true
fastlio_odom_parent_frame: "camera_init"
fastlio_odom_child_frame: "body"
fastlio_delta_max_translation: 0.5
fastlio_delta_max_yaw: 0.35
```

异常保护：

```text
如果 TF 查询失败，不更新 guess。
如果 delta 超过阈值，丢弃该 delta。
如果 NDT 已进入 recovery/reacquire，按 recovery 逻辑处理。
```

收益：

```text
NDT 每帧从更接近真实位置的初值开始优化。
实验室拐角局部极值吸引概率降低。
不会直接污染坐标系。
```

### Phase 3：多分辨率 NDT

目标：降低单分辨率 NDT 落入局部最优的概率。

当前：

```text
single registration_:
  ndt_resolution = 0.5
```

建议新增 coarse-to-fine：

```text
coarse NDT:
  resolution = 1.0
  voxel_leaf = 0.25
  max_iterations = 25

fine NDT:
  resolution = 0.5
  voxel_leaf = 0.15
  max_iterations = 40-60
```

流程：

```text
1. 使用 fastlio delta/current pose 得到 init_guess。
2. coarse NDT 先对齐，得到 coarse_result。
3. 如果 coarse fitness/correction 合理，用 coarse_result 作为 fine init。
4. fine NDT 精配准。
5. 最终结果仍走 pose jump、fitness、correction gates。
```

参数：

```yaml
multi_resolution_ndt: true
coarse_ndt_resolution: 1.0
coarse_voxel_leaf_size: 0.25
coarse_score_threshold: 0.8
coarse_max_iterations: 25

fine_ndt_resolution: 0.5
fine_voxel_leaf_size: 0.15
fine_score_threshold: 0.3
fine_max_iterations: 50
```

注意：

```text
多分辨率会增加 CPU 开销。
需要记录 align time，避免影响 10Hz 定位。
```

### Phase 4：局部地图质量检查与重建

目标：确认最里面两个拐角是否存在点云缺漏、重影或动态物体污染。

建议流程：

```text
1. 在 CloudCompare/RViz 中打开 hall.pcd。
2. 标出两个易漂拐角坐标。
3. 对每个拐角半径 2m/3m 区域统计:
   - 点数
   - 墙面连续性
   - 是否有双层墙/重影
   - 是否有桌椅/门/玻璃等动态物
4. 从 test3 bag 截取漂移前后的 /fast_lio/cloud_registered。
5. 和 PCD 同坐标叠加比较。
```

如果确认局部地图质量差，建议重新建图：

```text
1. 每个拐角原地慢速转一圈。
2. 从两个方向进出拐角。
3. 保持门、桌椅、设备状态和导航时一致。
4. 不只沿走廊中心线扫一遍。
5. 减少人员走动和临时障碍。
```

重建后验证：

```text
同一路线重复跑 3 次。
记录 NDT drift/rejected/pose_jump。
确认最里面两个拐角不再出现低 fitness 错匹配。
```

## 7. 推荐参数配置

### 7.1 不改代码的保守配置

适用于当前 world cloud 输入，不修改 range filter 和 odom guess。

```yaml
registration_method: "NDT_OMP"
ndt_resolution: 0.5
voxel_leaf_size: 0.15
ndt_step_size: 0.08
ndt_max_iterations: 60
transform_epsilon: 0.01
ndt_num_threads: 8

score_threshold: 0.3
reject_pose_jump: true
max_pose_jump_translation: 0.40
max_pose_jump_yaw: 0.30

scan_min_range: 0.0
scan_max_range: 100.0
min_scan_points: 50

use_odom: false
use_imu: false

force_2d_pose: true
force_2d_fixed_z: true
force_2d_z: 0.0

republish_last_good_tf_on_failure: false
max_last_good_tf_age_sec: 0.5
```

说明：

```text
scan_max_range 继续保持 100，是因为当前 range filter 是原点距离。
scan_min_range 设 0.0 或 0.5 都可以测试；如果 world cloud 近原点点导致异常，保持 1.0。
```

### 7.2 修正 range filter 后的推荐配置

适用于 `range_filter_mode: relative_to_pose`。

```yaml
registration_method: "NDT_OMP"
ndt_resolution: 0.5
voxel_leaf_size: 0.15
ndt_step_size: 0.08
ndt_max_iterations: 60

score_threshold: 0.3
reject_pose_jump: true
max_pose_jump_translation: 0.40
max_pose_jump_yaw: 0.30

range_filter_mode: "relative_to_pose"
scan_min_range: 0.5
scan_max_range: 20.0
min_scan_points: 80
```

可测试组：

```text
A: scan_max_range = 15.0
B: scan_max_range = 20.0
C: scan_max_range = 25.0
```

选择标准：

```text
实验室拐角处:
  filtered_points 足够
  fitness 不异常升高
  correction_translation 不突然变大
  pose_jump_candidate 减少
  不再沿墙滑移
```

### 7.3 增加 Fast-LIO delta guess 后的配置

```yaml
use_odom: false
use_fastlio_delta_guess: true
fastlio_odom_parent_frame: "camera_init"
fastlio_odom_child_frame: "body"
fastlio_delta_max_translation: 0.5
fastlio_delta_max_yaw: 0.35
```

NDT 参数可保持：

```yaml
ndt_resolution: 0.5
voxel_leaf_size: 0.15
score_threshold: 0.3
max_pose_jump_translation: 0.40
max_pose_jump_yaw: 0.30
```

### 7.4 多分辨率 NDT 配置

```yaml
multi_resolution_ndt: true

coarse_registration_method: "NDT_OMP"
coarse_ndt_resolution: 1.0
coarse_voxel_leaf_size: 0.25
coarse_ndt_max_iterations: 25
coarse_score_threshold: 0.8

fine_registration_method: "NDT_OMP"
fine_ndt_resolution: 0.5
fine_voxel_leaf_size: 0.15
fine_ndt_max_iterations: 50
fine_score_threshold: 0.3

multi_ndt_max_total_align_time_ms: 80
```

如果 CPU 压力太大：

```yaml
coarse_ndt_resolution: 1.2
fine_ndt_resolution: 0.6
fine_voxel_leaf_size: 0.18
```

## 8. 验证方案

### 8.1 Bag 离线验证

使用同一段 test3 bag，对不同参数组合重复跑。

统计：

```text
1. NDT accepted count
2. NDT rejected count
3. fitness_score p50/p90/p99/max
4. correction_translation p50/p90/p99/max
5. correction_yaw p50/p90/p99/max
6. pose_jump_candidate count
7. confirmed_pose_jump count
8. /pcl_pose frame-to-frame jump
9. align time p50/p90/p99
10. filtered_points p50/p90/min
```

重点窗口：

```text
实验室入口
实验室最里面两个拐角
点位 8-9
NDT 第一次 DEGRADED 前 10s
NDT 漂移后 10s
```

### 8.2 现场重复验证

路线：

```text
大厅原点区域 -> 实验室入口 -> 最里面两个拐角 -> 点位 8-9
```

每组参数至少跑 3 次。

通过条件：

```text
1. 不出现 robot pose 飞出地图。
2. 不出现连续错误 accepted。
3. 拐角处 correction_translation 不超过 0.3-0.4m。
4. pose_jump_candidate 明显减少。
5. recovery 触发次数减少。
6. Nav2 不再因定位跳变失败。
```

### 8.3 地图质量验证

需要生成两个局部可视化：

```text
1. PCD 投影到栅格地图的 overlay。
2. test3 漂移前后实时 cloud 与 PCD 的 overlay。
```

判断：

```text
如果 PCD 局部缺口明显:
  先重建图，再调 NDT。

如果 PCD 覆盖完整但 NDT 仍跳:
  优先上 fastlio delta guess + 多分辨率 NDT。

如果当前 scan 和 PCD 结构不一致:
  检查动态物体、门状态、桌椅、玻璃反射。
```

## 9. 实施优先级

推荐顺序：

```text
P0: 不改代码，只统一当前实际参数并做 baseline 统计。
P1: 修正 range_filter_mode=relative_to_pose。
P2: 增加 Fast-LIO delta guess，替代 use_odom。
P3: 做实验室拐角 PCD 局部质量检查，必要时重建图。
P4: 实现多分辨率 NDT。
P5: 再评估是否需要引入 GICP 或局部子图匹配。
```

不建议顺序：

```text
1. 直接打开 use_odom: true。
2. 直接把 scan_max_range 改成 15-20m。
3. 单纯继续降低 score_threshold。
4. 在没有局部地图质量检查前大幅改 NDT resolution。
```

## 10. 需要注意的代码点

### 10.1 `odomReceived()`

位置：

```text
src/lidar_localization/src/lidar_localization_component.cpp
```

当前逻辑按 ROS 标准 odom twist 积分，不适合直接接 Fast-LIO 非标准 odom。

### 10.2 `cloudReceived()` 距离过滤

当前逻辑：

```cpp
r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
```

需要支持：

```cpp
range_filter_mode == "relative_to_pose"
```

### 10.3 `initializeRegistration()`

当前只初始化一个 `registration_`，说明是单分辨率 NDT。

多分辨率需要增加：

```text
coarse_registration_
fine_registration_
```

或复用 registration 但每帧切换 target/source/参数；后者实现复杂且易错，不推荐。

## 11. 技术评估结论

当前 NDT 漂移不是简单参数问题。更准确地说，它是：

```text
实验室局部地图/几何弱
+ world cloud 输入导致 scan range 语义不正确
+ Fast-LIO odom 非标准，不能直接作为 NDT odom
+ 单分辨率 NDT 易陷入局部最优
```

短期建议：

```text
保持 use_odom=false。
保持 scan_max_range=100，直到 range filter 修正。
使用当前 score_threshold=0.3 + pose_jump gate。
先做 baseline 统计和局部地图质量检查。
```

中期建议：

```text
修正 range filter。
加入 Fast-LIO delta guess。
重建或补扫实验室最里面两个拐角。
```

长期建议：

```text
多分辨率 NDT + SC/HDL recovery + fusion/rusion 受控接管。
```
