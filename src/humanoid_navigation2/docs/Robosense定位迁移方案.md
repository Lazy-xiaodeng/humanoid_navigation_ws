# RoboSense lidar_localization 移植方案

目标：把 `/home/ubuntu/exp_code/robosense_localization` 里的 RoboSense 点云全局定位方法移植到当前 `/home/ubuntu/humanoid_ws`，作为一个新的、可独立开关的定位功能包；在 `humanoid_navigation2` 中新增 launch 文件启动它，用于和当前 Open3D prior-map 方法对比；不修改现有 `navigation2.launch.py`。

## 1. 现状判断

外部仓库是一个独立 ROS 工作空间，包含：

- `lidar_localization`：点云地图定位模块，PCL 点云处理，ceres/eigen 优化，读取 PCD 地图。
- `msf_localization`：ESKF 融合定位模块，README 中说明依赖 IMU、轮速、点云定位结果。

当前对比 Open3D 方法只需要先移植 `lidar_localization`。`msf_localization` 不建议第一阶段移植，因为当前导航栈已经由 Fast-LIO `/odom` 提供相对位姿，且现有 Nav2 架构已有 `prior_map_odom_bridge` 统一负责 `map->odom`。

外部 `lidar_localization` 当前行为：

- 输入点云：`/fast_lio/cloud_registered`
- 输入相对位姿：`/odom`
- 输出定位结果：`/lidar_pose_xyz`，类型 `nav_msgs/Odometry`
- 只发布定位结果与精修候选；`map -> odom` 固定由 `prior_map_odom_bridge` 独占发布
- 配置文件：硬编码读取 `PROJECT_DIR/config/config.yaml`
- 地图路径：代码里用 `PROJECT_DIR + map_path` 拼接，所以 `map_path: /map/xxx.pcd` 实际指向包目录下的 `map/xxx.pcd`

当前工作空间已有 `src/lidar_localization`，其包名是 `lidar_localization_ros2`，属于旧 NDT 定位方案。外部 RoboSense 包的原始包名是 `lidar_localization`，不能直接覆盖或混入现有包，否则容易造成包名、可执行文件、launch 和库名混淆。

## 2. 推荐架构

推荐采用“新包隔离 + 新 launch 编排 + bridge 统一 TF”的方式：

```text
/airy_points, /airy_imu
        |
        v
fast_lio_robosense
        |
        +--> /odom
        +--> /fast_lio/cloud_registered
                  |
                  v
robosense_lidar_localization
        |
        +--> /robosense_localization/odom 或 /lidar_pose_xyz
                  |
                  v
prior_map_odom_bridge
        |
        v
map -> odom
        |
        v
Nav2
```

核心原则：

- RoboSense 定位节点只作为“先验地图定位候选输出”，不要直接成为 TF owner。
- `map->odom` 仍由 `prior_map_odom_bridge` 发布，这样 Open3D 和 RoboSense 两套方法走同一个跳变保护、2D 投影、状态监控和 Nav2 接口。
- 新增 `navigation2_robosense_localization.launch.py`，不要改 `navigation2.launch.py`。
- 保留现有 Open3D launch 作为基线，RoboSense launch 只用于实验对比。

## 3. 包命名与目录规划

新增包建议命名为：

```text
src/robosense_lidar_localization/
```

不要继续使用外部原名 `lidar_localization`，原因：

- 当前工作空间已经有 `src/lidar_localization`，包名为 `lidar_localization_ros2`。
- RoboSense 原包可执行文件也叫 `lidar_localization_node`，直接引入会让调试时很难区分。
- 新包名可以清楚表达这是 RoboSense 方法，便于后续和 Open3D、HDL、ScanContext 并列管理。

建议目录：

```text
src/robosense_lidar_localization/
  CMakeLists.txt
  package.xml
  config/
    robosense_lidar_localization.yaml
  launch/
    robosense_lidar_localization.launch.py
  map/
    hall_open3d_grounded.pcd 或 robosense 专用地图
  common/
  lidar_matcher/
  lidar_localization/
  node/
```

`common/`、`lidar_matcher/`、`lidar_localization/`、`node/` 从外部仓库的 `lidar_localization` 目录复制。不要复制外部工作空间的 `build/`、`install/`、`log/`、`out/`。

## 4. 必要代码适配

### 4.1 包名与 CMake

把原始包名从 `lidar_localization` 改为 `robosense_lidar_localization`：

- `package.xml` 的 `<name>`
- 顶层 `CMakeLists.txt` 的 `project(...)`
- 子目录 install 目标路径中的 `${PROJECT_NAME}`

建议同时把安装库目标改名，避免和其他包里可能存在的 `liblidar_localization.so`、`liblidar_matcher.so` 冲突：

- `lidar_matcher` -> `robosense_lidar_matcher`
- `lidar_localization` -> `robosense_lidar_localization_core`
- executable 可保留 `lidar_localization_node`，但 launch 中节点名用 `robosense_lidar_localization_node`

### 4.2 配置文件读取

原代码硬编码：

```cpp
std::string config_file = std::string(PROJECT_DIR) + "/config/config.yaml";
```

建议改成 ROS2 参数：

```cpp
auto nh = rclcpp::Node::make_shared("robosense_lidar_localization_node");
nh->declare_parameter<std::string>("config_file", std::string(PACKAGE_PATH) + "/config/robosense_lidar_localization.yaml");
std::string config_file = nh->get_parameter("config_file").as_string();
```

这样 launch 可以显式传入 config 路径，后续调参不需要改源码。

### 4.3 地图路径

原代码把 `PROJECT_DIR` 和 `map_path` 直接拼接，这对安装后的 share 路径和绝对路径都不稳。建议支持两种形式：

- 绝对路径：`/home/ubuntu/humanoid_ws/src/humanoid_navigation2/pcd/hall_open3d_grounded.pcd`
- 包内相对路径：`map/hall_open3d_grounded.pcd`

规则：

- 如果 `map_path` 是绝对路径，直接使用。
- 如果是相对路径，则拼到 `PACKAGE_PATH` 或 package share 目录。

这样同一份 launch 可以切换 Open3D 使用的 grounded 地图和 RoboSense 专用地图。

### 4.4 禁止 RoboSense 节点直接发布 map->odom

配置里设置：

```yaml
# map -> odom 由 prior_map_odom_bridge 独占发布，RoboSense 节点不提供该开关
map_frame_id: "map"
odom_frame_id: "odom"
base_frame_id: "base_footprint"
```

原因：如果 RoboSense 节点和 `prior_map_odom_bridge` 同时发布 `map->odom`，TF 会出现双发布，Nav2 行为不可控。

### 4.5 输出话题规范化

RoboSense 原始输出是 `/lidar_pose_xyz`。为了和 Open3D 对比更清晰，建议通过代码或 launch remap 改成：

```text
/prior_localization/robosense_odom
```

语义：

- 类型：`nav_msgs/Odometry`
- `header.frame_id = map`
- `child_frame_id = base_footprint`
- `pose = map -> base_footprint`

然后在 `localization_runtime.yaml` 的 `prior_map_odom_bridge.ros__parameters` 中配置：

```text
prior_odom_topic: /prior_localization/robosense_odom
localized_frame: base_footprint
```

如果后续发现 RoboSense 输出实际是 `map -> body` 或其他中间 frame，则不要强行用 `base_footprint`，需要新增一个虚拟 frame，例如 `robosense_localized_base`，并发布对应 `odom -> robosense_localized_base` 或调整 bridge 的 `localized_frame`。

## 5. 新增 navigation2 launch 方案

新增文件：

```text
src/humanoid_navigation2/launch/navigation2_robosense_localization.launch.py
```

它不要复制整个 `navigation2.launch.py` 后硬改，而应尽量复用已有模块并保持实验入口清晰。推荐包含这些部分：

- FastDDS 环境变量
- 雷达驱动 `rslidar_sdk_node`
- Fast-LIO `fastlio_mapping`
- 必要静态 TF：`odom -> camera_init`、`body -> base_footprint`、`base_footprint -> clearing_lidar`
- `map_server`
- `robosense_lidar_localization.launch.py`
- `prior_map_odom_bridge`
- `robot_realpose_publisher`
- Nav2 planner/controller/bt_navigator/lifecycle

启动时序建议：

```text
0.0s  rslidar + Fast-LIO + static TF
1.0s  map_server
3.0s  map_server lifecycle
4.0s  robosense_lidar_localization
5.5s  prior_map_odom_bridge
7.5s  robot_realpose_publisher
bridge 首次接受定位后 3.0s 启动 Nav2 lifecycle
```

launch 只保留启动场景参数：

```text
use_sim_time:=false
robosense_config_file:=<share>/robosense_lidar_localization/config/robosense_lidar_localization.yaml
robosense_map_path:=/home/ubuntu/humanoid_ws/src/humanoid_navigation2/pcd/hall_open3d_grounded.pcd
localization_runtime_config_file:=<share>/humanoid_localization_runtime/config/localization_runtime.yaml
```

RoboSense 节点已从代码层移除 `map->odom` 广播能力；输入话题、帧名和门控参数统一由上述 YAML 管理。

## 6. 与 Open3D 的公平对比方式

为了让对比只反映定位算法差异，建议保持以下输入一致：

- 同一份 PCD 地图，优先用当前 Open3D 的 `hall_open3d_grounded.pcd`
- 同一条 Fast-LIO `/odom`
- 同一条点云输入 `/fast_lio/cloud_registered`
- 同一个 `prior_map_odom_bridge` 参数集
- 同一个 Nav2 参数文件 `nav2_params_xy_yaw.yaml`
- 同一个 2D 栅格地图 `maps/hall.yaml`

对比记录：

- `/prior_localization/robosense_odom`
- `/prior_localization/odom`，Open3D 输出
- `/tf` 中 `map->odom`
- `/robot_realpose`
- `/navigation/status`
- `/cmd_vel`

评估指标：

- 初始化成功时间
- 正常行走时 `map->odom` 修正幅度
- 到点旋转阶段是否发生跳变
- 失配后的恢复时间
- 与 Open3D 结果的 XY/Yaw 偏差
- CPU 占用和点云频率影响

## 7. 分阶段实施步骤

### 阶段 A：离线移植编译

1. 新建 `src/robosense_lidar_localization`。
2. 复制外部 `lidar_localization` 源码，不复制外部工作空间产物。
3. 改包名、CMake、install 规则。
4. 增加 launch 和 config。
5. 修改 config/map 路径读取逻辑。
6. `colcon build --packages-select robosense_lidar_localization`。

验收：

```bash
ros2 pkg executables robosense_lidar_localization
ros2 launch robosense_lidar_localization robosense_lidar_localization.launch.py
```

### 阶段 B：单节点 smoke test

只启动 Fast-LIO + RoboSense 定位，不启动 Nav2。

检查：

```bash
ros2 topic hz /prior_localization/robosense_odom
ros2 topic echo /prior_localization/robosense_odom --once
ros2 run tf2_ros tf2_echo odom base_footprint
```

要求：

- RoboSense 节点不发布 `map->odom`
- `/prior_localization/robosense_odom` 有稳定输出
- frame 语义明确，时间戳和 `/odom` 能对齐

### 阶段 C：接入 bridge

启动 `prior_map_odom_bridge`，并在定位运行层 YAML 中确认：

```text
prior_odom_topic: /prior_localization/robosense_odom
localized_frame: base_footprint
require_confidence: false
force_2d: true
```

第一轮建议 `jump_protection_mode=monitor`，先观察日志，不拦截真实 TF。

验收：

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 topic echo /robot_realpose --once
```

### 阶段 D：新增 Navigation2 实验 launch

新增：

```text
src/humanoid_navigation2/launch/navigation2_robosense_localization.launch.py
```

运行：

```bash
ros2 launch humanoid_navigation2 navigation2_robosense_localization.launch.py use_sim_time:=false
```

验收：

- 原 `navigation2.launch.py` 未修改。
- 新 launch 能独立启动 RoboSense 方案。
- TF 树只有一个 `map->odom` publisher。
- Nav2 能读到 `map -> odom -> base_footprint`。

### 阶段 E：对比实验

同一条路线分别运行：

```bash
ros2 launch humanoid_navigation2 navigation2.launch.py use_sim_time:=false
ros2 launch humanoid_navigation2 navigation2_robosense_localization.launch.py use_sim_time:=false
```

保存 rosbag 和监控日志，生成对比曲线：

- Open3D `map->odom` vs RoboSense `map->odom`
- Open3D pose vs RoboSense pose
- `/robot_realpose` 与目标点误差
- 导航状态变化和异常跳变时刻

## 8. 风险点与处理

### 坐标轴风险

当前 Open3D 方案已经使用 `fastlio_open3d_axis_adapter` 做 raw Fast-LIO 轴到 ROS 标准轴转换。RoboSense 外部仓库配置中也包含：

```yaml
convert_fastlio_odom_to_base: true
convert_registered_cloud_to_body: true
fastlio_odom_camera_quat_xyzw: [-0.5, -0.5, 0.5, 0.5]
fastlio_body_base_quat_xyzw: [0.5, 0.5, -0.5, 0.5]
```

这说明它已经针对当前 Fast-LIO 坐标做了适配。移植后要先确认输出 pose 是 `map->base_footprint` 还是 `map->body`，否则 bridge 的 `localized_frame` 会错。

### 双 TF publisher 风险

必须禁止 RoboSense 节点直接发布 `map->odom`，否则会和 bridge 冲突。检查方式：

```bash
ros2 topic info /tf -v
```

### 地图坐标风险

RoboSense 的 `lidar_vehicle_xyz/rpy` 和 Open3D grounded 地图必须使用同一套地面原点和轴定义。如果直接复用 `hall_open3d_grounded.pcd` 后定位偏移固定，优先检查：

- `lidar_vehicle_xyz`
- `lidar_vehicle_rpy`
- `init_position`
- `init_euler`
- 地图是否已经 grounded 到 `base_footprint`

### 计算负载风险

RoboSense matcher 参数里 `leaf_size`、`max_pair_size`、`num_threads`、`max_num_iterations` 会直接影响 CPU。第一版建议保守：

```yaml
leaf_size: 0.1
num_threads: 2
max_num_iterations: 2
max_pair_size: 800
is_pub_cloud: false
is_pub_map: false
```

### 初始化风险

RoboSense README 说明初始位姿默认 `[0,0,0]`，如果实际起点不在地图原点附近，可能进入错误局部最优。第一版实验应显式设置：

```yaml
init_position: [x, y, z]
init_euler: [roll, pitch, yaw]
```

也可以后续增加 `/initialpose` 订阅，把 RViz 给的初值写入定位器。

## 9. 推荐落地顺序

最稳妥的落地顺序是：

1. 只移植 `lidar_localization`，命名为 `robosense_lidar_localization`。
2. 先让它输出 `/prior_localization/robosense_odom`，关闭自身 TF。
3. 用 `prior_map_odom_bridge` 接管 `map->odom`。
4. 新增 `navigation2_robosense_localization.launch.py`，不要动现有 `navigation2.launch.py`。
5. 先 rosbag/offline smoke test，再实机短距离测试。
6. 最后再考虑是否引入 `msf_localization` 或 `/initialpose` 动态重初始化。
