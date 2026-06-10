# 人形机器人导航系统架构说明

## 📋 目录
- [系统概述](#系统概述)
- [两种工作模式](#两种工作模式)
- [TF树结构](#tf树结构)
- [节点职责说明](#节点职责说明)
- [启动流程](#启动流程)
- [常见问题](#常见问题)

---

## 系统概述

本系统基于**ROS2 + Fast-LIO + NDT + Nav2**实现人形机器人的3D定位与2D导航。

### 核心组件
- **雷达**: 速腾Airy系列3D激光雷达
- **定位**: Fast-LIO (3D激光里程计) + NDT (点云配准全局定位)
- **导航**: Nav2导航栈 (路径规划 + 运动控制)
- **地图**: PCD点云地图 (3D) + YAML栅格地图 (2D)

---

## 两种工作模式

### 模式1：建图模式（首次使用或更新地图）

**用途**: 在未知环境中构建地图

**启动命令**:
```bash
ros2 launch humanoid_navigation2 mapping_only.launch.py
```

**工作流程**:
```
雷达数据 → Fast-LIO (定位+保存PCD) → 点云转2D激光 → slam_toolbox (建图) → 保存地图
```

**输出文件**:
- `/home/ubuntu/humanoid_ws/src/humanoid_navigation2/pcd/hall.pcd` (3D点云地图)
- `/home/ubuntu/humanoid_ws/src/humanoid_navigation2/maps/hall.yaml` (2D栅格地图配置)
- `/home/ubuntu/humanoid_ws/src/humanoid_navigation2/maps/hall.pgm` (2D栅格地图图片)

**保存地图**:
```bash
# slam_toolbox会自动保存，也可以手动调用
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{filename: 'hall'}"
```

---

### 模式2：导航模式（日常使用）

**用途**: 在已有地图上进行定位和导航

**启动命令**:
```bash
ros2 launch humanoid_navigation2 navigation_only.launch.py
```

**工作流程**:
```
雷达数据 → Fast-LIO (输出点云) → NDT定位 (与PCD匹配) → 发布map→odom TF
                                                      ↓
                                              map_server (发布2D地图)
                                                      ↓
                                              Nav2导航栈 (路径规划+控制)
```

**关键节点**:
| 节点 | 作用 | 发布内容 |
|------|------|---------|
| **Fast-LIO** | 3D激光里程计 | `/fast_lio/cloud_registered` + TF: `odom→camera_init→body` |
| **NDT定位** | 全局定位 | TF: `map→odom` + `/pcl_pose` + `/path` |
| **map_server** | 加载2D地图 | `/map` (栅格地图话题) |
| **Nav2** | 路径规划和控制 | 导航指令、速度命令 |

---

## TF树结构

### 完整TF链路
```
map ──→ odom ──→ camera_init ──→ body ──→ base_footprint ──→ base_link ──→ 传感器
  ↑        ↑           ↑           ↑
NDT发布  Fast-LIO   Fast-LIO    Fast-LIO
(动态)   (动态)     (动态)     (动态)

静态变换桥接：
odom → camera_init    (static_transform_publisher, 零变换)
body → base_footprint (static_transform_publisher, 实测偏移)
base_footprint → base_link (robot_state_publisher)
base_link → 传感器   (robot_state_publisher)
```

### 各坐标系说明

| 坐标系 | 说明 | 发布者 |
|--------|------|--------|
| **map** | 全局地图坐标系，地图原点 | NDT定位节点 |
| **odom** | 里程计坐标系，无漂移 | NDT定位节点 (从Fast-LIO修正) |
| **camera_init** | Fast-LIO起始坐标系 | Fast-LIO |
| **body** | 雷达IMU坐标系 | Fast-LIO |
| **base_footprint** | 机器人地面投影 | 静态变换 (从body) |
| **base_link** | 机器人基座 | robot_state_publisher |

---

## 节点职责说明

### 1. Fast-LIO (`fast_lio_robosense`)

**输入**:
- `/airy_points` (3D雷达点云)
- `/airy_imu` (IMU数据)

**处理**:
- 点云去畸变
- 3D激光里程计（高频定位）
- ICP配准（局部）

**输出**:
- `/fast_lio/cloud_registered` (配准后的点云)
- TF: `odom → camera_init → body`

**注意**: Fast-LIO发布的是**局部里程计TF**，不包含全局地图对齐。

---

### 2. NDT定位节点 (`lidar_localization_ros2`)

**输入**:
- `/fast_lio/cloud_registered` (来自Fast-LIO的点云)
- PCD地图文件 (启动时加载)

**处理**:
- 点云体素滤波降采样
- NDT点云配准（与PCD地图对齐）
- 计算全局位姿

**输出**:
- **TF: `map → odom`** (核心输出，全局定位)
- `/pcl_pose` (位姿消息，带协方差)
- `/path` (运动轨迹)

**关键参数**:
```yaml
registration_method: "NDT_OMP"     # 多线程NDT
ndt_resolution: 1.0                # 网格分辨率(米)
map_path: ".../hall.pcd"           # PCD地图路径
base_frame_id: "odom"              # 发布map→odom变换
```

---

### 3. map_server (`nav2_map_server`)

**输入**:
- `hall.yaml` (地图配置文件)
- `hall.pgm` (栅格地图图片)

**处理**:
- 加载2D栅格地图
- 转换为ROS消息

**输出**:
- `/map` (地图话题，供Nav2 costmap使用)

**注意**: NDT节点**不发布**2D栅格地图，必须有map_server！

---

### 4. Nav2导航栈

**包含节点**:
- `planner_server`: 全局路径规划 (A*/Dijkstra)
- `controller_server`: 局部轨迹跟踪 (DWB/MPPI)
- `behavior_server`: 行为管理 (等待、旋转等)
- `bt_navigator`: 行为树执行
- `global_costmap`: 全局代价地图 (订阅`/map`)
- `local_costmap`: 局部代价地图 (实时障碍物)

**输入**:
- `/map` (来自map_server)
- `/scan` 或点云 (来自雷达)
- TF树 (定位信息)

**输出**:
- `/cmd_vel` (速度命令，控制机器人)

---

## 启动流程

### 首次建图流程

```bash
# 1. 确保雷达驱动已安装
ros2 pkg list | grep rslidar

# 2. 启动建图模式
ros2 launch humanoid_navigation2 mapping_only.launch.py

# 3. 在RViz中监控建图过程
# - 查看/fast_lio/cloud_registered点云
# - 查看slam_toolbox的地图构建进度

# 4. 手动或自动移动机器人完成地图扫描

# 5. 保存地图
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{filename: 'hall'}"

# 6. 退出后PCD地图自动保存到pcd/hall.pcd
```

### 导航启动流程

```bash
# 1. 确保已有地图文件
ls /home/ubuntu/humanoid_ws/src/humanoid_navigation2/pcd/hall.pcd
ls /home/ubuntu/humanoid_ws/src/humanoid_navigation2/maps/hall.yaml

# 2. 启动导航模式
ros2 launch humanoid_navigation2 navigation_only.launch.py

# 3. 在RViz中验证
# - 查看TF树是否完整 (map→odom→...→base_link)
# - 查看/map是否正确加载
# - 查看/pcl_pose定位是否稳定

# 4. 设置初始位姿 (如果自动定位失败)
# 在RViz中使用"2D Pose Estimate"

# 5. 发送导航目标
# 在RViz中使用"2D Nav Goal"
```

---

## 常见问题

### Q1: NDT定位节点和slam_toolbox有什么区别？

| 对比项 | NDT定位 | slam_toolbox定位模式 |
|--------|---------|---------------------|
| **输入** | 3D点云 + PCD地图 | 2D激光 + posegraph地图 |
| **算法** | NDT点云配准 |  scan-matching |
| **精度** | 高（3D信息） | 中（2D投影） |
| **速度** | 快（NDT_OMP多线程） | 慢（Ceres优化） |
| **适用** | 已有3D地图的场景 | 已有2D地图的场景 |

**建议**: 如果你有完整的PCD地图，优先使用NDT定位。

---

### Q2: 为什么需要map_server？NDT不能直接发布地图吗？

**不能**。原因如下：

1. **NDT是定位节点**，只负责计算机器人在地图中的位姿
2. **NDT处理的是3D点云**，而Nav2需要的是2D栅格地图
3. **Nav2的costmap**必须订阅`/map`话题（nav_msgs/OccupancyGrid类型）
4. **职责分离**：定位（NDT）与地图服务（map_server）应该独立

**正确架构**:
```
NDT定位 → 发布TF (map→odom)
                  ↓
              Nav2使用TF进行定位
                  
map_server → 发布/map话题
                  ↓
              Nav2 costmap使用/map进行规划
```

---

### Q3: 启动后TF树不完整怎么办？

**检查步骤**:

```bash
# 1. 查看当前TF树
ros2 run tf2_tools view_frames

# 2. 检查各节点是否运行
ros2 node list

# 3. 查看NDT节点日志
ros2 node info /lidar_localization

# 4. 手动测试静态变换
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 odom camera_init
```

**常见断点**:
- `map→odom`断开：NDT节点未启动或地图加载失败
- `odom→camera_init`断开：静态变换未发布
- `body→base_footprint`断开：TF桥接未配置

---

### Q4: 定位不稳定或跳变怎么办？

**调整NDT参数**:

```yaml
# 增加配准精度
ndt_resolution: 0.8          # 减小网格（默认1.0）
ndt_max_iterations: 50       # 增加迭代次数（默认35）
transform_epsilon: 0.001     # 减小收敛阈值（默认0.01）

# 降低误匹配率
score_threshold: 1.5         # 降低得分阈值（默认2.0）
voxel_leaf_size: 0.15        # 减小降采样网格（默认0.2）
```

**检查初始位姿**:
```yaml
# 方法1: 禁用自动初始位姿，手动设置
set_initial_pose: false

# 方法2: 设置为正确的初始位置
set_initial_pose: true
initial_pose_x: 5.0    # 实际X坐标
initial_pose_y: 3.0    # 实际Y坐标
```

---

### Q5: 如何验证NDT定位是否正常工作？

**验证步骤**:

```bash
# 1. 查看NDT发布的位姿话题
ros2 topic echo /pcl_pose

# 2. 查看TF变换
ros2 topic echo /tf | grep -A5 "map.*odom"

# 3. 在RViz中可视化
# - 添加PointCloud2: /fast_lio/cloud_registered (设置Fixed Frame为map)
# - 点云应该与地图完美对齐

# 4. 检查配准得分（开启debug）
# 在终端中查看日志，应该看到:
# "has converged: 1"
# "fitness score: 0.5" (小于2.0为正常)
```

---

## 文件结构

```
humanoid_ws/src/
├── lidar_localization_ros2/          # NDT定位节点
│   ├── param/
│   │   └── localization.yaml         # NDT参数配置
│   ├── launch/
│   │   └── lidar_localization.launch.py
│   └── src/
│       ├── lidar_localization_component.cpp
│       └── lidar_localization_node.cpp
│
├── humanoid_navigation2/             # 导航功能包
│   ├── launch/
│   │   ├── mapping_only.launch.py    # 建图模式
│   │   ├── navigation_only.launch.py # 导航模式
│   │   └── localization.launch.py    # 仅定位（不含导航）
│   ├── config/
│   │   ├── nav2_params.yaml          # Nav2参数
│   │   └── slam_toolbox_mapping.yaml # 建图参数
│   ├── pcd/
│   │   └── hall.pcd                  # 3D点云地图
│   └── maps/
│       ├── hall.yaml                  # 2D地图配置
│       └── hall.pgm                   # 2D地图图片
│
└── fast_lio_robosense/               # Fast-LIO节点
    └── config/
        └── robosenseAiry.yaml        # Fast-LIO参数
```

---

## 维护与更新

- **地图更新**: 环境变化后重新建图
- **参数调优**: 根据实际场景调整NDT参数
- **TF校准**: 定期检查传感器外参

**联系方式**: 有问题请查看ROS2日志或使用`ros2 doctor`诊断。
