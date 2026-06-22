# 人形导览机器人量产 Docker 化部署方案

版本日期：2026-06-16

适用前提：

- 后续量产机器与当前小主机保持同构。
- Ubuntu 版本、ROS 版本、网卡方案、雷达型号、目录结构、驱动能力尽量一致。
- 当前工作区以本仓库源码和现场操作文档为准。

参考依据：

- [人形机器人导览系统使用说明书](/home/ubuntu/software/Todesk/Files/humanoid_ws/workspace_archive/design_docs/system_operations/人形机器人导览使用说明书.md:1)
- [人形机器人建图导航系统功能使用说明书](/home/ubuntu/software/Todesk/Files/humanoid_ws/workspace_archive/design_docs/system_operations/系统功能使用说明书.md:1)
- [一键启动脚本使用说明](/home/ubuntu/software/Todesk/Files/humanoid_ws/workspace_archive/design_docs/system_operations/一键启动脚本使用说明.md:1)
- [当前工作区节点功能清单](/home/ubuntu/software/Todesk/Files/humanoid_ws/workspace_archive/design_docs/system_operations/当前工作区节点功能清单.md:1)

## 1. 目标与结论

目标不是单纯把源码塞进一个镜像，而是把当前机器人交付所需的运行环境、导航、建图、地图保存和 APP 交互整理成一套可重复部署的量产方案。

结合当前源码和现场使用方式，推荐结论如下：

1. 可以 Docker 化。
2. 导航和建图都可以在容器内运行。
3. 第一阶段不建议把所有内容硬塞进一个长期运行的单容器。
4. 最稳妥方案是“同一个基础镜像 + 两种运行角色”：
   - `navigation-runtime`：日常导航
   - `mapping-runtime`：建图和保存地图
5. 语音、桌面显示、人工调试工具建议先保留宿主机能力，不作为量产主链路强依赖。

## 2. 当前系统真实运行链路

依据源码，当前正式导航入口是：

```bash
./start_navigation.sh
```

它会分两层启动：

1. 控制层：`start_ros_control_plane.sh`
2. 地图绑定导航层：`start_navigation_stack.sh`

对应的 ROS launch 入口见：

- [start_navigation.sh](/home/ubuntu/software/Todesk/Files/humanoid_ws/start_navigation.sh:1)
- [start_ros_control_plane.sh](/home/ubuntu/software/Todesk/Files/humanoid_ws/start_ros_control_plane.sh:1)
- [start_navigation_stack.sh](/home/ubuntu/software/Todesk/Files/humanoid_ws/start_navigation_stack.sh:1)
- [robot_real.launch.py](/home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_bringup/launch/robot_real.launch.py:1)
- [robot_control_plane.launch.py](/home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_bringup/launch/robot_control_plane.launch.py:1)
- [robot_navigation_stack.launch.py](/home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_bringup/launch/robot_navigation_stack.launch.py:1)

建图入口是：

```bash
./start_mapping.sh <map_id>
```

建图结束后会自动完成以下动作：

1. 停止 rosbag 和 PCD 保存器。
2. 调用 `/map_save` 和 `slam_toolbox` 保存地图。
3. 生成 2D map、posegraph、原始 PCD、标准 PCD、Open3D/RoboSense prior-map PCD。
4. 生成 Scan Context 数据库。
5. 自动更新 `data/maps/map_registry.json`。
6. 自动初始化 `data/waypoints/<map_id>.json`。

对应脚本见：

- [start_mapping.sh](/home/ubuntu/software/Todesk/Files/humanoid_ws/start_mapping.sh:1)
- [robot_mapping.launch.py](/home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_bringup/launch/robot_mapping.launch.py:1)
- [mapping_only.launch.py](/home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_navigation2/launch/mapping_only.launch.py:1)

这说明容器方案必须覆盖：

- 导航运行
- 建图运行
- 地图产物保存
- 地图注册表回写
- 点位文件持久化

## 3. 同构量产假设下的总体策略

既然后续量产都使用和当前一样的小主机，那么不必一开始追求“跨机型通用镜像”，可以先做“同构主机专用镜像”。

这意味着：

1. 可以接受当前部分绝对路径短期不改。
2. 可以假设宿主机仍是 Ubuntu 24.04 + ROS 2 Jazzy 生态。
3. 可以假设雷达网卡命名、Intel oneAPI/OpenCL、FastDDS 运行方式与当前机一致。
4. 可以把重点从“高度抽象”转成“稳定交付”。

但仍需保留三条底线：

1. 运行时不在容器内重新编译工作区。
2. 地图、点位、日志必须持久化到宿主机。
3. 雷达/DDS/建图链路优先使用宿主网络。

## 4. 推荐部署架构

推荐采用一个基础镜像、两个运行模式：

### 4.1 基础镜像 `humanoid-nav-base`

镜像内包含：

- Ubuntu 24.04
- ROS 2 Jazzy
- `tools/apt_dependencies_ubuntu24_jazzy.txt` 中的运行依赖
- 工作区源码
- `colcon build` 后的 `install/`
- FastDDS 配置模板
- Open3D 运行依赖
- 如现场确认需要，则包含 Intel oneAPI / OpenCL 运行时

镜像构建时完成：

1. 安装 apt 依赖。
2. 安装 ROS 2 Jazzy。
3. 复制工作区源码。
4. 执行 `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`。
5. 设置容器启动入口，默认只加载环境，不在启动时再编译。

### 4.2 导航容器 `navigation-runtime`

职责：

- 启动控制层
- 启动当前地图的导航层
- 响应 APP WebSocket
- 支持运行期切图

建议启动入口：

```bash
./start_navigation.sh
```

但量产版应去掉“启动时编译”逻辑，改成纯运行入口。

### 4.3 建图容器 `mapping-runtime`

职责：

- 启动雷达、Fast-LIO、slam_toolbox 建图链路
- 录 rosbag
- 保存 PCD / 2D map / posegraph / SC 数据库
- 更新 `map_registry.json`
- 初始化该地图的点位文件

建议启动入口：

```bash
./start_mapping.sh <map_id>
```

建图容器不要求常驻，按需启动，用完退出。

## 5. 宿主机必须保留的环境

即使主要链路进容器，宿主机仍需具备以下能力。

### 5.1 基础系统

- Ubuntu 24.04 LTS
- Docker Engine
- 建议安装 Docker Compose plugin
- systemd

### 5.2 硬件与驱动

- 雷达网卡和 IP 配置
- Intel OpenCL/oneAPI 对应宿主机驱动能力
- 如要使用图形界面，还需 X11/桌面会话
- 如要使用语音，还需 PulseAudio 或兼容音频环境

### 5.3 宿主机固定目录

当前源码和文档存在路径差异：

- 文档多处使用 `/home/ubuntu/humanoid_ws`
- 当前工作区实际路径是 `/home/ubuntu/software/Todesk/Files/humanoid_ws`

这件事在量产前必须统一。推荐二选一：

1. 量产机统一使用 `/home/ubuntu/humanoid_ws`
2. 保持当前路径，并在镜像与部署脚本中全部使用同一路径

如果路径不统一，当前大量写死路径会导致：

- 地图文件找不到
- 启动 launch 默认参数错误
- 点位文件读写失败
- 建图保存位置和导航读取位置不一致

## 6. 容器运行所需的必须环境

### 6.1 必需环境变量

当前源码和脚本真实依赖的关键环境包括：

- `ROS_DISTRO=jazzy`
- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
- `FASTRTPS_DEFAULT_PROFILES_FILE`
- `RMW_FASTRTPS_USE_QOS_FROM_XML=1`
- `WORKSPACE`
- 需要 OpenCL 时的 `ONEAPI` 相关环境

见：

- [start_navigation.sh](/home/ubuntu/software/Todesk/Files/humanoid_ws/start_navigation.sh:22)
- [start_navigation_stack.sh](/home/ubuntu/software/Todesk/Files/humanoid_ws/start_navigation_stack.sh:19)
- [start_ros_control_plane.sh](/home/ubuntu/software/Todesk/Files/humanoid_ws/start_ros_control_plane.sh:17)
- [tools/bootstrap_ubuntu24_jazzy.sh](/home/ubuntu/software/Todesk/Files/humanoid_ws/tools/bootstrap_ubuntu24_jazzy.sh:1)

### 6.2 推荐容器运行参数

导航与建图容器建议默认使用：

- `--network host`
- 挂载 FastDDS XML 配置
- 挂载数据目录和日志目录
- 如使用 OpenCL，则挂载相关设备和运行时库

不建议默认使用 bridge 网络，因为当前链路依赖：

- 雷达 UDP 数据接收
- ROS 2 DDS 发现
- FastDDS 配置

### 6.3 必须挂载的目录

以下目录建议强制挂载到宿主机：

1. `data/`
2. `debug_logs/`
3. `src/humanoid_navigation2/maps/`
4. `src/humanoid_navigation2/pcd/`
5. `~/.config/fastdds_shm.xml` 对应路径
6. 如要保留建图录包，还需挂载 `BAG_ROOT`

原因如下：

- `data/maps/map_registry.json` 会被建图和切图逻辑修改
- `data/runtime_maps/` 会在切图时动态生成
- `data/waypoints/` 和 `data/dynamic_waypoints.json` 会被 APP/ROS 持续改写
- `maps/`、`pcd/` 会保存建图产物
- `debug_logs/` 是现场排障核心依据

## 7. 必须覆盖的业务数据

### 7.1 导航运行时数据

当前导航依赖以下关键数据：

- `data/maps/map_registry.json`
- `data/runtime_maps/<map_id>/robosense_lidar_localization.yaml`
- `data/waypoints/<map_id>.json`
- `data/dynamic_waypoints.json`
- 当前地图的 `yaml/pgm`
- 当前地图的 `*_open3d_grounded.pcd`

### 7.2 建图输出数据

`start_mapping.sh` 实际会产出并使用：

- `src/humanoid_navigation2/maps/<map_id>.yaml`
- `src/humanoid_navigation2/maps/<map_id>.pgm`
- `src/humanoid_navigation2/maps/<map_id>.posegraph`
- `src/humanoid_navigation2/maps/<map_id>.data`
- `src/humanoid_navigation2/maps/<map_id>_sc.bin`
- `src/humanoid_navigation2/pcd/<map_id>.pcd`
- `src/humanoid_navigation2/pcd/<map_id>_standard.pcd`
- `src/humanoid_navigation2/pcd/<map_id>_open3d_grounded.pcd`
- `data/maps/map_registry.json`
- `data/waypoints/<map_id>.json`
- rosbag 目录

这些结果如果不落在持久化卷上，建图成功也无法量产复用。

## 8. 代码层面当前必须注意的限制

### 8.1 大量绝对路径写死

当前源码中大量使用类似：

```text
/home/ubuntu/software/Todesk/Files/humanoid_ws/...
```

典型位置包括：

- [robot_real.launch.py](/home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_bringup/launch/robot_real.launch.py:20)
- [robot_navigation_stack.launch.py](/home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_bringup/launch/robot_navigation_stack.launch.py:16)
- [navigation_control_plane.launch.py](/home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_navigation/launch/navigation_control_plane.launch.py:25)
- [dynamic_waypoints_manager.py](/home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_navigation/humanoid_navigation/dynamic_waypoints_manager.py:65)
- [robosense_lidar_localization.yaml](/home/ubuntu/software/Todesk/Files/humanoid_ws/src/robosense_lidar_localization/config/robosense_lidar_localization.yaml:85)

在“同构主机 + 固定路径”前提下，第一阶段可以接受，但必须明确它是量产约束，不是通用设计。

### 8.2 启动脚本默认会重新编译

[start_navigation.sh](/home/ubuntu/software/Todesk/Files/humanoid_ws/start_navigation.sh:49) 当前会在启动时执行 `colcon build`。

这在量产阶段不建议保留，原因：

- 启动时间不可控
- 容器运行期出现编译波动
- 现场故障定位复杂
- 不利于版本封版

量产版应改为：

1. 镜像构建阶段编译
2. 运行阶段只 source 环境并启动

### 8.3 控制层和导航层已经天然适合拆分

当前架构已经把控制层和地图绑定导航层拆开：

- 控制层常驻
- 切图只重启导航层

这与容器化非常契合。即使先只用一个导航容器，也应保留这一层次设计，不建议回退成所有东西揉成单进程脚本。

### 8.4 语音链路仍强依赖宿主机会话

[start_all_services.sh](/home/ubuntu/software/Todesk/Files/humanoid_ws/start_all_services.sh:1) 显示语音链路依赖：

- `DISPLAY`
- `XDG_RUNTIME_DIR`
- `DBUS_SESSION_BUS_ADDRESS`
- `PULSE_SERVER`
- `/home/ubuntu/asr-client`

因此量产第一阶段建议：

1. 导航主链路先独立容器化
2. 语音保留宿主机
3. 后续再单独评估是否容器化语音

## 9. 推荐的量产部署分层

### 9.1 第一阶段：可交付版本

宿主机负责：

- Ubuntu
- Docker
- 雷达网卡/IP
- oneAPI/OpenCL 驱动基础
- 音频和桌面会话

容器负责：

- ROS 2 Jazzy
- 工作区 install
- 导航主链路
- 建图主链路
- 地图保存和注册
- WebSocket 与 APP 交互

### 9.2 第二阶段：进一步收敛

后续可继续评估是否把以下模块进一步容器化：

- 语音服务
- RViz 调试容器
- 监控和日志上传

## 10. 量产发布建议

### 10.1 镜像版本管理

每次交付使用固定版本标签，例如：

- `humanoid-nav:2026.06.16-nav`
- `humanoid-nav:2026.06.16-map`

避免现场使用 `latest`。

### 10.2 地图和点位作为设备数据管理

建议将“程序版本”和“现场地图数据”分开管理：

- 镜像：程序版本
- 宿主机卷：设备地图、点位、日志

这样升级程序不会覆盖客户现场地图。

### 10.3 上线前现场验收清单

量产机每台上线前至少确认：

1. 雷达数据正常
2. `/odom` 正常
3. 建图容器能保存完整地图
4. 新地图能自动写入 `map_registry.json`
5. 导航容器能读取新地图并成功切图
6. APP 能读写点位
7. 日志目录可落盘

## 11. 最终建议

基于当前源码和使用说明书，量产阶段最现实、风险最低的方案是：

1. 采用“同构小主机专用 Docker 方案”。
2. 使用单一基础镜像。
3. 拆成导航运行和建图运行两个角色。
4. 继续使用宿主网络。
5. 把地图、点位、日志全部持久化挂载。
6. 第一阶段不强推语音完全容器化。
7. 量产前统一工作区实际路径。
8. 量产镜像中移除启动时编译逻辑。

如果按这个方案推进，你们不需要先做大规模重构，就可以先获得：

- 可复制部署
- 可封版交付
- 建图和导航都能在容器内运行
- 地图和客户数据不会因重建容器丢失

## 12. 后续落地顺序

建议按以下顺序施工：

1. 统一量产机工作区路径。
2. 确定宿主机必须保留的驱动和服务。
3. 补齐 Dockerfile。
4. 将 `start_navigation.sh` 改成量产版纯运行入口。
5. 整理 `docker run` 或 `docker compose` 启动模板。
6. 选择一台同构小主机做完整验证：
   - 建图
   - 保存地图
   - 切图
   - 导航
   - APP 点位管理

完成后，这套方案就可以作为量产基线。
