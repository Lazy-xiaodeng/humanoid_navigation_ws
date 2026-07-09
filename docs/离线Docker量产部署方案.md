# 人形机器人导航系统离线 Docker 量产部署方案

版本日期：2026-06-29

适用工作区：`/home/ubuntu/humanoid_ws`

适用场景：客户现场只有 Ubuntu 24.04 基础系统，通过 U 盘离线部署人形机器人导览导航系统。导航服务默认安装为 systemd 服务但保持 disabled，由现场人员按需手动启动；建图由现场人员手动启动，并需要 RViz 实时观察效果。

## 1. 方案目标

本方案将 ROS 2 Jazzy、项目源码、依赖和已编译产物预先封装到 Docker 镜像中，现场部署时不再执行 ROS 安装、rosdep 下载、源码拉取和大规模编译。现场部署只负责安装 Docker、导入镜像、准备持久化目录、创建启动脚本和 systemd 服务。

目标效果：

1. 支持 Ubuntu 24.04 裸系统离线部署。
2. 支持 U 盘携带镜像和部署脚本，一台机器一键导入部署。
3. 导航入口保持兼容当前工作区逻辑，核心入口为 `/home/ubuntu/humanoid_ws/start_navigation.sh`。
4. 建图入口保持为 `/home/ubuntu/humanoid_ws/start_mapping.sh`，并支持 RViz 可视化。
5. 保留 APP 与 ROS 的 WebSocket 通信链路。
6. 保留机器人底层 WebSocket 控制链路。
7. 保留表情/动作串口设备访问。
8. 保留 FastDDS 共享内存，保障雷达、Fast-LIO、点云滤波等高带宽话题吞吐。
9. 地图、点位、日志、rosbag 等现场数据在宿主机持久化，升级镜像时不丢失。
10. 导航 systemd 服务创建后默认 disabled，不自动开机启动。

## 2. 当前系统事实

基于当前 Ubuntu 工作区 `/home/ubuntu/humanoid_ws`，已有以下关键入口和约束：

1. 导航入口：

   ```bash
   /home/ubuntu/humanoid_ws/start_navigation.sh
   ```

   当前脚本会加载 ROS 2 Jazzy、设置 FastDDS 环境变量、执行 `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`，然后启动：

   ```bash
   ros2 launch humanoid_bringup robot_real.launch.py use_sim_time:=false
   ```

2. 导航 launch 链路：

   ```text
   start_navigation.sh
     -> humanoid_bringup/robot_real.launch.py
       -> humanoid_description/display.launch.py
       -> humanoid_navigation2/navigation2_robosense_lidar.launch.py
       -> humanoid_navigation/navigation_fusion_sc.launch.py
       -> humanoid_websocket/websocket_server.launch.py
       -> humanoid_locomotion/locomotion.launch.py
   ```

3. 建图入口：

   ```bash
   /home/ubuntu/humanoid_ws/start_mapping.sh [map_name]
   ```

   默认地图名为 `hall`，启动：

   ```bash
   ros2 launch humanoid_bringup robot_mapping.launch.py \
     pcd_map_file:=... \
     rviz:=true \
     app_layer:=false
   ```

4. 建图输出目录：

   ```text
   /home/ubuntu/humanoid_ws/src/humanoid_navigation2/maps
   /home/ubuntu/humanoid_ws/src/humanoid_navigation2/pcd
   /home/ubuntu/humanoid_ws/data/maps/map_registry.json
   /home/ubuntu/humanoid_ws/data/waypoints
   /home/ubuntu/humanoid_ws/debug_logs
   /home/ubuntu/fast-lio-bags
   ```

5. APP 连接 ROS 端 WebSocket：

   当前配置文件为：

   ```text
   /home/ubuntu/humanoid_ws/src/humanoid_websocket/config/websocket_config.yaml
   ```

   ROS 端服务默认监听：

   ```text
   0.0.0.0:8765
   ```

   所以 Docker 容器必须使用宿主机网络：

   ```bash
   --network host
   ```

6. ROS 端连接机器人底层 WebSocket：

   默认地址：

   ```text
   ws://10.192.1.2:5000
   ```

   容器使用 `--network host` 后，该连接与宿主机网络行为一致。

7. 表情串口：

   `humanoid_locomotion/launch/locomotion.launch.py` 当前配置为：

   ```text
   /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
   ```

   虽然现场常说是 `ttyUSB0`，但实际 launch 使用的是稳定的 by-id 路径。Docker 运行时需要保证容器能访问 `/dev/serial/by-id/...` 以及其指向的底层 `/dev/ttyUSB*`。

8. FastDDS 共享内存：

   当前启动脚本依赖：

   ```bash
   RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.config/fastdds_shm.xml
   RMW_FASTRTPS_USE_QOS_FROM_XML=1
   ```

   Docker 运行时需要共享 IPC 命名空间或正确挂载 `/dev/shm`，推荐使用：

   ```bash
   --ipc=host
   ```

## 3. 总体架构

量产部署分为两个阶段。

第一阶段在开发机或构建机完成：

```text
源码工作区
  -> 构建 Docker 镜像
  -> 镜像内安装 ROS 2 Jazzy、依赖、源码
  -> 镜像内 colcon build 成功
  -> 导出 humanoid_nav_<version>.tar
  -> 生成 U 盘离线部署包
```

第二阶段在客户现场完成：

```text
Ubuntu 24.04 裸系统
  -> 插入 U 盘
  -> 运行 install_offline.sh
  -> 离线安装 Docker
  -> docker load 导入镜像
  -> 初始化 /home/ubuntu/humanoid_data
  -> 安装 run_navigation.sh / run_mapping.sh
  -> 创建 systemd 服务，保持 disabled
  -> 用户手动启动导航或建图
```

## 4. 镜像内容设计

镜像内固定包含：

```text
/opt/ros/jazzy
/home/ubuntu/humanoid_ws/src
/home/ubuntu/humanoid_ws/install
/home/ubuntu/humanoid_ws/start_navigation.sh
/home/ubuntu/humanoid_ws/start_mapping.sh
/home/ubuntu/humanoid_ws/stop_navigation.sh
/home/ubuntu/.config/fastdds_shm.xml
```

镜像内不应包含：

1. 私有 Git 仓库 token、账号密码或 SSH 私钥。
2. 现场机器人独有的大量 rosbag。
3. 客户现场新建地图和点位的唯一副本。
4. 临时调试日志。

镜像构建建议：

1. 使用 Ubuntu 24.04 + ROS 2 Jazzy 作为基础。
2. 构建时执行依赖安装和 `colcon build`。
3. 构建产物和源码一起保留，方便现场排查。
4. 镜像 tag 使用明确版本，例如：

   ```text
   humanoid_nav:2026.06.29-rc1
   humanoid_nav:1.0.0
   ```

5. 每次量产镜像必须记录源码 commit、构建时间、构建机信息和镜像 sha256。

## 5. 启动脚本调整建议

当前 `start_navigation.sh` 每次启动都会执行 `colcon build`。这对开发机合理，但对量产 Docker 不合适。推荐做一个小改造：

```bash
if [ "${SKIP_COLCON_BUILD:-0}" != "1" ]; then
  echo "Building workspace..."
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
else
  echo "SKIP_COLCON_BUILD=1, skip runtime build."
fi
```

这样保持开发机默认行为不变，容器运行时只需要设置：

```bash
SKIP_COLCON_BUILD=1
```

如果暂时不修改原脚本，也可以新增容器专用入口：

```text
/home/ubuntu/humanoid_ws/docker/run_navigation_in_container.sh
```

该入口只负责 source 环境、设置 FastDDS 环境变量、清理旧进程并启动 `robot_real.launch.py`，不在现场运行编译。

推荐优先采用 `SKIP_COLCON_BUILD=1` 的方式，因为它最少改动现有操作习惯，仍然保留 `./start_navigation.sh` 作为正式入口。

## 6. 宿主机持久化目录设计

宿主机建议统一使用：

```text
/home/ubuntu/humanoid_data
```

目录结构：

```text
/home/ubuntu/humanoid_data/
  maps/
  pcd/
  data/
    maps/
    waypoints/
  debug_logs/
  fast-lio-bags/
  config/
    fastdds_shm.xml
    websocket_config.yaml
  backups/
```

容器运行时挂载关系：

```text
/home/ubuntu/humanoid_data/maps
  -> /home/ubuntu/humanoid_ws/src/humanoid_navigation2/maps
  -> /home/ubuntu/humanoid_ws/install/humanoid_navigation2/share/humanoid_navigation2/maps

/home/ubuntu/humanoid_data/pcd
  -> /home/ubuntu/humanoid_ws/src/humanoid_navigation2/pcd
  -> /home/ubuntu/humanoid_ws/install/humanoid_navigation2/share/humanoid_navigation2/pcd

/home/ubuntu/humanoid_data/data
  -> /home/ubuntu/humanoid_ws/data

/home/ubuntu/humanoid_data/debug_logs
  -> /home/ubuntu/humanoid_ws/debug_logs

/home/ubuntu/humanoid_data/fast-lio-bags
  -> /home/ubuntu/fast-lio-bags

/home/ubuntu/humanoid_data/config/fastdds_shm.xml
  -> /home/ubuntu/.config/fastdds_shm.xml
```

注意：不要把宿主机的 `/home/ubuntu/humanoid_ws` 整体挂载到容器内。否则会覆盖镜像里已经编译好的 `install`、源码和依赖状态，失去 Docker 镜像稳定性的意义。

注意：地图和 PCD 需要同时挂载到 `src` 和 `install/share` 两套路径。当前建图脚本写入 `src/humanoid_navigation2/maps` 与 `src/humanoid_navigation2/pcd`，而正式导航中的部分 launch 会通过 `get_package_share_directory('humanoid_navigation2')` 读取 `install/humanoid_navigation2/share/humanoid_navigation2/maps` 和 `pcd`。双路径挂载可以保证建图保存后的文件立即被导航读取，不需要现场重新编译。

首次部署时，如果宿主机持久化目录为空，部署脚本应从镜像或 U 盘默认数据中初始化：

1. 默认地图。
2. 默认 PCD。
3. 默认 `map_registry.json`。
4. 默认点位文件。
5. 默认 FastDDS XML。

## 7. Docker 运行参数

导航容器建议参数：

```bash
docker run --rm \
  --name humanoid-navigation \
  --network host \
  --ipc host \
  --privileged \
  -e WORKSPACE=/home/ubuntu/humanoid_ws \
  -e SKIP_COLCON_BUILD=1 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/home/ubuntu/.config/fastdds_shm.xml \
  -e RMW_FASTRTPS_USE_QOS_FROM_XML=1 \
  -v /home/ubuntu/humanoid_data/maps:/home/ubuntu/humanoid_ws/src/humanoid_navigation2/maps \
  -v /home/ubuntu/humanoid_data/maps:/home/ubuntu/humanoid_ws/install/humanoid_navigation2/share/humanoid_navigation2/maps \
  -v /home/ubuntu/humanoid_data/pcd:/home/ubuntu/humanoid_ws/src/humanoid_navigation2/pcd \
  -v /home/ubuntu/humanoid_data/pcd:/home/ubuntu/humanoid_ws/install/humanoid_navigation2/share/humanoid_navigation2/pcd \
  -v /home/ubuntu/humanoid_data/data:/home/ubuntu/humanoid_ws/data \
  -v /home/ubuntu/humanoid_data/debug_logs:/home/ubuntu/humanoid_ws/debug_logs \
  -v /home/ubuntu/humanoid_data/fast-lio-bags:/home/ubuntu/fast-lio-bags \
  -v /home/ubuntu/humanoid_data/config/fastdds_shm.xml:/home/ubuntu/.config/fastdds_shm.xml:ro \
  humanoid_nav:<version> \
  bash -lc "cd /home/ubuntu/humanoid_ws && ./start_navigation.sh"
```

说明：

1. `--network host` 用于 ROS 2 DDS、雷达网络、APP WebSocket 和机器人底层 WebSocket。
2. `--ipc host` 用于 FastDDS 共享内存。
3. `--privileged` 是量产首版推荐方案，能最大程度减少串口、雷达、USB、CAN、共享内存权限问题。
4. 后续如果要收紧权限，可以逐步替换为 `--device`、`--group-add` 和精确能力授权。

如果现场安全要求不允许 `--privileged`，需要至少确认以下设备和权限：

```bash
--device /dev/ttyUSB0
--device /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
--group-add dialout
```

但 by-id 路径是 symlink，只挂 symlink 可能不够，实际仍要保证底层 `/dev/ttyUSB*` 在容器内可访问。

## 8. systemd 服务设计

导航服务文件：

```text
/etc/systemd/system/humanoid-navigation.service
```

建议内容：

```ini
[Unit]
Description=Humanoid Navigation Docker Service
After=docker.service network-online.target
Wants=network-online.target
Requires=docker.service

[Service]
Type=simple
User=ubuntu
WorkingDirectory=/home/ubuntu/humanoid_deploy
ExecStart=/home/ubuntu/humanoid_deploy/run_navigation.sh
ExecStop=/home/ubuntu/humanoid_deploy/stop_navigation.sh
Restart=on-failure
RestartSec=5
TimeoutStopSec=30

[Install]
WantedBy=multi-user.target
```

安装后执行：

```bash
sudo systemctl daemon-reload
sudo systemctl disable humanoid-navigation.service
```

默认不执行：

```bash
sudo systemctl enable humanoid-navigation.service
```

现场人员手动启动：

```bash
sudo systemctl start humanoid-navigation.service
```

手动停止：

```bash
sudo systemctl stop humanoid-navigation.service
```

查看状态：

```bash
systemctl status humanoid-navigation.service
docker logs -f humanoid-navigation
```

## 9. 建图运行设计

建图不建议作为默认开机服务。建图需要人工推行机器人、观察 RViz、手动 finish 或 abort，所以应提供单独脚本：

```text
/home/ubuntu/humanoid_deploy/run_mapping.sh
/home/ubuntu/humanoid_deploy/finish_mapping.sh
/home/ubuntu/humanoid_deploy/abort_mapping.sh
```

建图容器使用交互模式，并额外映射 X11：

```bash
xhost +local:docker

docker run --rm -it \
  --name humanoid-mapping \
  --network host \
  --ipc host \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e WORKSPACE=/home/ubuntu/humanoid_ws \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/home/ubuntu/.config/fastdds_shm.xml \
  -e RMW_FASTRTPS_USE_QOS_FROM_XML=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/ubuntu/.Xauthority:/home/ubuntu/.Xauthority:ro \
  -v /home/ubuntu/humanoid_data/maps:/home/ubuntu/humanoid_ws/src/humanoid_navigation2/maps \
  -v /home/ubuntu/humanoid_data/maps:/home/ubuntu/humanoid_ws/install/humanoid_navigation2/share/humanoid_navigation2/maps \
  -v /home/ubuntu/humanoid_data/pcd:/home/ubuntu/humanoid_ws/src/humanoid_navigation2/pcd \
  -v /home/ubuntu/humanoid_data/pcd:/home/ubuntu/humanoid_ws/install/humanoid_navigation2/share/humanoid_navigation2/pcd \
  -v /home/ubuntu/humanoid_data/data:/home/ubuntu/humanoid_ws/data \
  -v /home/ubuntu/humanoid_data/debug_logs:/home/ubuntu/humanoid_ws/debug_logs \
  -v /home/ubuntu/humanoid_data/fast-lio-bags:/home/ubuntu/fast-lio-bags \
  -v /home/ubuntu/humanoid_data/config/fastdds_shm.xml:/home/ubuntu/.config/fastdds_shm.xml:ro \
  humanoid_nav:<version> \
  bash -lc "cd /home/ubuntu/humanoid_ws && ./start_mapping.sh ${MAP_NAME:-hall}"
```

建图完成时，用户在建图终端输入：

```text
finish
```

或另开终端执行：

```bash
docker exec humanoid-mapping bash -lc "echo finish > /home/ubuntu/humanoid_ws/.start_mapping.command"
```

作废本次建图：

```bash
docker exec humanoid-mapping bash -lc "echo abort > /home/ubuntu/humanoid_ws/.start_mapping.command"
```

建图成功后，以下文件会保存到宿主机持久化目录：

```text
/home/ubuntu/humanoid_data/maps/<map_name>.yaml
/home/ubuntu/humanoid_data/maps/<map_name>.pgm
/home/ubuntu/humanoid_data/maps/<map_name>.posegraph
/home/ubuntu/humanoid_data/maps/<map_name>.data
/home/ubuntu/humanoid_data/maps/<map_name>_sc.bin
/home/ubuntu/humanoid_data/pcd/<map_name>.pcd
/home/ubuntu/humanoid_data/pcd/<map_name>_standard.pcd
/home/ubuntu/humanoid_data/pcd/<map_name>_open3d_grounded.pcd
/home/ubuntu/humanoid_data/data/maps/map_registry.json
/home/ubuntu/humanoid_data/data/waypoints/<map_name>.json
/home/ubuntu/humanoid_data/fast-lio-bags/<map_name>_mapping_<timestamp>
```

## 10. APP 和 WebSocket 配置

当前 APP 连接 ROS 端：

```text
ws://<机器人主机IP>:8765
```

容器使用 `--network host` 后，APP 看到的是宿主机 IP，不需要暴露 Docker 端口。

当前 ROS 端连接机器人底层：

```text
ws://10.192.1.2:5000
```

量产时需要确认每台机器人底层 IP 是否固定为 `10.192.1.2`。如果所有机器人一致，可以保留镜像默认配置。如果不同，应把 WebSocket 配置改成可现场覆盖。

推荐后续改造方向：

1. `websocket_config.yaml` 支持从环境变量读取机器人底层地址。
2. 或部署脚本把 `/home/ubuntu/humanoid_data/config/websocket_config.yaml` 挂载覆盖到容器内对应配置路径。
3. 每台机器人在 U 盘部署时只需要修改 `humanoid.env`。

首版量产如果机器人底层地址固定，可以先不做动态配置，降低改动风险。

## 11. FastDDS 共享内存配置

容器内必须满足：

```bash
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
FASTRTPS_DEFAULT_PROFILES_FILE=/home/ubuntu/.config/fastdds_shm.xml
RMW_FASTRTPS_USE_QOS_FROM_XML=1
```

Docker 运行必须带：

```bash
--ipc host
```

建议 FastDDS XML 由 U 盘部署包提供，并放到：

```text
/home/ubuntu/humanoid_data/config/fastdds_shm.xml
```

部署完成后可用以下方式验证：

```bash
docker exec humanoid-navigation bash -lc 'echo $RMW_IMPLEMENTATION; echo $FASTRTPS_DEFAULT_PROFILES_FILE'
ls -lh /dev/shm | grep -i fast
```

如果雷达频率不满，优先检查：

1. 容器是否带 `--ipc host`。
2. FastDDS XML 是否存在且路径正确。
3. `RMW_FASTRTPS_USE_QOS_FROM_XML=1` 是否生效。
4. 雷达点云话题 `/airy_points`、`/fast_lio/cloud_registered`、`/airy_points_filtered` 是否在 XML 中配置。

## 12. U 盘离线部署包结构

建议 U 盘目录：

```text
humanoid_offline_deploy/
  README.md
  VERSION
  SHA256SUMS
  images/
    humanoid_nav_1.0.0.tar
  docker_debs/
    containerd.io_*.deb
    docker-ce_*.deb
    docker-ce-cli_*.deb
    docker-buildx-plugin_*.deb
    docker-compose-plugin_*.deb
  config/
    fastdds_shm.xml
    humanoid.env
  scripts/
    install_offline.sh
    run_navigation.sh
    stop_navigation.sh
    run_mapping.sh
    finish_mapping.sh
    abort_mapping.sh
    check_deploy.sh
  systemd/
    humanoid-navigation.service
  defaults/
    maps/
    pcd/
    data/
```

`VERSION` 示例：

```text
image=humanoid_nav:1.0.0
source_commit=<git commit>
build_time=2026-06-29T20:00:00+08:00
ros_distro=jazzy
ubuntu=24.04
```

`humanoid.env` 示例：

```bash
IMAGE_NAME=humanoid_nav:1.0.0
CONTAINER_NAME=humanoid-navigation
WORKSPACE=/home/ubuntu/humanoid_ws
HUMANOID_DATA=/home/ubuntu/humanoid_data
MAP_NAME=hall
ROBOT_WS_SERVER=ws://10.192.1.2:5000
APP_WS_PORT=8765
SKIP_COLCON_BUILD=1
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
FASTRTPS_DEFAULT_PROFILES_FILE=/home/ubuntu/.config/fastdds_shm.xml
RMW_FASTRTPS_USE_QOS_FROM_XML=1
```

## 13. 现场部署流程

现场操作顺序：

1. 安装 Ubuntu 24.04，创建用户 `ubuntu`。
2. 插入 U 盘。
3. 进入 U 盘部署目录：

   ```bash
   cd /media/ubuntu/<U盘名>/humanoid_offline_deploy
   ```

4. 执行：

   ```bash
   sudo ./scripts/install_offline.sh
   ```

5. 脚本执行以下动作：

   ```text
   检查 Ubuntu 版本
   检查当前用户
   离线安装 Docker
   启动 docker 服务
   docker load 导入镜像
   创建 /home/ubuntu/humanoid_data
   初始化默认地图和配置
   安装 /home/ubuntu/humanoid_deploy 脚本
   安装 humanoid-navigation.service
   systemctl daemon-reload
   systemctl disable humanoid-navigation.service
   执行部署自检
   ```

6. 部署完成后检查：

   ```bash
   docker images | grep humanoid_nav
   systemctl is-enabled humanoid-navigation.service
   ```

   预期：

   ```text
   disabled
   ```

## 14. 现场使用流程

启动导航：

```bash
sudo systemctl start humanoid-navigation.service
```

查看导航：

```bash
systemctl status humanoid-navigation.service
docker logs -f humanoid-navigation
```

停止导航：

```bash
sudo systemctl stop humanoid-navigation.service
```

启动建图：

```bash
cd /home/ubuntu/humanoid_deploy
./run_mapping.sh hall
```

完成建图保存：

```bash
./finish_mapping.sh
```

作废建图：

```bash
./abort_mapping.sh
```

建图完成后重启导航，让导航加载新地图和点位：

```bash
sudo systemctl restart humanoid-navigation.service
```

## 15. 升级与回滚

升级流程：

1. 停止导航服务：

   ```bash
   sudo systemctl stop humanoid-navigation.service
   ```

2. 导入新镜像：

   ```bash
   docker load -i images/humanoid_nav_1.0.1.tar
   ```

3. 更新 `/home/ubuntu/humanoid_deploy/humanoid.env` 中的 `IMAGE_NAME`。
4. 启动服务：

   ```bash
   sudo systemctl start humanoid-navigation.service
   ```

回滚流程：

1. 停止服务。
2. 将 `IMAGE_NAME` 改回上一个版本。
3. 启动服务。

因为地图、点位、日志、bag 都在 `/home/ubuntu/humanoid_data`，镜像升级和回滚不会覆盖现场数据。

## 16. 部署自检清单

部署脚本应提供：

```bash
/home/ubuntu/humanoid_deploy/check_deploy.sh
```

检查项：

1. Docker 服务是否正常。
2. 镜像是否已导入。
3. `humanoid-navigation.service` 是否存在。
4. `humanoid-navigation.service` 是否 disabled。
5. `/home/ubuntu/humanoid_data` 目录是否存在。
6. FastDDS XML 是否存在。
7. 表情串口 by-id 路径是否存在。
8. APP WebSocket 端口 `8765` 是否未被其他进程占用。
9. 是否能启动一个短生命周期容器并 source ROS 环境。
10. 是否能在容器内找到关键包：

    ```bash
    ros2 pkg prefix humanoid_bringup
    ros2 pkg prefix humanoid_navigation2
    ros2 pkg prefix humanoid_websocket
    ros2 pkg prefix humanoid_locomotion
    ```

11. 是否能读取地图注册文件。
12. 是否能写入 debug log 目录。

## 17. 风险与处理

1. 串口设备名变化

   当前 launch 使用 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`，比 `/dev/ttyUSB0` 稳。现场如果 by-id 不存在，应检查 USB 串口芯片、线缆和 udev，而不是直接改成随机的 `ttyUSB0`。

2. FastDDS 共享内存未生效

   最常见原因是没有 `--ipc host` 或 XML 没挂载。症状是 `/airy_points` 频率不足、Fast-LIO 跑不满或点云链路延迟变大。

3. 建图 RViz 无法显示

   常见原因是 X11 权限或 DISPLAY 未传入。先确认宿主机本身已进入 Ubuntu 图形桌面，再执行 `xhost +local:docker`。

4. 现场修改被镜像覆盖

   量产原则是镜像不可变，现场可变数据只放 `/home/ubuntu/humanoid_data`。不要在容器里直接改源码作为长期方案；需要固化的修改应回到开发机重新构建镜像。

5. `start_navigation.sh` 运行时编译导致启动慢

   应通过 `SKIP_COLCON_BUILD=1` 跳过运行时编译，或使用容器专用导航入口。量产现场不应依赖临场编译。

6. 机器人底层 WebSocket IP 不一致

   如果不同批次机器人不是 `10.192.1.2`，需要把该配置做成部署参数。首版部署前必须确认。

## 18. 后续落地任务

建议按以下顺序落地：

1. 修改 `start_navigation.sh`，支持 `SKIP_COLCON_BUILD=1`。
2. 编写 Dockerfile，构建并验证镜像。
3. 编写 `run_navigation.sh`、`stop_navigation.sh`。
4. 编写 `run_mapping.sh`、`finish_mapping.sh`、`abort_mapping.sh`。
5. 编写 `install_offline.sh`，实现离线安装 Docker、导入镜像、初始化数据、安装 systemd。
6. 编写 `check_deploy.sh`。
7. 在一台干净 Ubuntu 24.04 机器上做完整离线部署演练。
8. 做导航验证：APP 连接、底层 WebSocket、串口表情、雷达频率、Fast-LIO、Nav2。
9. 做建图验证：RViz 显示、finish 保存、地图注册、导航加载新地图。
10. 冻结镜像版本，生成 U 盘部署包和 SHA256 校验文件。

## 19. 推荐结论

对于当前人形机器人导览系统，离线 Docker 部署比裸机一键安装更适合量产。裸机方案的不确定性集中在 apt 源、ROS 安装、rosdep、pip、私有仓库认证和现场编译；Docker 方案将这些不确定性前移到开发机和镜像构建阶段。

本系统的关键要求是 `--network host`、`--ipc host`、串口设备访问、FastDDS XML、地图/点位/日志持久化，以及建图场景的 X11/RViz 支持。只要这些边界处理好，现场部署脚本可以保持很薄，部署稳定性会明显高于裸机安装方案。
