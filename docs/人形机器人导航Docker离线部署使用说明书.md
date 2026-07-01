# 人形机器人导航 Docker 离线部署使用说明书

本文档面向现场实施人员和首次接触该系统的用户，按步骤说明如何从源码构建 Docker 镜像、制作 U 盘离线部署包、在新机器上安装、建图、启动导航，以及后续源码修复后如何更新镜像。

本文档约定：

- 开发机/构建机源码目录：`/home/ubuntu/humanoid_ws`
- 目标机器用户名：`ubuntu`
- 目标机器持久化数据目录：`/home/ubuntu/humanoid_data`
- 目标机器部署脚本目录：`/home/ubuntu/humanoid_deploy`
- 镜像名：`humanoid_nav:1.0.0`
- 当前只使用一张地图，地图名固定为：`hall`

重要说明：本 Docker 镜像使用的是 Ubuntu 工作区源码：

```bash
/home/ubuntu/humanoid_ws
```

不是 Todesk 工作区：

```bash
/home/ubuntu/software/Todesk/Files/humanoid_ws
```

---

## 1. 整体流程

完整流程分为四段：

1. 在开发机修改或确认源码。
2. 在开发机构建 Docker 镜像。
3. 导出离线镜像包，并准备 Docker 离线安装包。
4. 复制完整离线部署包到 U 盘。
5. 在目标机器离线安装、建图、启动导航。

后续如果修复 bug，也是同样流程：

```text
修改源码 -> 重新构建镜像 -> 重新导出 tar -> 拷到 U 盘 -> 目标机重新安装/更新 -> 重启导航服务
```

地图、PCD、点位、日志等现场数据放在 `/home/ubuntu/humanoid_data`，不会因为更新镜像而丢失。

---

## 2. 目录说明

开发机上的关键目录：

```text
/home/ubuntu/humanoid_ws
├── docker/
│   ├── Dockerfile
│   └── entrypoint.sh
├── deploy/
│   └── docker_offline/
│       ├── humanoid.env
│       ├── images/
│       │   ├── humanoid_nav_1.0.0.tar
│       │   └── humanoid_nav_1.0.0.tar.sha256
│       ├── docker_debs/
│       │   ├── SHA256SUMS
│       │   ├── docker-ce_*.deb
│       │   ├── docker-ce-cli_*.deb
│       │   ├── containerd.io_*.deb
│       │   └── ...
│       ├── scripts/
│       │   ├── build_image.sh
│       │   ├── save_image.sh
│       │   ├── fetch_docker_debs.sh
│       │   ├── install_offline.sh
│       │   ├── run_navigation.sh
│       │   ├── stop_navigation.sh
│       │   ├── run_mapping.sh
│       │   ├── finish_mapping.sh
│       │   ├── abort_mapping.sh
│       │   └── check_deploy.sh
│       ├── systemd/
│       │   └── humanoid-navigation.service
│       └── config/
│           └── fastdds_shm.xml
├── start_navigation.sh
└── start_mapping.sh
```

目标机器安装后的关键目录：

```text
/home/ubuntu/humanoid_deploy       # 部署脚本
/home/ubuntu/humanoid_data         # 现场持久化数据
├── maps                            # 2D 地图、posegraph、ScanContext 数据
├── pcd                             # 3D 点云地图
├── data                            # 点位、地图 registry 等业务数据
├── debug_logs                      # 导航/建图日志
├── fast-lio-bags                   # 建图 rosbag
└── config
    └── fastdds_shm.xml             # DDS 共享内存配置
```

---

## 3. 开发机构建镜像

以下命令都在开发机执行。

进入 Ubuntu 工作区：

```bash
cd /home/ubuntu/humanoid_ws
```

确认 Docker 可用：

```bash
docker --version
docker info
```

构建镜像：

```bash
./deploy/docker_offline/scripts/build_image.sh
```

构建完成后会看到类似输出：

```text
Built image: humanoid_nav:1.0.0
```

检查镜像是否存在：

```bash
docker image ls | grep humanoid_nav
```

可以做一次基础检查：

```bash
docker run --rm humanoid_nav:1.0.0 bash -lc \
'ros2 pkg prefix humanoid_bringup && \
 ros2 pkg prefix humanoid_navigation2 && \
 python3 -c "import open3d; print(open3d.__version__)"'
```

正常情况下应看到：

```text
/home/ubuntu/humanoid_ws/install/humanoid_bringup
/home/ubuntu/humanoid_ws/install/humanoid_navigation2
0.19.0
```

---

## 4. 导出离线镜像包

在开发机执行：

```bash
cd /home/ubuntu/humanoid_ws
./deploy/docker_offline/scripts/save_image.sh
```

生成文件：

```text
/home/ubuntu/humanoid_ws/deploy/docker_offline/images/humanoid_nav_1.0.0.tar
/home/ubuntu/humanoid_ws/deploy/docker_offline/images/humanoid_nav_1.0.0.tar.sha256
```

查看文件大小：

```bash
ls -lh deploy/docker_offline/images/
```

当前镜像包大约 12G。

校验镜像包：

```bash
cd /home/ubuntu/humanoid_ws/deploy/docker_offline
sha256sum -c images/humanoid_nav_1.0.0.tar.sha256
```

正常输出类似：

```text
images/humanoid_nav_1.0.0.tar: OK
```

---

## 5. 准备 Docker 离线安装包

如果目标机器是完全干净的 Ubuntu 24.04，并且现场不能联网，则必须准备 Docker 离线 deb 包。

在开发机执行：

```bash
cd /home/ubuntu/humanoid_ws
./deploy/docker_offline/scripts/fetch_docker_debs.sh
```

生成目录：

```text
/home/ubuntu/humanoid_ws/deploy/docker_offline/docker_debs/
```

这个目录里会包含 Docker Engine、containerd、buildx、compose，以及 Ubuntu 24.04 上 Docker 需要的本地依赖包。

检查文件：

```bash
ls -lh deploy/docker_offline/docker_debs/
cd /home/ubuntu/humanoid_ws/deploy/docker_offline/docker_debs
sha256sum -c SHA256SUMS
```

正常情况下应全部显示 `OK`。

注意：

- 这一步需要开发机能访问 Docker 官方源。
- 目标机器不需要访问 Docker 官方源。
- `docker_debs` 是给目标机器宿主机安装 Docker 用的，不会被打进导航镜像里。

---

## 6. 拷贝到 U 盘

把整个 `docker_offline` 目录复制到 U 盘，不要只复制 tar 文件。

示例：

```bash
cp -r /home/ubuntu/humanoid_ws/deploy/docker_offline /media/ubuntu/<U盘名>/
sync
```

复制完成后，U 盘中应该有：

```text
docker_offline/
├── humanoid.env
├── images/
│   ├── humanoid_nav_1.0.0.tar
│   └── humanoid_nav_1.0.0.tar.sha256
├── docker_debs/
│   ├── SHA256SUMS
│   ├── docker-ce_*.deb
│   ├── docker-ce-cli_*.deb
│   ├── containerd.io_*.deb
│   └── ...
├── scripts/
├── systemd/
└── config/
```

如果目标机器是完全干净的 Ubuntu 24.04，`docker_debs` 必须存在。否则目标机器没有 Docker 且不能联网时，安装会停止。

---

## 7. 目标机器首次离线安装

以下命令在目标机器执行。

插入 U 盘，进入部署目录：

```bash
cd /media/ubuntu/<U盘名>/docker_offline
```

校验镜像包：

```bash
sha256sum -c images/humanoid_nav_1.0.0.tar.sha256
```

执行安装：

```bash
sudo ./scripts/install_offline.sh
```

安装脚本会做这些事情：

- 检查或使用 `docker_debs` 离线安装 Docker。
- 加载 `humanoid_nav:1.0.0` 镜像。
- 创建 `/home/ubuntu/humanoid_data`。
- 初始化默认 `hall` 地图和 PCD。
- 安装部署脚本到 `/home/ubuntu/humanoid_deploy`。
- 安装 systemd 服务 `humanoid-navigation.service`。
- 服务默认保持 disabled，不会开机自启。
- 执行部署自检。

安装完成后检查：

```bash
cd /home/ubuntu/humanoid_deploy
./scripts/check_deploy.sh
```

正常最后应看到：

```text
[check] OK
```

---

## 8. 启动和停止导航

启动导航：

```bash
sudo systemctl start humanoid-navigation.service
```

查看状态：

```bash
systemctl status humanoid-navigation.service
```

查看日志：

```bash
journalctl -u humanoid-navigation.service -f
```

停止导航：

```bash
sudo systemctl stop humanoid-navigation.service
```

确认服务没有设置为开机自启：

```bash
systemctl is-enabled humanoid-navigation.service
```

正常应该显示：

```text
disabled
```

如果现场确认需要开机自启，再手动启用：

```bash
sudo systemctl enable humanoid-navigation.service
```

如果以后要取消开机自启：

```bash
sudo systemctl disable humanoid-navigation.service
```

---

## 9. 建图流程

当前系统只使用一张地图，地图名固定为 `hall`。

建议建图前先停止导航：

```bash
sudo systemctl stop humanoid-navigation.service
```

启动建图：

```bash
cd /home/ubuntu/humanoid_deploy
./scripts/run_mapping.sh hall
```

建图脚本会启动 RViz，用于实时查看建图效果。

建图完成后，不要直接关闭终端。打开另一个终端执行：

```bash
cd /home/ubuntu/humanoid_deploy
./scripts/finish_mapping.sh
```

`finish_mapping.sh` 会通知建图容器保存地图，并生成：

```text
/home/ubuntu/humanoid_data/maps/hall.yaml
/home/ubuntu/humanoid_data/maps/hall.pgm
/home/ubuntu/humanoid_data/maps/hall.posegraph
/home/ubuntu/humanoid_data/maps/hall.data
/home/ubuntu/humanoid_data/maps/hall_sc.bin
/home/ubuntu/humanoid_data/pcd/hall.pcd
/home/ubuntu/humanoid_data/pcd/hall_standard.pcd
/home/ubuntu/humanoid_data/pcd/hall_open3d_grounded.pcd
/home/ubuntu/humanoid_data/data/maps/map_registry.json
```

看到类似下面日志才算保存成功：

```text
Mapping completed successfully.
```

如果建图效果不好，想放弃本次建图：

```bash
cd /home/ubuntu/humanoid_deploy
./scripts/abort_mapping.sh
```

放弃建图不会注册本次地图。

建图成功后重启导航：

```bash
sudo systemctl restart humanoid-navigation.service
```

---

## 10. APP 和硬件连接检查

导航运行时需要注意：

- APP 服务通过 websocket 与 ROS 侧交互。
- 容器使用 `--network host`，所以端口直接使用主机网络。
- websocket 服务默认监听 `0.0.0.0:8765`。
- 底盘 websocket 地址需要和实际底盘 IP 一致。
- 表情/locomotion 串口设备需要接好，常见是 `ttyUSB0` 或 `/dev/serial/by-id/...`。
- 雷达/FastLIO 大点云通信依赖 DDS 共享内存，运行脚本已经使用 `--ipc host` 和 FastDDS 配置。

目标机器上可以检查串口：

```bash
ls -l /dev/ttyUSB*
ls -l /dev/serial/by-id/
```

查看容器是否在运行：

```bash
docker ps
```

进入导航容器查看环境：

```bash
docker exec -it humanoid-navigation bash
```

退出容器：

```bash
exit
```

---

## 11. 日志位置

systemd 服务日志：

```bash
journalctl -u humanoid-navigation.service -f
```

导航脚本日志：

```text
/home/ubuntu/humanoid_data/debug_logs/
```

建图日志：

```text
/home/ubuntu/humanoid_data/debug_logs/
```

建图 rosbag：

```text
/home/ubuntu/humanoid_data/fast-lio-bags/
```

查看最近日志：

```bash
ls -lt /home/ubuntu/humanoid_data/debug_logs | head
```

---

## 12. 修改源码后如何更新镜像

不要直接修改目标机器容器里的源码用于量产。

正确方式是在开发机的 Ubuntu 工作区修改源码：

```bash
cd /home/ubuntu/humanoid_ws
```

修复 bug 后重新构建镜像：

```bash
./deploy/docker_offline/scripts/build_image.sh
```

重新导出镜像：

```bash
./deploy/docker_offline/scripts/save_image.sh
```

再把新的 `deploy/docker_offline` 拷到 U 盘，到目标机器重新安装。

推荐每次修复 bug 后升级版本号。编辑：

```bash
/home/ubuntu/humanoid_ws/deploy/docker_offline/humanoid.env
```

例如从：

```bash
IMAGE_NAME=humanoid_nav:1.0.0
IMAGE_TAR=images/humanoid_nav_1.0.0.tar
```

改为：

```bash
IMAGE_NAME=humanoid_nav:1.0.1
IMAGE_TAR=images/humanoid_nav_1.0.1.tar
```

如果旧版本是 `1.0.1`，这次想发布新版本 `1.1.1`，就把：

```bash
IMAGE_NAME=humanoid_nav:1.0.1
IMAGE_TAR=images/humanoid_nav_1.0.1.tar
```

改为：

```bash
IMAGE_NAME=humanoid_nav:1.1.1
IMAGE_TAR=images/humanoid_nav_1.1.1.tar
```

保存后确认配置：

```bash
cd /home/ubuntu/humanoid_ws
grep -E '^(IMAGE_NAME|IMAGE_TAR)=' deploy/docker_offline/humanoid.env
```

应该看到：

```text
IMAGE_NAME=humanoid_nav:1.1.1
IMAGE_TAR=images/humanoid_nav_1.1.1.tar
```

然后重新构建和导出：

```bash
cd /home/ubuntu/humanoid_ws
./deploy/docker_offline/scripts/build_image.sh
./deploy/docker_offline/scripts/save_image.sh
```

新生成的离线镜像包会变成：

```text
/home/ubuntu/humanoid_ws/deploy/docker_offline/images/humanoid_nav_1.1.1.tar
/home/ubuntu/humanoid_ws/deploy/docker_offline/images/humanoid_nav_1.1.1.tar.sha256
```

拷贝到 U 盘前可以先校验：

```bash
cd /home/ubuntu/humanoid_ws/deploy/docker_offline
sha256sum -c images/humanoid_nav_1.1.1.tar.sha256
```

目标机器更新：

```bash
cd /media/ubuntu/<U盘名>/docker_offline
sudo ./scripts/install_offline.sh
sudo systemctl restart humanoid-navigation.service
```

目标机器安装后检查版本：

```bash
docker image ls | grep humanoid_nav
systemctl status humanoid-navigation.service
```

这种方式最清楚：现场一看镜像版本，就知道当前跑的是哪一版。

如果临时仍然使用同一个镜像名，比如还叫 `humanoid_nav:1.0.0`，目标机器需要强制重新加载镜像：

```bash
cd /media/ubuntu/<U盘名>/docker_offline
sudo FORCE_LOAD_IMAGE=1 ./scripts/install_offline.sh
sudo systemctl restart humanoid-navigation.service
```

否则目标机器发现已有 `humanoid_nav:1.0.0`，可能会跳过加载新 tar。

---

## 13. 更新镜像会不会覆盖地图

正常不会。

镜像里的源码和环境在 Docker image 中，现场地图和数据在主机目录：

```text
/home/ubuntu/humanoid_data
```

更新镜像时，`/home/ubuntu/humanoid_data` 不会被删除，也不会被覆盖。

安装脚本只会在 `maps`、`pcd`、`data` 目录为空时，才从镜像复制默认数据。现场已经建过图后，这些目录不为空，所以不会覆盖现场地图。

建议现场重要地图定期备份：

```bash
tar -czf humanoid_data_backup_$(date +%Y%m%d_%H%M%S).tar.gz /home/ubuntu/humanoid_data
```

---

## 14. 常见问题

### 14.1 docker: command not found

说明目标机器没有安装 Docker。

如果现场可以联网，可以先安装 Docker。  
如果现场不能联网，需要提前准备 Docker 离线 deb 包，并放到：

```text
docker_offline/docker_debs/
```

然后重新执行：

```bash
sudo ./scripts/install_offline.sh
```

### 14.2 启动导航后 APP 连不上

检查导航服务是否运行：

```bash
systemctl status humanoid-navigation.service
```

检查 websocket 端口：

```bash
ss -lntp | grep 8765
```

检查 APP 机器和机器人是否在同一网络，IP 是否正确。

### 14.3 找不到串口

检查设备：

```bash
ls -l /dev/ttyUSB*
ls -l /dev/serial/by-id/
```

重新插拔 USB 设备后再看。

### 14.4 建图时 RViz 打不开

确认目标机器有桌面环境，并且当前用户允许 Docker 访问 X11。

脚本已经自动执行：

```bash
xhost +local:docker
```

如果仍打不开，可以手动执行：

```bash
xhost +local:docker
cd /home/ubuntu/humanoid_deploy
./scripts/run_mapping.sh hall
```

### 14.5 建图后导航还是旧地图

确认建图时使用的是 `hall`：

```bash
./scripts/run_mapping.sh hall
```

确认保存成功：

```bash
ls -lh /home/ubuntu/humanoid_data/maps/hall.*
ls -lh /home/ubuntu/humanoid_data/pcd/hall*
```

重启导航：

```bash
sudo systemctl restart humanoid-navigation.service
```

### 14.6 更新镜像后代码没有变化

如果没有改版本号，目标机器可能跳过了镜像加载。

解决方式一：推荐改版本号，例如 `1.0.1`。

解决方式二：强制加载同名镜像：

```bash
sudo FORCE_LOAD_IMAGE=1 ./scripts/install_offline.sh
sudo systemctl restart humanoid-navigation.service
```

### 14.7 雷达点云频率不够或 FastLIO 很慢

确认导航是通过脚本启动的，不要手写简化版 `docker run`。

脚本里包含：

```text
--network host
--ipc host
FASTRTPS_DEFAULT_PROFILES_FILE=/home/ubuntu/.config/fastdds_shm.xml
RMW_FASTRTPS_USE_QOS_FROM_XML=1
```

这些是 DDS 共享内存和大点云通信需要的配置。

---

## 15. 现场推荐操作顺序

首次部署：

```bash
cd /media/ubuntu/<U盘名>/docker_offline
sha256sum -c images/humanoid_nav_1.0.0.tar.sha256
sudo ./scripts/install_offline.sh
cd /home/ubuntu/humanoid_deploy
./scripts/check_deploy.sh
```

首次建图：

```bash
sudo systemctl stop humanoid-navigation.service
cd /home/ubuntu/humanoid_deploy
./scripts/run_mapping.sh hall
```

另开终端保存地图：

```bash
cd /home/ubuntu/humanoid_deploy
./scripts/finish_mapping.sh
```

启动导航：

```bash
sudo systemctl restart humanoid-navigation.service
journalctl -u humanoid-navigation.service -f
```

后续更新代码：

```bash
# 开发机
cd /home/ubuntu/humanoid_ws
./deploy/docker_offline/scripts/build_image.sh
./deploy/docker_offline/scripts/save_image.sh

# 目标机器
cd /media/ubuntu/<U盘名>/docker_offline
sudo ./scripts/install_offline.sh
sudo systemctl restart humanoid-navigation.service
```

---

## 16. 一句话总结

构建镜像只在 `/home/ubuntu/humanoid_ws` 做；目标机器只负责 `install_offline.sh` 安装、`run_mapping.sh hall` 建图、`finish_mapping.sh` 保存、`systemctl restart humanoid-navigation.service` 启动导航。源码修复后不要改容器，重新构建镜像并重新离线安装。
