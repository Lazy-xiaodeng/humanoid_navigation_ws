# Ubuntu 24.04 一键部署说明

本说明用于在一台只安装了 Ubuntu 24.04 的新机器上部署当前 `humanoid_ws` 系统。脚本会安装 ROS 2 Jazzy、系统依赖、Python 依赖、可选 Intel oneAPI/OpenCL 运行环境，创建工作空间，从 GitHub 拉取源码，执行 `rosdep install` 和 `colcon build`，最后做基础自检。

ROS 2 Jazzy 官方文档确认 Ubuntu Noble 24.04 是 Jazzy 二进制包支持平台，并推荐使用 deb packages 安装方式。参考：

- https://docs.ros.org/en/jazzy/Installation.html
- https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

## 前提

- 系统：Ubuntu 24.04 LTS
- 用户：普通用户，具有 sudo 权限，不要直接用 root 执行脚本
- 网络：可以访问 Ubuntu apt 源、ROS 2 apt 源、GitHub、Intel oneAPI apt 源
- 空间：建议至少 30 GB 可用空间
- 内存：建议 16 GB 以上；内存较小时可降低 `PARALLEL_WORKERS`

## 新机器首次执行

先安装 Git 并拉取仓库：

```bash
sudo apt update
sudo apt install -y git
git clone https://github.com/Lazy-xiaodeng/humanoid_navigation_ws.git ~/humanoid_ws
cd ~/humanoid_ws
```

执行一键部署：

```bash
bash tools/bootstrap_ubuntu24_jazzy.sh
```

部署完成后，打开新终端，或执行：

```bash
source ~/.bashrc
cd ~/humanoid_ws
./start_navigation.sh
```

## 常用选项

跳过 Intel oneAPI，仅安装 OpenCL 头文件和通用运行依赖：

```bash
bash tools/bootstrap_ubuntu24_jazzy.sh --skip-oneapi
```

指定工作空间路径：

```bash
bash tools/bootstrap_ubuntu24_jazzy.sh --workspace /home/robot/humanoid_ws
```

只安装和拉源码，不编译：

```bash
bash tools/bootstrap_ubuntu24_jazzy.sh --no-build
```

编译成功后直接启动导航：

```bash
bash tools/bootstrap_ubuntu24_jazzy.sh --run-navigation
```

控制编译并发数：

```bash
PARALLEL_WORKERS=2 bash tools/bootstrap_ubuntu24_jazzy.sh
```

## 脚本做了什么

1. 检查系统必须是 Ubuntu 24.04。
2. 配置 ROS 2 Jazzy apt 源。
3. 安装 `tools/apt_dependencies_ubuntu24_jazzy.txt` 中列出的 apt 包。
4. 可选安装 Intel oneAPI 和 Intel OpenCL 运行时。
5. 使用 pip 用户级安装 `open3d`。
6. 初始化并更新 rosdep。
7. 如果目标路径已经是 Git 仓库，则执行 `git fetch`、`git checkout`、`git pull --ff-only`；如果目标路径为空，则 clone 仓库。
8. 执行 rosdep，并跳过当前工作空间内的自定义包名和少数需要手工处理的 key。
9. 从 `tools/fastdds_shm.xml` 写入 `~/.config/fastdds_shm.xml`。
10. 在 `~/.bashrc` 添加 ROS、工作空间、FastDDS、oneAPI 环境块。
11. 执行 `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`。
12. 检查 `humanoid_navigation2`、`humanoid_websocket`、BT 插件库和关键 Python 依赖是否可见。

## 失败后如何继续

脚本设计为可重复执行。修复网络、apt 源或依赖问题后，重新运行同一条命令即可。

如果已经拉取了源码，脚本会在已有 Git 工作空间内更新当前分支，不会删除 `build/`、`install/`、`log/`。如需完全干净编译，可手动执行：

```bash
cd ~/humanoid_ws
rm -rf build install log
bash tools/bootstrap_ubuntu24_jazzy.sh
```

## 仍需现场确认的内容

一键部署只能保证软件环境、源码和编译链路。以下内容仍需要按机器人现场环境确认：

- 雷达 IP、网卡、VLAN、防火墙和 UDP 端口
- 地图文件、PCD 文件、Scan Context 数据库是否匹配现场
- 机器人底盘、IMU、关节状态、WebSocket 上位机连接
- `ROS_DOMAIN_ID` 是否要和现场其他 ROS 设备隔离
- Intel GPU/OpenCL 运行时是否能被 `clinfo` 看到

## 验证命令

部署完成后可运行：

```bash
source ~/.bashrc
cd ~/humanoid_ws
ros2 pkg list | grep humanoid_navigation2
ros2 pkg executables humanoid_websocket
test -f install/humanoid_nav2_bt_nodes/lib/libhumanoid_nav2_bt_nodes.so
```

启动：

```bash
./start_navigation.sh
```
