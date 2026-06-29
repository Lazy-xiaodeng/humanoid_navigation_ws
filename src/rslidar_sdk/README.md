# rslidar_sdk 在本工程中的用途

## 这个包是做什么的

`rslidar_sdk` 是 RoboSense 雷达 ROS2 驱动包，负责从真实雷达读取 UDP 数据，解析点云和 IMU，并发布给 Fast-LIO 和后续导航链路。

这是 RoboSense 官方 SDK 体系的一部分，本工程主要使用它的 ROS2 节点能力。

## 当前状态

- 当前完整导航和建图链路会启动 `rslidar_sdk_node`。
- 当前雷达类型面向 RoboSense Airy。
- 点云类型使用 `XYZIRT`，包含 ring 和 timestamp 字段。
- 共享内存 DDS 配置对原始点云 10Hz 传输很关键。

## 主要输出

- `/rslidar_points`：原始雷达点云，供 Fast-LIO 订阅。
- `/rslidar_imu_data`：雷达/IMU 数据，供 Fast-LIO 订阅。

## 主要文件说明

- `node/rslidar_sdk_node.cpp`：ROS2 驱动节点入口。
- `config/config.yaml`：雷达驱动参数，包括雷达类型、点云格式、网络配置和输出话题。
- `launch/start.py`：通用启动文件。
- `launch/humble_start.py`、`launch/elequent_start.py`：不同 ROS2 版本/环境的启动入口。
- `src/rs_driver/`：RoboSense 驱动核心代码。

## 上下游链路

上游：

- 真实 RoboSense 雷达，通过以太网 UDP 输出数据。

下游：

- `fast_lio_robosense` 订阅点云和 IMU。
- bag 录制脚本记录原始雷达数据，用于离线回放和性能验证。

## 使用方式

正式导航通常由一键脚本启动：

```bash
./start_navigation.sh
```

单独启动雷达驱动：

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
ros2 launch rslidar_sdk start.py
```

查看点云频率：

```bash
ros2 topic hz /rslidar_points
```

## 维护注意事项

- 本包是硬件驱动，升级 SDK 前要确认配置文件、点云字段和话题名是否兼容 Fast-LIO。
- 如果原始点云频率下降，优先检查网络、DDS 共享内存、CPU 占用和订阅者数量。
- 不建议在驱动包中添加业务逻辑；业务处理应放在 Fast-LIO、点云滤波或定位 runtime 包中。

---

# 1 **rslidar_sdk**

 [中文介绍](README_CN.md)

## 1 Introduction

**rslidar_sdk** is the Software Development Kit of the RoboSense Lidar based on Ubuntu. It contains:

+ The lidar driver core [rs_driver](https://github.com/RoboSense-LiDAR/rs_driver),
+ The ROS support,
+ The ROS2 support,

To get point cloud through ROS/ROS2,  please just use this SDK.

To integrate the Lidar driver into your own projects, please use the rs_driver.

### 1.1 LiDAR Supported

- RS-LiDAR-16
- RS-LiDAR-32
- RS-Bpearl
- RS-Helios
- RS-Helios-16P
- RS-Ruby-128
- RS-Ruby-80
- RS-Ruby-48
- RS-Ruby-Plus-128
- RS-Ruby-Plus-80
- RS-Ruby-Plus-48
- RS-LiDAR-M1
- RS-LiDAR-M2
- RS-LiDAR-M3
- RS-LiDAR-E1
- RS-LiDAR-MX
- RS-LiDAR-AIRY

### 1.2 Point Type Supported

- XYZI - x, y, z, intensity
- XYZIRT - x, y, z, intensity, ring, timestamp

## 2 Download

### 2.1 Download via Git

Download the rslidar_sdk as below. Since it contains the submodule rs_driver, please also use `git submodule` to download the submodule properly.

```sh
git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git
cd rslidar_sdk
git submodule init
git submodule update
```

### 2.2 Download directly

Instead of using Git, user can also access [rslidar_sdk_release](https://github.com/RoboSense-LiDAR/rslidar_sdk/releases) to download the latest version of rslidar_sdk.

Please download the **rslidar_sdk.tar.gz** archive instead of Source code. The Source code zip file does not contain the submodule rs_driver, so it has to be downloaded manaully.
![](./img/01_01_download_page.png)

## 3 Dependencies

### 3.1 ROS

To run rslidar_sdk in the ROS environment, please install below libraries.

+ Ubuntu 16.04 - ROS Kinetic desktop
+ Ubuntu 18.04 - ROS Melodic desktop
+ Ubuntu 20.04 - ROS Noetic desktop

For installation, please refer to http://wiki.ros.org.

**It's highly recommanded to install ros-distro-desktop-full**. If you do so, the corresponding libraries, such as PCL, will be installed at the same time.

This brings a lot of convenience, since you don't have to handle version conflict.

### 3.2 ROS2

To use rslidar_sdk in the ROS2 environment, please install below libraries.

+ Ubuntu 16.04 - Not supported
+ Ubuntu 18.04 - ROS2 Eloquent desktop
+ Ubuntu 20.04 - ROS2 Galactic desktop
+ Ubuntu 22.04 - ROS2 Humble desktop

For installation, please refer to https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Install-Debians/

**Please do not install ROS and ROS2 on the same computer, to avoid possible conflict and manually install some libraries, such as Yaml.**

### 3.3 Yaml (Essential)

version: >= v0.5.2

*If ros-distro-desktop-full is installed, this step can be skipped*

Installation:

```sh
sudo apt-get update
sudo apt-get install -y libyaml-cpp-dev
```

### 3.4 libpcap (Essential)

version: >= v1.7.4

Installation:

```sh
sudo apt-get install -y  libpcap-dev
```

## 4 Compile & Run

### 4.1 Compile with ROS catkin tools

(1) Create a new workspace folder, and create a *src* folder in it. Then put the rslidar_sdk project into the *src* folder.

(2) Go back to the root of workspace, run the following commands to compile and run. (if using zsh, replace the 2nd command with *source devel/setup.zsh*).

```sh
catkin_make
source devel/setup.bash
roslaunch rslidar_sdk start.launch
```

### 4.2 Compile with ROS2 colcon

(1) Create a new workspace folder, and create a *src* folder in it. Then put the rslidar_sdk project in the *src* folder.

(2) Download the packet definition project in ROS2 through [link](https://github.com/RoboSense-LiDAR/rslidar_msg), then put the project rslidar_msg in the *src* folder you just created.

(3) Go back to the root of workspace, run the following commands to compile and run. (if using zsh, replace the 2nd command with *source install/setup.zsh*).

```sh
colcon build
source install/setup.bash
ros2 launch rslidar_sdk start.py
```

Another version of start.py may be used, since it is different on different versios of ROS2. For example, elequent_start.py is used instead for ROS2 elequent.

## 5 Introduction to parameters

To change behaviors of rslidar_sdk, change its parameters. please read the following links for detail information.

[Intro to parameters](doc/intro/02_parameter_intro.md)

[Intro to hidden parameters](doc/intro/03_hiding_parameters_intro.md)

## 6 Quick start

Below are some quick guides to use rslidar_sdk.

[Connect to online LiDAR and send point cloud through ROS](doc/howto/06_how_to_decode_online_lidar.md)

[Decode PCAP file and send point cloud through ROS](doc/howto/08_how_to_decode_pcap_file.md)

[Change Point Type](doc/howto/05_how_to_change_point_type.md)

## 7 Advanced Topics

[Online Lidar - Advanced topics](doc/howto/07_online_lidar_advanced_topics.md)

[PCAP file - Advanced topics](doc/howto/09_pcap_file_advanced_topics.md)

[Coordinate Transformation](doc/howto/10_how_to_use_coordinate_transformation.md)

[Record rosbag &amp; Replay it](doc/howto/11_how_to_record_replay_packet_rosbag.md)

[Solution for ROS2_humble frame rate reduction](doc/howto/13_how_to_solve_ROS2_humble_frame_rate_drop.md)
