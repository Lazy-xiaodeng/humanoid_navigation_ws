# humanoid_ws 系统迁移指南

本指南详细说明如何将 `humanoid_ws` 工作空间从当前系统迁移到新的 ROS2 Jazzy 环境。

---

## 📋 目录

1. [系统要求](#1-系统要求)
2. [环境准备](#2-环境准备)
3. [源码迁移](#3-源码迁移)
4. [编译工作空间](#4-编译工作空间)
5. [常见问题解决](#5-常见问题解决)
6. [运行测试](#6-运行测试)

---

## 1. 系统要求

### 硬件要求
- **CPU**: Intel Core Ultra 系列或其他支持 Intel GPU 的处理器
- **GPU**: Intel 集成显卡（用于 SYCL 加速，可选）
- **内存**: 最低 8GB，推荐 16GB+
- **存储**: 至少 20GB 可用空间

### 软件要求
- **操作系统**: Ubuntu 24.04 LTS (Noble)
- **ROS2 版本**: ROS2 Jazzy Jalisco
- **编译器**: GCC 13+ 或 Intel oneAPI 2025.3+
- **CMake**: 3.20+

---

## 2. 环境准备

### 2.1 安装 ROS2 Jazzy

```bash
# 设置 locale
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# 添加 ROS2 GPG 密钥
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# 添加软件源
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装 ROS2 Jazzy 桌面版
sudo apt update
sudo apt install ros-jazzy-desktop-full

# 设置环境变量
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 安装开发工具
sudo apt install python3-colcon-common-extensions python3-rosdep2 python3-vcstool
```

### 2.2 安装 Intel oneAPI（用于 SYCL GPU 加速）

```bash
# 添加 Intel oneAPI 软件源
wget -O- https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB \
| gpg --dearmor | sudo tee /usr/share/keyrings/oneapi-archive-keyring.gpg > /dev/null

echo "deb [signed-by=/usr/share/keyrings/oneapi-archive-keyring.gpg] https://apt.repos.intel.com/oneapi all main" | \
sudo tee /etc/apt/sources.list.d/oneAPI.list

sudo apt update

# 安装 oneAPI 基础组件和 DPC++ 编译器
sudo apt install intel-oneapi-basekit intel-oneapi-dpcpp-cpp intel-oneapi-opencl-rt

# 设置环境变量（添加到 ~/.bashrc）
echo "source /opt/intel/oneapi/setvars.sh" >> ~/.bashrc
source ~/.bashrc
```

### 2.3 安装 ROS2 依赖包

```bash
# 初始化 rosdep
sudo rosdep init
rosdep update

# 安装工作空间依赖
cd ~/humanoid_ws
rosdep install -i --from-path src --rosdistro jazzy -y

# 或者手动安装关键依赖
sudo apt install \
  ros-jazzy-cv-bridge \
  ros-jazzy-pcl-ros \
  ros-jazzy-pcl-conversions \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-eigen \
  ros-jazzy-sensor-msgs \
  ros-jazzy-geometry-msgs \
  ros-jazzy-nav-msgs \
  ros-jazzy-nav2-msgs \
  ros-jazzy-visualization-msgs \
  ros-jazzy-rviz2 \
  ros-jazzy-rviz-common \
  ros-jazzy-rviz-rendering \
  ros-jazzy-octomap \
  ros-jazzy-octomap-msgs \
  ros-jazzy-filters \
  ros-jazzy-ament-index-cpp \
  ros-jazzy-rosbag2-cpp \
  libeigen3-dev \
  libpcl-dev \
  libopencv-dev \
  libopencl-clang-dev \
  ocl-icd-opencl-dev
```

---

## 3. 源码迁移

### 3.1 拷贝源码

```bash
# 方法 A: 使用 tar 打包（推荐）
# 在旧系统上
cd ~/humanoid_ws
tar -czf humanoid_ws_src.tar.gz src/

# 拷贝到新系统
scp humanoid_ws_src.tar.gz user@new-system:~/

# 在新系统上解压
mkdir -p ~/humanoid_ws
cd ~/humanoid_ws
tar -xzf humanoid_ws_src.tar.gz

# 方法 B: 使用 rsync 同步
rsync -avz --exclude='build' --exclude='install' --exclude='log' \
  ~/humanoid_ws/src/ user@new-system:~/humanoid_ws/src/

# 方法 C: 使用 Git（如果已版本控制）
cd ~/humanoid_ws/src
git clone <repository-url> .
```

### 3.2 清理和权限修复

```bash
cd ~/humanoid_ws

# 清理旧的构建文件
rm -rf build/ install/ log/

# 修复文件权限
find src/ -type f -name "*.sh" -exec chmod +x {} \;

# 初始化 rosdep（如果未执行）
sudo rosdep init
rosdep update

# 安装缺失的依赖
rosdep install -i --from-path src --rosdistro jazzy -y --skip-keys=libconsole-bridge-dev
```

---

## 4. 编译工作空间

### 4.1 环境变量自动配置（重要！）

为了让新终端自动加载环境变量，请执行以下命令：

```bash
# 编辑 ~/.bashrc
nano ~/.bashrc

# 添加以下内容到文件末尾：
export ROS_DISTRO=jazzy
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# Intel oneAPI（如需 GPU 加速）
if [ -f /opt/intel/oneapi/setvars.sh ]; then
    source /opt/intel/oneapi/setvars.sh > /dev/null 2>&1
fi

# humanoid_ws 工作空间
if [ -f ~/humanoid_ws/install/setup.bash ]; then
    source ~/humanoid_ws/install/setup.bash
fi

# 保存后执行以下命令使其生效
source ~/.bashrc
```

**验证环境变量：**
```bash
# 运行环境检查脚本
~/humanoid_ws/check_env.sh

# 或手动检查
echo $ROS_DISTRO      # 应输出：jazzy
echo $AMENT_PREFIX_PATH  # 应包含：/opt/ros/jazzy
echo $ONEAPI_ROOT     # 应输出：/opt/intel/oneapi
```

### 4.2 标准编译

```bash
cd ~/humanoid_ws

# 设置环境变量
source /opt/ros/jazzy/setup.bash
source /opt/intel/oneapi/setvars.sh 2>/dev/null || true

# 编译（使用符号链接安装）
colcon build --symlink-install --parallel-workers $(nproc)

# 或者只编译特定包
colcon build --symlink-install --packages-select humanoid_point_cloud_filter
```

### 4.2 选择性编译（跳过有问题的包）

```bash
# 如果有包编译失败，可以先跳过
colcon build --symlink-install \
  --packages-ignore humanoid_navigation2 \
  --parallel-workers $(nproc)
```

### 4.3 设置环境变量

```bash
# 添加到 ~/.bashrc
echo '' >> ~/.bashrc
echo '# humanoid_ws 环境' >> ~/.bashrc
echo 'source ~/humanoid_ws/install/setup.bash' >> ~/.bashrc
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
echo 'source /opt/intel/oneapi/setvars.sh 2>/dev/null || true' >> ~/.bashrc

# 使配置生效
source ~/.bashrc
```

---

## 5. 常见问题解决

### 5.1 cv_bridge 头文件错误

**错误信息:**
```
fatal error: cv_bridge/cv_bridge.h: 没有那个文件或目录
```

**解决方案:**
```bash
# 安装 cv_bridge
sudo apt install ros-jazzy-cv-bridge

# 在 CMakeLists.txt 中添加
find_package(cv_bridge REQUIRED)
include_directories(${cv_bridge_INCLUDE_DIRS} /opt/ros/jazzy/include)
```

### 5.2 SYCL 编译器未找到

**错误信息:**
```
c++: error: unrecognized command-line option '-fsycl'
```

**解决方案:**
```bash
# 安装 Intel oneAPI
sudo apt install intel-oneapi-dpcpp-cpp

# 设置环境变量
source /opt/intel/oneapi/setvars.sh

# 或者使用 CPU 模式（修改 CMakeLists.txt）
# 将 USE_SYCL 设置为 OFF
```

### 5.3 OpenCL 设备未找到

**错误信息:**
```
未找到任何 OpenCL 平台！请检查驱动安装。
```

**解决方案:**
```bash
# 安装 OpenCL 驱动
sudo apt install intel-opencl-icd ocl-icd-opencl-dev

# 检查设备
clinfo

# 重启系统
sudo reboot
```

### 5.4 Python 包配置错误

**错误信息:**
```
CMake Error: The source directory does not appear to contain CMakeLists.txt
```

**解决方案:**
```bash
# 修改 package.xml
# 将 <build_type>ament_cmake</build_type> 改为 <build_type>ament_python</build_type>
# 移除 <buildtool_depend>ament_cmake</buildtool_depend>

# 清理并重新编译
rm -rf build/<package_name> install/<package_name>
colcon build --symlink-install --packages-select <package_name>
```

### 5.5 Kernel 文件未找到

**错误信息:**
```
无法打开 kernel 文件：/path/to/kernels/file.cl
```

**解决方案:**
```bash
# 检查 kernel 文件是否存在
ls -la src/<package_name>/kernels/

# 确保 CMakeLists.txt 中有安装配置
install(DIRECTORY kernels/
  DESTINATION share/${PROJECT_NAME}/kernels
)

# 重新编译并安装
colcon build --symlink-install --packages-select <package_name>
```

---

## 6. 运行测试

### 6.1 基础测试

```bash
# 设置环境
source ~/humanoid_ws/install/setup.bash

# 列出所有可用的节点
ros2 pkg executables | grep humanoid

# 测试点云滤波节点
ros2 run humanoid_point_cloud_filter point_cloud_filter_node

# 测试地形分析节点
ros2 run humanoid_terrain_analyzer terrain_analyzer_node

# 测试高程图映射
ros2 run elevation_mapping_opencl elevation_mapping_opencl_node
```

### 6.2 Launch 文件测试

```bash
# 启动点云滤波
ros2 launch humanoid_point_cloud_filter point_cloud_filter.launch.py

# 启动导航系统
ros2 launch humanoid_navigation2 navigation.launch.py

# 启动完整系统
ros2 launch humanoid_bringup bringup.launch.py
```

### 6.3 GPU 加速验证

```bash
# 检查 GPU 使用情况
intel_gpu_top  # 需要 intel-gpu-tools 包

# 查看 OpenCL 设备
clinfo | grep -E "Device Name|Device Type"

# 测试 SYCL 设备
sycl-ls --verbose
```

---

## 📝 快速检查清单

迁移完成后，请确认以下项目：

- [ ] ROS2 Jazzy 已安装并配置
- [ ] Intel oneAPI 已安装（如需 GPU 加速）
- [ ] 所有依赖包已安装 (`rosdep install`)
- [ ] 源码已拷贝且权限正确
- [ ] 工作空间编译成功 (`colcon build`)
- [ ] 环境变量已设置 (`source install/setup.bash`)
- [ ] 基础节点可以运行
- [ ] GPU 加速功能正常（如适用）

---

## 🔗 相关资源

- [ROS2 Jazzy 官方文档](https://docs.ros.org/en/jazzy/)
- [Intel oneAPI 文档](https://www.intel.com/content/www/us/en/developer/tools/oneapi.html)
- [colcon 使用指南](https://colcon.readthedocs.io/)
- [grid_map 文档](https://github.com/ANYbotics/grid_map)

---

## 📞 技术支持

如遇到问题，请检查：
1. 编译日志：`log/latest_build/`
2. 系统日志：`journalctl -u ros2`
3. 环境配置：`env | grep -E "ROS|CL|ONEAPI"`

---

**最后更新**: 2026 年 4 月 15 日
**适用版本**: ROS2 Jazzy + Ubuntu 24.04
