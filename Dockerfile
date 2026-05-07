FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive

# ============================================================
# 安装所有依赖（全部用 apt 包名硬编码，不依赖 rosdep / GitHub）
# ============================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    # ---- ROS2 构建工具 ----
    ros-jazzy-ament-cmake \
    ros-jazzy-ament-cmake-auto \
    ros-jazzy-ament-cmake-core \
    ros-jazzy-ament-cmake-gtest \
    ros-jazzy-ament-cmake-python \
    ros-jazzy-ament-copyright \
    ros-jazzy-ament-flake8 \
    ros-jazzy-ament-index-cpp \
    ros-jazzy-ament-lint-auto \
    ros-jazzy-ament-lint-common \
    ros-jazzy-ament-pep257 \
    ros-jazzy-rosidl-default-generators \
    ros-jazzy-rosidl-default-runtime \
    ros-jazzy-ros-environment \
    # ---- 通用 ROS2 核心 ----
    ros-jazzy-rclcpp \
    ros-jazzy-rclcpp-lifecycle \
    ros-jazzy-rclpy \
    ros-jazzy-rcutils \
    ros-jazzy-builtin-interfaces \
    ros-jazzy-ros-testing \
    # ---- 消息包 ----
    ros-jazzy-geometry-msgs \
    ros-jazzy-lifecycle-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-sensor-msgs-py \
    ros-jazzy-std-msgs \
    ros-jazzy-std-srvs \
    ros-jazzy-nav-msgs \
    ros-jazzy-visualization-msgs \
    # ---- TF2 ----
    ros-jazzy-tf2 \
    ros-jazzy-tf2-eigen \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-ros-py \
    ros-jazzy-tf2-sensor-msgs \
    # ---- 感知 ----
    ros-jazzy-cv-bridge \
    ros-jazzy-pcl-conversions \
    ros-jazzy-pcl-ros \
    ros-jazzy-pointcloud-to-laserscan \
    ros-jazzy-slam-toolbox \
    ros-jazzy-eigen3-cmake-module \
    ros-jazzy-filters \
    # ---- 导航 Nav2 ----
    ros-jazzy-nav2-behaviors \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-bt-navigator \
    ros-jazzy-nav2-common \
    ros-jazzy-nav2-controller \
    ros-jazzy-nav2-core \
    ros-jazzy-nav2-costmap-2d \
    ros-jazzy-nav2-lifecycle-manager \
    ros-jazzy-nav2-msgs \
    ros-jazzy-nav2-planner \
    ros-jazzy-nav2-regulated-pure-pursuit-controller \
    ros-jazzy-nav2-smac-planner \
    # ---- OctoMap ----
    ros-jazzy-octomap \
    ros-jazzy-octomap-msgs \
    ros-jazzy-octomap-rviz-plugins \
    ros-jazzy-octomap-server \
    # ---- 机器人描述与状态 ----
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-urdf \
    ros-jazzy-xacro \
    ros-jazzy-pluginlib \
    ros-jazzy-launch \
    ros-jazzy-launch-ros \
    # ---- 可视化 ----
    ros-jazzy-rviz2 \
    ros-jazzy-rviz-common \
    ros-jazzy-rviz-ogre-vendor \
    ros-jazzy-rviz-rendering \
    # ---- 数据记录 ----
    ros-jazzy-ros2bag \
    ros-jazzy-rosbag2-cpp \
    ros-jazzy-rosbag2-py \
    ros-jazzy-rosbag2-storage-default-plugins \
    # ---- 系统库 ----
    libpcl-dev \
    libeigen3-dev \
    libtbb-dev \
    libyaml-cpp-dev \
    libpcap0.8-dev \
    qtbase5-dev \
    libqt5core5t64 \
    libqt5gui5t64 \
    libqt5widgets5t64 \
    # ---- OpenCL (高程建图) ----
    ocl-icd-opencl-dev \
    # ---- Python 依赖 ----
    python3-numpy \
    python3-opencv \
    python3-pytest \
    python3-yaml \
    python3-pip \
    python3-websockets \
    python3-colcon-common-extensions \
    # ---- 开发调试工具 ----
    vim \
    nano \
    git \
    gdb \
    htop \
    strace \
    net-tools \
    iputils-ping \
    # 清理 apt 缓存
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# 设置工作空间目录
# ============================================================
RUN mkdir -p /ws/src
WORKDIR /ws

# ============================================================
# 入口脚本
# ============================================================
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# 将环境变量写入 bashrc，进入容器自动生效
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc \
    && echo "[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
