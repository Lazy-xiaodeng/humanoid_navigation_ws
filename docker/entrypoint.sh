#!/bin/bash
set -e

# 加载 ROS2 环境
source /opt/ros/jazzy/setup.bash

# 如果工作空间已经编译过，加载其环境
if [ -f /ws/install/setup.bash ]; then
    source /ws/install/setup.bash
    echo "[entrypoint] 已加载工作空间环境: /ws/install/setup.bash"
else
    echo "[entrypoint] 工作空间尚未编译，请运行: colcon build --symlink-install"
fi

# 执行传入的命令
exec "$@"
