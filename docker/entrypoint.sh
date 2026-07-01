#!/bin/bash
set -e

export WORKSPACE="${WORKSPACE:-/home/ubuntu/humanoid_ws}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-/home/ubuntu/.config/fastdds_shm.xml}"
export RMW_FASTRTPS_USE_QOS_FROM_XML="${RMW_FASTRTPS_USE_QOS_FROM_XML:-1}"
export Open3D_DIR="${Open3D_DIR:-/opt/open3d/lib/cmake/Open3D}"
export LD_LIBRARY_PATH="/opt/open3d/lib:${LD_LIBRARY_PATH}"

# 加载 ROS2 环境
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "[entrypoint] ERROR: /opt/ros/jazzy/setup.bash not found" >&2
    exit 1
fi

# 如果工作空间已经编译过，加载其环境
if [ -f "$WORKSPACE/install/setup.bash" ]; then
    source "$WORKSPACE/install/setup.bash"
    echo "[entrypoint] loaded workspace: $WORKSPACE/install/setup.bash"
else
    echo "[entrypoint] WARN: workspace install not found: $WORKSPACE/install/setup.bash"
fi

# 执行传入的命令
exec "$@"
