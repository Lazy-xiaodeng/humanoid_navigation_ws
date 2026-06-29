#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 默认使用脚本所在工作区，避免从 Todesk 脚本误启动主工作区。
WORKSPACE="${WORKSPACE:-$SCRIPT_DIR}"

cd "$WORKSPACE"

source /opt/ros/jazzy/setup.bash
source "$WORKSPACE/install/setup.bash"

# WebSocket 已并入 C++ 控制面入口：APP 网关、数据整合、机器人网关和表情运行层统一启动。
exec ros2 launch humanoid_bringup robot_control_plane.launch.py
