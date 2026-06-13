#!/bin/bash
set -eo pipefail
set +u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${WORKSPACE:-$SCRIPT_DIR}"
PID_FILE="$WORKSPACE/.start_navigation.pid"

cd "$WORKSPACE"

# 完整停止入口：只有关机、重启整套 ROS 系统、手工清理时使用。
# 地图切换时禁止调用本脚本，否则 websocket/地图上下文会被停掉，APP 无法收到切图进度。
if [ -x "$WORKSPACE/stop_navigation_stack.sh" ]; then
  "$WORKSPACE/stop_navigation_stack.sh" || true
fi
if [ -x "$WORKSPACE/stop_ros_control_plane.sh" ]; then
  "$WORKSPACE/stop_ros_control_plane.sh" || true
fi

rm -f "$PID_FILE"

if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
fi
if command -v ros2 >/dev/null 2>&1; then
  ros2 daemon stop >/dev/null 2>&1 || true
fi

echo "Full navigation system cleanup complete."
