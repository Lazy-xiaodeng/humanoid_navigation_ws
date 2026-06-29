#!/bin/bash
set -eo pipefail
set +u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 默认使用脚本所在 Todesk 工作区，避免误切到主工作区在线源码。
WORKSPACE="${WORKSPACE:-$SCRIPT_DIR}"
REQUESTED_MAP_ID="${MAP_ID:-${1:-}}"
START_TIME="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$WORKSPACE/debug_logs"
LOG_FILE="$LOG_DIR/start_navigation_${START_TIME}.log"
ROS_LOG_DIR="$LOG_DIR/ros_${START_TIME}"
PID_FILE="$WORKSPACE/.start_navigation.pid"

mkdir -p "$LOG_DIR" "$ROS_LOG_DIR"
export ROS_LOG_DIR
exec > >(tee -a "$LOG_FILE") 2>&1

echo "Starting Humanoid Navigation at $(date '+%F %T %z')"
echo "Workspace: $WORKSPACE"
echo "Requested map: ${REQUESTED_MAP_ID:-current registry map}"
echo "Debug log: $LOG_FILE"
echo "ROS log dir: $ROS_LOG_DIR"

cd "$WORKSPACE"

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "ERROR: /opt/ros/jazzy/setup.bash not found"
  exit 1
fi

# 清理当前终端里可能残留的其它 ROS 工作区环境，保证 Todesk 工作区自洽运行。
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/jazzy/setup.bash

OPEN3D_PREFIX="$WORKSPACE/third_party/open3d-0.18.0"
if [ -f "$OPEN3D_PREFIX/lib/cmake/Open3D/Open3DConfig.cmake" ]; then
  export Open3D_DIR="${Open3D_DIR:-$OPEN3D_PREFIX/lib/cmake/Open3D}"
  export LD_LIBRARY_PATH="$OPEN3D_PREFIX/lib:${LD_LIBRARY_PATH:-}"
else
  echo "WARN: Open3DConfig.cmake not found under $OPEN3D_PREFIX, humanoid_prior_localization_runtime may fail to build."
fi

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.config/fastdds_shm.xml}"
export RMW_FASTRTPS_USE_QOS_FROM_XML="${RMW_FASTRTPS_USE_QOS_FROM_XML:-1}"

echo "Building workspace..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

if [ ! -f "$WORKSPACE/install/local_setup.bash" ]; then
  echo "ERROR: $WORKSPACE/install/local_setup.bash not found after build."
  exit 1
fi

# 只叠加当前 Todesk 工作区的本地安装空间，避免混入主工作区 install 的旧包。
source "$WORKSPACE/install/local_setup.bash"

python3 - <<'PYCHECK'
import subprocess
import sys

required = (
    "humanoid_bringup",
    "humanoid_navigation2",
    "humanoid_control_runtime",
    "humanoid_route_runtime",
    "humanoid_app_gateway_runtime",
    "humanoid_robot_gateway_runtime",
    "humanoid_expression_runtime",
    "humanoid_obstacle_runtime",
)
missing = []
for package_name in required:
    result = subprocess.run(
        ["ros2", "pkg", "prefix", package_name],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    if result.returncode != 0:
        missing.append(package_name)
if missing:
    sys.exit("ERROR: missing ROS package after build: " + ", ".join(missing))
PYCHECK

echo "ROS environment loaded from /opt/ros/jazzy + $WORKSPACE/install/local_setup.bash"

# 一键启动是完整启动：先清理旧的分层进程，再启动控制层和当前地图导航层。
# 运行期切图不会调用本脚本，而是调用 switch_navigation_map.sh，只重启导航层。
if [ -x "$WORKSPACE/stop_navigation.sh" ]; then
  "$WORKSPACE/stop_navigation.sh" || true
fi

WORKSPACE="$WORKSPACE" ROS_LOG_DIR="$ROS_LOG_DIR" "$WORKSPACE/start_ros_control_plane.sh"
WORKSPACE="$WORKSPACE" ROS_LOG_DIR="$ROS_LOG_DIR" MAP_ID="$REQUESTED_MAP_ID" "$WORKSPACE/start_navigation_stack.sh"

echo "$$" > "$PID_FILE"
echo "NAVIGATION_SYSTEM_STARTED_SUCCESSFULLY"
echo "Control plane PID file: $WORKSPACE/.ros_control_plane.pid"
echo "Navigation stack PID file: $WORKSPACE/.navigation_stack.pid"
echo "Debug log: $LOG_FILE"
