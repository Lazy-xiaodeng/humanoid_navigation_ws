#!/bin/bash
set -euo pipefail

WORKSPACE=/home/ubuntu/humanoid_ws
LOG_FILE="$WORKSPACE/debug_output.txt"

# 每次启动只保留本次日志，避免旧错误混在最新日志里。
: > "$LOG_FILE"
exec > >(tee -a "$LOG_FILE") 2>&1

echo "Starting Humanoid Navigation at $(date)"

cd "$WORKSPACE"

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "ERROR: /opt/ros/jazzy/setup.bash not found"
  exit 1
fi

if [ ! -f "$WORKSPACE/install/setup.bash" ]; then
  echo "ERROR: $WORKSPACE/install/setup.bash not found. Run colcon build first."
  exit 1
fi

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

source "$WORKSPACE/install/setup.bash"

python3 - <<'PY'
import importlib.metadata as metadata

required = ("humanoid-navigation", "humanoid-websocket")
missing = []
for package_name in required:
    try:
        metadata.distribution(package_name)
    except metadata.PackageNotFoundError:
        missing.append(package_name)

if missing:
    raise SystemExit(
        "ERROR: missing Python package metadata after sourcing install/setup.bash: "
        + ", ".join(missing)
    )
PY

echo "ROS environment loaded from $WORKSPACE/install/setup.bash"

ros2 launch humanoid_bringup robot_real.launch.py use_sim_time:=false
