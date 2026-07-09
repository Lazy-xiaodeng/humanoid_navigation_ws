#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${WORKSPACE:-$SCRIPT_DIR}"

cd "$WORKSPACE"

source /opt/ros/jazzy/setup.bash
source "$WORKSPACE/install/setup.bash"

ros2 launch humanoid_bringup websocket_runtime.launch.py
