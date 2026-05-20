#!/bin/bash
set -euo pipefail

WORKSPACE=/home/ubuntu/humanoid_ws

cd "$WORKSPACE"

source /opt/ros/jazzy/setup.bash
source "$WORKSPACE/install/setup.bash"

ros2 launch humanoid_websocket websocket_server.launch.py
