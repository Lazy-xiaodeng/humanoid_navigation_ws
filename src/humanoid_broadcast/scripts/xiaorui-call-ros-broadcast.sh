#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/jazzy/setup.bash
if [[ -f "/home/ubuntu/software/Todesk/Files/humanoid_ws/install/setup.bash" ]]; then
  source /home/ubuntu/software/Todesk/Files/humanoid_ws/install/setup.bash
fi
set -u

exec ros2 run humanoid_broadcast call_broadcast_service
