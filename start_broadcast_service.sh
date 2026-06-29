#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

set +u
source /opt/ros/jazzy/setup.bash
if [[ -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  source "${WORKSPACE_DIR}/install/setup.bash"
fi
set -u

export XIAORUI_AUDIO_BACKEND="${XIAORUI_AUDIO_BACKEND:-auto}"
export XIAORUI_AUDIO_SINK="${XIAORUI_AUDIO_SINK:-auto}"
export XIAORUI_ALSA_DEVICE="${XIAORUI_ALSA_DEVICE:-auto}"
export XIAORUI_BROADCAST_PLAYER="${XIAORUI_BROADCAST_PLAYER:-dry_run}"
export XIAORUI_BROADCAST_DEFAULT_VOLUME="${XIAORUI_BROADCAST_DEFAULT_VOLUME:-72}"
export USE_CPP_BROADCAST_RUNTIME="${USE_CPP_BROADCAST_RUNTIME:-true}"

echo "Starting xiaorui broadcast service..."
echo "  XIAORUI_AUDIO_BACKEND=${XIAORUI_AUDIO_BACKEND}"
echo "  XIAORUI_AUDIO_SINK=${XIAORUI_AUDIO_SINK}"
echo "  XIAORUI_ALSA_DEVICE=${XIAORUI_ALSA_DEVICE}"
echo "  XIAORUI_BROADCAST_PLAYER=${XIAORUI_BROADCAST_PLAYER}"
echo "  USE_CPP_BROADCAST_RUNTIME=${USE_CPP_BROADCAST_RUNTIME}"

if [[ "${USE_CPP_BROADCAST_RUNTIME}" == "true" ]]; then
  if [[ -n "${CPP_BROADCAST_CONFIG_FILE:-}" ]]; then
    exec ros2 launch humanoid_broadcast_runtime broadcast_runtime.launch.py \
      config_file:="${CPP_BROADCAST_CONFIG_FILE}"
  fi
  exec ros2 launch humanoid_broadcast_runtime broadcast_runtime.launch.py
fi

exec ros2 launch humanoid_broadcast broadcast_service.launch.py
