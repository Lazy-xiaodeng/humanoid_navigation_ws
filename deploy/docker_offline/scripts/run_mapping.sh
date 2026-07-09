#!/bin/bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEPLOY_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
ENV_FILE="${ENV_FILE:-$DEPLOY_DIR/humanoid.env}"

if [ -f "$ENV_FILE" ]; then
  set -a
  source "$ENV_FILE"
  set +a
fi

IMAGE_NAME="${IMAGE_NAME:-humanoid_nav:1.0.0}"
MAPPING_CONTAINER_NAME="${MAPPING_CONTAINER_NAME:-humanoid-mapping}"
WORKSPACE="${WORKSPACE:-/home/ubuntu/humanoid_ws}"
HUMANOID_DATA="${HUMANOID_DATA:-/home/ubuntu/humanoid_data}"
MAP_NAME="${1:-${MAP_NAME:-hall}}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-/home/ubuntu/.config/fastdds_shm.xml}"
RMW_FASTRTPS_USE_QOS_FROM_XML="${RMW_FASTRTPS_USE_QOS_FROM_XML:-1}"

mkdir -p \
  "$HUMANOID_DATA/maps" \
  "$HUMANOID_DATA/pcd" \
  "$HUMANOID_DATA/data" \
  "$HUMANOID_DATA/debug_logs" \
  "$HUMANOID_DATA/fast-lio-bags" \
  "$HUMANOID_DATA/config"

if [ ! -f "$HUMANOID_DATA/config/fastdds_shm.xml" ]; then
  cp "$DEPLOY_DIR/config/fastdds_shm.xml" "$HUMANOID_DATA/config/fastdds_shm.xml"
fi

xhost +local:docker >/dev/null 2>&1 || true
docker rm -f "$MAPPING_CONTAINER_NAME" >/dev/null 2>&1 || true

XAUTH_ARGS=()
if [ -n "${XAUTHORITY:-}" ] && [ -f "$XAUTHORITY" ]; then
  XAUTH_ARGS=(-v "$XAUTHORITY:/home/ubuntu/.Xauthority:ro" -e XAUTHORITY=/home/ubuntu/.Xauthority)
elif [ -f /home/ubuntu/.Xauthority ]; then
  XAUTH_ARGS=(-v /home/ubuntu/.Xauthority:/home/ubuntu/.Xauthority:ro -e XAUTHORITY=/home/ubuntu/.Xauthority)
fi

exec docker run --rm -it \
  --name "$MAPPING_CONTAINER_NAME" \
  --network host \
  --ipc host \
  --privileged \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e QT_X11_NO_MITSHM=1 \
  -e WORKSPACE="$WORKSPACE" \
  -e RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION" \
  -e FASTRTPS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE" \
  -e RMW_FASTRTPS_USE_QOS_FROM_XML="$RMW_FASTRTPS_USE_QOS_FROM_XML" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  "${XAUTH_ARGS[@]}" \
  -v "$HUMANOID_DATA/maps:$WORKSPACE/src/humanoid_navigation2/maps" \
  -v "$HUMANOID_DATA/maps:$WORKSPACE/install/humanoid_navigation2/share/humanoid_navigation2/maps" \
  -v "$HUMANOID_DATA/pcd:$WORKSPACE/src/humanoid_navigation2/pcd" \
  -v "$HUMANOID_DATA/pcd:$WORKSPACE/install/humanoid_navigation2/share/humanoid_navigation2/pcd" \
  -v "$HUMANOID_DATA/data:$WORKSPACE/data" \
  -v "$HUMANOID_DATA/debug_logs:$WORKSPACE/debug_logs" \
  -v "$HUMANOID_DATA/fast-lio-bags:/home/ubuntu/fast-lio-bags" \
  -v "$HUMANOID_DATA/config/fastdds_shm.xml:/home/ubuntu/.config/fastdds_shm.xml:ro" \
  "$IMAGE_NAME" \
  bash -lc "cd '$WORKSPACE' && ./start_mapping.sh '$MAP_NAME'"
