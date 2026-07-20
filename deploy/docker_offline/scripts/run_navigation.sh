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
CONTAINER_NAME="${CONTAINER_NAME:-humanoid-navigation}"
WORKSPACE="${WORKSPACE:-/home/ubuntu/humanoid_ws}"
HUMANOID_DATA="${HUMANOID_DATA:-/home/ubuntu/humanoid_data}"
SKIP_COLCON_BUILD="${SKIP_COLCON_BUILD:-1}"
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

docker rm -f "$CONTAINER_NAME" >/dev/null 2>&1 || true

exec docker run --rm \
  --name "$CONTAINER_NAME" \
  --network host \
  --ipc host \
  --privileged \
  -e WORKSPACE="$WORKSPACE" \
  -e SKIP_COLCON_BUILD="$SKIP_COLCON_BUILD" \
  -e RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION" \
  -e FASTRTPS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE" \
  -e RMW_FASTRTPS_USE_QOS_FROM_XML="$RMW_FASTRTPS_USE_QOS_FROM_XML" \
  -v "$HUMANOID_DATA/maps:$WORKSPACE/src/humanoid_navigation2/maps" \
  -v "$HUMANOID_DATA/maps:$WORKSPACE/install/humanoid_navigation2/share/humanoid_navigation2/maps" \
  -v "$HUMANOID_DATA/pcd:$WORKSPACE/src/humanoid_navigation2/pcd" \
  -v "$HUMANOID_DATA/pcd:$WORKSPACE/install/humanoid_navigation2/share/humanoid_navigation2/pcd" \
  -v "$HUMANOID_DATA/data:$WORKSPACE/data" \
  -v "$HUMANOID_DATA/debug_logs:$WORKSPACE/debug_logs" \
  -v "$HUMANOID_DATA/fast-lio-bags:/home/ubuntu/fast-lio-bags" \
  -v "$HUMANOID_DATA/config/fastdds_shm.xml:/home/ubuntu/.config/fastdds_shm.xml:ro" \
  "$IMAGE_NAME" \
  bash -lc "cd '$WORKSPACE' && ./start_navigation.sh && nav_pid=\"\$(cat .start_navigation.pid)\" && trap './stop_navigation.sh || true; exit 0' INT TERM; while kill -0 \"\$nav_pid\" >/dev/null 2>&1; do sleep 5; done; echo 'navigation process exited'; exit 1"
