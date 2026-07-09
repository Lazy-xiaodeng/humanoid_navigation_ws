#!/bin/bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEPLOY_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
ENV_FILE="${ENV_FILE:-$DEPLOY_DIR/humanoid.env}"
OVERRIDE_IMAGE_NAME="${IMAGE_NAME:-}"
OVERRIDE_HUMANOID_DATA="${HUMANOID_DATA:-}"
OVERRIDE_WORKSPACE="${WORKSPACE:-}"

if [ -f "$ENV_FILE" ]; then
  set -a
  source "$ENV_FILE"
  set +a
fi

[ -n "$OVERRIDE_IMAGE_NAME" ] && IMAGE_NAME="$OVERRIDE_IMAGE_NAME"
[ -n "$OVERRIDE_HUMANOID_DATA" ] && HUMANOID_DATA="$OVERRIDE_HUMANOID_DATA"
[ -n "$OVERRIDE_WORKSPACE" ] && WORKSPACE="$OVERRIDE_WORKSPACE"

IMAGE_NAME="${IMAGE_NAME:-humanoid_nav:1.0.0}"
HUMANOID_DATA="${HUMANOID_DATA:-/home/ubuntu/humanoid_data}"
WORKSPACE="${WORKSPACE:-/home/ubuntu/humanoid_ws}"

require_path() {
  local path="$1"
  local kind="$2"

  if [ "$kind" = "dir" ] && [ ! -d "$path" ]; then
    echo "[check] missing directory: $path" >&2
    echo "[check] run install_offline.sh first, or set HUMANOID_DATA to an initialized data directory" >&2
    exit 1
  fi

  if [ "$kind" = "file" ] && [ ! -f "$path" ]; then
    echo "[check] missing file: $path" >&2
    echo "[check] run install_offline.sh first, or set HUMANOID_DATA to an initialized data directory" >&2
    exit 1
  fi
}

echo "[check] docker service"
docker info >/dev/null

echo "[check] image: $IMAGE_NAME"
docker image inspect "$IMAGE_NAME" >/dev/null

echo "[check] persistent data"
require_path "$HUMANOID_DATA/maps" dir
require_path "$HUMANOID_DATA/pcd" dir
require_path "$HUMANOID_DATA/data" dir
require_path "$HUMANOID_DATA/config/fastdds_shm.xml" file

echo "[check] default maps"
find "$HUMANOID_DATA/maps" -maxdepth 1 -type f | head -5
find "$HUMANOID_DATA/pcd" -maxdepth 1 -type f | head -5

echo "[check] ros packages in image"
docker run --rm \
  --network host \
  --ipc host \
  -v "$HUMANOID_DATA/maps:$WORKSPACE/src/humanoid_navigation2/maps" \
  -v "$HUMANOID_DATA/maps:$WORKSPACE/install/humanoid_navigation2/share/humanoid_navigation2/maps" \
  -v "$HUMANOID_DATA/pcd:$WORKSPACE/src/humanoid_navigation2/pcd" \
  -v "$HUMANOID_DATA/pcd:$WORKSPACE/install/humanoid_navigation2/share/humanoid_navigation2/pcd" \
  -v "$HUMANOID_DATA/config/fastdds_shm.xml:/home/ubuntu/.config/fastdds_shm.xml:ro" \
  "$IMAGE_NAME" \
  bash -lc "ros2 pkg prefix humanoid_bringup && ros2 pkg prefix humanoid_navigation2 && ros2 pkg prefix humanoid_websocket && ros2 pkg prefix humanoid_locomotion"

if command -v systemctl >/dev/null 2>&1 && systemctl list-unit-files humanoid-navigation.service >/dev/null 2>&1; then
  echo "[check] systemd enabled state: $(systemctl is-enabled humanoid-navigation.service || true)"
fi

echo "[check] OK"
