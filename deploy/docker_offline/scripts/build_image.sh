#!/bin/bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
ENV_FILE="$SCRIPT_DIR/../humanoid.env"

if [ -f "$ENV_FILE" ]; then
  set -a
  source "$ENV_FILE"
  set +a
fi

IMAGE_NAME="${IMAGE_NAME:-humanoid_nav:1.0.0}"
BASE_IMAGE="${BASE_IMAGE:-osrf/ros:jazzy-desktop-full}"

cd "$REPO_ROOT"
docker build \
  --network host \
  --build-arg BASE_IMAGE="$BASE_IMAGE" \
  -f docker/Dockerfile \
  -t "$IMAGE_NAME" \
  .

echo "Built image: $IMAGE_NAME"
