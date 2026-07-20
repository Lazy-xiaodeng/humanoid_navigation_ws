#!/bin/bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
ENV_FILE="$PACKAGE_ROOT/humanoid.env"

if [ -f "$ENV_FILE" ]; then
  set -a
  source "$ENV_FILE"
  set +a
fi

IMAGE_NAME="${IMAGE_NAME:-humanoid_nav:1.0.0}"
IMAGE_TAR="${IMAGE_TAR:-images/humanoid_nav_1.0.0.tar}"
IMAGE_TAR_PATH="$PACKAGE_ROOT/$IMAGE_TAR"

mkdir -p "$(dirname "$IMAGE_TAR_PATH")"
docker save -o "$IMAGE_TAR_PATH" "$IMAGE_NAME"
sha256sum "$IMAGE_TAR_PATH" > "$IMAGE_TAR_PATH.sha256"

echo "Saved image: $IMAGE_TAR_PATH"
echo "Checksum: $IMAGE_TAR_PATH.sha256"
