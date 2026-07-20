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

MAPPING_CONTAINER_NAME="${MAPPING_CONTAINER_NAME:-humanoid-mapping}"
WORKSPACE="${WORKSPACE:-/home/ubuntu/humanoid_ws}"

docker exec "$MAPPING_CONTAINER_NAME" bash -lc "echo finish > '$WORKSPACE/.start_mapping.command'"
