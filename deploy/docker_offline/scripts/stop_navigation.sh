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

CONTAINER_NAME="${CONTAINER_NAME:-humanoid-navigation}"
WORKSPACE="${WORKSPACE:-/home/ubuntu/humanoid_ws}"

if docker ps --format '{{.Names}}' | grep -Fxq "$CONTAINER_NAME"; then
  docker exec "$CONTAINER_NAME" bash -lc "cd '$WORKSPACE' && ./stop_navigation.sh || true" || true
  docker stop -t 20 "$CONTAINER_NAME" >/dev/null || true
fi
