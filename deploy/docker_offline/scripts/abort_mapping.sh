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
ABORT_WAIT_TIMEOUT_SEC="${ABORT_WAIT_TIMEOUT_SEC:-45}"

docker exec "$MAPPING_CONTAINER_NAME" bash -lc "echo abort > '$WORKSPACE/.start_mapping.command'"

echo "Abort command sent to $MAPPING_CONTAINER_NAME. Waiting for mapping container to stop..."

elapsed=0
while docker inspect "$MAPPING_CONTAINER_NAME" > /dev/null 2>&1; do
  running="$(docker inspect -f '{{.State.Running}}' "$MAPPING_CONTAINER_NAME" 2>/dev/null || echo false)"
  [ "$running" != "true" ] && break

  if [ "$elapsed" -ge "$ABORT_WAIT_TIMEOUT_SEC" ]; then
    echo "WARN: mapping container is still running after ${ABORT_WAIT_TIMEOUT_SEC}s."
    echo "Recent mapping logs:"
    docker logs --tail 80 "$MAPPING_CONTAINER_NAME" 2>/dev/null || true
    exit 1
  fi

  sleep 1
  elapsed=$((elapsed + 1))
done

echo "Mapping container stopped after abort."
