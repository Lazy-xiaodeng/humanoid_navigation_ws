#!/bin/bash
set -eo pipefail
set +u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${WORKSPACE:-$SCRIPT_DIR}"
TARGET_MAP_ID="${1:-${MAP_ID:-}}"
START_TIME="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$WORKSPACE/debug_logs"
LOG_FILE="$LOG_DIR/switch_navigation_map_${TARGET_MAP_ID:-unknown}_${START_TIME}.log"

mkdir -p "$LOG_DIR"
exec > >(tee -a "$LOG_FILE") 2>&1

echo "Switch navigation map at $(date '+%F %T %z')"
echo "Workspace: $WORKSPACE"
echo "Target map: $TARGET_MAP_ID"
echo "Log: $LOG_FILE"

if [ -z "$TARGET_MAP_ID" ]; then
  echo "ERROR: target map id is required"
  exit 2
fi

if [ ! -x "$WORKSPACE/stop_navigation_stack.sh" ]; then
  echo "ERROR: navigation stack stop script not executable: $WORKSPACE/stop_navigation_stack.sh"
  exit 3
fi
if [ ! -x "$WORKSPACE/start_navigation_stack.sh" ]; then
  echo "ERROR: navigation stack start script not executable: $WORKSPACE/start_navigation_stack.sh"
  exit 4
fi

echo "[0/2] Validating target map before stopping current navigation stack..."
WORKSPACE="$WORKSPACE" MAP_ID="$TARGET_MAP_ID" VALIDATE_ONLY=1 "$WORKSPACE/start_navigation_stack.sh"

# 切图只允许重启导航定位层：Nav2、地图服务器、RO/OP 定位、navigation_state_manager。
# websocket、点位管理、map_context_manager 必须保持运行，APP 才能持续收到 map_status/map_response。
echo "[1/2] Stopping map-bound navigation stack only..."
WORKSPACE="$WORKSPACE" "$WORKSPACE/stop_navigation_stack.sh" || true

echo "[2/2] Starting navigation stack with MAP_ID=$TARGET_MAP_ID..."
WORKSPACE="$WORKSPACE" MAP_ID="$TARGET_MAP_ID" "$WORKSPACE/start_navigation_stack.sh"

echo "Switch navigation map finished: $TARGET_MAP_ID"
