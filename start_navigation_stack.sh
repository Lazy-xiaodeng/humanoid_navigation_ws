#!/bin/bash
set -eo pipefail
set +u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${WORKSPACE:-$SCRIPT_DIR}"
REQUESTED_MAP_ID="${MAP_ID:-${1:-}}"
START_TIME="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$WORKSPACE/debug_logs"
LOG_FILE="$LOG_DIR/start_navigation_stack_${REQUESTED_MAP_ID:-current}_${START_TIME}.log"
ROS_LOG_DIR="${ROS_LOG_DIR:-$LOG_DIR/ros_nav_stack_${START_TIME}}"
PID_FILE="$WORKSPACE/.navigation_stack.pid"
ACTIVE_MAP_ENV_FILE="$WORKSPACE/.active_navigation_map.env"

mkdir -p "$LOG_DIR" "$ROS_LOG_DIR"
export ROS_LOG_DIR
exec > >(tee -a "$LOG_FILE") 2>&1

source_ros_env() {
  cd "$WORKSPACE"
  unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
  source /opt/ros/jazzy/setup.bash
  source "$WORKSPACE/install/local_setup.bash"
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
  export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.config/fastdds_shm.xml}"
  export RMW_FASTRTPS_USE_QOS_FROM_XML="${RMW_FASTRTPS_USE_QOS_FROM_XML:-1}"
}

resolve_map_env() {
  local workspace="$1"
  local requested_map_id="$2"
  local env_file="$3"

  python3 - "$workspace" "$requested_map_id" "$env_file" <<'PYMAP'
import json
import os
import shlex
import sys
from pathlib import Path

workspace = Path(sys.argv[1])
requested_map_id = str(sys.argv[2]).strip()
env_file = Path(sys.argv[3])
registry_path = workspace / "data/maps/map_registry.json"
base_ro_config = workspace / "src/humanoid_robosense_localization_runtime/config/robosense_lidar_localization.yaml"

if not registry_path.exists():
    raise SystemExit(f"ERROR: map registry not found: {registry_path}")

registry = json.loads(registry_path.read_text(encoding="utf-8"))
if not requested_map_id:
    requested_map_id = str(registry.get("current_map_id") or registry.get("default_map_id") or "hall").strip()

maps = registry.get("maps")
if not isinstance(maps, list):
    raise SystemExit(f"ERROR: invalid map registry, maps must be list: {registry_path}")

target = next((item for item in maps if str(item.get("map_id", "")).strip() == requested_map_id), None)
if not target:
    raise SystemExit(f"ERROR: map_id not registered: {requested_map_id}")
if not target.get("enabled", True):
    raise SystemExit(f"ERROR: map_id is disabled: {requested_map_id}")

map_yaml = Path(str(target.get("map_yaml_file", "")))
prior_map = Path(str(target.get("open3d_prior_map_file", "")))
if not map_yaml.exists():
    raise SystemExit(f"ERROR: 2D map yaml missing for {requested_map_id}: {map_yaml}")
if not prior_map.exists():
    raise SystemExit(f"ERROR: 3D prior map missing for {requested_map_id}: {prior_map}")
if not base_ro_config.exists():
    raise SystemExit(f"ERROR: RoboSense base config missing: {base_ro_config}")

runtime_dir = workspace / "data/runtime_maps" / requested_map_id
runtime_dir.mkdir(parents=True, exist_ok=True)
runtime_ro_config = runtime_dir / "robosense_lidar_localization.yaml"

# RoboSense 定位节点启动时只读取 YAML，所以这里为目标地图生成 runtime 配置。
lines = base_ro_config.read_text(encoding="utf-8").splitlines()
output = []
replaced = False
for line in lines:
    stripped = line.strip()
    if stripped.startswith("map_path:"):
        indent = line[: len(line) - len(line.lstrip())]
        output.append(f"{indent}map_path: {prior_map}")
        replaced = True
    else:
        output.append(line)
if not replaced:
    output.append(f"map_path: {prior_map}")
runtime_ro_config.write_text("\n".join(output) + "\n", encoding="utf-8")

if os.environ.get("VALIDATE_ONLY", "0") != "1":
    registry["current_map_id"] = requested_map_id
    registry_path.write_text(json.dumps(registry, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

values = {
    "ACTIVE_MAP_ID": requested_map_id,
    "ACTIVE_MAP_YAML_FILE": str(map_yaml),
    "ACTIVE_PRIOR_MAP_PATH": str(prior_map),
    "ACTIVE_ROBOSENSE_CONFIG_FILE": str(runtime_ro_config),
}
env_file.write_text("\n".join(f"{key}={shlex.quote(value)}" for key, value in values.items()) + "\n", encoding="utf-8")
print(f"Selected map: {requested_map_id}")
print(f"2D map yaml: {map_yaml}")
print(f"3D prior map: {prior_map}")
print(f"RoboSense runtime config: {runtime_ro_config}")
PYMAP
}


source_ros_env

# 先解析并校验目标地图，再停止旧导航层。这样目标地图文件缺失时不会把当前可用导航栈先停掉。
resolve_map_env "$WORKSPACE" "$REQUESTED_MAP_ID" "$ACTIVE_MAP_ENV_FILE"
if [ "${VALIDATE_ONLY:-0}" = "1" ]; then
  echo "Navigation stack map validation passed."
  exit 0
fi

if [ -x "$WORKSPACE/stop_navigation_stack.sh" ]; then
  "$WORKSPACE/stop_navigation_stack.sh" || true
fi

source "$ACTIVE_MAP_ENV_FILE"

setsid nohup ros2 launch humanoid_bringup robot_navigation_stack.launch.py \
  use_sim_time:=false \
  rviz:=false \
  map_id:="$ACTIVE_MAP_ID" \
  map_yaml_file:="$ACTIVE_MAP_YAML_FILE" \
  prior_map_path:="$ACTIVE_PRIOR_MAP_PATH" \
  robosense_config_file:="$ACTIVE_ROBOSENSE_CONFIG_FILE" \
  >> "$LOG_FILE" 2>&1 < /dev/null &

STACK_PID=$!
echo "$STACK_PID" > "$PID_FILE"

sleep 2
if kill -0 "$STACK_PID" >/dev/null 2>&1; then
  echo "NAVIGATION_STACK_STARTED_SUCCESSFULLY_PID_$STACK_PID"
  echo "Debug log: $LOG_FILE"
  exit 0
fi

echo "ERROR: Navigation stack crashed immediately."
rm -f "$PID_FILE"
exit 1
