#!/usr/bin/env bash
set -eo pipefail
set +u

# C++ 运行链路回归验证入口。
# 用途：
# 1. 构建当前工作区，并临时打开 runtime probe，确认纯逻辑模块协议/字段不退化。
# 2. 验证主要 launch 文件仍能被 ROS2 正确解析，避免启动参数或包名改坏。
# 3. 运行 APP WebSocket smoke，检查 APP request -> ROS topic -> integration response -> APP 的闭环。
# 4. 运行 C++ APP/Robot 网关空载 smoke，在不连接机器人、不抢占 8765 的情况下检查节点能启动。
# 5. 扫描关键可执行文件缺失 so，减少换机器部署时才暴露动态库问题。

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_TYPE="${BUILD_TYPE:-Release}"
SMOKE_ROS_DOMAIN_ID="${SMOKE_ROS_DOMAIN_ID:-231}"
export SMOKE_ROS_DOMAIN_ID

cd "${WORKSPACE}"

echo "== C++ runtime regression =="
echo "workspace=${WORKSPACE}"
echo "build_type=${BUILD_TYPE}"
echo "SMOKE_ROS_DOMAIN_ID=${SMOKE_ROS_DOMAIN_ID}"

# 避免调用者终端里已经 source 过其他工作区，导致 ros2 pkg/launch 找到旧 Python 包。
# 这里重新建立“ROS 官方 underlay + 当前工作区 overlay”的干净环境。
unset AMENT_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset PYTHONPATH
unset LD_LIBRARY_PATH

source /opt/ros/jazzy/setup.bash

echo
echo "== Build runtime packages with probes =="
colcon build --symlink-install \
  --packages-select \
    humanoid_app_gateway_runtime \
    humanoid_robot_gateway_runtime \
    humanoid_control_runtime \
    humanoid_route_runtime \
    humanoid_expression_runtime \
    humanoid_bringup \
  --cmake-args \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DBUILD_RUNTIME_PROBES=ON

source "${WORKSPACE}/install/setup.bash"

echo
echo "== Package presence check =="
for pkg in \
  humanoid_app_gateway_runtime \
  humanoid_robot_gateway_runtime \
  humanoid_control_runtime \
  humanoid_route_runtime \
  humanoid_expression_runtime
do
  ros2 pkg prefix "${pkg}" >/dev/null
  echo "PASS package ${pkg}"
done

echo
echo "== Old Python runtime package absence check =="
for old_pkg in humanoid_websocket humanoid_locomotion humanoid_navigation; do
  if colcon list --names-only | grep -qx "${old_pkg}"; then
    echo "ERROR: old Python runtime package still exists in src: ${old_pkg}" >&2
    exit 1
  fi
  if find install build -maxdepth 2 -type d -name "${old_pkg}" | grep -q .; then
    echo "ERROR: old Python runtime package still has local build/install remnants: ${old_pkg}" >&2
    exit 1
  fi
  echo "PASS old package absent from current workspace: ${old_pkg}"
done

echo
echo "== Run C++ probes =="
run_probe() {
  local label="$1"
  shift
  echo "-- ${label}"
  "$@" >/tmp/cpp_runtime_probe.out
  tail -n 1 /tmp/cpp_runtime_probe.out || true
}

run_probe_with_stdin() {
  local label="$1"
  local stdin_payload="$2"
  shift 2
  echo "-- ${label}"
  printf '%s' "${stdin_payload}" | "$@" >/tmp/cpp_runtime_probe.out
  tail -n 1 /tmp/cpp_runtime_probe.out || true
}

raw_robot_status_sample='{"accid":"HU_D04_01_289","sn":"SN_TEST","values":{"battery":89,"robot_status":"Walk","bat_vol":51.2,"bat_cur":-1.2,"bat_temp0":32.1,"control_ready_for_navigation":true},"health":{"system_info":"ok"},"latency":22.0,"timestamp":123.0}'
app_request_sample='{"protocol_version":"2.0","message_id":"req-smoke","message_type":"request","data_type":"system_status","source":"app","destination":"ros","data":{},"metadata":{}}'
app_subscription_sample='{"protocol_version":"2.0","message_id":"sub-smoke","message_type":"subscription","data_type":"subscription_manage","source":"app","destination":"ros","data":{"action":"subscribe","data_types":["system_status","navigation_status"],"interval":1.0},"metadata":{}}'

run_probe_with_stdin \
  "robot_status_adapter_probe" \
  "${raw_robot_status_sample}" \
  "${WORKSPACE}/install/humanoid_app_gateway_runtime/lib/humanoid_app_gateway_runtime/robot_status_adapter_probe"
run_probe_with_stdin \
  "app_protocol_probe request" \
  "${app_request_sample}" \
  "${WORKSPACE}/install/humanoid_app_gateway_runtime/lib/humanoid_app_gateway_runtime/app_protocol_probe" request
run_probe_with_stdin \
  "app_protocol_probe subscription" \
  "${app_subscription_sample}" \
  "${WORKSPACE}/install/humanoid_app_gateway_runtime/lib/humanoid_app_gateway_runtime/app_protocol_probe" subscription
run_probe_with_stdin \
  "protocol_builder_probe data_response" \
  '{"battery_level":89}' \
  "${WORKSPACE}/install/humanoid_app_gateway_runtime/lib/humanoid_app_gateway_runtime/protocol_builder_probe" \
  data_response client_a req-smoke system_status 123.0 122.5 msg-response
run_probe_with_stdin \
  "protocol_builder_probe push" \
  '{"event_type":"navigation_started"}' \
  "${WORKSPACE}/install/humanoid_app_gateway_runtime/lib/humanoid_app_gateway_runtime/protocol_builder_probe" \
  push navigation_status_update client_a 123.0 122.5 1 msg-push

fixed_probe_bins=(
  install/humanoid_app_gateway_runtime/lib/humanoid_app_gateway_runtime/data_store_probe
  install/humanoid_app_gateway_runtime/lib/humanoid_app_gateway_runtime/subscription_manager_probe
  install/humanoid_app_gateway_runtime/lib/humanoid_app_gateway_runtime/path_metrics_probe
  install/humanoid_app_gateway_runtime/lib/humanoid_app_gateway_runtime/pose_speed_adapter_probe
  install/humanoid_app_gateway_runtime/lib/humanoid_app_gateway_runtime/navigation_status_adapter_probe
  install/humanoid_app_gateway_runtime/lib/humanoid_app_gateway_runtime/map_waypoint_adapter_probe
  install/humanoid_app_gateway_runtime/lib/humanoid_app_gateway_runtime/client_registry_probe
  install/humanoid_app_gateway_runtime/lib/humanoid_app_gateway_runtime/integration_forwarder_probe
  install/humanoid_app_gateway_runtime/lib/humanoid_app_gateway_runtime/business_command_router_probe
  install/humanoid_robot_gateway_runtime/lib/humanoid_robot_gateway_runtime/robot_protocol_probe
  install/humanoid_robot_gateway_runtime/lib/humanoid_robot_gateway_runtime/response_waiter_probe
  install/humanoid_robot_gateway_runtime/lib/humanoid_robot_gateway_runtime/robot_status_parser_probe
  install/humanoid_robot_gateway_runtime/lib/humanoid_robot_gateway_runtime/action_result_builder_probe
  install/humanoid_robot_gateway_runtime/lib/humanoid_robot_gateway_runtime/walk_velocity_controller_probe
  install/humanoid_robot_gateway_runtime/lib/humanoid_robot_gateway_runtime/gesture_sync_probe
  install/humanoid_robot_gateway_runtime/lib/humanoid_robot_gateway_runtime/motion_controller_probe
  install/humanoid_robot_gateway_runtime/lib/humanoid_robot_gateway_runtime/robot_ws_client_probe
)
for probe in "${fixed_probe_bins[@]}"; do
  run_probe "${probe}" "${WORKSPACE}/${probe}"
done

echo
echo "== Launch parse check =="
launch_checks=(
  "humanoid_bringup robot_control_plane.launch.py app_websocket_server_enable:=false robot_ws_enable:=false robot_walk_velocity_send_enable:=false robot_motion_execution_enable:=false robot_gesture_sync_enable:=false"
  "humanoid_bringup robot_real.launch.py rviz:=false app_websocket_server_enable:=false robot_ws_enable:=false robot_walk_velocity_send_enable:=false robot_motion_execution_enable:=false robot_gesture_sync_enable:=false"
  "humanoid_app_gateway_runtime app_gateway_runtime.launch.py websocket_server_enable:=false data_integration_enable:=true"
  "humanoid_robot_gateway_runtime robot_gateway_runtime.launch.py robot_ws_enable:=false walk_velocity_send_enable:=false motion_execution_enable:=false gesture_sync_enable:=false"
  "humanoid_control_runtime control_runtime.launch.py"
  "humanoid_route_runtime route_runtime.launch.py nav2_execution_enable:=false"
  "humanoid_expression_runtime expression_runtime.launch.py"
)
for item in "${launch_checks[@]}"; do
  echo "-- ros2 launch --show-args ${item}"
  timeout 12 ros2 launch --show-args ${item} >/dev/null
done

echo
echo "== Missing shared library check =="
missing=0
while IFS= read -r exe; do
  if ldd "${exe}" 2>/dev/null | rg -q 'not found'; then
    echo "ERROR missing dependency in ${exe}" >&2
    ldd "${exe}" >&2 || true
    missing=1
  fi
done < <(
  find install -type f -perm -111 \
    \( -path '*/humanoid_app_gateway_runtime/lib/*' \
       -o -path '*/humanoid_robot_gateway_runtime/lib/*' \
       -o -path '*/humanoid_control_runtime/lib/*' \
       -o -path '*/humanoid_route_runtime/lib/*' \
       -o -path '*/humanoid_expression_runtime/lib/*' \) \
    | sort
)
if [ "${missing}" -ne 0 ]; then
  exit 1
fi
echo "PASS no missing shared libraries in runtime executables"

echo
echo "== APP WebSocket request/response smoke =="
"${WORKSPACE}/tools/run_ws_cpp_app_gateway_smoke.py"

echo
echo "== C++ app/robot gateway gray smoke =="
"${WORKSPACE}/tools/run_ws_cpp_gray_smoke.sh"

echo
echo "PASS cpp_runtime_regression"
