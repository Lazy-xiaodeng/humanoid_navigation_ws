#!/usr/bin/env bash
set -eo pipefail
set +u

# C++ WebSocket/机器人网关灰度链路空载 smoke。
# 用途：
# 1. 使用当前 runtime launch 拉起 C++ APP 网关、C++ 数据整合、C++ 机器人本体网关。
# 2. APP WebSocket 服务关闭，避免抢占 8765。
# 3. 机器人 WebSocket 连接关闭，避免误连机器人。
# 4. 使用独立 ROS_DOMAIN_ID，避免影响当前正在运行的导航/控制系统。

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_DOMAIN_ID="${SMOKE_ROS_DOMAIN_ID:-229}"
STARTUP_WAIT_SEC="${STARTUP_WAIT_SEC:-3}"
RUN_SEC="${RUN_SEC:-8}"

export ROS_DOMAIN_ID

cd "${WORKSPACE}"
unset AMENT_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset PYTHONPATH
unset LD_LIBRARY_PATH
source /opt/ros/jazzy/setup.bash
source "${WORKSPACE}/install/setup.bash"

LOG_DIR="${WORKSPACE}/debug_logs"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/ws_cpp_gray_smoke_$(date +%Y%m%d_%H%M%S).log"

echo "== WS C++ gray smoke =="
echo "workspace=${WORKSPACE}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "override with SMOKE_ROS_DOMAIN_ID=<id> if this domain conflicts"
echo "log=${LOG_FILE}"

ros2 launch humanoid_app_gateway_runtime app_gateway_runtime.launch.py \
  websocket_server_enable:=false \
  data_integration_enable:=true \
  >"${LOG_FILE}" 2>&1 &

APP_LAUNCH_PID=$!

ros2 launch humanoid_robot_gateway_runtime robot_gateway_runtime.launch.py \
  robot_ws_enable:=false \
  walk_velocity_send_enable:=false \
  motion_execution_enable:=false \
  gesture_sync_enable:=false \
  >>"${LOG_FILE}" 2>&1 &

ROBOT_LAUNCH_PID=$!

cleanup() {
  for pid in "${APP_LAUNCH_PID}" "${ROBOT_LAUNCH_PID}"; do
    if kill -0 "${pid}" >/dev/null 2>&1; then
      kill -INT "${pid}" >/dev/null 2>&1 || true
    fi
  done
  sleep 1
  for pid in "${APP_LAUNCH_PID}" "${ROBOT_LAUNCH_PID}"; do
    if kill -0 "${pid}" >/dev/null 2>&1; then
      kill -TERM "${pid}" >/dev/null 2>&1 || true
    fi
  done
}
trap cleanup EXIT

sleep "${STARTUP_WAIT_SEC}"

for pid in "${APP_LAUNCH_PID}" "${ROBOT_LAUNCH_PID}"; do
  if ! kill -0 "${pid}" >/dev/null 2>&1; then
    echo "ERROR: launch exited before status check: pid=${pid}"
    sed -n '1,220p' "${LOG_FILE}" || true
    exit 1
  fi
done

echo
echo "== Runtime status while smoke launch is running =="
APP_PORT=8765 ROS_TIMEOUT=2 "${WORKSPACE}/tools/check_ws_runtime_status.sh" | sed -n '1,100p'

if ! pgrep -f 'humanoid_app_gateway_runtime.*/app_gateway_node|app_gateway_node --ros-args' >/dev/null; then
  echo "ERROR: app_gateway_node not detected"
  sed -n '1,220p' "${LOG_FILE}" || true
  exit 1
fi
if ! pgrep -f 'humanoid_app_gateway_runtime.*/data_integration_node|data_integration_node --ros-args.*humanoid_app_gateway_runtime' >/dev/null; then
  echo "ERROR: C++ data_integration_node not detected"
  sed -n '1,220p' "${LOG_FILE}" || true
  exit 1
fi
if ! pgrep -f 'humanoid_robot_gateway_runtime.*/robot_gateway_node|robot_gateway_node --ros-args' >/dev/null; then
  echo "ERROR: robot_gateway_node not detected"
  sed -n '1,220p' "${LOG_FILE}" || true
  exit 1
fi

if ! rg -q 'websocket_server_enable=false' "${LOG_FILE}"; then
  echo "ERROR: app_gateway_node did not report websocket_server_enable=false"
  sed -n '1,220p' "${LOG_FILE}" || true
  exit 1
fi
if ! rg -q 'data_integration_enable=true' "${LOG_FILE}"; then
  echo "ERROR: data_integration_node did not report data_integration_enable=true"
  sed -n '1,220p' "${LOG_FILE}" || true
  exit 1
fi
robot_ws_param_ok=false
for _ in $(seq 1 24); do
  if rg -q 'robot_gateway_node 已启动：robot_ws_enable=false' "${LOG_FILE}"; then
    robot_ws_param_ok=true
    break
  fi
  if timeout "${ROS_TIMEOUT}" ros2 param get /robot_gateway_node robot_ws_enable 2>/dev/null | rg -q 'False|false'; then
    robot_ws_param_ok=true
    break
  fi
  sleep 0.25
done
if [ "${robot_ws_param_ok}" != "true" ]; then
  echo "ERROR: robot_gateway_node parameter robot_ws_enable is not false"
  timeout "${ROS_TIMEOUT}" ros2 param get /robot_gateway_node robot_ws_enable 2>/dev/null || true
  sed -n '1,220p' "${LOG_FILE}" || true
  exit 1
fi
if rg -q 'bind: Address already in use|WebSocket server 异常' "${LOG_FILE}"; then
  echo "ERROR: APP WebSocket unexpectedly tried to bind or failed"
  sed -n '1,220p' "${LOG_FILE}" || true
  exit 1
fi

sleep "${RUN_SEC}"

echo
echo "PASS ws_cpp_gray_smoke"
echo "Log: ${LOG_FILE}"
