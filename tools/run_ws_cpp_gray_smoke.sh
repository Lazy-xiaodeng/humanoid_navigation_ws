#!/usr/bin/env bash
set -eo pipefail
set +u

# C++ WebSocket 灰度链路空载 smoke。
# 用途：
# 1. 使用统一 websocket_server.launch.py 拉起 C++ APP 网关、C++ 数据整合、C++ 机器人本体网关。
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

ros2 launch humanoid_websocket websocket_server.launch.py \
  use_cpp_app_gateway:=true \
  cpp_app_websocket_server_enable:=false \
  cpp_data_integration_enable:=true \
  use_cpp_robot_gateway:=true \
  cpp_robot_ws_enable:=false \
  cpp_robot_walk_velocity_send_enable:=false \
  cpp_robot_motion_execution_enable:=false \
  >"${LOG_FILE}" 2>&1 &

LAUNCH_PID=$!

cleanup() {
  if kill -0 "${LAUNCH_PID}" >/dev/null 2>&1; then
    kill -INT "${LAUNCH_PID}" >/dev/null 2>&1 || true
    sleep 1
    kill -TERM "${LAUNCH_PID}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

sleep "${STARTUP_WAIT_SEC}"

if ! kill -0 "${LAUNCH_PID}" >/dev/null 2>&1; then
  echo "ERROR: launch exited before status check"
  sed -n '1,160p' "${LOG_FILE}" || true
  exit 1
fi

echo
echo "== Runtime status while smoke launch is running =="
APP_PORT=8765 ROS_TIMEOUT=2 "${WORKSPACE}/tools/check_ws_runtime_status.sh" | sed -n '1,80p'

if ! pgrep -f 'humanoid_app_gateway_runtime.*/app_gateway_node|app_gateway_node --ros-args' >/dev/null; then
  echo "ERROR: app_gateway_node not detected"
  sed -n '1,180p' "${LOG_FILE}" || true
  exit 1
fi
if ! pgrep -f 'humanoid_app_gateway_runtime.*/data_integration_node|data_integration_node --ros-args.*humanoid_app_gateway_runtime' >/dev/null; then
  echo "ERROR: C++ data_integration_node not detected"
  sed -n '1,180p' "${LOG_FILE}" || true
  exit 1
fi
if ! pgrep -f 'humanoid_robot_gateway_runtime.*/robot_gateway_node|robot_gateway_node --ros-args' >/dev/null; then
  echo "ERROR: robot_gateway_node not detected"
  sed -n '1,180p' "${LOG_FILE}" || true
  exit 1
fi

if ! rg -q 'websocket_server_enable=false' "${LOG_FILE}"; then
  echo "ERROR: app_gateway_node did not report websocket_server_enable=false"
  sed -n '1,180p' "${LOG_FILE}" || true
  exit 1
fi
if ! rg -q 'data_integration_enable=true' "${LOG_FILE}"; then
  echo "ERROR: data_integration_node did not report data_integration_enable=true"
  sed -n '1,180p' "${LOG_FILE}" || true
  exit 1
fi
if ! rg -q 'robot_ws_enable=false' "${LOG_FILE}"; then
  echo "ERROR: robot_gateway_node did not report robot_ws_enable=false"
  sed -n '1,180p' "${LOG_FILE}" || true
  exit 1
fi
if rg -q 'bind: Address already in use|WebSocket server 异常' "${LOG_FILE}"; then
  echo "ERROR: APP WebSocket unexpectedly tried to bind or failed"
  sed -n '1,180p' "${LOG_FILE}" || true
  exit 1
fi

sleep "${RUN_SEC}"

echo
echo "PASS ws_cpp_gray_smoke"
echo "Log: ${LOG_FILE}"
