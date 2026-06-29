#!/usr/bin/env bash
set -eo pipefail
set +u

# 检查 WebSocket / 数据整合 / 机器人本体网关当前运行状态。
# 用途：
# 1. 灰度切换前确认当前是 Python 链路还是 C++ 链路。
# 2. 检查 APP WebSocket 端口是否被占用，避免 C++ APP 网关启动时 bind 失败。
# 3. 查看关键进程 CPU/RSS，辅助判断 C++ 化后的资源变化。
# 4. 查看关键 ROS 节点和 topic 是否存在，辅助定位启动链路是否完整。

APP_PORT="${APP_PORT:-8765}"
ROS_TIMEOUT="${ROS_TIMEOUT:-3}"

echo "== WS runtime mode summary =="
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>} (process checks are OS-wide; ROS node/topic checks follow this domain)"
python_app_count="$(pgrep -fc 'humanoid_websocket.*/websocket_server_node|websocket_server_node --ros-args' || true)"
python_data_count="$(pgrep -fc 'humanoid_websocket.*/data_integration_node_recoverable|data_integration_node_recoverable --ros-args' || true)"
python_robot_count="$(pgrep -fc 'humanoid_websocket.*/websocket_client_node|websocket_client_node --ros-args' || true)"
cpp_app_count="$(pgrep -fc 'humanoid_app_gateway_runtime.*/app_gateway_node|app_gateway_node --ros-args' || true)"
cpp_data_count="$(pgrep -fc 'humanoid_app_gateway_runtime.*/data_integration_node|data_integration_node --ros-args.*humanoid_app_gateway_runtime' || true)"
cpp_robot_count="$(pgrep -fc 'humanoid_robot_gateway_runtime.*/robot_gateway_node|robot_gateway_node --ros-args' || true)"

echo "Python APP websocket_server_node: ${python_app_count}"
echo "Python data_integration_node_recoverable: ${python_data_count}"
echo "Python robot websocket_client_node: ${python_robot_count}"
echo "C++ app_gateway_node: ${cpp_app_count}"
echo "C++ data_integration_node: ${cpp_data_count}"
echo "C++ robot_gateway_node: ${cpp_robot_count}"

if [ "${python_app_count}" -gt 0 ] && [ "${cpp_app_count}" -gt 0 ]; then
  echo "WARN: Python APP 网关和 C++ APP 网关进程同时存在；如果在同一 ROS_DOMAIN_ID/端口下，可能抢占端口或重复转发。"
fi
if [ "${python_data_count}" -gt 0 ] && [ "${cpp_data_count}" -gt 0 ]; then
  echo "WARN: Python 数据整合和 C++ 数据整合进程同时存在；如果在同一 ROS_DOMAIN_ID 下，可能重复发布 integration 数据。"
fi
if [ "${python_robot_count}" -gt 0 ] && [ "${cpp_robot_count}" -gt 0 ]; then
  echo "WARN: Python 机器人本体网关和 C++ 机器人本体网关进程同时存在；如果在同一 ROS_DOMAIN_ID 下，可能重复订阅 /cmd_vel 或重复发布 /robot_status_raw。"
fi

echo
echo "== Matching processes =="
pgrep -af 'ros2 launch humanoid_bringup robot_control_plane|websocket_server_node|data_integration_node_recoverable|websocket_client_node|app_gateway_node|robot_gateway_node|humanoid_app_gateway_runtime|humanoid_robot_gateway_runtime' || true

echo
echo "== Resource snapshot =="
ps -eo pid=,pcpu=,pmem=,rss=,comm=,args= --sort=-pcpu \
  | awk '
    /websocket_server_node|data_integration_node_recoverable|websocket_client_node|app_gateway_node|robot_gateway_node|humanoid_app_gateway_runtime|humanoid_robot_gateway_runtime/ && !/awk/ {
      printf "pid=%s cpu=%s%% mem=%s%% rss=%.1fMB comm=%s args=", $1, $2, $3, $4 / 1024.0, $5;
      for (i = 6; i <= NF; ++i) {
        printf "%s%s", $i, (i == NF ? "" : " ");
      }
      printf "\n";
    }'

echo
echo "== APP WebSocket port ${APP_PORT} =="
if command -v ss >/dev/null 2>&1; then
  ss -ltnp 2>/dev/null | awk -v port=":${APP_PORT}" '$4 ~ port {print}' || true
elif command -v lsof >/dev/null 2>&1; then
  lsof -nP -iTCP:"${APP_PORT}" -sTCP:LISTEN || true
else
  echo "ss/lsof not available; skip port check"
fi

echo
echo "== ROS node list =="
if command -v ros2 >/dev/null 2>&1; then
  timeout "${ROS_TIMEOUT}" ros2 node list 2>/dev/null \
    | rg 'websocket|data_integration|app_gateway|robot_gateway|dynamic_waypoints|map_context' || true
else
  echo "ros2 not available in PATH; source ROS environment first if node list is needed"
fi

echo
echo "== Key topic info =="
if command -v ros2 >/dev/null 2>&1; then
  for topic in \
    /robot_status_raw \
    /joy_raw \
    /robot/action_result \
    /websocket/data_requests \
    /websocket/data_subscriptions \
    /integration/data_responses \
    /integration/push_messages \
    /integration/subscription_responses
  do
    echo "-- ${topic}"
    timeout "${ROS_TIMEOUT}" ros2 topic info "${topic}" 2>/dev/null || true
  done
else
  echo "ros2 not available in PATH; skip topic info"
fi
