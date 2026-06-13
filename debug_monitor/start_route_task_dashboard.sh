#!/usr/bin/env bash
set -eo pipefail

# 路线任务调试台启动脚本。
# 只启动本地 HTTP 调试页面服务，不会启动或停止任何导航节点。
WORKSPACE="/home/ubuntu/software/Todesk/Files/humanoid_ws"
cd "${WORKSPACE}"

# ROS 的 setup.bash 内部会读取 AMENT_TRACE_SETUP_FILES 等可选环境变量。
# 如果脚本开启 set -u，未定义变量会导致 source 阶段直接报“未绑定的变量”。
# 因此这里不启用 nounset，保证不同终端环境下都能稳定启动调试台。
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
  source "/opt/ros/jazzy/setup.bash"
fi

if [ -f "install/setup.bash" ]; then
  source "install/setup.bash"
fi

python3 debug_monitor/route_task_dashboard_server.py "$@"
