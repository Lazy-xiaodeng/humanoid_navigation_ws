#!/bin/bash

# 1. 设置 ROS 2 环境变量 (已更新为 jazzy)
source /opt/ros/jazzy/setup.bash

# 2. 进入工作空间并加载环境
cd /home/ubuntu/humanoid_ws
source install/setup.bash

# 3. 启动 WebSocket 服务
ros2 launch humanoid_websocket websocket_server.launch.py
