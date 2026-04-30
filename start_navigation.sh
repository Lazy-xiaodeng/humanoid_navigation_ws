#!/bin/bash
# 告诉脚本遇到错误不要立刻停下，或者打印命令 (可选)
set -x

# 设置环境变量，确保能找到 ROS2
source /opt/ros/jazzy/setup.bash # 假设你使用的是 humble，如果是 foxy 请修改

# source 你自己工作空间的环境
cd /home/ubuntu/humanoid_ws
source install/setup.bash

# 打印一下时间，方便之后看日志
echo "Starting Humanoid Navigation at $(date)"

# 运行启动文件，把原本打在屏幕上的日志输出到 debug_output.txt，并追加记录(>>)
ros2 launch humanoid_bringup robot_real.launch.py >> /home/ubuntu/humanoid_ws/debug_output.txt 2>&1
