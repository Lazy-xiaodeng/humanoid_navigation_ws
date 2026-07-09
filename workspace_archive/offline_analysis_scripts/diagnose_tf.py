#!/usr/bin/env python3
"""
诊断TF变换和坐标系问题的脚本
"""

import subprocess
import sys
import time

def run_command(cmd, description):
    """运行命令并打印结果"""
    print(f"\n{'='*60}")
    print(f"检查: {description}")
    print(f"命令: {cmd}")
    print(f"{'='*60}")
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    print(result.stdout)
    if result.stderr:
        print("STDERR:", result.stderr)
    return result.stdout

def main():
    print("\n🔍 TF变换和坐标系诊断工具")
    print("="*60)
    
    # 1. 检查TF树
    run_command(
        "ros2 run tf2_tools view_frames 2>/dev/null && ls -la frames*.pdf 2>/dev/null | tail -1",
        "生成TF树PDF"
    )
    
    # 2. 检查所有活跃的TF变换
    run_command(
        "ros2 topic echo /tf --once 2>/dev/null | head -50",
        "查看当前TF变换"
    )
    
    # 3. 检查NDT定位节点发布的TF
    run_command(
        "ros2 run tf2_tools tf2_monitor map odom 2>&1 | head -20",
        "监控 map -> odom 的TF变换"
    )
    
    # 4. 检查完整的TF链
    run_command(
        "ros2 run tf2_tools tf2_tree 2>&1 | grep -A 50 'map'",
        "查看从map开始的TF树"
    )
    
    # 5. 检查点云话题的frame_id
    run_command(
        "ros2 topic echo /fast_lio/cloud_registered --once 2>/dev/null | head -5",
        "检查注册点云的frame_id"
    )
    
    # 6. 检查NDT定位状态
    run_command(
        "ros2 topic echo /lidar_localization/pose --once 2>/dev/null | head -20",
        "查看NDT定位输出的位姿"
    )
    
    # 7. 查看机器人位姿
    run_command(
        "ros2 topic echo /odometry/filtered --once 2>/dev/null | head -20 || ros2 topic echo /odom --once 2>/dev/null | head -20",
        "查看机器人里程计位姿"
    )
    
    print("\n" + "="*60)
    print("📋 诊断建议:")
    print("="*60)
    print("""
1. 查看生成的 frames.pdf 文件，检查TF树是否完整
   应该是: map -> odom -> camera_init -> body -> base_footprint

2. 如果TF链断裂，说明某个static_transform_publisher没有正确启动

3. 检查NDT输出的位姿z坐标:
   - 如果z坐标是负数且很大，说明坐标系方向错误
   - 正常应该是接近0的小数（地面高度）

4. 在RViz中:
   - 设置 Fixed Frame = map
   - 添加 TF 显示插件
   - 观察各个坐标系的方向和位置

5. 运行以下命令手动检查TF:
   ros2 run tf2_ros tf2_echo map base_footprint
   
   如果输出的位置不合理（如z=-2.0），说明TF变换有问题
""")

if __name__ == "__main__":
    main()
