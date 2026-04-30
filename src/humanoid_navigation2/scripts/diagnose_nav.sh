#!/bin/bash
# 导航系统诊断脚本
# 用法: bash diagnose_nav.sh

echo "=========================================="
echo "导航系统诊断工具"
echo "=========================================="
echo ""

# 1. 检查地图文件
echo "【1】检查地图文件..."
MAP_YAML="/home/ubuntu/humanoid_ws/src/humanoid_navigation2/maps/hall.yaml"
MAP_PGM="/home/ubuntu/humanoid_ws/src/humanoid_navigation2/maps/hall.pgm"
PCD_MAP="/home/ubuntu/humanoid_ws/src/humanoid_navigation2/pcd/hall.pcd"

if [ -f "$MAP_YAML" ]; then
    echo "✓ 2D地图 YAML文件存在: $MAP_YAML"
    echo "  内容:"
    cat "$MAP_YAML" | sed 's/^/    /'
else
    echo "✗ 2D地图 YAML文件不存在: $MAP_YAML"
fi

if [ -f "$MAP_PGM" ]; then
    echo "✓ 2D地图 PGM图像存在: $MAP_PGM"
else
    echo "✗ 2D地图 PGM图像不存在: $MAP_PGM"
fi

if [ -f "$PCD_MAP" ]; then
    echo "✓ PCD点云地图存在: $PCD_MAP"
    PCD_SIZE=$(du -h "$PCD_MAP" | cut -f1)
    echo "  文件大小: $PCD_SIZE"
else
    echo "✗ PCD点云地图不存在: $PCD_MAP"
fi
echo ""

# 2. 检查ROS2节点
echo "【2】检查ROS2节点运行状态..."
NODES=("map_server" "lidar_localization" "fast_lio_node" "rslidar_sdk_node")
for node in "${NODES[@]}"; do
    if ros2 node list 2>/dev/null | grep -q "$node"; then
        echo "✓ 节点运行中: $node"
    else
        echo "✗ 节点未运行: $node"
    fi
done
echo ""

# 3. 检查TF变换
echo "【3】检查TF变换..."
TF_TREE=("map" "odom" "base_footprint" "camera_init")
for tf in "${TF_TREE[@]}"; do
    if ros2 topic echo --once /tf 2>/dev/null | grep -q "$tf"; then
        echo "✓ TF帧存在: $tf"
    else
        echo "? TF帧状态未知: $tf (需要运行中检查)"
    fi
done

# 检查map->odom变换
echo ""
echo "检查 map → odom 变换:"
ros2 run tf2_ros tf2_echo map odom 2>&1 | head -3 &
TF_PID=$!
sleep 2
kill $TF_PID 2>/dev/null
wait $TF_PID 2>/dev/null
echo ""

# 4. 检查话题
echo "【4】检查关键话题..."
TOPICS=("/map" "/odom" "/tf" "/fast_lio/cloud_registered")
for topic in "${TOPICS[@]}"; do
    COUNT=$(ros2 topic info "$topic" 2>/dev/null | grep "Publisher" | grep -o '[0-9]*')
    if [ -n "$COUNT" ] && [ "$COUNT" -gt 0 ]; then
        echo "✓ 话题活跃: $topic (发布者数: $COUNT)"
    else
        echo "✗ 话题无发布者: $topic"
    fi
done
echo ""

# 5. 检查生命周期节点状态
echo "【5】检查生命周期节点状态..."
LIFECYCLE_NODES=("map_server" "lidar_localization")
for node in "${LIFECYCLE_NODES[@]}"; do
    STATE=$(ros2 lifecycle get "$node" 2>/dev/null | grep -o 'unconfigured\|inactive\|active')
    if [ -n "$STATE" ]; then
        echo "✓ $node 状态: $STATE"
    else
        echo "? $node 状态未知 (节点可能未启动)"
    fi
done
echo ""

# 6. 检查NDT日志
echo "【6】最近NDT定位日志(最后20行):"
if ros2 node list 2>/dev/null | grep -q "lidar_localization"; then
    echo "NDT节点正在运行,请查看终端输出或使用以下命令查看日志:"
    echo "  ros2 topic echo /lidar_localization/status"
else
    echo "NDT节点未运行"
fi
echo ""

# 7. 建议操作
echo "=========================================="
echo "诊断完成 - 建议操作"
echo "=========================================="
echo ""
echo "如果map_server未启动或无/map话题:"
echo "  1. 手动测试map_server:"
echo "     ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$MAP_YAML"
echo "  2. 检查生命周期管理:"
echo "     ros2 lifecycle set /map_server configure"
echo "     ros2 lifecycle set /map_server activate"
echo ""
echo "如果NDT未发布map→odom变换:"
echo "  1. 检查PCD地图路径是否正确"
echo "  2. 在RViz中使用2D Pose Estimate设置初始位姿"
echo "  3. 查看NDT节点终端输出是否有匹配成功信息"
echo ""
echo "完整TF树查看:"
echo "  ros2 run tf2_tools view_frames"
echo ""
