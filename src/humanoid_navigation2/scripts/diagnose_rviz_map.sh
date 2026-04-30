#!/bin/bash
# 诊断RViz地图显示问题

echo "=========================================="
echo "RViz地图显示诊断工具"
echo "=========================================="
echo ""

# 1. 检查map_server是否运行
echo "【1】检查map_server节点..."
if ros2 node list 2>/dev/null | grep -q "map_server"; then
    echo "✓ map_server节点正在运行"
    
    # 检查生命周期状态
    STATE=$(ros2 lifecycle get /map_server 2>/dev/null | grep -o 'unconfigured\|inactive\|active')
    if [ "$STATE" = "active" ]; then
        echo "✓ map_server状态: active (正常)"
    else
        echo "✗ map_server状态: $STATE (应该是active)"
        echo "  修复: ros2 lifecycle set /map_server activate"
    fi
else
    echo "✗ map_server节点未运行"
fi
echo ""

# 2. 检查/map话题
echo "【2】检查/map话题..."
if ros2 topic list 2>/dev/null | grep -q "/map"; then
    echo "✓ /map话题存在"
    
    # 查看话题信息
    echo "话题详情:"
    ros2 topic info /map 2>/dev/null | sed 's/^/  /'
    
    # 检查是否有发布者
    PUB_COUNT=$(ros2 topic info /map 2>/dev/null | grep "Publisher" | grep -o '[0-9]*')
    if [ -n "$PUB_COUNT" ] && [ "$PUB_COUNT" -gt 0 ]; then
        echo "✓ 发布者数量: $PUB_COUNT"
    else
        echo "✗ 无发布者"
    fi
else
    echo "✗ /map话题不存在"
fi
echo ""

# 3. 检查地图数据
echo "【3】检查地图数据..."
echo "尝试接收地图消息（等待3秒）..."
MAP_MSG=$(ros2 topic echo /map --once 2>/dev/null | head -5)
if [ -n "$MAP_MSG" ]; then
    echo "✓ 能够接收到地图消息"
    echo "消息头:"
    echo "$MAP_MSG" | sed 's/^/  /'
else
    echo "✗ 无法接收地图消息"
    echo "  可能原因: QoS设置不匹配或map_server未发布"
fi
echo ""

# 4. 检查TF树
echo "【4】检查TF树..."
if ros2 topic list 2>/dev/null | grep -q "/tf"; then
    echo "✓ /tf话题存在"
    
    # 检查map帧是否存在
    if ros2 run tf2_ros tf2_echo map odom 2>&1 | head -1 &
    then
        sleep 2
        kill $! 2>/dev/null
    fi
else
    echo "✗ /tf话题不存在"
fi
echo ""

# 5. 检查RViz配置
echo "【5】RViz配置建议..."
echo ""
echo "要在RViz中显示地图，请按以下步骤操作:"
echo ""
echo "方法1: 使用Map显示类型"
echo "  1. 点击左下角 'Add' 按钮"
echo "  2. 选择 'By topic' 标签"
echo "  3. 找到 '/map' 话题"
echo "  4. 选择 'Map' 显示类型"
echo "  5. 点击 'OK'"
echo ""
echo "方法2: 手动添加Map显示"
echo "  1. 点击左下角 'Add' 按钮"
echo "  2. 选择 'By display type' 标签"
echo "  3. 选择 'Map'"
echo "  4. 点击 'OK'"
echo "  5. 在左侧Displays面板中:"
echo "     - Topic: 选择 '/map'"
echo "     - Transport Hints: 选择 'raw' 或 'compressed'"
echo "     - Update Topic: 勾选"
echo ""
echo "⚠ 重要: QoS设置"
echo "  - Durability Policy: 设置为 'Transient Local'"
echo "  - Reliability Policy: 设置为 'Reliable'"
echo "  - History Policy: 设置为 'Keep Last'"
echo "  - Depth: 设置为 1"
echo ""

# 6. 常见问题
echo "=========================================="
echo "常见问题排查"
echo "=========================================="
echo ""
echo "Q1: Map显示添加后看不到地图？"
echo "  A: 检查以下项:"
echo "     - Topic是否选择 '/map'"
echo "     - Alpha值是否太低（建议1.0）"
echo "     - Color Scheme是否合适（建议'map'或'costmap'）"
echo "     - 使用 'Reset' 按钮重置显示"
echo ""
echo "Q2: 显示'No map received'？"
echo "  A: 可能是QoS不匹配"
echo "     解决方法:"
echo "     1. 在RViz中右键Map显示"
echo "     2. 选择 'Configure'"
echo "     3. 修改QoS设置为上述推荐值"
echo ""
echo "Q3: 地图显示为空白或全黑？"
echo "  A: 检查地图数据:"
echo "     ros2 topic echo /map --once | grep -A5 'data'"
echo "     如果data为空，map_server可能未正确加载地图"
echo ""
echo "Q4: 地图闪烁或不显示？"
echo "  A: 检查TF树是否完整:"
echo "     ros2 run tf2_tools view_frames"
echo "     应该有 map → odom → ... → base_footprint"
echo ""

echo "=========================================="
echo "快速修复命令"
echo "=========================================="
echo ""
echo "# 如果map_server未激活:"
echo "ros2 lifecycle set /map_server configure"
echo "ros2 lifecycle set /map_server activate"
echo ""
echo "# 查看map_server发布的消息:"
echo "ros2 topic echo /map --once | head -20"
echo ""
echo "# 查看map_server日志:"
echo "ros2 node info /map_server"
echo ""
