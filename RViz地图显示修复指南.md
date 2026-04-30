# RViz地图显示问题修复指南

## 问题现象

在RViz中看不到2D静态地图（/map话题），显示为空白或"No map received"。

## 问题根源

**QoS Durability Policy不匹配**：

- **map_server**使用 `Transient Local` 持久性发布地图
- **RViz配置**使用 `Volatile` 持久性接收地图
- **结果**：RViz启动时错过了已经发布的地图消息，无法显示

### 技术解释

| QoS策略 | Transient Local | Volatile |
|---------|----------------|----------|
| **行为** | 保留最后一条消息，新订阅者立即收到 | 不保留消息，只发送给当前活跃的订阅者 |
| **适用场景** | 地图、配置等静态或低频数据 | 传感器数据、实时流等高频数据 |
| **RViz订阅** | ✓ 能收到历史消息 | ✗ 只能收到新消息 |

**时间线**：
```
T0: map_server启动，发布地图消息（只发布一次）
T1: RViz启动，订阅/map话题
T2: RViz等待新地图消息...（但map_server不会再发布）
T3: 显示"No map received"
```

## 已完成的修复

✅ 修改了 `/home/ubuntu/humanoid_ws/src/humanoid_navigation2/rviz/navigation.rviz`

### 修改内容

**修改前（错误）**：
```yaml
- Class: rviz_default_plugins/Map
  Name: SLAM_Map
  Topic:
    Depth: 5
    Durability Policy: Volatile    # ← 错误！
    Reliability Policy: Reliable
    Value: /map
```

**修改后（正确）**：
```yaml
- Class: rviz_default_plugins/Map
  Name: SLAM_Map
  Topic:
    Depth: 1                        # ← 改为1（只需要最后一条）
    Durability Policy: Transient Local  # ← 关键修改！
    Reliability Policy: Reliable
    Value: /map
```

修改了两个Map显示：
1. **SLAM_Map**：显示2D静态地图（灰色背景层）
2. **Map**：显示2D静态地图（备用显示）

## 使用方法

### 方法1：使用修复后的配置文件（推荐）

```bash
# 重新启动系统（会自动使用修复后的配置）
source ~/humanoid_ws/install/setup.bash
ros2 launch humanoid_bringup robot_real.launch.py
```

RViz应该能立即显示2D地图！

### 方法2：手动修改现有RViz配置

如果你正在运行RViz但不想重启：

1. 在RViz左侧Displays面板中，找到 **SLAM_Map** 显示
2. 展开 **Topic** 部分
3. 修改以下设置：
   - **Durability Policy**: 从 `Volatile` 改为 `Transient Local`
   - **Depth**: 从 `5` 改为 `1`
4. 同样修改 **Map** 显示（如果有）
5. 点击 **Reset** 按钮重置显示

### 方法3：添加新的Map显示

如果现有显示有问题，可以重新添加：

1. 点击左下角 **Add** 按钮
2. 选择 **By topic** 标签
3. 找到 `/map` 话题
4. 选择 **Map** 显示类型
5. 点击 **OK**
6. 选中新添加的Map显示，在右侧属性中设置：
   - **Topic** → **Durability Policy**: `Transient Local`
   - **Alpha**: `0.7`（透明度）
   - **Color Scheme**: `map`

## 验证方法

### 1. 检查RViz中是否显示地图

- ✅ **正常**：能看到灰色的2D地图，显示墙壁和空闲区域
- ❌ **异常**：显示"No map received"或空白

### 2. 检查话题状态

```bash
# 查看/map话题信息
ros2 topic info /map

# 应该显示：
# Type: nav_msgs/msg/OccupancyGrid
# Publisher count: 1
```

### 3. 接收地图消息

```bash
# 测试是否能接收到地图
ros2 topic echo /map --once | head -20

# 应该能看到地图数据，包括：
# header:
#   frame_id: map
# info:
#   resolution: 0.05
#   width: 830
#   height: 588
```

### 4. 运行诊断脚本

```bash
bash ~/humanoid_ws/src/humanoid_navigation2/scripts/diagnose_rviz_map.sh
```

## 其他可能的问题

### Q1: 修改后仍然看不到地图？

**检查清单**：

1. **Fixed Frame设置正确**：
   - RViz顶部Global Options → Fixed Frame
   - 应该设置为 `map` 或 `odom`
   - 如果是 `base_footprint`，地图可能不在视野范围内

2. **相机视角问题**：
   - 使用鼠标中键旋转视角
   - 使用滚轮缩放
   - 地图可能在很远的地方（地图原点在[-10.7, -20.268]）

3. **Alpha值太低**：
   - 选中Map显示
   - 检查Alpha值（建议0.7~1.0）
   - 如果为0，地图完全透明

4. **Map显示被禁用**：
   - 检查Map显示左侧的复选框是否勾选

### Q2: 地图显示但位置不对？

这是正常的！2D地图的原点在`[-10.7, -20.268]`，机器人可能在地图的中心位置。

**调整方法**：
1. 在RViz中右键点击空白区域
2. 选择 "Zoom to Fit" 或 "Reset View"
3. 或者手动移动视角找到地图

### Q3: 地图显示但很模糊？

检查地图的分辨率设置：
```bash
ros2 topic echo /map --once | grep -A5 "info:"
```

应该显示：
```
resolution: 0.05  # 5cm/像素
width: 830        # 830像素
height: 588       # 588像素
```

实际地图大小：
- 宽度：830 × 0.05 = 41.5米
- 高度：588 × 0.05 = 29.4米

### Q4: Costmap地图也看不到？

Costmap（`/global_costmap/costmap` 和 `/local_costmap/costmap`）使用Volatile是正常的，因为它们持续发布更新。

如果看不到Costmap：
1. 检查global_costmap/local_costmap节点是否运行
2. 检查是否有TF变换（map→odom→base_footprint）
3. 检查机器人是否收到初始位姿

## 技术细节

### map_server的QoS配置

map_server在创建发布者时使用以下QoS：

```cpp
// nav2_map_server源码
rclcpp::QoS qos(rclcpp::KeepLast(1));
qos.transient_local();    // 持久性：保留最后一条消息
qos.reliable();           // 可靠性：可靠传输
```

这确保了：
- 新启动的订阅者（如RViz）能立即收到地图
- 地图只在启动或变化时发布一次，不会占用带宽

### RViz订阅者的QoS匹配

RViz订阅者必须使用兼容的QoS策略：

| 策略 | map_server发布 | RViz订阅 | 是否兼容 |
|------|---------------|---------|---------|
| Durability | Transient Local | Transient Local | ✓ 兼容 |
| Durability | Transient Local | Volatile | ✗ 不兼容 |
| Reliability | Reliable | Reliable | ✓ 兼容 |
| Reliability | Reliable | Best Effort | ✓ 兼容 |

**关键点**：订阅者的Durability不能比发布者更严格。

## 相关文件

- **RViz配置**: `/home/ubuntu/humanoid_ws/src/humanoid_navigation2/rviz/navigation.rviz`
- **地图文件**: `/home/ubuntu/humanoid_ws/src/humanoid_navigation2/maps/hall.yaml`
- **地图图片**: `/home/ubuntu/humanoid_ws/src/humanoid_navigation2/maps/hall.pgm`
- **诊断脚本**: `/home/ubuntu/humanoid_ws/src/humanoid_navigation2/scripts/diagnose_rviz_map.sh`

## 总结

**问题**：RViz使用Volatile订阅地图，错过了已经发布的消息

**修复**：将RViz的Durability Policy改为Transient Local

**效果**：RViz启动时立即收到地图，正常显示2D静态地图

**注意**：如果你修改了RViz配置并保存，确保不要改回Volatile设置！
