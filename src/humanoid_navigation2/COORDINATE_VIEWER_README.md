# 🤖 机器人位姿 & 地图坐标查看器使用说明

## 功能简介

这是一个多功能的坐标查看工具，可以：

### 机器人位姿显示（实时）
- 🔄 **实时显示**机器人当前位置 (x, y, z)
- 🧭 **实时显示**机器人姿态（Roll, Pitch, Yaw 欧拉角）
- ⏰ 显示位姿数据的时间戳
- 📋 一键复制位姿信息到剪贴板
- 💾 保存机器人位姿到历史记录

### 地图点击坐标显示
- 📍 在 RViz 中点击地图时显示坐标 (x, y, z)
- 📋 一键复制坐标到剪贴板
- 💾 保存常用坐标点（展台点、导航目标点等）
- 🗂️ 坐标点分类管理（位姿/点击）

## 使用方法

### 1. 启动导航系统
首先启动你的导航系统（确保 RViz 正常运行）。

### 2. 启动坐标查看器
有三种方式启动：

**方式一：使用 ros2 run 命令（推荐）**
```bash
source /home/ubuntu/humanoid_ws/install/setup.bash
ros2 run humanoid_navigation2 map_coordinate_viewer
```

**方式二：使用 launch 文件**
```bash
source /home/ubuntu/humanoid_ws/install/setup.bash
ros2 launch humanoid_navigation2 map_coordinate_viewer.launch.py
```

**方式三：直接运行 Python 脚本**
```bash
python3 /home/ubuntu/humanoid_ws/src/humanoid_navigation2/humanoid_navigation2/map_coordinate_viewer.py
```

### 3. 查看机器人位姿
启动后，GUI 窗口会**自动实时显示**机器人位姿：
- 位置：X, Y, Z（米）
- 姿态：Roll, Pitch, Yaw（角度）
- 坐标系：通常是 `map` 或 `odom`
- 更新时间：实时刷新

### 4. 在 RViz 中点击地图（可选）
1. 在 RViz 工具栏中选择 **"Publish Point"** 工具
2. 在地图上点击任意位置
3. 坐标信息将显示在 GUI 窗口的"地图点击坐标"区域

## GUI 界面说明

### 🤖 机器人实时位姿区域

| 显示项 | 说明 |
|--------|------|
| X, Y, Z | 机器人在地图中的三维位置（米） |
| Roll | 翻滚角（度） |
| Pitch | 俯仰角（度） |
| Yaw | 偏航角（度） |
| 坐标系 | 位姿参考坐标系（如 map） |
| 更新时间 | 最后一次收到位姿数据的时间 |

**功能按钮：**
- **📋 复制位姿** - 复制完整位姿信息（位置 + 姿态）到剪贴板
- **💾 保存位姿** - 保存当前机器人位姿到历史记录

### 📍 地图点击坐标区域

| 显示项 | 说明 |
|--------|------|
| X, Y, Z | 点击位置的三维坐标（米） |
| 坐标系 | 坐标参考系（如 map） |
| 时间 | 点击发生的时间戳 |

**功能按钮：**
- **📋 复制** - 复制坐标到剪贴板，格式：`(x, y, z)`
- **💾 保存** - 保存当前坐标到历史记录

### 📜 已保存的坐标点列表

显示所有保存的坐标点，包含：
- 名称
- X, Y, Z 坐标
- 类型（位姿/点击）

**操作：**
- **双击** - 快速加载坐标到显示区域
- **🗑️ 清空历史** - 删除所有保存的坐标点
- **📂 打开保存目录** - 打开文件管理器查看保存的 JSON 文件

### 快速保存预设名称
点击"保存位姿"或"保存坐标"时，提供以下预设名称快速选择：
- 展台点
- 导航目标点
- 工作站
- 充电点
- 等待点

## 使用场景示例

### 场景 1：获取展台点坐标
1. 启动坐标查看器
2. 控制机器人移动到展台位置
3. 点击"💾 保存位姿"
4. 选择预设名称"展台点"或输入自定义名称
5. 坐标保存到 `~/.humanoid_navigation/saved_points.json`

### 场景 2：在地图上标记导航目标
1. 在 RViz 中选择 "Publish Point" 工具
2. 在地图上点击目标位置
3. 点击"📋 复制"获取坐标
4. 或者点击"💾 保存"以便以后使用

### 场景 3：查看机器人当前朝向
- 查看 **Yaw** 值了解机器人朝向
  - 0° = 朝向 X 轴正方向
  - 90° = 朝向 Y 轴正方向
  - 180° = 朝向 X 轴负方向
  - -90° = 朝向 Y 轴负方向

## 坐标数据格式

保存的坐标点 JSON 格式示例：
```json
[
  {
    "name": "展台点 A",
    "x": 1.500,
    "y": -2.300,
    "z": 0.000,
    "frame_id": "map",
    "timestamp": "2026-04-17T14:30:00.123456",
    "type": "robot_pose"
  },
  {
    "name": "导航目标点",
    "x": 3.200,
    "y": 1.500,
    "z": 0.000,
    "frame_id": "map",
    "timestamp": "2026-04-17T14:35:00.654321",
    "type": "clicked"
  }
]
```

带姿态信息的位姿保存格式：
```json
{
  "name": "充电站",
  "x": 0.000,
  "y": 0.000,
  "z": 0.000,
  "roll": 0.0,
  "pitch": 0.0,
  "yaw": 1.571,
  "frame_id": "map",
  "timestamp": "2026-04-17T14:40:00.000000",
  "type": "robot_pose"
}
```

## 终端输出示例

```
🤖 机器人位姿 & 地图坐标查看器已启动
💡 使用方法：
   1. 机器人位姿会实时显示（订阅 /pose 或 /odom 话题）
   2. 在 RViz 中选择 'Publish Point' 工具点击地图可查看坐标
   3. 可以保存常用坐标点以便快速复用

[INFO] [map_coordinate_viewer]: 🤖 地图坐标查看器已启动，显示机器人位姿和点击坐标...
[INFO] [map_coordinate_viewer]: 📍 收到坐标点：x=1.234, y=-2.567, z=0.000 [@ map]
```

## 订阅的 ROS 话题

| 话题 | 消息类型 | 用途 |
|------|----------|------|
| `/pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 机器人位姿（主要） |
| `/odom` | `nav_msgs/msg/Odometry` | 里程计（备用） |
| `/clicked_point` | `geometry_msgs/msg/PointStamped` | RViz 点击坐标 |

## 常见问题

### Q: 机器人位姿不更新？
A: 检查以下几点：
1. 确认导航系统已启动
2. 确认 `/pose` 或 `/odom` 话题有数据：`ros2 topic echo /pose`
3. 检查 RViz 的 Fixed Frame 设置

### Q: 点击地图后没有反应？
A: 检查以下几点：
1. 确保坐标查看器节点已启动
2. 确认 RViz 中选择的工具是 "Publish Point"
3. 检查话题：`ros2 topic echo /clicked_point`

### Q: Yaw 角度显示不正确？
A: Yaw 角定义：
- 0° = 沿 X 轴正方向（地图前方）
- 90° = 沿 Y 轴正方向（地图左方）
- 180° 或 -180° = 沿 X 轴负方向（地图后方）
- -90° = 沿 Y 轴负方向（地图右方）

### Q: 如何在导航代码中使用保存的坐标？
A: 读取 `~/.humanoid_navigation/saved_points.json` 文件，获取坐标后发布到 `/goal_pose` 话题

## 快捷操作

### 在代码中快速设置导航目标
```python
# 示例：发送导航目标
from geometry_msgs.msg import PoseStamped

goal = PoseStamped()
goal.header.frame_id = 'map'
goal.header.stamp = node.get_clock().now().to_msg()
goal.pose.position.x = 1.500  # 从保存的坐标获取
goal.pose.position.y = -2.300
goal.pose.position.z = 0.0
goal.pose.orientation.w = 1.0  # 朝向可根据保存的 yaw 计算

goal_pub.publish(goal)
```

### 从保存的文件中加载坐标
```python
import json
from pathlib import Path

# 加载保存的坐标点
save_path = Path.home() / '.humanoid_navigation' / 'saved_points.json'
with open(save_path, 'r', encoding='utf-8') as f:
    points = json.load(f)

# 查找特定名称的坐标
for point in points:
    if point['name'] == '展台点 A':
        print(f"展台点坐标：x={point['x']}, y={point['y']}, z={point['z']}")
        if 'yaw' in point:
            print(f"姿态：yaw={math.degrees(point['yaw']):.1f}°")
        break
```

## 技术细节

- **位姿解算**：四元数转欧拉角（Z-Y-X 顺序）
- **坐标精度**：显示精度为 3 位小数（毫米级）
- **角度精度**：显示精度为 1 位小数
- **更新频率**：取决于 `/pose` 或 `/odom` 话题的发布频率

## 文件位置

| 文件 | 路径 |
|------|------|
| 脚本 | `~/humanoid_ws/src/humanoid_navigation2/humanoid_navigation2/map_coordinate_viewer.py` |
| 保存的坐标 | `~/.humanoid_navigation/saved_points.json` |
| Launch 文件 | `~/humanoid_ws/src/humanoid_navigation2/launch/map_coordinate_viewer.launch.py` |
| 使用说明 | `~/humanoid_ws/src/humanoid_navigation2/scripts/COORDINATE_VIEWER_README.md` |

## 更新日志

- **v2.0** - 添加机器人实时位姿显示
  - 新增 Roll/Pitch/Yaw 欧拉角显示
  - 新增位姿保存功能
  - 坐标点分类管理（位姿/点击）
  - 优化 GUI 布局
