# 障碍物阻塞检测功能说明

## 功能概述

在机器人导航过程中，如果前方遇到障碍物导致机器人原地等待，系统会在等待超时（默认5秒）后自动向APP上报导航异常，原因为"检测到障碍物，前方路径被挡住"。

## 工作原理

1. **速度检测**：在导航执行过程中（`EXECUTING`状态），持续监测机器人的线速度和角速度
2. **阻塞判断**：当总速度（线速度+角速度的合成）低于阈值（默认0.05 m/s）时，判定为机器人停滞
3. **超时计时**：从检测到停滞开始计时，持续超过超时时间（默认5秒）后触发异常上报
4. **异常上报**：通过现有的WebSocket通信链路上报异常状态给APP
5. **自动恢复**：如果机器人在超时前恢复运动，自动重置阻塞检测状态

## 修改的文件

### `/home/ubuntu/humanoid_ws/src/humanoid_navigation/humanoid_navigation/navigation_state_manager.py`

#### 新增参数配置
- `obstacle_block_timeout`: 障碍物阻塞超时时间（秒），默认 `5.0`
- `velocity_threshold`: 判断机器人是否停滞的速度阈值（m/s），默认 `0.05`

#### 新增状态变量
- `is_blocked_by_obstacle`: 是否正在被障碍物阻塞
- `block_start_time`: 阻塞开始时间戳
- `block_reported`: 是否已上报过阻塞（防止重复上报）

#### 新增核心函数

1. **`check_obstacle_blockage()`**
   - 在 `odom_callback` 中被调用
   - 检测机器人速度是否低于阈值
   - 管理阻塞检测状态机

2. **`handle_obstacle_block_timeout(block_duration)`**
   - 处理阻塞超时事件
   - 上报异常状态到 `/navigation/status`
   - 发送错误确认消息

3. **`reset_block_detection()`**
   - 重置阻塞检测状态
   - 在导航完成/取消/失败时自动调用

## 上报的数据格式

当触发障碍物阻塞超时时，会发布以下状态更新：

```json
{
  "event_type": "navigation_obstacle_blocked",
  "event_data": {
    "reason": "检测到障碍物，前方路径被挡住",
    "block_duration": 5.2,
    "blocked_waypoint_id": "waypoint_001",
    "blocked_waypoint_name": "大厅入口",
    "blocked_waypoint_index": 2,
    "total_waypoints": 5,
    "position": [1.5, 2.3, 0.0]
  },
  "timestamp": 1234567890.123,
  "current_state": "executing",
  "navigation_mode": "multi_point",
  "sequence_id": "multi_1234567890"
}
```

同时在状态摘要中新增字段：
```json
{
  "obstacle_blocked": true,
  "block_duration": 5.2,
  "block_reported": true
}
```

## 数据流向

```
里程计 /odom
    ↓
navigation_state_manager (odom_callback)
    ↓
check_obstacle_blockage() 检测阻塞
    ↓
handle_obstacle_block_timeout() 超时处理
    ↓
publish_status_update("navigation_obstacle_blocked")
    ↓
/navigation/status topic
    ↓
data_integration_node 订阅并增强
    ↓
/integration/push_messages
    ↓
websocket_server 转发给APP
    ↓
APP 收到异常通知
```

## 配置参数调整

如果需要调整检测灵敏度，可以在启动文件中修改参数：

```python
# 在 navigation.launch.py 中
Node(
    package='humanoid_navigation',
    executable='navigation_state_manager',
    parameters=[{
        'obstacle_block_timeout': 5.0,      # 阻塞超时时间（秒）
        'velocity_threshold': 0.05,         # 速度阈值（m/s）
        # ... 其他参数
    }]
)
```

### 参数建议值

- **obstacle_block_timeout**: 
  - 快速响应：3.0秒
  - 标准：5.0秒（推荐）
  - 宽松：8.0秒

- **velocity_threshold**:
  - 敏感：0.02 m/s（微小移动就会重置）
  - 标准：0.05 m/s（推荐）
  - 宽松：0.10 m/s（允许较大速度波动）

## 行为特性

1. **不中断导航**：上报异常后，机器人会继续尝试导航（Nav2的局部规划器会持续尝试绕过障碍物）
2. **单次上报**：每次导航任务中，阻塞超时只上报一次（`block_reported` 标志防止重复）
3. **自动重置**：导航任务结束（完成/取消/失败）后，阻塞检测状态自动重置
4. **恢复检测**：如果机器人在超时前恢复运动（速度超过阈值），阻塞计时自动重置

## 测试建议

1. **正常场景测试**：在导航路径上放置障碍物，观察5秒后是否上报异常
2. **恢复场景测试**：放置障碍物后快速移除，验证阻塞检测是否正确重置
3. **参数调优测试**：调整 `velocity_threshold` 观察不同场景下的检测效果

## 注意事项

- 该功能依赖于 `/odom` 话题的速度信息，确保里程计节点正常工作
- 速度阈值需要根据机器人底盘特性调优，避免误判
- 如果需要在阻塞时主动取消导航，可以取消注释 `handle_obstacle_block_timeout` 函数末尾的代码
