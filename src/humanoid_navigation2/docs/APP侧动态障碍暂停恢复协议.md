# APP 侧动态障碍暂停恢复协议

本文档说明 ROS 侧“动态障碍物暂停等待、障碍消失后自动恢复导航”的 websocket 消息协议，供 APP 开发对接。

## 1. 总体行为

当机器人导航中检测到前方路径被障碍物持续挡住时，ROS 会自动进入“因障碍暂停”状态：

1. 取消当前 Nav2 goal，让机器人停止。
2. 推送 `navigation_paused`，其中 `pause_source=obstacle_wait`。
3. 等待期间每 4 秒推送一次 `navigation_obstacle_blocked`。
4. ROS 持续检查 local costmap 前方窗口。
5. 当前方连续多帧 clear 后，ROS 自动恢复当前 waypoint 导航。
6. 恢复时推送 `navigation_resumed`，其中 `resume_source=obstacle_wait`。

注意：这条自动恢复链路不依赖 APP 的“继续”按钮。APP 后续如果支持人工脱困后点击继续，可复用本文档第 7 节的 `resume_navigation` 命令。

## 2. Websocket 外层格式

APP 收到的导航事件由 `data_integration_node_recoverable.py` 统一包装为 websocket push。

外层固定结构如下：

```json
{
  "protocol_version": "1.0",
  "message_id": "push_1781240000000_1234",
  "timestamp": 1781240000.123,
  "message_type": "push",
  "data_type": "navigation_status",
  "source": "data_integration",
  "destination": "all",
  "data": {},
  "metadata": {
    "status": "success",
    "error_code": "",
    "error_message": "",
    "request_id": "",
    "data_freshness": 0.0,
    "qos_level": "realtime",
    "push_reason": "navigation_event"
  }
}
```

APP 主要解析：

- `message_type == "push"`
- `data_type == "navigation_status"`
- `data.event_type`
- `data.event_data`

`data` 中除了 ROS 原始事件字段，还会被数据整合节点补充一些增强字段，例如：

- `current_pose`
- `current_path`
- `progress_percentage`
- `estimated_remaining_time`
- `system_timestamp`
- `performance_metrics`
- `robot_speed`
- `current_velocity`

APP 做 UI 状态判断时，建议优先使用 `data.event_type` 和 `data.event_data`，增强字段只用于展示。

## 3. 事件一：因障碍暂停

### 3.1 触发时机

导航执行中，ROS 检测到机器人持续停滞超过 `obstacle_block_timeout`，默认 5 秒，认为前方路径被障碍物挡住。

ROS 会推送：

- `event_type = "navigation_paused"`
- `event_data.pause_source = "obstacle_wait"`
- `event_data.resume_mode = "auto"`
- `event_data.waiting_for_obstacle_clear = true`

### 3.2 示例 JSON

```json
{
  "message_type": "push",
  "data_type": "navigation_status",
  "source": "data_integration",
  "destination": "all",
  "data": {
    "event_type": "navigation_paused",
    "event_data": {
      "pause_source": "obstacle_wait",
      "reason": "检测到障碍物，前方路径被挡住",
      "resume_mode": "auto",
      "waiting_for_obstacle_clear": true,
      "pause_location": {
        "x": 1.234,
        "y": 2.345,
        "z": 0.0
      },
      "pause_time": 1781240000.123,
      "pause_duration": 0,
      "current_waypoint_id": "wp_001",
      "current_waypoint_name": "展厅入口",
      "waypoint_index": 0,
      "total_waypoints": 5,
      "block_duration": 5.2,
      "clear_confirmed_frames": 0,
      "clear_required_frames": 5
    },
    "timestamp": 1781240000.123,
    "current_state": "paused",
    "navigation_mode": "single_point",
    "sequence_id": "single_1781240000",
    "progress_percentage": 34.5,
    "system_timestamp": 1781240000.130
  },
  "metadata": {
    "status": "success",
    "qos_level": "realtime",
    "push_reason": "navigation_event"
  }
}
```

### 3.3 APP 建议处理

- 将导航 UI 状态置为“已暂停”。
- 根据 `pause_source=obstacle_wait` 展示“因障碍暂停”或“等待障碍物消失”。
- 不要把该事件当成导航失败。
- `resume_mode=auto` 表示 ROS 会在障碍 clear 后自动恢复。

## 4. 事件二：等待期间障碍提醒

### 4.1 触发时机

机器人处于 `obstacle_wait` 暂停状态时，ROS 每 4 秒推送一次。

ROS 会推送：

- `event_type = "navigation_obstacle_blocked"`
- `event_data.reason = "检测到障碍物，前方路径被挡住"`
- `event_data.waiting_for_obstacle_clear = true`

### 4.2 示例 JSON

```json
{
  "message_type": "push",
  "data_type": "navigation_status",
  "source": "data_integration",
  "destination": "all",
  "data": {
    "event_type": "navigation_obstacle_blocked",
    "event_data": {
      "reason": "检测到障碍物，前方路径被挡住",
      "block_duration": 13.4,
      "blocked_waypoint_id": "wp_001",
      "blocked_waypoint_name": "展厅入口",
      "blocked_waypoint_index": 0,
      "total_waypoints": 5,
      "position": [1.0, 2.0, 0.0],
      "waiting_for_obstacle_clear": true,
      "clear_confirmed_frames": 0,
      "clear_required_frames": 5,
      "pause_source": "obstacle_wait",
      "front_obstacle_stats": {
        "available": true,
        "frame_id": "odom_ground",
        "window_front_min_x_m": 0.15,
        "window_front_max_x_m": 1.2,
        "window_half_width_m": 0.45,
        "sample_cells": 414,
        "occupied_cells": 18,
        "max_cost": 254,
        "blocked": true
      }
    },
    "timestamp": 1781240013.456,
    "current_state": "paused",
    "navigation_mode": "single_point",
    "sequence_id": "single_1781240000",
    "progress_percentage": 34.5,
    "system_timestamp": 1781240013.460
  },
  "metadata": {
    "status": "success",
    "qos_level": "realtime",
    "push_reason": "navigation_event"
  }
}
```

### 4.3 APP 建议处理

- 用该事件刷新“障碍等待中”的提示文案。
- 文案建议直接使用 `event_data.reason`。
- 如果 APP 有 toast/弹窗，注意做去重或覆盖更新，因为该事件会每 4 秒推一次。
- 不建议每次都播放完整错误音效；更适合做状态维持提醒。

## 5. 事件三：障碍消失后自动恢复

### 5.1 触发时机

ROS 通过 local costmap 前方窗口判断障碍消失，并且连续 clear 达到 `clear_required_frames`，默认 5 帧。

ROS 会自动重新导航到当前 waypoint，并推送：

- `event_type = "navigation_resumed"`
- `event_data.resume_source = "obstacle_wait"`
- `event_data.resume_reason = "obstacle_cleared_auto_resume"`

### 5.2 示例 JSON

```json
{
  "message_type": "push",
  "data_type": "navigation_status",
  "source": "data_integration",
  "destination": "all",
  "data": {
    "event_type": "navigation_resumed",
    "event_data": {
      "resumed_waypoint_id": "wp_001",
      "resumed_waypoint_name": "展厅入口",
      "waypoint_index": 0,
      "total_waypoints": 5,
      "pause_duration_actual": 18.7,
      "resume_reason": "obstacle_cleared_auto_resume",
      "resume_source": "obstacle_wait",
      "front_obstacle_stats": {
        "available": true,
        "frame_id": "odom_ground",
        "window_front_min_x_m": 0.15,
        "window_front_max_x_m": 1.2,
        "window_half_width_m": 0.45,
        "sample_cells": 414,
        "occupied_cells": 0,
        "max_cost": 0,
        "blocked": false
      }
    },
    "timestamp": 1781240019.000,
    "current_state": "executing",
    "navigation_mode": "single_point",
    "sequence_id": "single_1781240000",
    "progress_percentage": 34.5,
    "system_timestamp": 1781240019.010
  },
  "metadata": {
    "status": "success",
    "qos_level": "realtime",
    "push_reason": "navigation_event"
  }
}
```

### 5.3 APP 建议处理

- 清除“障碍等待中”提示。
- UI 状态切回“导航中”。
- 如果之前按钮显示为“继续”，自动恢复后应回到正常导航态。

## 6. 用户手动暂停的区分

用户从 APP 主动点击暂停时，ROS 仍推送 `navigation_paused`，但字段不同：

- `event_data.pause_source = "user_request"`
- `event_data.resume_mode = "manual"`
- `event_data.waiting_for_obstacle_clear = false`

示例：

```json
{
  "message_type": "push",
  "data_type": "navigation_status",
  "data": {
    "event_type": "navigation_paused",
    "event_data": {
      "pause_source": "user_request",
      "reason": "用户手动暂停导航",
      "resume_mode": "manual",
      "waiting_for_obstacle_clear": false,
      "pause_location": {
        "x": 1.234,
        "y": 2.345,
        "z": 0.0
      },
      "pause_time": 1781240100.123,
      "pause_duration": 0,
      "current_waypoint_id": "wp_001",
      "current_waypoint_name": "展厅入口",
      "waypoint_index": 0,
      "total_waypoints": 5
    },
    "timestamp": 1781240100.123,
    "current_state": "paused",
    "navigation_mode": "single_point",
    "sequence_id": "single_1781240000"
  }
}
```

APP 区分规则：

- `pause_source == "user_request"`：用户手动暂停。
- `pause_source == "obstacle_wait"`：系统因障碍暂停，等待障碍 clear。
- `waiting_for_obstacle_clear == true`：显示障碍等待态。

## 7. APP 主动继续命令

动态障碍自动恢复不依赖该命令。后续如果 APP 要做“人工脱困后点击继续”，可使用现有 `resume_navigation`。

APP 发给 websocket server：

```json
{
  "protocol_version": "1.0",
  "message_id": "req_1781240200_0001",
  "timestamp": 1781240200.000,
  "message_type": "request",
  "data_type": "navigation_control",
  "source": "app",
  "destination": "websocket_server",
  "data": {
    "command_type": "resume_navigation",
    "reason": "manual_recovery_finished"
  },
  "metadata": {
    "request_id": "",
    "qos_level": "standard"
  }
}
```

当前 websocket server 主要透传以下字段到 ROS：

- `command_type`
- `waypoint_id`
- `waypoint_ids`
- `exhibition_ids`

因此 `reason` 字段目前未必会透传到 `navigation_state_manager`。如果 APP 需要 ROS 精确区分“人工脱困完成后继续”，建议后续同步扩展 `websocket_server.py` 的 `navigation_control` 透传字段。

## 8. 导航确认消息

除 `navigation_status` 事件外，ROS 侧还会通过 `navigation_command_result` 推送命令确认。动态障碍进入等待时，会保留一次兼容确认：

```json
{
  "message_type": "push",
  "data_type": "navigation_command_result",
  "source": "data_integration",
  "destination": "all",
  "data": {
    "ack_type": "navigation_obstacle_blocked",
    "status": "error",
    "message": "检测到障碍物，前方路径被挡住",
    "timestamp": 1781240000.123
  },
  "metadata": {
    "status": "error",
    "error_code": "nav_error",
    "error_message": "检测到障碍物，前方路径被挡住"
  }
}
```

APP 推荐以 `data_type=navigation_status` 作为 UI 状态主来源。

`navigation_command_result` 更适合用于命令回执或兼容旧逻辑，不建议用它覆盖主导航状态。

## 9. 周期状态摘要

除离散事件外，`/navigation/status` 还会周期性发布状态摘要。数据整合后 APP 也可能收到同类状态。

障碍等待相关字段：

```json
{
  "current_state": "paused",
  "detailed_state": "OBSTACLE_WAITING",
  "pause_source": "obstacle_wait",
  "pause_reason": "检测到障碍物，前方路径被挡住",
  "resume_mode": "auto",
  "waiting_for_obstacle_clear": true,
  "obstacle_wait_active": true,
  "obstacle_wait_duration": 12.3,
  "obstacle_clear_confirm_count": 0,
  "obstacle_clear_required_frames": 5,
  "front_obstacle_blocked": true,
  "front_obstacle_stats": {
    "available": true,
    "frame_id": "odom_ground",
    "sample_cells": 414,
    "occupied_cells": 18,
    "max_cost": 254,
    "blocked": true
  }
}
```

APP 可以用周期状态做断线重连后的 UI 恢复：

- `obstacle_wait_active=true`：显示障碍等待态。
- `current_state=paused` 且 `pause_source=obstacle_wait`：显示因障碍暂停。
- `current_state=executing`：恢复导航中。

## 10. 字段说明

| 字段 | 类型 | 说明 |
|---|---:|---|
| `event_type` | string | 导航事件类型，例如 `navigation_paused` |
| `event_data` | object | 事件业务字段 |
| `pause_source` | string | 暂停来源：`user_request`、`obstacle_wait`、`localization_recovery`、`prior_map_degraded` |
| `resume_mode` | string | 恢复模式：`manual` 表示 APP/用户继续，`auto` 表示 ROS 自动恢复 |
| `waiting_for_obstacle_clear` | bool | 是否正在等待障碍消失 |
| `block_duration` | number | 障碍阻塞或等待时长，单位秒 |
| `clear_confirmed_frames` | number | 已连续确认 clear 的帧数 |
| `clear_required_frames` | number | 自动恢复所需连续 clear 帧数，默认 5 |
| `resume_source` | string | 恢复来源，例如 `obstacle_wait` |
| `resume_reason` | string | 恢复原因，例如 `obstacle_cleared_auto_resume` |
| `front_obstacle_stats` | object | ROS 侧前方 costmap 窗口统计，仅用于调试和展示 |
| `front_obstacle_stats.blocked` | bool | 当前前方窗口是否仍有高代价障碍 |
| `front_obstacle_stats.occupied_cells` | number | 前方窗口内高代价栅格数量 |
| `front_obstacle_stats.max_cost` | number | 前方窗口最大 cost |
| `current_state` | string | 导航状态：`executing`、`paused`、`idle` 等 |
| `detailed_state` | string | 更细状态，例如 `OBSTACLE_WAITING` |

## 11. APP UI 推荐规则

| 条件 | UI 状态建议 |
|---|---|
| `event_type=navigation_paused` 且 `pause_source=obstacle_wait` | 显示“因障碍暂停/等待障碍消失” |
| `event_type=navigation_obstacle_blocked` | 保持障碍等待提示，刷新等待时长 |
| `event_type=navigation_resumed` 且 `resume_source=obstacle_wait` | 清除障碍提示，切回导航中 |
| `event_type=navigation_paused` 且 `pause_source=user_request` | 显示“已手动暂停” |
| 周期状态 `obstacle_wait_active=true` | 断线重连后恢复障碍等待 UI |

## 12. 注意事项

1. `navigation_obstacle_blocked` 在等待期间会每 4 秒推一次，APP 需要避免重复弹窗堆叠。
2. 动态障碍自动恢复由 ROS 侧完成，APP 不需要主动发继续命令。
3. 人工脱困后点击继续属于后续 APP 能力，可复用 `resume_navigation`。
4. 当前 `reason` 透传到 ROS 的命令链路还未完全扩展，后续如需精细区分继续原因，需要同步改 `websocket_server.py`。
5. APP 判断主状态时，优先看 `navigation_status.data.event_type/event_data`，不要只依赖 `navigation_command_result`。
