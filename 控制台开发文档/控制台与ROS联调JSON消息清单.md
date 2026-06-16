# 控制台与ROS联调JSON消息清单

## 1. 文档目的

本文档用于给控制台前端、后端网关、ROS 联调三方提供一份统一的 JSON 消息参考清单，目标是把第一版 MVP 涉及到的请求、ACK、状态推送、错误消息全部收口，避免联调时出现：

1. 按钮点了但不知道该等哪个消息
2. ACK 和真实状态推送混淆
3. 前端解析字段口径不一致
4. 暂停原因、异常原因、切图原因表达不统一

说明：

1. 本文档以“控制台前端 <-> 后端网关”消息为主。
2. 后端再如何转换成 ROS 内部消息，不在本文档展开。
3. 第一版采用 `protocol_version = "2.0"`。
4. 所有示例都尽量给出完整 JSON，不做省略号简写。

## 2. 通用消息包格式

建议第一版所有消息统一遵循同一外层结构。

### 2.1 通用字段说明

```json
{
  "protocol_version": "2.0",
  "type": "message_type",
  "request_id": "req_20260614_000001",
  "timestamp": 1781395200000,
  "source": "web_console",
  "target": "gateway",
  "data": {}
}
```

字段说明：

1. `protocol_version`
   当前协议版本，第一版固定使用 `"2.0"`。

2. `type`
   消息类型，例如：
   `start_route_task`、`command_ack`、`navigation_state_changed`。

3. `request_id`
   请求唯一 ID。
   命令消息必须有。
   ACK 应回传同一个 `request_id`。
   主动推送类状态消息可为空字符串，也可由后端生成事件 ID。

4. `timestamp`
   毫秒时间戳。

5. `source`
   消息发送方，例如：
   `web_console`、`gateway`、`ros_bridge`。

6. `target`
   消息接收方，例如：
   `gateway`、`web_console`。

7. `data`
   真正的业务负载。

## 3. 命令消息与 ACK 消息规则

第一版强烈建议遵守以下规则：

1. 所有控制命令都必须有 `request_id`。
2. 后端收到命令后必须先回 `command_ack`。
3. 前端不能把 `command_ack` 当成业务状态变化。
4. 页面主状态必须等待后续 `*_state_changed` 推送。

例如：

1. 前端发 `pause_route_task`
2. 后端返回 `command_ack`
3. ROS 真正暂停成功后，再推 `navigation_state_changed`

## 4. 查询类消息

## 4.1 查询当前机器人快照

### 4.1.1 请求

```json
{
  "protocol_version": "2.0",
  "type": "get_robot_snapshot",
  "request_id": "req_20260614_100001",
  "timestamp": 1781395200000,
  "source": "web_console",
  "target": "gateway",
  "data": {
    "robot_id": "XR-102"
  }
}
```

### 4.1.2 响应

```json
{
  "protocol_version": "2.0",
  "type": "get_robot_snapshot_result",
  "request_id": "req_20260614_100001",
  "timestamp": 1781395200123,
  "source": "gateway",
  "target": "web_console",
  "data": {
    "success": true,
    "error_code": "",
    "error_message": "",
    "robot": {
      "robot_id": "XR-102",
      "robot_name": "小贝机器人 XR-102",
      "online": true,
      "battery_percent": 78,
      "current_map_id": "hall_a_floor_1",
      "current_map_name": "A栋一层导览图",
      "localization_status": "good",
      "localization_score": 0.92,
      "pose": {
        "x": 23.45,
        "y": 16.87,
        "yaw_deg": 135.6
      },
      "velocity": {
        "linear_mps": 0.42,
        "angular_radps": 0.12
      },
      "navigation_main_state": "running",
      "navigation_reason_code": "",
      "navigation_reason_text": "",
      "recording": false
    }
  }
}
```

前端解析重点：

1. 用于页面初始化。
2. 可作为断线重连后的第一页快照。

## 4.2 查询地图列表

### 4.2.1 请求

```json
{
  "protocol_version": "2.0",
  "type": "get_map_list",
  "request_id": "req_20260614_100002",
  "timestamp": 1781395200200,
  "source": "web_console",
  "target": "gateway",
  "data": {
    "robot_id": "XR-102"
  }
}
```

### 4.2.2 响应

```json
{
  "protocol_version": "2.0",
  "type": "get_map_list_result",
  "request_id": "req_20260614_100002",
  "timestamp": 1781395200280,
  "source": "gateway",
  "target": "web_console",
  "data": {
    "success": true,
    "error_code": "",
    "error_message": "",
    "current_map_id": "hall_a_floor_1",
    "maps": [
      {
        "map_id": "hall_a_floor_1",
        "map_name": "A栋一层导览图",
        "is_current": true
      },
      {
        "map_id": "hall_b_floor_1",
        "map_name": "B栋一层实验室图",
        "is_current": false
      }
    ]
  }
}
```

## 4.3 查询当前路线任务快照

### 4.3.1 请求

```json
{
  "protocol_version": "2.0",
  "type": "get_route_task_snapshot",
  "request_id": "req_20260614_100003",
  "timestamp": 1781395200300,
  "source": "web_console",
  "target": "gateway",
  "data": {
    "robot_id": "XR-102"
  }
}
```

### 4.3.2 响应

```json
{
  "protocol_version": "2.0",
  "type": "get_route_task_snapshot_result",
  "request_id": "req_20260614_100003",
  "timestamp": 1781395200400,
  "source": "gateway",
  "target": "web_console",
  "data": {
    "success": true,
    "error_code": "",
    "error_message": "",
    "route_task": {
      "route_task_id": "NAV-20240614-001",
      "route_name": "展厅上午导览",
      "navigation_main_state": "running",
      "reason_code": "",
      "reason_text": "",
      "current_waypoint_id": "13",
      "current_waypoint_name": "任务点13",
      "next_waypoint_id": "14",
      "next_waypoint_name": "辅助点14",
      "jump_target_waypoint_id": "",
      "jump_target_waypoint_name": "",
      "progress_percent": 62.5,
      "sequence": [
        {
          "waypoint_id": "10",
          "waypoint_name": "任务点10",
          "waypoint_role": "task",
          "walk_direction": "forward",
          "need_broadcast": true,
          "status": "completed"
        },
        {
          "waypoint_id": "11",
          "waypoint_name": "辅助点11",
          "waypoint_role": "transit",
          "walk_direction": "forward",
          "need_broadcast": false,
          "status": "completed"
        },
        {
          "waypoint_id": "12",
          "waypoint_name": "辅助点12",
          "waypoint_role": "transit",
          "walk_direction": "forward",
          "need_broadcast": false,
          "status": "completed"
        },
        {
          "waypoint_id": "13",
          "waypoint_name": "任务点13",
          "waypoint_role": "task",
          "walk_direction": "forward",
          "need_broadcast": true,
          "status": "running"
        },
        {
          "waypoint_id": "14",
          "waypoint_name": "辅助点14",
          "waypoint_role": "transit",
          "walk_direction": "forward",
          "need_broadcast": false,
          "status": "pending"
        },
        {
          "waypoint_id": "16",
          "waypoint_name": "任务点16",
          "waypoint_role": "task",
          "walk_direction": "forward",
          "need_broadcast": true,
          "status": "pending"
        }
      ]
    }
  }
}
```

## 5. 控制类命令消息

## 5.1 开始路线任务

### 5.1.1 请求

说明：

1. 第一版支持传点位 ID 列表。
2. 如有需要，也兼容传完整点位详情列表。

```json
{
  "protocol_version": "2.0",
  "type": "start_route_task",
  "request_id": "req_20260614_200001",
  "timestamp": 1781395201000,
  "source": "web_console",
  "target": "gateway",
  "data": {
    "robot_id": "XR-102",
    "route_task_id": "NAV-20240614-001",
    "route_name": "展厅上午导览",
    "map_id": "hall_a_floor_1",
    "waypoint_ref_mode": "id_list",
    "waypoint_ids": [
      "10",
      "11",
      "12",
      "13",
      "14",
      "16"
    ],
    "waypoints_revision": 3,
    "start_mode": "from_first_waypoint"
  }
}
```

### 5.1.2 ACK

```json
{
  "protocol_version": "2.0",
  "type": "command_ack",
  "request_id": "req_20260614_200001",
  "timestamp": 1781395201080,
  "source": "gateway",
  "target": "web_console",
  "data": {
    "success": true,
    "command_type": "start_route_task",
    "accepted": true,
    "error_code": "",
    "error_message": "",
    "ack_message": "开始路线任务命令已受理"
  }
}
```

### 5.1.3 前端处理规则

1. 收到 ACK 后，只结束按钮 loading。
2. 页面真正进入 `starting` 或 `running`，必须等 `navigation_state_changed`。

## 5.2 暂停路线任务

### 5.2.1 请求

```json
{
  "protocol_version": "2.0",
  "type": "pause_route_task",
  "request_id": "req_20260614_200002",
  "timestamp": 1781395202000,
  "source": "web_console",
  "target": "gateway",
  "data": {
    "robot_id": "XR-102",
    "route_task_id": "NAV-20240614-001",
    "pause_reason": "manual_pause"
  }
}
```

### 5.2.2 ACK

```json
{
  "protocol_version": "2.0",
  "type": "command_ack",
  "request_id": "req_20260614_200002",
  "timestamp": 1781395202070,
  "source": "gateway",
  "target": "web_console",
  "data": {
    "success": true,
    "command_type": "pause_route_task",
    "accepted": true,
    "error_code": "",
    "error_message": "",
    "ack_message": "暂停路线任务命令已受理"
  }
}
```

## 5.3 继续路线任务

### 5.3.1 请求

```json
{
  "protocol_version": "2.0",
  "type": "resume_route_task",
  "request_id": "req_20260614_200003",
  "timestamp": 1781395203000,
  "source": "web_console",
  "target": "gateway",
  "data": {
    "robot_id": "XR-102",
    "route_task_id": "NAV-20240614-001",
    "resume_reason": "manual_resume"
  }
}
```

### 5.3.2 ACK

```json
{
  "protocol_version": "2.0",
  "type": "command_ack",
  "request_id": "req_20260614_200003",
  "timestamp": 1781395203060,
  "source": "gateway",
  "target": "web_console",
  "data": {
    "success": true,
    "command_type": "resume_route_task",
    "accepted": true,
    "error_code": "",
    "error_message": "",
    "ack_message": "继续路线任务命令已受理"
  }
}
```

## 5.4 终止路线任务

### 5.4.1 请求

```json
{
  "protocol_version": "2.0",
  "type": "stop_route_task",
  "request_id": "req_20260614_200004",
  "timestamp": 1781395204000,
  "source": "web_console",
  "target": "gateway",
  "data": {
    "robot_id": "XR-102",
    "route_task_id": "NAV-20240614-001",
    "stop_reason": "manual_stop"
  }
}
```

### 5.4.2 ACK

```json
{
  "protocol_version": "2.0",
  "type": "command_ack",
  "request_id": "req_20260614_200004",
  "timestamp": 1781395204080,
  "source": "gateway",
  "target": "web_console",
  "data": {
    "success": true,
    "command_type": "stop_route_task",
    "accepted": true,
    "error_code": "",
    "error_message": "",
    "ack_message": "终止路线任务命令已受理"
  }
}
```

## 5.5 跳点

### 5.5.1 请求

```json
{
  "protocol_version": "2.0",
  "type": "jump_route_waypoint",
  "request_id": "req_20260614_200005",
  "timestamp": 1781395205000,
  "source": "web_console",
  "target": "gateway",
  "data": {
    "robot_id": "XR-102",
    "route_task_id": "NAV-20240614-001",
    "target_waypoint_id": "16",
    "target_waypoint_name": "任务点16",
    "jump_reason": "operator_selected_jump"
  }
}
```

### 5.5.2 ACK

```json
{
  "protocol_version": "2.0",
  "type": "command_ack",
  "request_id": "req_20260614_200005",
  "timestamp": 1781395205070,
  "source": "gateway",
  "target": "web_console",
  "data": {
    "success": true,
    "command_type": "jump_route_waypoint",
    "accepted": true,
    "error_code": "",
    "error_message": "",
    "ack_message": "跳点命令已受理"
  }
}
```

前端处理规则：

1. ACK 仅表示后端受理。
2. 是否真正跳到目标点，以后续 `route_task_progress_changed` 和 `navigation_state_changed` 为准。

## 5.6 切换地图

### 5.6.1 请求

```json
{
  "protocol_version": "2.0",
  "type": "switch_map",
  "request_id": "req_20260614_200006",
  "timestamp": 1781395206000,
  "source": "web_console",
  "target": "gateway",
  "data": {
    "robot_id": "XR-102",
    "target_map_id": "hall_b_floor_1",
    "target_map_name": "B栋一层实验室图"
  }
}
```

### 5.6.2 ACK

```json
{
  "protocol_version": "2.0",
  "type": "command_ack",
  "request_id": "req_20260614_200006",
  "timestamp": 1781395206080,
  "source": "gateway",
  "target": "web_console",
  "data": {
    "success": true,
    "command_type": "switch_map",
    "accepted": true,
    "error_code": "",
    "error_message": "",
    "ack_message": "切换地图命令已受理"
  }
}
```

## 5.7 开始录包

### 5.7.1 请求

```json
{
  "protocol_version": "2.0",
  "type": "start_recording",
  "request_id": "req_20260614_200007",
  "timestamp": 1781395207000,
  "source": "web_console",
  "target": "gateway",
  "data": {
    "robot_id": "XR-102",
    "record_name": "nav_debug_20260614_01"
  }
}
```

### 5.7.2 ACK

```json
{
  "protocol_version": "2.0",
  "type": "command_ack",
  "request_id": "req_20260614_200007",
  "timestamp": 1781395207080,
  "source": "gateway",
  "target": "web_console",
  "data": {
    "success": true,
    "command_type": "start_recording",
    "accepted": true,
    "error_code": "",
    "error_message": "",
    "ack_message": "开始录包命令已受理"
  }
}
```

## 5.8 停止录包

### 5.8.1 请求

```json
{
  "protocol_version": "2.0",
  "type": "stop_recording",
  "request_id": "req_20260614_200008",
  "timestamp": 1781395208000,
  "source": "web_console",
  "target": "gateway",
  "data": {
    "robot_id": "XR-102"
  }
}
```

### 5.8.2 ACK

```json
{
  "protocol_version": "2.0",
  "type": "command_ack",
  "request_id": "req_20260614_200008",
  "timestamp": 1781395208080,
  "source": "gateway",
  "target": "web_console",
  "data": {
    "success": true,
    "command_type": "stop_recording",
    "accepted": true,
    "error_code": "",
    "error_message": "",
    "ack_message": "停止录包命令已受理"
  }
}
```

## 6. 实时状态推送消息

## 6.1 导航主状态变更

```json
{
  "protocol_version": "2.0",
  "type": "navigation_state_changed",
  "request_id": "",
  "timestamp": 1781395210000,
  "source": "gateway",
  "target": "web_console",
  "data": {
    "robot_id": "XR-102",
    "route_task_id": "NAV-20240614-001",
    "navigation_main_state": "obstacle_paused",
    "reason_code": "dynamic_obstacle_blocked",
    "reason_text": "检测到前方动态障碍物，已自动暂停导航",
    "can_start": false,
    "can_pause": false,
    "can_resume": true,
    "can_stop": true,
    "can_jump": false,
    "started_at": 1781395201000,
    "paused_at": 1781395210000,
    "resumed_at": null,
    "finished_at": null
  }
}
```

前端解析规则：

1. 这是页面主状态的单一事实来源。
2. 顶部条、按钮区、异常卡片都要跟它联动。

## 6.2 路线任务进度变更

```json
{
  "protocol_version": "2.0",
  "type": "route_task_progress_changed",
  "request_id": "",
  "timestamp": 1781395210100,
  "source": "gateway",
  "target": "web_console",
  "data": {
    "robot_id": "XR-102",
    "route_task_id": "NAV-20240614-001",
    "route_name": "展厅上午导览",
    "current_waypoint_id": "13",
    "current_waypoint_name": "任务点13",
    "next_waypoint_id": "14",
    "next_waypoint_name": "辅助点14",
    "jump_target_waypoint_id": "16",
    "jump_target_waypoint_name": "任务点16",
    "progress_percent": 62.5,
    "sequence": [
      {
        "waypoint_id": "10",
        "waypoint_name": "任务点10",
        "waypoint_role": "task",
        "walk_direction": "forward",
        "need_broadcast": true,
        "status": "completed"
      },
      {
        "waypoint_id": "11",
        "waypoint_name": "辅助点11",
        "waypoint_role": "transit",
        "walk_direction": "forward",
        "need_broadcast": false,
        "status": "completed"
      },
      {
        "waypoint_id": "12",
        "waypoint_name": "辅助点12",
        "waypoint_role": "transit",
        "walk_direction": "forward",
        "need_broadcast": false,
        "status": "completed"
      },
      {
        "waypoint_id": "13",
        "waypoint_name": "任务点13",
        "waypoint_role": "task",
        "walk_direction": "forward",
        "need_broadcast": true,
        "status": "running"
      },
      {
        "waypoint_id": "14",
        "waypoint_name": "辅助点14",
        "waypoint_role": "transit",
        "walk_direction": "forward",
        "need_broadcast": false,
        "status": "pending"
      },
      {
        "waypoint_id": "16",
        "waypoint_name": "任务点16",
        "waypoint_role": "task",
        "walk_direction": "forward",
        "need_broadcast": true,
        "status": "pending"
      }
    ]
  }
}
```

## 6.3 机器人实时状态变更

```json
{
  "protocol_version": "2.0",
  "type": "robot_realtime_state_changed",
  "request_id": "",
  "timestamp": 1781395210200,
  "source": "gateway",
  "target": "web_console",
  "data": {
    "robot_id": "XR-102",
    "battery_percent": 61,
    "velocity": {
      "linear_mps": 0.00,
      "angular_radps": 0.00
    },
    "pose": {
      "x": 21.34,
      "y": 16.26,
      "yaw_deg": 132.1
    },
    "network_latency_ms": 56,
    "localization_status": "low_confidence",
    "localization_score": 0.32,
    "behavior_mode": "navigation"
  }
}
```

## 6.4 系统健康状态变更

```json
{
  "protocol_version": "2.0",
  "type": "system_health_changed",
  "request_id": "",
  "timestamp": 1781395210300,
  "source": "gateway",
  "target": "web_console",
  "data": {
    "robot_id": "XR-102",
    "health": {
      "nav_module": "ok",
      "localization_module": "warn",
      "obstacle_module": "ok",
      "power_module": "ok",
      "comm_module": "ok"
    }
  }
}
```

## 6.5 播报状态变更

```json
{
  "protocol_version": "2.0",
  "type": "broadcast_state_changed",
  "request_id": "",
  "timestamp": 1781395210400,
  "source": "gateway",
  "target": "web_console",
  "data": {
    "robot_id": "XR-102",
    "route_task_id": "NAV-20240614-001",
    "waiting": true,
    "broadcast_id": "broadcast_13",
    "broadcast_text": "欢迎来到展厅13号点",
    "waiting_since": 1781395210400,
    "last_finished_at": 1781395199000
  }
}
```

前端解析规则：

1. `waiting=true` 时，页面可显示“等待 APP 播报完成”。
2. 这不是异常态，除非同时收到异常导航主状态。

## 6.6 障碍物 / 人工脱困状态变更

```json
{
  "protocol_version": "2.0",
  "type": "obstacle_state_changed",
  "request_id": "",
  "timestamp": 1781395210500,
  "source": "gateway",
  "target": "web_console",
  "data": {
    "robot_id": "XR-102",
    "blocked": true,
    "reason": "manual_escape",
    "blocked_since": 1781395210000,
    "snapshot_url": "https://example.com/robot/obstacle_snapshot_20260614_01.jpg",
    "guidance_text": "请确认机器人周边无碰撞风险后，再点击继续"
  }
}
```

## 6.7 地图切换状态变更

```json
{
  "protocol_version": "2.0",
  "type": "map_switch_state_changed",
  "request_id": "",
  "timestamp": 1781395210600,
  "source": "gateway",
  "target": "web_console",
  "data": {
    "robot_id": "XR-102",
    "current_map_id": "hall_a_floor_1",
    "current_map_name": "A栋一层导览图",
    "target_map_id": "hall_b_floor_1",
    "target_map_name": "B栋一层实验室图",
    "switching": true,
    "phase": "restarting_localization",
    "phase_text": "正在重启定位链路",
    "ready_for_navigation": false
  }
}
```

后续切图完成示例：

```json
{
  "protocol_version": "2.0",
  "type": "map_switch_state_changed",
  "request_id": "",
  "timestamp": 1781395211800,
  "source": "gateway",
  "target": "web_console",
  "data": {
    "robot_id": "XR-102",
    "current_map_id": "hall_b_floor_1",
    "current_map_name": "B栋一层实验室图",
    "target_map_id": "hall_b_floor_1",
    "target_map_name": "B栋一层实验室图",
    "switching": false,
    "phase": "completed",
    "phase_text": "地图切换完成，定位已稳定",
    "ready_for_navigation": true
  }
}
```

## 6.8 录包状态变更

```json
{
  "protocol_version": "2.0",
  "type": "recording_state_changed",
  "request_id": "",
  "timestamp": 1781395210700,
  "source": "gateway",
  "target": "web_console",
  "data": {
    "robot_id": "XR-102",
    "recording": true,
    "started_at": 1781395210700,
    "save_path": "/home/ubuntu/bags/nav_debug_20260614_01",
    "file_name": "nav_debug_20260614_01"
  }
}
```

## 6.9 事件日志新增

```json
{
  "protocol_version": "2.0",
  "type": "event_log_added",
  "request_id": "",
  "timestamp": 1781395210800,
  "source": "gateway",
  "target": "web_console",
  "data": {
    "robot_id": "XR-102",
    "event": {
      "id": "log_20260614_000101",
      "timestamp": 1781395210800,
      "level": "warn",
      "category": "obstacle",
      "title": "检测到障碍物，导航已自动暂停",
      "detail": "机器人在前往任务点13途中检测到动态障碍物，已进入自动暂停状态"
    }
  }
}
```

## 7. 错误消息

## 7.1 通用命令失败 ACK

```json
{
  "protocol_version": "2.0",
  "type": "command_ack",
  "request_id": "req_20260614_200006",
  "timestamp": 1781395212000,
  "source": "gateway",
  "target": "web_console",
  "data": {
    "success": false,
    "command_type": "switch_map",
    "accepted": false,
    "error_code": "MAP_SWITCH_DENIED_TASK_RUNNING",
    "error_message": "当前仍有路线任务执行中，禁止切换地图",
    "ack_message": ""
  }
}
```

前端处理规则：

1. 弹错误提示。
2. 不切换页面主状态。
3. 结束按钮 loading。

## 7.2 主动错误推送

```json
{
  "protocol_version": "2.0",
  "type": "error_event",
  "request_id": "",
  "timestamp": 1781395212100,
  "source": "gateway",
  "target": "web_console",
  "data": {
    "robot_id": "XR-102",
    "error_code": "LOCALIZATION_RECOVERY_FAILED",
    "error_message": "地图切换后定位恢复失败，请人工确认环境或重新切图",
    "error_level": "error",
    "related_command_type": "switch_map"
  }
}
```

## 8. 推荐错误码

第一版建议先统一以下错误码：

1. `ROBOT_OFFLINE`
2. `WS_NOT_CONNECTED`
3. `ROUTE_TASK_NOT_FOUND`
4. `ROUTE_TASK_ALREADY_RUNNING`
5. `ROUTE_TASK_NOT_RUNNING`
6. `ROUTE_TASK_CANNOT_JUMP_NOW`
7. `TARGET_WAYPOINT_NOT_FOUND`
8. `TARGET_WAYPOINT_INVALID`
9. `MAP_NOT_FOUND`
10. `MAP_SWITCH_DENIED_TASK_RUNNING`
11. `MAP_SWITCH_FAILED`
12. `LOCALIZATION_RECOVERY_FAILED`
13. `RECORDING_ALREADY_RUNNING`
14. `RECORDING_NOT_RUNNING`
15. `INVALID_REQUEST`
16. `INTERNAL_SERVER_ERROR`

## 9. 前端页面状态映射建议

根据消息类型，建议前端这样更新 store：

1. `get_robot_snapshot_result`
   初始化 `robot`、`navigation`、`map`

2. `get_route_task_snapshot_result`
   初始化 `routeTask`

3. `navigation_state_changed`
   更新 `navigation`

4. `route_task_progress_changed`
   更新 `routeTask`

5. `robot_realtime_state_changed`
   更新 `robot`

6. `broadcast_state_changed`
   更新 `broadcast`

7. `obstacle_state_changed`
   更新 `obstacle`

8. `map_switch_state_changed`
   更新 `map`

9. `recording_state_changed`
   更新 `recording`

10. `event_log_added`
    追加 `logs`

## 10. 第一版最小联调闭环

如果只为 MVP 先打通，建议至少验证以下闭环：

1. 页面初始化：
   `get_robot_snapshot`
   `get_map_list`
   `get_route_task_snapshot`

2. 开始任务：
   `start_route_task`
   `command_ack`
   `navigation_state_changed`
   `route_task_progress_changed`

3. 手动暂停：
   `pause_route_task`
   `command_ack`
   `navigation_state_changed`

4. 继续任务：
   `resume_route_task`
   `command_ack`
   `navigation_state_changed`

5. 跳点：
   `jump_route_waypoint`
   `command_ack`
   `route_task_progress_changed`

6. 障碍物暂停：
   `obstacle_state_changed`
   `navigation_state_changed`

7. 地图切换：
   `switch_map`
   `command_ack`
   `map_switch_state_changed`

8. 录包：
   `start_recording`
   `command_ack`
   `recording_state_changed`

## 11. 当前建议

这份清单适合做第一版联调口径文档，但如果你们准备直接开后端和前端施工，建议下一步再补两样：

1. 一份 `JSON 字段字典`
2. 一份 `前端 mock 示例全集`

这样前端就能直接把 mock 文件和 adapter 一次性写出来。
