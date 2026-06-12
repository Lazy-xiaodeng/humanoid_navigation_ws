# APP侧路线任务_跳步_辅助点_倒走_障碍暂停恢复开发文档

> 本文档面向 APP 前端、APP 后端、ROS 联调同学。
> 目标是让 APP 侧按 ROS 当前新协议完成路线任务、任意跳步、辅助点无痕通过、倒走点、到点播报、障碍暂停恢复、人工脱困后继续等功能。

## 1. 总体口径

1. 新导航控制链路只使用 route task 新协议，不再使用旧的单点、多点、展台、旧暂停、旧继续、旧终止命令。
2. APP 启动路线时一次性下发完整 `route_waypoints`，ROS 按数组顺序执行，不按数字大小重新排序。
3. 点位分两类：`task` 是任务点，会停车并最终对齐角度；`transit` 是辅助点/途径点，只用于规划通过，不停车、不对齐、不播报。
4. 任务点可以不播报：`waypoint_role="task"`、`stop_and_align=true`、`need_broadcast=false`，机器人到点对齐后自动继续下一段。
5. 倒走点不是单独命令，而是点位属性：`walk_direction="backward"`。ROS 会在最终 task 收尾段使用倒走行为树。
6. 跳步只能跳到 `task` 点，不能跳到 `transit` 点。
7. 从 A 跳到 D 后，D 完成后继续走 D 后面的任务点；从 F 跳到 B 后，B 完成后继续走 B 后面的任务点。
8. 跳步会按原路线顺序吸收当前锚点到目标任务点之间的 `transit` 点。例如当前锚点 7，跳到 15，若 11、12 是 transit，则执行段为 `11 -> 12 -> 15`。
9. 障碍物暂停后，ROS 会先停车并推送障碍事件；障碍自动消失时 ROS 可自动恢复，人工清障/人工脱困后 APP 也可以发送 `resume_route_task` 继续。
10. WebSocket 即时 `command_ack` 只表示服务端收到/转发命令，不表示导航业务成功。
11. APP 判断命令业务成功或失败，统一看 `data_type="navigation_status"` 且 `data.event_type="navigation_command_result"`。
12. ROS 主动事件也都走 `data_type="navigation_status"`，真正事件类型看 `data.event_type`。

## 2. APP前端改造

### 2.1 点位设置页面

前端保存点位时，除原有坐标和姿态外，需要新增这些可编辑属性：

| 字段 | UI 控件建议 | 必填 | 说明 |
| --- | --- | --- | --- |
| `waypoint_role` | 单选：任务点/辅助点 | 是 | `task` 表示停车对齐的任务点；`transit` 表示无痕通过的辅助点 |
| `need_broadcast` | 开关 | task 点必填 | 是否到点后等待 APP 播报；transit 固定为 `false` |
| `broadcast_id` | 文本/选择播报词 | need_broadcast=true 时必填 | APP 播报资源 ID，ROS 只做上下文校验，不播放音频 |
| `broadcast_text` | 文本 | 可选 | 给前端/后端展示和播报使用，ROS 不依赖该字段 |
| `broadcast_blocking` | 开关 | 可选 | 建议 task 播报点为 `true`，表示 ROS 等待 `broadcast_finished` |
| `stop_and_align` | 开关 | task 点建议固定 true | task 点到达后最终对齐；transit 固定为 `false` |
| `walk_direction` | 单选：正走/倒走 | 可选 | `forward` 或 `backward`，只建议 task 点使用 |
| `route_order` | 排序序号 | 是 | APP 路线中的执行顺序，后端最终按 UI 顺序生成 `route_waypoints` |
| `frame_id` | 默认 map | 是 | 当前系统建议固定 `map` |
| `position` | 坐标 | 是 | `[x, y, z]` |
| `orientation` | 四元数 | 是 | `[x, y, z, w]`，不要只传 yaw |

前端交互建议：

1. 任务点默认：`waypoint_role=task`、`stop_and_align=true`、`walk_direction=forward`。
2. 辅助点默认：`waypoint_role=transit`、`need_broadcast=false`、`broadcast_id=""`、`stop_and_align=false`。
3. 倒走点只能选择为任务点，建议前端禁止 `waypoint_role=transit` 且 `walk_direction=backward` 的组合。
4. “任务点但不播报”允许存在，用于只停车转向不讲解的点。
5. 若客户把过门前的对齐点设置成 `transit`，ROS 不保证到该点时角度一定对齐；如果必须强制对齐，应设置成 `task + need_broadcast=false`。

### 2.2 路线执行页面

前端需要新增/调整这些按钮和状态：

| 功能 | 按钮/状态 | 对应命令 |
| --- | --- | --- |
| 开始路线任务 | 开始导航 | `start_route_task` |
| 手动暂停 | 暂停 | `pause_route_task` |
| 手动继续 | 继续 | `resume_route_task` |
| 障碍/人工脱困后继续 | 继续/已清障继续 | `resume_route_task` |
| 终止任务 | 终止 | `stop_route_task` |
| 任意跳步 | 跳到某任务点 | `jump_to_waypoint` |
| 播报完成 | 播报播放完毕后自动发送 | `broadcast_finished` |

前端 UI 规则：

1. `transit` 点可以展示在路线进度中，但不能作为跳步目标。
2. 收到 `waypoint_passed` 时，只刷新辅助点已通过状态，不弹播报，不显示“到点等待”。
3. 收到 `broadcast_requested` 时，前端/后端开始播报，并在播报完成后发送 `broadcast_finished`。
4. 收到 `task_waypoint_completed` 时，标记对应 task 完成。
5. 收到 `jump_updated` 时，更新当前执行段、高亮新目标、清除旧播报等待态。
6. 收到 `navigation_paused` 且 `pause_source="obstacle_wait"` 时，显示“障碍物阻塞，等待恢复/请清障”。
7. 障碍等待期间收到周期 `navigation_obstacle_blocked` 时，只刷新等待时长，不重复弹窗。
8. 收到 `navigation_resumed` 时，清除暂停/障碍提示。
9. 收到 `route_task_completed` 时，显示整条路线完成。
10. 收到 `navigation_failed` 时，显示失败并保留现场信息，便于导出日志。

## 3. APP后端改造

### 3.1 后端职责

APP 后端负责把前端点位和路线配置转换成 ROS 可识别的协议：

1. 保存点位库：维护点位坐标、姿态、任务/辅助属性、播报属性、倒走属性。
2. 生成路线任务：按前端路线顺序组装完整 `route_waypoints`。
3. 生成唯一 ID：每次命令带唯一 `message_id`，每条路线任务带唯一 `task_session_id`。
4. 透传到 ROS：WebSocket 外层 `message_id` 要复制到内层 `request_message_id`。
5. 解析 ROS 推送：所有导航业务事件从 `data_type="navigation_status"`、`data.event_type` 分发。
6. 管理播报闭环：收到 `broadcast_requested` 后开始播报，播报完成后发送 `broadcast_finished`。
7. 管理障碍恢复：收到障碍暂停后显示状态；人工清障后发送 `resume_route_task`。

### 3.2 点位库和路线任务的关系

点位库命令 `set_waypoint/update_waypoint/delete_waypoint/get_waypoints/clear_waypoints` 只负责保存基础点位。

真正开始导航时，后端必须把路线中每个点完整展开为 `route_waypoints`。ROS 状态机不会再根据旧单点/多点命令去查点位库补数据。

## 4. APP到ROS命令格式

以下示例使用 JSONC，带 `//` 中文注释，便于开发同学理解。实际 WebSocket 发送时如果使用严格 JSON，需要去掉注释。

### 4.1 统一外层格式

```jsonc
{
  "protocol_version": "2.0",                 // 协议版本，固定使用 2.0
  "message_id": "cmd_20260613_000001",       // APP 生成的唯一消息 ID，用于匹配后续 navigation_command_result
  "timestamp": 1781280000.123,               // APP 发包时间戳，单位秒
  "message_type": "command",                 // command 表示 APP 主动下发命令
  "data_type": "navigation_control",         // 路线任务控制固定为 navigation_control
  "source": "app",                           // 发送端，固定 app
  "destination": "ros",                      // 接收端，固定 ros
  "data": {
    "command_type": "start_route_task",      // 具体命令类型，见下方各命令
    "command_data": {}                       // 具体命令参数，见下方各命令
  },
  "metadata": {
    "operator_id": "user_001",               // 可选，操作人 ID，便于审计
    "client_id": "pad_001",                  // 可选，APP 设备 ID
    "request_source": "route_page"           // 可选，触发页面或业务来源
  }
}
```

### 4.2 开始路线任务 start_route_task

```jsonc
{
  "protocol_version": "2.0",                    // 协议版本
  "message_id": "cmd_start_route_0001",         // 本次开始命令唯一 ID
  "timestamp": 1781280000.123,                  // APP 发包时间
  "message_type": "command",                    // 固定 command
  "data_type": "navigation_control",            // 固定 navigation_control
  "source": "app",                              // 固定 app
  "destination": "ros",                         // 固定 ros
  "data": {
    "command_type": "start_route_task",         // 开始路线任务
    "command_data": {
      "request_message_id": "cmd_start_route_0001", // 必填，复制外层 message_id
      "task_session_id": "session_20260613_001",    // 必填，本次路线任务会话 ID，整条路线期间保持不变
      "route_id": "route_exhibition_001",           // 必填，路线 ID，后续 pause/resume/stop/jump/broadcast_finished 都要一致
      "route_name": "展厅讲解路线",                  // 可选，路线名称，仅展示和日志使用
      "route_waypoints": [                           // 必填，完整路线点位数组，ROS 按数组顺序执行
        {
          "waypoint_id": "10",                       // 必填，点位 ID，建议字符串
          "waypoint_name": "展厅任务点10",             // 可选，点位名称，用于 UI 和日志
          "waypoint_role": "task",                   // 必填，task=任务点；transit=辅助点
          "frame_id": "map",                         // 必填，当前系统建议固定 map
          "position": [1.0, 2.0, 0.0],                // 必填，x/y/z 坐标，单位米
          "orientation": [0.0, 0.0, 0.0, 1.0],        // 必填，四元数 x/y/z/w，ROS 会校验并归一化
          "need_broadcast": true,                    // task 点是否需要 APP 播报
          "broadcast_id": "broadcast_10",             // need_broadcast=true 时必填
          "broadcast_text": "欢迎来到展厅10号点",       // 可选，APP 播放文本，ROS 不依赖
          "broadcast_blocking": true,                // 建议 true，表示 ROS 等待 broadcast_finished 后继续
          "stop_and_align": true,                    // task 点固定 true，到点后最终对齐角度
          "walk_direction": "forward",               // forward=正走；backward=倒走
          "properties": {                            // 可选，冗余保存，便于兼容点位库
            "waypoint_role": "task",                 // 同 waypoint_role
            "walk_direction": "forward"              // 同 walk_direction
          }
        },
        {
          "waypoint_id": "11",                       // 辅助点 ID
          "waypoint_name": "过门辅助点11",             // 辅助点名称
          "waypoint_role": "transit",                // transit 表示无痕通过
          "frame_id": "map",                         // 坐标系
          "position": [2.0, 2.0, 0.0],                // 坐标
          "orientation": [0.0, 0.0, 0.707, 0.707],    // 姿态可传，但 transit 不做最终强制对齐
          "need_broadcast": false,                   // transit 固定 false
          "broadcast_id": "",                        // transit 固定空字符串
          "broadcast_text": "",                      // transit 不播报
          "broadcast_blocking": false,               // transit 固定 false
          "stop_and_align": false,                   // transit 固定 false，不停车不 spin
          "walk_direction": "forward",               // transit 建议 forward
          "properties": {
            "waypoint_role": "transit",              // 冗余保存辅助点属性
            "walk_direction": "forward"              // 冗余保存行走方向
          }
        },
        {
          "waypoint_id": "14",                       // 倒走任务点
          "waypoint_name": "倒走展示点14",             // 名称
          "waypoint_role": "task",                   // 倒走点仍然必须是 task
          "frame_id": "map",                         // 坐标系
          "position": [4.0, 2.0, 0.0],                // 坐标
          "orientation": [0.0, 0.0, 1.0, 0.0],        // 最终目标姿态
          "need_broadcast": false,                   // 可以不播报
          "broadcast_id": "",                        // 不播报时为空
          "broadcast_text": "",                      // 不播报时为空
          "broadcast_blocking": false,               // 不播报时 false
          "stop_and_align": true,                    // task 点仍要最终对齐
          "walk_direction": "backward",              // backward 表示最终 task 收尾段使用倒走行为树
          "properties": {
            "waypoint_role": "task",                 // 冗余保存任务点属性
            "walk_direction": "backward"             // 冗余保存倒走属性
          }
        }
      ]
    }
  },
  "metadata": {
    "operator_id": "user_001",                      // 操作人
    "client_id": "pad_001"                          // APP 设备
  }
}
```

前置条件：

1. 当前没有正在运行的 route task。
2. 当前导航不处于其它忙碌状态。
3. `route_waypoints` 至少包含一个 `task` 点。
4. `waypoint_id` 不能重复。
5. `task` 点如果 `need_broadcast=true`，必须提供非空 `broadcast_id`。

成功判断：

1. 先可能收到 WebSocket `command_ack`，只表示收到/转发。
2. 必须等待 `navigation_command_result.command_type="start_route_task"` 且 `status="success"`。
3. 后续是否到点、是否播报、是否完成，继续看其它导航事件。

### 4.3 暂停路线任务 pause_route_task

```jsonc
{
  "protocol_version": "2.0",                      // 协议版本
  "message_id": "cmd_pause_route_0001",           // 暂停命令唯一 ID
  "timestamp": 1781280100.123,                    // 发包时间
  "message_type": "command",                      // 固定 command
  "data_type": "navigation_control",              // 固定 navigation_control
  "source": "app",                                // 固定 app
  "destination": "ros",                           // 固定 ros
  "data": {
    "command_type": "pause_route_task",           // 暂停路线任务
    "command_data": {
      "request_message_id": "cmd_pause_route_0001", // 必填，复制外层 message_id
      "task_session_id": "session_20260613_001",    // 必填，当前任务会话 ID
      "route_id": "route_exhibition_001",           // 必填，当前路线 ID
      "reason": "user_manual_pause",                // 可选，暂停原因，建议枚举：user_manual_pause
      "pause_parameters": {
        "pause_duration": 0                          // 可选，0 表示不限时，手动继续
      }
    }
  },
  "metadata": {
    "operator_id": "user_001",                    // 操作人
    "client_id": "pad_001"                        // APP 设备
  }
}
```

成功后 ROS 会取消当前 route goal、停车、保留 route task 上下文，并推送 `navigation_paused`。

### 4.4 继续路线任务 resume_route_task

该命令同时用于三种场景：

1. APP 手动暂停后的继续。
2. 障碍物等待期间，人工清障后点击继续。
3. 人工移动机器人脱困后点击继续。

```jsonc
{
  "protocol_version": "2.0",                        // 协议版本
  "message_id": "cmd_resume_route_0001",            // 继续命令唯一 ID
  "timestamp": 1781280200.123,                      // 发包时间
  "message_type": "command",                        // 固定 command
  "data_type": "navigation_control",                // 固定 navigation_control
  "source": "app",                                  // 固定 app
  "destination": "ros",                             // 固定 ros
  "data": {
    "command_type": "resume_route_task",            // 继续路线任务
    "command_data": {
      "request_message_id": "cmd_resume_route_0001", // 必填，复制外层 message_id
      "task_session_id": "session_20260613_001",     // 必填，当前任务会话 ID
      "route_id": "route_exhibition_001",            // 必填，当前路线 ID
      "reason": "manual_obstacle_cleared",           // 可选，继续原因：user_manual_resume/manual_obstacle_cleared/manual_escape_done
      "resume_source": "app_manual",                 // 可选，建议 app_manual
      "manual_clear_obstacle": true,                 // 可选，人工清障/人工脱困后建议 true
      "operator_confirmed_safe": true                // 可选，现场确认安全后再继续
    }
  },
  "metadata": {
    "operator_id": "user_001",                      // 操作人
    "client_id": "pad_001"                          // APP 设备
  }
}
```

注意：

1. 如果当前暂停在导航段中，ROS 会重启当前 active segment。
2. 如果当前暂停在等待播报阶段，ROS 不会重新导航，只恢复等待 `broadcast_finished`。
3. 如果定位未满足启动条件，ROS 会返回 `localization_resume_blocked`。

### 4.5 终止路线任务 stop_route_task

```jsonc
{
  "protocol_version": "2.0",                       // 协议版本
  "message_id": "cmd_stop_route_0001",             // 终止命令唯一 ID
  "timestamp": 1781280300.123,                     // 发包时间
  "message_type": "command",                       // 固定 command
  "data_type": "navigation_control",               // 固定 navigation_control
  "source": "app",                                 // 固定 app
  "destination": "ros",                            // 固定 ros
  "data": {
    "command_type": "stop_route_task",             // 终止路线任务
    "command_data": {
      "request_message_id": "cmd_stop_route_0001", // 必填，复制外层 message_id
      "task_session_id": "session_20260613_001",   // 必填，当前任务会话 ID
      "route_id": "route_exhibition_001",          // 必填，当前路线 ID
      "reason": "user_manual_stop",                // 可选，终止原因
      "stop_parameters": {
        "clear_task_context": true                  // 可选，建议 true，终止后清理上下文
      }
    }
  },
  "metadata": {
    "operator_id": "user_001",                     // 操作人
    "client_id": "pad_001"                         // APP 设备
  }
}
```

终止后 ROS 会清理当前 route task 上下文。APP 后续不能再对旧 session 发送跳步、播报完成或继续。

### 4.6 任意跳步 jump_to_waypoint

```jsonc
{
  "protocol_version": "2.0",                         // 协议版本
  "message_id": "cmd_jump_route_0001",               // 跳步命令唯一 ID
  "timestamp": 1781280400.123,                       // 发包时间
  "message_type": "command",                         // 固定 command
  "data_type": "navigation_control",                 // 固定 navigation_control
  "source": "app",                                   // 固定 app
  "destination": "ros",                              // 固定 ros
  "data": {
    "command_type": "jump_to_waypoint",              // 任意跳到某个任务点
    "command_data": {
      "request_message_id": "cmd_jump_route_0001",   // 必填，复制外层 message_id
      "task_session_id": "session_20260613_001",     // 必填，当前任务会话 ID
      "route_id": "route_exhibition_001",            // 必填，当前路线 ID
      "target_waypoint_id": "15",                    // 必填，目标 task 点 ID，不能是 transit
      "interrupt_broadcast": true,                   // 建议固定 true，等待播报时允许打断并跳走
      "reason": "user_jump_to_waypoint"              // 可选，跳步原因
    }
  },
  "metadata": {
    "operator_id": "user_001",                       // 操作人
    "client_id": "pad_001"                           // APP 设备
  }
}
```

跳步规则：

1. 目标点必须是 `waypoint_role="task"`。
2. 正向跳和反向跳都支持，例如 7 跳 15、15 跳 3。
3. ROS 会根据当前进度锚点和目标点，在原路线顺序中吸收中间 transit。
4. 跳步成功后，APP 会收到 `jump_updated`，其中 `execution_waypoint_ids` 是新执行段。
5. 如果正在等待某点播报，且 `interrupt_broadcast=true`，ROS 会打断旧播报等待；APP 应停止旧播报 UI。

### 4.7 播报完成 broadcast_finished

```jsonc
{
  "protocol_version": "2.0",                            // 协议版本
  "message_id": "cmd_broadcast_finished_0001",          // 播报完成命令唯一 ID
  "timestamp": 1781280500.123,                          // 发包时间
  "message_type": "command",                            // 固定 command
  "data_type": "navigation_control",                    // 固定 navigation_control
  "source": "app",                                      // 固定 app
  "destination": "ros",                                 // 固定 ros
  "data": {
    "command_type": "broadcast_finished",               // 告诉 ROS 当前播报已完成
    "command_data": {
      "request_message_id": "cmd_broadcast_finished_0001", // 必填，复制外层 message_id
      "task_session_id": "session_20260613_001",           // 必填，当前任务会话 ID
      "route_id": "route_exhibition_001",                  // 必填，当前路线 ID
      "waypoint_id": "10",                                 // 必填，必须等于 broadcast_requested 的 waypoint_id
      "broadcast_id": "broadcast_10",                      // 必填，必须等于 broadcast_requested 的 broadcast_id
      "broadcast_result": "completed",                     // 必填/可省略，当前只支持 completed
      "finished_at": 1781280500.123                        // 可选，APP 播报完成时间
    }
  },
  "metadata": {
    "operator_id": "user_001",                            // 操作人或系统播放模块
    "client_id": "pad_001"                                // APP 设备
  }
}
```

注意：

1. 只有收到 `broadcast_requested` 后，APP 才能发送对应的 `broadcast_finished`。
2. `waypoint_id`、`broadcast_id`、`task_session_id`、`route_id` 必须和 ROS 等待上下文一致。
3. 重复发送同一条 `broadcast_finished`，ROS 会返回 success，`result_reason="duplicate_broadcast_finished"`，APP 不需要报错。
4. `broadcast_result` 目前只支持 `completed`。

## 5. 点位管理命令

点位管理命令仍走 `data_type="waypoint_management"`，用于保存/更新点位库。后端启动导航时再从点位库组装完整 `route_waypoints`。

### 5.1 保存点位 set_waypoint

```jsonc
{
  "protocol_version": "2.0",                   // 协议版本
  "message_id": "cmd_set_waypoint_0001",       // 点位保存命令 ID
  "timestamp": 1781280600.123,                 // 发包时间
  "message_type": "command",                   // 固定 command
  "data_type": "waypoint_management",          // 点位管理固定 waypoint_management
  "source": "app",                             // 固定 app
  "destination": "ros",                        // 固定 ros
  "data": {
    "command_type": "set_waypoint",            // 新增或覆盖保存点位
    "command_data": {
      "waypoint_data": {
        "id": "11",                            // 必填，点位 ID
        "name": "过门辅助点11",                  // 必填/建议，点位名称
        "type": "navigation_target",           // 必填，当前点位类型建议统一 navigation_target
        "frame_id": "map",                     // 坐标系
        "position": [2.0, 2.0, 0.0],            // 坐标
        "orientation": [0.0, 0.0, 0.707, 0.707],// 姿态
        "properties": {
          "waypoint_role": "transit",          // 新增属性：task 或 transit
          "need_broadcast": false,             // 新增属性：是否播报
          "broadcast_id": "",                  // 新增属性：播报 ID
          "broadcast_text": "",                // 新增属性：播报文本
          "broadcast_blocking": false,         // 新增属性：是否等待播报完成
          "stop_and_align": false,             // 新增属性：是否停车对齐
          "walk_direction": "forward",         // 新增属性：forward 或 backward
          "route_order": 11,                   // 可选，前端路线排序用
          "speed": 0.5                         // 可选，点位速度属性，必须在 ROS 允许范围内
        }
      }
    }
  },
  "metadata": {
    "operator_id": "user_001",                 // 操作人
    "client_id": "pad_001"                     // APP 设备
  }
}
```

### 5.2 更新/删除/查询点位

`update_waypoint` 的 `waypoint_data` 格式与 `set_waypoint` 一致。

```jsonc
{
  "protocol_version": "2.0",                    // 协议版本
  "message_id": "cmd_update_waypoint_0001",     // 更新命令 ID
  "timestamp": 1781280610.123,                  // 发包时间
  "message_type": "command",                    // 固定 command
  "data_type": "waypoint_management",           // 点位管理
  "source": "app",                              // 固定 app
  "destination": "ros",                         // 固定 ros
  "data": {
    "command_type": "update_waypoint",          // 更新点位
    "command_data": {
      "waypoint_data": {
        "id": "11",                             // 必填，要更新的点位 ID
        "type": "navigation_target",            // 必填，点位类型
        "name": "过门辅助点11-更新",              // 可更新字段
        "frame_id": "map",                      // 可更新字段
        "position": [2.1, 2.0, 0.0],             // 可更新字段
        "orientation": [0.0, 0.0, 0.707, 0.707], // 可更新字段
        "properties": {
          "waypoint_role": "transit",           // 可更新字段
          "need_broadcast": false,              // 可更新字段
          "broadcast_id": "",                   // 可更新字段
          "broadcast_text": "",                 // 可更新字段
          "broadcast_blocking": false,          // 可更新字段
          "stop_and_align": false,              // 可更新字段
          "walk_direction": "forward"           // 可更新字段
        }
      }
    }
  },
  "metadata": {
    "operator_id": "user_001",                  // 操作人
    "client_id": "pad_001"                      // APP 设备
  }
}
```

```jsonc
{
  "protocol_version": "2.0",                 // 协议版本
  "message_id": "cmd_delete_waypoint_0001",  // 删除命令 ID
  "timestamp": 1781280620.123,               // 发包时间
  "message_type": "command",                 // 固定 command
  "data_type": "waypoint_management",        // 点位管理
  "source": "app",                           // 固定 app
  "destination": "ros",                      // 固定 ros
  "data": {
    "command_type": "delete_waypoint",       // 删除点位
    "command_data": {
      "waypoint_id": "11",                   // 必填，点位 ID
      "waypoint_type": "navigation_target"   // 必填，点位类型
    }
  },
  "metadata": {
    "operator_id": "user_001",               // 操作人
    "client_id": "pad_001"                   // APP 设备
  }
}
```

```jsonc
{
  "protocol_version": "2.0",                  // 协议版本
  "message_id": "cmd_get_waypoints_0001",     // 查询命令 ID
  "timestamp": 1781280630.123,                // 发包时间
  "message_type": "command",                  // 固定 command
  "data_type": "waypoint_management",         // 点位管理
  "source": "app",                            // 固定 app
  "destination": "ros",                       // 固定 ros
  "data": {
    "command_type": "get_waypoints",          // 查询点位
    "command_data": {
      "waypoint_type": "navigation_target",   // 可选，navigation_target 或 all
      "include_details": true                 // true 返回完整点位；false 只返回 ID 列表
    }
  },
  "metadata": {
    "operator_id": "user_001",                // 操作人
    "client_id": "pad_001"                    // APP 设备
  }
}
```

## 6. ROS到APP主动推送

### 6.1 统一推送外层

```jsonc
{
  "protocol_version": "2.0",                    // 协议版本
  "message_id": "push_1781280700000",           // ROS/数据集成层生成的推送 ID
  "timestamp": 1781280700.123,                  // 推送时间
  "message_type": "push",                       // push 表示 ROS 主动推送
  "data_type": "navigation_status",             // 导航相关事件固定 navigation_status
  "source": "data_integration",                 // 发送端，数据集成层
  "destination": "all",                         // 推送给所有 APP 客户端
  "data": {
    "event_type": "navigation_command_result",  // 真正事件类型，APP 按这个字段分发
    "event_data": {}                            // 事件详细数据，见下方各事件
  },
  "metadata": {
    "status": "success",                        // success 或 error
    "push_reason": "navigation_event",          // navigation_event 表示导航离散事件
    "qos_level": "realtime",                    // realtime 表示立即推送
    "error_code": "",                           // error 时填错误码
    "error_message": ""                         // error 时填错误信息
  }
}
```

### 6.2 命令业务结果 navigation_command_result

```jsonc
{
  "event_type": "navigation_command_result",       // 固定，表示某条 APP 命令的业务结果
  "event_data": {
    "request_message_id": "cmd_start_route_0001",  // 对应 APP 命令内层 request_message_id
    "ack_type": "navigation_command_result",       // 固定业务 ack 类型
    "command_type": "start_route_task",            // 原始命令类型
    "task_session_id": "session_20260613_001",     // 原始命令携带的会话 ID，缺失时为空
    "route_id": "route_exhibition_001",            // 原始命令携带的路线 ID，缺失时为空
    "status": "success",                           // success=业务接受；error=业务拒绝
    "result_reason": "",                           // success 的补充原因，例如 first_task_already_reached
    "error_code": "",                              // error 时的机器可读错误码
    "message": "route task accepted and first segment started", // 人类可读消息
    "event_id": "route_task_session_20260613_001_navigation_command_result_1_1781280700123", // 去重/追踪 ID
    "timestamp": 1781280700.123                     // 事件时间
  }
}
```

APP 处理：

1. 用 `request_message_id` 匹配原始按钮操作。
2. `status="success"` 后再更新按钮状态。
3. `status="error"` 后按 `error_code` 弹提示，不要继续等待该命令成功。

### 6.3 最终对齐事件 final_align_started / final_align_completed

```jsonc
{
  "event_type": "final_align_started",             // 开始最终 task 对齐
  "event_data": {
    "task_session_id": "session_20260613_001",     // 当前任务会话
    "route_id": "route_exhibition_001",            // 当前路线
    "segment_id": "seg_10_to_13_1",                // 当前执行段 ID
    "waypoint_id": "13",                           // 正在最终对齐的 task 点
    "walk_direction": "forward",                   // forward 或 backward
    "behavior_tree": "navigate_w_replanning_and_spin.xml", // 实际使用的行为树
    "align_reason": "final_task_pose",             // 对齐原因
    "event_id": "route_task_session_20260613_001_final_align_started_1_1781280800123", // 事件 ID
    "timestamp": 1781280800.123                    // 事件时间
  }
}
```

```jsonc
{
  "event_type": "final_align_completed",           // 最终 task 对齐完成
  "event_data": {
    "task_session_id": "session_20260613_001",     // 当前任务会话
    "route_id": "route_exhibition_001",            // 当前路线
    "segment_id": "seg_10_to_13_1",                // 当前执行段 ID
    "waypoint_id": "13",                           // 已完成对齐的 task 点
    "event_id": "route_task_session_20260613_001_final_align_completed_1_1781280810123", // 事件 ID
    "timestamp": 1781280810.123                    // 事件时间
  }
}
```

APP 处理：这两个事件主要用于状态展示和日志；真正是否播报看后续 `broadcast_requested`。

### 6.4 辅助点通过 waypoint_passed

```jsonc
{
  "event_type": "waypoint_passed",                 // 辅助点已通过
  "event_data": {
    "task_session_id": "session_20260613_001",     // 当前任务会话
    "route_id": "route_exhibition_001",            // 当前路线
    "segment_id": "seg_10_to_13_1",                // 当前执行段 ID
    "waypoint_id": "11",                           // 已通过的 transit 点 ID
    "waypoint_role": "transit",                    // 固定 transit
    "passed_transit_waypoint_ids": ["11"],         // 当前段已通过的 transit 列表
    "current_target_task_id": "13",                // 当前最终目标 task
    "event_id": "route_task_session_20260613_001_waypoint_passed_1_1781280750123", // 事件 ID
    "timestamp": 1781280750.123                    // 事件时间
  }
}
```

APP 处理：只更新路线进度，不播报、不弹到点确认、不允许停留。

### 6.5 请求播报 broadcast_requested

```jsonc
{
  "event_type": "broadcast_requested",             // ROS 请求 APP 播报
  "event_data": {
    "task_session_id": "session_20260613_001",     // 当前任务会话
    "route_id": "route_exhibition_001",            // 当前路线
    "segment_id": "seg_10_to_13_1",                // 当前执行段 ID
    "waypoint_id": "13",                           // 需要播报的 task 点
    "broadcast_id": "broadcast_13",                // APP 要播放的播报资源 ID
    "current_target_task_id": "13",                // 当前目标 task
    "event_id": "route_task_session_20260613_001_broadcast_requested_1_1781280820123", // 事件 ID
    "timestamp": 1781280820.123                    // 事件时间
  }
}
```

APP 处理：

1. 按 `broadcast_id` 播放音频/文字。
2. 播放完后发送 `broadcast_finished`。
3. 如果用户在播报中跳步，APP 应停止旧播报，并等待新 `jump_updated` / 新 `broadcast_requested`。

### 6.6 任务点完成 task_waypoint_completed

```jsonc
{
  "event_type": "task_waypoint_completed",         // 单个 task 点完成
  "event_data": {
    "task_session_id": "session_20260613_001",     // 当前任务会话
    "route_id": "route_exhibition_001",            // 当前路线
    "segment_id": "seg_10_to_13_1",                // 当前执行段 ID
    "waypoint_id": "13",                           // 已完成的 task 点
    "completed_task_ids": ["10", "13"],            // 已完成 task 列表
    "skipped_task_ids": [],                        // 被跳过或打断未完成的 task 列表
    "next_target_task_id": "14",                   // 下一个目标 task，末尾可为空
    "event_id": "route_task_session_20260613_001_task_waypoint_completed_1_1781280830123", // 事件 ID
    "timestamp": 1781280830.123                    // 事件时间
  }
}
```

APP 处理：标记 task 完成；如果 `next_target_task_id` 非空，继续展示下一目标。

### 6.7 跳步更新 jump_updated

```jsonc
{
  "event_type": "jump_updated",                    // 跳步已接受并完成新段重建
  "event_data": {
    "task_session_id": "session_20260613_001",     // 当前任务会话
    "route_id": "route_exhibition_001",            // 当前路线
    "segment_id": "seg_7_to_15_2",                 // 新执行段 ID
    "target_waypoint_id": "15",                    // 新目标 task 点
    "segment_direction": "forward",                // forward=正向；backward=反向
    "execution_waypoint_ids": ["11", "12", "15"],  // 本段实际要经过的点，包含 transit 和最终 task
    "skipped_task_ids": ["8", "9", "10"],          // 被跳过或打断未完成的 task
    "interrupt_broadcast": true,                   // 是否打断旧播报等待
    "interrupted_broadcast": {                     // 如果跳步打断了等待播报，这里非空
      "waypoint_id": "10",                         // 被打断播报的点
      "broadcast_id": "broadcast_10"               // 被打断播报 ID
    },
    "event_id": "route_task_session_20260613_001_jump_updated_1_1781280840123", // 事件 ID
    "timestamp": 1781280840.123                    // 事件时间
  }
}
```

APP 处理：切换当前目标和路线高亮；若 `interrupted_broadcast` 非空，停止旧播报 UI。

### 6.8 整条路线完成 route_task_completed

```jsonc
{
  "event_type": "route_task_completed",            // 整条路线任务完成
  "event_data": {
    "task_session_id": "session_20260613_001",     // 当前任务会话
    "route_id": "route_exhibition_001",            // 当前路线
    "completed_waypoint_id": "25",                 // 最后完成的 task 点
    "completed_task_ids": ["1", "2", "10", "13", "25"], // 已完成 task 列表
    "skipped_task_ids": ["3", "4"],                // 被跳过 task 列表
    "completed_at": 1781280900.123,                // 完成时间
    "result": "success",                           // 当前固定 success
    "summary": {
      "task_count": 10,                            // 路线中的 task 总数
      "completed_count": 8,                        // 已完成 task 数
      "skipped_count": 2                           // 跳过 task 数
    },
    "event_id": "route_task_session_20260613_001_route_task_completed_1_1781280900123", // 事件 ID
    "timestamp": 1781280900.123                    // 事件时间
  }
}
```

APP 处理：结束路线任务 UI，允许重新开始新路线。

### 6.9 导航失败 navigation_failed

```jsonc
{
  "event_type": "navigation_failed",               // 导航失败事件
  "event_data": {
    "route_task": true,                            // true 表示路线任务失败
    "task_session_id": "session_20260613_001",     // 当前任务会话
    "route_id": "route_exhibition_001",            // 当前路线
    "reason": "NavigateThroughPoses feedback timeout", // 失败原因文本
    "failure_code": "feedback_timeout",            // 机器可读失败码
    "current_target_task_id": "15",                // 失败时目标 task
    "segment_id": "seg_7_to_15_2",                 // 失败执行段
    "segment_direction": "forward",                // 执行方向
    "execution_waypoint_ids": ["11", "12", "15"],  // 失败段点位列表
    "passed_transit_waypoint_ids": ["11"],         // 已通过 transit
    "completed_task_ids": ["1", "2", "7"],         // 已完成 task
    "skipped_task_ids": ["8", "9", "10"],          // 已跳过 task
    "failed_at": 1781280950.123,                   // 失败时间
    "event_id": "route_task_session_20260613_001_navigation_failed_1_1781280950123", // 事件 ID
    "timestamp": 1781280950.123                    // 事件时间
  }
}
```

APP 处理：进入失败态，提示用户人工处理；不要自动重发旧命令。

### 6.10 障碍暂停 navigation_paused

```jsonc
{
  "event_type": "navigation_paused",               // 导航暂停事件
  "event_data": {
    "pause_source": "obstacle_wait",               // obstacle_wait=障碍暂停；route_task_user_request=APP 手动暂停
    "reason": "检测到障碍物，前方路径被挡住",         // 暂停原因
    "resume_mode": "auto",                         // auto=ROS 可自动恢复；manual=需要 APP resume_route_task
    "waiting_for_obstacle_clear": true,            // 是否正在等待障碍清除
    "pause_location": {
      "x": 1.23,                                   // 暂停时机器人 x
      "y": 2.34,                                   // 暂停时机器人 y
      "z": 0.0                                     // 暂停时机器人 z
    },
    "pause_time": 1781281000.123,                  // 暂停开始时间
    "pause_duration": 0,                           // 0 表示不限时
    "current_waypoint_id": "15",                   // 当前底层目标点，可能为空
    "current_waypoint_name": "实验室任务点15",       // 当前底层目标名称，可能为空
    "waypoint_index": 3,                           // 当前点索引，兼容字段
    "total_waypoints": 25,                         // 总点数，兼容字段
    "block_duration": 5.2,                         // 障碍持续阻塞时长
    "clear_confirmed_frames": 0,                   // 已连续确认 clear 帧数
    "clear_required_frames": 3,                    // 需要连续 clear 帧数
    "task_session_id": "session_20260613_001",     // route task 场景可能携带
    "route_id": "route_exhibition_001",            // route task 场景可能携带
    "current_target_task_id": "15",                // route task 当前目标 task
    "segment_id": "seg_7_to_15_2",                 // route task 当前段
    "route_task": true                             // true 表示当前是路线任务
  }
}
```

APP 处理：

1. `pause_source="obstacle_wait"`：显示障碍等待 UI。
2. `resume_mode="auto"`：可以提示“障碍消失后自动恢复”，同时保留“人工清障后继续”按钮。
3. `pause_source="route_task_user_request"`：显示手动暂停 UI，需要用户点继续。

### 6.11 障碍持续阻塞 navigation_obstacle_blocked

```jsonc
{
  "event_type": "navigation_obstacle_blocked",      // 障碍仍在阻塞
  "event_data": {
    "reason": "检测到障碍物，前方路径被挡住",          // 原因
    "block_duration": 12.4,                         // 已阻塞时长
    "blocked_waypoint_id": "15",                    // 被阻塞目标点
    "blocked_waypoint_name": "实验室任务点15",        // 被阻塞目标点名称
    "blocked_waypoint_index": 3,                    // 被阻塞索引
    "total_waypoints": 25,                          // 总点数
    "position": [4.0, 2.0, 0.0],                    // 目标点位置
    "waiting_for_obstacle_clear": true,             // 固定 true
    "clear_confirmed_frames": 0,                    // 已连续 clear 帧数
    "clear_required_frames": 3,                     // 需要连续 clear 帧数
    "pause_source": "obstacle_wait",                // 固定 obstacle_wait
    "front_obstacle_stats": {                       // 可选，前方障碍统计
      "min_distance": 0.42,                         // 最近障碍距离
      "blocked_cells": 18                           // 阻塞栅格数量
    }
  }
}
```

APP 处理：周期刷新提示，不重复弹窗。

### 6.12 导航恢复 navigation_resumed

```jsonc
{
  "event_type": "navigation_resumed",              // 导航恢复事件
  "event_data": {
    "task_session_id": "session_20260613_001",     // route task 会话 ID，route task 场景有
    "route_id": "route_exhibition_001",            // route task 路线 ID，route task 场景有
    "route_task": true,                            // 是否路线任务
    "current_target_task_id": "15",                // 当前目标 task
    "segment_id": "seg_7_to_15_2",                 // 当前执行段
    "pause_duration_actual": 8.5,                  // 实际暂停时长
    "resume_reason": "manual_obstacle_cleared",    // 恢复原因
    "resume_source": "obstacle_wait",              // obstacle_wait 或 route_task_user_request
    "awaiting_broadcast": false,                   // 是否恢复到等待播报阶段
    "waiting_broadcast_waypoint_id": "",           // 如果正在等待播报，这里是点位 ID
    "waiting_broadcast_id": ""                     // 如果正在等待播报，这里是播报 ID
  }
}
```

APP 处理：清除暂停/障碍 UI；如果 `awaiting_broadcast=true`，继续完成播报闭环，不要等待机器人移动。

### 6.13 导航终止 navigation_stopped

```jsonc
{
  "event_type": "navigation_stopped",              // 导航终止事件
  "event_data": {
    "task_session_id": "session_20260613_001",     // 被终止任务会话
    "route_id": "route_exhibition_001",            // 被终止路线
    "route_task": true,                            // true 表示路线任务
    "reason": "user_manual_stop",                  // 终止原因
    "stopped_at": 1781281100.123                   // 终止时间
  }
}
```

APP 处理：结束当前路线 UI，清理所有等待中的播报、跳步、暂停状态。

## 7. WebSocket即时ACK示例

WebSocket 层可能先返回类似下面的即时确认：

```jsonc
{
  "protocol_version": "2.0",                    // 协议版本
  "message_id": "ack_cmd_start_route_0001",     // WebSocket 层 ack ID
  "timestamp": 1781280000.223,                  // ack 时间
  "message_type": "response",                   // response 表示对收到包的响应
  "data_type": "command_ack",                   // command_ack 只表示收到/转发
  "source": "websocket_server",                 // WebSocket 服务端
  "destination": "app",                         // 返回给 APP
  "data": {
    "status": "forwarded",                      // forwarded 表示已转发到 ROS topic
    "command_type": "navigation_control",       // 被转发的数据类型
    "message": "command received and forwarded" // 可读消息
  },
  "metadata": {
    "request_id": "cmd_start_route_0001",        // 对应 APP 外层 message_id
    "status": "success",                        // WebSocket 层状态
    "error_code": "",                           // WebSocket 层错误码
    "error_message": ""                         // WebSocket 层错误消息
  }
}
```

重要：APP 不能把 `command_ack` 当作路线任务开始成功。真正业务结果必须等 `navigation_command_result`。

## 8. 错误码和APP建议文案

| error_code | 含义 | APP建议 |
| --- | --- | --- |
| `missing_task_session_id` | 缺少任务会话 ID | 后端生成并补齐 `task_session_id` |
| `missing_route_id` | 缺少路线 ID | 后端补齐 `route_id` |
| `invalid_route_waypoints` | 路线点数组非法 | 检查 `route_waypoints` 是否数组且字段完整 |
| `duplicate_waypoint_id` | 路线中点位 ID 重复 | 提示配置重复点位 |
| `invalid_waypoint_role` | 点位角色非法 | 只能使用 `task/transit` |
| `missing_waypoint_pose` | 点位缺少 position/orientation | 提示重新保存点位姿态 |
| `missing_broadcast_id` | 需要播报但播报 ID 为空 | 前端必须选择播报资源 |
| `missing_task_waypoints` | 路线没有 task 点 | 至少配置一个任务点 |
| `route_task_already_running` | 已有路线任务运行中 | 禁止重复开始 |
| `navigation_busy` | 导航系统忙 | 等当前导航结束或终止 |
| `route_task_not_running` | 当前无路线任务 | 刷新 UI，禁止继续控制旧任务 |
| `invalid_task_session` | 任务会话不匹配 | APP 刷新当前任务状态 |
| `invalid_route_id` | 路线 ID 不匹配 | APP 刷新当前路线状态 |
| `invalid_target_waypoint` | 跳步目标不存在 | 禁用不存在点位 |
| `target_waypoint_not_task` | 跳步目标不是任务点 | 禁止选择 transit 作为跳步目标 |
| `interrupt_broadcast_false_not_supported` | 等待播报时不允许不打断跳步 | 跳步固定发送 `interrupt_broadcast=true` |
| `broadcast_not_waiting` | ROS 当前不在等待播报 | 不要重复发送播报完成 |
| `broadcast_context_mismatch` | 播报点位或播报 ID 不匹配 | 使用 `broadcast_requested` 中的 ID 回传 |
| `unsupported_broadcast_result` | 播报结果不支持 | 只发送 `completed` |
| `invalid_route_task_state` | 当前状态不允许该控制 | 刷新 UI 状态 |
| `route_task_not_paused` | 当前不是暂停态，不能继续 | 禁用继续按钮 |
| `localization_resume_blocked` | 定位未满足恢复条件 | 提示等待定位稳定 |
| `missing_active_segment` | 当前执行段丢失 | 提示终止并重新开始 |
| `send_goal_failed` | Nav2 goal 发送失败 | 提示导航启动失败，导出日志 |
| `feedback_timeout` | Nav2 反馈超时 | 提示导航卡住，人工检查 |
| `goal_rejected` | Nav2 拒绝目标 | 检查点位和地图 |
| `goal_canceled` | goal 被取消 | 若非用户操作，导出日志 |
| `goal_failed` | goal 执行失败 | 检查路径/障碍/定位 |
| `final_align_start_failed` | 最终对齐启动失败 | 检查行为树和目标姿态 |
| `final_pose_goal_failed` | 最终 task 收尾失败 | 检查目标点附近空间 |
| `obstacle_auto_resume_failed` | 障碍自动恢复失败 | APP 提示人工处理 |

## 9. APP状态机建议

推荐 APP 后端维护这些状态：

| 状态 | 进入条件 | 退出条件 |
| --- | --- | --- |
| `idle` | 无任务 | 发送 start 后进入 `starting` |
| `starting` | 已发 start，等待业务 ack | `navigation_command_result success/error` |
| `running` | 路线任务运行中 | pause/obstacle/broadcast/failed/completed |
| `waiting_broadcast` | 收到 `broadcast_requested` | 发送并收到 `broadcast_finished success` 后等待 task 完成事件 |
| `paused_user` | 收到手动暂停成功 | resume 成功 |
| `paused_obstacle` | 收到障碍暂停 | 自动 `navigation_resumed` 或手动 `resume_route_task` 成功 |
| `jumping` | 已发 jump，等待业务 ack/jump_updated | jump success 且收到 jump_updated |
| `failed` | 收到 `navigation_failed` 或 command error | 用户终止/重新开始 |
| `completed` | 收到 `route_task_completed` | 用户确认后回 idle |
| `stopped` | 收到 stop success 或 navigation_stopped | 回 idle |

## 10. 验收用例

1. 顺序路线 1-25，11、12、19-25 为 transit，14-16 为 backward task，其它为 forward task。
2. 顺序执行时，transit 只产生 `waypoint_passed`，不产生 `broadcast_requested`。
3. 顺序执行时，task 点产生 `final_align_started/final_align_completed`。
4. task 点 `need_broadcast=true` 时，ROS 推 `broadcast_requested`，APP 播报完成后发 `broadcast_finished`，再继续。
5. task 点 `need_broadcast=false` 时，ROS 对齐后自动 `task_waypoint_completed` 并继续。
6. 7 跳 15 时，如果 11、12 是 transit，`jump_updated.execution_waypoint_ids=["11","12","15"]`。
7. 15 跳 3 时，支持反向跳，执行段按原路线反向吸收 transit。
8. 跳到 transit 点时，ROS 返回 `target_waypoint_not_task`。
9. 等待播报时跳步，`interrupt_broadcast=true` 后 ROS 打断旧播报等待并推 `jump_updated.interrupted_broadcast`。
10. 障碍暂停后，APP 收到 `navigation_paused(pause_source=obstacle_wait)` 和周期 `navigation_obstacle_blocked`。
11. 障碍自动清除时，APP 收到 `navigation_resumed`。
12. 人工清障/脱困后点击继续，APP 发送 `resume_route_task`，ROS 返回 `navigation_command_result success` 并恢复当前段。
13. 手动暂停后继续，当前段恢复，不丢失跳步、transit、倒走、播报上下文。
14. 终止后旧 session 的 jump/broadcast/resume 都返回 `route_task_not_running` 或 session 不匹配。

## 11. 开发收口提醒

1. APP 前端不再展示旧单点/多点/展台导航入口作为新路线任务控制按钮。
2. APP 后端不要把 `command_ack` 当业务成功。
3. APP 后端必须把外层 `message_id` 同步写到内层 `request_message_id`。
4. APP 后端必须保存当前 `task_session_id/route_id`，所有后续控制命令都要带。
5. APP 后端必须用 ROS `broadcast_requested` 中的 `waypoint_id/broadcast_id` 原样回传 `broadcast_finished`。
6. `transit` 点不能播报、不能最终对齐、不能作为跳步目标。
7. 需要停车但不播报的点，请配置为 `task + need_broadcast=false`，不要配置成 `transit`。
8. 过门等需要强制朝向的场景，若实测 transit 不够稳定，应改为 task 无播报点或后续增加局部约束方案。
