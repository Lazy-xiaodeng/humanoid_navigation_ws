# APP侧路线任务_任意跳步_播报协同开发文档

本文档面向 APP 前端和 APP 后端，描述“路线任务、任意跳步、辅助点无痕通过、到点播报协同”的开发口径。

本版本以 ROS 侧当前实现为准，APP 需要一次性下发完整路线，由 ROS 在路线任务内部自动执行、跳步、等待播报和继续推进。

## 1. 核心结论

1. APP 不再按旧逻辑“到一个点，等 APP 播报完，再下发下一个点”。
2. APP 启动路线任务时，一次性把完整 `route_waypoints` 下发给 ROS。
3. `route_waypoints` 的数组顺序就是路线顺序，ROS 不按点位编号重新排序。
4. 点位必须显式区分 `task` 和 `transit`。
5. `task` 是主任务点，可以停车、对齐、播报、计入完成进度。
6. `transit` 是辅助点/途径点，只用于通过窄门、门洞、狭窄区域等，不停车、不对齐、不播报、不计入主任务进度。
7. 任意跳步支持正向和反向，例如 `A -> D`、`F -> B`、`A -> G`、`G -> C`。
8. 跳步时 ROS 直接规划到目标任务点，但会自动吸收当前锚点和目标任务点之间的 `transit` 辅助点。
9. 跳步目标必须是 `task` 点，不能跳到 `transit` 点。
10. 从 `A` 跳到 `D`，`D` 完成后继续去 `E`。
11. 从 `F` 跳到 `B`，`B` 完成后继续去 `C`。
12. 到达需要播报的 `task` 点后，ROS 推送 `broadcast_requested`，APP 播报完成后回传 `broadcast_finished`，ROS 再继续下一段。
13. 跳步如果发生在等待播报期间，APP 默认发送 `interrupt_broadcast=true`，ROS 会退出旧播报等待并切到新目标。
14. WebSocket 的 `command_ack` 只表示“服务器收到并转发了命令”，不能当业务成功。
15. APP 判断业务成功或失败，必须看 `data_type="navigation_status"` 且 `data.event_type="navigation_command_result"`。

## 2. APP前端改造

### 2.1 点位设置页面

前端设置点位时，除了原有坐标和朝向，还必须支持以下业务属性。

必填字段：

1. `waypoint_id`：点位 ID，建议保存为字符串，例如 `"7"`、`"15"`、`"door_01"`。
2. `waypoint_name`：点位显示名称，例如 `"7号展项"`。
3. `frame_id`：坐标系，默认 `"map"`。
4. `position`：位置数组 `[x, y, z]`。
5. `orientation`：四元数数组 `[x, y, z, w]`。
6. `waypoint_role`：点位角色，只允许 `"task"` 或 `"transit"`。
7. `route_order`：路线顺序，前端保存路线时用于排序；最终下发 ROS 时以数组顺序为准。

任务点字段：

1. `need_broadcast`：是否需要播报。
2. `broadcast_id`：播报资源 ID；当 `need_broadcast=true` 时必填。
3. `broadcast_title`：播报标题，APP 展示和管理用，ROS 不依赖。
4. `broadcast_text`：播报文案，APP 播报系统用，ROS 不依赖。
5. `broadcast_audio_url`：播报音频地址，APP 播报系统用，ROS 不依赖。
6. `broadcast_blocking`：是否等待播报完成后再继续，任务点建议固定为 `true`。
7. `stop_and_align`：到任务点是否停车并对齐目标朝向，任务点建议固定为 `true`。

辅助点字段：

1. `need_broadcast`：必须保存为 `false`。
2. `broadcast_id`：必须保存为空字符串 `""`。
3. `broadcast_blocking`：必须保存为 `false`。
4. `stop_and_align`：必须保存为 `false`。
5. `transit_note`：辅助点用途说明，APP 管理端展示用，例如 `"过门前正对门洞"`。

前端点位编辑对象示例：

```json
{
  "waypoint_id": "11",
  "waypoint_name": "过门辅助点11",
  "frame_id": "map",
  "position": [3.2, 1.4, 0.0],
  "orientation": [0.0, 0.0, 0.3826834, 0.9238795],
  "waypoint_role": "transit",
  "route_order": 11,
  "need_broadcast": false,
  "broadcast_id": "",
  "broadcast_title": "",
  "broadcast_text": "",
  "broadcast_audio_url": "",
  "broadcast_blocking": false,
  "stop_and_align": false,
  "transit_note": "从展厅进入实验室前的过门辅助点",
  "enabled": true
}
```

任务点示例：

```json
{
  "waypoint_id": "15",
  "waypoint_name": "15号实验室讲解点",
  "frame_id": "map",
  "position": [6.8, 2.1, 0.0],
  "orientation": [0.0, 0.0, 0.7071068, 0.7071068],
  "waypoint_role": "task",
  "route_order": 15,
  "need_broadcast": true,
  "broadcast_id": "broadcast_point_15_intro",
  "broadcast_title": "实验室介绍",
  "broadcast_text": "这里是实验室展示区。",
  "broadcast_audio_url": "https://app.example.com/audio/broadcast_point_15_intro.mp3",
  "broadcast_blocking": true,
  "stop_and_align": true,
  "transit_note": "",
  "enabled": true
}
```

### 2.2 点位设置页面校验

保存点位或保存路线前，前端应做以下校验。

1. `waypoint_id` 不允许为空。
2. 同一条路线内 `waypoint_id` 不允许重复，包括 `task` 和 `transit`。
3. `position` 必须是 3 个数字。
4. `orientation` 必须是 4 个数字，不能全为 0。
5. `waypoint_role` 只能是 `"task"` 或 `"transit"`。
6. `task` 点允许设置 `need_broadcast`。
7. `task.need_broadcast=true` 时，`broadcast_id` 必须非空。
8. `transit` 点必须禁用播报配置 UI，并强制 `need_broadcast=false`。
9. `transit` 点必须禁用停车对齐 UI，并强制 `stop_and_align=false`。
10. 一条路线至少要有一个 `task` 点。

### 2.3 路线编辑页面

路线编辑页需要展示完整路线顺序，而不是只展示任务点。

推荐 UI：

1. 每一行显示点位名称、点位 ID、点位角色、是否播报、是否停车对齐。
2. `task` 用“任务点”标签。
3. `transit` 用“辅助点/途径点”标签。
4. `transit` 行用较弱颜色展示，但不能隐藏，否则客户无法确认过门辅助点是否在路线中。
5. 支持拖拽或上移下移调整路线顺序。
6. 保存路线时，后端必须按 UI 当前顺序持久化。

示例顺序：

```text
7 task  7号展项
11 transit 过门辅助点11
12 transit 过门辅助点12
15 task  15号实验室讲解点
16 task  16号结束点
```

当机器人当前锚点是 `7`，APP 跳到 `15` 时，ROS 会执行 `7 -> 11 -> 12 -> 15`。其中 `11`、`12` 只作为 through 途径点，无痕通过。

### 2.4 导航控制页面

前端导航控制页面至少需要以下区域。

1. 路线选择区：选择要执行的路线。
2. 路线预览区：展示完整 `task/transit` 顺序。
3. 启动按钮：发送 `start_route_task`。
4. 跳步按钮：路线运行中可用，打开任务点列表。
5. 当前目标展示：显示 `current_target_task_id` 和目标名称。
6. 辅助点进度展示：显示当前段 `execution_waypoint_ids` 和 `passed_transit_waypoint_ids`。
7. 播报状态展示：收到 `broadcast_requested` 后显示“等待播报/正在播报”。
8. 障碍物暂停提示：收到障碍物暂停状态后显示“前方障碍物，导航暂停等待”。
9. 完成状态展示：收到 `route_task_completed` 后显示路线完成。
10. 错误状态展示：收到 `navigation_command_result.status=error` 或 `navigation_failed` 后显示失败原因。

跳步按钮逻辑：

1. 只展示可跳转的 `task` 点。
2. `transit` 点不允许作为跳步目标。
3. 当前正在等待播报时，点击跳步仍允许。
4. 跳步命令默认带 `interrupt_broadcast=true`。
5. 如果本地播放器正在播旧点位音频，收到 `jump_updated.interrupted_broadcast` 非空后立刻停止旧播报 UI。

播报按钮逻辑：

1. APP 不应该主动判断机器人到点后是否播报。
2. APP 只在收到 ROS 的 `broadcast_requested` 后开始播报。
3. 播报完成后 APP 发送 `broadcast_finished`。
4. APP 不要在还没收到 `broadcast_requested` 时提前发送 `broadcast_finished`。
5. 播报失败时首版建议 APP 本地重试或提示人工处理，不要给 ROS 发送 `broadcast_result="failed"`，当前 ROS 会返回 `unsupported_broadcast_result`。

导航控制按钮口径：

1. 开始路线任务使用 `start_route_task`。
2. 任意跳步使用新增 `jump_to_waypoint`。
3. 播报完成使用新增 `broadcast_finished`。
4. 暂停路线任务使用新增 `pause_route_task`。
5. 继续路线任务使用新增 `resume_route_task`，尤其障碍物暂停后，APP 人工清障并点击继续时也使用该命令恢复当前 route task 段。
6. 终止路线任务使用新增 `stop_route_task`，ROS 会安全取消当前 through goal、发布停止状态并清理 route task 运行态。
7. 旧 `start_single_navigation`、`start_multi_point_navigation`、`start_exhibition_navigation`、`pause_navigation`、`resume_navigation`、`stop_navigation`、`cancel_navigation` 不再作为 APP 导航协议使用。
8. 如果 APP 误发旧命令，ROS 会按未知导航命令处理；APP 新版本不应再展示这些旧按钮或生成这些旧 command_type。

## 3. APP后端改造

### 3.1 后端数据模型

后端建议把点位基础信息和路线点位关系分开存储。

点位表建议字段：

1. `waypoint_id`
2. `waypoint_name`
3. `frame_id`
4. `position_x`
5. `position_y`
6. `position_z`
7. `orientation_x`
8. `orientation_y`
9. `orientation_z`
10. `orientation_w`
11. `waypoint_role`
12. `need_broadcast`
13. `broadcast_id`
14. `broadcast_title`
15. `broadcast_text`
16. `broadcast_audio_url`
17. `broadcast_blocking`
18. `stop_and_align`
19. `transit_note`
20. `enabled`

路线点位关系表建议字段：

1. `route_id`
2. `waypoint_id`
3. `route_order`
4. `enabled`

后端组包时，必须按 `route_order` 排序生成 `route_waypoints` 数组。ROS 只认数组顺序，不会按点位编号重新排序。

### 3.2 后端生成 task_session_id

每次点击“开始路线任务”时，后端生成新的 `task_session_id`。

推荐格式：

```text
route_session_20260612_001
```

规则：

1. 一次路线执行一个 `task_session_id`。
2. 同一条路线重新开始，也要生成新的 `task_session_id`。
3. 后续 `jump_to_waypoint` 和 `broadcast_finished` 必须携带同一个 `task_session_id`。

### 3.3 后端组装 route_waypoints

后端从数据库读取路线后，转换为 ROS 需要的数组格式。

转换规则：

1. `waypoint_id` 转字符串，并去掉首尾空格。
2. `frame_id` 为空时填 `"map"`。
3. `position` 转成 `[x, y, z]` 数组。
4. `orientation` 转成 `[x, y, z, w]` 数组。
5. `waypoint_role` 只允许 `"task"` 或 `"transit"`。
6. `task` 保留 `need_broadcast`、`broadcast_id`、`broadcast_blocking`、`stop_and_align`。
7. `task.need_broadcast=true` 时，`broadcast_id` 必须非空。
8. `transit` 强制 `need_broadcast=false`、`broadcast_id=""`、`broadcast_blocking=false`、`stop_and_align=false`。
9. 不要过滤掉 `transit`，否则过门辅助点会丢失。
10. 不要只下发目标点和终点，必须下发完整路线。

后端生成的单个 route waypoint 示例：

```json
{
  "waypoint_id": "12",
  "waypoint_name": "过门辅助点12",
  "frame_id": "map",
  "position": [4.1, 1.6, 0.0],
  "orientation": [0.0, 0.0, 0.3826834, 0.9238795],
  "waypoint_role": "transit",
  "need_broadcast": false,
  "broadcast_id": "",
  "broadcast_blocking": false,
  "stop_and_align": false,
  "properties": {
    "waypoint_name": "过门辅助点12",
    "route_order": 12,
    "transit_note": "过门后辅助点",
    "broadcast_title": "",
    "broadcast_text": "",
    "broadcast_audio_url": ""
  }
}
```

### 3.4 后端发送 ROS 命令的统一外层

APP 后端发给 ROS WebSocket 的消息必须包含以下外层字段。

```json
{
  "protocol_version": "2.0",
  "message_id": "app_cmd_20260612_100000_001",
  "timestamp": 1781260000.000,
  "message_type": "command",
  "data_type": "navigation_control",
  "source": "app_backend",
  "destination": "websocket_server",
  "data": {}
}
```

字段说明：

1. `protocol_version` 固定使用 `"2.0"`。
2. `message_id` 必须每条命令唯一。
3. `message_id` 会被 ROS 透传为 `navigation_command_result.event_data.request_message_id`。
4. `message_type` 固定为 `"command"`。
5. `data_type` 固定为 `"navigation_control"`。
6. `data.command_type` 才是真正业务命令，例如 `start_route_task`、`jump_to_waypoint`、`broadcast_finished`。

## 4. APP发给ROS的完整JSON

### 4.1 订阅导航状态

APP 连接 WebSocket 后，建议订阅 `navigation_status`，否则可能收不到路线任务事件。

```json
{
  "protocol_version": "2.0",
  "message_id": "app_sub_20260612_095959_001",
  "timestamp": 1781259999.500,
  "message_type": "subscription",
  "data_type": "navigation_status",
  "source": "app_backend",
  "destination": "websocket_server",
  "data": {
    "action": "subscribe",
    "data_types": ["navigation_status"],
    "push_frequency": 1.0
  }
}
```

后端处理：

1. 建立 WebSocket 后立即发送。
2. 断线重连后重新发送。
3. 收到 `subscription_ack` 后只表示订阅请求已转发，不代表路线任务状态已经同步。

### 4.2 开始路线任务 start_route_task

```json
{
  "protocol_version": "2.0",
  "message_id": "app_cmd_20260612_100000_001",
  "timestamp": 1781260000.000,
  "message_type": "command",
  "data_type": "navigation_control",
  "source": "app_backend",
  "destination": "websocket_server",
  "data": {
    "command_type": "start_route_task",
    "task_session_id": "route_session_20260612_001",
    "route_id": "route_main_demo",
    "route_waypoints": [
      {
        "waypoint_id": "7",
        "waypoint_name": "7号展项",
        "frame_id": "map",
        "position": [1.0, 0.5, 0.0],
        "orientation": [0.0, 0.0, 0.0, 1.0],
        "waypoint_role": "task",
        "need_broadcast": true,
        "broadcast_id": "broadcast_point_7_intro",
        "broadcast_blocking": true,
        "stop_and_align": true,
        "properties": {
          "waypoint_name": "7号展项",
          "route_order": 7,
          "broadcast_title": "7号展项介绍",
          "broadcast_text": "这里是7号展项。",
          "broadcast_audio_url": "https://app.example.com/audio/broadcast_point_7_intro.mp3",
          "transit_note": ""
        }
      },
      {
        "waypoint_id": "11",
        "waypoint_name": "过门辅助点11",
        "frame_id": "map",
        "position": [3.2, 1.4, 0.0],
        "orientation": [0.0, 0.0, 0.3826834, 0.9238795],
        "waypoint_role": "transit",
        "need_broadcast": false,
        "broadcast_id": "",
        "broadcast_blocking": false,
        "stop_and_align": false,
        "properties": {
          "waypoint_name": "过门辅助点11",
          "route_order": 11,
          "broadcast_title": "",
          "broadcast_text": "",
          "broadcast_audio_url": "",
          "transit_note": "过门前辅助点"
        }
      },
      {
        "waypoint_id": "12",
        "waypoint_name": "过门辅助点12",
        "frame_id": "map",
        "position": [4.1, 1.6, 0.0],
        "orientation": [0.0, 0.0, 0.3826834, 0.9238795],
        "waypoint_role": "transit",
        "need_broadcast": false,
        "broadcast_id": "",
        "broadcast_blocking": false,
        "stop_and_align": false,
        "properties": {
          "waypoint_name": "过门辅助点12",
          "route_order": 12,
          "broadcast_title": "",
          "broadcast_text": "",
          "broadcast_audio_url": "",
          "transit_note": "过门后辅助点"
        }
      },
      {
        "waypoint_id": "15",
        "waypoint_name": "15号实验室讲解点",
        "frame_id": "map",
        "position": [6.8, 2.1, 0.0],
        "orientation": [0.0, 0.0, 0.7071068, 0.7071068],
        "waypoint_role": "task",
        "need_broadcast": true,
        "broadcast_id": "broadcast_point_15_intro",
        "broadcast_blocking": true,
        "stop_and_align": true,
        "properties": {
          "waypoint_name": "15号实验室讲解点",
          "route_order": 15,
          "broadcast_title": "实验室介绍",
          "broadcast_text": "这里是实验室展示区。",
          "broadcast_audio_url": "https://app.example.com/audio/broadcast_point_15_intro.mp3",
          "transit_note": ""
        }
      },
      {
        "waypoint_id": "16",
        "waypoint_name": "16号结束点",
        "frame_id": "map",
        "position": [7.5, 2.8, 0.0],
        "orientation": [0.0, 0.0, 1.0, 0.0],
        "waypoint_role": "task",
        "need_broadcast": false,
        "broadcast_id": "",
        "broadcast_blocking": true,
        "stop_and_align": true,
        "properties": {
          "waypoint_name": "16号结束点",
          "route_order": 16,
          "broadcast_title": "",
          "broadcast_text": "",
          "broadcast_audio_url": "",
          "transit_note": ""
        }
      }
    ],
    "request_message_id": "app_cmd_20260612_100000_001"
  }
}
```

后端处理：

1. 从前端路线 ID 查询完整路线。
2. 生成新的 `task_session_id`。
3. 按路线保存顺序生成 `route_waypoints`。
4. 把 `message_id` 同步写入 `data.request_message_id`，方便日志排查。
5. 发送后等待 `navigation_command_result`。
6. 收到 WebSocket `command_ack` 不能把 UI 改成“路线已开始”，只能显示“命令已发送”。
7. 收到 `navigation_command_result.status="success"` 后，才认为 ROS 业务层接受了本次启动。
8. 后续到点、播报、完成都继续等 `navigation_status` 事件。

### 4.3 正向跳步 jump_to_waypoint

当前路线执行中，从 `7` 跳到 `15`。

```json
{
  "protocol_version": "2.0",
  "message_id": "app_cmd_20260612_100030_002",
  "timestamp": 1781260030.000,
  "message_type": "command",
  "data_type": "navigation_control",
  "source": "app_backend",
  "destination": "websocket_server",
  "data": {
    "command_type": "jump_to_waypoint",
    "task_session_id": "route_session_20260612_001",
    "route_id": "route_main_demo",
    "target_waypoint_id": "15",
    "interrupt_broadcast": true,
    "reason": "operator_jump_forward",
    "request_message_id": "app_cmd_20260612_100030_002"
  }
}
```

预期语义：

1. 目标 `15` 必须是 `task`。
2. ROS 根据当前锚点和 `route_waypoints` 顺序吸收中间 `transit`。
3. 如果当前锚点是 `7`，新段执行列表是 `["11", "12", "15"]`。
4. `11`、`12` 不停车、不对齐、不播报。
5. `15` 完成后继续去 `16`。

### 4.4 反向跳步 jump_to_waypoint

当前路线执行中，从 `15` 或 `16` 附近跳回 `7`。

```json
{
  "protocol_version": "2.0",
  "message_id": "app_cmd_20260612_100200_003",
  "timestamp": 1781260200.000,
  "message_type": "command",
  "data_type": "navigation_control",
  "source": "app_backend",
  "destination": "websocket_server",
  "data": {
    "command_type": "jump_to_waypoint",
    "task_session_id": "route_session_20260612_001",
    "route_id": "route_main_demo",
    "target_waypoint_id": "7",
    "interrupt_broadcast": true,
    "reason": "operator_jump_backward",
    "request_message_id": "app_cmd_20260612_100200_003"
  }
}
```

预期语义：

1. ROS 支持反向跳步。
2. 如果当前锚点是 `15`，目标是 `7`，中间存在 `11`、`12`，新段会按反向顺序执行相关辅助点。
3. 跳回 `7` 后，`7` 完成再继续去路线中的下一个任务点。
4. 由于主任务顺序里 `7` 后面的任务点是 `15`，所以 `7` 完成后会继续去 `15`。

### 4.5 暂停路线任务 pause_route_task

```json
{
  "protocol_version": "2.0",
  "message_id": "app_cmd_20260612_100050_004",
  "timestamp": 1781260050.000,
  "message_type": "command",
  "data_type": "navigation_control",
  "source": "app_backend",
  "destination": "websocket_server",
  "data": {
    "command_type": "pause_route_task",
    "task_session_id": "route_session_20260612_001",
    "route_id": "route_main_demo",
    "pause_parameters": {
      "pause_duration": 0,
      "safe_pause_location": true
    },
    "reason": "operator_pause_route_task",
    "request_message_id": "app_cmd_20260612_100050_004"
  }
}
```

后端处理：

1. 必须携带当前运行中的 `task_session_id`。
2. 必须携带当前运行中的 `route_id`。
3. 发送后等待 `navigation_command_result.command_type="pause_route_task"`。
4. 收到成功 ack 后，前端进入路线任务暂停态。
5. ROS 会保留当前 `active_segment`、已完成 task、已跳过 task 和播报等待上下文。

### 4.6 继续路线任务 resume_route_task

APP 手动暂停后点击继续，或者障碍物暂停后人工清障并点击继续，都发送该命令。

```json
{
  "protocol_version": "2.0",
  "message_id": "app_cmd_20260612_100080_005",
  "timestamp": 1781260080.000,
  "message_type": "command",
  "data_type": "navigation_control",
  "source": "app_backend",
  "destination": "websocket_server",
  "data": {
    "command_type": "resume_route_task",
    "task_session_id": "route_session_20260612_001",
    "route_id": "route_main_demo",
    "reason": "operator_resume_after_obstacle_cleared",
    "request_message_id": "app_cmd_20260612_100080_005"
  }
}
```

后端处理：

1. 必须携带当前运行中的 `task_session_id`。
2. 必须携带当前运行中的 `route_id`。
3. 发送后等待 `navigation_command_result.command_type="resume_route_task"`。
4. 如果当前是障碍物等待态，ROS 会退出障碍等待并重新启动当前 through 段。
5. 如果当前是等待播报态，ROS 不重新发导航 goal，APP 继续完成当前播报闭环。

### 4.7 终止路线任务 stop_route_task

```json
{
  "protocol_version": "2.0",
  "message_id": "app_cmd_20260612_100090_006",
  "timestamp": 1781260090.000,
  "message_type": "command",
  "data_type": "navigation_control",
  "source": "app_backend",
  "destination": "websocket_server",
  "data": {
    "command_type": "stop_route_task",
    "task_session_id": "route_session_20260612_001",
    "route_id": "route_main_demo",
    "stop_parameters": {
      "emergency_stop": false,
      "preserve_state": false,
      "reason": "operator_stop_route_task"
    },
    "reason": "operator_stop_route_task",
    "request_message_id": "app_cmd_20260612_100090_006"
  }
}
```

后端处理：

1. 必须携带当前运行中的 `task_session_id`。
2. 必须携带当前运行中的 `route_id`。
3. 发送后等待 `navigation_command_result.command_type="stop_route_task"`。
4. 收到成功 ack 后，前端退出路线任务运行态。
5. 后续还会收到 `navigation_stopped` 状态事件，里面包含已完成 task、已跳过 task 和停止原因。

### 4.8 播报完成 broadcast_finished

APP 收到 `broadcast_requested` 并完成本地播报后发送。

```json
{
  "protocol_version": "2.0",
  "message_id": "app_cmd_20260612_100120_007",
  "timestamp": 1781260120.000,
  "message_type": "command",
  "data_type": "navigation_control",
  "source": "app_backend",
  "destination": "websocket_server",
  "data": {
    "command_type": "broadcast_finished",
    "task_session_id": "route_session_20260612_001",
    "route_id": "route_main_demo",
    "waypoint_id": "15",
    "broadcast_id": "broadcast_point_15_intro",
    "broadcast_result": "completed",
    "broadcast_duration_sec": 18.6,
    "reason": "audio_playback_completed",
    "request_message_id": "app_cmd_20260612_100120_007"
  }
}
```

后端处理：

1. `task_session_id` 必须来自当前运行路线任务。
2. `route_id` 必须来自当前运行路线。
3. `waypoint_id` 必须使用 `broadcast_requested.event_data.waypoint_id`。
4. `broadcast_id` 必须使用 `broadcast_requested.event_data.broadcast_id`。
5. `broadcast_result` 建议固定传 `"completed"`。
6. 如果省略 `broadcast_result`，ROS WebSocket 当前会按 `"completed"` 透传。
7. 如果显式传 `""`、`"failed"`、`"cancelled"`，ROS 会返回 `unsupported_broadcast_result`。
8. 发送后等待 `navigation_command_result.command_type="broadcast_finished"`。
9. 收到成功 ack 后，APP 可以把播报 UI 置为完成，但路线是否继续、任务点是否完成仍看后续 `task_waypoint_completed`。

## 5. ROS发给APP的完整JSON

ROS 侧经数据整合节点推送给 APP 时，顶层统一是 `data_type="navigation_status"`。APP 后端和前端都应该读取 `data.event_type` 判断业务事件。

### 5.1 命令业务结果 navigation_command_result

启动路线成功示例：

```json
{
  "protocol_version": "2.0",
  "message_id": "push_1781260000100_a1b2c3",
  "timestamp": 1781260000.100,
  "message_type": "push",
  "data_type": "navigation_status",
  "source": "data_integration",
  "destination": "all",
  "data": {
    "event_type": "navigation_command_result",
    "event_data": {
      "request_message_id": "app_cmd_20260612_100000_001",
      "ack_type": "navigation_command_result",
      "command_type": "start_route_task",
      "task_session_id": "route_session_20260612_001",
      "route_id": "route_main_demo",
      "status": "success",
      "result_reason": "route_task_started",
      "error_code": "",
      "message": "路线任务已启动",
      "event_id": "route_task_route_session_20260612_001_navigation_command_result_1_1781260000100",
      "timestamp": 1781260000.100
    },
    "timestamp": 1781260000.100,
    "current_state": "executing",
    "navigation_mode": "route_task",
    "sequence_id": 1,
    "current_pose": {},
    "current_path": {},
    "progress_percentage": 0.0,
    "estimated_remaining_time": 0.0,
    "system_timestamp": 1781260000.100,
    "performance_metrics": {}
  },
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

APP 解析：

1. 判断 `data.event_type=="navigation_command_result"`。
2. 用 `data.event_data.request_message_id` 对应原始按钮命令。
3. 用 `data.event_data.command_type` 判断是启动、跳步还是播报完成的 ack。
4. `status="success"` 表示命令业务层接受或完成同步处理。
5. `status="error"` 表示命令被拒绝，前端要解除按钮 loading 并展示错误。

错误示例：

```json
{
  "protocol_version": "2.0",
  "message_id": "push_1781260030100_d4e5f6",
  "timestamp": 1781260030.100,
  "message_type": "push",
  "data_type": "navigation_status",
  "source": "data_integration",
  "destination": "all",
  "data": {
    "event_type": "navigation_command_result",
    "event_data": {
      "request_message_id": "app_cmd_20260612_100030_002",
      "ack_type": "navigation_command_result",
      "command_type": "jump_to_waypoint",
      "task_session_id": "route_session_20260612_001",
      "route_id": "route_main_demo",
      "status": "error",
      "result_reason": "",
      "error_code": "invalid_target_waypoint",
      "message": "target waypoint 11 is not a task waypoint",
      "event_id": "route_task_route_session_20260612_001_navigation_command_result_2_1781260030100",
      "timestamp": 1781260030.100
    },
    "timestamp": 1781260030.100,
    "current_state": "executing",
    "navigation_mode": "route_task",
    "sequence_id": 1,
    "current_pose": {},
    "current_path": {},
    "progress_percentage": 20.0,
    "estimated_remaining_time": 60.0,
    "system_timestamp": 1781260030.100,
    "performance_metrics": {}
  },
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

### 5.2 经过辅助点 waypoint_passed

```json
{
  "protocol_version": "2.0",
  "message_id": "push_1781260050000_aabbcc",
  "timestamp": 1781260050.000,
  "message_type": "push",
  "data_type": "navigation_status",
  "source": "data_integration",
  "destination": "all",
  "data": {
    "event_type": "waypoint_passed",
    "event_data": {
      "segment_id": "segment_route_session_20260612_001_7_to_15_1",
      "waypoint_id": "11",
      "waypoint_role": "transit",
      "passed_transit_waypoint_ids": ["11"],
      "current_target_task_id": "15",
      "event_id": "route_task_route_session_20260612_001_waypoint_passed_3_1781260050000",
      "timestamp": 1781260050.000,
      "task_session_id": "route_session_20260612_001",
      "route_id": "route_main_demo"
    },
    "timestamp": 1781260050.000,
    "current_state": "executing",
    "navigation_mode": "route_task",
    "sequence_id": 1,
    "current_pose": {},
    "current_path": {},
    "progress_percentage": 30.0,
    "estimated_remaining_time": 45.0,
    "system_timestamp": 1781260050.000,
    "performance_metrics": {}
  },
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

APP 解析：

1. 只更新辅助点通过状态。
2. 不触发播报。
3. 不把该点计入任务点完成数。
4. 不弹出“到达点位”播报 UI。

### 5.3 请求播报 broadcast_requested

```json
{
  "protocol_version": "2.0",
  "message_id": "push_1781260100000_bbccdd",
  "timestamp": 1781260100.000,
  "message_type": "push",
  "data_type": "navigation_status",
  "source": "data_integration",
  "destination": "all",
  "data": {
    "event_type": "broadcast_requested",
    "event_data": {
      "segment_id": "segment_route_session_20260612_001_7_to_15_1",
      "waypoint_id": "15",
      "broadcast_id": "broadcast_point_15_intro",
      "current_target_task_id": "15",
      "event_id": "route_task_route_session_20260612_001_broadcast_requested_4_1781260100000",
      "timestamp": 1781260100.000,
      "task_session_id": "route_session_20260612_001",
      "route_id": "route_main_demo"
    },
    "timestamp": 1781260100.000,
    "current_state": "waiting_broadcast",
    "navigation_mode": "route_task",
    "sequence_id": 1,
    "current_pose": {},
    "current_path": {},
    "progress_percentage": 60.0,
    "estimated_remaining_time": 30.0,
    "system_timestamp": 1781260100.000,
    "performance_metrics": {}
  },
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

APP 解析：

1. 校验 `task_session_id` 和当前任务一致。
2. 校验 `route_id` 和当前路线一致。
3. 使用 `broadcast_id` 查找本地播报资源。
4. 前端进入“正在播报”状态。
5. 播放完成后发送 `broadcast_finished`。
6. 如果此时用户点击跳步，发送 `jump_to_waypoint` 且 `interrupt_broadcast=true`。

### 5.4 跳步段已更新 jump_updated

```json
{
  "protocol_version": "2.0",
  "message_id": "push_1781260030200_ccddee",
  "timestamp": 1781260030.200,
  "message_type": "push",
  "data_type": "navigation_status",
  "source": "data_integration",
  "destination": "all",
  "data": {
    "event_type": "jump_updated",
    "event_data": {
      "segment_id": "segment_route_session_20260612_001_7_to_15_2",
      "target_waypoint_id": "15",
      "segment_direction": "forward",
      "execution_waypoint_ids": ["11", "12", "15"],
      "skipped_task_ids": [],
      "interrupt_broadcast": true,
      "interrupted_broadcast": {
        "waypoint_id": "7",
        "broadcast_id": "broadcast_point_7_intro"
      },
      "event_id": "route_task_route_session_20260612_001_jump_updated_5_1781260030200",
      "timestamp": 1781260030.200,
      "task_session_id": "route_session_20260612_001",
      "route_id": "route_main_demo"
    },
    "timestamp": 1781260030.200,
    "current_state": "executing",
    "navigation_mode": "route_task",
    "sequence_id": 1,
    "current_pose": {},
    "current_path": {},
    "progress_percentage": 25.0,
    "estimated_remaining_time": 50.0,
    "system_timestamp": 1781260030.200,
    "performance_metrics": {}
  },
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

APP 解析：

1. 用 `target_waypoint_id` 刷新当前目标任务点。
2. 用 `segment_direction` 展示正向或反向跳步。
3. 用 `execution_waypoint_ids` 高亮本次实际执行段。
4. 用 `skipped_task_ids` 更新已跳过任务点。
5. 如果 `interrupted_broadcast.waypoint_id` 非空，停止旧播报 UI。
6. 收到 `jump_updated` 后可以认为 ROS 已经切到新 through 段。

无播报被打断时，`interrupted_broadcast` 仍然存在，但为空对象：

```json
{
  "protocol_version": "2.0",
  "message_id": "push_1781260200200_ddeeff",
  "timestamp": 1781260200.200,
  "message_type": "push",
  "data_type": "navigation_status",
  "source": "data_integration",
  "destination": "all",
  "data": {
    "event_type": "jump_updated",
    "event_data": {
      "segment_id": "segment_route_session_20260612_001_15_to_7_3",
      "target_waypoint_id": "7",
      "segment_direction": "backward",
      "execution_waypoint_ids": ["12", "11", "7"],
      "skipped_task_ids": ["15"],
      "interrupt_broadcast": true,
      "interrupted_broadcast": {},
      "event_id": "route_task_route_session_20260612_001_jump_updated_6_1781260200200",
      "timestamp": 1781260200.200,
      "task_session_id": "route_session_20260612_001",
      "route_id": "route_main_demo"
    },
    "timestamp": 1781260200.200,
    "current_state": "executing",
    "navigation_mode": "route_task",
    "sequence_id": 1,
    "current_pose": {},
    "current_path": {},
    "progress_percentage": 35.0,
    "estimated_remaining_time": 55.0,
    "system_timestamp": 1781260200.200,
    "performance_metrics": {}
  },
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

### 5.5 任务点完成 task_waypoint_completed

```json
{
  "protocol_version": "2.0",
  "message_id": "push_1781260121000_eeff00",
  "timestamp": 1781260121.000,
  "message_type": "push",
  "data_type": "navigation_status",
  "source": "data_integration",
  "destination": "all",
  "data": {
    "event_type": "task_waypoint_completed",
    "event_data": {
      "segment_id": "segment_route_session_20260612_001_7_to_15_1",
      "waypoint_id": "15",
      "completed_task_ids": ["7", "15"],
      "skipped_task_ids": [],
      "next_target_task_id": "16",
      "event_id": "route_task_route_session_20260612_001_task_waypoint_completed_7_1781260121000",
      "timestamp": 1781260121.000,
      "task_session_id": "route_session_20260612_001",
      "route_id": "route_main_demo"
    },
    "timestamp": 1781260121.000,
    "current_state": "executing",
    "navigation_mode": "route_task",
    "sequence_id": 1,
    "current_pose": {},
    "current_path": {},
    "progress_percentage": 75.0,
    "estimated_remaining_time": 20.0,
    "system_timestamp": 1781260121.000,
    "performance_metrics": {}
  },
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

APP 解析：

1. 把 `waypoint_id` 标记为任务点完成。
2. 更新 `completed_task_ids`。
3. 更新 `skipped_task_ids`。
4. 如果 `next_target_task_id` 非空，前端显示下一目标。
5. 如果 `next_target_task_id` 为空，等待 `route_task_completed`。

### 5.6 路线任务完成 route_task_completed

```json
{
  "protocol_version": "2.0",
  "message_id": "push_1781260180000_ff0011",
  "timestamp": 1781260180.000,
  "message_type": "push",
  "data_type": "navigation_status",
  "source": "data_integration",
  "destination": "all",
  "data": {
    "event_type": "route_task_completed",
    "event_data": {
      "task_session_id": "route_session_20260612_001",
      "route_id": "route_main_demo",
      "completed_waypoint_id": "16",
      "completed_task_ids": ["7", "15", "16"],
      "skipped_task_ids": [],
      "completed_at": 1781260180.000,
      "result": "success",
      "summary": {
        "task_count": 3,
        "completed_count": 3,
        "skipped_count": 0
      },
      "event_id": "route_task_route_session_20260612_001_route_task_completed_8_1781260180000",
      "timestamp": 1781260180.000
    },
    "timestamp": 1781260180.000,
    "current_state": "completed",
    "navigation_mode": "route_task",
    "sequence_id": 1,
    "current_pose": {},
    "current_path": {},
    "progress_percentage": 100.0,
    "estimated_remaining_time": 0.0,
    "system_timestamp": 1781260180.000,
    "performance_metrics": {}
  },
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

APP 解析：

1. 标记当前路线任务完成。
2. 清理当前播报等待状态。
3. 清理当前跳步按钮 loading。
4. 展示完成统计。
5. 允许用户重新开始新路线。

### 5.7 路线任务失败 navigation_failed

```json
{
  "protocol_version": "2.0",
  "message_id": "push_1781260090000_001122",
  "timestamp": 1781260090.000,
  "message_type": "push",
  "data_type": "navigation_status",
  "source": "data_integration",
  "destination": "all",
  "data": {
    "event_type": "navigation_failed",
    "event_data": {
      "reason": "NavigateThroughPoses goal failed",
      "failure_code": "through_goal_failed",
      "route_task": true,
      "task_session_id": "route_session_20260612_001",
      "route_id": "route_main_demo",
      "current_target_task_id": "15",
      "segment_id": "segment_route_session_20260612_001_7_to_15_1",
      "segment_direction": "forward",
      "execution_waypoint_ids": ["11", "12", "15"],
      "passed_transit_waypoint_ids": ["11"],
      "completed_task_ids": ["7"],
      "skipped_task_ids": [],
      "failed_at": 1781260090.000,
      "event_id": "route_task_route_session_20260612_001_navigation_failed_9_1781260090000",
      "timestamp": 1781260090.000
    },
    "timestamp": 1781260090.000,
    "current_state": "failed",
    "navigation_mode": "route_task",
    "sequence_id": 1,
    "current_pose": {},
    "current_path": {},
    "progress_percentage": 40.0,
    "estimated_remaining_time": 0.0,
    "system_timestamp": 1781260090.000,
    "performance_metrics": {}
  },
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

APP 解析：

1. 判断 `data.event_data.route_task=true`，这是路线任务失败。
2. 展示 `failure_code` 和 `reason`。
3. 展示失败目标 `current_target_task_id`。
4. 展示失败段 `execution_waypoint_ids`。
5. 清理播报等待状态。
6. 禁用继续自动推进，等待人工处理。

### 5.8 周期状态中的 route_task 快照

除离散事件外，APP 也可能收到周期 `navigation_status`，其中包含当前 route task 快照。

```json
{
  "protocol_version": "2.0",
  "message_id": "push_1781260060000_112233",
  "timestamp": 1781260060.000,
  "message_type": "push",
  "data_type": "navigation_status",
  "source": "data_integration",
  "destination": "all",
  "data": {
    "current_state": "executing",
    "navigation_mode": "route_task",
    "sequence_id": 1,
    "current_waypoint_index": 0,
    "total_waypoints": 3,
    "progress_percentage": 35.0,
    "route_task": {
      "task_session_id": "route_session_20260612_001",
      "route_id": "route_main_demo",
      "current_anchor_task_id": "7",
      "current_anchor_task_index": 0,
      "current_target_task_id": "15",
      "current_target_task_index": 1,
      "master_route_task_ids": ["7", "15", "16"],
      "completed_task_ids": ["7"],
      "skipped_task_ids": [],
      "awaiting_broadcast": false,
      "waiting_broadcast_waypoint_id": "",
      "waiting_broadcast_id": "",
      "active_segment": {
        "segment_id": "segment_route_session_20260612_001_7_to_15_1",
        "from_task_id": "7",
        "target_task_id": "15",
        "target_task_index": 1,
        "segment_direction": "forward",
        "transit_waypoint_ids": ["11", "12"],
        "execution_waypoint_ids": ["11", "12", "15"],
        "passed_transit_waypoint_ids": ["11"]
      },
      "route_task_version": 1,
      "active_goal_generation": 2,
      "last_feedback_age_sec": 0.4
    },
    "current_pose": {},
    "current_path": {},
    "estimated_remaining_time": 45.0,
    "system_timestamp": 1781260060.000,
    "performance_metrics": {}
  },
  "metadata": {
    "status": "success",
    "error_code": "",
    "error_message": "",
    "request_id": "",
    "data_freshness": 0.1,
    "qos_level": "standard"
  }
}
```

APP 解析：

1. 周期快照用于页面刷新、断线重连后的状态恢复。
2. 周期快照不能替代 `navigation_command_result`。
3. 周期快照不能替代 `broadcast_requested`。
4. 如果 `route_task.awaiting_broadcast=true`，APP 应恢复播报等待 UI。
5. 如果 `route_task.active_segment` 存在，APP 应恢复路线段高亮。

## 6. 障碍物暂停相关状态

动态障碍物暂停恢复是导航状态事件，不改变 route task 业务协议。

APP 收到障碍物阻塞时应暂停 UI 倒计时或显示等待，不要自行发送下一点任务。

示例：

```json
{
  "protocol_version": "2.0",
  "message_id": "push_1781260070000_223344",
  "timestamp": 1781260070.000,
  "message_type": "push",
  "data_type": "navigation_status",
  "source": "data_integration",
  "destination": "all",
  "data": {
    "event_type": "navigation_obstacle_blocked",
    "event_data": {
      "reason": "obstacle_detected",
      "pause_source": "obstacle_wait",
      "message": "前方检测到障碍物，导航已暂停等待",
      "task_session_id": "route_session_20260612_001",
      "route_id": "route_main_demo",
      "current_target_task_id": "15",
      "segment_id": "segment_route_session_20260612_001_7_to_15_1"
    },
    "timestamp": 1781260070.000,
    "current_state": "paused",
    "navigation_mode": "route_task",
    "sequence_id": 1,
    "current_pose": {},
    "current_path": {},
    "progress_percentage": 38.0,
    "estimated_remaining_time": 45.0,
    "system_timestamp": 1781260070.000,
    "performance_metrics": {}
  },
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

APP 解析：

1. 展示障碍物等待提示。
2. 不清空 route task 状态。
3. 不发送 `broadcast_finished`。
4. 不自动发送跳步。
5. 等 ROS 推送恢复事件后恢复导航 UI。

恢复示例：

```json
{
  "protocol_version": "2.0",
  "message_id": "push_1781260078000_334455",
  "timestamp": 1781260078.000,
  "message_type": "push",
  "data_type": "navigation_status",
  "source": "data_integration",
  "destination": "all",
  "data": {
    "event_type": "navigation_resumed",
    "event_data": {
      "resume_source": "obstacle_wait",
      "message": "障碍物已消失，导航继续执行",
      "task_session_id": "route_session_20260612_001",
      "route_id": "route_main_demo",
      "current_target_task_id": "15",
      "segment_id": "segment_route_session_20260612_001_7_to_15_1"
    },
    "timestamp": 1781260078.000,
    "current_state": "executing",
    "navigation_mode": "route_task",
    "sequence_id": 1,
    "current_pose": {},
    "current_path": {},
    "progress_percentage": 38.0,
    "estimated_remaining_time": 45.0,
    "system_timestamp": 1781260078.000,
    "performance_metrics": {}
  },
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

## 7. 后端事件处理状态机

APP 后端建议维护一个路线任务运行态对象。

```json
{
  "task_session_id": "route_session_20260612_001",
  "route_id": "route_main_demo",
  "running": true,
  "current_target_task_id": "15",
  "current_segment_id": "segment_route_session_20260612_001_7_to_15_1",
  "segment_direction": "forward",
  "execution_waypoint_ids": ["11", "12", "15"],
  "passed_transit_waypoint_ids": ["11"],
  "completed_task_ids": ["7"],
  "skipped_task_ids": [],
  "awaiting_broadcast": false,
  "waiting_broadcast_waypoint_id": "",
  "waiting_broadcast_id": "",
  "last_event_id": "route_task_route_session_20260612_001_waypoint_passed_3_1781260050000",
  "last_request_message_id": "app_cmd_20260612_100030_002"
}
```

处理规则：

1. 收到任何 `navigation_status`，先取 `event_type=data.event_type`。
2. 如果没有 `event_type`，再检查是否有 `data.route_task` 周期快照。
3. 离散事件按 `event_data.event_id` 去重。
4. 命令 ack 按 `event_data.request_message_id` 对应按钮 loading。
5. `task_session_id` 不等于当前任务时，不更新当前页面，可进入历史日志。
6. `route_id` 不等于当前路线时，不更新当前页面，可进入历史日志。
7. `broadcast_requested` 到来后置 `awaiting_broadcast=true`。
8. `broadcast_finished` 成功 ack 后不要立刻完成点位，等待 `task_waypoint_completed`。
9. `jump_updated` 到来后清理旧播报 UI，更新当前目标和执行段。
10. `route_task_completed` 到来后清理运行态。
11. `navigation_failed.route_task=true` 到来后进入失败态，等待人工处理。

## 8. 错误码处理建议

APP 后端和前端至少要识别以下错误码。

| error_code | 含义 | APP处理 |
| --- | --- | --- |
| `missing_task_session_id` | 启动或控制命令缺少任务会话 ID | 后端修复组包，前端提示启动失败 |
| `missing_route_id` | 启动命令缺少路线 ID | 后端修复组包 |
| `invalid_route_waypoints` | route_waypoints 不是数组或点位格式错误 | 前端/后端检查路线配置 |
| `duplicate_waypoint_id` | 同一路线存在重复点位 ID | 禁止保存或启动 |
| `invalid_waypoint_role` | 点位角色不是 task/transit | 禁止保存或启动 |
| `missing_waypoint_pose` | 坐标或朝向缺失/非法 | 提示重新录点 |
| `missing_task_waypoints` | 路线里没有任务点 | 禁止启动 |
| `missing_broadcast_id` | task 需要播报但没有 broadcast_id | 禁止启动 |
| `route_task_already_running` | 已有路线任务运行中又启动新路线 | 前端提示先结束当前任务 |
| `navigation_busy` | 导航状态忙，不能启动 | 前端提示稍后再试 |
| `route_task_not_running` | 未运行路线任务却发送 jump/broadcast | 后端检查状态 |
| `invalid_task_session` | task_session_id 不匹配 | 后端使用当前 session 重发 |
| `invalid_route_id` | route_id 不匹配 | 后端使用当前 route_id 重发 |
| `invalid_target_waypoint` | 跳步目标不存在 | 前端刷新路线点列表 |
| `target_waypoint_not_task` | 跳到了辅助点 | 前端禁止选择 transit |
| `interrupt_broadcast_false_not_supported` | 等待播报时不允许不打断播报跳步 | 跳步固定传 true |
| `broadcast_not_waiting` | ROS 当前不在等待播报 | 后端不要重复发送 broadcast_finished |
| `broadcast_context_mismatch` | waypoint_id 或 broadcast_id 与当前等待播报不一致 | 后端用 broadcast_requested 的字段回传 |
| `unsupported_broadcast_result` | broadcast_result 不是 completed | APP 不要传 failed/cancelled/空字符串 |
| `unknown_navigation_command` | APP 发送了旧命令或未定义命令 | 前端/后端修正 command_type，只使用 route task 新命令 |
| `route_task_not_paused` | 发送 `resume_route_task` 时路线任务并不在暂停态 | 前端刷新当前状态，避免重复点击继续 |
| `missing_active_segment` | 恢复路线任务时当前执行段丢失 | 提示路线任务状态异常，建议停止后重新开始 |

## 9. 典型流程

### 9.1 正常完整路线

```text
APP start_route_task
ROS navigation_command_result success
ROS 到达 task 7
ROS broadcast_requested 7
APP 播报 7
APP broadcast_finished 7
ROS navigation_command_result success
ROS task_waypoint_completed 7
ROS waypoint_passed 11
ROS waypoint_passed 12
ROS 到达 task 15
ROS broadcast_requested 15
APP 播报 15
APP broadcast_finished 15
ROS task_waypoint_completed 15
ROS 到达 task 16
ROS task_waypoint_completed 16
ROS route_task_completed
```

### 9.2 从 7 跳到 15

```text
APP start_route_task
ROS navigation_command_result success
APP jump_to_waypoint target_waypoint_id=15
ROS navigation_command_result success
ROS jump_updated execution_waypoint_ids=["11","12","15"]
ROS waypoint_passed 11
ROS waypoint_passed 12
ROS 到达 task 15
ROS broadcast_requested 15
APP broadcast_finished 15
ROS task_waypoint_completed 15 next_target_task_id=16
ROS 继续执行 16
```

### 9.3 从 15 跳回 7

```text
APP jump_to_waypoint target_waypoint_id=7
ROS navigation_command_result success
ROS jump_updated segment_direction=backward execution_waypoint_ids=["12","11","7"]
ROS waypoint_passed 12
ROS waypoint_passed 11
ROS 到达 task 7
ROS broadcast_requested 7
APP broadcast_finished 7
ROS task_waypoint_completed 7 next_target_task_id=15
ROS 继续执行 15
```

### 9.4 等待播报时跳步

```text
ROS broadcast_requested waypoint_id=7 broadcast_id=broadcast_point_7_intro
APP 正在播报 7
APP jump_to_waypoint target_waypoint_id=15 interrupt_broadcast=true
ROS navigation_command_result success
ROS jump_updated interrupted_broadcast={"waypoint_id":"7","broadcast_id":"broadcast_point_7_intro"}
APP 停止 7 的播报 UI
ROS 执行新段 ["11","12","15"]
```

## 10. APP联调验收清单

前端验收：

1. 点位设置页可以设置 `task/transit`。
2. `transit` 不能设置播报。
3. `transit` 不能设置停车对齐。
4. `task.need_broadcast=true` 时必须填写播报资源。
5. 路线预览能显示完整顺序，包括辅助点。
6. 跳步弹窗只显示 `task`。
7. 收到 `broadcast_requested` 后才开始播报。
8. 收到 `jump_updated.interrupted_broadcast` 后能停止旧播报 UI。
9. 收到 `waypoint_passed` 不弹播报。
10. 收到 `route_task_completed` 后路线 UI 正常完成。

后端验收：

1. 启动路线时一次性下发完整 `route_waypoints`。
2. `route_waypoints` 包含 `task` 和 `transit`。
3. `route_waypoints` 顺序与 UI 保存顺序一致。
4. 所有 ID 都按字符串发送。
5. `task_session_id` 每次执行唯一。
6. `jump_to_waypoint` 带 `task_session_id`、`route_id`、`target_waypoint_id`、`interrupt_broadcast=true`。
7. `broadcast_finished` 带 `task_session_id`、`route_id`、`waypoint_id`、`broadcast_id`、`broadcast_result="completed"`。
8. `navigation_command_result` 按 `request_message_id` 对应原始命令。
9. APP 不把 WebSocket `command_ack` 当业务成功。
10. APP 对 `navigation_failed.route_task=true` 能展示失败段和失败目标。

ROS联调验收：

1. `start_route_task` 返回 `navigation_command_result success`。
2. 正向跳步能触发 `jump_updated segment_direction=forward`。
3. 反向跳步能触发 `jump_updated segment_direction=backward`。
4. 跳步经过辅助点时，`execution_waypoint_ids` 包含 transit。
5. transit 只触发 `waypoint_passed`，不触发 `broadcast_requested`。
6. task 需要播报时触发 `broadcast_requested`。
7. APP 回传 `broadcast_finished` 后，ROS 继续下一任务点。
8. 跳到 `D` 后，`D` 完成继续去 `E`。
9. 跳到 `B` 后，`B` 完成继续去 `C`。
10. 障碍物暂停期间 route task 状态不丢失，恢复后继续当前段。
