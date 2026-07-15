# APP协议从 Ubuntu 旧链路迁移到 Todesk 路线任务链路说明

本文档面向 APP 前端、APP 后端和 ROS 联调同学，用于把当前仍按 `/home/ubuntu/humanoid_ws` 旧 Python 链路交互的 APP，迁移到 Todesk 工作区 C++ 路线任务链路。

当前结论：

1. Ubuntu 主工作区旧链路使用 `humanoid_websocket + humanoid_navigation`，导航命令主要是 `start_single_navigation / start_multi_point_navigation / pause_navigation / resume_navigation / stop_navigation`。
2. Todesk 工作区新链路使用 `humanoid_app_gateway_runtime + humanoid_control_runtime + humanoid_route_runtime`，导航命令统一改为路线任务协议：`start_route_task / pause_route_task / resume_route_task / stop_route_task / jump_to_waypoint / broadcast_finished`。
3. APP 如果继续按旧命令发送，在 Todesk 新链路上无法获得跳点、辅助点、播报闭环、多地图和路线任务状态机能力。
4. APP 如果直接按新协议发送到 Ubuntu 旧链路，旧 Python 导航管理器不识别 `jump_to_waypoint / broadcast_finished` 等命令。

## 1. 总体链路变化

旧 Ubuntu 链路：

```text
APP WebSocket
  -> humanoid_websocket/websocket_server.py
  -> /app/waypoint_command 或 /app/navigation_command
  -> humanoid_navigation/dynamic_waypoints_manager.py
  -> /navigation/requests
  -> humanoid_navigation/navigation_state_manager.py
```

新 Todesk 链路：

```text
APP WebSocket
  -> humanoid_app_gateway_runtime/app_gateway_node
  -> /app/waypoint_command、/app/navigation_command、/app/map_command
  -> humanoid_control_runtime/dynamic_waypoints_manager_cpp、map_context_manager_cpp
  -> /navigation/requests
  -> humanoid_route_runtime/navigation_state_manager_cpp
```

APP 侧需要关注的是 WebSocket JSON 协议，不需要关心 ROS 内部 topic 名称。但理解内部链路有助于排查问题：

| APP data_type | Todesk 内部目标 | 作用 |
| --- | --- | --- |
| `waypoint_management` | `/app/waypoint_command` -> `dynamic_waypoints_manager_cpp` | 点位增删改查、点位库 revision 更新 |
| `navigation_control` | `/app/navigation_command` -> `dynamic_waypoints_manager_cpp` -> `/navigation/requests` -> `navigation_state_manager_cpp` | 路线任务、暂停、继续、终止、跳点、播报完成 |
| `map_management` | `/app/map_command` -> `map_context_manager_cpp` | 地图查询、切图 |
| `robot_control` | `/app/robot_control` -> `robot_gateway_node` | 上半身动作 |
| `facial_control` | `/robot/facial_raw_cmd` -> `facial_driver_cpp` | 表情 |
| `initial_pose` | `/initialpose` | 设置初始位姿 |

## 2. 必改命令映射

APP 旧命令不能继续原样使用，需要改成新路线任务命令。

| 旧 Ubuntu 命令 | 新 Todesk 命令 | APP 改造说明 |
| --- | --- | --- |
| `start_single_navigation` | `start_route_task` | 单点也按一条路线处理，`route_waypoint_ids` 只放一个任务点 ID |
| `start_multi_point_navigation` | `start_route_task` | 多点路线按 `route_waypoint_ids` 顺序下发 |
| `start_exhibition_navigation` | `start_route_task` | 展厅路线由 APP 后端按展示路线生成有序 ID 列表 |
| `pause_navigation` | `pause_route_task` | 手动暂停路线任务 |
| `resume_navigation` | `resume_route_task` | 手动继续、人工清障后继续都用它 |
| `stop_navigation` | `stop_route_task` | 终止整条路线任务 |
| 旧链路无 | `jump_to_waypoint` | 跳到当前路线快照内某个任务点 |
| 旧链路无 | `broadcast_finished` | APP 播报完成后通知 ROS 继续 |

旧命令示例：

```json
{
  "message_type": "command",
  "data_type": "navigation_control",
  "data": {
    "command_type": "start_multi_point_navigation",
    "waypoint_ids": ["1", "2", "3"]
  }
}
```

新命令示例：

```json
{
  "protocol_version": "2.0",
  "message_id": "cmd_start_route_0001",
  "timestamp": 1781280000.123,
  "message_type": "command",
  "data_type": "navigation_control",
  "source": "app",
  "destination": "ros",
  "data": {
    "command_type": "start_route_task",
    "request_message_id": "cmd_start_route_0001",
    "task_session_id": "session_20260711_0001",
    "route_id": "route_hall_demo_001",
    "map_id": "hall",
    "waypoints_revision": "1781285566.123",
    "route_waypoint_ids": ["1", "2", "3"]
  },
  "metadata": {
    "operator_id": "user_001",
    "client_id": "pad_001"
  }
}
```

### 2.1 关键业务旧逻辑 vs 新逻辑

这一节用于帮助 APP 团队理解：不是只把 `command_type` 改名，而是整条业务闭环从“点位导航命令”升级成了“路线任务状态机”。APP UI 状态也不能再只看按钮点击或即时 ack，而要看 ROS 主动推送的事实事件。

#### 2.1.1 开始导航

旧 Ubuntu 逻辑：

```text
1. APP 发送 navigation_control。
2. data.command_type 使用 start_single_navigation / start_multi_point_navigation / start_exhibition_navigation。
3. websocket_server.py 只抽取 waypoint_id / waypoint_ids / exhibition_ids 等旧字段。
4. websocket_server.py 发布到 /app/navigation_command。
5. dynamic_waypoints_manager.py 校验点位并补充 waypoint_data / waypoints_data。
6. dynamic_waypoints_manager.py 发布 /navigation/requests。
7. navigation_state_manager.py 按旧命令直接执行单点、多点或展台导航。
8. APP 主要收到旧 navigation_status / acknowledgment 类状态，缺少 route task、跳点、播报闭环上下文。
```

旧命令特点：

1. 多点路线主要是简单点位列表。
2. 没有 `task_session_id` 和 `route_id`，APP 很难区分“上一轮任务的迟到事件”和“当前任务事件”。
3. 没有 `waypoints_revision`，APP 和 ROS 点位库不同步时存在按旧坐标执行的风险。
4. 没有 `task/transit` 角色，辅助点和任务点语义不清晰。
5. 没有标准 `broadcast_requested -> broadcast_finished` 播报闭环。
6. 没有当前路线快照内的任意跳点能力。

新 Todesk 逻辑：

```text
1. APP 发送 navigation_control。
2. data.command_type 使用 start_route_task。
3. APP 必须携带 task_session_id、route_id、map_id。
4. 推荐携带 route_waypoint_ids + waypoints_revision。
5. app_gateway_node 归一化命令并发布到 /app/navigation_command。
6. dynamic_waypoints_manager_cpp 轻校验、归一化 ID，把 APP 命令封装成 /navigation/requests。
7. navigation_state_manager_cpp 进入 route task 状态机。
8. 如果使用 ID 列表模式，ROS 先用 waypoints_revision 校验 APP 和 ROS 点位库是否一致。
9. ROS 冻结本次路线快照，按 task/transit、播报、倒走、最终对齐等属性执行。
10. ROS 通过 navigation_status 推送 navigation_command_result，APP 用它判断开始命令业务成功或失败。
11. 后续路线进度、辅助点通过、任务点完成、播报请求、跳点、完成/失败都通过 navigation_status.event_type 推送。
```

APP 迁移要点：

1. 不再发送 `start_single_navigation / start_multi_point_navigation / start_exhibition_navigation`。
2. 单点、多点、展厅路线都统一发送 `start_route_task`。
3. APP 后端生成并保存当前任务的 `task_session_id` 和 `route_id`。
4. APP 用 `route_waypoint_ids` 的数组顺序表示路线顺序。
5. APP 必须先保存点位并记录最新 `waypoints_revision`，再启动 ID 列表模式。
6. APP 不能用 WebSocket 即时 `command_ack` 判断导航已经开始，必须等 `navigation_command_result(status=success, command_type=start_route_task)`。

#### 2.1.2 暂停导航

旧 Ubuntu 逻辑：

```text
1. APP 发送 navigation_control + pause_navigation。
2. websocket_server.py 转发到 /app/navigation_command。
3. dynamic_waypoints_manager.py 继续封装到 /navigation/requests。
4. navigation_state_manager.py 暂停当前旧导航流程。
5. APP 根据旧 ack 或状态变化切 UI。
```

新 Todesk 逻辑：

```text
1. APP 发送 navigation_control + pause_route_task。
2. 必须携带当前 task_session_id 和 route_id。
3. route task 状态机校验当前任务是否运行、session 是否匹配。
4. ROS 取消/暂停当前 Nav2 goal，并发布零速度。
5. ROS 先推 navigation_command_result 表示暂停命令业务是否被接受。
6. ROS 再推 navigation_paused 作为事实暂停事件。
```

APP 迁移要点：

1. 按钮点击后可以显示“暂停中”，但不能立即认为机器人已暂停。
2. `navigation_command_result(status=success, command_type=pause_route_task)` 表示暂停命令被接受。
3. `navigation_paused` 才表示事实暂停。
4. 手动暂停通常看 `pause_source="route_task_user_request"`。
5. 障碍暂停也会推 `navigation_paused`，但 `pause_source="obstacle_wait"`，APP UI 文案应不同。

#### 2.1.3 继续导航和人工清障继续

旧 Ubuntu 逻辑：

```text
1. APP 发送 navigation_control + resume_navigation。
2. navigation_state_manager.py 尝试恢复旧导航流程。
3. 障碍恢复、人工继续和普通继续的事件上下文不够清晰。
```

新 Todesk 逻辑：

```text
1. APP 发送 navigation_control + resume_route_task。
2. 必须携带当前 task_session_id 和 route_id。
3. 如果是手动暂停后的继续，ROS 按当前 active segment 继续执行。
4. 如果是障碍暂停后的人工清障继续，APP 也发送同一个 resume_route_task，但 reason 建议使用 manual_obstacle_cleared。
5. ROS 推 navigation_command_result 表示继续命令是否被接受。
6. ROS 推 navigation_resumed 表示事实恢复。
```

APP 迁移要点：

1. 普通继续和人工清障继续都使用 `resume_route_task`。
2. APP 可通过 `reason` 区分业务来源，例如 `user_resume`、`manual_obstacle_cleared`。
3. UI 恢复到“导航中”应以 `navigation_resumed` 为准。

#### 2.1.4 终止导航

旧 Ubuntu 逻辑：

```text
1. APP 发送 navigation_control + stop_navigation。
2. navigation_state_manager.py 停止当前旧导航。
3. APP 根据旧 ack 或状态刷新 UI。
```

新 Todesk 逻辑：

```text
1. APP 发送 navigation_control + stop_route_task。
2. 必须携带当前 task_session_id 和 route_id。
3. route task 状态机校验任务上下文。
4. ROS 取消当前 Nav2 goal、清理障碍等待运行态、清理路线任务状态。
5. ROS 推 navigation_command_result 表示终止命令是否被接受。
6. ROS 推 navigation_stopped 表示事实终止。
```

APP 迁移要点：

1. 终止后要清理本地当前任务上下文。
2. UI 显示“已终止”应以 `navigation_stopped` 为准。
3. 终止后旧的 `task_session_id` 不应再用于继续、跳点或播报完成。

#### 2.1.5 任意跳点

旧 Ubuntu 逻辑：

```text
旧链路没有标准 jump_to_waypoint。
APP 侧如果想跳点，通常只能停止后重新发一个新的多点导航，或者依赖非标准临时逻辑。
这种做法会丢失当前路线任务上下文、已完成点、播报等待和辅助点吸收语义。
```

新 Todesk 逻辑：

```text
1. APP 只能在已有 route task 运行中发送 jump_to_waypoint。
2. APP 必须携带当前 task_session_id、route_id、target_waypoint_id。
3. ROS 只允许跳到当前路线快照内的 task 点。
4. ROS 不重新读取点位库，避免导航中点位变化影响当前任务。
5. ROS 取消旧 Nav2 goal，按当前锚点到目标 task 的路线段重新规划。
6. ROS 自动吸收中间 transit 辅助点。
7. ROS 推 navigation_command_result 表示跳点命令是否被接受。
8. ROS 推 jump_updated 表示当前路线目标和进度已更新。
```

APP 迁移要点：

1. 跳点弹窗只展示 `waypoint_role="task"` 的点。
2. 不允许跳到 `transit` 辅助点。
3. APP 收到 `jump_updated` 后再刷新当前目标、高亮点位和剩余路线。
4. 如果收到 `target_waypoint_not_task`，说明 APP 把辅助点作为跳点目标发给 ROS，需要修正 UI 过滤。

#### 2.1.6 到点播报

旧 Ubuntu 逻辑：

```text
旧链路更多依赖 APP 或导航页根据“到点状态”自行判断播报。
ROS 缺少标准的“请求 APP 播报”和“APP 播报完成后继续”的闭环命令。
```

新 Todesk 逻辑：

```text
1. 路线点位 properties 中配置 need_broadcast、broadcast_id、broadcast_text、broadcast_blocking。
2. ROS 到达需要播报的 task 点并完成最终对齐后，推 broadcast_requested。
3. APP 收到 broadcast_requested 后开始播报。
4. APP 播报完成后发送 navigation_control + broadcast_finished。
5. ROS 校验 task_session_id、route_id、broadcast_id 是否匹配当前等待上下文。
6. ROS 推 navigation_command_result 表示播报完成命令是否接受。
7. ROS 推 task_waypoint_completed，并继续下一段路线。
```

APP 迁移要点：

1. APP 不要再仅靠本地“到点”推断播报，必须以 `broadcast_requested` 为准。
2. APP 发送 `broadcast_finished` 时必须带回 ROS 推送中的 `broadcast_id`。
3. 如果 APP 重复发送同一个播报完成，ROS 可能返回重复忽略事件，APP 应做幂等处理。
4. 任务点如果 `need_broadcast=false`，ROS 不会等待 APP 播报完成，会自动继续。

#### 2.1.7 点位保存和点位同步

旧 Ubuntu 逻辑：

```text
1. APP 发送 waypoint_management + set/update/delete/get/clear。
2. ROS 保存点位并推送 waypoints_data。
3. 开始多点导航时 APP 直接发 waypoint_ids。
4. 缺少强制 revision 校验，APP 与 ROS 点位不同步时风险较高。
```

新 Todesk 逻辑：

```text
1. APP 仍发送 waypoint_management + set/update/delete/get/clear。
2. 点位数据必须包含 map_id 和 route task 需要的 properties。
3. dynamic_waypoints_manager_cpp 保存点位后刷新 waypoints_revision。
4. ROS 推 waypoint_response 和 waypoints_data。
5. APP 必须记录最新 waypoints_revision。
6. APP 使用 route_waypoint_ids 启动路线时必须带 waypoints_revision。
7. ROS 校验 revision 一致后才从本地点位库补全路线快照。
```

APP 迁移要点：

1. 点位保存成功以 `waypoint_response(status=success)` 为准。
2. APP 后端必须持久化最新 `waypoints_revision`。
3. 启动路线前如果没有 revision，应先发 `get_waypoints` 或等待 `waypoints_data`。
4. 收到 `waypoints_revision_mismatch` 时，APP 必须重新同步点位，不要继续启动。

#### 2.1.8 地图查询和切图

旧 Ubuntu 逻辑：

```text
旧 websocket_server.py 没有完整 map_management 路由。
APP 通常按固定地图或旧页面状态处理，切图能力不完整。
```

新 Todesk 逻辑：

```text
1. APP 发送 map_management + get_map_list / get_current_map / switch_map。
2. app_gateway_node 发布到 /app/map_command。
3. map_context_manager_cpp 查询地图注册表或执行切图流程。
4. ROS 通过 map_response 返回命令结果。
5. 切图开始时 result_reason=map_switch_restart_started。
6. 地图和定位稳定后 result_reason=map_ready 或 map_state=ready。
7. APP 只有等地图 ready 后才能允许开始该地图路线任务。
```

APP 迁移要点：

1. 进入导航页先查询 `get_current_map` 和 `get_map_list`。
2. 切图后开始导航按钮应置灰，直到收到 `map_ready`。
3. 开始路线任务必须带当前 `map_id`。

## 3. 统一 WebSocket 外层格式

APP 发给 ROS 的命令统一使用：

```json
{
  "protocol_version": "2.0",
  "message_id": "cmd_xxx",
  "timestamp": 1781280000.123,
  "message_type": "command",
  "data_type": "navigation_control",
  "source": "app",
  "destination": "ros",
  "data": {},
  "metadata": {}
}
```

关键要求：

1. `message_id` 必须唯一。
2. `data.request_message_id` 建议复制外层 `message_id`，便于 APP 匹配后续业务结果。
3. `message_type` 可以使用 `command`；Todesk C++ 网关也兼容 `business_command`，但新 APP 建议统一用 `command`。
4. WebSocket 即时 `command_ack` 只表示网关收到并已转发，不代表导航业务成功。
5. 导航业务是否成功，必须看 `data_type="navigation_status"` 且 `data.event_type="navigation_command_result"`。

外层字段要求：

| 字段 | 是否必填 | 说明 |
| --- | --- | --- |
| `protocol_version` | 必填 | 当前协议版本固定建议 `2.0`；请求/订阅链路会校验该字段 |
| `message_id` | 必填 | APP 生成的唯一消息 ID；ROS 会用它关联即时 ack 和业务结果 |
| `timestamp` | 必填 | APP 发送时间戳，单位秒；用于日志、排查和消息时序分析 |
| `message_type` | 必填 | 命令用 `command`，订阅用 `subscription`，数据请求用 `request` |
| `data_type` | 必填 | 外层业务类型，例如 `navigation_control`、`waypoint_management`、`map_management` |
| `source` | 必填 | 建议固定 `app` |
| `destination` | 建议必填 | 建议固定 `ros` |
| `data` | 必填 | 命令参数对象；网关要求它必须是 JSON object |
| `metadata` | 可选 | 调试、追踪、扩展字段 |

内层通用字段要求：

| 字段 | 是否必填 | 说明 |
| --- | --- | --- |
| `data.command_type` | 大部分命令必填 | 点位、导航、地图命令都必须携带；缺失会被网关拒绝 |
| `data.request_message_id` | 建议必填 | 建议复制外层 `message_id`；导航、地图、音量等业务结果会优先使用它回传 |

## 4. 点位管理改造

点位管理入口不变，仍使用：

```json
{
  "protocol_version": "2.0",
  "message_id": "cmd_set_waypoint_0001",
  "timestamp": 1781280000.123,
  "message_type": "command",
  "data_type": "waypoint_management",
  "source": "app",
  "destination": "ros",
  "data": {
    "command_type": "set_waypoint",
    "map_id": "hall",
    "waypoint_data": {
      "id": "1",
      "name": "任务点1",
      "type": "navigation_target",
      "frame_id": "map",
      "map_id": "hall",
      "position": [1.0, 2.0, 0.0],
      "orientation": [0.0, 0.0, 0.0, 1.0],
      "properties": {
        "waypoint_role": "task",
        "need_broadcast": true,
        "broadcast_id": "broadcast_1",
        "broadcast_text": "欢迎来到点位1",
        "broadcast_blocking": true,
        "stop_and_align": true,
        "walk_direction": "forward",
        "route_order": 1
      }
    },
    "request_message_id": "cmd_set_waypoint_0001"
  },
  "metadata": {}
}
```

点位 `properties` 是路线任务能力的关键：

| 字段 | 取值 | 说明 |
| --- | --- | --- |
| `waypoint_role` | `task` / `transit` | `task` 是会停车对齐的任务点；`transit` 是辅助点，只经过不停车 |
| `need_broadcast` | `true` / `false` | 是否需要 APP 到点播报 |
| `broadcast_id` | 字符串 | 播报资源 ID；`need_broadcast=true` 时建议必填 |
| `broadcast_text` | 字符串 | APP 播报文案，ROS 不播放音频 |
| `broadcast_blocking` | `true` / `false` | 是否等待 APP 播报完成 |
| `stop_and_align` | `true` / `false` | 是否到点后最终对齐 |
| `walk_direction` | `forward` / `backward` | 正走或倒走，通常只给任务点使用 |
| `route_order` | 数字 | APP UI 排序字段；ROS 实际按 `route_waypoint_ids` 数组顺序执行 |

APP 必须记录 ROS 返回的 `waypoints_revision`：

1. 保存、更新、删除、清空点位成功后，ROS 会通过 `waypoint_response` 和 `waypoints_data` 返回最新 `waypoints_revision`。
2. APP 后端必须保存最新 revision。
3. 后续使用 `route_waypoint_ids` 启动路线时必须携带这个 revision。

点位命令字段要求：

| command_type | 必须携带字段 | 说明 |
| --- | --- | --- |
| `set_waypoint` | 源码硬校验：`waypoint_data.id` 或 `waypoint_data.waypoint_id`、`waypoint_data.type`；产品业务必须显式带：`waypoint_data.position`、`waypoint_data.orientation` | 新增点位；`name` 不填时默认使用 ID，`frame_id` 默认 `map`；不要依赖 pose 默认值 |
| `update_waypoint` | `waypoint_data.id` 或 `waypoint_data.waypoint_id`、`waypoint_data.type` | 增量更新点位；只覆盖 APP 显式携带的字段 |
| `delete_waypoint` | `waypoint_id`、`waypoint_type` | 删除指定地图下的指定类型点位 |
| `get_waypoints` | `command_type` | `map_id` 不填时使用当前/默认地图；`waypoint_type` 可选，`include_details` 可选 |
| `clear_waypoints` | `map_id` 或 `clear_scope="all_maps"` | 多地图模式下不带 `map_id` 会被拒绝，避免误删其他地图点位 |

点位类型 `waypoint_type` 必须是 ROS 侧支持的类型，例如当前导航任务点通常使用 `navigation_target`。如果点位后续要参与路线任务，`properties.waypoint_role` 必须是 `task` 或 `transit`，并且 `task + need_broadcast=true` 时必须携带 `broadcast_id`。虽然保存点位时源码会兼容缺失 `position/orientation` 并填默认值，但 APP 必须显式发送真实 pose，否则后续路线可能在错误位置执行。

## 5. 开始路线任务

推荐 APP 默认使用 ID 列表模式：

```json
{
  "protocol_version": "2.0",
  "message_id": "cmd_start_route_ids_0001",
  "timestamp": 1781280000.123,
  "message_type": "command",
  "data_type": "navigation_control",
  "source": "app",
  "destination": "ros",
  "data": {
    "command_type": "start_route_task",
    "request_message_id": "cmd_start_route_ids_0001",
    "task_session_id": "session_20260711_0001",
    "route_id": "route_hall_demo_001",
    "map_id": "hall",
    "waypoints_revision": "1781285566.123",
    "route_waypoint_ids": ["1", "2", "3", "4"]
  },
  "metadata": {}
}
```

规则：

1. `task_session_id`：本次任务会话 ID，每次开始路线都要生成新的。
2. `route_id`：路线 ID，APP 自己维护。
3. `map_id`：当前地图 ID。
4. `route_waypoint_ids`：按路线执行顺序排列，不能为空。
5. `waypoints_revision`：ID 列表模式必填。
6. `route_waypoints` 和 `route_waypoint_ids` 只能二选一。
7. 启动成功后，ROS 会冻结本次路线快照；导航过程中修改点位只影响下一次启动。

`start_route_task` 必填字段：

| 字段 | 是否必填 | 说明 |
| --- | --- | --- |
| `command_type` | 必填 | 固定 `start_route_task` |
| `request_message_id` | 建议必填 | 建议复制外层 `message_id` |
| `task_session_id` | 必填 | 本次任务会话 ID；后续暂停、继续、终止、跳点、播报完成都必须带同一个值 |
| `route_id` | 必填 | 本次路线 ID；后续控制命令必须带同一个值 |
| `map_id` | 必填 | 本次路线所属地图；缺失会返回 `missing_map_id` |
| `route_waypoint_ids` | ID 模式必填 | 非空数组，不能有重复 ID；必须和 `waypoints_revision` 一起使用 |
| `waypoints_revision` | ID 模式必填 | 必须等于 ROS 当前点位库 revision，否则返回 `waypoints_revision_mismatch` |
| `route_waypoints` | 完整快照模式必填 | 非空数组；不能和 `route_waypoint_ids` 同时出现 |

`route_waypoints[]` 每个点位的字段要求：

| 字段 | 是否必填 | 说明 |
| --- | --- | --- |
| `waypoint_id` | 必填 | 点位 ID，不能为空，数组内不能重复 |
| `waypoint_role` | 必填 | 只能是 `task` 或 `transit`；也可以放在 `properties.waypoint_role` |
| `position` | 必填 | `[x, y, z]` 数组 |
| `orientation` | 必填 | 四元数 `[x, y, z, w]` 数组，范数不能为 0 |
| `map_id` | 建议携带 | 不填时使用外层路线 `map_id`；如果携带，必须和外层 `map_id` 一致 |
| `frame_id` | 可选 | 默认 `map` |
| `need_broadcast` | task 点可选 | 默认 `false`；也可以放在 `properties.need_broadcast` |
| `broadcast_id` | 条件必填 | `waypoint_role=task` 且 `need_broadcast=true` 时必填 |
| `broadcast_text` | 可选 | APP 播报展示/播放文案 |
| `broadcast_blocking` | 可选 | 默认 `true`；是否等待 APP 回传 `broadcast_finished` |
| `stop_and_align` | 可选 | 默认 `true`；任务点是否停车并最终对齐 |
| `walk_direction` | 可选 | 默认 `forward` |

如果 APP 暂时无法保证 ROS 点位库已同步，也可以发送完整快照模式：

```json
{
  "protocol_version": "2.0",
  "message_id": "cmd_start_route_full_0001",
  "timestamp": 1781280000.123,
  "message_type": "command",
  "data_type": "navigation_control",
  "source": "app",
  "destination": "ros",
  "data": {
    "command_type": "start_route_task",
    "request_message_id": "cmd_start_route_full_0001",
    "task_session_id": "session_20260711_0002",
    "route_id": "route_hall_demo_001",
    "map_id": "hall",
    "route_waypoints": [
      {
        "waypoint_id": "1",
        "waypoint_role": "task",
        "frame_id": "map",
        "position": [1.0, 2.0, 0.0],
        "orientation": [0.0, 0.0, 0.0, 1.0],
        "need_broadcast": true,
        "broadcast_id": "broadcast_1",
        "stop_and_align": true,
        "walk_direction": "forward"
      }
    ]
  },
  "metadata": {}
}
```

## 6. 暂停、继续、终止、跳点、播报完成

### 6.1 暂停 pause_route_task

必填字段：`command_type`、`task_session_id`、`route_id`。`reason` 和 `pause_parameters` 可选。`task_session_id`、`route_id` 必须和当前正在运行的路线任务一致，否则会返回 `invalid_task_session` 或 `invalid_route_id`。

```json
{
  "protocol_version": "2.0",
  "message_id": "cmd_pause_route_0001",
  "timestamp": 1781280100.123,
  "message_type": "command",
  "data_type": "navigation_control",
  "source": "app",
  "destination": "ros",
  "data": {
    "command_type": "pause_route_task",
    "request_message_id": "cmd_pause_route_0001",
    "task_session_id": "session_20260711_0001",
    "route_id": "route_hall_demo_001",
    "reason": "user_pause"
  },
  "metadata": {}
}
```

### 6.2 继续 resume_route_task

手动继续、人工清障后继续都使用这个命令：

必填字段：`command_type`、`task_session_id`、`route_id`。`reason` 可选。该命令用于用户暂停后的继续，也用于障碍暂停后 APP/人工确认清障后的继续。

```json
{
  "protocol_version": "2.0",
  "message_id": "cmd_resume_route_0001",
  "timestamp": 1781280120.123,
  "message_type": "command",
  "data_type": "navigation_control",
  "source": "app",
  "destination": "ros",
  "data": {
    "command_type": "resume_route_task",
    "request_message_id": "cmd_resume_route_0001",
    "task_session_id": "session_20260711_0001",
    "route_id": "route_hall_demo_001",
    "reason": "manual_obstacle_cleared"
  },
  "metadata": {}
}
```

### 6.3 终止 stop_route_task

必填字段：`command_type`、`task_session_id`、`route_id`。`reason`、`stop_parameters` 可选。终止成功后当前路线任务结束，后续如果继续导航需要重新发送 `start_route_task`。

```json
{
  "protocol_version": "2.0",
  "message_id": "cmd_stop_route_0001",
  "timestamp": 1781280140.123,
  "message_type": "command",
  "data_type": "navigation_control",
  "source": "app",
  "destination": "ros",
  "data": {
    "command_type": "stop_route_task",
    "request_message_id": "cmd_stop_route_0001",
    "task_session_id": "session_20260711_0001",
    "route_id": "route_hall_demo_001",
    "reason": "user_stop",
    "stop_parameters": {
      "clear_remaining": true
    }
  },
  "metadata": {}
}
```

### 6.4 跳点 jump_to_waypoint

必填字段：`command_type`、`task_session_id`、`route_id`、`target_waypoint_id`。`target_waypoint_id` 必须是当前路线快照内的任务点，不能是辅助点。

```json
{
  "protocol_version": "2.0",
  "message_id": "cmd_jump_route_0001",
  "timestamp": 1781280160.123,
  "message_type": "command",
  "data_type": "navigation_control",
  "source": "app",
  "destination": "ros",
  "data": {
    "command_type": "jump_to_waypoint",
    "request_message_id": "cmd_jump_route_0001",
    "task_session_id": "session_20260711_0001",
    "route_id": "route_hall_demo_001",
    "target_waypoint_id": "3",
    "reason": "user_jump",
    "interrupt_broadcast": true
  },
  "metadata": {}
}
```

跳点规则：

1. 只能跳到 `waypoint_role="task"` 的任务点。
2. 不能跳到 `transit` 辅助点。
3. 跳点只在当前路线快照内执行，不重新查询最新点位库。
4. 跳点后，ROS 会按原路线顺序吸收当前锚点到目标任务点之间的辅助点。
5. APP 收到 `jump_updated` 后应更新当前高亮目标和路线进度。

### 6.5 播报完成 broadcast_finished

ROS 到达需要播报的任务点后，会推送 `broadcast_requested`。APP 播报完成后必须回发：

```json
{
  "protocol_version": "2.0",
  "message_id": "cmd_broadcast_finished_0001",
  "timestamp": 1781280200.123,
  "message_type": "command",
  "data_type": "navigation_control",
  "source": "app",
  "destination": "ros",
  "data": {
    "command_type": "broadcast_finished",
    "request_message_id": "cmd_broadcast_finished_0001",
    "task_session_id": "session_20260711_0001",
    "route_id": "route_hall_demo_001",
    "waypoint_id": "1",
    "broadcast_id": "broadcast_1",
    "broadcast_result": "completed",
    "broadcast_duration_sec": 12.3,
    "reason": "app_tts_completed"
  },
  "metadata": {}
}
```

规则：

1. APP 只在收到 `broadcast_requested` 后发送 `broadcast_finished`。
2. 必填字段：`command_type`、`task_session_id`、`route_id`、`waypoint_id`、`broadcast_id`。
3. `waypoint_id` 和 `broadcast_id` 必须分别与 ROS 推送的 `broadcast_requested.data.event_data.waiting_broadcast_waypoint_id` / `broadcast_requested.data.event_data.broadcast_id` 对齐。
4. `task_session_id`、`route_id` 必须和当前正在运行的路线任务一致。
5. `broadcast_result` 当前建议使用 `completed`。
6. `broadcast_duration_sec` 可选，用于日志和 APP 播报耗时统计。
7. 如果 APP 播报失败，需要和 ROS/产品侧再确认是否允许传 `failed`；当前正常链路按 `completed` 设计。

## 7. 地图管理命令

Todesk 新链路支持 `map_management`，Ubuntu 旧 WS 链路没有完整路由该入口。

地图命令字段要求：

| command_type | 必须携带字段 | 说明 |
| --- | --- | --- |
| `get_map_list` | `command_type` | `request_message_id` 建议携带，用于匹配回包 |
| `get_current_map` | `command_type` | `request_message_id` 建议携带 |
| `switch_map` | `command_type`、`target_map_id` | `reason` 可选；切图完成要等 `map_ready`，不能只看即时 ack |

### 7.1 查询地图列表

```json
{
  "protocol_version": "2.0",
  "message_id": "cmd_get_map_list_0001",
  "timestamp": 1781280300.123,
  "message_type": "command",
  "data_type": "map_management",
  "source": "app",
  "destination": "ros",
  "data": {
    "command_type": "get_map_list",
    "request_message_id": "cmd_get_map_list_0001"
  },
  "metadata": {}
}
```

### 7.2 查询当前地图

```json
{
  "protocol_version": "2.0",
  "message_id": "cmd_get_current_map_0001",
  "timestamp": 1781280301.123,
  "message_type": "command",
  "data_type": "map_management",
  "source": "app",
  "destination": "ros",
  "data": {
    "command_type": "get_current_map",
    "request_message_id": "cmd_get_current_map_0001"
  },
  "metadata": {}
}
```

### 7.3 切换地图

```json
{
  "protocol_version": "2.0",
  "message_id": "cmd_switch_map_0001",
  "timestamp": 1781280302.123,
  "message_type": "command",
  "data_type": "map_management",
  "source": "app",
  "destination": "ros",
  "data": {
    "command_type": "switch_map",
    "target_map_id": "hall",
    "reason": "user_selected_map",
    "request_message_id": "cmd_switch_map_0001"
  },
  "metadata": {}
}
```

APP 判断切图是否完成：

1. 先收到 `map_response`，`result_reason="map_switch_restart_started"`，表示切图流程开始。
2. 再等待 `map_response` 或 `map_status` 中出现 `result_reason="map_ready"` 或 `map_state="ready"`。
3. 只有地图 ready 后才允许开始该地图导航。

## 8. APP需要订阅和消费的数据

APP 建议连接后发订阅：

```json
{
  "protocol_version": "2.0",
  "message_id": "sub_nav_0001",
  "timestamp": 1781280400.123,
  "message_type": "subscription",
  "data_type": "subscription_manage",
  "source": "app",
  "destination": "ros",
  "data": {
    "action": "subscribe",
    "data_types": [
      "navigation_status",
      "system_status",
      "waypoints_data",
      "map_status",
      "map_response",
      "action_result",
      "system_exception"
    ],
    "push_frequency": 1.0
  },
  "metadata": {}
}
```

重点消费：

| data_type | APP 用途 |
| --- | --- |
| `connection_ack` | 获取 client_id、支持的 data_type 和命令类型 |
| `waypoints_data` | 同步点位库和 `waypoints_revision` |
| `waypoint_response` | 点位命令成功/失败 |
| `map_status` / `map_response` | 地图列表、当前地图、切图进度 |
| `navigation_status` | 路线任务业务结果和导航事件 |
| `system_status` | 电量、机器人状态、`robot_accid`、`robot_sn` |
| `action_result` | 上半身动作执行结果 |
| `system_exception` | 导航、动作、表情、定位异常 |

## 9. 导航状态事件

所有导航业务事件都看：

```text
message.data_type == "navigation_status"
message.data.event_type
```

常用事件：

| event_type | APP 行为 |
| --- | --- |
| `navigation_command_result` | 命令业务结果。判断 start/pause/resume/stop/jump/broadcast_finished 是否被 ROS 接受 |
| `waypoint_passed` | 辅助点通过，更新路线进度 |
| `broadcast_requested` | ROS 请求 APP 开始播报 |
| `task_waypoint_completed` | 任务点完成 |
| `jump_updated` | 跳点已生效，更新当前目标和剩余路线 |
| `route_task_completed` | 整条路线完成 |
| `navigation_paused` | 导航暂停，可能是用户暂停或障碍暂停 |
| `navigation_obstacle_blocked` | 障碍持续阻塞，刷新等待时长 |
| `navigation_resumed` | 导航恢复 |
| `navigation_stopped` | 导航终止 |
| `navigation_failed` | 导航失败 |
| `final_align_started` / `final_align_completed` | 到任务点后的最终对齐开始/完成 |

`navigation_command_result` 示例：

```json
{
  "message_type": "push",
  "data_type": "navigation_status",
  "source": "data_integration",
  "destination": "all",
  "data": {
    "event_type": "navigation_command_result",
    "event_data": {
      "request_message_id": "cmd_start_route_ids_0001",
      "command_type": "start_route_task",
      "task_session_id": "session_20260711_0001",
      "route_id": "route_hall_demo_001",
      "status": "success",
      "result_reason": "route_task_started",
      "error_code": "",
      "message": "route task started",
      "timestamp": 1781280000.456
    }
  },
  "metadata": {
    "status": "success",
    "error_code": "",
    "error_message": ""
  }
}
```

## 10. 机器人身份和速度/动作

Todesk 新链路里机器人本体网关会从机器人回传状态中动态学习 `accid`，APP 不需要把机器人 `accid` 写死在客户端代码里。

APP 可从 `system_status` 读取：

```json
{
  "data_type": "system_status",
  "data": {
    "robot_accid": "HU_D04_01_289",
    "robot_sn": "XR102-SN-0001",
    "robot_identity": {
      "accid": "HU_D04_01_289",
      "sn": "XR102-SN-0001"
    }
  }
}
```

动作命令仍使用 `robot_control`：

字段要求：外层 `data_type` 固定 `robot_control`；`data.action` 必须是 `execute_gesture` 才会触发动作链路；`data.parameters.gesture_id` 必须携带，表示动作库里的动作 ID。`parameters.loop` 可选。当前动作执行结果通过 `action_result` 回传。

```json
{
  "protocol_version": "2.0",
  "message_id": "cmd_gesture_0001",
  "timestamp": 1781280500.123,
  "message_type": "command",
  "data_type": "robot_control",
  "source": "app",
  "destination": "ros",
  "data": {
    "action": "execute_gesture",
    "parameters": {
      "gesture_id": "wave_greet_bye",
      "loop": false
    }
  },
  "metadata": {}
}
```

表情命令仍使用 `facial_control`：

字段要求：外层 `data_type` 固定 `facial_control`；`data.action` 必填，取值为表情库中的动作名，例如 `talk`。缺少 `action` 会被网关直接拒绝。

```json
{
  "protocol_version": "2.0",
  "message_id": "cmd_face_0001",
  "timestamp": 1781280501.123,
  "message_type": "command",
  "data_type": "facial_control",
  "source": "app",
  "destination": "ros",
  "data": {
    "action": "talk"
  },
  "metadata": {}
}
```

初始位姿命令使用 `initial_pose`：

字段要求：外层 `data_type` 固定 `initial_pose`；`data.x`、`data.y`、`data.yaw` 建议必填，`data.frame_id` 可选且默认 `map`。源码层面缺省数值会按 `0.0` 处理，但 APP 不应依赖默认值，否则容易误把机器人重定位到地图原点。

```json
{
  "protocol_version": "2.0",
  "message_id": "cmd_initial_pose_0001",
  "timestamp": 1781280502.123,
  "message_type": "command",
  "data_type": "initial_pose",
  "source": "app",
  "destination": "ros",
  "data": {
    "x": 1.0,
    "y": 2.0,
    "yaw": 0.0,
    "frame_id": "map"
  },
  "metadata": {}
}
```

广播音量命令走 `navigation_control`：

字段要求：`data.command_type` 固定 `set_broadcast_volume`；`broadcast_volume` 或 `volume_percent` 二选一，取值范围 `0-100`。如果都不带，ROS 当前会按默认值 `72` 处理，但 APP 应显式携带，避免 UI 显示和真实音量不一致。

```json
{
  "protocol_version": "2.0",
  "message_id": "cmd_volume_0001",
  "timestamp": 1781280503.123,
  "message_type": "command",
  "data_type": "navigation_control",
  "source": "app",
  "destination": "ros",
  "data": {
    "command_type": "set_broadcast_volume",
    "volume_percent": 60,
    "request_message_id": "cmd_volume_0001"
  },
  "metadata": {}
}
```

## 11. 错误码和 APP 处理建议

| 错误码 | 常见原因 | APP 处理建议 |
| --- | --- | --- |
| `missing_task_session_id` | 开始/控制路线任务未带 `task_session_id` | 后端补齐会话 ID |
| `missing_route_id` | 未带 `route_id` | 后端补齐路线 ID |
| `missing_map_id` | `start_route_task` 未带 `map_id` | 当前地图未选定，先同步地图 |
| `missing_waypoints_revision` | ID 模式未带 revision | 先查询/等待 `waypoints_data` |
| `waypoints_revision_mismatch` | APP 和 ROS 点位版本不一致 | 重新同步点位库后再启动 |
| `ambiguous_route_waypoint_source` | 同时传 `route_waypoints` 和 `route_waypoint_ids` | 只保留一种启动方式 |
| `waypoint_id_not_found` | 某个 ID 在 ROS 点位库不存在 | 重新同步点位或检查路线配置 |
| `invalid_target_waypoint` | 跳点目标不存在 | APP 禁止选择不存在点位 |
| `target_waypoint_not_task` | 跳到辅助点 | APP 跳点列表只展示任务点 |
| `route_task_not_running` | 暂停/继续/终止/跳点时无任务运行 | 刷新 UI 状态 |
| `invalid_task_session` | 控制命令 session 与当前任务不一致 | 使用当前任务 session |
| `broadcast_context_mismatch` | 播报完成 ID 和当前等待播报不一致 | 使用 ROS `broadcast_requested` 中的 ID |
| `unsupported_command` | 仍发送旧命令或未知命令 | 改成新命令 |

## 12. APP 改造清单

后端必须改：

1. 把旧导航命令映射到新路线任务命令。
2. 为每次路线任务生成 `task_session_id`。
3. 为每条路线维护 `route_id`。
4. 每个命令外层生成唯一 `message_id`，内层 `request_message_id` 复制它。
5. 记录 ROS 返回的 `waypoints_revision`。
6. 默认使用 `route_waypoint_ids + waypoints_revision` 启动路线。
7. 收到 `broadcast_requested` 后，在播报完成时发送 `broadcast_finished`。
8. 跳点时只允许选择 `task` 点。
9. 业务成功/失败以 `navigation_command_result` 为准，而不是 WebSocket 即时 `command_ack`。
10. 增加 `map_management` 支持：查询地图、切图、等待 `map_ready`。

前端必须改：

1. 点位编辑页增加 `task/transit`、是否播报、播报 ID、是否对齐、正走/倒走等属性。
2. 路线进度页区分任务点和辅助点。
3. 跳点弹窗只展示任务点。
4. 播报由 `broadcast_requested` 驱动，不要仅靠本地到点推断。
5. 暂停/继续/终止按钮状态以 ROS 事实事件校准。
6. 障碍暂停时显示 `navigation_paused(pause_source="obstacle_wait")`，障碍持续刷新看 `navigation_obstacle_blocked`。
7. 系统状态栏读取 `system_status.robot_accid`，不要再写死机器人身份。

## 13. 联调最小流程

1. APP 建立 WebSocket 连接，等待 `connection_ack`。
2. APP 订阅 `navigation_status / system_status / waypoints_data / map_status / map_response / action_result / system_exception`。
3. APP 发送 `map_management.get_current_map`，确认当前地图。
4. APP 保存或更新点位，等待 `waypoint_response success`。
5. APP 记录最新 `waypoints_revision`。
6. APP 发送 `start_route_task`。
7. APP 等待 `navigation_command_result(status=success, command_type=start_route_task)`。
8. APP 根据后续 `navigation_status.event_type` 更新 UI。
9. 如果收到 `broadcast_requested`，APP 播报，播报完成后发送 `broadcast_finished`。
10. 测试跳点：APP 发送 `jump_to_waypoint`，等待 `navigation_command_result + jump_updated`。
11. 测试暂停/继续/终止：分别发送 `pause_route_task / resume_route_task / stop_route_task`。

## 14. 兼容注意事项

1. `system_command` 当前在 `connection_ack.supported_commands` 中会出现，但业务路由层没有真正支持。APP 不要依赖它。
2. 旧 `start_single_navigation / start_multi_point_navigation / pause_navigation / resume_navigation / stop_navigation` 不属于 Todesk 新路线任务协议。
3. 如果 APP 需要同时兼容 ubuntu 旧链路和 Todesk 新链路，建议根据 `connection_ack.supported_data_types` 是否包含 `map_response`、是否支持 `map_management`，或由后端配置明确选择协议版本。
4. Todesk 新链路推荐 APP 直接切到 route task 协议，不建议在 APP 侧长期保留两套 UI 状态机。
5. 如果现场仍跑 `/home/ubuntu/humanoid_ws` 旧 Python 链路，必须先把 ROS 侧切换/同步到 Todesk C++ runtime 链路，否则 APP 新协议会被旧导航管理器拒绝或忽略。
