# APP侧路线任务_跳步_辅助点_倒走_障碍暂停恢复开发文档

> 本文档面向 APP 前端、APP 后端、ROS 联调同学。
> 目标是让 APP 侧按 ROS 当前新协议完成路线任务、任意跳步、辅助点无痕通过、倒走点、到点播报、障碍暂停恢复、人工脱困后继续等功能。

## 1. 总体口径

1. 新导航控制链路只使用 route task 新协议，不再使用旧的单点、多点、展台、旧暂停、旧继续、旧终止命令。
2. APP 启动路线支持两种方式：推荐下发 `route_waypoint_ids + waypoints_revision`，ROS 从本地点位库补全；也兼容下发完整 `route_waypoints` 快照。
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
13. `route_waypoints` 和 `route_waypoint_ids` 只能二选一，不能同时传；同时传会返回 `ambiguous_route_waypoint_source`。
14. ID 列表模式必须携带最新 `waypoints_revision`，否则 ROS 会拒绝启动，避免 APP 和 ROS 点位库不同步。
15. ROS 与机器人本体建立连接后，会从机器人高频 `notify_robot_info` 原始消息中动态学习 `robot_accid`，不再要求每次换机器人都手改客户端代码。
16. APP 如果订阅 `system_status`，现在会额外收到 `robot_accid`、`robot_sn`、`robot_identity`，可据此识别当前连接的是哪一台机器人。

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

### 2.3 手动按钮状态和 ROS 事实状态的关系

APP 前端可以继续保留“开始 / 暂停 / 继续 / 终止”四个按钮，不必为障碍暂停额外新增一套按钮。但按钮点击只能代表“用户意图”，最终 UI 状态必须用 ROS 推送的 `navigation_status` 事实事件校准。

推荐前端状态栏拆成两层：

| 层级 | 作用 | 来源 |
| --- | --- | --- |
| 命令处理中 | 用户刚点击按钮，等待 ROS 业务确认 | `navigation_command_result` |
| 机器人事实状态 | 机器人真实处于运行、暂停、障碍等待、恢复、终止、失败 | `navigation_paused/navigation_resumed/navigation_stopped/navigation_failed` 等状态事件 |

APP 监听规则：

1. `navigation_command_result` 只表示“APP 刚发的命令是否被 ROS 业务层接受”，不要把它当作最终 UI 状态。
2. `navigation_paused` 才是进入暂停态的事实事件。
3. `navigation_resumed` 才是恢复导航中的事实事件。
4. `navigation_stopped` 才是终止完成的事实事件。
5. `navigation_failed` 才是导航失败的事实事件。
6. `navigation_obstacle_blocked` 是障碍等待期间的周期刷新事件，只刷新状态栏和等待时长，不重复弹窗。

手动暂停 UI 流程：

1. 用户点击“暂停”。
2. 前端显示“暂停中...”，按钮临时禁用或显示 loading。
3. 后端发送 `pause_route_task`。
4. 收到 `navigation_command_result(command_type="pause_route_task", status="success")` 后，状态栏可显示“暂停命令已接受”。
5. 收到 `navigation_paused(pause_source="route_task_user_request")` 后，UI 正式显示“已暂停：用户手动暂停”。

障碍暂停 UI 流程：

1. ROS 检测到动态障碍并自动暂停。
2. APP 收到 `navigation_paused(pause_source="obstacle_wait")`。
3. 前端 UI 自动切到暂停态，状态栏显示“障碍物阻塞，等待恢复/请清障”。
4. 如果障碍自动消失，APP 收到 `navigation_resumed(resume_source="obstacle_wait", resume_reason="obstacle_cleared_auto_resume")`，UI 切回“导航中”。
5. 如果现场人工清障或人工脱困，用户点击同一个“继续”按钮，后端发送 `resume_route_task`，最终仍以 `navigation_resumed` 作为 UI 恢复依据。

当前源码真实字段口径：

| 场景 | 事件 | 关键字段 |
| --- | --- | --- |
| 手动暂停 | `navigation_paused` | `pause_source="route_task_user_request"`，`resume_mode="manual"`，`waiting_for_obstacle_clear=false` |
| 障碍暂停 | `navigation_paused` | `pause_source="obstacle_wait"`，`resume_mode="auto"`，`waiting_for_obstacle_clear=true` |
| 手动暂停后继续 | `navigation_resumed` | `resume_source="route_task_user_request"`，`resume_reason` 为 APP 传入原因 |
| 障碍自动恢复 | `navigation_resumed` | `resume_source="obstacle_wait"`，`resume_reason="obstacle_cleared_auto_resume"` |
| 障碍人工继续 | `navigation_resumed` | `resume_source="obstacle_wait"`，`resume_reason` 为 APP 传入原因，例如 `manual_obstacle_cleared` |

兼容提醒：早期文档曾出现 `pause_source="user_request"`，当前路线任务源码实际使用 `route_task_user_request`。APP 可以兼容 `user_request`，但新协议判断手动暂停应优先使用 `route_task_user_request`。

## 3. APP后端改造

### 3.1 后端职责

APP 后端负责把前端点位和路线配置转换成 ROS 可识别的协议：

1. 保存点位库：维护点位坐标、姿态、任务/辅助属性、播报属性、倒走属性。
2. 生成路线任务：按前端路线顺序生成 `route_waypoint_ids`，并携带最新 `waypoints_revision`；必要时也可组装完整 `route_waypoints` 作为兜底。
3. 生成唯一 ID：每次命令带唯一 `message_id`，每条路线任务带唯一 `task_session_id`。
4. 透传到 ROS：WebSocket 外层 `message_id` 要复制到内层 `request_message_id`。
5. 解析 ROS 推送：所有导航业务事件从 `data_type="navigation_status"`、`data.event_type` 分发。
6. 管理播报闭环：收到 `broadcast_requested` 后开始播报，播报完成后发送 `broadcast_finished`。
7. 管理障碍恢复：收到障碍暂停后显示状态；人工清障后发送 `resume_route_task`。

### 3.2 点位库和路线任务的关系

点位库命令 `set_waypoint/update_waypoint/delete_waypoint/get_waypoints/clear_waypoints` 只负责保存基础点位。

当前 ROS 支持两种开始路线方式：

| 模式 | APP 发送字段 | 使用场景 | 是否依赖 ROS 点位库 |
| --- | --- | --- | --- |
| ID 列表模式，推荐默认使用 | `route_waypoint_ids + waypoints_revision` | 点位已提前保存到 ROS，本次路线只需要按顺序引用点位 ID | 是，ROS 校验 revision 后补全 |
| 完整快照模式，保留兜底 | `route_waypoints` | 调试、回放、跨机器临时路线、APP 想完全控制本次快照 | 否，ROS 直接使用 APP 下发快照 |

重要规则：

1. APP 在导航前必须先通过点位管理命令保存/更新点位。
2. ROS 保存/更新/删除/清空点位成功后，会在 `waypoint_response` 和 `waypoints_data` 中返回最新 `waypoints_revision`。
3. APP 后端必须记录最新 `waypoints_revision`。
4. 使用 ID 列表模式启动时，APP 必须携带该 `waypoints_revision`。
5. ROS 收到 ID 列表模式后，会先比较 APP 传入 revision 和 ROS 当前 revision。
6. revision 一致后，ROS 才按 `route_waypoint_ids` 数组顺序从本地点位库补全完整点位。
7. revision 不一致时，ROS 返回 `waypoints_revision_mismatch`，APP 应重新同步点位库后再启动。
8. 启动后 ROS 会冻结本次路线快照，导航中即使点位库变化，也不会影响当前任务和跳步计算。
9. `route_waypoint_ids` 数组顺序就是路线顺序，ROS 不按 ID 数字大小排序。
10. `route_waypoints` 和 `route_waypoint_ids` 只能传一种，同时传会返回 `ambiguous_route_waypoint_source`。

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
    "...": "..."                             // 具体命令参数直接放在 data 下，不要再包 command_data
  },
  "metadata": {
    "operator_id": "user_001",               // 可选，操作人 ID，便于审计
    "client_id": "pad_001",                  // 可选，APP 设备 ID
    "request_source": "route_page"           // 可选，触发页面或业务来源
  }
}
```

### 4.2 开始路线任务 start_route_task

`start_route_task` 支持两种启动模式。APP 后端只能选择其中一种：

1. 推荐默认使用 `route_waypoint_ids + waypoints_revision`。
2. 调试/兜底时使用完整 `route_waypoints`。
3. 两种字段不能同时出现。
4. 两种模式启动成功后，ROS 内部都会冻结成完整 `route_waypoints` 快照，后续跳步、辅助点吸收、播报、倒走逻辑完全一致。

#### 4.2.1 推荐模式：只发送点位 ID 列表

适用条件：

1. APP 已经提前通过点位管理命令把点位保存到 ROS。
2. APP 已记录 ROS 返回的最新 `waypoints_revision`。
3. 导航中不允许修改本次路线涉及的点位。

```jsonc
{
  "protocol_version": "2.0",                    // 协议版本
  "message_id": "cmd_start_route_ids_0001",     // 本次开始命令唯一 ID
  "timestamp": 1781280000.123,                  // APP 发包时间
  "message_type": "command",                    // 固定 command
  "data_type": "navigation_control",            // 固定 navigation_control
  "source": "app",                              // 固定 app
  "destination": "ros",                         // 固定 ros
  "data": {
    "command_type": "start_route_task",         // 开始路线任务
    "request_message_id": "cmd_start_route_ids_0001", // 必填，复制外层 message_id
    "task_session_id": "session_20260613_001",        // 必填，本次路线任务会话 ID
    "route_id": "route_exhibition_001",               // 必填，路线 ID
    "route_name": "展厅讲解路线",                      // 可选，路线名称，仅展示和日志使用
    "waypoints_revision": "1781285566.123",            // 必填，APP 当前记录的 ROS 点位库版本
    "route_waypoint_ids": [                            // 必填，有序点位 ID 列表
      "10",                                            // 第 1 个点位 ID，可以是 task
      "11",                                            // 第 2 个点位 ID，可以是 transit
      "12",                                            // 第 3 个点位 ID，可以是 transit
      "13",                                            // 第 4 个点位 ID，可以是 task
      "14"                                             // 第 5 个点位 ID，可以是 backward task
    ]
  },
  "metadata": {
    "operator_id": "user_001",                          // 操作人
    "client_id": "pad_001"                              // APP 设备
  }
}
```

ROS 处理规则：

1. `route_waypoint_ids` 中的顺序就是路线顺序。
2. ROS 不会按数字大小重新排序。
3. ROS 会校验 `waypoints_revision` 是否等于当前点位库版本。
4. revision 一致后，ROS 按 ID 从点位库读取 position/orientation/properties。
5. ID 不存在时返回 `waypoint_id_not_found`。
6. revision 缺失时返回 `missing_waypoints_revision`。
7. revision 不一致时返回 `waypoints_revision_mismatch`。
8. `route_waypoint_ids=[]` 返回 `invalid_route_waypoint_ids`。

#### 4.2.2 兼容模式：发送完整路线点位快照

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
    "request_message_id": "cmd_start_route_0001", // 必填，复制外层 message_id
    "task_session_id": "session_20260613_001",    // 必填，本次路线任务会话 ID，整条路线期间保持不变
    "route_id": "route_exhibition_001",           // 必填，路线 ID，后续 pause/resume/stop/jump/broadcast_finished 都要一致
    "route_name": "展厅讲解路线",                  // 可选，路线名称，仅展示和日志使用
    "route_waypoints": [                           // 完整路线点位数组，ROS 按数组顺序执行
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
3. ID 列表模式必须提供非空 `route_waypoint_ids` 和最新 `waypoints_revision`。
4. 完整快照模式必须提供非空 `route_waypoints`。
5. 两种模式最终都必须至少包含一个 `task` 点。
6. `waypoint_id` 不能重复。
7. `task` 点如果 `need_broadcast=true`，必须提供非空 `broadcast_id`。

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
    "request_message_id": "cmd_pause_route_0001", // 必填，复制外层 message_id
    "task_session_id": "session_20260613_001",    // 必填，当前任务会话 ID
    "route_id": "route_exhibition_001",           // 必填，当前路线 ID
    "reason": "user_manual_pause",                // 可选，暂停原因，建议枚举：user_manual_pause
    "pause_parameters": {
      "pause_duration": 0                          // 可选，0 表示不限时，手动继续
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
    "request_message_id": "cmd_resume_route_0001", // 必填，复制外层 message_id
    "task_session_id": "session_20260613_001",     // 必填，当前任务会话 ID
    "route_id": "route_exhibition_001",            // 必填，当前路线 ID
    "reason": "manual_obstacle_cleared",           // 可选，继续原因：user_manual_resume/manual_obstacle_cleared/manual_escape_done
    "resume_source": "app_manual",                 // 可选，建议 app_manual
    "manual_clear_obstacle": true,                 // 可选，人工清障/人工脱困后建议 true
    "operator_confirmed_safe": true                // 可选，现场确认安全后再继续
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
    "request_message_id": "cmd_stop_route_0001", // 必填，复制外层 message_id
    "task_session_id": "session_20260613_001",   // 必填，当前任务会话 ID
    "route_id": "route_exhibition_001",          // 必填，当前路线 ID
    "reason": "user_manual_stop",                // 可选，终止原因
    "stop_parameters": {
      "clear_task_context": true                  // 可选，建议 true，终止后清理上下文
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
    "request_message_id": "cmd_jump_route_0001",   // 必填，复制外层 message_id
    "task_session_id": "session_20260613_001",     // 必填，当前任务会话 ID
    "route_id": "route_exhibition_001",            // 必填，当前路线 ID
    "target_waypoint_id": "15",                    // 必填，目标 task 点 ID，不能是 transit
    "interrupt_broadcast": true,                   // 建议固定 true，等待播报时允许打断并跳走
    "reason": "user_jump_to_waypoint"              // 可选，跳步原因
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
    "request_message_id": "cmd_broadcast_finished_0001", // 必填，复制外层 message_id
    "task_session_id": "session_20260613_001",           // 必填，当前任务会话 ID
    "route_id": "route_exhibition_001",                  // 必填，当前路线 ID
    "waypoint_id": "10",                                 // 必填，必须等于 broadcast_requested 的 waypoint_id
    "broadcast_id": "broadcast_10",                      // 必填，必须等于 broadcast_requested 的 broadcast_id
    "broadcast_result": "completed",                     // 必填/可省略，当前只支持 completed
    "finished_at": 1781280500.123                        // 可选，APP 播报完成时间
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

点位管理命令仍走 `data_type="waypoint_management"`，用于保存/更新点位库。所有业务字段同样直接放在外层 `data` 下，不要再包 `command_data`。后端启动导航时推荐只下发 `route_waypoint_ids + waypoints_revision`，ROS 会从本地点位库补全完整点位。

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
    "waypoint_id": "11",                     // 必填，点位 ID
    "waypoint_type": "navigation_target"     // 必填，点位类型
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
    "waypoint_type": "navigation_target",     // 可选，navigation_target 或 all
    "include_details": true                   // true 返回完整点位；false 只返回 ID 列表
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

### 6.10 手动暂停 navigation_paused

```jsonc
{
  "protocol_version": "2.0",                       // 固定协议版本
  "message_id": "push_navigation_paused_0001",     // ROS 生成的推送消息 ID
  "message_type": "push",                          // 固定 push，表示 ROS 主动推送
  "data_type": "navigation_status",                // 导航状态类消息固定 navigation_status
  "source": "data_integration",                    // 推送来源：数据整合节点
  "destination": "all",                            // 推送目标：所有已连接 APP 客户端
  "timestamp": 1781281000.223,                      // WebSocket 推送时间戳
  "data": {                                        // 业务数据体
    "event_type": "navigation_paused",             // 事实状态事件：导航已暂停
    "event_data": {                                // 暂停事件详情
      "pause_source": "route_task_user_request",   // 暂停来源：APP 用户手动暂停路线任务
      "reason": "user_manual_pause",              // 暂停原因：APP pause_route_task 传入的 reason
      "resume_mode": "manual",                    // 恢复模式：需要 APP/用户点击继续
      "waiting_for_obstacle_clear": false,         // 手动暂停不是障碍等待
      "pause_location": {                          // 暂停时机器人位置，可能为 null
        "x": 1.23,                                 // 暂停时机器人 x，单位米
        "y": 2.34,                                 // 暂停时机器人 y，单位米
        "z": 0.0                                   // 暂停时机器人 z，单位米
      },
      "pause_time": 1781281000.123,                // ROS 进入暂停态的时间戳
      "pause_duration": 0,                         // 暂停时长限制，0 表示不限时
      "current_waypoint_id": "15",                 // 当前底层目标点 ID，可能为空
      "current_waypoint_name": "实验室任务点15",    // 当前底层目标点名称，可能为空
      "waypoint_index": 3,                         // 当前底层点索引，兼容字段
      "total_waypoints": 25,                       // 当前任务总点数，兼容字段
      "task_session_id": "session_20260613_001",   // 当前路线任务会话 ID
      "route_id": "route_exhibition_001",          // 当前路线 ID
      "current_target_task_id": "15",              // 当前路线任务目标 task ID
      "segment_id": "seg_7_to_15_2",               // 当前执行段 ID
      "route_task": true                           // true 表示当前处于路线任务模式
    },
    "timestamp": 1781281000.123,                    // 原始 /navigation/status 事件时间戳
    "current_state": "paused",                     // ROS 导航状态机当前状态
    "navigation_mode": "route_task",               // 当前导航模式
    "current_pose": {},                             // 数据整合层附加的当前定位快照
    "current_path": {},                             // 数据整合层附加的当前路径快照
    "progress_percentage": 35.5,                    // 数据整合层估算的导航进度
    "estimated_remaining_time": 40.0,               // 数据整合层估算剩余时间，单位秒
    "system_timestamp": 1781281000.223,             // 数据整合层处理时间戳
    "performance_metrics": {}                       // 数据整合层附加性能指标
  },
  "metadata": {                                     // 推送元信息
    "push_reason": "navigation_event",             // 推送原因：导航离散事件
    "data_freshness": 0.0,                          // 数据新鲜度，单位秒
    "qos_level": "realtime"                         // 实时事件
  }
}
```

APP 处理：

1. 正式切到 `paused_user`。
2. 状态栏显示“已暂停：用户手动暂停”或展示 `reason`。
3. “继续”按钮保持可用，点击后发送 `resume_route_task`。

### 6.11 障碍暂停 navigation_paused

```jsonc
{
  "protocol_version": "2.0",                       // 固定协议版本
  "message_id": "push_navigation_paused_obstacle_0001", // ROS 生成的推送消息 ID
  "message_type": "push",                          // 固定 push，表示 ROS 主动推送
  "data_type": "navigation_status",                // 导航状态类消息固定 navigation_status
  "source": "data_integration",                    // 推送来源：数据整合节点
  "destination": "all",                            // 推送目标：所有已连接 APP 客户端
  "timestamp": 1781281005.223,                      // WebSocket 推送时间戳
  "data": {                                        // 业务数据体
    "event_type": "navigation_paused",             // 事实状态事件：导航已因障碍暂停
    "event_data": {                                // 暂停事件详情
      "pause_source": "obstacle_wait",             // 暂停来源：动态障碍等待
      "reason": "检测到障碍物，前方路径被挡住",      // 暂停原因：ROS 检测到障碍
      "resume_mode": "auto",                       // 恢复模式：障碍消失后 ROS 可自动恢复
      "waiting_for_obstacle_clear": true,          // 正在等待障碍清除
      "pause_location": {                          // 暂停时机器人位置，可能为 null
        "x": 1.23,                                 // 暂停时机器人 x，单位米
        "y": 2.34,                                 // 暂停时机器人 y，单位米
        "z": 0.0                                   // 暂停时机器人 z，单位米
      },
      "pause_time": 1781281005.123,                // ROS 进入障碍等待的时间戳
      "pause_duration": 0,                         // 暂停时长限制，0 表示不限时
      "current_waypoint_id": "15",                 // 当前底层目标点 ID，可能为空
      "current_waypoint_name": "实验室任务点15",    // 当前底层目标点名称，可能为空
      "waypoint_index": 3,                         // 当前底层点索引，兼容字段
      "total_waypoints": 25,                       // 当前任务总点数，兼容字段
      "block_duration": 5.2,                       // 进入暂停前障碍持续阻塞时长，单位秒
      "clear_confirmed_frames": 0,                 // 已连续确认 clear 的帧数
      "clear_required_frames": 5,                  // 自动恢复前需要连续 clear 的帧数
      "task_session_id": "session_20260613_001",   // 当前路线任务会话 ID
      "route_id": "route_exhibition_001",          // 当前路线 ID
      "current_target_task_id": "15",              // 当前路线任务目标 task ID
      "segment_id": "seg_7_to_15_2",               // 当前执行段 ID
      "route_task": true                           // true 表示当前处于路线任务模式
    },
    "timestamp": 1781281005.123,                    // 原始 /navigation/status 事件时间戳
    "current_state": "paused",                     // ROS 导航状态机当前状态
    "navigation_mode": "route_task",               // 当前导航模式
    "current_pose": {},                             // 数据整合层附加的当前定位快照
    "current_path": {},                             // 数据整合层附加的当前路径快照
    "progress_percentage": 35.5,                    // 数据整合层估算的导航进度
    "estimated_remaining_time": 40.0,               // 数据整合层估算剩余时间，单位秒
    "system_timestamp": 1781281005.223,             // 数据整合层处理时间戳
    "performance_metrics": {}                       // 数据整合层附加性能指标
  },
  "metadata": {                                     // 推送元信息
    "push_reason": "navigation_event",             // 推送原因：导航离散事件
    "data_freshness": 0.0,                          // 数据新鲜度，单位秒
    "qos_level": "realtime"                         // 实时事件
  }
}
```

APP 处理：

1. 正式切到 `paused_obstacle`。
2. 状态栏显示“障碍物阻塞，等待障碍消失/请人工清障”。
3. `resume_mode="auto"` 时，提示“障碍消失后会自动恢复”，但可以保留同一个“继续”按钮用于人工清障后继续。
4. 用户点击“继续”时，后端发送 `resume_route_task`，最终仍等待 `navigation_resumed` 决定 UI 是否恢复。

### 6.12 障碍持续阻塞 navigation_obstacle_blocked

```jsonc
{
  "protocol_version": "2.0",                       // 固定协议版本
  "message_id": "push_navigation_obstacle_blocked_0001", // ROS 生成的推送消息 ID
  "message_type": "push",                          // 固定 push，表示 ROS 主动推送
  "data_type": "navigation_status",                // 导航状态类消息固定 navigation_status
  "source": "data_integration",                    // 推送来源：数据整合节点
  "destination": "all",                            // 推送目标：所有已连接 APP 客户端
  "timestamp": 1781281012.523,                      // WebSocket 推送时间戳
  "data": {                                        // 业务数据体
    "event_type": "navigation_obstacle_blocked",   // 障碍等待期间的周期刷新事件
    "event_data": {                                // 障碍阻塞详情
      "reason": "检测到障碍物，前方路径被挡住",      // 阻塞原因
      "block_duration": 12.4,                      // 已阻塞/等待时长，单位秒
      "blocked_waypoint_id": "15",                 // 被阻塞目标点 ID
      "blocked_waypoint_name": "实验室任务点15",     // 被阻塞目标点名称
      "blocked_waypoint_index": 3,                 // 被阻塞目标点索引，兼容字段
      "total_waypoints": 25,                       // 当前任务总点数，兼容字段
      "position": [4.0, 2.0, 0.0],                 // 被阻塞目标点位置
      "waiting_for_obstacle_clear": true,          // 固定 true，表示仍在等待障碍清除
      "clear_confirmed_frames": 0,                 // 已连续确认 clear 的帧数
      "clear_required_frames": 5,                  // 自动恢复前需要连续 clear 的帧数
      "pause_source": "obstacle_wait",             // 固定 obstacle_wait
      "front_obstacle_stats": {                    // 前方障碍统计，字段可能随检测实现扩展
        "blocked": true,                           // 前方窗口是否仍被阻塞
        "occupied_cells": 18,                      // 阻塞栅格数量
        "checked_cells": 120,                      // 检查栅格总数
        "window_front_min_x_m": 0.15,              // 前方检测窗口最小 x，单位米
        "window_front_max_x_m": 1.2,               // 前方检测窗口最大 x，单位米
        "window_half_width_m": 0.45                // 前方检测窗口半宽，单位米
      }
    },
    "timestamp": 1781281012.423,                    // 原始 /navigation/status 事件时间戳
    "current_state": "paused",                     // ROS 导航状态机当前状态
    "navigation_mode": "route_task",               // 当前导航模式
    "current_pose": {},                             // 数据整合层附加的当前定位快照
    "current_path": {},                             // 数据整合层附加的当前路径快照
    "progress_percentage": 35.5,                    // 数据整合层估算的导航进度
    "estimated_remaining_time": 40.0,               // 数据整合层估算剩余时间，单位秒
    "system_timestamp": 1781281012.523,             // 数据整合层处理时间戳
    "performance_metrics": {}                       // 数据整合层附加性能指标
  },
  "metadata": {                                     // 推送元信息
    "push_reason": "navigation_event",             // 推送原因：导航离散事件
    "data_freshness": 0.0,                          // 数据新鲜度，单位秒
    "qos_level": "realtime"                         // 实时事件
  }
}
```

APP 处理：只刷新障碍等待状态栏、等待时长和调试信息，不重复弹窗，不改变按钮结构。

### 6.13 导航恢复 navigation_resumed

```jsonc
{
  "protocol_version": "2.0",                       // 固定协议版本
  "message_id": "push_navigation_resumed_0001",    // ROS 生成的推送消息 ID
  "message_type": "push",                          // 固定 push，表示 ROS 主动推送
  "data_type": "navigation_status",                // 导航状态类消息固定 navigation_status
  "source": "data_integration",                    // 推送来源：数据整合节点
  "destination": "all",                            // 推送目标：所有已连接 APP 客户端
  "timestamp": 1781281020.223,                      // WebSocket 推送时间戳
  "data": {                                        // 业务数据体
    "event_type": "navigation_resumed",            // 事实状态事件：导航已恢复
    "event_data": {                                // 恢复事件详情
      "task_session_id": "session_20260613_001",   // 当前路线任务会话 ID，route task 场景有
      "route_id": "route_exhibition_001",          // 当前路线 ID，route task 场景有
      "route_task": true,                          // true 表示路线任务
      "current_target_task_id": "15",              // 当前目标 task ID，手动 resume 场景有
      "segment_id": "seg_7_to_15_2",               // 当前执行段 ID，手动 resume 场景有
      "resumed_waypoint_id": "15",                 // 自动障碍恢复时的底层恢复目标 ID，可能为空
      "resumed_waypoint_name": "实验室任务点15",    // 自动障碍恢复时的底层恢复目标名称，可能为空
      "waypoint_index": 3,                         // 自动障碍恢复时的底层目标索引，兼容字段
      "total_waypoints": 25,                       // 当前任务总点数，兼容字段
      "pause_duration_actual": 8.5,                // 实际暂停时长，单位秒
      "resume_reason": "manual_obstacle_cleared",  // 恢复原因：APP 传入或 ROS 自动恢复原因
      "resume_source": "obstacle_wait",            // 恢复来源：obstacle_wait 或 route_task_user_request
      "awaiting_broadcast": false,                 // 是否恢复到等待播报阶段
      "waiting_broadcast_waypoint_id": "",         // 如果正在等待播报，这里是点位 ID
      "waiting_broadcast_id": "",                  // 如果正在等待播报，这里是播报 ID
      "front_obstacle_stats": {                    // 障碍恢复场景可能携带最近一次前方窗口统计
        "blocked": false,                          // false 表示前方窗口已 clear
        "occupied_cells": 0,                       // 阻塞栅格数量
        "checked_cells": 120                       // 检查栅格总数
      }
    },
    "timestamp": 1781281020.123,                    // 原始 /navigation/status 事件时间戳
    "current_state": "executing",                  // ROS 导航状态机当前状态
    "navigation_mode": "route_task",               // 当前导航模式
    "current_pose": {},                             // 数据整合层附加的当前定位快照
    "current_path": {},                             // 数据整合层附加的当前路径快照
    "progress_percentage": 36.0,                    // 数据整合层估算的导航进度
    "estimated_remaining_time": 38.0,               // 数据整合层估算剩余时间，单位秒
    "system_timestamp": 1781281020.223,             // 数据整合层处理时间戳
    "performance_metrics": {}                       // 数据整合层附加性能指标
  },
  "metadata": {                                     // 推送元信息
    "push_reason": "navigation_event",             // 推送原因：导航离散事件
    "data_freshness": 0.0,                          // 数据新鲜度，单位秒
    "qos_level": "realtime"                         // 实时事件
  }
}
```

APP 处理：清除暂停/障碍 UI；如果 `awaiting_broadcast=true`，继续完成播报闭环，不要等待机器人移动。

常见恢复来源：

1. `resume_source="route_task_user_request"`：手动暂停后继续。
2. `resume_source="obstacle_wait"` 且 `resume_reason="obstacle_cleared_auto_resume"`：障碍消失后 ROS 自动恢复。
3. `resume_source="obstacle_wait"` 且 `resume_reason="manual_obstacle_cleared"` 或 `manual_escape_done`：障碍/脱困后用户点击 APP 继续。

### 6.14 导航终止 navigation_stopped

```jsonc
{
  "protocol_version": "2.0",                       // 固定协议版本
  "message_id": "push_navigation_stopped_0001",    // ROS 生成的推送消息 ID
  "message_type": "push",                          // 固定 push，表示 ROS 主动推送
  "data_type": "navigation_status",                // 导航状态类消息固定 navigation_status
  "source": "data_integration",                    // 推送来源：数据整合节点
  "destination": "all",                            // 推送目标：所有已连接 APP 客户端
  "timestamp": 1781281100.223,                      // WebSocket 推送时间戳
  "data": {                                        // 业务数据体
    "event_type": "navigation_stopped",            // 事实状态事件：导航已终止
    "event_data": {                                // 终止事件详情
      "task_session_id": "session_20260613_001",   // 被终止任务会话 ID
      "route_id": "route_exhibition_001",          // 被终止路线 ID
      "route_task": true,                          // true 表示路线任务
      "reason": "user_manual_stop",                // 终止原因：APP stop_route_task 传入的 reason
      "emergency_stop": false,                     // 是否急停式终止
      "current_target_task_id": "15",              // 终止前当前目标 task ID
      "segment_id": "seg_7_to_15_2",               // 终止前当前执行段 ID
      "completed_task_ids": ["1", "2", "7"],       // 终止前已完成 task ID 列表
      "skipped_task_ids": ["8", "9"],              // 终止前已跳过 task ID 列表
      "completed_count": 3,                        // 已完成 task 数量
      "skipped_count": 2,                          // 已跳过 task 数量
      "task_count": 20,                            // 路线 task 总数
      "stopped_at": 1781281100.123                 // 终止时间戳
    },
    "timestamp": 1781281100.123,                    // 原始 /navigation/status 事件时间戳
    "current_state": "cancelled",                  // ROS 导航状态机当前状态
    "navigation_mode": "route_task",               // 当前导航模式
    "current_pose": {},                             // 数据整合层附加的当前定位快照
    "current_path": {},                             // 数据整合层附加的当前路径快照
    "progress_percentage": 36.0,                    // 数据整合层估算的导航进度
    "estimated_remaining_time": 0.0,                // 数据整合层估算剩余时间，终止后通常为 0
    "system_timestamp": 1781281100.223,             // 数据整合层处理时间戳
    "performance_metrics": {}                       // 数据整合层附加性能指标
  },
  "metadata": {                                     // 推送元信息
    "push_reason": "navigation_event",             // 推送原因：导航离散事件
    "data_freshness": 0.0,                          // 数据新鲜度，单位秒
    "qos_level": "realtime"                         // 实时事件
  }
}
```

APP 处理：结束当前路线 UI，清理所有等待中的播报、跳步、暂停状态。

## 6.15 系统状态中的机器人身份字段 system_status

ROS 现在会把机器人身份信息沿着下面这条链路透传给 APP：

1. `websocket_client` 连接机器人本体后，从高频 `notify_robot_info` 原始消息中优先提取顶层 `accid`。
2. 如底层消息里还带 `sn` / `robot_sn` / `serial_number`，ROS 也会一并记录。
3. `message_bridge` 会把这些字段补进 `/robot_status_processed`。
4. `data_integration_node_recoverable` 会把这些字段合并到 `system_status` 顶层。
5. APP 订阅 `system_status` 后即可直接识别当前机器人，无需再额外查询其它链路。

字段说明：

1. `robot_accid`：机器人控制身份 ID。APP 建议优先用它识别当前机器人。
2. `robot_sn`：机器人序列号。如果底层未上报，可能为空字符串。
3. `robot_identity`：身份对象，结构固定为 `{ "accid": "...", "sn": "..." }`，便于前后端统一解析。

```jsonc
{
  "protocol_version": "2.0",                  // 固定协议版本
  "message_id": "push_system_status_0001",    // ROS 推送消息 ID
  "timestamp": 1781281200.223,                  // WebSocket 推送时间戳
  "message_type": "push",                     // push 表示 ROS 主动推送
  "data_type": "system_status",               // 系统状态数据类型
  "source": "data_integration",               // 推送来源
  "destination": "all",                       // 推送给所有已连接客户端
  "data": {
    "battery_level": 74,                        // 电量百分比
    "signal_quality": 85,                       // 信号质量百分比
    "signal_status": "Good",                  // 信号质量等级
    "network_latency": "124ms",               // 网络延迟估计
    "robot_accid": "HU_D04_01_289",           // 机器人 accid，APP 建议优先用它识别设备
    "robot_sn": "XR102-SN-0001",              // 机器人序列号；若底层未上报可能为空串
    "robot_identity": {                         // 机器人身份对象，便于前后端统一解析
      "accid": "HU_D04_01_289",               // 与 robot_accid 相同
      "sn": "XR102-SN-0001"                   // 与 robot_sn 相同
    },
    "robot_status": "Walk",                   // 当前机器人底层状态
    "system_health": "normal",                // 系统综合健康状态
    "operational_status": "navigating",       // 当前业务运行模式
    "timestamp": 1781281200.123,                // 数据打包时间戳
    "details": {                                // 其它详细系统信息
      "power_info": {
        "total_voltage": 47.58,
        "total_current": 5.10,
        "bat_temperature": 44.0
      },
      "motion_busy": false,
      "current_motion": "",
      "control_ready_for_navigation": true,
      "health_check": {
        "peripheral": "OK",
        "system_info": "system info"
      }
    }
  },
  "metadata": {
    "status": "success",                      // 推送状态
    "error_code": "",                         // 错误码，成功时为空
    "error_message": "",                      // 错误信息，成功时为空
    "request_id": "",                         // 主动推送通常为空
    "data_freshness": 0.05,                     // 数据新鲜度，单位秒
    "qos_level": "standard"                   // 系统状态通常是标准实时性
  }
}
```

APP 处理建议：

1. 设备识别优先读取 `data.robot_accid`。
2. 如果需要同时展示序列号，再读取 `data.robot_sn`。
3. 若 `robot_sn` 为空，不应判定为异常；底层可能只上报 `accid`。
4. 如果 APP 有“当前机器人信息”栏，建议显示 `robot_accid + robot_sn` 组合信息。

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
| `invalid_route_waypoints` | 未提供有效完整点位快照，或完整点位数组非法 | 检查 `route_waypoints` 是否数组且字段完整；若使用 ID 模式则不要传该字段 |
| `invalid_route_waypoint_ids` | ID 列表非法或为空 | 检查 `route_waypoint_ids` 必须是非空数组，且元素不能是空字符串 |
| `ambiguous_route_waypoint_source` | 同时传了完整点位和 ID 列表 | `route_waypoints` 与 `route_waypoint_ids` 只能二选一 |
| `missing_waypoints_revision` | ID 列表模式缺少点位库版本 | APP 先从 `waypoint_response/waypoints_data` 记录最新 `waypoints_revision` |
| `waypoints_revision_mismatch` | APP 传入版本与 ROS 当前点位库版本不一致 | 重新同步点位库，拿到最新 revision 后再启动 |
| `waypoints_cache_not_ready` | ROS 状态管理器尚未收到点位缓存 | 延迟重试，或先请求/等待点位数据推送 |
| `waypoint_id_not_found` | ID 列表中的某个点位在 ROS 点位库不存在 | 重新保存点位或刷新 APP 点位库 |
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
