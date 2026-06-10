# APP 侧路线任务、任意跳步与播报协同改造方案

## 1. 文档目的

本文档定义 app 侧针对以下需求的详细改造方案：

1. app 不再逐点派发导航，而是一次性下发整轮路线任务。
2. app 支持任意跳步：
   - `A -> D`
   - `D -> B`
   - `A -> G`
   - `G -> C`
3. app 能正确接收 ROS 通知：
   - 已到达可播报点位，可以开始播报
   - 跳步重规划已生效
   - 当前点只是途经点，不需要播报
4. app 在播报结束后，向 ROS 发送“可以继续走了”命令。
5. app 在正常任务和跳步任务中，对点位状态、任务状态、播报状态进行统一管理。

本文档与 ROS 侧方案配套使用：

- [ROS侧路线任务_任意跳步_播报协同实现方案.md](/home/ubuntu/humanoid_ws/src/humanoid_navigation2/docs/ROS侧路线任务_任意跳步_播报协同实现方案.md:1)

## 2. app 侧职责重构

### 2.1 从“逐点派单”改为“任务协同”

改造前：

1. app 发 A 点导航
2. ROS 到 A
3. app 决定播不播
4. app 再发 B 点导航

改造后：

1. app 一次下发完整路线任务
2. ROS 自主驱动路线执行
3. app 负责：
   - 展示当前任务进度
   - 收到 `broadcast_requested` 后开始播报
   - 播报结束后发 `broadcast_finished`
   - 用户点击跳步时发 `jump_to_waypoint`

### 2.2 app 侧新的边界

app 负责：

1. 生成 `task_session_id`
2. 组装整轮路线任务
3. 把每个点位的播报属性一起发给 ROS
4. 维护当前任务 UI 状态
5. 执行播报
6. 发送跳步、暂停、恢复、终止等业务命令

app 不再负责：

1. 到 A 后发 B
2. 到 B 后发 C
3. 到点后再临时决定这个点是不是应该让 ROS 继续跑下一点

## 3. websocket 协议总原则

建议继续沿用现有统一消息封装格式：

1. `message_type`
   - `command`
   - `push`
   - `response`
2. `data_type`
   - 对导航控制命令统一使用 `navigation_control`
   - 对 ROS 推送的导航状态统一使用 `navigation_status`

这样改动最小，能复用现有 websocket server 和 data integration 的大部分基础能力。

## 4. app -> ROS 的关键命令

### 4.1 启动整轮路线任务

app 发给 websocket server 的完整消息建议如下：

```json
{
  "protocol_version": "1.0",
  "message_id": "command_1718000000_ab12cd34",
  "timestamp": 1718000000.123,
  "message_type": "command",
  "data_type": "navigation_control",
  "source": "app",
  "destination": "websocket_server",
  "data": {
    "command_type": "start_route_task",
    "task_session_id": "tour_20260610_001",
    "route_id": "route_exhibition_001",
    "route_waypoints": [
      {
        "waypoint_id": "A",
        "waypoint_name": "入口讲解点",
        "need_broadcast": true,
        "broadcast_id": "A_intro",
        "broadcast_blocking": true,
        "stop_and_align": true
      },
      {
        "waypoint_id": "B",
        "waypoint_name": "展位B",
        "need_broadcast": true,
        "broadcast_id": "B_intro",
        "broadcast_blocking": true,
        "stop_and_align": true
      },
      {
        "waypoint_id": "C",
        "waypoint_name": "过道点C",
        "need_broadcast": false,
        "broadcast_id": "",
        "broadcast_blocking": false,
        "stop_and_align": true
      },
      {
        "waypoint_id": "D",
        "waypoint_name": "展位D",
        "need_broadcast": true,
        "broadcast_id": "D_intro",
        "broadcast_blocking": true,
        "stop_and_align": true
      }
    ]
  },
  "metadata": {
    "status": "success",
    "error_code": "",
    "error_message": "",
    "request_id": "req_start_route_001",
    "data_freshness": 0.0,
    "qos_level": "standard"
  }
}
```

### 4.2 跳步命令

用户点击跳到目标点时，app 发：

```json
{
  "protocol_version": "1.0",
  "message_id": "command_1718000100_ef56gh78",
  "timestamp": 1718000100.234,
  "message_type": "command",
  "data_type": "navigation_control",
  "source": "app",
  "destination": "websocket_server",
  "data": {
    "command_type": "jump_to_waypoint",
    "task_session_id": "tour_20260610_001",
    "target_waypoint_id": "D",
    "interrupt_broadcast": true,
    "reason": "user_jump_request"
  },
  "metadata": {
    "status": "success",
    "error_code": "",
    "error_message": "",
    "request_id": "req_jump_001",
    "data_freshness": 0.0,
    "qos_level": "realtime"
  }
}
```

说明：

1. `command_type = jump_to_waypoint`
2. `data_type = navigation_control`
3. `interrupt_broadcast = true`
   - 表示如果当前正在播报，允许打断当前播报，直接切跳步

### 4.3 播报完成命令

这是本次需求最关键的命令之一。

当 app 收到 ROS 的 `broadcast_requested`，并且本地播报播放结束后，必须发送：

```json
{
  "protocol_version": "1.0",
  "message_id": "command_1718000200_ij90kl12",
  "timestamp": 1718000200.456,
  "message_type": "command",
  "data_type": "navigation_control",
  "source": "app",
  "destination": "websocket_server",
  "data": {
    "command_type": "broadcast_finished",
    "task_session_id": "tour_20260610_001",
    "waypoint_id": "B",
    "broadcast_id": "B_intro",
    "broadcast_result": "completed",
    "broadcast_duration_sec": 12.6
  },
  "metadata": {
    "status": "success",
    "error_code": "",
    "error_message": "",
    "request_id": "req_broadcast_finish_001",
    "data_freshness": 0.0,
    "qos_level": "realtime"
  }
}
```

字段说明：

1. `command_type`
   - 固定为 `broadcast_finished`
2. `task_session_id`
   - 当前任务 id
3. `waypoint_id`
   - 当前完成播报的点位
4. `broadcast_id`
   - 与 ROS 请求时的播报标识保持一致
5. `broadcast_result`
   - 建议默认 `completed`
   - 也可预留 `skipped`、`interrupted`
6. `broadcast_duration_sec`
   - 方便日志与对账

### 4.4 暂停、恢复、终止命令

这三类命令建议保留原有 `navigation_control` 大类，统一走任务级语义：

1. `pause_route_task`
2. `resume_route_task`
3. `stop_route_task`

建议也带上 `task_session_id`，避免误控。

## 5. ROS -> app 的关键推送

### 5.1 可播报通知

这是 app 必须新增处理的核心事件。

ROS 推送给 app 的 websocket 消息建议如下：

```json
{
  "protocol_version": "1.0",
  "message_id": "push_1718000188_mn34op56",
  "timestamp": 1718000188.400,
  "message_type": "push",
  "data_type": "navigation_status",
  "source": "data_integration",
  "destination": "all",
  "data": {
    "event_type": "broadcast_requested",
    "event_data": {
      "task_session_id": "tour_20260610_001",
      "route_id": "route_exhibition_001",
      "waypoint_id": "B",
      "waypoint_name": "展位B",
      "waypoint_index": 1,
      "broadcast_id": "B_intro",
      "broadcast_blocking": true,
      "execution_role": "target_stop",
      "need_broadcast": true,
      "stop_and_align_completed": true,
      "arrival_source": "target_stop_reached",
      "pose": {
        "x": 1.23,
        "y": 4.56,
        "yaw": 1.57
      }
    },
    "timestamp": 1718000188.321,
    "current_state": "waiting_broadcast",
    "navigation_mode": "route_task",
    "sequence_id": "tour_20260610_001"
  },
  "metadata": {
    "status": "success",
    "error_code": "",
    "error_message": "",
    "request_id": "",
    "data_freshness": 0.079,
    "qos_level": "realtime",
    "push_reason": "navigation_event"
  }
}
```

app 侧处理规则：

1. `message_type == "push"`
2. `data_type == "navigation_status"`
3. `data.event_type == "broadcast_requested"`

满足以上条件时，app 必须进入“当前点等待播报”逻辑。

### 5.2 途经点通知

当 ROS 只是经过某个中间点，而不需要停靠播报时，建议推送：

```json
{
  "message_type": "push",
  "data_type": "navigation_status",
  "data": {
    "event_type": "waypoint_passed",
    "event_data": {
      "task_session_id": "tour_20260610_001",
      "waypoint_id": "C",
      "waypoint_name": "过道点C",
      "execution_role": "pass_through"
    }
  }
}
```

app 收到后：

1. 更新路线进度 UI
2. 不触发播报
3. 不弹讲解卡片
4. 不把当前状态切到 `broadcasting`

### 5.3 跳步重规划通知

建议 ROS 推送：

```json
{
  "message_type": "push",
  "data_type": "navigation_status",
  "data": {
    "event_type": "route_jump_replanned",
    "event_data": {
      "task_session_id": "tour_20260610_001",
      "jump_from_waypoint_id": "A",
      "jump_target_waypoint_id": "D",
      "active_segment_waypoint_ids": ["B", "C", "D"],
      "pass_through_waypoint_ids": ["B", "C"],
      "target_stop_waypoint_id": "D"
    }
  }
}
```

app 收到后：

1. 刷新当前任务路线显示
2. 将 `B/C` 标记为“本次跳步中仅途经”
3. 将 `D` 标记为“本次跳步目标”

## 6. app 本地状态模型

建议 app 本地维护一份任务状态对象：

```json
{
  "task_session_id": "tour_20260610_001",
  "route_id": "route_exhibition_001",
  "task_status": "idle",
  "current_waypoint_id": "",
  "current_waypoint_name": "",
  "current_waypoint_index": 0,
  "awaiting_broadcast": false,
  "current_broadcast_id": "",
  "is_broadcasting": false,
  "master_route_waypoint_ids": ["A", "B", "C", "D"],
  "active_segment_waypoint_ids": [],
  "pass_through_waypoint_ids": [],
  "target_stop_waypoint_id": ""
}
```

字段说明：

1. `task_status`
   - 建议值：
   - `idle`
   - `running`
   - `navigating`
   - `waiting_broadcast`
   - `broadcasting`
   - `paused`
   - `jump_replanning`
   - `completed`
   - `failed`
2. `awaiting_broadcast`
   - ROS 已到点，等待 app 开始和完成播报
3. `is_broadcasting`
   - app 本地播放器正在播报
4. `active_segment_waypoint_ids`
   - 当前 ROS 正在执行的子路线
5. `pass_through_waypoint_ids`
   - 本段中仅途经、不播报的点

## 7. 正常执行时 app 的交互流程

### 7.1 启动任务

1. 用户点击开始
2. app 生成 `task_session_id`
3. app 组装完整 `route_waypoints`
4. app 发送 `start_route_task`
5. app 本地状态切到 `running`

### 7.2 收到导航进行中事件

1. 收到 `segment_started`
2. app 更新当前执行段
3. UI 高亮当前目标方向

### 7.3 收到途经点

1. 收到 `waypoint_passed`
2. 更新进度条
3. 不触发播报

### 7.4 收到可播报通知

1. 收到 `broadcast_requested`
2. app 校验 `task_session_id`、`waypoint_id`
3. 本地状态切到 `waiting_broadcast`
4. 立即开始对应 `broadcast_id` 的播报
5. 本地状态切到 `broadcasting`

### 7.5 播报完成

1. 播放器结束
2. app 发送 `broadcast_finished`
3. 本地状态先切回 `running`
4. 等待 ROS 后续新的 `segment_started` 或 `navigation_status` 推送

## 8. 跳步时 app 的交互流程

### 8.1 正常导航中跳步

1. 用户点击目标点 `D`
2. app 发送 `jump_to_waypoint`
3. 本地状态切到 `jump_replanning`
4. 收到 `route_jump_replanned`
5. 更新当前子路线 UI
6. 收到新的 `segment_started`
7. 状态恢复为 `navigating`

### 8.2 正在播报时跳步

若 app 当前正在播报，且用户点击跳步：

1. 若业务选择允许中断播报
   - 立即停止本地播放器
   - 发送 `jump_to_waypoint`，带 `interrupt_broadcast=true`
2. app 本地应清理：
   - `is_broadcasting=false`
   - `awaiting_broadcast=false`
   - `current_broadcast_id=""`
3. 等待 ROS 返回 `route_jump_replanned`

## 9. app 如何判断某点是否应该开始播报

app 不应该自己根据“到点了”盲猜是否要播报。

正确规则是：

1. 只有收到 ROS 的 `broadcast_requested` 才开始播报。
2. 即使主路线中这个点平时是讲解点，只要本次执行里 ROS 没发 `broadcast_requested`，app 就不能播。

这正好解决跳步时的需求：

1. `B/C` 在主路线里可能是播报点
2. 但这次跳步执行中，它们被 ROS 降级成 `pass_through`
3. ROS 只会发 `waypoint_passed`
4. app 因为没有收到 `broadcast_requested`，所以不会播报

## 10. app UI 改造建议

### 10.1 任务视图

建议新增以下展示：

1. 当前任务 id
2. 当前执行段
3. 当前点位状态：
   - 导航中
   - 等待播报
   - 正在播报
   - 已途经
4. 本次跳步目标点
5. 本次跳步中哪些点只是途经

### 10.2 路线列表样式

建议对每个点位提供明确状态标记：

1. `未执行`
2. `执行中`
3. `已播报完成`
4. `本次跳步仅途经`
5. `本次跳步目标`

### 10.3 跳步交互

用户点击某点跳步时，建议弹确认：

1. `将沿原路线跳转到 D`
2. `中间点 B、C 本次仅途经，不播报`

这样用户对结果更有预期。

## 11. websocket_server 需要配套修改的地方

当前 `websocket_server.py` 在 `navigation_control` 下发时，只扁平化了有限字段。

为了支持本方案，app 侧联调前需要确认服务端补充透传以下字段：

1. `task_session_id`
2. `route_id`
3. `route_waypoints`
4. `target_waypoint_id`
5. `interrupt_broadcast`
6. `broadcast_id`
7. `broadcast_result`
8. `broadcast_duration_sec`
9. `reason`

否则即使 app 发了完整 JSON，也会在 websocket server 转 ROS topic 时被丢字段。

## 12. app 侧错误处理建议

### 12.1 收到 `broadcast_requested` 但找不到播报资源

建议：

1. app 立即上报本地错误
2. 可选择发送 `broadcast_finished`
3. `broadcast_result` 设为 `skipped`

示例：

```json
{
  "data": {
    "command_type": "broadcast_finished",
    "task_session_id": "tour_20260610_001",
    "waypoint_id": "B",
    "broadcast_id": "B_intro",
    "broadcast_result": "skipped",
    "broadcast_duration_sec": 0.0
  }
}
```

### 12.2 播报被手动打断

若用户主动跳步导致播报中断：

1. 优先发送 `jump_to_waypoint`
2. 不建议再单独补发一个 `broadcast_finished(interrupted)`，避免与跳步逻辑打架
3. 由 ROS 侧 `interrupt_broadcast=true` 接管切换

### 12.3 收到过期任务的推送

app 必须校验：

1. `task_session_id`
2. `route_id`

若推送不属于当前正在显示的任务：

1. 不更新当前 UI
2. 仅记录日志

## 13. 开发落地顺序

### 阶段 1：协议层

1. 支持 `start_route_task`
2. 支持 `jump_to_waypoint`
3. 支持 `broadcast_finished`
4. 支持处理 `broadcast_requested`

### 阶段 2：状态管理

1. app 本地任务状态对象上线
2. 区分 `waiting_broadcast` 与 `broadcasting`
3. 区分 `waypoint_passed` 与 `broadcast_requested`

### 阶段 3：UI 与体验

1. 路线状态展示
2. 跳步结果展示
3. 播报等待提示
4. 途经点标识

## 14. 最终结论

app 侧改造的核心不是“加一个跳步按钮”，而是角色转变：

1. app 从“逐点派发导航”改为“整轮任务协同者”
2. app 从“到点后决定下一步去哪”改为“收到 ROS 的业务触发后执行播报并确认完成”

关键实现点如下：

1. app 一次性下发整轮任务，包含每个点的播报属性。
2. ROS 到正式停靠点后通过：
   - `message_type = push`
   - `data_type = navigation_status`
   - `event_type = broadcast_requested`
   通知 app 开始播报。
3. app 播报完成后通过：
   - `message_type = command`
   - `data_type = navigation_control`
   - `command_type = broadcast_finished`
   通知 ROS 继续执行。
4. 跳步时，app 只发目标点；是否让中间点变成途经点，由 ROS 基于任务上下文决定。
5. app 只根据 `broadcast_requested` 决定是否播报，从而自动兼容“中间播报点被跳步降级成途经点”的场景。

这套方案能和 ROS 侧的路线任务模型严格对齐，方便后续直接进入接口联调和代码实现。
