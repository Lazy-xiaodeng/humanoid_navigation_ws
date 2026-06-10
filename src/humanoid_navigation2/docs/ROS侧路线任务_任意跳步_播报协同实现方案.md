# ROS 侧路线任务、任意跳步与播报协同实现方案

## 1. 文档目的

本文档定义 ROS 侧针对以下需求的完整实现方案：

1. app 一次下发整轮任务，而不是逐点派单。
2. 支持任意跳步：
   - `A -> D`
   - `D -> B`
   - `A -> G`
   - `G -> C`
3. 跳步后仍然沿原始完整路线经过中间点，而不是直接抄近路到目标点。
4. 正常任务执行时，机器人在需要播报的点位停车、对齐、等待 app 播报完成后再继续。
5. 跳步执行时，目标点之前原本需要播报的中间点自动降级为“途经点”，不停车、不对齐、不播报。
6. 整体执行尽量丝滑，优先采用 `NavigateThroughPoses` / 整段路线跟随，而不是逐点串行 `NavigateToPose`。

本文档只关注 ROS 侧。app 侧详细改造见：

- [APP侧路线任务_任意跳步_播报协同改造方案.md](/home/ubuntu/humanoid_ws/src/humanoid_navigation2/docs/APP侧路线任务_任意跳步_播报协同改造方案.md:1)

## 2. 背景与现状问题

当前系统的问题不只是“能不能改索引从 A 跳到 D”，而是整轮任务的驱动方式本身不适合丝滑执行。

当前链路本质是：

1. app 下发当前点位导航。
2. ROS 到点后上报完成。
3. app 决定是否播报。
4. app 不播报或播报完成后，再下发下一个点位。

这套模式的局限：

1. 路线推进权在 app，而不在 ROS。
2. ROS 无法长期持有“整轮完整路线”的上下文。
3. ROS 无法在运行中基于原始路线重建任意跳步子路线。
4. 每个点都是独立任务，中间天然存在切换停顿。
5. 需要播报和不需要播报的业务逻辑，无法在 ROS 内统一建模。

因此，若要实现“任意跳步 + 丝滑经过中间点 + 播报协同”，必须把导航抽象从“单点命令”升级为“路线任务”。

## 3. 总体设计原则

ROS 侧按以下原则重构：

1. 一轮讲解/巡航是一个 `route_task`。
2. `route_task` 必须保存原始完整主路线 `master_route`。
3. 所有跳步都基于 `master_route` 的索引截段来构造子路线。
4. 主路线属性和本次执行属性必须分离。
5. ROS 自主驱动路线执行，app 不再负责发下一个导航点。
6. 正式停靠点才允许停车、最终对齐、播报等待。
7. 跳步时，目标点之前的中间点全部临时降级为途经点。
8. 途经点只作为 through poses / checkpoint，不触发停车、最终对齐和播报。
9. 播报完成信号由 app 回传 ROS，ROS 再继续后续执行。

## 4. 核心概念

### 4.1 主路线

`master_route` 是用户配置好的原始完整有序路线，例如：

```text
[A, B, C, D, E, F, G]
```

它始终保留，不因为跳步而被覆盖。

### 4.2 子路线

`active_segment` 是当前实际执行的一段子路线，由 `master_route` 截取而来。

示例：

1. 当前从 `A` 跳到 `D`
   - `active_segment = [B, C, D]`
2. 当前从 `D` 跳到 `B`
   - `active_segment = [C, B]`
3. 正常从 `B` 执行到 `C`
   - `active_segment = [C]`

### 4.3 正式停靠点

正式停靠点指本次执行上下文中真正要“停下来完成业务”的点，具有如下行为：

1. 到点后允许停车。
2. 需要最终朝向对齐。
3. 若配置 `effective_need_broadcast=true`，则触发播报等待。
4. 完成后才会进入后续段。

### 4.4 途经点

途经点指本次执行上下文中仅作为路线连续性控制点的点，具有如下行为：

1. 不停车。
2. 不做最终朝向对齐。
3. 不触发播报。
4. 不产生“任务停靠完成”语义。

### 4.5 主路线属性 vs 执行属性

必须区分两层属性。

主路线属性是静态定义：

```json
{
  "waypoint_id": "B",
  "need_broadcast": true,
  "broadcast_id": "B_intro",
  "default_stop_and_align": true
}
```

执行属性是运行时根据当前任务上下文动态计算得到：

```json
{
  "waypoint_id": "B",
  "execution_role": "pass_through",
  "effective_need_broadcast": false,
  "effective_stop_and_align": false
}
```

最终执行必须只看 `effective_*` 字段，不直接使用主路线原始属性。

## 5. ROS 侧任务模型

建议在 ROS 内维护统一任务对象：

```json
{
  "task_session_id": "tour_20260610_001",
  "route_id": "route_exhibition_001",
  "master_route": [
    {
      "waypoint_id": "A",
      "need_broadcast": true,
      "broadcast_id": "A_intro",
      "broadcast_blocking": true,
      "default_stop_and_align": true
    },
    {
      "waypoint_id": "B",
      "need_broadcast": true,
      "broadcast_id": "B_intro",
      "broadcast_blocking": true,
      "default_stop_and_align": true
    },
    {
      "waypoint_id": "C",
      "need_broadcast": false,
      "broadcast_blocking": false,
      "default_stop_and_align": true
    }
  ],
  "current_anchor_index": 0,
  "current_target_index": 0,
  "active_segment": [],
  "active_segment_direction": "forward",
  "awaiting_broadcast": false
}
```

字段说明：

1. `task_session_id`
   - 一轮任务唯一 id。
2. `master_route`
   - 原始完整有序路线。
3. `current_anchor_index`
   - 当前确认到达或作为跳步起点的锚点索引。
4. `current_target_index`
   - 本次子路线的最终目标索引。
5. `active_segment`
   - 当前正在执行的子路线。
6. `active_segment_direction`
   - `forward` 或 `backward`。
7. `awaiting_broadcast`
   - 当前是否在等待 app 的播报完成信号。

## 6. 导航控制命令协议

### 6.1 启动整轮任务

建议新增任务级命令 `start_route_task`。

app 通过 websocket 发给服务端：

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
        "need_broadcast": true,
        "broadcast_id": "A_intro",
        "broadcast_blocking": true,
        "stop_and_align": true
      },
      {
        "waypoint_id": "B",
        "need_broadcast": true,
        "broadcast_id": "B_intro",
        "broadcast_blocking": true,
        "stop_and_align": true
      },
      {
        "waypoint_id": "C",
        "need_broadcast": false,
        "broadcast_blocking": false,
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

`websocket_server` 路由到 `/app/navigation_command` 时，需要把以下字段原样透传给 `dynamic_waypoints_manager`：

```json
{
  "command_type": "start_route_task",
  "task_session_id": "tour_20260610_001",
  "route_id": "route_exhibition_001",
  "route_waypoints": [
    {
      "waypoint_id": "A",
      "need_broadcast": true,
      "broadcast_id": "A_intro",
      "broadcast_blocking": true,
      "stop_and_align": true
    }
  ],
  "client_id": "client_xxx",
  "timestamp": 1718000000.123
}
```

ROS 收到该命令后：

1. 校验所有点位存在。
2. 加载完整 `master_route`。
3. 初始化任务状态。
4. 启动第一段执行。

### 6.2 跳步命令

建议新增命令 `jump_to_waypoint`：

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

ROS 收到后：

1. 校验当前任务 id 一致。
2. 查找目标点在 `master_route` 中的索引。
3. 以 `current_anchor_index` 为起点、`target_index` 为终点重建子路线。
4. 中间点自动改成 `pass_through`。
5. 最终目标点改成 `target_stop`。
6. 取消当前未完成的导航 action。
7. 启动新的 `active_segment`。

### 6.3 播报完成命令

建议新增命令 `broadcast_finished`，由 app 在播报结束后发送。

app 通过 websocket 发送：

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

`websocket_server` 路由到 `/app/navigation_command` 后，建议发布的 ROS topic 内容为：

```json
{
  "command_type": "broadcast_finished",
  "task_session_id": "tour_20260610_001",
  "waypoint_id": "B",
  "broadcast_id": "B_intro",
  "broadcast_result": "completed",
  "broadcast_duration_sec": 12.6,
  "client_id": "client_xxx",
  "timestamp": 1718000200.456
}
```

ROS 收到后：

1. 校验当前正处于 `WAITING_BROADCAST`。
2. 校验 `task_session_id` 与 `waypoint_id` 匹配。
3. 清除等待态。
4. 继续执行下一段子路线。

## 7. ROS 通知 app “已到达可播报点位”的协议

### 7.1 内部 ROS 事件

建议 `navigation_state_manager` 在到达正式停靠点、且 `effective_need_broadcast=true` 时，先向 `/navigation/status` 发布事件：

```json
{
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
}
```

### 7.2 app 侧收到的 websocket 推送

`data_integration_node_recoverable.py` 已经支持把事件型导航状态包装成 `message_type=push`、`data_type=navigation_status` 的 websocket 推送。为了兼容现有链路，建议继续沿用这个 `TYPE`：

1. `message_type = "push"`
2. `data_type = "navigation_status"`
3. `data.event_type = "broadcast_requested"`

app 最终收到的完整 websocket 消息建议如下：

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

### 7.3 为什么继续走 `navigation_status`

建议继续走 `navigation_status`，而不是新开一个 `broadcast_event`，原因是：

1. 现有 websocket 整合链路已经对 `navigation_status` 做了实时事件推送。
2. app 现有导航订阅逻辑更容易复用。
3. `broadcast_requested` 本质仍是导航任务状态变化的一部分。
4. 可减少 websocket 服务端和整合节点的新增分支。

## 8. 状态事件建议

ROS 对 app 的推送建议新增以下事件：

1. `route_task_started`
2. `segment_started`
3. `waypoint_passed`
4. `target_waypoint_reached`
5. `broadcast_requested`
6. `route_jump_replanned`
7. `route_task_paused`
8. `route_task_resumed`
9. `route_task_completed`
10. `route_task_failed`

其中关键区别如下：

1. `waypoint_passed`
   - 中间途经点事件。
   - 仅用于 UI 显示，不触发业务停靠。
2. `target_waypoint_reached`
   - 正式停靠点事件。
   - 到点后可能进入播报等待。
3. `broadcast_requested`
   - ROS 明确告诉 app：当前点需要播报，请开始业务播放。

## 9. 任务状态机设计

建议 ROS 侧路线任务状态机如下：

### 9.1 主状态

1. `IDLE`
2. `TASK_RUNNING`
3. `SEGMENT_NAVIGATING`
4. `WAITING_BROADCAST`
5. `TASK_PAUSED`
6. `TASK_JUMP_REPLANNING`
7. `TASK_FAILED`
8. `TASK_COMPLETED`

### 9.2 状态流转

正常执行：

```text
IDLE
  -> TASK_RUNNING
  -> SEGMENT_NAVIGATING
  -> target_waypoint_reached
  -> WAITING_BROADCAST   (如果需要播报)
  -> SEGMENT_NAVIGATING  (收到 broadcast_finished 后)
  -> TASK_COMPLETED
```

跳步执行：

```text
SEGMENT_NAVIGATING or WAITING_BROADCAST
  -> TASK_JUMP_REPLANNING
  -> SEGMENT_NAVIGATING
```

### 9.3 锚点更新规则

`current_anchor_index` 的更新建议如下：

1. 正常正式停靠点到达后，更新为该点索引。
2. 跳步期间，途中 `pass_through` 点不作为业务锚点，但可作为导航进度锚点。
3. 若当前在 `WAITING_BROADCAST`，则当前停靠点已视为确认锚点。
4. 若机器人正在两个点之间运动时收到跳步命令，起点锚点建议取“当前最后确认有效锚点”。

## 10. 任意跳步算法

### 10.1 正向跳步

主路线：

```text
[A, B, C, D, E, F, G]
```

若当前锚点为 `A`，目标为 `D`：

```text
active_segment = [B, C, D]
direction = forward
```

执行属性：

1. `B` -> `pass_through`
2. `C` -> `pass_through`
3. `D` -> `target_stop`

### 10.2 反向跳步

若当前锚点为 `D`，目标为 `B`：

```text
active_segment = [C, B]
direction = backward
```

执行属性：

1. `C` -> `pass_through`
2. `B` -> `target_stop`

### 10.3 边界情况

1. 跳到当前锚点自身
   - 若当前在播报等待，可选择继续等待或重新触发播报。
   - 建议默认忽略，返回“已在目标点”。
2. 跳到不存在于 `master_route` 的点
   - 直接拒绝。
3. 当前任务未启动时发跳步
   - 直接拒绝。
4. 当前任务完成或失败后发跳步
   - 直接拒绝。

## 11. 播报协同规则

### 11.1 正常任务执行时如何判断某点是否需要播报

答案是：由 app 在任务开始时把该点的播报属性一并下发给 ROS，ROS 不在到点时临时询问。

建议每个点至少带以下字段：

1. `need_broadcast`
2. `broadcast_id`
3. `broadcast_blocking`

ROS 到达正式停靠点后：

1. 若 `effective_need_broadcast=false`
   - 直接进入下一段执行。
2. 若 `effective_need_broadcast=true`
   - 发布 `broadcast_requested`
   - 进入 `WAITING_BROADCAST`
   - 停止推进后续导航，直到收到 `broadcast_finished`

### 11.2 为什么不建议到点时再问 app

如果到点时再让 app 决策，会增加以下复杂度：

1. 状态机要多一个“等待业务决定是否播报”的中间状态。
2. 每次到点都要承受一次网络往返延迟。
3. 正常任务与跳步任务会出现两套决策逻辑。

因此建议：

1. app 在任务开始前就把静态播报属性下发给 ROS。
2. ROS 到点后只根据 `effective_*` 执行。

## 12. 跳步时如何把原本要播报的中间点降级成途经点

这是本方案的关键。

假设主路线定义为：

```text
A(stop+broadcast), B(stop+broadcast), C(stop+broadcast), D(stop+broadcast)
```

若当前从 `A` 跳到 `D`，则重建子路线后：

1. `B`：`execution_role = pass_through`
2. `C`：`execution_role = pass_through`
3. `D`：`execution_role = target_stop`

同时做如下覆盖：

1. `B.effective_stop_and_align = false`
2. `B.effective_need_broadcast = false`
3. `C.effective_stop_and_align = false`
4. `C.effective_need_broadcast = false`
5. `D.effective_stop_and_align = true`
6. `D.effective_need_broadcast = D.need_broadcast`

也就是说：

主路线中 B/C 虽然本来是讲解点，但在本次跳步上下文中，它们只是导航途经点。

## 13. 导航执行策略

### 13.1 为什么要采用 `NavigateThroughPoses`

为了尽量丝滑，中间多个途经点不能再各自作为独立 `NavigateToPose` 任务串行执行。

推荐策略：

1. 把一个连续子路线段一次性交给 Nav2。
2. 中间点作为 through poses。
3. 段末正式停靠点作为本段最终业务目标点。

这样可减少：

1. 每个点单独下发 action 的切换延迟。
2. 每个点到达后重新建树和状态切换的停顿。
3. 中间点因独立目标收敛而带来的明显减速。

### 13.2 子路线分段原则

建议每次只执行到“下一个正式停靠点”为止。

示例 1：正常执行

```text
master_route = [A, B, C, D]
```

若每个点都要播报，则每段都只有一个正式停靠点：

1. 当前段：到 `A`
2. 播报完成
3. 下一段：到 `B`
4. 播报完成
5. 下一段：到 `C`

示例 2：跳步 `A -> D`

```text
active_segment = [B, C, D]
```

其中：

1. `B/C` 为 through poses
2. `D` 为本段正式停靠点

这一段整体作为一次 `NavigateThroughPoses` 执行。

### 13.3 途经点与停靠点的行为树差异

推荐准备两类 BT：

1. `navigate_through_pass_points.xml`
   - 用于连续 through poses 段。
   - 中间点不触发最终 yaw 对齐。
2. `navigate_xy_then_yaw.xml`
   - 用于正式停靠点最终姿态对齐。

推荐实现方式：

1. 中间途经阶段：
   - 只关注路径连续通过，不做最终姿态对齐。
2. 最终停靠阶段：
   - 到目标点后允许最终朝向对齐。

如果 `NavigateThroughPoses` 的最后一个 goal 也会触发不必要的全段收敛停顿，可采用“两阶段段内策略”：

1. 先用 `NavigateThroughPoses` 连续通过所有中间途经点，到达目标点附近 XY。
2. 再用单独的 `NavigateToPose` 完成正式停靠点最终对齐。

这样仍然比“B/C/D 全部分别独立 `NavigateToPose`”更丝滑。

## 14. ROS 节点改造建议

### 14.1 `dynamic_waypoints_manager.py`

职责升级建议：

1. 从“点位管理 + 简单导航请求转发”
2. 升级为“点位管理 + 路线任务命令入口”

新增支持的命令：

1. `start_route_task`
2. `jump_to_waypoint`
3. `broadcast_finished`
4. `pause_route_task`
5. `resume_route_task`
6. `stop_route_task`

主要改造点：

1. 校验整轮任务中的所有点位是否存在。
2. 把每个点位的 route-level 属性透传给状态管理器。
3. 不再默认把多点任务拆成 app 逐点推进模式。

### 14.2 `navigation_state_manager.py`

这是 ROS 侧的核心改造节点。

建议新增能力：

1. 维护 `route_task` 内存态。
2. 保存 `master_route`。
3. 维护 `current_anchor_index` / `current_target_index`。
4. 实现任意跳步子路线重建。
5. 区分 `pass_through` 与 `target_stop`。
6. 管理 `WAITING_BROADCAST`。
7. 支持 `NavigateThroughPoses` / `NavigateToPose` 混合执行策略。

建议新增方法：

1. `handle_start_route_task()`
2. `handle_jump_to_waypoint()`
3. `handle_broadcast_finished()`
4. `build_active_segment()`
5. `apply_execution_overrides()`
6. `start_active_segment_navigation()`
7. `handle_segment_waypoint_passed()`
8. `handle_target_waypoint_reached()`
9. `advance_after_broadcast()`

### 14.3 `websocket_server.py`

该节点当前对 `navigation_control` 的扁平化字段较少，仅支持：

1. `waypoint_id`
2. `waypoint_ids`
3. `exhibition_ids`

为了支持路线任务与播报协同，建议扩展透传字段：

1. `task_session_id`
2. `route_id`
3. `route_waypoints`
4. `target_waypoint_id`
5. `interrupt_broadcast`
6. `broadcast_id`
7. `broadcast_result`
8. `broadcast_duration_sec`
9. `reason`

### 14.4 数据整合与推送节点

相关节点需要扩展以下能力：

1. 推送新的任务级状态事件。
2. 将 `broadcast_requested` 明确转发给 app。
3. 将 `waypoint_passed` 与 `target_waypoint_reached` 区分处理。
4. 同步任务级字段：
   - `task_session_id`
   - `current_anchor_waypoint_id`
   - `current_target_waypoint_id`
   - `awaiting_broadcast`
   - `active_segment`

## 15. 失败与异常处理

### 15.1 跳步中断播报

若当前处于 `WAITING_BROADCAST` 且收到跳步：

1. 若 `interrupt_broadcast=true`
   - 立即结束等待态。
   - 发布 `route_jump_replanned`。
   - 切换到新子路线执行。
2. 若 `interrupt_broadcast=false`
   - 记录挂起跳步请求。
   - 当前播报完成后再切路线。

推荐默认支持 `interrupt_broadcast=true`，因为客户“跳步”通常意味着强业务优先级。

### 15.2 导航失败

导航失败时建议保留当前系统的可恢复思路，但恢复粒度从“单路点失败”升级到“当前子路线失败”：

1. 记录失败点、失败段和所属任务。
2. 可选择：
   - 重新执行当前子路线
   - 跳过到当前段最终目标
   - 终止整轮任务

### 15.3 播报超时

建议为 `WAITING_BROADCAST` 增加超时参数，例如：

1. `broadcast_wait_timeout_sec = 120`

超时后策略可配置：

1. 继续执行
2. 进入失败态
3. 再次通知 app

建议默认：

1. 先推送一次超时告警
2. 再等待一小段缓冲
3. 最终进入任务失败或人工处理态

## 16. 建议的实现阶段

### 阶段 1：任务模型与协议重构

目标：

1. 引入 `route_task`
2. app 一次下发整轮任务
3. ROS 维护 `master_route`
4. 引入 `broadcast_requested` / `broadcast_finished`

先不要求最丝滑，只先跑通新的任务驱动关系。

### 阶段 2：任意跳步与途经点降级

目标：

1. 支持任意前跳、后跳
2. 子路线重建
3. 中间点动态降级为 `pass_through`
4. 跳步时屏蔽中间点停车、对齐、播报

### 阶段 3：`NavigateThroughPoses` 丝滑执行

目标：

1. 连续 through poses 段改为 `NavigateThroughPoses`
2. 正式停靠点单独处理最终停靠语义
3. 消除逐点串行 `NavigateToPose` 带来的明显切换顿挫

### 阶段 4：优化与恢复

目标：

1. 播报超时策略
2. 跳步中的异常恢复
3. 子路线失败重试
4. 任务级监控与日志分析

## 17. 最终结论

本需求若要真正满足客户预期，ROS 侧必须从“逐点导航命令执行器”升级为“路线任务执行器”。

关键点总结如下：

1. 原始完整路线必须常驻保存为 `master_route`。
2. 任意跳步本质是基于 `master_route` 的索引截段，而不是简单改当前目标点。
3. 正向跳、反向跳必须统一支持。
4. 主路线属性和本次执行属性必须分离。
5. 正常执行时，点位是否播报由 app 在任务开始时一次性告诉 ROS。
6. ROS 到正式停靠点后通过 `broadcast_requested` 与 app 协同，并等待 `broadcast_finished`。
7. 跳步时，目标点之前的中间点统一降级为 `pass_through`，不停车、不对齐、不播报。
8. 若要尽量丝滑，连续子路线必须优先采用 `NavigateThroughPoses` / 整段路线跟随方案。

这套方案能同时覆盖：

1. 正常任务逐站讲解。
2. 任意正向跳步。
3. 任意反向跳步。
4. 播报等待与继续。
5. 跳步时中间讲解点自动变途经点。

它也是后续 app 侧协议、状态展示和异常处理的统一基础。
