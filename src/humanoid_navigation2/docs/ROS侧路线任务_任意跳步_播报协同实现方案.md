# ROS 侧路线任务、任意跳步与播报协同实现方案

## 1. 文档目的

本文档定义 ROS 侧针对以下最终需求的实现方案：

1. app 一次下发整轮任务，而不是逐点派单。
2. 支持任意跳步：
   - `A -> D`
   - `D -> B`
   - `A -> G`
   - `G -> C`
3. 跳步时直接以目标任务点为新的导航目标，不再要求按中间任务点顺序执行。
4. 仍然保留“辅助点 / 途经点”能力，用于过门、过窄通道、对正方向、回程引导。
5. 跳步时自动吸收当前点与目标点之间、按顺序出现的辅助点。
6. 正常任务执行时，机器人在需要播报的任务点停车、对齐、等待 app 播报完成后再继续。
7. 跳步目标点完成后，从该点继续沿主任务序列向后执行。

本文档只关注 ROS 侧。app 侧详细改造见：

- [APP侧路线任务_任意跳步_播报协同改造方案.md](/home/ubuntu/humanoid_ws/src/humanoid_navigation2/docs/APP侧路线任务_任意跳步_播报协同改造方案.md:1)

## 2. 最终业务语义

### 2.1 点位分两类

系统中的点位分为两类：

1. `任务点 task`
   - 有业务语义
   - 可加入主任务序列
   - 可触发播报
   - 可作为跳步目标
   - 完成后推进任务进度
2. `辅助点 transit`
   - 无讲解业务语义
   - 不触发播报
   - 不作为任务完成点
   - 只用于导航 through point

### 2.2 跳步时不再执行中间任务点

假设主任务序列为：

```text
[A, B, C, D, E, F, G]
```

若当前在 `A`，用户跳到 `D`：

1. 中间任务点 `B/C` 不再执行
2. ROS 直接以 `D` 作为新的目标任务点
3. 由导航侧重新规划从当前位置到 `D` 的路径

### 2.3 跳步时保留中间辅助点

若当前点与目标点之间按顺序存在辅助点，则辅助点仍自动加入执行路径。

例如：

```text
7(task) -> 11(transit) -> 12(transit) -> 15(task)
```

若当前在 `7`，用户跳到 `15`，则默认执行：

```text
7 -> 11 -> 12 -> 15
```

其中：

1. 中间任务点不执行
2. 中间辅助点自动作为 through points 插入

### 2.4 跳步后的任务推进规则

固定采用以下规则：

**跳到哪个任务点，哪个任务点完成后，就从它继续沿主任务序列向后执行。**

示例：

1. `A -> D`
   - `D` 完成后继续 `E`
2. `F -> B`
   - `B` 完成后继续 `C`

## 3. 总体设计原则

ROS 侧按以下原则重构：

1. 一轮讲解/巡航是一个 `route_task`。
2. `route_task` 保留原始完整主任务序列 `master_route`。
3. 跳步不会修改 `master_route` 本身，只会改变当前执行目标和执行路径。
4. 任务点与辅助点必须分类型管理。
5. ROS 自主驱动路线执行，app 不再负责发下一个导航点。
6. 正式停靠点才允许停车、最终对齐、播报等待。
7. 辅助点只作为 through point，不触发停车、最终对齐和播报。
8. 播报完成信号由 app 回传 ROS，ROS 再继续后续执行。

## 4. 核心数据模型

### 4.1 主任务序列

`master_route` 是用户配置好的原始完整有序任务点序列，例如：

```text
[1, 2, 3, 4, 5, 6, 7, 15, 16, 17]
```

这里强调：

1. `master_route` 只包含 `task` 点
2. `transit` 点不进入主任务序列

### 4.2 点位类型

建议每个点位至少增加如下属性：

```json
{
  "waypoint_id": "11",
  "name": "实验室门前对正点",
  "waypoint_role": "transit",
  "need_broadcast": false
}
```

任务点示例：

```json
{
  "waypoint_id": "15",
  "name": "实验室讲解点",
  "waypoint_role": "task",
  "need_broadcast": true,
  "broadcast_id": "lab_intro"
}
```

建议 `waypoint_role` 取值：

1. `task`
2. `transit`

### 4.3 当前执行路径

`active_segment` 是当前实际执行的一段路径，由以下元素组成：

1. 当前起点任务点
2. 中间自动吸收的辅助点
3. 最终目标任务点

例如：

```text
7(task) -> 11(transit) -> 12(transit) -> 15(task)
```

则当前执行段可表示为：

```json
{
  "segment_start_task_id": "7",
  "segment_target_task_id": "15",
  "segment_direction": "forward",
  "transit_waypoint_ids": ["11", "12"],
  "execution_waypoint_ids": ["11", "12", "15"]
}
```

### 4.4 任务状态对象

建议 ROS 内维护如下任务对象：

```json
{
  "task_session_id": "tour_20260610_001",
  "route_id": "route_exhibition_001",
  "master_route_task_ids": ["1", "2", "3", "4", "5", "6", "7", "15", "16"],
  "current_anchor_task_index": 0,
  "current_target_task_index": 0,
  "active_segment": {
    "segment_start_task_id": "7",
    "segment_target_task_id": "15",
    "segment_direction": "forward",
    "transit_waypoint_ids": ["11", "12"],
    "execution_waypoint_ids": ["11", "12", "15"]
  },
  "awaiting_broadcast": false
}
```

## 5. 默认吸收规则

### 5.1 顺序规则

建议当前版本采用“按顺序自动吸收辅助点”作为默认规则。

前提：

1. 客户在 UI 上按顺序添加点位
2. 保存后的点位序列本身带有顺序语义

### 5.2 正常顺序执行

若存在：

```text
10(task), 11(transit), 12(transit), 13(task)
```

则从 `10` 到 `13` 默认执行：

```text
10 -> 11 -> 12 -> 13
```

### 5.3 跳步执行

若当前在 `7`，用户跳到 `15`，并且中间顺序区间内有：

```text
11(transit), 12(transit)
```

则默认执行：

```text
7 -> 11 -> 12 -> 15
```

而不是：

```text
7 -> 8 -> 9 -> 10 -> 13 -> 14 -> 15
```

也不是乱序插入：

```text
7 -> 11 -> 12 -> 8 -> 9
```

### 5.4 吸收算法

从当前任务点到目标任务点之间：

1. 中间的 `task` 点在跳步时不执行
2. 中间的 `transit` 点自动作为 through point 加入
3. `transit` 点加入时严格按顺序执行

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
        "waypoint_id": "7",
        "waypoint_role": "task",
        "need_broadcast": true,
        "broadcast_id": "point_7_intro",
        "broadcast_blocking": true,
        "stop_and_align": true
      },
      {
        "waypoint_id": "11",
        "waypoint_role": "transit",
        "need_broadcast": false,
        "broadcast_blocking": false,
        "stop_and_align": false
      },
      {
        "waypoint_id": "12",
        "waypoint_role": "transit",
        "need_broadcast": false,
        "broadcast_blocking": false,
        "stop_and_align": false
      },
      {
        "waypoint_id": "15",
        "waypoint_role": "task",
        "need_broadcast": true,
        "broadcast_id": "point_15_intro",
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

ROS 收到后：

1. 校验所有点位存在。
2. 校验 `waypoint_role` 合法。
3. 构造只包含 `task` 点的 `master_route_task_ids`。
4. 保存所有点位的原始顺序与属性。
5. 启动第一段执行。

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
    "target_waypoint_id": "15",
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
2. 校验目标点存在且 `waypoint_role=task`。
3. 查找当前锚点任务点与目标任务点在顺序序列中的区间。
4. 过滤出该区间内的 `transit` 点。
5. 构造新的 `active_segment`。
6. 取消当前未完成的导航 action。
7. 启动新的执行段。

### 6.3 播报完成命令

建议新增命令 `broadcast_finished`：

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
    "waypoint_id": "15",
    "broadcast_id": "point_15_intro",
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

ROS 收到后：

1. 校验当前正处于 `WAITING_BROADCAST`。
2. 校验 `task_session_id` 与 `waypoint_id` 匹配。
3. 清除等待态。
4. 将该任务点设置为新的任务进度锚点。
5. 继续执行后续任务点。

## 7. ROS 通知 app “已到达可播报点位”的协议

建议继续沿用：

1. `message_type = push`
2. `data_type = navigation_status`
3. `event_type = broadcast_requested`

内部 ROS 事件示例：

```json
{
  "event_type": "broadcast_requested",
  "event_data": {
    "task_session_id": "tour_20260610_001",
    "route_id": "route_exhibition_001",
    "waypoint_id": "15",
    "waypoint_name": "实验室讲解点",
    "waypoint_index": 7,
    "broadcast_id": "point_15_intro",
    "broadcast_blocking": true,
    "execution_role": "target_stop",
    "need_broadcast": true,
    "stop_and_align_completed": true,
    "arrival_source": "target_stop_reached"
  },
  "timestamp": 1718000188.321,
  "current_state": "waiting_broadcast",
  "navigation_mode": "route_task",
  "sequence_id": "tour_20260610_001"
}
```

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

其中：

1. `waypoint_passed`
   - 仅用于 `transit` 点
   - 只更新 UI 进度
2. `target_waypoint_reached`
   - 仅用于 `task` 点
   - 表示本段正式停靠完成
3. `broadcast_requested`
   - 仅对需要播报的 `task` 点发送

## 9. 任务状态机设计

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

## 10. 任意跳步算法

### 10.1 正向跳步

假设顺序序列中：

```text
7(task), 8(task), 9(task), 10(task), 11(transit), 12(transit), 13(task), 14(task), 15(task)
```

若当前锚点为 `7`，目标为 `15`，则：

1. 中间 `task` 点 `8 9 10 13 14` 不执行
2. 中间 `transit` 点 `11 12` 自动吸收

最终执行段：

```text
7 -> 11 -> 12 -> 15
```

### 10.2 反向跳步

假设顺序序列中：

```text
B(task), C(task), D(task), E(transit), F(transit), G(task)
```

若当前锚点为 `G`，目标为 `B`，则：

1. 中间 `task` 点 `C/D` 不执行
2. 中间 `transit` 点 `F/E` 自动吸收

最终执行段按顺序反向取值：

```text
G -> F -> E -> B
```

### 10.3 边界情况

1. 跳到当前锚点自身
   - 建议直接返回“已在目标点”
2. 跳到不存在的点
   - 直接拒绝
3. 跳到 `transit` 点
   - 建议直接拒绝
4. 当前任务未启动时发跳步
   - 直接拒绝
5. 当前任务完成或失败后发跳步
   - 直接拒绝

### 10.4 跳步后的任务推进规则

固定采用以下规则：

**跳到哪个任务点，哪个任务点完成后，就从它继续沿主任务序列向后执行。**

示例：

1. `A -> D`
   - `D` 完成后继续 `E`
2. `F -> B`
   - `B` 完成后继续 `C`

## 11. 播报协同规则

### 11.1 正常任务执行时如何判断某点是否需要播报

答案是：由 app 在任务开始时把 `task` 点的播报属性一并下发给 ROS，ROS 不在到点时临时询问。

建议每个 `task` 点至少带以下字段：

1. `need_broadcast`
2. `broadcast_id`
3. `broadcast_blocking`

`transit` 点固定不播报。

### 11.2 ROS 到点后的处理

1. 若到达的是 `transit` 点
   - 只发 `waypoint_passed`
   - 不停车等待
2. 若到达的是 `task` 点且 `effective_need_broadcast=false`
   - 直接进入下一段执行
3. 若到达的是 `task` 点且 `effective_need_broadcast=true`
   - 发布 `broadcast_requested`
   - 进入 `WAITING_BROADCAST`

## 12. 导航执行策略

### 12.1 为什么要采用 `NavigateThroughPoses`

为了尽量丝滑，中间多个 `transit` 点不能各自作为独立 `NavigateToPose` 串行执行。

推荐策略：

1. 本段最终目标是 `task` 点
2. 中间 `transit` 点作为 through poses
3. 通过 `NavigateThroughPoses` 一次执行整段

### 12.2 示例

若当前执行段为：

```text
7 -> 11 -> 12 -> 15
```

则：

1. `11`、`12` 作为 through poses
2. `15` 作为最终任务目标点

必要时可采用“两阶段执行”：

1. 先用 `NavigateThroughPoses` 连续通过所有 `transit` 点，到达 `15` 附近
2. 再用单独的 `NavigateToPose` 完成 `15` 的最终对齐

## 13. ROS 节点改造建议

### 13.1 `dynamic_waypoints_manager.py`

新增支持的命令：

1. `start_route_task`
2. `jump_to_waypoint`
3. `broadcast_finished`
4. `pause_route_task`
5. `resume_route_task`
6. `stop_route_task`

新增或透传的关键字段：

1. `task_session_id`
2. `route_id`
3. `route_waypoints`
4. `target_waypoint_id`
5. `interrupt_broadcast`
6. `broadcast_id`
7. `broadcast_result`
8. `broadcast_duration_sec`
9. `waypoint_role`

### 13.2 `navigation_state_manager.py`

建议新增能力：

1. 维护 `route_task` 内存态
2. 保存 `master_route_task_ids`
3. 保存完整顺序点位表
4. 支持 `task/transit` 点位类型
5. 实现“顺序区间自动吸收辅助点”
6. 管理 `WAITING_BROADCAST`
7. 支持 `NavigateThroughPoses` / `NavigateToPose` 混合执行

建议新增方法：

1. `handle_start_route_task()`
2. `handle_jump_to_waypoint()`
3. `handle_broadcast_finished()`
4. `build_active_segment()`
5. `collect_transit_waypoints_between_tasks()`
6. `start_active_segment_navigation()`
7. `handle_waypoint_passed()`
8. `handle_target_waypoint_reached()`
9. `advance_after_broadcast()`

### 13.3 websocket / data integration 节点

需要扩展以下能力：

1. 推送新的任务级状态事件
2. 将 `broadcast_requested` 明确转发给 app
3. 将 `waypoint_passed` 与 `target_waypoint_reached` 区分处理
4. 同步任务级字段：
   - `task_session_id`
   - `current_anchor_task_id`
   - `current_target_task_id`
   - `awaiting_broadcast`
   - `active_segment`

## 14. 失败与异常处理

### 14.1 跳步中断播报

若当前处于 `WAITING_BROADCAST` 且收到跳步：

1. 若 `interrupt_broadcast=true`
   - 立即结束等待态
   - 切换到新执行段
2. 若 `interrupt_broadcast=false`
   - 记录挂起跳步请求
   - 当前播报完成后再切换

### 14.2 导航失败

导航失败时建议恢复粒度从“单路点失败”升级到“当前执行段失败”：

1. 记录失败任务点、失败执行段和所属任务
2. 可选择：
   - 重试当前段
   - 重新跳步到目标段末任务点
   - 终止整轮任务

### 14.3 播报超时

建议增加：

1. `broadcast_wait_timeout_sec = 120`

超时后策略可配置：

1. 继续执行
2. 进入失败态
3. 再次通知 app

## 15. 建议的实现阶段

### 阶段 1：任务模型与协议重构

目标：

1. 引入 `route_task`
2. app 一次下发整轮任务
3. ROS 维护任务点主序列与点位类型
4. 引入 `broadcast_requested` / `broadcast_finished`

### 阶段 2：点位类型与辅助点吸收

目标：

1. 支持 `task/transit` 点位类型
2. 正常段自动吸收中间 `transit`
3. 跳步段自动吸收中间 `transit`

### 阶段 3：任意跳步

目标：

1. 支持任意前跳、后跳
2. 跳步时跳过中间任务点
3. 跳步目标点完成后从该点继续往后执行

### 阶段 4：丝滑执行与异常处理

目标：

1. `NavigateThroughPoses` 丝滑执行
2. 播报超时策略
3. 并发保护和重试

## 16. 最终结论

本需求的最终实现语义是：

1. 主任务序列只由 `task` 点组成
2. `transit` 点由客户手动设置，用于导航 through
3. 跳步时直接以目标任务点为新的导航目标
4. 跳步时不执行中间任务点
5. 跳步时自动吸收当前点与目标点之间按顺序出现的辅助点
6. 辅助点不停车、不播报、不计入任务完成点
7. 目标任务点完成后，从该点继续沿主任务序列向后执行

一句话总结：

**跳步时直接去目标任务点，但自动保留中间辅助点作为无痕途经点；目标任务点完成后，从该点继续往后执行任务。**
