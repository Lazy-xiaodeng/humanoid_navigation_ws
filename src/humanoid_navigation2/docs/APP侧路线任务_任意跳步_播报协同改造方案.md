# APP 侧路线任务、任意跳步与播报协同改造方案

## 1. 文档目的

本文档定义 app 侧针对最终确认需求的详细改造方案，目标是让 app 与 ROS 侧围绕“整轮路线任务 + 任意跳步 + 辅助点自动吸收 + 播报协同”形成统一实现。

本文档配套 ROS 侧方案使用：

- [ROS侧路线任务_任意跳步_播报协同实现方案.md](/home/ubuntu/humanoid_ws/src/humanoid_navigation2/docs/ROS侧路线任务_任意跳步_播报协同实现方案.md:1)

## 2. 最终业务语义

### 2.1 点位分两类

app 侧需要支持客户在配置点位时，显式设置点位类型：

1. `task`
   - 任务点
   - 有讲解/业务语义
   - 可触发播报
   - 可作为跳步目标
   - 完成后推进主任务进度
2. `transit`
   - 辅助点 / 途经点
   - 无讲解业务语义
   - 不触发播报
   - 不作为跳步目标
   - 只用于导航 through point

### 2.2 正常执行规则

当 app 一次性下发整轮路线任务后，ROS 按主任务序列自主执行。

若主路径中存在：

```text
10(task) -> 11(transit) -> 12(transit) -> 13(task)
```

则实际执行效果应为：

```text
10 -> 11 -> 12 -> 13
```

其中：

1. `10` 和 `13` 是任务点
2. `11` 和 `12` 是辅助点
3. 辅助点不播报、不停车等待、不做最终对齐

### 2.3 跳步执行规则

若当前正在执行任务，用户选择从 `A` 跳到 `D`：

1. 不再执行中间任务点 `B/C`
2. ROS 直接把 `D` 作为新的目标任务点
3. 但如果 `A` 与 `D` 之间按顺序存在辅助点，则自动吸收进入执行段

例如：

```text
7(task) -> 11(transit) -> 12(transit) -> 15(task)
```

当用户从 `7` 跳到 `15` 时，实际执行为：

```text
7 -> 11 -> 12 -> 15
```

而不是：

```text
7 -> 8 -> 9 -> 10 -> 13 -> 14 -> 15
```

### 2.4 跳步完成后的续走规则

跳到哪个任务点，哪个任务点完成后，就从它继续沿主任务序列向后执行。

示例：

1. `A -> D`
   - `D` 完成后继续 `E`
2. `F -> B`
   - `B` 完成后继续 `C`

## 3. APP 侧职责变化

### 3.1 从逐点派单改成整轮协同

改造前：

1. app 发 A 点
2. ROS 到 A
3. app 决定播报
4. app 再发 B 点

改造后：

1. app 一次性下发整轮路线任务
2. ROS 自主执行整轮路径
3. app 负责播报、跳步、暂停恢复、状态展示

### 3.2 APP 负责的内容

app 负责：

1. 生成整轮任务并下发
2. 为每个点位携带点位类型 `waypoint_role`
3. 为任务点携带播报属性
4. 展示当前主任务进度与当前执行状态
5. 收到 ROS 的“可播报通知”后执行播报
6. 播报完成后通知 ROS 继续
7. 用户点击跳步时发送目标任务点

app 不再负责：

1. 到一个点后再补发下一个点
2. 在每个点临时决定 ROS 是否继续跑下一点
3. 逐点串联控制整轮导航

## 4. APP 侧数据模型建议

### 4.1 点位配置模型

建议 app 点位配置中至少增加以下字段：

```json
{
  "waypoint_id": "11",
  "waypoint_name": "实验室门前对正点",
  "waypoint_role": "transit",
  "need_broadcast": false,
  "broadcast_id": "",
  "broadcast_blocking": false,
  "stop_and_align": false
}
```

任务点示例：

```json
{
  "waypoint_id": "13",
  "waypoint_name": "实验室讲解点",
  "waypoint_role": "task",
  "need_broadcast": true,
  "broadcast_id": "lab_intro",
  "broadcast_blocking": true,
  "stop_and_align": true
}
```

字段建议解释：

1. `waypoint_role`
   - `task` 或 `transit`
2. `need_broadcast`
   - 是否需要播报
3. `broadcast_id`
   - 对应播报资源 id
4. `broadcast_blocking`
   - 播报是否阻塞继续执行
5. `stop_and_align`
   - 到该点是否需要停靠和最终对齐

约束建议：

1. `transit` 点默认 `need_broadcast = false`
2. `transit` 点默认 `stop_and_align = false`
3. `task` 点允许配置播报
4. 跳步目标只能选 `task` 点

### 4.2 路线配置模型

建议路线编辑结果仍按用户在 UI 中的保存顺序输出，顺序本身即为吸收辅助点时的默认依据。

例如：

```text
1(task), 2(task), 3(task), 10(task), 11(transit), 12(transit), 13(task)
```

则 app 无需额外计算跳步时的 through path，只需要把完整有序列表发给 ROS，ROS 按顺序识别中间辅助点。

## 5. websocket 协议总原则

建议继续沿用当前统一封装：

1. `message_type`
   - `command`
   - `push`
   - `response`
2. `data_type`
   - 导航控制使用 `navigation_control`
   - 导航状态使用 `navigation_status`

这样对现有 websocket server / data integration 改动最小。

## 6. app -> ROS 的关键命令

### 6.1 启动整轮路线任务

app 一次性下发整轮任务，完整消息建议如下：

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
        "waypoint_id": "10",
        "waypoint_name": "展厅出口讲解点",
        "waypoint_role": "task",
        "need_broadcast": true,
        "broadcast_id": "hall_exit_intro",
        "broadcast_blocking": true,
        "stop_and_align": true
      },
      {
        "waypoint_id": "11",
        "waypoint_name": "门前对正点",
        "waypoint_role": "transit",
        "need_broadcast": false,
        "broadcast_id": "",
        "broadcast_blocking": false,
        "stop_and_align": false
      },
      {
        "waypoint_id": "12",
        "waypoint_name": "过门中继点",
        "waypoint_role": "transit",
        "need_broadcast": false,
        "broadcast_id": "",
        "broadcast_blocking": false,
        "stop_and_align": false
      },
      {
        "waypoint_id": "13",
        "waypoint_name": "实验室讲解点",
        "waypoint_role": "task",
        "need_broadcast": true,
        "broadcast_id": "lab_intro",
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

app 侧要求：

1. `route_waypoints` 必须保持配置顺序
2. 每个点必须带 `waypoint_role`
3. `transit` 点也要一起下发，不能省略

### 6.2 跳步命令

用户点击跳步时，app 只需要告诉 ROS “跳到哪个任务点”，不需要由 app 自己计算中间辅助点。

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

说明：

1. `target_waypoint_id` 必须是 `task` 点
2. `interrupt_broadcast = true`
   - 若当前处于播报中，允许打断当前播报并切换跳步
3. app 不需要发送 `11/12` 这类辅助点列表
   - 由 ROS 根据完整有序路线自动吸收

### 6.3 播报完成命令

当 app 收到 ROS 的 `broadcast_requested`，并完成本地播报后，必须发送：

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
    "waypoint_id": "13",
    "broadcast_id": "lab_intro",
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
2. `waypoint_id`
   - 当前播报完成的任务点
3. `broadcast_result`
   - 建议至少支持：
   - `completed`
   - `skipped`
   - `interrupted`

### 6.4 暂停、恢复、终止命令

建议保留任务级控制：

1. `pause_route_task`
2. `resume_route_task`
3. `stop_route_task`

统一带上 `task_session_id`。

## 7. ROS -> app 的关键推送

### 7.1 可播报通知

ROS 到达需要播报的任务点，并完成停车/对齐后，向 app 推送：

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
      "waypoint_id": "13",
      "waypoint_name": "实验室讲解点",
      "waypoint_role": "task",
      "broadcast_id": "lab_intro",
      "need_broadcast": true,
      "blocking": true
    }
  },
  "metadata": {
    "status": "success",
    "error_code": "",
    "error_message": "",
    "request_id": "",
    "data_freshness": 0.0,
    "qos_level": "realtime"
  }
}
```

app 收到后执行：

1. 更新 UI 为“等待播报 / 正在播报”
2. 启动本地播报
3. 播报完成后发送 `broadcast_finished`

### 7.2 辅助点通过通知

ROS 经过辅助点时，建议可选推送：

```json
{
  "protocol_version": "1.0",
  "message_id": "push_1718000190_pass001",
  "timestamp": 1718000190.100,
  "message_type": "push",
  "data_type": "navigation_status",
  "source": "data_integration",
  "destination": "all",
  "data": {
    "event_type": "waypoint_passed",
    "event_data": {
      "task_session_id": "tour_20260610_001",
      "waypoint_id": "11",
      "waypoint_name": "门前对正点",
      "waypoint_role": "transit"
    }
  },
  "metadata": {
    "status": "success",
    "error_code": "",
    "error_message": "",
    "request_id": "",
    "data_freshness": 0.0,
    "qos_level": "standard"
  }
}
```

app 可用于：

1. 轨迹 UI 展示
2. 调试日志
3. 客户侧可视化

但收到该事件后：

1. 不播报
2. 不弹等待
3. 不要求 app 发继续命令

### 7.3 跳步生效通知

建议 ROS 在跳步切换完成后推送：

```json
{
  "protocol_version": "1.0",
  "message_id": "push_1718000195_jump001",
  "timestamp": 1718000195.300,
  "message_type": "push",
  "data_type": "navigation_status",
  "source": "data_integration",
  "destination": "all",
  "data": {
    "event_type": "jump_updated",
    "event_data": {
      "task_session_id": "tour_20260610_001",
      "from_waypoint_id": "7",
      "target_waypoint_id": "15",
      "transit_waypoint_ids": ["11", "12"],
      "resume_main_route_from_waypoint_id": "15"
    }
  },
  "metadata": {
    "status": "success",
    "error_code": "",
    "error_message": "",
    "request_id": "",
    "data_freshness": 0.0,
    "qos_level": "realtime"
  }
}
```

app 可据此：

1. 更新当前目标点 UI
2. 显示本次跳步实际执行段
3. 明确后续会从 `15` 继续往后执行

### 7.4 任务点完成通知

建议 ROS 在任务点真正完成后推送：

```json
{
  "protocol_version": "1.0",
  "message_id": "push_1718000210_done001",
  "timestamp": 1718000210.500,
  "message_type": "push",
  "data_type": "navigation_status",
  "source": "data_integration",
  "destination": "all",
  "data": {
    "event_type": "task_waypoint_completed",
    "event_data": {
      "task_session_id": "tour_20260610_001",
      "waypoint_id": "13",
      "waypoint_role": "task",
      "next_task_waypoint_id": "14"
    }
  },
  "metadata": {
    "status": "success",
    "error_code": "",
    "error_message": "",
    "request_id": "",
    "data_freshness": 0.0,
    "qos_level": "standard"
  }
}
```

## 8. APP 侧交互逻辑

### 8.1 正常执行

1. app 下发整轮任务
2. ROS 自主前往下一个任务点
3. 若途中经过辅助点，只更新状态，不需要 app 干预
4. ROS 到达需要播报的任务点后推送 `broadcast_requested`
5. app 开始播报
6. app 播报结束后发送 `broadcast_finished`
7. ROS 继续执行下一段

### 8.2 跳步执行

1. 用户在 app 上选择跳步目标任务点
2. app 发送 `jump_to_waypoint`
3. ROS 按完整有序路线识别当前点与目标点之间的辅助点
4. ROS 形成新的执行段
5. 若存在辅助点，则无痕 through
6. 到达目标任务点后，如需播报则推送 `broadcast_requested`
7. 目标任务点完成后，ROS 从该目标点继续向后执行主任务序列

## 9. APP UI 改造建议

### 9.1 点位配置页

需要新增：

1. 点位类型设置
   - `任务点`
   - `辅助点`
2. 播报属性设置
   - 是否播报
   - 播报资源
3. 停靠对齐属性设置
   - 是否需要停靠对齐

建议限制：

1. 辅助点默认不允许配置播报
2. 辅助点默认不允许作为跳步目标

### 9.2 路线执行页

建议展示：

1. 当前任务 session
2. 当前目标任务点
3. 当前是否处于播报等待中
4. 当前是否正在执行跳步
5. 本次跳步吸收的辅助点列表

### 9.3 跳步选择交互

建议：

1. 跳步弹窗只展示 `task` 点
2. 不展示 `transit` 点作为可跳转目标
3. 用户选中后直接发 `jump_to_waypoint`

## 10. APP 侧实现拆分建议

### 10.1 数据层

需要改造：

1. 路线点位模型增加 `waypoint_role`
2. 播报模型与路线任务模型打通
3. websocket 协议模型新增：
   - `start_route_task`
   - `jump_to_waypoint`
   - `broadcast_finished`
   - `broadcast_requested`
   - `waypoint_passed`
   - `jump_updated`
   - `task_waypoint_completed`

### 10.2 业务层

需要改造：

1. 从逐点派发改为整轮任务派发
2. 增加任务 session 状态机
3. 增加播报等待与播报完成回传逻辑
4. 增加跳步命令下发逻辑
5. 增加跳步后 UI 状态刷新逻辑

### 10.3 UI 层

需要改造：

1. 点位配置界面新增点位类型字段
2. 路线执行界面新增当前目标点/跳步态展示
3. 播报等待态与播报完成态展示

## 11. 联调重点

联调时要重点验证以下场景：

1. 正常路线：
   - `10(task) -> 11(transit) -> 12(transit) -> 13(task)`
   - 确认 `11/12` 不播报、不停顿
2. 正向跳步：
   - 当前 `7`，跳到 `15`
   - 确认中间任务点不执行，仅自动吸收中间辅助点
3. 反向跳步：
   - 当前 `15`，跳到 `7`
   - 确认完成 `7` 后继续 `8`
4. 播报阻塞：
   - 到任务点后不收到 `broadcast_finished` 时，ROS 不继续
5. 跳步打断播报：
   - 播报进行中时触发跳步，确认 app 与 ROS 状态一致

## 12. 结论

app 侧这次的核心不是“多一个跳步按钮”，而是整体从“逐点派发导航”升级为“整轮路线任务协同”。

最终落地模型应统一为：

1. app 一次下发完整有序路线
2. 点位显式区分 `task` / `transit`
3. ROS 负责执行与自动吸收中间辅助点
4. app 负责播报和用户交互
5. 跳步后从目标任务点继续向后执行

只要 app 侧按本文档完成任务模型、UI 配置能力和 websocket 交互改造，就能与 ROS 侧形成一致且丝滑的整体方案。
