# APP 侧多地图一期改造方案

## 1. 文档目的

本文档给 APP 前端、APP 后端和 ROS 侧联调用。内容按当前 Todesk 工作区源码整理，重点覆盖：

1. APP 前端需要新增或调整哪些 UI 和状态逻辑。
2. APP 后端需要如何保存地图/点位/路线数据，并转换成 ROS 当前协议。
3. 前端、后端、ROS 三者在切图、点位、导航任务中的完整交互流程。
4. 所有涉及多地图一期的 JSON 消息示例，字段尽量完整展开，可直接按字段实现。

当前 ROS 侧已经实现的是“导航前切图”，不是“导航中跨地图切图”。切图时 ROS 控制层常驻，导航定位层按目标地图重启。

## 2. 当前 ROS 侧真实能力

当前源码中的关键节点和入口：

1. `websocket_server.py` 接收 APP websocket 消息。
2. APP 发 `data_type=map_management` 时，websocket 转发到 ROS topic `/app/map_command`。
3. APP 发 `data_type=waypoint_management` 时，websocket 转发到 ROS topic `/app/waypoint_command`。
4. APP 发 `data_type=navigation_control` 时，websocket 转发到 ROS topic `/app/navigation_command`。
5. `map_context_manager.py` 处理地图查询和 `switch_map`。
6. `dynamic_waypoints_manager.py` 处理点位增删改查，并按 `map_id` 保存到 `data/waypoints/<map_id>.json`。
7. `navigation_state_manager.py` 只保留新 route task 导航协议：`start_route_task / pause_route_task / resume_route_task / stop_route_task / jump_to_waypoint / broadcast_finished`。

当前 ROS 主动回传给 APP 的关键数据：

1. `map_response`：地图命令响应，例如地图列表、当前地图、切图开始、切图完成、切图失败。
2. `map_status`：地图状态周期推送，例如当前地图、切图中、等待定位、ready。
3. `waypoint_response`：点位保存/更新/删除/清空/查询响应。
4. `waypoints_data`：点位库全量或变更推送，包含 `waypoints_revision`。
5. `navigation_status`：导航周期状态和 route task 事件，事件由 `data.event_type` 区分。

## 3. 一期业务边界

APP 侧必须遵守以下边界：

1. 用户先选择地图，再设置该地图下的点位。
2. 点位必须绑定 `map_id`，不同地图允许出现相同 `waypoint_id`。
3. 一轮路线任务只能属于一张地图，不能混入多个 `map_id`。
4. 切图必须在没有导航任务运行、没有暂停、没有等待播报时执行。
5. 切图完成并收到 `map_ready` 后，APP 才允许开始该地图路线任务。
6. 如果 APP 已经把点位完整保存到 ROS，本轮导航推荐只发 `route_waypoint_ids + waypoints_revision`，不要每次发送几十个完整点位。
7. 如果 APP 后端临时组装路线，也兼容发送完整 `route_waypoints`，但不能同时发送 `route_waypoints` 和 `route_waypoint_ids`。
8. 真实第二地图需要真实 2D 栅格地图和 3D prior pcd。复制版 `hall1` 只能证明切图链路，不等于真实定位效果。

## 4. 前端改造

### 4.1 地图选择区

导航页和点位管理页都需要显示地图上下文。

建议字段：

1. 当前 UI 选中地图：`selected_map_id`。
2. ROS 当前运行地图：`robot_current_map_id`，来自 `map_status.current_map_id`。
3. 地图状态：`map_state`，来自 `map_status.map_state`。
4. 定位状态：`localization_state`，来自 `map_status.localization_state`。

推荐 UI 文案：

1. `已选择地图：hall1`
2. `机器人当前地图：hall`
3. `地图状态：ready`
4. `定位状态：stable`

地图按钮建议：

1. `刷新地图列表`：发送 `get_map_list`。
2. `切换到此地图`：发送 `switch_map`。
3. `刷新当前地图`：发送 `get_current_map`。

### 4.2 切图状态栏

前端不能只看按钮是否点击成功，要监听 ROS 后续状态。

`map_state` 建议展示规则：

1. `ready`：地图就绪，可以开始导航。
2. `switching`：正在切换地图，导航按钮禁用。
3. `localization_resetting`：已重启导航定位层，正在注入初始位姿。
4. `waiting_localization`：等待 RO/OP 定位稳定。
5. `failed`：切图失败，显示 `map_response.data.error_code/message`。

`localization_state` 建议展示规则：

1. `restarting_navigation_stack`：正在重启导航定位层。
2. `waiting_localization`：等待定位健康。
3. `stable`：定位稳定。
4. `navigation_stack_restart_failed`：导航层重启失败。
5. `restart_failed`：切图脚本启动失败。

### 4.3 点位管理页面

点位列表必须按地图过滤。

新增或编辑点位时，前端至少要让用户配置：

1. `waypoint_id`：点位 ID，建议字符串。
2. `waypoint_name`：点位名称。
3. `map_id`：所属地图，等于当前页面选中地图。
4. `waypoint_role`：`task` 任务点，`transit` 辅助点。
5. `walk_direction`：`forward` 正走，`backward` 倒走。
6. `need_broadcast`：是否需要 APP 到点播报。
7. `broadcast_id`：需要播报时必填。
8. `broadcast_text`：APP 播放内容，ROS 不依赖但建议保存。
9. `position`：地图坐标 `[x, y, z]`。
10. `orientation`：四元数 `[x, y, z, w]`。

注意：

1. `transit` 辅助点不会停车播报，主要用于过门、过窄通道等路线约束。
2. `task` 任务点可以 `need_broadcast=false`，此时机器人仍会到点并最终对齐角度，但不会等待 APP 播报。
3. 第一版不要允许用户直接把一个点从 `hall` 改成 `hall1`，建议用复制/重新标点，避免坐标系错乱。

### 4.4 导航控制页面

导航控制按钮：

1. `开始`：发送 `start_route_task`。
2. `暂停`：发送 `pause_route_task`。
3. `继续`：发送 `resume_route_task`。
4. `终止`：发送 `stop_route_task`。
5. `跳点`：发送 `jump_to_waypoint`。

按钮是否可用建议：

1. `map_state != ready` 时禁用开始。
2. `robot_current_map_id != selected_map_id` 时提示先切图。
3. `navigation_status.is_active=true` 时禁止切图。
4. `navigation_status.pause_source=obstacle_wait/manual_escape` 时，继续按钮仍然走 `resume_route_task`。
5. 等待播报时，前端不应该点继续导航，而是播报结束后发 `broadcast_finished`。

### 4.5 前端本地状态对象建议

```jsonc
{
  "selected_map_id": "hall1",                 // UI 当前选中的地图
  "robot_current_map_id": "hall",             // ROS 当前运行地图，来自 map_status.current_map_id
  "map_state": "ready",                       // ready/switching/localization_resetting/waiting_localization/failed
  "localization_state": "stable",             // stable/waiting_localization/restarting_navigation_stack 等
  "task_session_id": "tour_20260613_001",     // 当前一轮任务 ID，由 APP 生成
  "route_id": "route_hall1_main",             // 当前路线 ID，由 APP 生成或后端保存
  "session_map_id": "hall1",                  // 当前任务锁定地图
  "task_status": "idle",                      // idle/switching_map/navigating/paused/broadcast_wait/completed/failed/stopped
  "waypoints_revision_by_map": {              // APP 记录的各地图 ROS 点位库版本
    "hall": "1781334566.203",                 // hall 点位版本
    "hall1": "initial_empty_hall1"             // hall1 点位版本
  },
  "current_waypoint_id": "",                  // 当前正在执行或等待播报的点位
  "last_error_code": "",                      // 最近一次 ROS 错误码
  "last_error_message": ""                    // 最近一次 ROS 错误说明
}
```

## 5. 后端改造

### 5.1 后端地图模型

后端需要保存地图列表和当前选择地图。

建议地图对象：

```jsonc
{
  "map_id": "hall1",                          // 地图唯一 ID，和 ROS map_registry.json 中 map_id 一致
  "display_name": "备用展厅 hall1",             // APP 展示名称
  "enabled": true,                            // 是否允许选择
  "description": "二号展厅地图",                // 备注说明
  "created_at": 1781334566.203,               // 后端创建时间
  "updated_at": 1781334566.203                // 后端更新时间
}
```

### 5.2 后端点位模型

后端保存点位时必须带 `map_id`。

推荐点位对象：

```jsonc
{
  "waypoint_id": "10",                        // 点位 ID，同一地图内唯一；不同地图可重复
  "waypoint_name": "展厅任务点10",              // 点位名称
  "map_id": "hall",                           // 所属地图 ID
  "waypoint_role": "task",                    // task=任务点；transit=辅助点
  "frame_id": "map",                          // 坐标系，当前固定 map
  "position": [1.0, 2.0, 0.0],                // x/y/z，单位米
  "orientation": [0.0, 0.0, 0.0, 1.0],        // 四元数 x/y/z/w
  "need_broadcast": true,                     // 是否需要 APP 播报
  "broadcast_id": "broadcast_10",             // 播报 ID，need_broadcast=true 时必填
  "broadcast_text": "欢迎来到展厅10号点",        // 播报文本，APP 使用，ROS 不依赖
  "broadcast_blocking": true,                 // 建议 true，ROS 等待 broadcast_finished
  "stop_and_align": true,                     // task 点 true；transit 点 false
  "walk_direction": "forward",                // forward=正走；backward=倒走
  "properties": {                             // 冗余属性，兼容旧前端/后端结构
    "map_id": "hall",                         // 同顶层 map_id
    "waypoint_role": "task",                  // 同顶层 waypoint_role
    "walk_direction": "forward"               // 同顶层 walk_direction
  }
}
```

### 5.3 后端路线模型

一条路线必须只属于一张地图。

```jsonc
{
  "route_id": "route_hall_main_001",           // 路线 ID
  "route_name": "hall 主路线",                 // 路线名称
  "map_id": "hall",                           // 路线所属地图
  "ordered_waypoint_ids": ["1", "2", "3"],     // 有序点位 ID，顺序就是路线拓扑
  "enabled": true,                            // 是否可执行
  "created_at": 1781334566.203,               // 创建时间
  "updated_at": 1781334566.203                // 更新时间
}
```

后端在点击开始前必须校验：

1. 路线不为空。
2. 路线全部点位都属于同一个 `map_id`。
3. 路线至少包含一个 `waypoint_role=task` 的任务点。
4. 如果使用 ID 列表模式，后端必须有该地图最新 `waypoints_revision`。

## 6. WebSocket 基础消息格式

APP 发给 ROS websocket 的外层格式统一如下。

```jsonc
{
  "protocol_version": "2.0",                  // 固定协议版本
  "message_id": "app_msg_1781334566203",      // APP 生成，全局唯一；ROS 会用它做 request_message_id
  "timestamp": 1781334566.203,                // APP 发送时间，秒级时间戳
  "message_type": "command",                  // command=request/控制命令
  "data_type": "map_management",              // map_management/waypoint_management/navigation_control
  "source": "app_backend",                    // 发送方，建议 app_backend
  "destination": "robot",                     // 目标，建议 robot
  "data": {},                                 // 具体业务数据，见后续章节
  "metadata": {                               // 可选元数据
    "operator_id": "user_001",                // 操作人 ID
    "client_id": "tablet_001",                // 客户端 ID
    "trace_id": "trace_1781334566203"         // 链路追踪 ID
  }
}
```

ROS 推给 APP 的外层格式通常如下。

```jsonc
{
  "protocol_version": "2.0",                  // 固定协议版本
  "message_id": "map_response_1781334566203", // ROS 生成的消息 ID
  "timestamp": 1781334566.203,                // ROS 发送时间
  "message_type": "response",                 // response 或 push
  "data_type": "map_response",                // map_response/map_status/waypoint_response/waypoints_data/navigation_status
  "source": "map_context_manager",            // ROS 来源节点
  "destination": "all",                       // all 或指定客户端
  "data": {},                                 // 具体业务数据
  "metadata": {                               // 标准元数据
    "status": "success",                      // success/error
    "error_code": "",                         // 错误码，成功为空
    "error_message": "",                      // 错误说明，成功为空
    "request_id": "app_msg_1781334566203"     // 对应 APP message_id/request_message_id
  }
}
```

## 7. 地图管理 JSON

### 7.1 查询地图列表

APP 发送：

```jsonc
{
  "protocol_version": "2.0",                  // 固定协议版本
  "message_id": "app_map_list_1781334566203", // APP 请求 ID
  "timestamp": 1781334566.203,                // 发送时间
  "message_type": "command",                  // 命令消息
  "data_type": "map_management",              // 地图管理命令
  "source": "app_backend",                    // APP 后端
  "destination": "robot",                     // ROS 机器人
  "data": {
    "command_type": "get_map_list",           // 查询地图列表
    "request_message_id": "app_map_list_1781334566203" // 可选；不传时 websocket 用外层 message_id
  },
  "metadata": {
    "operator_id": "user_001",                // 操作人
    "client_id": "tablet_001",                // 客户端
    "trace_id": "trace_map_list_001"          // 追踪 ID
  }
}
```

ROS 返回：

```jsonc
{
  "protocol_version": "2.0",                  // 固定协议版本
  "message_id": "map_response_1781334566500", // ROS 响应 ID
  "timestamp": 1781334566.500,                // 响应时间
  "message_type": "response",                 // 响应消息
  "data_type": "map_response",                // 地图响应
  "source": "map_context_manager",            // 来源节点
  "destination": "all",                       // 广播
  "data": {
    "status": "success",                      // success=查询成功
    "current_map_id": "hall",                 // ROS 当前激活地图
    "default_map_id": "hall",                 // 默认地图
    "maps": [
      {
        "map_id": "hall",                     // 地图 ID
        "display_name": "默认展厅地图",          // 展示名
        "enabled": true,                      // 是否启用
        "description": "当前默认运行地图",       // 描述
        "waypoints_file": "/home/ubuntu/software/Todesk/Files/humanoid_ws/data/waypoints/hall.json", // ROS 点位文件
        "map_yaml_file": "/home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_navigation2/maps/hall.yaml", // 2D 地图 yaml
        "map_pgm_file": "/home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_navigation2/maps/hall.pgm", // 2D 栅格图
        "open3d_prior_map_file": "/home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_navigation2/pcd/hall_open3d_grounded.pcd", // 3D 定位地图
        "initial_pose": {
          "frame_id": "map",                  // 初始位姿坐标系
          "position": [0.0, 0.0, 0.0],        // 初始位置
          "orientation": [0.0, 0.0, 0.0, 1.0] // 初始姿态四元数
        }
      }
    ],
    "command_type": "get_map_list",           // 对应命令
    "request_message_id": "app_map_list_1781334566203", // 对应 APP 请求
    "timestamp": 1781334566.500               // 业务时间戳
  },
  "metadata": {
    "status": "success",                      // 成功
    "error_code": "",                         // 无错误
    "error_message": "",                      // 无错误说明
    "request_id": "app_map_list_1781334566203" // 对应 APP 请求
  }
}
```

### 7.2 查询当前地图

APP 发送：

```jsonc
{
  "protocol_version": "2.0",                  // 固定协议版本
  "message_id": "app_get_current_map_1781334566203", // APP 请求 ID
  "timestamp": 1781334566.203,                // 发送时间
  "message_type": "command",                  // 命令消息
  "data_type": "map_management",              // 地图管理命令
  "source": "app_backend",                    // APP 后端
  "destination": "robot",                     // ROS 机器人
  "data": {
    "command_type": "get_current_map",        // 查询当前地图
    "request_message_id": "app_get_current_map_1781334566203" // 可选
  },
  "metadata": {
    "operator_id": "user_001",                // 操作人
    "client_id": "tablet_001",                // 客户端
    "trace_id": "trace_current_map_001"       // 追踪 ID
  }
}
```

ROS 返回：

```jsonc
{
  "protocol_version": "2.0",                  // 固定协议版本
  "message_id": "map_response_1781334566500", // ROS 响应 ID
  "timestamp": 1781334566.500,                // 响应时间
  "message_type": "response",                 // 响应消息
  "data_type": "map_response",                // 地图响应
  "source": "map_context_manager",            // 来源节点
  "destination": "all",                       // 广播
  "data": {
    "status": "success",                      // success=查询成功
    "current_map_id": "hall",                 // 当前激活地图
    "default_map_id": "hall",                 // 默认地图
    "current_map": {
      "map_id": "hall",                       // 当前地图 ID
      "display_name": "默认展厅地图",            // 展示名
      "enabled": true,                        // 是否启用
      "description": "当前默认运行地图",         // 描述
      "waypoints_file": "/home/ubuntu/software/Todesk/Files/humanoid_ws/data/waypoints/hall.json", // 点位文件
      "map_yaml_file": "/home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_navigation2/maps/hall.yaml", // 2D 地图
      "open3d_prior_map_file": "/home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_navigation2/pcd/hall_open3d_grounded.pcd", // 3D 地图
      "initial_pose": {
        "frame_id": "map",                    // 坐标系
        "position": [0.0, 0.0, 0.0],          // 初始位置
        "orientation": [0.0, 0.0, 0.0, 1.0]   // 初始姿态
      }
    },
    "command_type": "get_current_map",        // 对应命令
    "request_message_id": "app_get_current_map_1781334566203", // 对应请求
    "timestamp": 1781334566.500               // 业务时间
  },
  "metadata": {
    "status": "success",                      // 成功
    "error_code": "",                         // 无错误
    "error_message": "",                      // 无错误说明
    "request_id": "app_get_current_map_1781334566203" // 对应请求
  }
}
```

### 7.3 切换地图

APP 发送：

```jsonc
{
  "protocol_version": "2.0",                  // 固定协议版本
  "message_id": "app_switch_map_1781334566203", // APP 请求 ID
  "timestamp": 1781334566.203,                // 发送时间
  "message_type": "command",                  // 命令消息
  "data_type": "map_management",              // 地图管理命令
  "source": "app_backend",                    // APP 后端
  "destination": "robot",                     // ROS 机器人
  "data": {
    "command_type": "switch_map",             // 切换地图
    "target_map_id": "hall1",                 // 目标地图 ID
    "reason": "user_selected_map",            // 切图原因，ROS 透传/日志使用
    "request_message_id": "app_switch_map_1781334566203" // 可选；建议传
  },
  "metadata": {
    "operator_id": "user_001",                // 操作人
    "client_id": "tablet_001",                // 客户端
    "trace_id": "trace_switch_map_001"        // 追踪 ID
  }
}
```

ROS 首次接受切图返回：

```jsonc
{
  "protocol_version": "2.0",                  // 固定协议版本
  "message_id": "map_response_1781334566500", // ROS 响应 ID
  "timestamp": 1781334566.500,                // 响应时间
  "message_type": "response",                 // 响应消息
  "data_type": "map_response",                // 地图响应
  "source": "map_context_manager",            // 来源节点
  "destination": "all",                       // 广播
  "data": {
    "status": "success",                      // success=切图流程已启动，不等于定位已 ready
    "result_reason": "map_switch_restart_started", // 切图重启已开始
    "message": "切图已开始：控制层保持在线，导航定位层将按目标地图重启", // 展示文案
    "previous_map_id": "hall",                // 切图前地图
    "current_map_id": "hall1",                // ROS 已切换上下文到目标地图
    "target_map_id": "hall1",                 // 目标地图
    "map_yaml_file": "/home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_navigation2/maps/hall1.yaml", // 目标 2D 地图
    "open3d_prior_map_file": "/home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_navigation2/pcd/hall1_open3d_grounded.pcd", // 目标 3D 地图
    "map_switch_script": "/home/ubuntu/software/Todesk/Files/humanoid_ws/switch_navigation_map.sh", // ROS 切图脚本
    "command_type": "switch_map",             // 对应命令
    "request_message_id": "app_switch_map_1781334566203", // 对应 APP 请求
    "timestamp": 1781334566.500               // 业务时间
  },
  "metadata": {
    "status": "success",                      // 成功
    "error_code": "",                         // 无错误
    "error_message": "",                      // 无错误说明
    "request_id": "app_switch_map_1781334566203" // 对应请求
  }
}
```

ROS 切图完成返回：

```jsonc
{
  "protocol_version": "2.0",                  // 固定协议版本
  "message_id": "map_response_1781334572000", // ROS 响应 ID
  "timestamp": 1781334572.000,                // 响应时间
  "message_type": "response",                 // 响应消息
  "data_type": "map_response",                // 地图响应
  "source": "map_context_manager",            // 来源节点
  "destination": "all",                       // 广播
  "data": {
    "status": "success",                      // success=切图完成
    "result_reason": "map_ready",             // 地图 ready，APP 可开始导航
    "message": "地图切换完成，定位已稳定，可以开始导航", // 展示文案
    "current_map_id": "hall1",                // 当前地图
    "target_map_id": "hall1",                 // 目标地图
    "map_state": "ready",                     // 地图状态
    "localization_state": "stable",           // 定位状态
    "command_type": "switch_map",             // 对应命令
    "request_message_id": "app_switch_map_1781334566203", // 对应 APP 请求
    "timestamp": 1781334572.000               // 业务时间
  },
  "metadata": {
    "status": "success",                      // 成功
    "error_code": "",                         // 无错误
    "error_message": "",                      // 无错误说明
    "request_id": "app_switch_map_1781334566203" // 对应请求
  }
}
```

ROS 切图状态周期推送：

```jsonc
{
  "protocol_version": "2.0",                  // 固定协议版本
  "message_id": "map_status_1781334568000",   // ROS 推送 ID
  "timestamp": 1781334568.000,                // 推送时间
  "message_type": "push",                     // 主动推送
  "data_type": "map_status",                  // 地图状态
  "source": "map_context_manager",            // 来源节点
  "destination": "all",                       // 广播
  "data": {
    "status": "success",                      // 推送状态
    "current_map_id": "hall1",                // 当前地图
    "default_map_id": "hall",                 // 默认地图
    "map_state": "waiting_localization",      // ready/switching/localization_resetting/waiting_localization/failed
    "localization_state": "waiting_localization", // 定位状态
    "localization_stable_count": 1,            // 已连续稳定帧数
    "switch_target_map_id": "hall1",          // 正在切换的目标地图；无切图时为空
    "navigation_active": false,               // 是否存在导航任务，true 时拒绝切图
    "navigation_detailed_state": "IDLE",      // 导航详细状态
    "maps_count": 2,                          // 注册地图数量
    "timestamp": 1781334568.000               // 业务时间
  },
  "metadata": {
    "status": "success",                      // 成功
    "error_code": "",                         // 无错误
    "error_message": "",                      // 无错误说明
    "request_id": ""                          // 周期推送无请求 ID
  }
}
```

### 7.4 地图错误响应

缺地图文件示例：

```jsonc
{
  "protocol_version": "2.0",                  // 固定协议版本
  "message_id": "map_response_1781334566500", // ROS 响应 ID
  "timestamp": 1781334566.500,                // 响应时间
  "message_type": "response",                 // 响应消息
  "data_type": "map_response",                // 地图响应
  "source": "map_context_manager",            // 来源节点
  "destination": "all",                       // 广播
  "data": {
    "status": "error",                        // error=失败
    "error_code": "map_file_missing",         // 地图 yaml 不存在
    "message": "地图文件不存在: /home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_navigation2/maps/hall1.yaml", // 失败说明
    "current_map_id": "hall",                 // 当前地图
    "target_map_id": "hall1",                 // 目标地图
    "map_yaml_file": "/home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_navigation2/maps/hall1.yaml", // 缺失文件
    "command_type": "switch_map",             // 对应命令
    "request_message_id": "app_switch_map_1781334566203", // 对应 APP 请求
    "timestamp": 1781334566.500               // 业务时间
  },
  "metadata": {
    "status": "error",                        // 失败
    "error_code": "map_file_missing",         // 错误码
    "error_message": "地图文件不存在: /home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_navigation2/maps/hall1.yaml", // 错误说明
    "request_id": "app_switch_map_1781334566203" // 对应请求
  }
}
```

APP 需要识别的地图错误码：

1. `map_not_registered`：地图未注册。
2. `map_disabled`：地图被禁用。
3. `map_file_missing`：2D 地图 yaml 缺失。
4. `prior_map_file_missing`：3D 定位地图 pcd 缺失。
5. `map_switch_in_progress`：已有切图正在执行。
6. `map_switch_script_missing`：ROS 切图脚本缺失。
7. `map_switch_restart_failed`：导航定位层重启失败。
8. `map_switch_localization_timeout`：切图后等待定位稳定超时。
9. `map_switch_rejected_route_task_active`：当前导航/暂停/播报等待中，拒绝切图。
10. `unknown_map_command`：未知地图命令。
11. `invalid_map_command`：地图命令 JSON 解析失败。

## 8. 点位管理 JSON

### 8.1 设置点位

APP 发送：

```jsonc
{
  "protocol_version": "2.0",                  // 固定协议版本
  "message_id": "app_set_wp_10_1781334566203", // APP 请求 ID
  "timestamp": 1781334566.203,                // 发送时间
  "message_type": "command",                  // 命令消息
  "data_type": "waypoint_management",         // 点位管理
  "source": "app_backend",                    // APP 后端
  "destination": "robot",                     // ROS 机器人
  "data": {
    "command_type": "set_waypoint",           // 新增或覆盖点位
    "map_id": "hall",                         // 点位所属地图
    "waypoint_type": "manual",                // 点位分类，现阶段可固定 manual
    "waypoint_data": {
      "waypoint_id": "10",                    // 点位 ID
      "waypoint_name": "展厅任务点10",          // 点位名称
      "map_id": "hall",                       // 所属地图，必须和外层 map_id 一致
      "waypoint_role": "task",                // task=任务点；transit=辅助点
      "frame_id": "map",                      // 坐标系
      "position": [1.0, 2.0, 0.0],            // 位置 x/y/z
      "orientation": [0.0, 0.0, 0.0, 1.0],    // 姿态四元数 x/y/z/w
      "need_broadcast": true,                 // 是否需要播报
      "broadcast_id": "broadcast_10",         // 播报 ID
      "broadcast_text": "欢迎来到展厅10号点",    // 播报文本
      "broadcast_blocking": true,             // true=ROS 等待 APP broadcast_finished
      "stop_and_align": true,                 // task 点 true；transit 点 false
      "walk_direction": "forward",            // forward/backward
      "properties": {
        "map_id": "hall",                     // 冗余 map_id
        "waypoint_role": "task",              // 冗余 role
        "walk_direction": "forward"           // 冗余行走方向
      }
    }
  },
  "metadata": {
    "operator_id": "user_001",                // 操作人
    "client_id": "tablet_001",                // 客户端
    "trace_id": "trace_set_wp_10"             // 追踪 ID
  }
}
```

ROS 返回：

```jsonc
{
  "protocol_version": "2.0",                  // 固定协议版本
  "message_id": "response_1781334566",        // ROS 响应 ID
  "timestamp": 1781334566.500,                // 响应时间
  "message_type": "response",                 // 响应消息
  "data_type": "waypoint_response",           // 点位响应
  "source": "waypoints_manager",              // 来源节点
  "destination": "all",                       // 广播
  "data": {
    "response_type": "success",               // success/error
    "message": "路点保存成功",                  // 展示文案
    "map_id": "hall",                         // 操作地图
    "waypoints_revision": "1781334566.500",   // 操作后该地图点位库版本
    "result": {
      "map_id": "hall",                       // 操作地图
      "waypoints_revision": "1781334566.500", // 最新版本
      "waypoints_revisions_by_map": {
        "hall": "1781334566.500",             // hall 最新版本
        "hall1": "initial_empty_hall1"         // hall1 当前版本
      }
    }
  },
  "metadata": {
    "status": "success",                      // 成功
    "error_code": "",                         // 无错误
    "error_message": "",                      // 无错误说明
    "request_id": ""                          // 当前 waypoint_response 未强绑定外层请求 ID
  }
}
```

### 8.2 查询点位

APP 发送：

```jsonc
{
  "protocol_version": "2.0",                  // 固定协议版本
  "message_id": "app_get_wps_hall_1781334566203", // APP 请求 ID
  "timestamp": 1781334566.203,                // 发送时间
  "message_type": "command",                  // 命令消息
  "data_type": "waypoint_management",         // 点位管理
  "source": "app_backend",                    // APP 后端
  "destination": "robot",                     // ROS 机器人
  "data": {
    "command_type": "get_waypoints",          // 查询点位
    "map_id": "hall",                         // 查询哪张图
    "waypoint_type": "",                      // 可选，空表示全部类型
    "include_details": true                   // true=返回详细点位
  },
  "metadata": {
    "operator_id": "user_001",                // 操作人
    "client_id": "tablet_001",                // 客户端
    "trace_id": "trace_get_wps_hall"          // 追踪 ID
  }
}
```

ROS 会通过 `waypoint_response` 和/或 `waypoints_data` 返回。APP 应重点保存 `waypoints_revision`，后续 ID 列表启动需要它。

### 8.3 清空点位

只清空某张地图：

```jsonc
{
  "protocol_version": "2.0",                  // 固定协议版本
  "message_id": "app_clear_wps_hall_1781334566203", // APP 请求 ID
  "timestamp": 1781334566.203,                // 发送时间
  "message_type": "command",                  // 命令消息
  "data_type": "waypoint_management",         // 点位管理
  "source": "app_backend",                    // APP 后端
  "destination": "robot",                     // ROS 机器人
  "data": {
    "command_type": "clear_waypoints",        // 清空点位
    "map_id": "hall",                         // 只清空 hall
    "clear_scope": "current_map",             // current_map=只清当前地图；all_maps=清全部地图
    "waypoint_type": ""                       // 可选，空表示全部类型
  },
  "metadata": {
    "operator_id": "user_001",                // 操作人
    "client_id": "tablet_001",                // 客户端
    "trace_id": "trace_clear_wps_hall"        // 追踪 ID
  }
}
```

### 8.4 点位库推送

ROS 推送：

```jsonc
{
  "protocol_version": "2.0",                  // 固定协议版本
  "message_id": "push_1781334566",            // ROS 推送 ID
  "timestamp": 1781334566.500,                // 推送时间
  "message_type": "push",                     // 主动推送
  "data_type": "waypoints_data",              // 点位数据
  "source": "waypoints_manager",              // 来源节点
  "destination": "all",                       // 广播
  "data": {
    "update_type": "full_update",             // full_update/initial_load 等
    "timestamp": 1781334566.500,              // 业务时间
    "map_id": "hall",                         // 本次推送主地图
    "default_map_id": "hall",                 // 默认地图
    "waypoints_revision": "1781334566.500",   // 当前地图版本
    "waypoints_revisions_by_map": {
      "hall": "1781334566.500",               // hall 版本
      "hall1": "initial_empty_hall1"           // hall1 版本
    },
    "data": {
      "waypoints": {
        "manual": {
          "10": {
            "waypoint_id": "10",              // 点位 ID
            "waypoint_name": "展厅任务点10",    // 点位名
            "map_id": "hall",                 // 所属地图
            "waypoint_role": "task",          // task/transit
            "position": [1.0, 2.0, 0.0],      // 位置
            "orientation": [0.0, 0.0, 0.0, 1.0], // 姿态
            "need_broadcast": true,           // 是否播报
            "walk_direction": "forward"       // 行走方向
          }
        }
      },
      "waypoints_by_map": {
        "hall": {
          "manual": {
            "10": {
              "waypoint_id": "10",            // hall 下 10 号点
              "map_id": "hall"                // 所属地图
            }
          }
        },
        "hall1": {
          "manual": {}                        // hall1 当前无点位
        }
      }
    },
    "metadata": {
      "total_count": 25,                       // 当前地图点位数
      "total_count_all_maps": 25,              // 全部地图点位数
      "map_id": "hall",                       // 当前地图
      "waypoints_revision": "1781334566.500", // 当前地图版本
      "waypoints_revisions_by_map": {
        "hall": "1781334566.500",             // hall 版本
        "hall1": "initial_empty_hall1"         // hall1 版本
      }
    }
  },
  "metadata": {
    "status": "success",                      // 成功
    "error_code": "",                         // 无错误
    "error_message": "",                      // 无错误说明
    "request_id": ""                          // 推送无请求 ID
  }
}
```

## 9. 导航控制 JSON

### 9.1 开始路线任务，推荐 ID 列表模式

适用场景：导航前点位已经通过 `set_waypoint/update_waypoint` 保存到 ROS。

APP 发送：

```jsonc
{
  "protocol_version": "2.0",                  // 固定协议版本
  "message_id": "app_start_route_1781334566203", // APP 请求 ID
  "timestamp": 1781334566.203,                // 发送时间
  "message_type": "command",                  // 命令消息
  "data_type": "navigation_control",          // 导航控制
  "source": "app_backend",                    // APP 后端
  "destination": "robot",                     // ROS 机器人
  "data": {
    "command_type": "start_route_task",       // 开始路线任务
    "task_session_id": "tour_20260613_001",   // 一轮任务 ID，APP 生成
    "route_id": "route_hall_main_001",        // 路线 ID
    "map_id": "hall",                         // 路线所属地图，必须等于当前 ROS ready 地图
    "waypoints_revision": "1781334566.500",   // APP 当前记录的 hall 点位版本，ID 模式必填
    "route_waypoint_ids": [                   // 有序点位 ID，顺序就是路线拓扑
      "1",                                    // 第 1 个点
      "2",                                    // 第 2 个点
      "3",                                    // 第 3 个点
      "10",                                   // 第 10 个点
      "11",                                   // 辅助点也在路线里
      "12",                                   // 辅助点也在路线里
      "13"                                    // 任务点
    ],
    "route_waypoints": [],                    // ID 模式必须为空或不传，不能同时传完整点位
    "request_message_id": "app_start_route_1781334566203" // 可选；建议传
  },
  "metadata": {
    "operator_id": "user_001",                // 操作人
    "client_id": "tablet_001",                // 客户端
    "trace_id": "trace_start_route_001"       // 追踪 ID
  }
}
```

ROS 业务 ACK：

```jsonc
{
  "protocol_version": "2.0",                  // 固定协议版本
  "message_id": "push_1781334566",            // ROS 推送 ID
  "timestamp": 1781334566.500,                // 推送时间
  "message_type": "push",                     // 主动推送
  "data_type": "navigation_status",           // 导航状态
  "source": "data_integration_node",          // 可能由数据整合节点包装
  "destination": "all",                       // 广播
  "data": {
    "event_type": "navigation_command_result", // 业务 ACK 事件
    "event_data": {
      "request_message_id": "app_start_route_1781334566203", // 对应 APP 请求
      "ack_type": "navigation_command_result", // 固定 ACK 类型
      "command_type": "start_route_task",     // 对应命令
      "task_session_id": "tour_20260613_001", // 当前任务 ID
      "route_id": "route_hall_main_001",      // 当前路线 ID
      "map_id": "hall",                       // 当前路线地图
      "status": "success",                    // success=命令被业务层接受
      "result_reason": "route_task_started",  // 成功原因，可能随具体分支变化
      "error_code": "",                       // 成功为空
      "message": "route task accepted and first segment started", // 说明
      "event_id": "route_task_tour_20260613_001_navigation_command_result_1_1781334566500", // 事件 ID，用于去重
      "timestamp": 1781334566.500              // 业务事件时间
    },
    "timestamp": 1781334566.500,              // 外层业务时间
    "current_state": "executing",             // 当前导航状态
    "navigation_mode": "route_task"           // 导航模式
  },
  "metadata": {
    "status": "success",                      // 推送成功
    "error_code": "",                         // 无错误
    "error_message": "",                      // 无错误说明
    "request_id": ""                          // 推送消息通常为空
  }
}
```

### 9.2 开始路线任务，完整点位模式

适用场景：后端不依赖 ROS 点位库，直接下发完整路线快照。

```jsonc
{
  "protocol_version": "2.0",                  // 固定协议版本
  "message_id": "app_start_route_full_1781334566203", // APP 请求 ID
  "timestamp": 1781334566.203,                // 发送时间
  "message_type": "command",                  // 命令消息
  "data_type": "navigation_control",          // 导航控制
  "source": "app_backend",                    // APP 后端
  "destination": "robot",                     // ROS 机器人
  "data": {
    "command_type": "start_route_task",       // 开始路线任务
    "task_session_id": "tour_20260613_002",   // 一轮任务 ID
    "route_id": "route_hall_main_001",        // 路线 ID
    "map_id": "hall",                         // 路线所属地图
    "route_waypoint_ids": [],                 // 完整点位模式必须为空或不传
    "waypoints_revision": "",                 // 完整点位模式不强制使用 revision
    "route_waypoints": [
      {
        "waypoint_id": "10",                  // 点位 ID
        "waypoint_name": "展厅任务点10",        // 点位名称
        "map_id": "hall",                     // 所属地图
        "waypoint_role": "task",              // task=任务点
        "frame_id": "map",                    // 坐标系
        "position": [1.0, 2.0, 0.0],          // 位置
        "orientation": [0.0, 0.0, 0.0, 1.0],  // 姿态
        "need_broadcast": true,               // 需要播报
        "broadcast_id": "broadcast_10",       // 播报 ID
        "broadcast_text": "欢迎来到10号点",      // 播报文本
        "broadcast_blocking": true,           // 等待播报完成
        "stop_and_align": true,               // 到点对齐
        "walk_direction": "forward",          // 正走
        "properties": {
          "map_id": "hall",                   // 冗余地图
          "waypoint_role": "task",            // 冗余 role
          "walk_direction": "forward"         // 冗余行走方向
        }
      },
      {
        "waypoint_id": "11",                  // 点位 ID
        "waypoint_name": "过门辅助点11",        // 点位名称
        "map_id": "hall",                     // 所属地图
        "waypoint_role": "transit",           // transit=辅助点
        "frame_id": "map",                    // 坐标系
        "position": [1.5, 2.2, 0.0],          // 位置
        "orientation": [0.0, 0.0, 0.0, 1.0],  // 姿态
        "need_broadcast": false,              // 辅助点不播报
        "broadcast_id": "",                   // 辅助点为空
        "broadcast_text": "",                 // 辅助点为空
        "broadcast_blocking": false,          // 不等待
        "stop_and_align": false,              // 不停车对齐
        "walk_direction": "forward",          // 正走
        "properties": {
          "map_id": "hall",                   // 冗余地图
          "waypoint_role": "transit",         // 冗余 role
          "walk_direction": "forward"         // 冗余行走方向
        }
      }
    ],
    "request_message_id": "app_start_route_full_1781334566203" // 可选；建议传
  },
  "metadata": {
    "operator_id": "user_001",                // 操作人
    "client_id": "tablet_001",                // 客户端
    "trace_id": "trace_start_route_full_001"  // 追踪 ID
  }
}
```

### 9.3 暂停路线任务

```jsonc
{
  "protocol_version": "2.0",                  // 固定协议版本
  "message_id": "app_pause_route_1781334566203", // APP 请求 ID
  "timestamp": 1781334566.203,                // 发送时间
  "message_type": "command",                  // 命令消息
  "data_type": "navigation_control",          // 导航控制
  "source": "app_backend",                    // APP 后端
  "destination": "robot",                     // ROS 机器人
  "data": {
    "command_type": "pause_route_task",       // 暂停路线任务
    "task_session_id": "tour_20260613_001",   // 当前任务 ID
    "route_id": "route_hall_main_001",        // 当前路线 ID
    "map_id": "hall",                         // 当前路线地图
    "reason": "manual_pause",                 // manual_pause=用户手动暂停
    "pause_parameters": {
      "pause_source": "app_button",           // 暂停来源
      "keep_route_context": true              // 保留路线任务上下文
    },
    "request_message_id": "app_pause_route_1781334566203" // 对应请求
  },
  "metadata": {
    "operator_id": "user_001",                // 操作人
    "client_id": "tablet_001",                // 客户端
    "trace_id": "trace_pause_route_001"       // 追踪 ID
  }
}
```

### 9.4 继续路线任务

```jsonc
{
  "protocol_version": "2.0",                  // 固定协议版本
  "message_id": "app_resume_route_1781334566203", // APP 请求 ID
  "timestamp": 1781334566.203,                // 发送时间
  "message_type": "command",                  // 命令消息
  "data_type": "navigation_control",          // 导航控制
  "source": "app_backend",                    // APP 后端
  "destination": "robot",                     // ROS 机器人
  "data": {
    "command_type": "resume_route_task",      // 继续路线任务
    "task_session_id": "tour_20260613_001",   // 当前任务 ID
    "route_id": "route_hall_main_001",        // 当前路线 ID
    "map_id": "hall",                         // 当前路线地图
    "reason": "manual_resume",                // manual_resume=用户点击继续
    "request_message_id": "app_resume_route_1781334566203" // 对应请求
  },
  "metadata": {
    "operator_id": "user_001",                // 操作人
    "client_id": "tablet_001",                // 客户端
    "trace_id": "trace_resume_route_001"      // 追踪 ID
  }
}
```

### 9.5 终止路线任务

```jsonc
{
  "protocol_version": "2.0",                  // 固定协议版本
  "message_id": "app_stop_route_1781334566203", // APP 请求 ID
  "timestamp": 1781334566.203,                // 发送时间
  "message_type": "command",                  // 命令消息
  "data_type": "navigation_control",          // 导航控制
  "source": "app_backend",                    // APP 后端
  "destination": "robot",                     // ROS 机器人
  "data": {
    "command_type": "stop_route_task",        // 终止路线任务
    "task_session_id": "tour_20260613_001",   // 当前任务 ID
    "route_id": "route_hall_main_001",        // 当前路线 ID
    "map_id": "hall",                         // 当前路线地图
    "reason": "manual_stop",                  // manual_stop=用户主动终止
    "stop_parameters": {
      "clear_route_context": true,            // true=终止后清理路线上下文
      "cancel_nav2_goal": true                // true=取消当前 Nav2 goal
    },
    "request_message_id": "app_stop_route_1781334566203" // 对应请求
  },
  "metadata": {
    "operator_id": "user_001",                // 操作人
    "client_id": "tablet_001",                // 客户端
    "trace_id": "trace_stop_route_001"        // 追踪 ID
  }
}
```

### 9.6 任意跳点

```jsonc
{
  "protocol_version": "2.0",                  // 固定协议版本
  "message_id": "app_jump_route_1781334566203", // APP 请求 ID
  "timestamp": 1781334566.203,                // 发送时间
  "message_type": "command",                  // 命令消息
  "data_type": "navigation_control",          // 导航控制
  "source": "app_backend",                    // APP 后端
  "destination": "robot",                     // ROS 机器人
  "data": {
    "command_type": "jump_to_waypoint",       // 跳到指定任务点
    "task_session_id": "tour_20260613_001",   // 当前任务 ID
    "route_id": "route_hall_main_001",        // 当前路线 ID
    "map_id": "hall",                         // 当前路线地图
    "target_waypoint_id": "15",               // 目标任务点 ID，只能是 task 点
    "interrupt_broadcast": true,              // 如果当前正在等待播报，是否中断播报并跳点
    "reason": "user_jump",                    // 跳点原因
    "request_message_id": "app_jump_route_1781334566203" // 对应请求
  },
  "metadata": {
    "operator_id": "user_001",                // 操作人
    "client_id": "tablet_001",                // 客户端
    "trace_id": "trace_jump_route_001"        // 追踪 ID
  }
}
```

ROS 跳点成功事件：

```jsonc
{
  "protocol_version": "2.0",                  // 固定协议版本
  "message_id": "push_1781334566",            // ROS 推送 ID
  "timestamp": 1781334566.500,                // 推送时间
  "message_type": "push",                     // 主动推送
  "data_type": "navigation_status",           // 导航状态
  "source": "data_integration_node",          // 数据整合节点
  "destination": "all",                       // 广播
  "data": {
    "event_type": "jump_updated",             // 跳点后路线段已更新
    "event_data": {
      "event_id": "route_task_tour_20260613_001_jump_updated_9_1781334566500", // 事件 ID
      "timestamp": 1781334566.500,            // 事件时间
      "task_session_id": "tour_20260613_001", // 当前任务
      "route_id": "route_hall_main_001",      // 当前路线
      "map_id": "hall",                       // 当前地图
      "target_waypoint_id": "15",             // 目标任务点
      "segment_direction": "forward",         // forward/backward，表示新段方向
      "execution_waypoint_ids": ["11", "12", "15"], // 实际执行段，含吸收的 transit 辅助点
      "passed_transit_waypoint_ids": [],      // 新段已通过的辅助点，刚切段通常为空
      "completed_task_ids": ["1", "2", "3"],  // 已完成任务点快照
      "skipped_task_ids": ["4", "5", "6"]     // 被跳过的任务点快照
    },
    "timestamp": 1781334566.500,              // 外层时间
    "current_state": "executing",             // 当前状态
    "navigation_mode": "route_task"           // 导航模式
  },
  "metadata": {
    "status": "success",                      // 成功
    "error_code": "",                         // 无错误
    "error_message": "",                      // 无错误说明
    "request_id": ""                          // 推送无请求 ID
  }
}
```

### 9.7 播报完成

ROS 请求 APP 播报：

```jsonc
{
  "protocol_version": "2.0",                  // 固定协议版本
  "message_id": "push_1781334566",            // ROS 推送 ID
  "timestamp": 1781334566.500,                // 推送时间
  "message_type": "push",                     // 主动推送
  "data_type": "navigation_status",           // 导航状态
  "source": "data_integration_node",          // 数据整合节点
  "destination": "all",                       // 广播
  "data": {
    "event_type": "broadcast_requested",      // APP 需要播报
    "event_data": {
      "event_id": "route_task_tour_20260613_001_broadcast_requested_5_1781334566500", // 事件 ID
      "timestamp": 1781334566.500,            // 事件时间
      "task_session_id": "tour_20260613_001", // 当前任务
      "route_id": "route_hall_main_001",      // 当前路线
      "map_id": "hall",                       // 当前地图
      "waypoint_id": "10",                    // 到达的任务点
      "waypoint_name": "展厅任务点10",          // 点位名
      "broadcast_id": "broadcast_10",         // 播报 ID，APP 完成时原样回传
      "broadcast_text": "欢迎来到展厅10号点",    // APP 播放文本
      "need_broadcast": true,                 // 确认需要播报
      "broadcast_blocking": true              // true=ROS 正在等待播报完成
    },
    "timestamp": 1781334566.500,              // 外层时间
    "current_state": "reached_waypoint",      // 到点等待
    "navigation_mode": "route_task"           // 导航模式
  },
  "metadata": {
    "status": "success",                      // 成功
    "error_code": "",                         // 无错误
    "error_message": "",                      // 无错误说明
    "request_id": ""                          // 推送无请求 ID
  }
}
```

APP 播报完成后发送：

```jsonc
{
  "protocol_version": "2.0",                  // 固定协议版本
  "message_id": "app_broadcast_finished_1781334566203", // APP 请求 ID
  "timestamp": 1781334566.203,                // 发送时间
  "message_type": "command",                  // 命令消息
  "data_type": "navigation_control",          // 导航控制
  "source": "app_backend",                    // APP 后端
  "destination": "robot",                     // ROS 机器人
  "data": {
    "command_type": "broadcast_finished",     // 播报完成
    "task_session_id": "tour_20260613_001",   // 当前任务 ID
    "route_id": "route_hall_main_001",        // 当前路线 ID
    "map_id": "hall",                         // 当前地图
    "waypoint_id": "10",                      // 刚播报完成的点位
    "broadcast_id": "broadcast_10",           // ROS 请求中的 broadcast_id 原样回传
    "broadcast_result": "completed",          // completed=完成；failed=播报失败
    "broadcast_duration_sec": 8.5,            // 播报耗时，秒
    "reason": "tts_finished",                 // 完成原因
    "request_message_id": "app_broadcast_finished_1781334566203" // 对应请求
  },
  "metadata": {
    "operator_id": "user_001",                // 操作人
    "client_id": "tablet_001",                // 客户端
    "trace_id": "trace_broadcast_finished_001" // 追踪 ID
  }
}
```

## 10. 导航状态与障碍物暂停

APP 前端必须监听 `navigation_status` 周期状态，不要只靠按钮点击后本地改 UI。

周期状态示例：

```jsonc
{
  "protocol_version": "2.0",                  // 固定协议版本
  "message_id": "push_1781334566",            // ROS 推送 ID
  "timestamp": 1781334566.500,                // 推送时间
  "message_type": "push",                     // 主动推送
  "data_type": "navigation_status",           // 导航状态
  "source": "data_integration_node",          // 数据整合节点
  "destination": "all",                       // 广播
  "data": {
    "timestamp": 1781334566.500,              // 状态时间
    "current_state": "paused",                // idle/executing/paused/reached_waypoint/completed 等
    "navigation_mode": "route_task",          // route_task
    "current_waypoint_index": 5,              // 当前路线索引
    "total_waypoints": 25,                    // 路线总点数
    "progress_percentage": 42.5,              // 进度百分比
    "is_active": true,                        // 是否存在活动导航
    "navigation_duration": 120.0,             // 导航持续时间
    "detailed_state": "OBSTACLE_WAIT",        // 详细状态
    "obstacle_blocked": true,                 // 是否检测到障碍物阻塞
    "waiting_for_obstacle_clear": true,       // 是否等待障碍物清除
    "obstacle_wait_active": true,             // 障碍物等待状态
    "obstacle_wait_duration": 12.3,           // 已等待秒数
    "pause_source": "obstacle_wait",          // manual_pause/obstacle_wait/manual_escape/broadcast_wait/navigation_failed
    "pause_reason": "front_obstacle_blocked", // 暂停原因
    "resume_mode": "manual",                  // auto/manual，具体以 ROS 当前状态为准
    "front_obstacle_blocked": true,           // 前方障碍是否阻塞
    "front_obstacle_stats": {
      "min_distance": 0.35,                   // 最近障碍距离，米
      "blocked_points": 18                    // 阻塞点数量
    },
    "active_map_id": "hall",                  // 当前导航地图
    "map_state": "ready",                     // 地图状态
    "map_localization_state": "stable",       // 地图定位状态
    "localization_healthy": true,             // 定位是否健康
    "current_pose": {
      "frame_id": "map",                      // 位姿坐标系
      "position": {
        "x": 1.0,                             // 当前 x
        "y": 2.0,                             // 当前 y
        "z": 0.0                              // 当前 z
      },
      "orientation": {
        "x": 0.0,                             // 四元数 x
        "y": 0.0,                             // 四元数 y
        "z": 0.0,                             // 四元数 z
        "w": 1.0                              // 四元数 w
      }
    },
    "current_velocity": {
      "linear": {
        "x": 0.1,                             // x 方向线速度
        "y": 0.0,                             // y 方向线速度
        "z": 0.0                              // z 方向线速度
      },
      "angular": {
        "x": 0.0,                             // x 方向角速度
        "y": 0.0,                             // y 方向角速度
        "z": 0.05                             // z 方向角速度
      }
    },
    "route_task": {
      "task_session_id": "tour_20260613_001", // 当前任务 ID
      "route_id": "route_hall_main_001",      // 当前路线 ID
      "map_id": "hall",                       // 当前路线地图
      "awaiting_broadcast": false,            // 是否等待 APP 播报
      "waiting_broadcast_waypoint_id": "",    // 等待播报的点位
      "completed_task_ids": ["1", "2"],       // 已完成任务点
      "skipped_task_ids": [],                 // 已跳过任务点
      "passed_transit_waypoint_ids": ["11"]   // 当前段已通过辅助点
    }
  },
  "metadata": {
    "status": "success",                      // 成功
    "error_code": "",                         // 无错误
    "error_message": "",                      // 无错误说明
    "request_id": ""                          // 周期状态无请求 ID
  }
}
```

UI 规则：

1. `pause_source=manual_pause`：显示“用户已暂停”，继续按钮可用。
2. `pause_source=obstacle_wait`：显示“前方有障碍，等待清障”，继续按钮根据 `resume_mode` 和现场策略决定是否可用。
3. `pause_source=manual_escape`：显示“人工脱困后等待继续”，继续按钮可用。
4. `pause_source=broadcast_wait` 或 `route_task.awaiting_broadcast=true`：显示“等待播报完成”，播报完成后发 `broadcast_finished`，不要发 `resume_route_task`。
5. `pause_source=navigation_failed`：显示失败文案，需要终止或重新开始。

## 11. APP 三方流程

本章用更通俗的方式描述每个功能按钮背后的完整链路：

用户操作 -> 前端处理 -> 后端处理 -> 后端发给 ROS -> ROS 处理 -> ROS 回传 -> 后端转发/落库 -> 前端更新 UI。

### 11.1 APP 启动/进入导航页

1. 前端连接后端 websocket。
2. 前端进入导航页后，先显示 `连接中/加载地图中`，不要默认认为当前地图是 `hall`。
3. 后端收到前端进入页面事件后，向 ROS 发送 `data_type=map_management, command_type=get_map_list`。
4. 后端再向 ROS 发送 `data_type=map_management, command_type=get_current_map`。
5. ROS 的 `map_context_manager` 读取 `data/maps/map_registry.json`，返回 `map_response`。
6. 后端收到 `map_response` 后，把地图列表、`current_map_id`、`default_map_id` 转发给前端。
7. 前端用地图列表渲染下拉框，用 `current_map_id` 显示“机器人当前地图”。
8. 后端同时监听 ROS 主动推送的 `map_status / waypoints_data / navigation_status`，并持续转发给前端。
9. 前端收到 `map_status` 后更新地图状态栏，例如 `ready / switching / waiting_localization / failed`。
10. 前端收到 `waypoints_data` 后缓存每张地图的 `waypoints_revision`，后面启动路线任务时要用。

### 11.2 用户点击“刷新地图列表”

1. 用户点击前端按钮 `刷新地图列表`。
2. 前端把按钮置为加载中，并调用后端接口，例如 `POST /map/list` 或 websocket 内部事件。
3. 后端生成 `message_id`，发送给 ROS：

```jsonc
{
  "data_type": "map_management",              // 地图管理入口
  "data": {
    "command_type": "get_map_list",           // 查询地图列表
    "request_message_id": "cmd_get_map_list_0001" // 请求 ID
  }
}
```

4. ROS 收到后读取地图注册表，发布 `map_response`。
5. 后端收到 `map_response.data.status=success` 后，更新后端缓存的地图列表。
6. 后端把 `maps/current_map_id/default_map_id` 返回给前端。
7. 前端刷新地图选择器，并取消按钮加载状态。
8. 如果 ROS 返回 `status=error`，后端把 `error_code/message` 原样给前端，前端弹出错误提示。

### 11.3 用户选择地图但还没有点击切换

1. 前端设置 `selected_map_id`。
2. 后端加载该地图下的路线和点位。
3. 若 `selected_map_id != robot_current_map_id`，前端提示“需要切换地图”。
4. 此时前端只切换页面数据和点位列表，不代表 ROS 已切图。
5. 如果用户只是查看点位，不需要发 `switch_map`。
6. 如果用户准备在这张地图上导航，前端必须要求用户点击 `切换到此地图`，或者在点击开始前提示先切图。

### 11.4 用户点击“切换到此地图”

1. 用户在前端选择 `hall1` 后点击 `切换到此地图`。
2. 前端先做本地判断：如果当前有路线任务执行中、暂停中、等待播报中，前端应直接提示先终止任务，不建议发切图命令。
3. 前端把地图状态栏改为 `请求切图中`，禁用开始导航按钮。
4. 后端收到前端切图请求后，生成 `message_id`，发送给 ROS：

```jsonc
{
  "data_type": "map_management",              // 地图管理入口
  "data": {
    "command_type": "switch_map",             // 切换地图
    "target_map_id": "hall1",                 // 用户选择的目标地图
    "reason": "user_selected_map",            // 切图原因
    "request_message_id": "cmd_switch_map_0001" // 请求 ID
  }
}
```

5. ROS 的 `map_context_manager` 收到后先校验：目标地图是否注册、是否 enabled、2D yaml 是否存在、3D prior pcd 是否存在、当前是否有导航任务。
6. 如果校验失败，ROS 返回 `map_response status=error`，例如 `map_file_missing / prior_map_file_missing / map_switch_rejected_route_task_active`。
7. 后端收到错误后，记录切图失败日志，并把 `error_code/message/current_map_id/target_map_id` 转发给前端。
8. 前端显示失败原因，恢复切图按钮，继续禁止目标地图的开始导航。
9. 如果校验通过，ROS 返回第一条成功响应：`map_response.data.result_reason=map_switch_restart_started`。
10. 后端收到 `map_switch_restart_started` 后，把状态转发前端。
11. 前端显示 `正在切换地图，导航定位层重启中`，继续禁用开始导航。
12. ROS 在后台只重启导航定位层，不停止 websocket、点位管理、地图管理控制层。
13. 切图过程中 ROS 会持续推 `map_status`，例如 `switching / localization_resetting / waiting_localization`。
14. 后端把每条 `map_status` 转发给前端。
15. 前端按 `map_status.map_state/localization_state` 更新进度文案。
16. ROS 重启定位后发布目标地图 `initial_pose`，等待 `/localization/prior_map_odom_bridge_status` 健康。
17. 定位稳定后，ROS 返回 `map_response.data.result_reason=map_ready`，同时 `map_status.current_map_id=hall1,map_state=ready`。
18. 后端收到 `map_ready` 后，把后端当前机器人地图更新为 `hall1`。
19. 前端收到 `map_ready` 后显示 `地图 hall1 已就绪`，恢复开始导航按钮。

### 11.5 用户新增或保存点位

1. 前端在当前地图上标点。
2. 前端让用户选择点位属性：`任务点 task` 或 `辅助点 transit`，是否播报，正走/倒走。
3. 前端把点位坐标、姿态、属性发给后端。
4. 后端补齐 `map_id / waypoint_id / waypoint_name / waypoint_role / walk_direction / need_broadcast / stop_and_align` 等字段。
5. 后端发送给 ROS：

```jsonc
{
  "data_type": "waypoint_management",         // 点位管理入口
  "data": {
    "command_type": "set_waypoint",           // 保存点位
    "map_id": "hall",                         // 当前地图
    "waypoint_type": "manual",                // 点位类型
    "waypoint_data": {
      "waypoint_id": "11",                    // 点位 ID
      "waypoint_name": "过门辅助点11",          // 点位名称
      "map_id": "hall",                       // 所属地图
      "waypoint_role": "transit",             // task/transit
      "frame_id": "map",                      // 坐标系
      "position": [2.0, 2.0, 0.0],            // 坐标
      "orientation": [0.0, 0.0, 0.707, 0.707], // 姿态
      "need_broadcast": false,                // 是否播报
      "broadcast_id": "",                     // 播报 ID
      "broadcast_blocking": false,            // 是否等待播报完成
      "stop_and_align": false,                // 是否停车对齐
      "walk_direction": "forward"             // 正走/倒走
    }
  }
}
```

6. ROS 的 `dynamic_waypoints_manager` 收到后，按 `map_id` 保存到 `data/waypoints/<map_id>.json`。
7. ROS 更新该地图的 `waypoints_revision`。
8. ROS 返回 `waypoint_response`，并推送最新 `waypoints_data`。
9. 后端收到 `waypoint_response response_type=success` 后，保存新的 `waypoints_revision`。
10. 后端把保存成功和最新点位版本返回给前端。
11. 前端刷新点位列表，并展示保存成功。
12. 如果 ROS 返回错误，后端原样转发错误，前端提示用户检查点位 ID、坐标、地图选择。

### 11.6 用户编辑点位

1. 用户在前端点位列表点击 `编辑`。
2. 前端加载该点位详情，允许修改名称、播报、角色、行走方向、坐标姿态。
3. 前端不建议允许直接修改 `map_id`；如果确实要换地图，应做复制点位或重新标点。
4. 后端收到编辑提交后，发送 `waypoint_management / update_waypoint`。
5. ROS 更新该地图点位文件，并刷新 `waypoints_revision`。
6. 后端收到 `waypoint_response success` 后更新后端缓存。
7. 前端刷新点位详情和列表。

### 11.7 用户删除或清空点位

1. 用户点击 `删除点位` 或 `清空当前地图点位`。
2. 前端弹确认框，明确显示地图名，例如 `确认清空 hall1 下所有点位？`。
3. 后端发送 `delete_waypoint` 或 `clear_waypoints`，必须带 `map_id`。
4. ROS 只操作对应 `map_id` 的点位文件。
5. ROS 返回 `waypoint_response` 并推送 `waypoints_data`。
6. 后端更新该地图点位缓存和 `waypoints_revision`。
7. 前端刷新点位列表。

### 11.8 用户点击“开始导航”

1. 前端确认 `selected_map_id == robot_current_map_id`。
2. 前端确认 `map_state=ready`。
3. 后端确认路线全部点位属于 `selected_map_id`。
4. 后端生成新的 `task_session_id`。
5. 后端取出该地图最新 `waypoints_revision`。
6. 后端优先用 ID 列表模式发送 `start_route_task`：

```jsonc
{
  "data_type": "navigation_control",          // 导航控制入口
  "data": {
    "command_type": "start_route_task",       // 开始路线任务
    "task_session_id": "tour_20260613_001",   // 本轮任务 ID
    "route_id": "route_hall_main_001",        // 路线 ID
    "map_id": "hall",                         // 路线地图
    "waypoints_revision": "1781334566.500",   // 当前地图点位版本
    "route_waypoint_ids": ["1", "2", "3"]     // 有序路线点 ID
  }
}
```

7. ROS 的 `navigation_state_manager` 收到后校验：`task_session_id`、`route_id`、`map_id`、当前地图是否 ready、点位版本是否一致、ID 是否存在、路线是否至少有一个 task 点。
8. 如果校验失败，ROS 推送 `navigation_status.event_type=navigation_command_result,status=error`。
9. 后端收到错误后，把 `error_code/message` 转给前端。
10. 前端恢复开始按钮，并根据错误码提示，例如点位版本不一致时提示刷新点位。
11. 如果校验成功，ROS 推送 `navigation_command_result,status=success`。
12. 后端收到成功后，把任务状态置为 `navigating`。
13. 前端进入导航中 UI：开始按钮禁用，暂停/终止/跳点按钮可用。
14. 后续前端不再靠本地计时猜状态，而是监听 ROS 推送的 `navigation_status`。

### 11.9 导航过程中经过辅助点

1. ROS 执行路线时，如果某个点是 `waypoint_role=transit`，不会停车播报，也不会最终 spin 对齐。
2. ROS 通过该辅助点后推送 `navigation_status.event_type=waypoint_passed`。
3. 后端收到后转发给前端。
4. 前端只更新路线进度和地图高亮，不弹播报，不切换到等待状态。
5. 如果同一段内重复收到同一个辅助点事件，前端可按 `event_id` 去重。

### 11.10 到达需要播报的任务点

1. ROS 到达 `task` 点并完成最终对齐后，判断 `need_broadcast=true`。
2. ROS 推送 `navigation_status.event_type=broadcast_requested`。
3. 后端收到后，把 `waypoint_id / broadcast_id / broadcast_text / task_session_id / route_id / map_id` 转发给前端。
4. 前端进入 `等待播报` 状态，暂停继续按钮不要当作播报完成按钮。
5. 前端或后端调用 APP 语音播报能力播放 `broadcast_text`。
6. 播报完成后，后端向 ROS 发送 `broadcast_finished`：

```jsonc
{
  "data_type": "navigation_control",          // 导航控制入口
  "data": {
    "command_type": "broadcast_finished",     // 播报完成
    "task_session_id": "tour_20260613_001",   // 当前任务 ID
    "route_id": "route_hall_main_001",        // 当前路线 ID
    "map_id": "hall",                         // 当前地图
    "waypoint_id": "10",                      // 播报点位
    "broadcast_id": "broadcast_10",           // ROS 请求中的 broadcast_id
    "broadcast_result": "completed",          // completed/failed
    "broadcast_duration_sec": 8.5             // 播报耗时
  }
}
```

7. ROS 校验播报上下文是否匹配当前等待点。
8. 校验失败时，ROS 返回 `navigation_command_result,error_code=broadcast_context_mismatch`。
9. 校验成功时，ROS 返回 `navigation_command_result,status=success`，随后继续下一段导航。
10. 前端收到成功后退出等待播报状态，继续显示导航中。

### 11.11 到达不需要播报的任务点

1. ROS 到达 `task` 点并完成最终对齐。
2. 如果 `need_broadcast=false`，ROS 不推 `broadcast_requested`。
3. ROS 直接推 `task_waypoint_completed`。
4. 后端转发给前端。
5. 前端把该任务点标记为已完成。
6. ROS 自动继续下一个任务点或完成整条路线。

### 11.12 用户点击“跳点”

1. 用户点击 `跳点` 按钮。
2. 前端弹出任务点列表，只允许选择 `waypoint_role=task` 的点，不允许选择 `transit` 辅助点。
3. 用户选择目标点，例如 `15`。
4. 前端把目标点发给后端。
5. 后端发送给 ROS：

```jsonc
{
  "data_type": "navigation_control",          // 导航控制入口
  "data": {
    "command_type": "jump_to_waypoint",       // 任意跳点
    "task_session_id": "tour_20260613_001",   // 当前任务 ID
    "route_id": "route_hall_main_001",        // 当前路线 ID
    "map_id": "hall",                         // 当前地图
    "target_waypoint_id": "15",               // 目标任务点
    "interrupt_broadcast": true,              // 等待播报时是否中断播报并跳点
    "reason": "user_jump"                     // 跳点原因
  }
}
```

6. ROS 校验当前任务是否运行、session 是否匹配、目标点是否存在、目标是否为 task 点。
7. 如果目标是 transit，ROS 返回 `navigation_command_result,error_code=target_waypoint_not_task`。
8. 如果目标合法，ROS 取消当前导航段，按当前路线顺序和方向重新构建执行段。
9. 如果当前点到目标点之间有辅助点，ROS 会把这些 transit 吸收到新段里。
10. ROS 推送 `navigation_command_result,status=success`，表示跳点命令被接受。
11. ROS 推送 `jump_updated`，包含 `target_waypoint_id / execution_waypoint_ids / segment_direction / skipped_task_ids`。
12. 后端把两个事件都转发给前端。
13. 前端收到 `navigation_command_result success` 后关闭跳点弹窗。
14. 前端收到 `jump_updated` 后更新路线高亮：跳过的任务点标记为 skipped，新执行段高亮。

### 11.13 用户点击“暂停”

1. 用户点击暂停按钮。
2. 前端先把按钮置为 `暂停中...`，但不要直接当作业务成功。
3. 后端发送 `pause_route_task`，带当前 `task_session_id / route_id / map_id`。
4. ROS 校验当前任务是否可暂停。
5. ROS 返回 `navigation_command_result,status=success,result_reason=route_task_paused`。
6. 后端收到成功后，把任务状态更新为 `paused`。
7. 前端显示 `已暂停`，暂停按钮变成继续按钮。
8. 如果 ROS 返回错误，例如 `route_task_not_running`，前端恢复原按钮并提示错误。

### 11.14 用户点击“继续”

1. 用户点击继续按钮。
2. 前端把按钮置为 `继续中...`。
3. 后端发送 `resume_route_task`，带当前 `task_session_id / route_id / map_id`。
4. ROS 判断当前暂停来源：
   - 如果是用户手动暂停，允许继续。
   - 如果是人工脱困等待继续，允许继续。
   - 如果仍处于障碍物阻塞且策略不允许继续，可能返回错误或继续等待。
5. ROS 返回 `navigation_command_result,status=success,result_reason=route_task_resumed`。
6. 后端转发成功给前端。
7. 前端显示 `导航中`，恢复暂停按钮。
8. 后续仍以周期 `navigation_status` 为准更新状态栏。

### 11.15 ROS 检测到障碍物自动暂停

1. 导航执行中，ROS 检测前方障碍物持续阻塞。
2. ROS 自动暂停当前导航，不需要 APP 先发命令。
3. ROS 周期推送 `navigation_status`，其中：
   - `current_state=paused`
   - `pause_source=obstacle_wait`
   - `waiting_for_obstacle_clear=true`
   - `obstacle_wait_active=true`
   - `front_obstacle_blocked=true`
4. 后端收到该状态后转发给前端。
5. 前端状态栏显示 `前方有障碍，等待清障`。
6. 如果 ROS 自动确认障碍消失并恢复，会推送 `navigation_resumed` 或周期状态变回执行中。
7. 如果需要人工清障或人工移动机器人，用户清障后点击 `继续`，仍然走 `resume_route_task`。
8. 前端不要把障碍暂停和用户手动暂停做成两套按钮，只要状态文案不同即可。

### 11.16 用户点击“终止”

1. 用户点击终止按钮。
2. 前端弹确认框，提示终止后本轮任务结束。
3. 后端发送 `stop_route_task`，带当前 `task_session_id / route_id / map_id`。
4. ROS 取消当前 Nav2 goal，清理 route task 上下文。
5. ROS 返回 `navigation_command_result,status=success,result_reason=route_task_stopped`。
6. ROS 可能继续推送 `navigation_stopped`。
7. 后端收到成功后清理当前任务缓存。
8. 前端回到未开始状态，开始按钮可用，暂停/继续/跳点/终止按钮禁用。
9. 下一次开始任务必须生成新的 `task_session_id`。

### 11.17 整条路线完成

1. ROS 完成最后一个 task 点。
2. 如果最后一个 task 需要播报，ROS 会先等待 `broadcast_finished`。
3. 播报闭环完成后，ROS 推送 `route_task_completed`。
4. 后端收到后，把任务结果、完成点、跳过点、耗时等保存到任务记录。
5. 前端显示 `路线完成`，进度到 100%。
6. 前端禁用暂停/继续/跳点/终止按钮。
7. 前端允许用户重新开始，重新开始时生成新的 `task_session_id`。

### 11.18 切图和导航冲突

如果导航正在执行、暂停、等待播报，APP 仍发送 `switch_map`，ROS 会返回：

```jsonc
{
  "data_type": "map_response",                // 地图响应
  "data": {
    "status": "error",                        // 失败
    "error_code": "map_switch_rejected_route_task_active", // 导航活动中拒绝切图
    "message": "当前导航状态不允许切图: OBSTACLE_WAIT", // 说明
    "current_map_id": "hall",                 // 当前地图
    "target_map_id": "hall1",                 // 目标地图
    "map_state": "ready",                     // 地图状态
    "command_type": "switch_map",             // 对应命令
    "request_message_id": "app_switch_map_1781334566203" // 对应请求
  },
  "metadata": {
    "status": "error",                        // 失败
    "error_code": "map_switch_rejected_route_task_active", // 错误码
    "error_message": "当前导航状态不允许切图: OBSTACLE_WAIT", // 错误说明
    "request_id": "app_switch_map_1781334566203" // 对应请求
  }
}
```

APP 应提示：`当前任务未结束，请先终止任务后再切换地图。`

## 12. 后端解析规则

### 12.1 命令 ACK 和状态推送怎么区分

1. websocket 立即返回或连接层成功，只代表消息发出，不代表业务成功。
2. 地图命令业务结果看 `map_response`。
3. 点位命令业务结果看 `waypoint_response`。
4. 导航命令业务结果看 `navigation_status.data.event_type=navigation_command_result`。
5. 导航实时状态看没有 `event_type` 的周期 `navigation_status`。

### 12.2 `navigation_status` 事件解析

APP 要按 `data.event_type` 判断业务事件：

1. `navigation_command_result`：命令业务 ACK。
2. `broadcast_requested`：ROS 到达需要播报的任务点，APP 播报后发 `broadcast_finished`。
3. `waypoint_passed`：通过辅助点，APP 可更新进度但不播报。
4. `jump_updated`：跳点后新执行段已经生成，APP 更新路线高亮。
5. `task_waypoint_completed`：任务点完成，APP 更新已完成点。
6. `route_task_completed`：整条路线完成，APP 解锁任务。
7. `navigation_failed`：导航失败，APP 展示错误并允许终止/重试。

### 12.3 常见导航错误码

1. `missing_task_session_id`：缺任务 ID。
2. `missing_route_id`：缺路线 ID。
3. `missing_map_id`：缺地图 ID。
4. `active_map_mismatch`：路线地图和当前 ready 地图不一致。
5. `map_not_ready`：地图未 ready。
6. `invalid_route_waypoints`：路线点位非法。
7. `ambiguous_route_waypoint_source`：同时传了 `route_waypoints` 和 `route_waypoint_ids`。
8. `missing_waypoints_revision`：ID 列表模式缺版本号。
9. `waypoints_revision_mismatch`：APP 版本和 ROS 点位版本不一致。
10. `invalid_route_waypoint_ids`：ID 列表为空或含非法 ID。
11. `route_waypoint_not_found`：某个 ID 在该地图点位库中不存在。
12. `missing_task_waypoints`：路线没有任务点。
13. `invalid_waypoint_role`：点位角色不是 `task/transit`。
14. `missing_waypoint_pose`：点位缺 position/orientation。
15. `route_task_not_running`：当前没有路线任务。
16. `invalid_task_session`：任务 ID 不匹配。
17. `invalid_route_id`：路线 ID 不匹配。
18. `invalid_target_waypoint`：跳点目标不存在。
19. `target_waypoint_not_task`：跳点目标不是任务点。
20. `broadcast_context_mismatch`：播报完成回执和当前等待播报点不匹配。

## 13. APP 联调验收清单

前端验收：

1. 能显示地图列表。
2. 能显示 ROS 当前地图和 UI 选中地图。
3. 切图过程中按钮禁用，状态栏显示 `switching/localization_resetting/waiting_localization`。
4. 收到 `map_ready` 后开始按钮恢复可用。
5. 导航过程中切图按钮禁用或收到拒绝后有明确提示。
6. 点位管理页按地图过滤，不串图。
7. 跳点弹窗只允许选择 `task` 点，不允许选择 `transit` 点。
8. 障碍物暂停时状态栏显示暂停原因，继续按钮仍走 `resume_route_task`。
9. 等待播报时显示播报状态，播报结束发 `broadcast_finished`。

后端验收：

1. 所有点位保存都带 `map_id`。
2. 每张地图保存自己的 `waypoints_revision`。
3. `start_route_task` 使用 `route_waypoint_ids` 时必须带对应地图 revision。
4. 一轮任务只生成一个 `task_session_id`。
5. 暂停/继续/终止/跳点/播报完成都复用同一 `task_session_id + route_id + map_id`。
6. 路线启动前校验所有点位属于同一 `map_id`。
7. 后端不再发送旧的单点、多点、展台、旧暂停、旧继续、旧终止命令。

ROS 联调验收：

1. `get_map_list` 能返回 `hall/hall1`。
2. `switch_map hall1` 在 hall1 真实地图文件缺失时返回明确错误。
3. 用复制版 hall1 可证明切图链路能完成到 `map_ready`。
4. 用真实 hall1 后，需要验证 RO/OP 定位是否稳定进入 healthy。
5. 切图后 `start_route_task(map_id=hall1)` 只能在 `current_map_id=hall1` 且 `map_state=ready` 时成功。

## 14. 最终口径

APP 侧可以按以下一句话理解多地图一期：

“APP 先选择地图并显式请求 ROS 切图，ROS 完成 2D/3D 地图切换和定位 ready 后，APP 再用该地图下的点位 ID 列表启动 route task；任务运行期间不允许切图，但允许暂停、继续、终止、跳点、播报协同和障碍物暂停恢复。”
