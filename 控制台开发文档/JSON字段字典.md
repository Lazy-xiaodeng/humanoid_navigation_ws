# JSON字段字典

## 1. 文档目的

本文档用于补充《控制台与ROS联调JSON消息清单》中的字段级说明，目标是把第一版 MVP 涉及到的字段逐一说明清楚，方便前端、后端、ROS 三方统一口径。

## 2. 通用外层字段

## 2.1 `protocol_version`

1. 类型：`string`
2. 是否必填：是
3. 示例：`"2.0"`
4. 含义：协议版本号

## 2.2 `type`

1. 类型：`string`
2. 是否必填：是
3. 示例：`"start_route_task"`
4. 含义：消息类型

常见取值：

1. `get_robot_snapshot`
2. `get_map_list`
3. `get_route_task_snapshot`
4. `start_route_task`
5. `pause_route_task`
6. `resume_route_task`
7. `stop_route_task`
8. `jump_route_waypoint`
9. `switch_map`
10. `start_recording`
11. `stop_recording`
12. `command_ack`
13. `navigation_state_changed`
14. `route_task_progress_changed`
15. `robot_realtime_state_changed`
16. `system_health_changed`
17. `broadcast_state_changed`
18. `obstacle_state_changed`
19. `map_switch_state_changed`
20. `recording_state_changed`
21. `event_log_added`
22. `error_event`

## 2.3 `request_id`

1. 类型：`string`
2. 是否必填：命令消息必填；主动推送建议填空字符串或事件 ID
3. 示例：`"req_20260614_200001"`
4. 含义：请求唯一标识

## 2.4 `timestamp`

1. 类型：`number`
2. 是否必填：是
3. 示例：`1781395201000`
4. 含义：毫秒时间戳

## 2.5 `source`

1. 类型：`string`
2. 是否必填：是
3. 示例：`"web_console"`
4. 含义：发送方标识

推荐取值：

1. `web_console`
2. `gateway`
3. `ros_bridge`

## 2.6 `target`

1. 类型：`string`
2. 是否必填：是
3. 示例：`"gateway"`
4. 含义：接收方标识

## 2.7 `data`

1. 类型：`object`
2. 是否必填：是
3. 含义：业务负载

## 3. 机器人与地图字段

## 3.1 `robot_id`

1. 类型：`string`
2. 是否必填：大多数请求与状态推送必填
3. 示例：`"XR-102"`
4. 含义：机器人唯一标识

## 3.2 `robot_name`

1. 类型：`string`
2. 是否必填：查询结果或状态快照中建议携带
3. 示例：`"小贝机器人 XR-102"`
4. 含义：机器人显示名称

## 3.3 `online`

1. 类型：`boolean`
2. 是否必填：快照结果建议必填
3. 示例：`true`
4. 含义：机器人是否在线

## 3.4 `current_map_id`

1. 类型：`string`
2. 是否必填：地图相关结果与状态建议必填
3. 示例：`"hall_a_floor_1"`
4. 含义：当前激活地图 ID

## 3.5 `current_map_name`

1. 类型：`string`
2. 是否必填：建议必填
3. 示例：`"A栋一层导览图"`
4. 含义：当前地图名称

## 3.6 `target_map_id`

1. 类型：`string`
2. 是否必填：切图命令与切图状态中必填
3. 示例：`"hall_b_floor_1"`
4. 含义：目标地图 ID

## 3.7 `target_map_name`

1. 类型：`string`
2. 是否必填：建议必填
3. 示例：`"B栋一层实验室图"`
4. 含义：目标地图名称

## 4. 导航主状态字段

## 4.1 `navigation_main_state`

1. 类型：`string`
2. 是否必填：导航状态推送、任务快照建议必填
3. 含义：导航页面主状态

推荐枚举：

1. `idle`
2. `starting`
3. `running`
4. `manual_paused`
5. `obstacle_paused`
6. `manual_escape_wait_resume`
7. `broadcast_wait`
8. `localization_recovering`
9. `map_switching`
10. `failed`
11. `stopping`
12. `completed`

## 4.2 `reason_code`

1. 类型：`string`
2. 是否必填：建议必填，可为空字符串
3. 示例：`"dynamic_obstacle_blocked"`
4. 含义：当前主状态对应的原因码

## 4.3 `reason_text`

1. 类型：`string`
2. 是否必填：建议必填，可为空字符串
3. 示例：`"检测到前方动态障碍物，已自动暂停导航"`
4. 含义：给前端直接显示的原因文案

## 4.4 `can_start`

1. 类型：`boolean`
2. 是否必填：导航状态推送中建议携带
3. 含义：当前是否允许点击 `开始`

## 4.5 `can_pause`

1. 类型：`boolean`
2. 是否必填：导航状态推送中建议携带
3. 含义：当前是否允许点击 `暂停`

## 4.6 `can_resume`

1. 类型：`boolean`
2. 是否必填：导航状态推送中建议携带
3. 含义：当前是否允许点击 `继续`

## 4.7 `can_stop`

1. 类型：`boolean`
2. 是否必填：导航状态推送中建议携带
3. 含义：当前是否允许点击 `终止`

## 4.8 `can_jump`

1. 类型：`boolean`
2. 是否必填：导航状态推送中建议携带
3. 含义：当前是否允许点击 `跳点`

## 4.9 `started_at`

1. 类型：`number | null`
2. 是否必填：建议携带
3. 含义：任务开始时间戳

## 4.10 `paused_at`

1. 类型：`number | null`
2. 是否必填：建议携带
3. 含义：任务最近一次进入暂停态的时间戳

## 4.11 `resumed_at`

1. 类型：`number | null`
2. 是否必填：建议携带
3. 含义：任务最近一次恢复的时间戳

## 4.12 `finished_at`

1. 类型：`number | null`
2. 是否必填：建议携带
3. 含义：任务结束时间戳

## 5. 路线任务字段

## 5.1 `route_task_id`

1. 类型：`string`
2. 是否必填：路线任务相关消息必填
3. 示例：`"NAV-20240614-001"`
4. 含义：路线任务唯一 ID

## 5.2 `route_name`

1. 类型：`string`
2. 是否必填：建议携带
3. 示例：`"展厅上午导览"`
4. 含义：路线任务名称

## 5.3 `progress_percent`

1. 类型：`number | null`
2. 是否必填：建议携带
3. 示例：`62.5`
4. 含义：路线任务完成百分比

## 5.4 `current_waypoint_id`

1. 类型：`string`
2. 是否必填：任务执行中建议必填
3. 含义：当前执行目标点 ID

## 5.5 `current_waypoint_name`

1. 类型：`string`
2. 是否必填：建议携带
3. 含义：当前执行目标点名称

## 5.6 `next_waypoint_id`

1. 类型：`string`
2. 是否必填：有后续点时建议携带
3. 含义：下一个目标点 ID

## 5.7 `next_waypoint_name`

1. 类型：`string`
2. 是否必填：有后续点时建议携带
3. 含义：下一个目标点名称

## 5.8 `jump_target_waypoint_id`

1. 类型：`string`
2. 是否必填：建议携带，可为空字符串
3. 含义：当前跳点目标 ID

## 5.9 `jump_target_waypoint_name`

1. 类型：`string`
2. 是否必填：建议携带，可为空字符串
3. 含义：当前跳点目标名称

## 5.10 `sequence`

1. 类型：`array`
2. 是否必填：任务快照、任务进度更新建议必填
3. 含义：路线序列

元素对象字段：

1. `waypoint_id`
2. `waypoint_name`
3. `waypoint_role`
4. `walk_direction`
5. `need_broadcast`
6. `status`

## 5.11 `waypoint_id`

1. 类型：`string`
2. 是否必填：是
3. 示例：`"13"`
4. 含义：点位 ID

## 5.12 `waypoint_name`

1. 类型：`string`
2. 是否必填：建议必填
3. 示例：`"任务点13"`
4. 含义：点位名称

## 5.13 `waypoint_role`

1. 类型：`string`
2. 是否必填：是
3. 含义：点位角色

推荐枚举：

1. `task`
2. `transit`

## 5.14 `walk_direction`

1. 类型：`string`
2. 是否必填：建议必填
3. 含义：行走方向

推荐枚举：

1. `forward`
2. `backward`

## 5.15 `need_broadcast`

1. 类型：`boolean`
2. 是否必填：建议必填
3. 含义：该点是否需要播报

## 5.16 `status`

1. 类型：`string`
2. 是否必填：建议必填
3. 含义：该点当前执行状态

推荐枚举：

1. `pending`
2. `running`
3. `completed`
4. `skipped`
5. `blocked`

## 6. 点位引用模式字段

## 6.1 `waypoint_ref_mode`

1. 类型：`string`
2. 是否必填：开始任务时建议必填
3. 含义：前端给后端 / ROS 传点位的方式

推荐枚举：

1. `id_list`
2. `full_list`

## 6.2 `waypoint_ids`

1. 类型：`string[]`
2. 是否必填：`waypoint_ref_mode = id_list` 时必填
3. 含义：点位 ID 列表

## 6.3 `waypoints_revision`

1. 类型：`number`
2. 是否必填：建议必填
3. 含义：点位版本号，用于避免前端点位集和 ROS 本地点位集不一致

## 7. 机器人实时状态字段

## 7.1 `battery_percent`

1. 类型：`number | null`
2. 是否必填：建议必填
3. 含义：电量百分比

## 7.2 `velocity`

1. 类型：`object`
2. 是否必填：建议必填
3. 含义：实时速度对象

子字段：

1. `linear_mps`
2. `angular_radps`

## 7.3 `linear_mps`

1. 类型：`number | null`
2. 是否必填：建议必填
3. 含义：线速度，单位米每秒

## 7.4 `angular_radps`

1. 类型：`number | null`
2. 是否必填：建议必填
3. 含义：角速度，单位弧度每秒

## 7.5 `pose`

1. 类型：`object`
2. 是否必填：建议必填
3. 含义：机器人当前位姿

子字段：

1. `x`
2. `y`
3. `yaw_deg`

## 7.6 `x`

1. 类型：`number | null`
2. 是否必填：建议必填
3. 含义：机器人地图坐标 X

## 7.7 `y`

1. 类型：`number | null`
2. 是否必填：建议必填
3. 含义：机器人地图坐标 Y

## 7.8 `yaw_deg`

1. 类型：`number | null`
2. 是否必填：建议必填
3. 含义：机器人朝向角，单位度

## 7.9 `network_latency_ms`

1. 类型：`number | null`
2. 是否必填：建议携带
3. 含义：网络延迟

## 7.10 `localization_status`

1. 类型：`string`
2. 是否必填：建议必填
3. 含义：定位状态

推荐枚举：

1. `good`
2. `normal`
3. `low_confidence`
4. `lost`
5. `recovering`

## 7.11 `localization_score`

1. 类型：`number | null`
2. 是否必填：建议携带
3. 含义：定位置信分数

## 7.12 `behavior_mode`

1. 类型：`string`
2. 是否必填：建议携带
3. 含义：当前行为模式

推荐枚举：

1. `idle`
2. `navigation`
3. `broadcast_wait`
4. `manual_pause`
5. `map_switching`

## 8. 系统健康字段

## 8.1 `health`

1. 类型：`object`
2. 是否必填：建议必填
3. 含义：系统健康状态对象

子字段：

1. `nav_module`
2. `localization_module`
3. `obstacle_module`
4. `power_module`
5. `comm_module`

每个字段推荐枚举：

1. `ok`
2. `warn`
3. `error`
4. `unknown`

## 9. 播报字段

## 9.1 `waiting`

1. 类型：`boolean`
2. 是否必填：播报状态消息中必填
3. 含义：是否正在等待播报完成

## 9.2 `broadcast_id`

1. 类型：`string`
2. 是否必填：等待播报时建议携带
3. 含义：播报 ID

## 9.3 `broadcast_text`

1. 类型：`string`
2. 是否必填：可选
3. 含义：当前播报文本

## 9.4 `waiting_since`

1. 类型：`number | null`
2. 是否必填：建议携带
3. 含义：开始等待播报的时间戳

## 9.5 `last_finished_at`

1. 类型：`number | null`
2. 是否必填：建议携带
3. 含义：最近一次播报完成时间

## 10. 障碍与人工脱困字段

## 10.1 `blocked`

1. 类型：`boolean`
2. 是否必填：障碍状态消息中必填
3. 含义：当前是否被阻塞

## 10.2 `reason`

1. 类型：`string | null`
2. 是否必填：建议携带
3. 含义：阻塞原因

推荐枚举：

1. `dynamic_obstacle`
2. `manual_escape`
3. `unknown`

## 10.3 `blocked_since`

1. 类型：`number | null`
2. 是否必填：建议携带
3. 含义：进入阻塞态的时间

## 10.4 `snapshot_url`

1. 类型：`string | null`
2. 是否必填：可选
3. 含义：障碍现场截图地址

## 10.5 `guidance_text`

1. 类型：`string | null`
2. 是否必填：建议携带
3. 含义：前端直接展示给操作员的引导文案

## 11. 切图字段

## 11.1 `switching`

1. 类型：`boolean`
2. 是否必填：切图状态消息中必填
3. 含义：当前是否处于切图中

## 11.2 `phase`

1. 类型：`string`
2. 是否必填：建议必填
3. 含义：切图阶段代码

推荐枚举：

1. `precheck`
2. `stopping_navigation`
3. `switching_map_asset`
4. `restarting_localization`
5. `waiting_localization_ready`
6. `completed`
7. `failed`

## 11.3 `phase_text`

1. 类型：`string`
2. 是否必填：建议必填
3. 含义：切图阶段中文说明

## 11.4 `ready_for_navigation`

1. 类型：`boolean`
2. 是否必填：建议必填
3. 含义：当前地图切换和定位恢复是否已准备好可导航

## 12. 录包字段

## 12.1 `recording`

1. 类型：`boolean`
2. 是否必填：录包状态消息中必填
3. 含义：当前是否正在录包

## 12.2 `record_name`

1. 类型：`string`
2. 是否必填：开始录包请求中建议必填
3. 含义：录包名称

## 12.3 `save_path`

1. 类型：`string | null`
2. 是否必填：录包状态消息中建议携带
3. 含义：录包保存路径

## 12.4 `file_name`

1. 类型：`string | null`
2. 是否必填：录包状态消息中建议携带
3. 含义：录包文件名或目录名

## 13. ACK 字段

## 13.1 `success`

1. 类型：`boolean`
2. 是否必填：ACK 与结果消息中必填
3. 含义：命令是否成功受理，或查询是否成功

## 13.2 `command_type`

1. 类型：`string`
2. 是否必填：`command_ack` 中必填
3. 含义：对应的命令类型

## 13.3 `accepted`

1. 类型：`boolean`
2. 是否必填：`command_ack` 中必填
3. 含义：命令是否被受理

## 13.4 `ack_message`

1. 类型：`string`
2. 是否必填：建议携带
3. 含义：ACK 中文说明

## 14. 错误字段

## 14.1 `error_code`

1. 类型：`string`
2. 是否必填：失败时必填；成功时建议空字符串
3. 含义：错误码

## 14.2 `error_message`

1. 类型：`string`
2. 是否必填：失败时必填；成功时建议空字符串
3. 含义：错误说明

## 14.3 `error_level`

1. 类型：`string`
2. 是否必填：主动错误推送时建议必填
3. 含义：错误级别

推荐枚举：

1. `info`
2. `warn`
3. `error`

## 14.4 `related_command_type`

1. 类型：`string`
2. 是否必填：可选
3. 含义：与错误相关的命令类型

## 15. 事件日志字段

## 15.1 `event`

1. 类型：`object`
2. 是否必填：日志推送中必填
3. 含义：日志对象

子字段：

1. `id`
2. `timestamp`
3. `level`
4. `category`
5. `title`
6. `detail`

## 15.2 `level`

1. 类型：`string`
2. 是否必填：建议必填
3. 含义：日志级别

推荐枚举：

1. `info`
2. `success`
3. `warn`
4. `error`

## 15.3 `category`

1. 类型：`string`
2. 是否必填：建议必填
3. 含义：日志分类

推荐枚举：

1. `system`
2. `nav`
3. `broadcast`
4. `map`
5. `obstacle`
6. `user`

## 16. 第一版前端解析建议

建议前端按以下方式理解字段：

1. `reason_code` 用于逻辑判断。
2. `reason_text` 优先用于页面展示。
3. `can_*` 字段优先用于按钮状态。
4. `navigation_main_state` 是页面主状态单一来源。
5. `route_task.sequence` 是路线列表和地图点位渲染的主要来源。

## 17. 当前建议

如果后续继续细化，可以再补一份：

1. `错误码字典.md`

把每个错误码的触发条件、前端表现、用户提示、是否允许重试全部单独写清楚。
