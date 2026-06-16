# APP与ROS交互全链路梳理

## 1. 文档目的

本文基于当前工作区代码，梳理 APP 与 ROS 侧的真实交互链路，重点回答：

1. APP 发出的各类命令分别进入哪个节点。
2. ROS 侧是通过话题、Action 还是别的方式继续向下游模块传递。
3. 导航、路点、机器人动作、面部表情、数据查询/订阅分别怎么走。
4. APP 收到的“命令已受理”和“业务真实完成/状态变化”分别来自哪里。

本文只基于当前仓库代码现状，不按未来方案推演。

## 2. 总体结论

当前链路的核心结论是：

1. APP 不直接连 Nav2，也不直接连导航节点。
2. APP 先通过 `humanoid_websocket` 包里的 `websocket_server.py` 接入 ROS。
3. `websocket_server` 收到 APP 消息后，不会直接执行业务，而是把命令转成 ROS 话题消息继续分发。
4. 导航链路不是“APP -> 直接往 Nav2 发目标”，而是：
   `APP -> WebSocket服务端 -> /app/navigation_command -> dynamic_waypoints_manager -> /navigation/requests -> navigation_state_manager -> Nav2 Action(navigate_to_pose)`
5. 状态回传也不是导航节点直接回 WebSocket，而是：
   `navigation_state_manager / websocket_client / 其它节点 -> data_integration_node_recoverable -> /integration/push_messages 或 /integration/data_responses -> websocket_server -> APP`
6. 所以当前系统本质上是“WebSocket网关 + ROS topic 总线 + Nav2 Action”的分层架构。

## 3. 关键节点职责

### 3.1 APP入口层

`src/humanoid_websocket/humanoid_websocket/websocket_server.py`

职责：

1. 接收 APP WebSocket 连接。
2. 解析统一 JSON 协议。
3. 把命令路由到不同 ROS 话题。
4. 把数据查询、订阅请求转给数据整合节点。
5. 把数据整合节点的响应/推送再发回 APP。

它创建的关键发布器：

1. `/app/waypoint_command`
2. `/app/navigation_command`
3. `/app/robot_control`
4. `/app/system_command`
5. `/robot/facial_raw_cmd`
6. `/initialpose`
7. `/websocket/data_requests`
8. `/websocket/data_subscriptions`

它订阅的关键回流话题：

1. `/navigation/waypoints_data`
2. `/integration/data_responses`
3. `/integration/push_messages`
4. `/integration/subscription_responses`

### 3.2 路点/导航编排层

`src/humanoid_navigation/humanoid_navigation/dynamic_waypoints_manager.py`

职责：

1. 管理点位增删改查。
2. 维护点位持久化文件 `data/dynamic_waypoints.json`。
3. 接收 `/app/waypoint_command` 和 `/app/navigation_command`。
4. 把导航控制命令再包装后发到 `/navigation/requests`。
5. 把当前点位全集发到 `/navigation/waypoints_data`。

这个节点不执行实际导航，它是“业务编排/点位数据中转层”。

### 3.3 导航执行层

`src/humanoid_navigation/humanoid_navigation/navigation_state_manager.py`

职责：

1. 接收 `/navigation/requests`。
2. 维护当前导航状态机。
3. 根据当前命令组织单点、多点、展台导航。
4. 通过 Nav2 Action `navigate_to_pose` 真正下发导航。
5. 发布导航 ACK 到 `/navigation/acknowledgments`。
6. 发布导航状态/事件到 `/navigation/status`。
7. 必要时发布 `/cmd_vel` 零速、发布定位恢复请求。

关键点：

1. 这个节点与 Nav2 的对接不是简单发 topic，而是 `ActionClient(NavigateToPose, 'navigate_to_pose')`。
2. 也就是说，真正驱动导航的是 Nav2 Action Server，不是某个自定义订阅 `/app/navigation_command` 的节点。

### 3.4 机器人本体桥接层

`src/humanoid_websocket/humanoid_websocket/websocket_client.py`

职责：

1. 主动连接机器人本体 WebSocket 服务器。
2. 接收机器人底层的 `notify_robot_info` 等消息。
3. 发布 `/robot_status_raw`、`/robot/action_result` 等 ROS 话题。
4. 订阅 `/app/robot_control` 和 `/cmd_vel`，再转成机器人厂商 WebSocket 命令。

所以机器人动作控制链路里，ROS 侧还存在第二个 WebSocket 客户端，它不是给 APP 用的，而是给机器人本体用的。

### 3.5 数据汇聚/推送层

`src/humanoid_websocket/humanoid_websocket/data_integration_node_recoverable.py`

职责：

1. 订阅 ROS 内部各类状态话题。
2. 把它们整理成统一 JSON 数据。
3. 处理 APP 的数据请求和订阅请求。
4. 周期推送、事件立即推送、异常推送。
5. 再通过 `/integration/*` 话题把数据送回 `websocket_server`。

### 3.6 原始数据标准化层

`src/humanoid_websocket/humanoid_websocket/message_bridge.py`

职责：

1. 把 `/imu_raw` 转成标准 `/imu`。
2. 把 `/joint_states_raw` 转成标准 `/joint_states`。
3. 把 `/robot_status_raw` 进一步加工成 `/robot_status_processed`。

这个节点不是 APP 命令入口，而是“原始数据格式桥接器”。

## 4. APP侧消息分类

当前 `websocket_server` 识别 5 类消息：

1. `request`
2. `response`
3. `push`
4. `subscription`
5. `command`

其中真正与 APP 主动交互最相关的是 3 类：

1. `command`：APP 下发业务命令。
2. `request`：APP 主动拉一次数据。
3. `subscription`：APP 订阅持续推送。

### 4.1 当前代码声明支持的数据类型

连接确认里声明的主要 `supported_data_types`：

1. `robot_pose`
2. `robot_speed`
3. `navigation_path`
4. `navigation_path_monitor`
5. `gait_velocity_test`
6. `navigation_status`
7. `system_status`
8. `waypoints_data`

数据整合节点实际稳定支持并配置了推送频率的主要类型：

1. `robot_pose`
2. `robot_speed`
3. `navigation_status`
4. `navigation_path`
5. `system_status`
6. `gesture_list`
7. `facial_gesture_list`
8. `system_exception`

### 4.2 当前代码声明支持的命令类型

`websocket_server` 里对外声明/实现的命令主要有：

1. `waypoint_management`
2. `navigation_control`
3. `robot_control`
4. `facial_control`
5. `initial_pose`

注意：

1. 代码里虽然创建了 `/app/system_command` 发布器，也有 `handle_system_command()`。
2. 但 `handle_business_command()` 当前没有把 `system_command` 路由进去。
3. 也就是说，`system_command` 目前在现代码里属于“预留了发布能力，但主分发入口没有真正接上”。

## 5. 真实链路一：路点管理

### 5.1 APP发命令

APP 发 `command`，`data_type=waypoint_management`。

内部 `data.command_type` 典型包括：

1. `set_waypoint`
2. `update_waypoint`
3. `delete_waypoint`
4. `get_waypoints`
5. `clear_waypoints`

### 5.2 ROS内部分发

链路如下：

1. APP -> `websocket_server.handle_business_command()`
2. `websocket_server.route_to_waypoint_manager()`
3. 发布到 `/app/waypoint_command`
4. `dynamic_waypoints_manager.app_waypoint_callback()` 订阅该话题
5. 进入对应的 `handle_set_waypoint()` / `handle_update_waypoint()` / `handle_delete_waypoint()` 等

### 5.3 实际处理方式

这里完全是通过 ROS 话题传递 JSON 字符串，不是 service。

处理结果：

1. 点位更新后保存到 `data/dynamic_waypoints.json`
2. 再通过 `/navigation/waypoints_data` 发布统一格式点位数据
3. `websocket_server` 订阅 `/navigation/waypoints_data`
4. 点位变更再被广播回 APP

### 5.4 返回APP的消息

点位处理结果不是同步函数返回给 APP，而是通过统一消息发布回流：

1. `dynamic_waypoints_manager.send_app_response()`
2. 发布到 `/navigation/waypoints_data`
3. `websocket_server.business_waypoints_callback()` 收到
4. 广播给所有 WebSocket 客户端

所以路点管理也是“topic 异步回传”，不是“收到命令函数内直接产生最终业务结果”。

## 6. 真实链路二：导航命令

这是本文的主链路。

### 6.1 APP发开始导航命令

APP 发 `command`，`data_type=navigation_control`。

内部 `data.command_type` 典型包括：

1. `start_single_navigation`
2. `start_multi_point_navigation`
3. `start_exhibition_navigation`
4. `stop_navigation`
5. `pause_navigation`
6. `resume_navigation`
7. `retry_failed_waypoint`
8. `skip_failed_waypoint`
9. `abort_failed_navigation`

### 6.2 APP到ROS入口

链路如下：

1. APP -> `websocket_server.handle_business_command()`
2. `websocket_server.route_to_waypoint_manager()`
3. 发布 JSON 到 `/app/navigation_command`

这里还是先发 ROS topic，不是直接调 Nav2。

### 6.3 路点管理器二次包装

`dynamic_waypoints_manager.app_navigation_callback()` 收到 `/app/navigation_command` 后：

1. 不执行导航。
2. 调 `send_navigation_request()`。
3. 重新包装成 `request_type=navigation_command` 的请求结构。
4. 必要时把单点/多点对应的点位详情一起塞进请求。
5. 发布到 `/navigation/requests`。

所以 `/app/navigation_command` 只是 APP 控制入口；
`/navigation/requests` 才是导航状态管理器真正消费的内部业务请求总线。

### 6.4 导航状态管理器接管

`navigation_state_manager.navigation_request_callback()` 订阅 `/navigation/requests`：

1. 解析 `request_type`
2. 若是 `navigation_command`
3. 调 `handle_navigation_command()`

然后根据 `command_type` 进入：

1. `handle_start_single_navigation()`
2. `handle_start_multi_point_navigation()`
3. `handle_start_exhibition_navigation()`
4. `handle_stop_navigation()`
5. `handle_pause_navigation()`
6. `handle_resume_navigation()`
7. 失败恢复相关处理函数

### 6.5 真正下发给导航系统的方式

不是简单往某个“导航节点订阅话题”里发目标，而是：

1. `navigate_to_waypoint()` 构造 `NavigateToPose.Goal`
2. 调 `self.nav_to_pose_client.send_goal_async(...)`
3. 对接 Nav2 的 `navigate_to_pose` Action Server

因此可以明确得到以下结论：

1. APP 发开始导航后，ROS 侧前半段确实是“往 topic 里发 JSON，再由消费节点读取处理”。
2. 但到了真正下发给导航系统这一步，不再是自定义 topic，而是 ROS2 Action。
3. 也就是：
   - APP 到业务管理层：topic
   - 业务管理层到 Nav2：Action

### 6.6 Nav2执行中的回调

`navigation_state_manager` 对 Nav2 有 3 个关键异步回调：

1. `nav2_goal_response_callback()`
   - 判断目标是否被 Nav2 接受
2. `nav2_feedback_callback()`
   - 接收剩余距离、剩余时间等反馈
3. `nav2_result_callback()`
   - 接收最终成功/取消/失败

再分流到：

1. `handle_nav2_succeeded()`
2. `handle_nav2_cancelled()`
3. `handle_nav2_failed()`

### 6.7 导航状态怎么回给APP

导航状态不是在 `websocket_server` 里直接生成，而是分两路回流：

#### A. 命令受理/失败类 ACK

1. `navigation_state_manager.send_acknowledgment()`
2. 发布到 `/navigation/acknowledgments`
3. `data_integration_node_recoverable.navigation_ack_callback()` 订阅
4. 转换成 `push`，`data_type=navigation_command_result`
5. 发布到 `/integration/push_messages`
6. `websocket_server.unified_push_message_callback()` 收到
7. 转发给 APP

这条链主要对应：

1. `navigation_started`
2. `navigation_failed`
3. `navigation_completed`
4. `navigation_resumed`
5. `navigation_pending`
6. 其它导航命令结果

#### B. 真实运行状态/事件

1. `navigation_state_manager.publish_status_update()`
2. 或 `publish_navigation_status()`
3. 发布到 `/navigation/status`
4. `data_integration_node_recoverable.navigation_status_callback()` 订阅
5. 存入 `data_storage['navigation_status']`
6. 若是离散事件，立即调用 `publish_navigation_status_event()`
7. 同时订阅用户还会按频率收到周期状态推送
8. 最终通过 `/integration/push_messages` -> `websocket_server` -> APP

也就是说：

1. ACK 用 `/navigation/acknowledgments`
2. 真实过程/状态用 `/navigation/status`

这两类必须区分。

## 7. 真实链路三：机器人动作控制

这里的“动作控制”不是 Nav2，而是机器人本体动作库。

### 7.1 APP发动作命令

APP 发 `command`，`data_type=robot_control`。

当前 `websocket_server` 会取：

1. `data.action`
2. `data.parameters`

然后包装为：

1. `command_type = action`
2. `parameters = ...`

再发布到 `/app/robot_control`。

### 7.2 ROS内部分发

链路如下：

1. APP -> `websocket_server.handle_robot_control()`
2. 发布到 `/app/robot_control`
3. `websocket_client.robot_control_callback()` 订阅

当前真正实现的主分支是：

1. `command_type == "execute_gesture"`

它会继续取：

1. `parameters.gesture_id`

然后启动动作线程，进入 `execute_upper_body_motion()`。

### 7.3 ROS到机器人本体的方式

这里不是 ROS topic 往下传，而是：

1. `websocket_client` 本身就是机器人本体 WebSocket 客户端
2. 通过 `send_command()` 给机器人底层 WebSocket 服务发厂商协议命令

典型过程：

1. `request_set_motion_engine` 切到 `Menu`
2. `request_execute_atomic_motion` 执行动作
3. 等待底层动作完成通知
4. `request_set_motion_engine` 切回 `Walk`

### 7.4 动作结果怎么回APP

动作完成后：

1. `websocket_client.publish_action_result()`
2. 发布到 `/robot/action_result`
3. `data_integration_node_recoverable.action_result_callback()` 订阅
4. 立即转成推送消息
5. 经 `/integration/push_messages` -> `websocket_server` -> APP

所以动作链路是：

`APP -> websocket_server -> /app/robot_control -> websocket_client -> 机器人本体WebSocket -> /robot/action_result -> data_integration -> websocket_server -> APP`

## 8. 真实链路四：表情/面部控制

### 8.1 APP发命令

APP 发 `command`，`data_type=facial_control`。

内部关键字段是：

1. `data.action`

### 8.2 ROS内部分发

链路如下：

1. APP -> `websocket_server.handle_facial_control()`
2. 直接把 `action` 原文发布到 `/robot/facial_raw_cmd`

这里没有经过 `dynamic_waypoints_manager`，也没有经过 `websocket_client`。

### 8.3 真正执行端

`src/humanoid_locomotion/humanoid_locomotion/facial_driver.py`

订阅：

1. `/robot/facial_raw_cmd`

执行方式：

1. 从 `facial_gestures.yaml` 查动作序列
2. 通过串口发送给仿生头控制板

因此这条链路是：

`APP -> websocket_server -> /robot/facial_raw_cmd -> facial_driver -> 串口 -> 仿生头硬件`

### 8.4 回APP现状

当前面部控制在 `websocket_server.handle_facial_control()` 内部会立即给 APP 一个 ACK：

1. `status=executed`

但这只是“指令已发布到 ROS 侧”，不是串口硬件执行完成回执。

当前代码里没有看到表情执行完成后的独立结果回流链路。

所以表情控制目前更接近：

1. 有命令受理 ACK
2. 无完整硬件完成闭环反馈

## 9. 真实链路五：初始位姿设置

### 9.1 APP发命令

APP 发 `command`，`data_type=initial_pose`。

字段典型包括：

1. `x`
2. `y`
3. `yaw`
4. `frame_id`

### 9.2 ROS处理方式

`websocket_server.handle_initial_pose()`：

1. 把 `(x, y, yaw)` 转成 `PoseWithCovarianceStamped`
2. 发布到 `/initialpose`

所以这条链是标准 ROS 定位初始化方式，相当于代替 RViz 的 “2D Pose Estimate”。

## 10. 真实链路六：APP查询数据

### 10.1 APP发 request

APP 发 `message_type=request`，例如：

1. `robot_pose`
2. `navigation_status`
3. `navigation_path`
4. `system_status`
5. `gesture_list`

### 10.2 ROS内部分发

链路如下：

1. APP -> `websocket_server.handle_data_request()`
2. 原始请求发布到 `/websocket/data_requests`
3. `data_integration_node_recoverable.data_request_callback()` 订阅
4. 检查数据类型支持、是否可用、是否过期
5. 从内存 `data_storage` 里取数据
6. 发布到 `/integration/data_responses`
7. `websocket_server.unified_data_response_callback()` 再发回 APP

### 10.3 数据来源

`data_integration_node_recoverable` 的主要数据来源：

1. `/robot_realpose` -> `robot_pose`
2. `/odom`、`/robot_realpose` -> `robot_speed`
3. `/navigation/status` -> `navigation_status`
4. `/plan` -> `navigation_path`
5. `/robot_status_processed` -> `system_status`
6. 本地 YAML -> `gesture_list`、`facial_gesture_list`

所以数据查询不是实时向业务节点现算，而是“先由数据整合节点缓存，再响应查询”。

## 11. 真实链路七：APP订阅数据推送

### 11.1 APP发 subscription

APP 发 `message_type=subscription`。

典型字段：

1. `data.action = subscribe/unsubscribe`
2. `data.data_types = [...]`
3. `data.push_frequency = ...`

### 11.2 ROS内部分发

链路如下：

1. APP -> `websocket_server.handle_subscription_request()`
2. 转发到 `/websocket/data_subscriptions`
3. `data_integration_node_recoverable.subscription_callback()` 订阅
4. `SubscriptionManager` 记录订阅关系
5. 定时器 `push_data_updates()` 按频率检查并推送
6. 推送消息发到 `/integration/push_messages`
7. `websocket_server` 根据 `destination` 再发给对应连接端

### 11.3 推送机制

数据整合节点有两种推送：

1. 周期推送
   - 例如 `robot_pose`、`robot_speed`、`navigation_status`
2. 事件立即推送
   - 例如 `waypoint_started`、`navigation_completed`、`navigation_failed`、动作结果、异常事件

所以订阅并不意味着只会收到定时消息，也会收到某些离散事件的立即推送。

## 12. 机器人状态数据怎么来的

这部分也很重要，因为 APP 常常要显示电量、控制模式、动作忙闲、是否可开始导航。

链路如下：

1. `websocket_client` 连接机器人本体 WebSocket
2. 收到底层 `notify_robot_info`
3. 解析后发布 `/robot_status_raw`
4. `message_bridge.robot_status_callback()` 再加工
5. 发布 `/robot_status_processed`
6. `data_integration_node_recoverable.robot_status_callback()` 订阅
7. 存成 `system_status`
8. APP 通过 request/subscription 获得

因此：

1. APP 看到的系统状态主要不是直接来自导航节点。
2. 而是来自机器人底层 WebSocket -> ROS桥接 -> 数据整合节点。

## 13. “topic处理”与“Action处理”的边界

这个问题可以明确回答成下面这样：

### 13.1 纯 topic 异步处理的部分

1. APP 到 `websocket_server`
2. `websocket_server` 到 `dynamic_waypoints_manager`
3. `dynamic_waypoints_manager` 到 `navigation_state_manager`
4. `websocket_server` 到 `data_integration_node_recoverable`
5. `websocket_client` / `message_bridge` / `navigation_state_manager` 到 `data_integration_node_recoverable`
6. `data_integration_node_recoverable` 到 `websocket_server`
7. 表情控制到 `facial_driver`

这些都主要是：

1. 往某个 ROS topic 里发 `std_msgs/String` JSON
2. 由订阅节点解析再处理

### 13.2 Action 的部分

真正的导航执行：

1. `navigation_state_manager`
2. `ActionClient(NavigateToPose, 'navigate_to_pose')`
3. Nav2 Action Server

这是当前导航执行最核心的非 topic 机制。

### 13.3 非ROS topic 的部分

还有两条链路不止 ROS topic：

1. `websocket_client` 与机器人本体之间是 WebSocket 厂商协议。
2. `facial_driver` 与仿生头之间是串口。

## 14. ACK 与真实状态的区别

当前代码严格分成两类回包：

### 14.1 ACK/受理结果

含义：

1. 命令收到了。
2. 格式通过了。
3. 当前阶段允许尝试执行。

典型来源：

1. `websocket_server` 对 request/subscription/部分 command 的即时 ACK
2. `navigation_state_manager.send_acknowledgment()` 对导航命令受理结果
3. `websocket_server.handle_facial_control()` 对表情命令受理结果

### 14.2 真实业务状态/完成结果

含义：

1. 导航真正启动/暂停/恢复/到达/失败。
2. 动作真正完成。
3. 定位恢复真正发生。
4. 系统异常真正发生。

典型来源：

1. `/navigation/status`
2. `/robot/action_result`
3. `/localization/recovery_status`
4. `/robot_status_processed`

APP 如果要做可靠 UI，应该优先以这类推送为准。

## 15. 当前代码里已经实现的主要交互内容

### 15.1 已有完整闭环

1. 路点增删改查
2. 单点导航
3. 多点导航
4. 展台导航
5. 停止/暂停/恢复导航
6. 导航失败恢复类事件回推
7. 动作执行与动作结果回推
8. 数据查询
9. 数据订阅
10. 初始位姿设置

### 15.2 已有入口但闭环不完整

1. 面部表情控制
   - 有命令入口
   - 有串口执行
   - 但没有看到独立的“执行完成结果”回推
2. `system_command`
   - 有发布器和处理函数
   - 但主命令分发没有真正接入

## 16. 两个核心问题的结论

### 16.1 “ROS侧接收到这些命令后，内部从接收到处理完成到给app返回对应消息推送，整个流程怎么处理的？”

答案是：

1. 先由 `websocket_server` 接 APP WebSocket 消息。
2. 根据消息类型转成 ROS 内部话题消息。
3. 由 `dynamic_waypoints_manager`、`navigation_state_manager`、`websocket_client`、`facial_driver`、`data_integration_node_recoverable` 等节点分别消费。
4. 业务节点执行后，不直接回 WebSocket，而是再发 ROS 话题。
5. `data_integration_node_recoverable` 统一做数据缓存、增强、事件推送、异常推送。
6. 最终由 `websocket_server` 把 `/integration/*` 里的消息转回 APP。

### 16.2 “是通过往话题里面发送数据，然后消费节点读到话题里面的数据进行处理还是怎么处理的？”

答案是：

1. 大部分业务编排链路，确实是“往 ROS topic 里发送 JSON 字符串，再由消费节点订阅处理”。
2. 但真正导航目标下发给 Nav2 时，不是普通 topic，而是 ROS2 Action。
3. 机器人动作控制到机器人本体时，也不是 ROS topic，而是 WebSocket 厂商协议。
4. 表情控制到仿生头硬件时，是串口协议。

所以准确说法应当是：

1. APP 到 ROS 业务层：主要靠 topic
2. ROS 业务层到 Nav2：靠 Action
3. ROS 到机器人本体：靠 WebSocket
4. ROS 到表情硬件：靠串口

## 17. 后续可补充的内容

后续如需继续补充，可优先增加两份材料：

1. “按命令类型分类的时序图版”
   - 开始导航、暂停恢复、动作执行、表情控制、数据订阅分别画时序
2. “APP字段到ROS内部字段映射表”
   - 把每个 JSON 字段最终落到哪个 topic、哪个函数、哪个状态字段，列成表

这样后续无论用于 APP 联调、后端网关对接，还是现场排障，都可以直接使用。

## 18. 消息总线矩阵

这一节按“话题/接口 -> 谁发 -> 谁收 -> 用途”列出，适合作为二开时的总线参考。

### 18.1 APP入口与业务命令总线

| 话题/接口 | 生产者 | 消费者 | 作用 |
| --- | --- | --- | --- |
| WebSocket `command` | APP | `websocket_server` | APP 业务命令总入口 |
| `/app/waypoint_command` | `websocket_server` | `dynamic_waypoints_manager` | 点位增删改查命令 |
| `/app/navigation_command` | `websocket_server` | `dynamic_waypoints_manager` | 导航控制命令入口 |
| `/app/robot_control` | `websocket_server` | `websocket_client` | 机器人动作控制入口 |
| `/app/system_command` | `websocket_server` | 当前未见实际消费者 | 预留系统命令总线 |
| `/robot/facial_raw_cmd` | `websocket_server`、`data_integration` | `facial_driver` | 面部表情/原始表情指令 |
| `/initialpose` | `websocket_server` | AMCL/定位相关节点 | 设置初始位姿 |

### 18.2 路点与导航业务编排总线

| 话题/接口 | 生产者 | 消费者 | 作用 |
| --- | --- | --- | --- |
| `/navigation/requests` | `dynamic_waypoints_manager` | `navigation_state_manager` | 导航业务请求入口 |
| `/navigation/waypoints_data` | `dynamic_waypoints_manager` | `navigation_state_manager`、`websocket_server` | 当前点位全集/点位响应 |
| `/navigation/acknowledgments` | `navigation_state_manager` | `dynamic_waypoints_manager`、`data_integration` | 导航命令 ACK/受理结果 |
| `/navigation/status` | `navigation_state_manager` | `data_integration`、其它监控节点 | 导航状态摘要与离散事件 |
| `/navigation/current_path` | `navigation_state_manager` | 调试/可视化节点 | 当前剩余路点路径 |
| `/goal_pose` | `navigation_state_manager` | 调试/可视化用途 | 当前目标位姿镜像 |

### 18.3 真正导航执行接口

| 接口 | 调用者 | 被调用者 | 作用 |
| --- | --- | --- | --- |
| `navigate_to_pose` Action | `navigation_state_manager` | Nav2 Action Server | 真正执行导航目标 |
| `cancel_goal_async()` | `navigation_state_manager` | Nav2 Action Server | 取消当前导航 |
| `/cmd_vel` | `navigation_state_manager`、teleop 等 | `websocket_client` | 零速保护/手动速度控制 |

### 18.4 机器人本体与原始数据桥接总线

| 话题/接口 | 生产者 | 消费者 | 作用 |
| --- | --- | --- | --- |
| 机器人本体 WebSocket | 机器人底层 | `websocket_client` | 底层状态与动作执行接口 |
| `/robot_status_raw` | `websocket_client` | `message_bridge`、`navigation_state_manager` | 机器人原始状态 |
| `/robot_status_processed` | `message_bridge` | `data_integration` | 供 APP 使用的系统状态 |
| `/robot/action_result` | `websocket_client` | `data_integration` | 动作最终结果 |
| `/imu_raw` | 外部原始来源 | `message_bridge` | 原始 IMU 数据 |
| `/imu` | `message_bridge` | 导航/其它节点 | 标准 IMU |
| `/joint_states_raw` | 外部原始来源 | `message_bridge` | 原始关节数据 |
| `/joint_states` | `message_bridge` | 其它 ROS 节点 | 标准关节数据 |
| `/joy_raw` | `websocket_client` | 遥控相关节点 | 遥控器按键/摇杆数据 |

### 18.5 数据查询/订阅/回推总线

| 话题/接口 | 生产者 | 消费者 | 作用 |
| --- | --- | --- | --- |
| WebSocket `request` | APP | `websocket_server` | 单次拉取数据 |
| `/websocket/data_requests` | `websocket_server` | `data_integration` | 请求内部总线 |
| `/integration/data_responses` | `data_integration` | `websocket_server` | 单次响应回流 |
| WebSocket `subscription` | APP | `websocket_server` | 订阅/退订 |
| `/websocket/data_subscriptions` | `websocket_server` | `data_integration` | 订阅管理内部总线 |
| `/integration/subscription_responses` | `data_integration` | `websocket_server` | 订阅操作结果回流 |
| `/integration/push_messages` | `data_integration` | `websocket_server` | 周期推送/事件推送/异常推送 |

### 18.6 定位恢复与故障恢复相关总线

| 话题/接口 | 生产者 | 消费者 | 作用 |
| --- | --- | --- | --- |
| `/localization/recovery_status` | 定位恢复相关节点 | `navigation_state_manager`、`data_integration` | 定位恢复进度与结果 |
| `/localization/recovery_requests` | `navigation_state_manager` | 定位恢复相关节点 | 请求重定位/上下文恢复 |
| `/localization/prior_map_odom_bridge_status` | prior-map bridge | `navigation_state_manager` | 定位健康状态 |

## 19. 逐功能详细时序

这一节把最核心的几条链用“步骤流”的形式写出来。

### 19.1 设置路点

1. APP 通过 WebSocket 发送 `command`。
2. `message_type=command`，`data_type=waypoint_management`。
3. `websocket_server` 校验消息结构。
4. `route_to_waypoint_manager()` 将内层 `command_type`、`waypoint_data` 等拍平成 JSON。
5. 发布到 `/app/waypoint_command`。
6. `dynamic_waypoints_manager.app_waypoint_callback()` 收到后根据 `command_type` 分流。
7. 若是 `set_waypoint`/`update_waypoint`：
   - 校验速度等 `properties`
   - 更新内存中的 `self.waypoints`
   - 保存到 `data/dynamic_waypoints.json`
8. `dynamic_waypoints_manager.publish_waypoints_data()` 把全量点位重新发布到 `/navigation/waypoints_data`。
9. `navigation_state_manager` 更新本地点位缓存。
10. `websocket_server.business_waypoints_callback()` 也收到同一条消息，并广播给 APP。

二开建议：

1. 如需接入自定义点位存储系统，可替换 `dynamic_waypoints_manager` 的持久化逻辑。
2. 如只需监听点位变化，直接订阅 `/navigation/waypoints_data` 即可。

### 19.2 开始单点导航

1. APP 发送 `command`，`data_type=navigation_control`。
2. 内层 `command_type=start_single_navigation`，携带 `waypoint_id`。
3. `websocket_server` 发布到 `/app/navigation_command`。
4. `dynamic_waypoints_manager.app_navigation_callback()` 收到。
5. `send_navigation_request()` 从本地点位表中找到该点位详情。
6. 组装 `request_type=navigation_command` 的请求。
7. 发布到 `/navigation/requests`。
8. `navigation_state_manager.navigation_request_callback()` 收到。
9. 调 `handle_navigation_command()` -> `handle_start_single_navigation()`。
10. 状态管理器检查：
    - 当前是否空闲
    - prior-map 定位是否允许启动
    - 机器人是否处于可导航状态
11. 如果通过，调用 `send_acknowledgment("navigation_started", ...)` 发布到 `/navigation/acknowledgments`。
12. 然后调用 `navigate_to_waypoint()`。
13. `navigate_to_waypoint()`：
    - 把点位转成 `PoseStamped`
    - 构造 `NavigateToPose.Goal`
    - 调 `send_goal_async()`
14. Nav2 Action Server 接受目标后，进入 `nav2_goal_response_callback()`。
15. 导航过程中 Nav2 不断触发 `nav2_feedback_callback()`。
16. 状态管理器持续发布：
    - 周期摘要 `publish_navigation_status()`
    - 离散事件 `publish_status_update("waypoint_started"/"navigation_progress_update"... )`
17. `data_integration` 接收 `/navigation/status`。
18. 若是离散事件，则立即经 `/integration/push_messages` 推到 APP。
19. 若 APP 订阅了 `navigation_status`，也会周期收到状态。
20. 当 Nav2 成功到达，`nav2_result_callback()` -> `handle_nav2_succeeded()`。
21. 发布 `waypoint_reached`，若是最后一个点，再发布 `navigation_completed`。
22. ACK 和事件都最终由 `data_integration` + `websocket_server` 回推 APP。

### 19.3 开始多点导航

和单点导航相比，差异主要在：

1. APP 下发 `command_type=start_multi_point_navigation`。
2. `dynamic_waypoints_manager` 会把所有 `waypoint_ids` 展开成 `waypoints_data` 一起附带到 `/navigation/requests`。
3. `navigation_state_manager.handle_start_multi_point_navigation()` 验证每个点位都存在。
4. `start_navigation_sequence()` 先取第一个点位。
5. 每次 `handle_nav2_succeeded()` 后：
   - 当前索引加 1
   - 若还有剩余点位，1 秒后自动 `navigate_to_waypoint(next_waypoint_data)`
6. 直到所有点位完成，再发 `navigation_completed`。

这说明当前多点导航不是一次性发 `NavigateThroughPoses`，而是“状态管理器自己循环多次调用 `NavigateToPose`”。

### 19.4 暂停导航

1. APP 发 `command_type=pause_navigation`。
2. 仍然经过 `websocket_server -> /app/navigation_command -> dynamic_waypoints_manager -> /navigation/requests`。
3. `navigation_state_manager.handle_pause_navigation()` 执行：
   - 检查当前状态必须是 `EXECUTING`
   - 把状态切成 `PAUSED`
   - 调 `cancel_navigation()` 取消当前 Nav2 goal
4. Nav2 取消是通过 Action `cancel_goal_async()` 完成，不是发 topic。
5. 状态管理器发布：
   - `/navigation/acknowledgments`：`pause_navigation success`
   - `/navigation/status`：`navigation_paused`
6. `data_integration` 把 ACK 转成 `navigation_command_result`，把事件转成 `navigation_status` 即时推送。

### 19.5 恢复导航

1. APP 发 `command_type=resume_navigation`。
2. 状态管理器验证当前必须是 `PAUSED`。
3. 如果不是定位恢复锁定状态，也不是失败恢复受限状态，则：
   - 切回 `EXECUTING`
   - 发布 `navigation_resumed`
   - 再次对 `self.current_waypoint` 调 `navigate_to_waypoint()`
4. 所以“恢复导航”本质是对当前缓存 waypoint 重新发一次 Nav2 goal。

### 19.6 停止导航

1. APP 发 `command_type=stop_navigation`。
2. 状态管理器执行 `handle_stop_navigation()`：
   - 如果有 pending request，先取消 pending
   - 否则取消当前 Nav2 goal
   - 补发一帧零速度 `publish_zero_cmd_vel()`
   - 发布 `navigation_stopped`
   - 重置本地状态机

### 19.7 动作执行

1. APP 发 `command`，`data_type=robot_control`。
2. 当前有效实现是 `data.action=execute_gesture`。
3. `websocket_server.handle_robot_control()` 把它转成：
   - `command_type=execute_gesture`
   - `parameters.gesture_id=...`
4. 发布到 `/app/robot_control`。
5. `websocket_client.robot_control_callback()` 收到。
6. 如果当前无动作在执行，则启动后台线程 `_run_motion_task()`。
7. 线程里通过 `asyncio.run_coroutine_threadsafe()` 把动作执行任务投递到机器人本体 WebSocket loop。
8. `execute_upper_body_motion()` 顺序执行：
   - 如不在 `Menu`，先发 `request_set_motion_engine(mode=1)`
   - 等待机器人状态切到 `Menu`
   - 发 `request_execute_atomic_motion`
   - 等待机器人底层完成通知
   - 再发 `request_set_motion_engine(mode=0)` 切回 `Walk`
9. 执行完后 `publish_action_result()` 发布 `/robot/action_result`。
10. `data_integration.action_result_callback()` 把它转成 `push action_result` 立即推给 APP。

二开切入点：

1. 如果要换成另一套机器人动作协议，主要改 `websocket_client.py`。
2. 如果只想额外监听动作结果，订阅 `/robot/action_result` 即可。

### 19.8 表情/面部控制

1. APP 发 `command`，`data_type=facial_control`。
2. `websocket_server.handle_facial_control()` 直接提取 `data.action`。
3. 发布字符串到 `/robot/facial_raw_cmd`。
4. `facial_driver.callback()` 收到后：
   - 如表情库里有该动作，按 YAML 中配置的命令序列逐条发串口
   - 如表情库没有，尝试当原始串口指令直发
5. `websocket_server` 会立刻回一个 `command_ack` 给 APP。

注意：

1. 这是“ROS 已接收并下发表情指令”的 ACK。
2. 当前代码没有看到单独的“表情硬件执行完成”事件上报链路。

### 19.9 数据查询

1. APP 发 `request`。
2. `websocket_server` 不解析业务内容，只校验后原样发到 `/websocket/data_requests`。
3. `data_integration.data_request_callback()` 收到后：
   - 校验 `message_type=request`
   - 取 `data_type`
   - 检查是否支持
   - 检查数据是否可用、是否新鲜
4. 然后从 `data_storage` 中取缓存。
5. 经 `prepare_data_response()` 做轻量裁剪。
6. 发布到 `/integration/data_responses`。
7. `websocket_server.unified_data_response_callback()` 把它路由给目标 client。

### 19.10 数据订阅

1. APP 发 `subscription`。
2. `websocket_server` 更新本地 `client_subscriptions` 镜像，并转发到 `/websocket/data_subscriptions`。
3. `data_integration.subscription_callback()` 调 `SubscriptionManager.subscribe()/unsubscribe()`。
4. `send_subscription_response()` 经 `/integration/subscription_responses` 回给 `websocket_server`。
5. `data_integration.push_timer` 每 0.1s 检查一次哪些数据类型该推。
6. 按每种数据的 `push_configs` 频率控制推送。
7. 推送消息经 `/integration/push_messages` 返回，由 `websocket_server.route_push_message()` 根据订阅关系发送给对应 client。

## 20. ACK、事件、周期状态三种消息的区别

二开时这三类消息最容易混淆，必须区分。

### 20.1 ACK

来源：

1. `websocket_server` 立即返回
2. 或 `navigation_state_manager.send_acknowledgment()`

含义：

1. 命令已收到
2. 语法正确
3. 当前允许尝试执行

不能表示：

1. 导航一定已完成
2. 动作一定已完成
3. 表情硬件一定已执行

### 20.2 离散事件

来源：

1. `navigation_state_manager.publish_status_update()`
2. `websocket_client.publish_action_result()`
3. 定位恢复状态回调
4. 异常推送

典型事件：

1. `waypoint_started`
2. `waypoint_reached`
3. `navigation_completed`
4. `navigation_failed`
5. `navigation_paused`
6. `navigation_resumed`
7. `action_result`
8. `system_exception`

这类消息最适合驱动 APP 的状态切换和弹窗。

### 20.3 周期状态

来源：

1. `navigation_state_manager.publish_navigation_status()`
2. `data_integration` 定时推送

典型内容：

1. 当前状态摘要
2. 当前位姿
3. 当前速度
4. 当前系统状态

这类消息最适合驱动页面上的持续展示区域。

## 21. 当前支持立即推送的导航离散事件

`data_integration.should_push_navigation_status_immediately()` 当前配置会立即推送这些导航事件：

1. `waypoint_started`
2. `waypoint_reached`
3. `navigation_completed`
4. `navigation_stopped`
5. `navigation_failed`
6. `navigation_cancelled`
7. `navigation_paused`
8. `navigation_resumed`
9. `navigation_obstacle_blocked`
10. `navigation_pending_cancelled`
11. `navigation_retry_failed_waypoint`
12. `navigation_skip_failed_waypoint`
13. `navigation_aborted`
14. `navigation_localization_recovery_requested`
15. `navigation_localization_recovery_started`
16. `navigation_localization_recovery_progress`
17. `navigation_localization_recovery_failed`
18. `navigation_localization_manual_override`
19. `navigation_localization_recovered`
20. `navigation_localization_resume_waiting`
21. `navigation_localization_resume_failed`

这意味着：

1. 如需在 APP 里做“到点播报”“失败弹窗”“定位恢复提示”，应优先消费这些立即推送事件。

## 22. 二开插入点建议

### 22.1 修改 APP 协议

优先改：

1. `websocket_server.py`

适合做：

1. 增加新的 `data_type`
2. 增加新的 `command`
3. 调整 APP 消息字段格式
4. 做鉴权、租户、session 控制

### 22.2 修改点位管理逻辑

优先改：

1. `dynamic_waypoints_manager.py`

适合做：

1. 点位字段扩展
2. 持久化改数据库
3. 点位批量导入导出
4. 点位分组/场景管理

### 22.3 修改导航控制逻辑

优先改：

1. `navigation_state_manager.py`

适合做：

1. 新的导航命令
2. 路线编排
3. 到点判定策略
4. 暂停/恢复/失败恢复策略
5. 与 Nav2 的交互方式

注意：

1. 当前这里是 `NavigateToPose` 多次串行。
2. 如需改成 `NavigateThroughPoses` 或自定义导航器，主要在这里调整。

### 22.4 修改机器人动作协议

优先改：

1. `websocket_client.py`

适合做：

1. 更换厂商动作协议
2. 调整动作引擎切换逻辑
3. 动作结果格式统一

### 22.5 修改表情控制硬件

优先改：

1. `facial_driver.py`

适合做：

1. 串口协议替换
2. 改成 CAN/UDP/SDK
3. 增加完成回执

### 22.6 新增 APP 实时展示面板

优先改：

1. `data_integration_node_recoverable.py`

适合做：

1. 新增缓存数据类型
2. 新增推送频率控制
3. 新增异常事件模型
4. 把多个 ROS 话题聚合成一个对 APP 友好的标准 JSON

## 23. 当前代码里的未闭环点与风险点

### 23.1 `system_command` 主入口未接上

现状：

1. `websocket_server` 有 `/app/system_command` 发布器。
2. 也有 `handle_system_command()`。
3. 但 `handle_business_command()` 没有分发到这里。

影响：

1. 若按“支持系统命令”理解去接，会发现当前主入口还无法从 APP 发过去。

### 23.2 表情控制缺少执行完成回执

现状：

1. APP 可以发 `facial_control`。
2. ROS 可以下发串口。
3. 但没有独立的“表情完成事件”回流。

影响：

1. 若要做“表情执行完成后再继续下一步”的业务，需要补状态回执链路。

### 23.3 `/navigation/sequences` 目前只看到订阅侧

现状：

1. `websocket_server` 订阅了 `/navigation/sequences`。
2. 但当前仓库里主链路没有明显看到谁在稳定发布它。

影响：

1. 若依赖“导航序列更新事件”，需要先确认运行时是否真的有发布者。

### 23.4 WebSocket 服务端维护了一份客户端订阅镜像

现状：

1. `websocket_server` 本地维护 `client_subscriptions`
2. `data_integration` 也维护一份 `SubscriptionManager`

影响：

1. 二开时如果改订阅逻辑，要注意两边镜像一致性。

## 24. 可直接采用的分层结论

理解“内部消息怎么处理”时，可以直接把系统看成 5 层：

1. APP接入层
   - `websocket_server`
2. ROS业务编排层
   - `dynamic_waypoints_manager`
   - `navigation_state_manager`
3. ROS数据汇聚层
   - `data_integration_node_recoverable`
4. 机器人本体桥接层
   - `websocket_client`
   - `message_bridge`
5. 真实执行端
   - Nav2 Action Server
   - 机器人本体 WebSocket 服务
   - 仿生头串口控制板

二开时通常不需要一次改全链路，而是先判断需求落在哪一层：

1. 改 APP 协议：改第 1 层
2. 改路点与导航业务：改第 2 层
3. 改状态模型和推送：改第 3 层
4. 改底层机器人协议：改第 4 层
5. 改真正导航器或硬件执行：改第 5 层

## 25. 实际运行与启动链路

为了说明这些节点在部署时如何组成系统，这里补上启动视角。

### 25.1 WebSocket相关节点组

`src/humanoid_websocket/launch/websocket_server.launch.py` 会一起启动：

1. `websocket_server`
2. `data_integration_node_recoverable`
3. `websocket_client`
4. `message_bridge`

这意味着在当前设计里，APP 网关层、机器人本体桥接层、数据整合层是一起部署的一组服务。

### 25.2 导航业务层节点组

`src/humanoid_navigation/launch/navigation_fusion.launch.py` 会启动：

1. `dynamic_waypoints_manager`
2. `navigation_state_manager`

这说明路点管理器和导航状态管理器在运行时通常是成对出现的。

### 25.3 表情硬件层节点组

`src/humanoid_locomotion/launch/locomotion.launch.py` 会启动：

1. `facial_driver`

所以表情驱动是单独一组，可独立替换或单独停启。

### 25.4 实机总入口

`src/humanoid_bringup/launch/robot_real.launch.py` 会按阶段启动：

1. 基础描述/显示层
2. 导航栈
3. APP 层
   - `navigation_fusion_sc.launch.py` 或同类导航业务入口
   - `websocket_server.launch.py`
   - `locomotion.launch.py`
4. RViz

从运行结构上可以把它理解为：

1. 导航核心
2. APP通信层
3. 底层动作/表情层

最终都会在同一套 bringup 入口里被拉起。

### 25.5 运行时依赖关系

从启动关系和代码依赖看，建议优先把这几类依赖关系理解清楚：

1. `websocket_server` 必须依赖 `data_integration`，否则 request/subscription/push 不完整。
2. `navigation_state_manager` 必须依赖 `dynamic_waypoints_manager` 提供点位与导航请求入口。
3. `data_integration` 依赖：
   - `navigation_state_manager` 提供 `/navigation/status`
   - `websocket_client`/`message_bridge` 提供系统状态、动作结果、定位等数据
4. `facial_driver` 不依赖导航层，只依赖 `/robot/facial_raw_cmd`

## 26. APP字段到ROS内部映射

这一节从“APP 发的字段最后落到哪里”这个问题来写。

### 26.1 `waypoint_management` 字段映射

| APP消息位置 | ROS内部落点 | 处理函数 | 说明 |
| --- | --- | --- | --- |
| `message_type=command` | `websocket_server.handle_business_command()` | `handle_business_command` | 识别业务命令 |
| `data_type=waypoint_management` | 路由到 `/app/waypoint_command` | `route_to_waypoint_manager` | 进入点位管理链路 |
| `data.command_type` | `command_type` | `dynamic_waypoints_manager.app_waypoint_callback` | 决定增删改查分支 |
| `data.waypoint_data.id` | 点位 ID | `handle_set_waypoint` / `handle_update_waypoint` | 点位主键 |
| `data.waypoint_data.type` | `WaypointType(...)` | `handle_set_waypoint` | 点位类型 |
| `data.waypoint_data.position` | `WaypointData.position` | `handle_set_waypoint` | 三维坐标 |
| `data.waypoint_data.orientation` | `WaypointData.orientation` | `handle_set_waypoint` | 四元数 |
| `data.waypoint_data.properties` | `WaypointData.properties` | `normalize_waypoint_speed_properties` | 扩展属性 |

### 26.2 `navigation_control` 字段映射

| APP消息位置 | ROS内部落点 | 处理函数 | 说明 |
| --- | --- | --- | --- |
| `data_type=navigation_control` | `/app/navigation_command` | `websocket_server.route_to_waypoint_manager` | 导航控制入口 |
| `data.command_type` | `command_data.command_type` | `navigation_state_manager.handle_navigation_command` | 导航命令分发键 |
| `data.waypoint_id` | `command_data.waypoint_id` | `handle_start_single_navigation` | 单点导航目标 |
| `data.waypoint_ids` | `command_data.waypoint_ids` | `handle_start_multi_point_navigation` | 多点导航目标序列 |
| `data.exhibition_ids` | `command_data.exhibition_ids` | 当前代码里预留传递 | 展台类扩展字段 |
| `data.pause_parameters` | `command_data.pause_parameters` | `handle_pause_navigation` | 暂停参数 |
| `data.stop_parameters` | `command_data.stop_parameters` | `handle_stop_navigation` | 停止参数 |

### 26.3 `robot_control` 字段映射

| APP消息位置 | ROS内部落点 | 处理函数 | 说明 |
| --- | --- | --- | --- |
| `data_type=robot_control` | `/app/robot_control` | `websocket_server.handle_robot_control` | 动作控制入口 |
| `data.action` | `command_type` | `websocket_client.robot_control_callback` | 当前主要支持 `execute_gesture` |
| `data.parameters.gesture_id` | `motion_name` | `_run_motion_task` / `execute_upper_body_motion` | 动作名 |
| `data.parameters` 其它字段 | `parameters` 原样透传 | 当前多数未消费 | 可扩展自定义动作参数 |

### 26.4 `facial_control` 字段映射

| APP消息位置 | ROS内部落点 | 处理函数 | 说明 |
| --- | --- | --- | --- |
| `data_type=facial_control` | `/robot/facial_raw_cmd` | `websocket_server.handle_facial_control` | 表情控制入口 |
| `data.action` | `String.data` | `facial_driver.callback` | 表情动作名或原始指令 |

### 26.5 `initial_pose` 字段映射

| APP消息位置 | ROS内部落点 | 处理函数 | 说明 |
| --- | --- | --- | --- |
| `data.x` | `/initialpose.pose.pose.position.x` | `handle_initial_pose` | 初始位姿 x |
| `data.y` | `/initialpose.pose.pose.position.y` | `handle_initial_pose` | 初始位姿 y |
| `data.yaw` | 四元数 `z/w` | `handle_initial_pose` | yaw 转四元数 |
| `data.frame_id` | `/initialpose.header.frame_id` | `handle_initial_pose` | 坐标系 |

### 26.6 `request` 数据类型映射

| APP `data_type` | `data_integration` 数据源 | 主要来源话题/配置 |
| --- | --- | --- |
| `robot_pose` | `data_storage['robot_pose']` | `/robot_realpose` |
| `robot_speed` | `data_storage['robot_speed']` | `/robot_realpose` 差分、`/odom` fallback |
| `navigation_status` | `data_storage['navigation_status']` | `/navigation/status` |
| `navigation_path` | `data_storage['navigation_path']` | `/plan` |
| `system_status` | `data_storage['system_status']` | `/robot_status_processed` |
| `gesture_list` | `data_storage['gesture_list']` | 本地 `gestures.yaml` |
| `facial_gesture_list` | `data_storage['facial_gesture_list']` | 本地 `facial_gestures.yaml` |

### 26.7 事件回推字段映射

| ROS内部来源 | APP看到的 `data_type` | 说明 |
| --- | --- | --- |
| `/navigation/acknowledgments` | `navigation_command_result` | 命令 ACK |
| `/navigation/status` 离散事件 | `navigation_status` | 真实业务事件 |
| `/robot/action_result` | `action_result` | 动作完成结果 |
| 异常聚合器 | `system_exception` | 异常/告警弹窗 |

## 27. 典型二开接入模式

二开通常不是重写整个系统，而是挂接在现有总线上。下面按接入模式整理。

### 27.1 只读监听模式

适合：

1. 做监控面板
2. 做审计日志
3. 做旁路数据采集

推荐监听点：

1. `/navigation/status`
2. `/navigation/acknowledgments`
3. `/navigation/waypoints_data`
4. `/robot/action_result`
5. `/robot_status_processed`
6. `/integration/push_messages`

优点：

1. 对现有链路零侵入
2. 兼容性最好

### 27.2 旁路增强模式

适合：

1. 新增状态聚合
2. 新增异常检测
3. 新增业务事件

推荐方式：

1. 额外写一个节点订阅现有总线
2. 输出新的 `/xxx/status` 或 `/xxx/events`
3. 再在 `data_integration` 里接入新的数据类型推给 APP

典型例子：

1. 订阅 `/navigation/status` 和 `/robot_status_processed`
2. 推导出“导航可执行评分”
3. 发布 `/navigation/execution_score`
4. 让 `data_integration` 再转成 APP 数据

### 27.3 替换执行器模式

适合：

1. 需要保留 APP 协议和上层业务
2. 但替换底层执行逻辑

典型做法：

1. 保留 `websocket_server`
2. 保留 `dynamic_waypoints_manager`
3. 替换 `navigation_state_manager`
4. 或保留 `navigation_state_manager` 但替换其对 Nav2 的调用方式

要求：

1. 新执行器最好继续发布 `/navigation/acknowledgments`
2. 继续发布 `/navigation/status`
3. 这样 APP 层和数据整合层不用改

### 27.4 替换APP网关模式

适合：

1. 已有自己的后端网关
2. 不想让 APP 直接连 ROS 侧 WebSocket

做法：

1. 保留 ROS 内部 `/app/*`、`/navigation/*`、`/integration/*` 这些约定
2. 自己的后端网关模拟 `websocket_server` 的角色
3. 按现有协议桥接到 ROS

这样可以把 `websocket_server` 替掉，但尽量不动业务节点。

## 28. 新增命令时的推荐落点

### 28.1 新增导航相关命令

推荐改：

1. `websocket_server.handle_business_command()`
2. `websocket_server.route_to_waypoint_manager()`
3. `dynamic_waypoints_manager.send_navigation_request()`
4. `navigation_state_manager.handle_navigation_command()`

原则：

1. APP 入口层只做协议解析和透传
2. 真正业务逻辑尽量落在 `navigation_state_manager`

### 28.2 新增纯系统控制命令

如果命令和导航无关，比如：

1. 切地图
2. 开始录包
3. 停止录包
4. 重启某模块

更适合：

1. 补齐 `system_command` 的入口分发
2. 新增独立的 `system_manager` 节点订阅 `/app/system_command`

不要塞进 `navigation_state_manager`。

### 28.3 新增机器人本体控制命令

如果是：

1. 新的动作协议
2. 新的步态模式
3. 新的遥控/机身控制

推荐改：

1. `websocket_client.robot_control_callback()`

如果是高速连续控制，还要同时考虑：

1. `cmd_vel_callback()`
2. `walk_command_loop()`

## 29. 新增回推事件时的推荐落点

### 29.1 导航业务事件

推荐由 `navigation_state_manager` 发布：

1. `/navigation/status`
2. `event_type=xxx`

然后：

1. 把 `event_type` 加进 `data_integration.should_push_navigation_status_immediately()`

这样 APP 才能及时收到。

### 29.2 动作业务事件

推荐由 `websocket_client` 发布：

1. `/robot/action_result`

再由 `data_integration.action_result_callback()` 统一转发。

### 29.3 表情完成事件

这是当前最值得优先补齐的一条链。

建议做法：

1. `facial_driver` 执行完某个动作后，新增发布 `/robot/facial_action_result`
2. `data_integration` 订阅这个话题
3. 转成新的 `push facial_action_result`
4. APP 就能获得“表情完成”闭环

## 30. 二开时的兼容性约束

如需尽量少改前端，建议保持以下兼容：

1. `websocket_server` 外层协议结构不变
2. `/navigation/acknowledgments` 的 `ack_type/status/message` 基本字段不变
3. `/navigation/status` 的 `event_type/current_state/navigation_mode/sequence_id` 尽量保留
4. `action_result` 的 `status/result/message` 尽量保留
5. `system_status` 继续保持 `battery_level/signal_quality/robot_status/system_health/operational_status`

这样即使替换了内部实现，也能最大程度兼容现有 APP。

## 31. 实施建议

正式接手二开时，建议按下面顺序理解和改造：

1. 先只读跑通
   - 先订阅 `/navigation/status`、`/navigation/acknowledgments`、`/robot_status_processed`
   - 看懂现网数据形态
2. 再补协议映射
   - 对照本节字段表，把 APP 字段和 ROS 字段一一对上
3. 然后决定改哪层
   - 改协议层、业务层、执行层还是数据整合层
4. 最后再做闭环增强
   - `system_command`
   - 表情完成事件
   - 新业务事件即时推送

如果目标只是“在现有系统上加功能”，最稳的策略通常是：

1. 不改 `websocket_server` 外层协议
2. 不删现有 `/navigation/*`、`/integration/*` 总线
3. 通过新增节点旁路增强
4. 最后再决定是否替换核心执行器
