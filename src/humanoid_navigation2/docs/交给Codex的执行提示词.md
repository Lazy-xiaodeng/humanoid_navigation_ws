# 交给 Codex 的执行提示词

下面这段可以直接复制给另一台机器上的 Codex 使用。

```text
请基于当前仓库实现“整轮路线任务 + 任意跳步 + task/transit 辅助点 + 播报协同”功能。

先不要直接写代码，先阅读并理解以下文档：
1. src/humanoid_navigation2/docs/跳步与辅助点需求确认说明.md
2. src/humanoid_navigation2/docs/ROS侧路线任务_任意跳步_播报协同实现方案.md
3. src/humanoid_navigation2/docs/APP侧路线任务_任意跳步_播报协同改造方案.md
4. src/humanoid_navigation2/docs/路线任务_任意跳步_播报协同开发排期.md
5. src/humanoid_navigation2/docs/路线任务功能代码改造落地说明.md

先根据代码实际情况确认以下事实，再开始修改：
1. 现场实际运行入口到底是 humanoid_navigation/humanoid_navigation/navigation_state_manager.py
2. 还是 humanoid_navigation/humanoid_navigation/navigation_state_manager_recoverable.py
3. websocket 导航命令当前经过哪些 topic 转发
4. navigation_status 事件当前经过哪些节点推送到 APP

当前最终需求不是旧版“沿 A-B-C-D 继续走，只把中间点降级”的逻辑。
当前最终需求是：
1. APP 一次性下发整轮路线任务
2. 每个点位支持两种角色：
   - task：任务点
   - transit：辅助点/途经点
3. 正常执行时：
   - 相邻任务点之间如果有中间 transit 点，ROS 自动吸收
4. 跳步时：
   - 直接跳到目标 task 点
   - 中间 task 点不执行
   - 当前点与目标点之间按顺序存在的 transit 点自动吸收
5. 到达需要播报的 task 点后：
   - ROS 推送 broadcast_requested 给 APP
   - APP 播报完成后回传 broadcast_finished
   - ROS 再继续执行
6. 跳步目标点完成后：
   - 从该目标 task 点继续沿主任务序列向后执行

关键示例：
1. 当前有 7(task) -> 11(transit) -> 12(transit) -> 15(task)
2. 当用户从 7 跳到 15 时
3. 应执行 7 -> 11 -> 12 -> 15
4. 不是 7 -> 8 -> 9 -> ... -> 15

请优先修改这些 ROS 文件：
1. src/humanoid_navigation/humanoid_navigation/dynamic_waypoints_manager.py
2. src/humanoid_navigation/humanoid_navigation/navigation_state_manager.py
3. 如果实际运行入口是 recoverable 版本，则同步修改：
   - src/humanoid_navigation/humanoid_navigation/navigation_state_manager_recoverable.py
4. src/humanoid_websocket/humanoid_websocket/websocket_server.py
5. src/humanoid_websocket/humanoid_websocket/data_integration_node_recoverable.py

推荐施工顺序：
1. 先打通 websocket -> dynamic_waypoints_manager -> navigation_state_manager 的新字段透传
2. 让状态管理器支持新命令：
   - start_route_task
   - jump_to_waypoint
   - broadcast_finished
3. 先实现 route_task 数据结构和主任务序列
4. 再实现正常执行时自动吸收 transit 点
5. 再实现 jump_to_waypoint
6. 再实现 broadcast_requested / broadcast_finished 闭环
7. 最后再接入 NavigateThroughPoses 做丝滑 through

协议要求：
1. APP -> ROS 支持：
   - start_route_task
   - jump_to_waypoint
   - broadcast_finished
2. ROS -> APP 支持事件：
   - broadcast_requested
   - waypoint_passed
   - jump_updated
   - task_waypoint_completed

实现约束：
1. 尽量最小侵入
2. 不要破坏现有单点导航、多点导航、展台导航能力
3. 不要把 task/transit 直接混同于现有 WaypointType 枚举
4. 第一版建议把新字段放在 waypoint properties 或 route_waypoints JSON 中透传
5. 先保证功能语义正确，再做丝滑体验优化

非常重要的检查项：
1. websocket_server 不能只转发 waypoint_id / waypoint_ids 这些旧字段，必须把 route_waypoints、task_session_id、target_waypoint_id、broadcast_id 等新字段一起透传
2. data_integration_node_recoverable 需要把以下事件加入立即推送白名单：
   - broadcast_requested
   - waypoint_passed
   - jump_updated
   - task_waypoint_completed
3. transit 点不能推进主任务索引
4. 只有 task 点完成后才能推进主任务进度

完成后请至少验证这些场景：
1. 正常路线：10(task) -> 11(transit) -> 12(transit) -> 13(task)
2. 确认 11/12 不播报、不等待
3. 当前 7 跳到 15，确认执行 7 -> 11 -> 12 -> 15
4. 当前 15 跳回 7，确认 7 完成后继续 8
5. 到 task 点后只有收到 broadcast_finished 才继续
6. 播报中跳步时状态机不乱

开始工作时，请先输出：
1. 你确认的实际运行入口文件
2. 你准备修改的文件清单
3. 你的实现步骤

然后再开始改代码。
```
