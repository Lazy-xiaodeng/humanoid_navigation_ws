# 路线任务 ROS 侧接力验证总结

更新时间：2026-06-12 10:39:46

## 1. 接力范围

本轮只处理 Todesk 实施目录：

```text
/home/ubuntu/software/Todesk/Files/humanoid_ws
```

未修改正在运行的主工作区源码：

```text
/home/ubuntu/humanoid_ws
```

## 2. 本轮接力做了什么

1. 读取并核对路线任务函数级清单、代码修改记录、ROS 环境接力验证说明。
2. 验证路线任务状态机、websocket 透传、data integration、launch 入口和文档护栏。
3. 发现运行环境缺少 `/navigate_through_poses`，补齐 6 套 `nav2_params*.yaml` 的 bt_navigator navigator 注册。
4. 补强 `validate_route_task_static.py`，增加 through navigator 配置检查。
5. 发现 Todesk 运行文件仍引用 `/home/ubuntu/humanoid_ws`，完成路径隔离修复。
6. 再次补强静态脚本，核心运行文件如果引用主工作区路径则直接失败。
7. 重新构建 Todesk 相关包并验证安装产物。
8. 用命名空间隔离方式临时启动 `bt_navigator`，证明 Todesk 参数能注册 `NavigateThroughPoses` action。

## 3. 已通过的验证

1. 路线任务静态校验通过。
2. Python 语法检查通过。
3. XML/UTF-8 读取检查通过。
4. 6 个 Nav2 YAML 解析通过。
5. `colcon build --packages-select humanoid_navigation humanoid_websocket humanoid_navigation2 humanoid_bringup` 通过。
6. Todesk install 下 `humanoid_navigation / humanoid_navigation2 / humanoid_websocket / humanoid_bringup` package prefix 正确。
7. `ros2 launch ... --show-args` 解析通过，并且默认参数路径指向 Todesk install。
8. Todesk install 产物中没有 `/home/ubuntu/humanoid_ws` 路径残留。
9. 临时 `/route_task_check/bt_navigator` configure 后注册：

```text
/route_task_check/navigate_through_poses
/route_task_check/navigate_to_pose
```

10. 临时 through action 类型确认：

```text
nav2_msgs/action/NavigateThroughPoses
```

## 4. 当前结论

ROS 侧源码、配置、构建产物和 Nav2 through action 注册能力已经落实到可以进入完整整机启动和 APP 联调的程度。

当前不能声称已经完成“实机端到端验收”，因为本轮没有启动完整 Nav2 栈，也没有让 APP 真正下发 `start_route_task / jump_to_waypoint / broadcast_finished` 做路线执行。

## 4.1 阶段收口口径

截至 2026-06-12，本功能的 ROS 侧模拟验证已经进入阶段收口状态。后续不建议继续基于“理论风险”无限追加协议护栏或边界模拟，否则会带来过度工程和源码复杂度上升。

当前 ROS 侧可以冻结的范围：

1. route task 基础状态机语义。
2. `NavigateThroughPoses` through 段执行入口和行为树配置。
3. 任意正向/反向跳步。
4. 跳步区间 transit 吸收。
5. 跳步完成后按目标 task 的后继继续主路线。
6. task 播报等待、播报完成推进。
7. 跳步打断播报。
8. 关键协议字段归一化和错误 ack。
9. WebSocket route task 字段透传的基础口径。

后续只在以下情况继续改 ROS 源码：

1. APP 真链路联调暴露协议字段不一致。
2. 完整 Nav2 栈实机/仿真暴露 through goal 执行失败。
3. 真实路线测试发现跳步、回跳、辅助点吸收或播报闭环与需求不一致。
4. 过门/辅助点安全验证失败，需要新增路径或控制约束。
5. 日志出现明确异常、失败事件或状态机卡死。

不再建议继续处理的内容：

1. 没有真实复现依据的极端坏包组合。
2. 不影响当前需求闭环的协议美化。
3. 只为了让测试覆盖率更细而增加的模拟分支。
4. 和 APP/实机验收无关的字段冗余保护。

## 5. 下一步实机验收命令

启动 Todesk install 对应的完整导航栈后，先执行：

```bash
ros2 action list | grep navigate_through_poses
ros2 action info /navigate_through_poses
ros2 action type /navigate_through_poses
```

期望：

```text
/navigate_through_poses
nav2_msgs/action/NavigateThroughPoses
```

然后再做 APP 全链路：

1. `start_route_task`
2. 到 task 后 `broadcast_requested`
3. APP 回 `broadcast_finished`
4. 正向 jump，例如 `7 -> 15`
5. 反向 jump，例如 `15 -> 3`
6. transit 只发 `waypoint_passed`，不播报、不完成 task
7. 失败路径返回 `navigation_failed(route_task=true)` 和 `failure_code`

## 6. 风险提示

1. Todesk 构建时仍提示 underlay 覆盖 `/home/ubuntu/humanoid_ws/install` 中的部分包。实际运行前应保证 source 顺序为 Todesk install 优先。
2. 临时 bt_navigator 验证只证明 action 注册，不证明完整导航执行成功。
3. 真正“完美运行”仍需要完整 Nav2、定位、地图、APP websocket 联调共同证明。


## 7. 再审查补充：through 专用行为树

更新时间：2026-06-12 10:48:56

再审查发现一个关键运行时风险：`NavigateThroughPoses` 不能复用单点 `navigate_xy_then_yaw.xml`，因为单点 BT 使用 `{goal}` 和 `ComputePathToPose`，而 through navigator 需要 `{goals}` 和 `ComputePathThroughPoses`。

已新增并安装：

```text
navigate_through_poses_no_backup.xml
```

已更新三个 Nav2 launch，新增 `through_bt_xml_file`，并让 `default_nav_through_poses_bt_xml` 指向 through 专用 BT。

该修复已通过静态校验、XML 解析、构建和临时 `bt_navigator` action 注册验证。

## 8. 再审查补充：route task 状态机语义模拟回归

更新时间：2026-06-12 15:05:00

在 through action 注册和配置收口之后，又继续对 ROS 侧 `navigation_state_manager` 做了轻量动态模拟。

本轮模拟只启动 Todesk 目录下的 `navigation_state_manager`，由脚本模拟 APP 命令、TF、odom、机器人 ready 和定位 healthy。该模拟用于验证 route task 状态机语义，不替代真实 Nav2 路径跟随、真实地图、真实 APP websocket 联调。

新增模拟脚本：

```text
src/humanoid_navigation2/scripts/simulate_route_task_semantics.py
```

该脚本已通过 `humanoid_navigation2/setup.py` 安装到：

```text
install/humanoid_navigation2/share/humanoid_navigation2/scripts/simulate_route_task_semantics.py
```

源码路径运行方式：

```bash
source install/setup.bash
ros2 run humanoid_navigation navigation_state_manager --ros-args \
  -p require_walk_mode_for_navigation:=false \
  -p localization_allow_start_with_last_good_tf:=true \
  -p localization_health_timeout_sec:=30.0

python3 src/humanoid_navigation2/scripts/simulate_route_task_semantics.py --scenario all
```

install 产物运行方式：

```bash
python3 install/humanoid_navigation2/share/humanoid_navigation2/scripts/validate_route_task_static.py
python3 install/humanoid_navigation2/share/humanoid_navigation2/scripts/simulate_route_task_semantics.py --scenario all
```

建议用独立 `ROS_DOMAIN_ID` 运行，避免和现场正在运行的导航图互相干扰。

### 8.1 已覆盖的动态模拟场景

1. `bidirectional_jump`

验证正向跳和反向跳：

```text
B -> G: execution_waypoint_ids = ["E", "F", "G"]
G -> B: execution_waypoint_ids = ["F", "E", "B"]
```

同时验证：

1. 正向跳会跳过中间 task。
2. 正向跳会吸收区间 transit。
3. 反向跳会按反向顺序重新吸收 transit。
4. 跳回后完成的 task 会从 `skipped_task_ids` 移除。
5. 跳回 `B` 后继续按主任务顺序执行 `C -> D -> G -> H`。
6. 最终 `completed_task_ids` 与 `skipped_task_ids` 不重叠。
7. `segment_id` 按段递增，不在快捷完成分支复用同一个段号。

2. `jump_to_no_broadcast_shortcut`

验证等待播报时跳到无需播报且当前位置已到达的 task：

1. 先发布 `jump_updated` 和 `jump_to_waypoint success ack`。
2. 再同步发布 `task_waypoint_completed / route_task_completed`。
3. 被打断播报的原 task 计入 `skipped_task_ids`。
4. 目标 task 不会同时出现在 `skipped_task_ids`。
5. 最终 `task_count = completed_count + skipped_count`。

3. `protocol_errors`

验证 route task 运行期间的错误命令不会污染当前任务：

1. 重复 `start_route_task` 返回 `route_task_already_running`。
2. 跳当前目标返回 `already_current_target`，不触发 `jump_updated`。
3. 错误 `task_session_id` 的 `jump_to_waypoint` 返回 `invalid_task_session`。
4. 错误 `task_session_id` 的 `broadcast_finished` 返回 `invalid_task_session`。
5. 错误 `route_id` 的 `jump_to_waypoint` 返回 `invalid_route_id`。
6. 错误 `route_id` 的 `broadcast_finished` 返回 `invalid_route_id`。
7. 跳到 transit 返回 `target_waypoint_not_task`。
8. 等待播报时 `interrupt_broadcast=false` 返回 `interrupt_broadcast_false_not_supported`。
9. 播报上下文不匹配返回 `broadcast_context_mismatch`。
10. 不支持的播报结果返回 `unsupported_broadcast_result`。
11. 重复 `broadcast_finished` 返回 `duplicate_broadcast_finished`，不误完成下一个点。

4. `start_validation`

验证 route 未运行和 start 坏包不会留下脏状态：

1. route 未运行时 `jump_to_waypoint` 返回 `route_task_not_running`。
2. route 未运行时 `broadcast_finished` 返回 `route_task_not_running`。
3. 缺 `task_session_id` 返回 `missing_task_session_id`。
4. 缺 `route_id` 返回 `missing_route_id`。
5. `route_waypoints` 非数组返回 `invalid_route_waypoints`。
6. waypoint role 非法返回 `invalid_waypoint_role`。
7. 重复 task waypoint ID 返回 `duplicate_waypoint_id`。
8. task/transit 使用相同 waypoint ID 返回 `duplicate_waypoint_id`。
9. `need_broadcast=true` 但 `broadcast_id` 为空返回 `missing_broadcast_id`。
10. 只有 transit、没有 task 返回 `missing_task_waypoints`。
11. waypoint 缺 position 返回 `missing_waypoint_pose`。
12. waypoint orientation 为零长度四元数返回 `missing_waypoint_pose`。
13. 多个坏包之后，合法 `start_route_task` 仍可正常完成，证明未残留脏 route task 状态。

### 8.2 本轮模拟发现并收口的风险点

1. 反向跳步时，如果上一段已经通过过 transit，等待播报后再反向跳会错误去重 transit。

处理结果：只有仍在 through 导航途中再次跳步时才扣除当前段已 passed transit；如果已到达 task 并等待播报，反向跳必须重新吸收区间 transit。

2. 跳回之前被 skipped 的 task 后，最终 summary 可能同时出现 completed 和 skipped。

处理结果：`finalize_task_waypoint_completion()` 中实际完成 task 后，会从 `skipped_task_ids` 移除该 task。

3. 等待播报时被 jump 打断的 task 既不 completed 也不 skipped，导致最终统计不闭合。

处理结果：被打断播报的 task 先计入 skipped；如果后续又跳回并完成，再自动从 skipped 移除。

4. 快捷完成分支没有推进 generation，可能导致 `segment_id` 复用。

处理结果：首点已到达和 active segment 已到达两个快捷分支都推进 generation。

5. 跳到无需播报且当前位置已到达的 task 时，状态机可能同步 reset route task，导致 jump handler 回来后读取 `active_segment=None`。

处理结果：快捷完成 jump 先发布 `jump_updated` 和 `jump_to_waypoint success ack`，再进入同步完成流程。

6. 跳回某个之前 skipped 的目标点时，`jump_updated` 可能同时显示该点为 target 和 skipped。

处理结果：目标点重新成为 active target 后，立即从 `skipped_task_ids` 移除。

7. 节点 Ctrl-C 退出时出现重复 `rclpy.shutdown()` traceback。

处理结果：main 入口改为 node 存在才 destroy，且仅在 `rclpy.ok()` 时 shutdown。当前退出无 Python traceback，只剩 ROS shutdown 后 rosout 写入提示。

8. `jump_to_waypoint / broadcast_finished` 只校验 `task_session_id`，没有校验 `route_id`。

风险说明：如果 APP 重连、缓存或多路线切换时发来同 session 但错误 route 的命令，ROS 侧可能误接受 jump 或播报完成回执，造成路线 UI 与 ROS 执行态不一致。

处理结果：两类命令现在都会先校验 `task_session_id`，再校验 `route_id`；错误 route 返回 `invalid_route_id`，且不会触发 `jump_updated`、不会完成播报、不会推进任务。动态模拟已加入 `err_jump_bad_route / err_broadcast_bad_route` 两个回归场景，源码版和 install 版均通过。

9. `route_waypoints` 未拒绝重复 `waypoint_id`。

风险说明：如果一条路线里出现两个同 ID 点位，`find_route_waypoint_by_id()` 会命中第一个，后续 `source_index`、task 顺序、jump 目标和播报上下文都可能错位。尤其 task 和 transit 同 ID 时，APP 看起来是同一个点，ROS 内部却可能按第一个匹配点执行，问题现场很难排查。

处理结果：`normalize_route_task_waypoints()` 现在维护 `seen_waypoint_ids`，同一条路线中任意重复 ID 都直接拒绝，错误码为 `duplicate_waypoint_id`。动态模拟已加入重复 task ID、task/transit 同 ID 两个坏包，源码版和 install 版均通过。

10. `need_broadcast=true` 时允许空 `broadcast_id`。

风险说明：空播报 ID 会让 `broadcast_requested` 和 `broadcast_finished` 的闭环匹配失去明确业务标识。APP 重连、缓存回执或多个播报资源切换时，空 ID 很容易造成 UI 卡在播报中、重复回执幂等判断不清晰，或者现场无法追踪到底是哪段播报。

处理结果：`normalize_route_task_waypoints()` 现在要求 task 点 `need_broadcast=true` 时必须有非空 `broadcast_id`，否则 `start_route_task` 直接返回 `missing_broadcast_id`。判断在 `broadcast_id` 归一化之后执行，因此 `" broadcast_7 "` 仍会被 trim 成合法 ID，trim 后为空才拒绝。动态模拟已加入 `missing_broadcast_id_start` 坏包，源码版和 install 版均通过。

11. route waypoint orientation 允许零长度四元数。

风险说明：`[0,0,0,0]` 虽然是有限数字数组，但不是合法旋转。若这种姿态进入 `NavigateThroughPoses`，Nav2/TF/控制器侧可能出现姿态解释异常、朝向不可预期，或者错误只在真实规划时才暴露。

处理结果：`normalize_route_task_orientation()` 现在会计算四元数范数，拒绝范数小于等于 `1e-6` 的零长度姿态，并将合法四元数归一化成单位四元数。动态模拟已加入 `zero_orientation_start` 坏包，返回 `missing_waypoint_pose`，源码版和 install 版均通过。

12. 桥接层 `broadcast_finished` 缺字段提示未包含 `route_id`。

风险说明：状态机已经严格要求 `broadcast_finished` 携带正确 `route_id`，否则返回 `invalid_route_id`。如果桥接层 warning 仍只提示 `task_session_id / waypoint_id / broadcast_id`，APP 联调或现场排查时容易误以为播报完成回执不需要带 `route_id`。

处理结果：`dynamic_waypoints_manager.py` 的 `broadcast_finished` 缺字段提示已补齐 `route_id`，并新增静态护栏要求桥接层提示字段包含 `route_id`。桥接层仍不硬拒绝缺字段命令，继续交给状态机统一返回 `navigation_command_result`。

13. APP 运行中命令漏传 `route_id` 的坏包缺少动态回归覆盖。

风险说明：APP 联调时可能只带 `task_session_id`，漏传 `route_id`。状态机虽然会把缺失字段归一化为空字符串并返回 `invalid_route_id`，但如果没有动态模拟覆盖，后续改动可能误把缺失 `route_id` 当成合法命令，导致跳步或播报完成误推进。

处理结果：`protocol_errors` 动态场景新增 `err_jump_missing_route` 和 `err_broadcast_missing_route`，要求两者都返回 `invalid_route_id`，且不能触发 `jump_updated`、不能新增播报请求、不能完成当前 task。源码版和 install 版动态模拟均通过。

14. `navigation_command_result` 会用 active route task 回填缺失的命令 ID。

风险说明：`navigation_command_result` 是 APP 某一次命令的业务 ack，应反映这次命令实际携带的 `task_session_id / route_id`。如果 APP 漏传 `route_id`，但 ROS 回包又把 `route_id` 回填成当前 active route，APP 日志会出现 `route_id=当前路线` 且 `error_code=invalid_route_id` 的矛盾信息，现场容易误判为 ROS 误拒绝。

处理结果：`send_route_task_ack()` 不再使用 `active_route_task` 回填缺失的 `task_session_id / route_id`；普通 route task 业务事件仍由 `publish_route_task_event()` 自动补齐当前 active 上下文。动态模拟新增断言：`err_jump_missing_route` 和 `err_broadcast_missing_route` 的错误 ack 中 `route_id` 必须保持为空字符串。源码版和 install 版动态模拟均通过。

### 8.3 已通过的最新验证命令

```bash
python3 -m py_compile \
  src/humanoid_navigation/humanoid_navigation/dynamic_waypoints_manager.py \
  src/humanoid_navigation/humanoid_navigation/navigation_state_manager.py \
  src/humanoid_navigation2/scripts/validate_route_task_static.py \
  src/humanoid_navigation2/scripts/simulate_route_task_semantics.py

python3 src/humanoid_navigation2/scripts/validate_route_task_static.py

python3 src/humanoid_navigation2/scripts/simulate_route_task_semantics.py --scenario all

python3 install/humanoid_navigation2/share/humanoid_navigation2/scripts/validate_route_task_static.py

colcon build --packages-select humanoid_navigation humanoid_navigation2
```

最新结果：

```text
路线任务静态校验通过：关键 through、协议、入口和事件不变量均存在。
install 路径 validate_route_task_static.py -> 通过
源码路径 simulate_route_task_semantics.py --scenario all -> status: pass
install 路径 simulate_route_task_semantics.py --scenario all -> status: pass
colcon build --packages-select humanoid_navigation humanoid_navigation2 -> 通过
broadcast_finished 桥接层 route_id 缺字段提示静态护栏 -> 通过
err_jump_missing_route / err_broadcast_missing_route -> invalid_route_id
err_jump_missing_route / err_broadcast_missing_route 的 navigation_command_result.route_id -> ""
```

### 8.4 当前剩余风险

当前可模拟的 ROS 状态机语义已经覆盖：

1. 正向/反向任意跳步。
2. transit 吸收和反向重吸收。
3. 播报等待、播报完成、重复播报回执。
4. 跳步打断播报。
5. 无播报快捷完成。
6. 启动坏包和运行中协议错误。
7. 重复 waypoint ID 输入保护。
8. 播报点 `broadcast_id` 必填保护。
9. route waypoint orientation 零长度四元数保护。
10. completed/skipped 统计闭合。
11. broadcast_finished route_id 上下文校验和桥接层提示一致性。
12. 运行中 jump/broadcast 命令漏传 route_id 的拒绝语义。
13. navigation_command_result 不替 APP 回填缺失的 task_session_id / route_id。

仍不能声称“完美实机运行”，因为以下内容还必须由真实环境证明：

1. 完整 Nav2 栈下 `NavigateThroughPoses` 实际行走是否稳定。
2. 真实地图和真实控制器下 transit 过门是否满足安全距离。
3. APP websocket 真链路是否按 `navigation_command_result / jump_updated / broadcast_requested / task_waypoint_completed / route_task_completed` 正确消费。
4. 真实定位抖动、障碍物阻塞、Nav2 goal rejected/canceled/failed 时的恢复表现。
5. Todesk install 与主工作区 underlay 覆盖 warning 在现场 source 顺序下是否完全可控。
