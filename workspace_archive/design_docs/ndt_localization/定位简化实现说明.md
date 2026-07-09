# 定位架构简化：去 Fusion 节点 — 修改文档

> 修改日期: 2026-05-27
> 基准 commit: `f73993e` (fusion版本, 归档)
> 修改后 commit: (待提交)

## 1. 修改动机

Fusion 节点 (`localization_odom_fusion.py`, 1992行) 是最大的 bug 来源:
- 14 次 commit 持续修补, 复杂度失控
- 5 态状态机 + 10+ 参数, 与 nav_state_manager 状态机互相耦合
- Fusion 和 NDT 同时发布 map→odom TF, 竞态覆盖 → prior 污染
- Fusion HEALTHY 时崩溃, nav_state_manager 无感知 (最近一次测试的根因)
- 两套 recovery 路径 (fusion LOST vs nav 失败) 逻辑重复

核心洞察: **Fusion 做的事, NDT 和 nav_state_manager 自己就能做**。

## 2. 架构变化

### 修改前 (3 节点)

```
NDT → (map→odom TF + ndt_status) → Fusion (5态状态机) → fusion_status → nav_state_manager
                                                         → recovery_request → HDL
```

### 修改后 (2 节点)

```
NDT → (map→odom TF + ndt_status) → nav_state_manager (直接监听)
                                  → nav_state_manager → recovery_request → HDL
```

## 3. 修改文件清单

| 文件 | 操作 | 改动量 |
|---|---|---|
| `src/humanoid_navigation/humanoid_navigation/navigation_state_manager_fusion.py` | 重写 | -120行 删 +100行 新增 |
| `src/humanoid_navigation2/launch/navigation2_fusion_sc_v2.launch.py` | 修改 | -50行 |
| `src/humanoid_navigation2/launch/navigation2_fusion_sc.launch.py` | 修改 | -47行 |
| `src/humanoid_navigation2/launch/navigation2_fusion.launch.py` | 修改 | -47行 |
| `src/humanoid_bringup/launch/robot_real.launch.py` | 修改 | -1行(注释) |
| `src/lidar_localization/src/lidar_localization_component.cpp` | 修改 | -70行 |
| `src/lidar_localization/include/lidar_localization/lidar_localization_component.hpp` | 修改 | -12行 |
| `src/humanoid_navigation2/humanoid_navigation2/hdl_bootstrap_to_initialpose.py` | 修改 | -4行(注释) |
| `src/humanoid_navigation2/humanoid_navigation2/localization_odom_fusion.py` | **保留不删** (归档) | 0 |

> `localization_odom_fusion.py` 保留在仓库中但不再被任何 launch 文件引用。如需回退, 切回 `f73993e` 即可。

## 4. 逐文件修改详情

### 4.1 navigation_state_manager_fusion.py

**删除的代码** (约 120 行):
- `_on_fusion_status()` 回调 — 整个 fusion 状态解析 + SET-BASED 判据
- `BLOCKED_STATES` 类变量 — `{"DEGRADED", "TRANSITIONING", "LOST"}`
- `_handle_localization_blocked()` — fusion 不健康时的暂停处理
- `_handle_fusion_recovered()` — fusion 恢复时的 resume 处理
- `_check_fusion_status_timeout()` — 5s 超时检测
- `_get_fusion_state()` — 辅助函数
- `/localization/fusion_status` subscription
- `latest_fusion_state` / `last_fusion_status_time` 成员变量
- `fusion_mode` launch argument

**新增的代码** (约 100 行):
- `_on_ndt_status_direct()` — 直接订阅 `/localization/ndt_status`, 解析 JSON, 三层检测:
  - pose_jump 检测: `reason ∈ {pose_jump_candidate, confirmed_pose_jump}` 连续 2 帧 → PAUSE
  - 健康恢复检测: `state=accepted, converged=True, fitness<0.3, reason=ok` 连续 3 帧 → RESUME
  - inlier=0 虚假健康检测: `inlier=0, fitness<0.01, state=accepted` 持续 30s → recovery
- `_check_ndt_status_timeout()` — 3s 无 ndt_status 更新 → PAUSE + recovery
- `_handle_ndt_degraded(reason)` — 统一暂停入口: PAUSE + zero cmd + cancel Nav2 + 发 recovery
- `_handle_ndt_recovered()` — 统一恢复入口: resume 被中断的导航

**参数**:
- `NDT_DEGRADED_REASONS` = `{"pose_jump_candidate", "confirmed_pose_jump"}`
- `NDT_HEALTHY_CONSECUTIVE_FRAMES` = 3
- `INLIER_ZERO_LOST_SEC` = 30.0
- `INLIER_ZERO_FITNESS_CEILING` = 0.01

**替换的引用** (8 处):
- `self.latest_fusion_state != "HEALTHY"` → `not self._is_ndt_healthy`
- `self.latest_fusion_state` → `"NDT异常"` / `"NDT不健康"` (日志消息)
- `[FUSION]` → `[NDT]` (日志前缀)

### 4.2 navigation2_fusion_sc_v2.launch.py

删除:
1. Fusion 节点定义 (lines 565-609): `localization_odom_fusion_node = nav2_python_node(...)` 及其全部 25 个参数
2. `DeclareLaunchArgument('fusion_mode', ...)` 
3. `TimerAction(period=8.0, actions=[localization_odom_fusion_node])`
4. NDT 参数: `fusion_status_timeout_sec`, `allow_ndt_tf_when_fusion_timeout`

保留:
- 所有 HDL/SC/NDT/Nav2 节点定义不变
- `nav2_python_node` 辅助函数 (hdl_bootstrap, scancontext_to_initialpose 仍使用)

### 4.3 navigation2_fusion_sc.launch.py & navigation2_fusion.launch.py

同样删除 fusion 节点定义和维护性注释块。

### 4.4 robot_real.launch.py

仅更新注释: `NDT+Fusion+SC+HDL` → `NDT+SC+HDL (已去 fusion)`

### 4.5 NDT C++ (lidar_localization_component)

删除:
- `fusion_status_timeout_sec` 参数声明 + 读取
- `allow_ndt_tf_when_fusion_timeout` 参数声明 + 读取
- `/localization/fusion_status` subscription 创建
- `fusionStatusCallback()` 函数 (~30行)
- `shouldSuppressTF()` 函数 (~20行)
- .hpp 中 6 个 fusion 相关成员变量 + 2 个函数声明

简化:
- `shouldSuppressTF()` 调用点改为 `false` / `true` (始终允许 TF 发布)
- NDT 现在完全自主发布 TF, 不再等待 fusion 授权

### 4.6 hdl_bootstrap_to_initialpose.py

仅更新注释 (P0-3 逻辑不变, 去掉 fusion Format B 引用)。

## 5. 简化后的流程

### 5.1 冷启动

```
1. 系统启动, NDT 等待 /initialpose
2. HDL/SC bootstrap → 全局定位 → /initialpose → NDT 收到
3. NDT initialpose_relax (4s) → 收敛 → 发布 map→odom TF + ndt_status
4. nav_state_manager 检测: ndt_status reason="ok", TF 可用
5. _is_ndt_healthy = True → 允许导航
6. 用户发导航 → EXECUTING
```

### 5.2 导航中 NDT 跳变

```
1. NDT 检测 pose_jump → ndt_status{reason:"pose_jump_candidate"}
2. nav_state_manager _on_ndt_status_direct: 
   reason ∈ NDT_DEGRADED_REASONS → _ndt_pose_jump_count++
   count >= 2 → _handle_ndt_degraded("pose_jump_candidate")
3. _handle_ndt_degraded:
   - PAUSED + LOCALIZATION_RECOVERY
   - begin_localization_stop_hold() → 30Hz zero cmd_vel
   - cancel_navigation()
   - send_acknowledgment("navigation_auto_paused")
   - request_navigation_context_recovery_for_localization()
4. NDT 侧: republish_last_good_tf (保持 TF 树存活)
5. HDL 收到 recovery 请求 → 全局搜索 → /initialpose
6. NDT 收到 /initialpose → initialpose_relax → 恢复 → 发布新 TF
7. nav_state_manager: ndt_status reason="ok" 连续 3 帧 → _handle_ndt_recovered
8. try_resume_after_localization_recovery() → EXECUTING
```

### 5.3 inlier=0 虚假健康检测

```
1. NDT 长期报告: inlier=0, fitness<0.01, state="accepted", converged=True
2. nav_state_manager 累积 inlier_zero_elapsed
3. elapsed > 30s → _handle_ndt_degraded("inlier_zero_false_healthy")
4. 触发 recovery 流程
```

### 5.4 NDT status 超时

```
1. ndt_status 超过 3s 无更新 (NDT 崩溃/挂起)
2. _check_ndt_status_timeout → _is_ndt_healthy = False
3. 如果正在导航 → _handle_ndt_degraded("ndt_status_timeout")
4. 等待 NDT 恢复或 recovery
```

## 6. 风险与缓解

### 6.1 设计风险

| 风险 | 缓解 |
|---|---|
| NDT pose_jump 阈值 (0.4m/0.3rad) 可能过于敏感 | NDT 参数可通过 launch 调整, 默认值为实测经验值 |
| inlier=0 检测需 30s 才触发 — 可能太慢 | 30s 是安全值, 可根据实机测试调短 |
| nav_state_manager 新增 ~100 行, 职责膨胀 | 远小于 fusion 的 1992 行, 且逻辑清晰 |
| 失去 TRANSITIONING 平滑过渡 | robot 静止时直接切换 TF, 不需要平滑 |
| 失去 fusion 锁定期 (防止假恢复) | NDT 自己的 initialpose_relax (4s) 提供等效保护 |
| hdl_bootstrap P0-3 prior 预占逻辑依赖 `navigation_context_segment` source | 该 source 名称在 nav_state_manager 的 recovery 请求中保留 |

### 6.2 当前代码审查发现的阻断项

以下问题会影响“异常触发 → 暂停 → 发 prior → 重定位 → 验收 → 恢复导航”链路是否能顺利跑通。修复前不建议直接实机长跑。

| # | 问题 | 影响 | 建议修复 |
|---|---|---|---|
| R1 | NDT 真正拒绝跳变时发布 `state="rejected", reason="pose_jump"`；但 nav_state_manager 只监听 `pose_jump_candidate`/`confirmed_pose_jump` | 大跳变被 NDT 拒绝后, nav_state_manager 可能不暂停、不发 recovery | `NDT_DEGRADED_REASONS` 加入 `pose_jump`; `confirmed_pose_jump` 是已接受的自恢复结果, 不应触发 degraded |
| R2 | nav_state_manager 读取 `inlier_fraction`, 但 NDT status JSON 当前没有发布该字段 | `inlier=0` 检测实际无可靠输入; 默认 0 可能把所有低 fitness accepted 帧误判为 inlier=0 | NDT 明确发布 `inlier_fraction` 或 `corr_count`; 否则先禁用 inlier=0 判据 |
| R3 | 方案要求 last_good TF, 但 v1/v2 launch 覆盖为 `republish_last_good_tf_on_failure: False` | pose_jump/rejected 后 TF 可能快速断档, 与“冻结 map->odom 作为恢复先验”的设计不一致 | launch 与方案统一: 若要冻结 prior, 设为 `true` 并设置合理 `max_last_good_tf_age_sec`; 若不冻结, 文档和恢复逻辑要改成完全依赖 waypoint prior |
| R4 | `_handle_ndt_recovered()` 没有设置 `localization_resume_pending=True` 和 `localization_recovered_at` | NDT 自己恢复健康时, 仅靠连续健康帧可能不会 resume 导航 | 在 `_handle_ndt_recovered()` 内补齐恢复状态, 或统一等待 HDL/SC 发布 `localization_recovered` |
| R5 | 定位不健康时缓存的新导航请求会被 `try_execute_pending_navigation()` 定时器反复取出再塞回 | 可能反复发 pending ack/log, 造成 APP 状态抖动 | pending 执行前增加 `_is_ndt_healthy` 检查; 定位未恢复时保持 pending, 不重新 dispatch |
| R6 | payload 带 `allow_full_global_fallback: True`, 但 hdl_bootstrap 未消费该字段; prior 多次失败后直接 `fail_current_recovery()` | “prior 定位不上就 full global, 一直到恢复”的流程没有闭环 | hdl_bootstrap 在 prior 尝试失败/达到上限后切换 `use_prior=False` 并调用 full global; 或消费 payload 的 fallback 标志 |
| R7 | `navigation2.launch.py` 和 `navigation_stack.launch.py` 仍启动 `localization_odom_fusion` | 如果误用旧入口, 会重新出现双 map->odom TF publisher | 清理旧 launch 或显式标记废弃; 所有入口都应保证只有 NDT 发布 map->odom |

### 6.3 关键时序核对

| 阶段 | 期望时序 | 当前风险 |
|---|---|---|
| NDT 跳变触发 | NDT 发布 `pose_jump`/`pose_jump_candidate` → nav_state_manager 连续帧确认 → PAUSE + cancel Nav2 + zero cmd | `reason="pose_jump"` 未被监听会漏触发; `confirmed_pose_jump` 被当异常会误触发 |
| 冻结 TF / prior 生成 | NDT 拒帧后保留 last_good map->odom; nav_state_manager 用当前路点上下文生成 prior | launch 当前关闭 last_good TF; prior 实现主要是 previous-current 线段投影, next waypoint 只进入 context |
| HDL/SC 重定位 | recovery request 带 prior → HDL/SC 带 prior 搜索 → 失败后 full global | HDL helper 目前不消费 `allow_full_global_fallback`, prior 失败后可能进入失败冷却而不是 full global |
| 恢复验收 | `/initialpose` 后 NDT initialpose_relax → NDT accepted/stable + TF fresh → 发布 `localization_recovered` → nav_state_manager resume | NDT 自恢复路径缺少 `localization_resume_pending`; HDL 路径依赖 `/pcl_pose` 和 map->odom TF fresh, 这个逻辑存在但需实机验证时间阈值 |
| 新导航请求 | 定位不健康时 pending; 恢复后延迟执行 | pending 定时器当前可能在定位未恢复时重复 dispatch |

### 6.4 修复优先级

1. 先修 R1/R2/R3/R4: 这四项直接决定异常后能否触发、能否保持 TF/prior、能否恢复导航。
2. 再修 R6: 保证 prior 失败后 full global fallback, 否则“直到恢复为止”不成立。
3. 最后清理 R5/R7: 避免状态抖动和误用旧 launch。

## 7. 构建验证

```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
  --packages-select humanoid_navigation humanoid_navigation2 lidar_localization_ros2
```
结果: ✅ 3 packages 构建成功 (仅预存 warning)
