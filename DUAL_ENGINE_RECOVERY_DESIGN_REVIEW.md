# 双引擎 Recovery 方案评审文档

日期：2026-05-27  
评审对象：`/home/ubuntu/humanoid_ws/DUAL_ENGINE_RECOVERY_DESIGN.md`  
背景：基于 test3 导航漂移问题，评审“双引擎全局重定位 + Fusion 停车接管”方案的可行性、风险点、遗漏逻辑和建议修改。

## 1. 总体结论

这套方案的方向是可行的，核心思路比“只用 NDT + HDL 停车全局搜”和“fusion 长时间 odom 接管继续导航”都更合理。

它的正确方向是：

1. NDT 异常后，fusion/rusion 立即冻结或接管 `map->odom`。
2. 导航进入受控停车，不继续扩大漂移。
3. 用冻结 TF + odom 位移推算 recovery prior。
4. ScanContext 和 HDL 协同重定位，避免单一引擎失败。
5. 最终结果必须经过 NDT 连续帧和下游 costmap/TF 验收。
6. 定位稳定后再恢复当前 waypoint 或执行 pending navigation request。

但当前文档还有几个关键风险没有完全收住。最大的风险是：方案里同时存在“fusion 全状态唯一发布 TF”和“NDT HEALTHY 发 TF、fusion DEGRADED/LOST 发 TF”两套架构。这个必须先统一，否则 test3 里的 TF 抢占问题可能以更隐蔽的方式继续存在。

## 2. 阻断级问题

### 2.1 TF ownership 定义冲突

原文冲突点：

- 第 52-56 行：`localization_odom_fusion publishes map->odom (唯一发布者，所有状态)`。
- 第 84 行：`HEALTHY | NDT 发 TF，fusion 不发 | 同左，不变`。
- 第 615 行：`NDT 在 HEALTHY，fusion 在 DEGRADED+`。

这是架构级冲突，不是文档措辞问题。

如果实现时按不同段落理解，会出现两种危险情况：

1. HEALTHY 到 DEGRADED 切换瞬间，NDT 和 fusion 仍可能短时双发 `map->odom`。
2. fusion 状态延迟、掉线或重启时，下游 TF 可能在 NDT 与 fusion 之间跳源。

建议二选一：

方案 A，推荐最终架构：

```text
NDT 永远不发布 map->odom TF。
NDT 只发布 /pcl_pose 和 /localization/ndt_status。
fusion/rusion 是全状态唯一 map->odom 发布者。
```

优点：

- TF ownership 最清楚。
- 下游永远只看 fusion/rusion 输出。
- 不存在 HEALTHY/DEGRADED 切换竞态。
- 后续 recovery 验收、平滑切换、冻结逻辑都集中在一个节点。

缺点：

- 启动链路和 fusion 节点职责更重。
- fusion 出问题会影响整条定位 TF 链，需要 watchdog 和降级策略。

方案 B，过渡架构：

```text
NDT 在 HEALTHY 发布 map->odom。
fusion 只在 DEGRADED/TRANSITIONING/LOST 发布 map->odom。
```

这个方案改动较小，但必须补保护：

1. NDT 订阅 fusion 状态后，非 HEALTHY 必须立即停止 TF。
2. fusion 进入非 HEALTHY 后必须立即发布冻结 TF。
3. 切换窗口内导航必须 blocked，不能继续走。
4. fusion 崩溃时不能直接让系统自动恢复正常导航。

评审建议：如果这是要继续长期维护的系统，应走方案 A。如果只是快速验证 test3 修复，可以用方案 B，但文档必须明确它是过渡方案。

### 2.2 recovery 输出 ownership 也需要唯一裁判

原文第 369-371 行写：

```text
两个引擎都通过发布 /initialpose 的方式输出结果
```

后文又说 HDL 只作为 SC fallback，不独立响应 recovery request。这两处语义冲突。

风险：

```text
SC 发布 initialpose A
HDL 随后发布 initialpose B
NDT 正在 reacquire 时被打断
fusion 看到 /pcl_pose 跳变
系统进入反复 recovery 或假恢复
```

这和 test3 的 TF 抢占本质一样，只是抢占对象从 `map->odom` 变成了 `/initialpose`。

建议硬规则：

```text
全系统只能有一个节点发布最终 /initialpose。
```

推荐新增或明确一个仲裁层：

```text
recovery_arbiter
  subscribes:
    /localization/recovery_requests
    /scancontext/candidates
    /hdl/candidates
    /localization/ndt_status
    /pcl_pose

  publishes:
    /initialpose
    /localization/recovery_status
```

如果短期不新建 `recovery_arbiter`，那就让 `scancontext_to_initialpose` 临时承担 arbiter 职责，但 HDL 不能直接发布 `/initialpose`。

## 3. 高风险问题

### 3.1 DEGRADED 停车触发条件太窄

原文第 120-123 行：

```python
if new_state == "DEGRADED" and old_state == "HEALTHY":
    self._handle_fusion_degraded()
```

这个只覆盖 `HEALTHY -> DEGRADED`。

遗漏场景：

- `INITIALIZING -> DEGRADED`
- `TRANSITIONING -> DEGRADED`
- `HEALTHY -> LOST`
- fusion 重启后状态从未知态直接变非健康态
- navigation manager 启动晚于 fusion，首次收到状态就是 `DEGRADED`

建议改成状态类判断：

```python
HEALTHY_STATES = {"HEALTHY"}
BLOCKED_STATES = {"DEGRADED", "TRANSITIONING", "LOST", "RECOVERY", "RECOVERY_VALIDATING"}

was_healthy = old_state in HEALTHY_STATES
is_blocked = new_state in BLOCKED_STATES

if is_blocked and not self.localization_blocked:
    self._handle_localization_blocked(new_state)
elif new_state in HEALTHY_STATES and self.localization_blocked:
    self._handle_fusion_recovered()
```

原则：只要定位不健康，就不能启动或继续导航。不要只绑定某一个状态跳变。

### 3.2 fusion_status 超时回退到 HEALTHY 有安全风险

原文第 236-245 行建议：

```cpp
if (fusion_status_age > 5.0) {
    fusion_state_ = "HEALTHY";
}
```

这个策略可以避免 NDT 永久闭嘴，但也可能在 fusion 崩溃时把错误 NDT TF 重新放回系统。

危险场景：

```text
NDT 已漂移
fusion DEGRADED/LOST 接管
fusion 崩溃
5 秒后 NDT 自动恢复 TF 发布
下游重新读到错误 map->odom
navigation manager 如果没有同步 blocked，可能恢复导航
```

建议替代策略：

1. NDT 端超时后不要直接视为系统 HEALTHY，而是进入 `FUSION_TIMEOUT`。
2. `FUSION_TIMEOUT` 下允许 NDT 独立模式发布 TF，只能在明确参数允许时启用。
3. navigation manager 收到 fusion 超时或 fusion 状态缺失时，必须保持 localization blocked。

推荐参数：

```yaml
fusion_tf_gate_enabled: true
fusion_status_timeout_sec: 5.0
allow_ndt_tf_when_fusion_timeout: false
```

如果为了兼容旧 launch，需要默认独立运行：

```text
只有在检测到 fusion 曾经发布过非 HEALTHY 状态后，超时才进入 FUSION_TIMEOUT。
从未收到 fusion_status 时，NDT 保持独立模式。
```

### 3.3 TF 链 prior 需要补时间一致性和 fallback 修正

原文第 264-304 行使用：

```python
lookup_transform('map', 'body', Time())
```

方向是对的，比手工坐标转换稳。但这里有两个细节风险。

风险 1：时间不一致。

`Time()` 查最新 TF 时，可能拿到：

```text
较旧的 frozen map->odom
+ 最新的 odom->body
```

通常可接受，但 recovery prior 应该把时间戳和 odom delta 记录清楚，便于判断先验可信度。

建议 recovery request 增加字段：

```json
{
  "prior": {
    "frame_id": "map",
    "stamp": "...",
    "x": 5.8,
    "y": 12.1,
    "radius_m": 3.0,
    "yaw_constrained": false,
    "source": "frozen_tf_chain",
    "odom_displacement_m": 0.42,
    "frozen_age_sec": 2.1,
    "ndt_correction_at_freeze_m": 0.62
  }
}
```

风险 2：fallback 用 `frozen_map_odom.x/y` 不等于机器人位置。

`frozen_map_odom` 是 odom 原点在 map 下的位置，不是 `base/body` 在 map 下的位置。fallback 应使用冻结时保存的 `map_T_body`：

```text
frozen_map_T_body = frozen_map_T_odom * frozen_odom_T_body
```

如果没有保存 `frozen_odom_body`，再退化到大范围全局搜索，而不是把 `frozen_map_odom` 当机器人位置。

建议 fusion 在进入 DEGRADED 时保存：

```text
frozen_map_T_odom
frozen_odom_T_body
frozen_map_T_body
ndt_status_at_freeze
freeze_stamp
```

### 3.4 SC 主、HDL fallback 不足以防长廊误匹配

原文第 379-403 行将 SC 作为主引擎，HDL 仅在 SC 失败后 fallback。

这能解决“SC 搜不到”的问题，但不能解决“SC 搜到了错误高置信候选”的问题。长廊、重复展区、相似墙面结构中，错误候选可能看起来很像成功。

建议改为候选验证模式：

```text
SC 负责快速召回 top-K candidates。
HDL/GICP 负责几何验证 top-K candidates。
arbiter 根据 score margin、多帧一致性、prior 距离、NDT 验收决定是否发布 /initialpose。
```

推荐流程：

```text
1. SC prior search -> top-K
2. 对 top-K 做几何验证
3. 如果 top1 明显优于 top2，且几何验证通过 -> 发布 /initialpose
4. 如果 top1/top2 margin 太小 -> 触发 HDL 扩展搜索
5. 如果 SC 无候选 -> HDL prior search
6. 如果 prior search 全失败 -> full global search
```

这样两个引擎不是简单主备，而是“召回 + 验证 + fallback”。

## 4. 中风险问题

### 4.1 DEGRADED 立即 cancel 可能过于激进

立即停车是对的，但是否立即 cancel Nav2 goal 可以分级。

建议：

```text
soft block:
  进入 DEGRADED 后立即发布 zero cmd / velocity hold
  暂停发送新目标
  短窗口观察 NDT 是否恢复

hard block:
  DEGRADED 持续超过 0.5-1.0s
  或 pose_jump/high_fitness/stale 明确失稳
  或 odom 位移超过阈值
  -> cancel goal + recovery
```

这样可以避免单帧抖动导致频繁 cancel/resume。

如果你的场景更重视安全而不是连续性，也可以 DEGRADED 直接 cancel，但需要配合防抖：

```yaml
degraded_consecutive_frames: 2-3
degraded_min_duration_before_cancel_sec: 0.5
```

### 4.2 锁定期参数偏长

文档中有：

```text
min_degraded_lock_sec: 15
max_degraded_lock_sec: 30
```

既然 DEGRADED 后已经停车，就没必要等 15-30 秒再 recovery。test3 这种情况应该在静止确认后尽快进入 recovery。

建议：

```yaml
runtime_stationary_settle_sec: 1.0
runtime_relocalize_buffer_refill_sec: 1.0-1.5
degraded_to_recovery_timeout_sec: 2.0-3.0
max_degraded_lock_sec: 5.0
```

长时间等待只会让用户感觉机器人卡死，并不能增加重定位成功率。

### 4.3 pending navigation 只能缓存一个请求

原文第 160-166 行使用单变量：

```python
self.pending_navigation_request = ...
```

风险：

- APP 连续下发多个点，前一个被覆盖。
- 用户取消导航，但 pending request 还在，恢复后误执行。
- 当前 waypoint 恢复和 pending request 的优先级没有严格定义。

建议补策略：

```text
如果 localization blocked:
  - start_navigation: 默认只保留最后一个 pending request，覆盖前一个，并通知 APP
  - cancel_navigation: 清空 pending request 和 current interrupted waypoint
  - resume_navigation: blocked 时拒绝，恢复后再允许
```

或者使用队列：

```python
self.pending_navigation_queue = deque(maxlen=3)
```

但对机器人导航来说，通常“只保留最后一个用户意图”更简单。

### 4.4 恢复验收条件不够完整

原文第 420-424 行主要依赖 NDT status：

```text
连续 N 帧 accepted + fitness < threshold
```

这还不够。test3 的风险并不只是 NDT rejected，而是下游 TF 和 costmap 被错误位姿污染。

建议恢复验收至少包括：

```text
NDT:
  consecutive accepted >= N
  fitness < threshold
  correction_translation < threshold
  correction_yaw < threshold
  inlier_fraction 合理，不能长期为 0

Fusion/TF:
  map->odom 单次跳变 < max_recovery_tf_jump
  /pcl_pose 与 recovery candidate 差距 < dynamic gate
  TF tree 查询稳定，无 lookup timeout

Costmap/Nav2:
  robot pose within map bounds
  local/global costmap 不报 robot/sensor out of bounds
  恢复后延迟 0.5-1.0s 再 resume

Navigation:
  当前 waypoint 未被误标记完成
  pending request 未被错误消费
```

### 4.5 验收命令不能只靠 grep `/tf`

原文第 471-482 行用：

```bash
ros2 topic echo /tf | grep "map.*odom"
```

这个只能看到 TF 内容，看不到 publisher，也不容易判断双源竞争。

建议验收方式：

```bash
ros2 topic info /tf -v
```

检查 `/tf` publisher 中是否仍存在 NDT 和 fusion 双源。

还应写一个 bag/offline 检查脚本：

```text
统计 map->odom 发布频率
统计 map->odom 帧间跳变
统计 DEGRADED/LOST 期间 map->odom 是否为冻结值
统计 /cmd_vel 在 localization blocked 后是否归零
统计 /initialpose 是否只有一个节点发布
```

## 5. 推荐修订后的核心流程

推荐最终版本：

```text
NDT
  publishes:
    /pcl_pose
    /localization/ndt_status
  does not publish:
    map->odom

fusion/rusion
  publishes:
    map->odom
    /localization/fusion_status
    /localization/recovery_requests
  owns:
    TF freeze
    odom prior computation
    recovery validation state

navigation_state_manager
  owns:
    localization_blocked
    cancel/zero cmd
    current waypoint preservation
    pending navigation request
    resume after validation

recovery_arbiter
  owns:
    SC + HDL scheduling
    candidate scoring
    only publisher of /initialpose

ScanContext
  role:
    fast candidate retrieval, top-K

HDL/GICP
  role:
    geometry validation and fallback global search
```

推荐事件链：

```text
1. NDT status abnormal or /pcl_pose jump/stale
2. fusion enters DEGRADED
3. fusion freezes map->odom
4. navigation_state_manager enters localization_blocked
5. zero cmd hold; optional short soft-stop window
6. if NDT does not recover quickly -> hard recovery
7. cancel current Nav2 goal
8. wait stationary
9. clear recovery buffer and collect fresh stationary scans
10. fusion computes prior
11. recovery_arbiter triggers SC prior search
12. HDL/GICP validates SC candidates
13. if ambiguous/fail -> HDL expanded/global search
14. arbiter publishes one /initialpose
15. NDT enters reacquire window
16. fusion validates NDT/TF/costmap stability
17. navigation_state_manager resumes interrupted waypoint or pending request
```

## 6. 建议实施顺序

### Phase 0：统一 ownership

必须先定：

```text
map->odom 谁发？
/initialpose 谁发？
navigation blocked 谁管？
```

建议：

```text
map->odom: fusion/rusion only
/initialpose: recovery_arbiter only
navigation blocked: navigation_state_manager only
```

### Phase 1：解决 TF 抢占和停车

改动：

1. NDT 禁止发布或受控关闭 `map->odom` TF。
2. fusion/rusion 在 HEALTHY/DEGRADED/LOST 全状态发布 `map->odom`。
3. navigation_state_manager 收到非 HEALTHY fusion 状态后进入 localization blocked。
4. blocked 后 zero cmd hold，并禁止新 goal 下发到 Nav2。

验收：

```text
DEGRADED/LOST 期间 /tf 没有双源 map->odom。
localization blocked 后 /cmd_vel 在 0.3-0.5s 内归零。
APP 新点位不会绕过 blocked 状态。
```

### Phase 2：实现 prior

改动：

1. fusion 进入 DEGRADED 时保存冻结快照。
2. recovery 前计算 `map_T_body_prior`。
3. recovery request 携带 prior、radius、source、age、odom displacement。

验收：

```text
prior 与 bag 中真实机器人位置误差在预期半径内。
TF 查询失败时不会错误使用 map_T_odom 原点当机器人位置。
```

### Phase 3：双引擎候选仲裁

改动：

1. SC 输出 top-K candidates。
2. HDL/GICP 验证 candidates。
3. 只有 arbiter 发布 `/initialpose`。
4. HDL fallback 不直接响应 `/localization/recovery_requests`。

验收：

```text
一次 recovery 只有一个 /initialpose publisher。
SC 错误高置信候选不会直接绕过几何验证。
SC 失败后 HDL 能被触发。
```

### Phase 4：恢复验收和导航恢复

改动：

1. NDT reacquire window。
2. fusion 验证 NDT 连续 accepted、TF jump、costmap bounds。
3. navigation_state_manager 恢复 interrupted waypoint 或 pending request。

验收：

```text
恢复后 Nav2 重新规划。
不会跳过当前 waypoint。
不会误执行已取消的 pending request。
costmap 不再出现 robot/sensor out of bounds 后才 resume。
```

## 7. 建议补充到原文档的风险表

| 风险 | 当前文档是否覆盖 | 建议补充 |
|------|------------------|----------|
| TF ownership 两套定义冲突 | 未覆盖 | 明确最终架构或过渡架构 |
| `/initialpose` 双源竞争 | 部分覆盖但前后冲突 | 只能 arbiter 发布 |
| fusion 崩溃后 NDT 自动恢复错误 TF | 部分覆盖 | 超时进入 FUSION_TIMEOUT，不自动恢复导航 |
| SC 高置信误匹配 | 未充分覆盖 | SC top-K + HDL/GICP 验证 |
| DEGRADED 跳变条件漏判 | 未覆盖 | 非 HEALTHY 统一 blocked |
| TF prior 时间不一致 | 未覆盖 | prior 带 stamp/age/source/displacement |
| fallback prior 错把 map->odom 当机器人位置 | 未覆盖 | 保存 frozen_map_T_body |
| pending request 覆盖/取消语义 | 部分覆盖 | 定义保留最后一个或队列 |
| NDT accepted 但 costmap 越界 | 未覆盖 | 加 costmap/map bounds 验收 |
| 验收命令看不出 TF publisher | 未覆盖 | 使用 topic info -v 和 bag 检查脚本 |

## 8. 最小可落地修订版

如果希望先快速验证，不一次性引入完整 arbiter，可以按这个最小版本落地：

```text
P0:
  NDT 非 HEALTHY 不发 TF。
  fusion DEGRADED/LOST 发冻结 TF。
  navigation manager 非 HEALTHY 立即 blocked + zero cmd。

P1:
  fusion recovery request 携带 prior。
  SC 使用 prior 先搜，失败后全局搜。
  HDL 暂不并行，只作为手动或显式 fallback。

P2:
  确保只有 SC bridge 发布 /initialpose。
  NDT 连续稳定 + costmap 不越界后 resume。

P3:
  再接 HDL candidate 验证和 fallback。
```

这个版本可以先解决 test3 的主要问题：TF 抢占、继续走导致漂移扩大、recovery 无 prior、恢复后直接跑。

## 9. 评审结论

这份方案值得继续推进，但应先把复杂度收束到三个唯一 ownership：

```text
TF ownership: fusion/rusion
initialpose ownership: recovery_arbiter 或临时 SC bridge
navigation blocked ownership: navigation_state_manager
```

只要这三条边界不清楚，系统就容易从“抢 TF”演化成“抢 initialpose”“抢导航状态”。先把 ownership 定清，再做双引擎协作，整体架构才会稳定。
