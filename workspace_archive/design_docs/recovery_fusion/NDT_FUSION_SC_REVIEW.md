# NDT + Odom Fusion + ScanContext 修改评审记录

> 评审时间: 2026-05-26  
> 评审对象: `NDT_FUSION_SC_MODIFICATION_REPORT.md` 及相关代码  
> 范围: 仅分析和静态检查，未修改代码实现  

---

## 一、总体结论

当前融合方向是合理的：用 `localization_odom_fusion` 在 NDT 退化时冻结 `map->odom`，用 Fast-LIO odom 短时兜底，真正 LOST 后再由 ScanContext 触发全局重定位。

但当前代码和报告存在几个关键不一致点，其中高风险问题会直接影响 DEGRADED 恢复、LOST 触发时机和实机导航稳定性：

1. `_is_healthy()` 的 pose 一致性 gate 实际实现与报告描述不一致。
2. fusion 节点通过 TF 查询 `map->odom` 时无法区分 NDT 和 fusion 自己发布的 TF。
3. `scancontext_to_initialpose` 会因为 `/pcl_pose` stale 提前触发 recovery，绕过 fusion 的 LOST 决策。
4. SC bridge 相比原 HDL bridge 缺少运行期 recovery 的静止/压零/校验保护。

建议先修复前两个架构级问题，再进入实机参数调试。

---

## 二、高风险问题

### 1. `_is_healthy()` 恢复 gate 与报告不一致

**代码出处**

- `src/humanoid_navigation2/humanoid_navigation2/localization_odom_fusion.py`
- `_is_healthy()`，约第 1030-1061 行

当前代码：

```python
if self.frozen_map_odom is not None and self.latest_ndt_map_odom is not None:
    ndt_dx = self.latest_ndt_map_odom['x'] - self.frozen_map_odom['x']
    ndt_dy = self.latest_ndt_map_odom['y'] - self.frozen_map_odom['y']
    pose_jump = math.hypot(ndt_dx, ndt_dy)
    if pose_jump > 0.8:
        return False
```

**报告描述**

报告中写的是：

```python
odom_dx = odom_body['x'] - self.frozen_odom_body[0]
odom_dy = odom_body['y'] - self.frozen_odom_body[1]
expected_x = self.frozen_map_odom['x'] + odom_dx
expected_y = self.frozen_map_odom['y'] + odom_dy
ndt_dx = self.latest_ndt_map_odom['x'] - expected_x
ndt_dy = self.latest_ndt_map_odom['y'] - expected_y
pose_jump = math.hypot(ndt_dx, ndt_dy)
```

**问题说明**

实际代码只比较 `latest_ndt_map_odom` 和 `frozen_map_odom`，没有把 DEGRADED 期间机器人通过 odom 走过的距离算进去。

这会导致两个问题：

- 如果机器人在 DEGRADED 期间实际走了超过 0.8m，即使 NDT 恢复到正确位置，也可能因为和冻结点差太远而被拒绝。
- 如果 `latest_ndt_map_odom` 实际拿到的是 fusion 自己发布的冻结 TF，pose jump 会接近 0，从而误判健康。

**风险影响**

- DEGRADED 后长时间无法自然恢复，只能等 LOST 触发 SC recovery。
- 或者错误恢复为 HEALTHY，导致错误 `map->odom` 被下游接受。
- 报告中的“伪恢复防护”并没有按设计落地。

**修改建议**

优先明确恢复 gate 比较对象：

1. 如果要校验 NDT 当前 robot pose，应比较 `map_T_base` 级别的位置，而不是只比较 `map_T_odom`。
2. 如果继续比较 `map->odom`，必须正确推导 odom 增量对 `map->odom` 的预期影响，不能简单把 camera_init/body 的原始轴增量加到 map 平面坐标上。
3. `_is_healthy()` 需要接收当前 `odom_body`，或在函数内部查询当前 odom，并使用统一坐标系计算。

建议更稳的实现方向：

- 使用 `/pcl_pose` 或 NDT 原始候选 topic 获取 NDT 输出位姿。
- 使用 fusion 当前冻结 TF + 当前 odom TF 计算 odom 兜底下的 robot map 位姿。
- 比较 NDT robot map 位姿与 odom 兜底 robot map 位姿的 XY/yaw 差。

---

### 2. TF 查询无法区分 NDT 和 fusion 自己发布的 `map->odom`

**代码出处**

- `src/humanoid_navigation2/humanoid_navigation2/localization_odom_fusion.py`
- `_lookup_ndt_map_odom()`，约第 555-580 行
- `_publish_map_odom_tf()`，约第 1088-1115 行

查询代码：

```python
transform = self.tf_buffer.lookup_transform(
    FRAME_MAP,
    FRAME_ODOM,
    Time(),
    timeout=Duration(seconds=0.1),
)
```

发布代码：

```python
t.header.frame_id = FRAME_MAP
t.child_frame_id = FRAME_ODOM
self.tf_broadcaster.sendTransform(t)
```

**问题说明**

fusion 节点在 DEGRADED/TRANSITIONING 状态下会发布 `map->odom`。NDT 定位节点也发布同一个 `map->odom`。TF buffer 查询时只能按 frame 查询，不能指定发布者。

因此 `_lookup_ndt_map_odom()` 名义上想拿 NDT 的 `map->odom`，但实际可能拿到 fusion 自己刚发布的冻结或插值 TF。

**风险影响**

- `latest_ndt_map_odom` 被 fusion 自己污染。
- `_is_healthy()` 可能读到冻结 TF，误认为 NDT 位姿和冻结值一致。
- `_enter_transitioning()` 的 `transition_to` 可能不是 NDT 恢复目标，而是 fusion 自己的冻结值。
- 诊断数据中的 NDT map->odom 不可信。

**修改建议**

不要通过同名 TF 读取 NDT 原始输出。可选方案：

1. 让 NDT 节点额外发布一个专用 topic，例如 `/localization/ndt_map_odom` 或 `/localization/ndt_candidate_pose`。
2. fusion 订阅 NDT 的 `/pcl_pose`，把它作为 NDT 输出源。
3. 修改 NDT 节点在融合模式下不要直接发布 `map->odom`，改为只发布 topic，由 fusion 成为唯一 `map->odom` 发布者。
4. 或者让 NDT 发布到不同 child frame，例如 `ndt_odom_candidate`，但这会牵涉 TF 树设计，建议优先 topic 化。

推荐方向：

- NDT 只发布状态和候选位姿。
- fusion 根据健康状态决定是否把候选位姿转换成正式 `map->odom`。
- 下游只消费 fusion 发布的唯一 `map->odom`。

---

### 3. SC bridge 会绕过 fusion LOST 决策提前 recovery

**代码出处**

- `src/humanoid_navigation2/humanoid_navigation2/scancontext_to_initialpose.py`
- `timer_callback()`，约第 188-230 行
- `navigation2_fusion_sc.launch.py`，约第 448-479 行

launch 参数：

```python
'monitor_localization': True,
'localization_pose_stale_sec': 2.5,
'runtime_trigger_period_sec': 8.0,
```

触发逻辑：

```python
localization_stale = (
    self.monitor_localization
    and self.last_localization_pose_time > 0.0
    and now - self.last_localization_pose_time > self.localization_pose_stale_sec
)
localization_missing = self.monitor_localization and self.last_localization_pose_time <= 0.0
external_recovery = now < self.external_recovery_until
recovery_needed = localization_stale or localization_missing or external_recovery
```

**问题说明**

报告描述的主流程是：

```text
NDT 退化 -> fusion DEGRADED -> odom 兜底 -> 超时/位移过大 -> fusion LOST -> 请求 SC recovery
```

但当前 SC bridge 会持续监控 `/pcl_pose`。NDT DEGRADED 后 `/pcl_pose` 可能停止更新或只发布 last good，超过 2.5s 后 SC bridge 会自行启动 recovery，不需要等 fusion LOST，也不需要 fusion 发布 `/localization/recovery_requests`。

**风险影响**

- robot 仍在 DEGRADED odom 兜底导航时，SC bridge 可能突然发布 `/initialpose`。
- fusion 还没判定 LOST，APP 层也可能还没有进入定位恢复暂停流程。
- NDT 被重初始化的时机不受 fusion 控制，破坏状态机设计。

**修改建议**

建议把 SC bridge 分成启动初始化和运行期恢复两个模式：

1. 启动阶段允许 `localization_missing` 触发 SC initialpose。
2. 运行期关闭 `/pcl_pose` stale 自动触发，只响应 `/localization/recovery_requests`。
3. 或者增加参数，例如 `runtime_recovery_requires_external_request=True`。
4. 若保留 stale 触发，也必须先检查 fusion 状态：只有 fusion 为 `LOST` 才允许运行期 recovery。

短期 launch 建议：

- 将运行期 `monitor_localization` 关闭，或增加 startup-only 逻辑。
- 保留 fusion 主导的 `/localization/recovery_requests`。

---

### 4. SC bridge 缺少 HDL bridge 中的运行期保护

**代码出处**

- `src/humanoid_navigation2/launch/navigation2_fusion_sc.launch.py`
- `scancontext_to_initialpose_node` 参数，约第 448-479 行
- 对比 `navigation2_fusion.launch.py` 中 `hdl_bootstrap_to_initialpose_node`，约第 547-637 行

HDL bridge 原有保护包括：

```python
'wait_stationary_before_runtime_relocalize': True,
'runtime_stationary_settle_sec': 1.0,
'runtime_stationary_max_xy_delta': 0.08,
'runtime_stationary_max_yaw_delta': 0.08,
'publish_zero_cmd_vel_during_recovery': True,
'recovery_stop_cmd_vel_topic': '/cmd_vel',
'map_to_odom_tf_stale_sec': 3.0,
'ndt_recovery_max_correction_translation': 0.80,
'ndt_recovery_max_correction_yaw': 0.45,
```

SC bridge 当前主要是：

```python
'recovery_settle_sec': 6.0,
'require_ndt_stable_status_for_recovery': True,
'ndt_recovery_required_stable_status_count': 3,
'publish_repetitions': 8,
```

**问题说明**

SC bridge 可以发布 `/initialpose` 并等待 NDT `accepted`，但缺少以下保护：

- runtime recovery 前等待机器人静止。
- recovery 过程中持续发布零速度。
- 校验 `map->odom` 是否 stale。
- 校验 NDT correction translation/yaw 是否在合理范围。
- 对运行期 recovery 和启动 initialpose 使用不同参数。

**风险影响**

- 运动中重定位，导致 initialpose 与实际 scan/odom 不一致。
- NDT 短暂 accepted 但实际位姿跳变过大。
- APP 层虽然 LOST 时会暂停导航，但 recovery 节点本身没有底层强制停机保护。

**修改建议**

把 HDL bridge 中对运行期恢复有价值的保护迁移到 SC bridge：

1. recovery 前等待机器人静止，基于 `/odom` 或 TF 采样。
2. recovery 过程中持续发布 `/cmd_vel=0`，直到 NDT 验收结束。
3. NDT 验收不能只看 `state == accepted`，还应检查 `correction_translation`、`correction_yaw`、`fitness_score`。
4. 启动阶段和运行期 recovery 参数分离。
5. SC recovery 成功后再发布 `localization_recovered`。

---

## 三、中风险问题

### 5. fusion 对 `localization_recovered` 事件过度信任

**代码出处**

- `src/humanoid_navigation2/humanoid_navigation2/localization_odom_fusion.py`
- `_on_recovery_status()`，约第 420-426 行

当前代码：

```python
if event_type == 'localization_recovered':
    self._reset_state()
    self.state = FusionState.HEALTHY
    self._recovery_in_progress = False
    self._publish_fusion_status()
```

**问题说明**

fusion 在 LOST 状态收到 `localization_recovered` 后直接切回 HEALTHY，没有自行复核：

- 当前 NDT status 是否新鲜。
- 当前 `fitness_score` 是否低于阈值。
- NDT 位姿和 odom/fusion 期望是否一致。
- `map->odom` 是否确实由新 NDT 结果更新。

**风险影响**

如果 SC bridge 误判 recovered，fusion 会立即把控制权交回 NDT，可能导致错误 `map->odom` 被下游接受。

**修改建议**

fusion 收到 `localization_recovered` 后进入一个 `RECOVERING/VERIFYING` 中间态，要求：

- NDT status 连续若干帧 accepted。
- status age < 1s。
- fitness < healthy threshold 或 recovery 专用阈值。
- 位姿 jump/correction 在合理范围。

满足后再切 HEALTHY。

---

### 6. `/localization/fusion_status` 不是 JSON

**代码出处**

- `src/humanoid_navigation2/humanoid_navigation2/localization_odom_fusion.py`
- `_publish_fusion_status()`，约第 1121-1146 行

当前代码：

```python
msg.data = str(status)
```

APP 层解析：

- `src/humanoid_navigation/humanoid_navigation/navigation_state_manager_fusion.py`
- `_on_fusion_status()`，约第 315-327 行

```python
data = ast.literal_eval(msg.data) if isinstance(msg.data, str) else json.loads(msg.data)
```

**问题说明**

当前 APP 层能解析 Python dict 字符串，但报告兼容性矩阵里把接口描述成 JSON。其他节点如果按 JSON 解析会失败。

**风险影响**

- 后续监控、WebSocket、日志工具按 JSON 接入时不兼容。
- 与 `/localization/recovery_status` 的 JSON 风格不一致。

**修改建议**

统一改为 `json.dumps(status, ensure_ascii=False)`。

为兼容旧 APP，可短期让 APP 同时支持 JSON 和 legacy dict 字符串。

---

### 7. SC 启动时序可能偏紧

**代码出处**

- `src/humanoid_navigation2/launch/navigation2_fusion_sc.launch.py`
- 启动时序，约第 912-921 行

当前时序：

```python
TimerAction(period=4.0, actions=[scancontext_global_localizer_node])
TimerAction(period=5.0, actions=[scancontext_to_initialpose_node])
TimerAction(period=5.0, actions=[ndt_localization_node])
TimerAction(period=7.0, actions=[ndt_lifecycle_manager])
TimerAction(period=8.0, actions=[localization_odom_fusion_node])
```

**问题说明**

SC localizer 需要加载：

- ScanContext 数据库：`hall_sc_fastlio_registered.bin`，约 6.3MB。
- GICP 地图：`hall.pcd`，约 137MB。

4s 后启动 localizer、5s 后 bridge 和 NDT 同时启动，可能出现 bridge 首次 trigger 时 SC service 尚未 ready 或地图尚未完成加载。

**风险影响**

- 启动阶段 initialpose 延迟或失败。
- Nav2 的 `wait_for_tf` 长时间等待。
- 日志中出现 service not ready、no cloud、database empty 等信息。

**修改建议**

实机验证 SC localizer 从进程启动到 service ready 的耗时。根据结果调整：

- SC localizer 提前启动。
- bridge 等 service ready 后再进入 startup trigger。
- 或给 bridge 增加 startup delay/service-ready 日志。

---

### 8. `min_publish_interval_sec=12` 可能压掉 recovery 候选

**代码出处**

- `src/humanoid_navigation2/humanoid_navigation2/scancontext_to_initialpose.py`
- `best_pose_callback()`，约第 328-344 行
- launch 参数约第 471-475 行

当前逻辑：

```python
if now - self.last_publish_time < self.min_publish_interval_sec:
    return
```

**问题说明**

如果启动阶段刚发布过 `/initialpose`，短时间内 fusion LOST 触发的新 SC best pose 会被直接丢弃，直到 12s 间隔结束。

**风险影响**

- LOST 后恢复延迟增加。
- 外部 recovery 请求期间可能没有任何 `/initialpose` 发出。

**修改建议**

区分普通重复发布抑制和外部 recovery 请求：

- 外部请求或 global recovery 成功时允许绕过 `min_publish_interval_sec`。
- 或将 interval 改为针对同一候选 pose 的去抖，而不是所有 best pose 全局抑制。

---

## 四、文档与代码不一致点

### 1. `_is_healthy` 的实现描述不准确

报告中 2.5 节描述的是 odom 推算 gate，实际代码不是该逻辑。需要更新报告，或修正代码实现。

### 2. “NDT 独占发布 map->odom”和“fusion 是唯一发布者”冲突

`navigation2_fusion_sc.launch.py` 注释约第 400-405 行同时出现：

- “NDT 独占发布 map->odom”
- “fusion 节点是唯一 map->odom 发布者”

实际情况是：

- HEALTHY 时希望 NDT 发布正式 `map->odom`。
- DEGRADED/TRANSITIONING 时 fusion 也发布 `map->odom`。

这不是严格意义上的唯一发布者，容易误导后续维护。

建议明确设计：

- 方案 A：NDT 永远不直接发布正式 TF，fusion 是唯一发布者。
- 方案 B：NDT 和 fusion 分状态发布，但不能把 TF 查询当作 NDT 原始输出源。

### 3. 兼容性矩阵只覆盖 topic 名称，未覆盖语义差异

SC bridge 和 HDL bridge 都发布 `/localization/recovery_status`，但语义保护不完全等价：

- HDL bridge 有更多 runtime recovery 校验。
- SC bridge 当前更轻量，容易提前触发。

建议在报告中补充“接口兼容不等于行为等价”。

---

## 五、建议修复优先级

### P0: 必须先修

1. 解决 fusion 读取 NDT 原始位姿时的 TF 发布者混淆问题。
2. 让运行期 SC recovery 只由 fusion LOST 请求触发，避免 `/pcl_pose` stale 自行 recovery。

### P1: 强烈建议修

1. 重写 `_is_healthy()` 的 pose gate，按统一坐标系比较 robot pose。
2. SC bridge 增加运行期静止/压零/验收保护。
3. fusion 收到 `localization_recovered` 后增加 VERIFYING 阶段。

### P2: 清理和一致性

1. `/localization/fusion_status` 改成 JSON。
2. 修正 launch 注释和报告中与实际不一致的描述。
3. 根据实机日志调整 SC 启动延迟和 `min_publish_interval_sec`。

---

## 六、建议验证项

修复后建议至少验证以下场景：

1. 正常启动：SC 发布 `/initialpose`，NDT accepted，fusion 进入 HEALTHY，Nav2 等 TF 后启动。
2. 短时 NDT 退化：fusion 进入 DEGRADED，SC 不提前发布 `/initialpose`，NDT 自然恢复后 fusion 平滑切 HEALTHY。
3. 长时 NDT 退化：fusion 到 LOST 后发布 `/localization/recovery_requests`，SC 才开始运行期 recovery。
4. 运动中 LOST：APP 暂停导航，SC recovery 前确认机器人静止或持续压零。
5. 伪恢复：NDT 低 fitness 但 robot pose 与 odom 兜底差距过大时，fusion 拒绝恢复。
6. 错误 SC 候选：SC 发布 initialpose 后 NDT rejected，bridge 不发布 recovered，并继续重试或进入 global recovery。

---

## 七、静态检查结果

已做的静态检查：

```bash
python3 -c "import ast, pathlib; [ast.parse(pathlib.Path(p).read_text()) for p in ['src/humanoid_navigation2/humanoid_navigation2/localization_odom_fusion.py','src/humanoid_navigation2/humanoid_navigation2/scancontext_to_initialpose.py','src/humanoid_navigation2/launch/navigation2_fusion_sc.launch.py','src/humanoid_navigation/launch/navigation_fusion_sc.launch.py']] ; print('ast ok')"
```

结果：

```text
ast ok
```

说明 Python 语法层面通过，但不代表 ROS 运行期接口、TF 行为和状态机语义正确。
