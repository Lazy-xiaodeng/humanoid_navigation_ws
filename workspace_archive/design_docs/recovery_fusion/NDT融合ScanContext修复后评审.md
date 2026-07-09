# NDT + Odom Fusion + ScanContext 修复后复审记录

> 复审时间: 2026-05-26  
> 复审对象: `NDT融合ScanContext修改报告.md` 第十一节修复记录及源码  
> 范围: 只评审，不修改源码  

---

## 一、总体结论

本轮修复已经覆盖了前次评审中两个 P0 问题的主路径：

1. fusion 通过 `/pcl_pose` 获取 NDT 原始 `map->odom`，避免 DEGRADED/TRANSITIONING 期间 TF 查询读到 fusion 自己发布的冻结 TF。
2. SC bridge 增加 `enable_runtime_auto_recovery`，并在 fusion+SC launch 中关闭运行期自动 recovery，使运行期重定位由 fusion LOST 请求主导。
3. `/localization/fusion_status` 已改为 JSON，APP 层保留旧格式兼容。

但仍有若干残留风险，尤其是 `/pcl_pose` freshness 未校验。当前实现已比上一版明显更可控，但还不算完全闭环。

---

## 二、前次问题复核

### P0-1: TF 查询混淆

**结论**: 主问题已基本解决，但有时序残余风险。

**修复代码出处**

- `src/humanoid_navigation2/humanoid_navigation2/localization_odom_fusion.py`
- 新增 `/pcl_pose` 订阅，约第 249-257 行

```python
self.pcl_pose_sub = self.create_subscription(
    PoseWithCovarianceStamped,
    '/pcl_pose',
    self._on_pcl_pose,
    QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE),
)
```

- 新增 `_on_pcl_pose()`，约第 399-420 行

```python
def _on_pcl_pose(self, msg: PoseWithCovarianceStamped):
    self.latest_pcl_map_odom = {
        'x': msg.pose.pose.position.x,
        'y': msg.pose.pose.position.y,
        'z': msg.pose.pose.position.z,
        'qx': msg.pose.pose.orientation.x,
        'qy': msg.pose.pose.orientation.y,
        'qz': msg.pose.pose.orientation.z,
        'qw': msg.pose.pose.orientation.w,
    }
    self.latest_pcl_pose_time = time.monotonic()
```

- `_enter_transitioning()` 改用 `/pcl_pose`，约第 934-944 行

```python
pcl_odom = self.latest_pcl_map_odom
if pcl_odom is None:
    ...
self.transition_to = pcl_odom.copy()
```

- `_is_healthy()` 改用 `/pcl_pose`，约第 1092-1101 行

```python
pcl_odom = self.latest_pcl_map_odom
if self.frozen_map_odom is not None and pcl_odom is not None:
    ndt_dx = pcl_odom['x'] - self.frozen_map_odom['x']
    ndt_dy = pcl_odom['y'] - self.frozen_map_odom['y']
    pose_jump = math.hypot(ndt_dx, ndt_dy)
    if pose_jump > 0.8:
        return False
```

**确认点**

NDT 当前配置中 `base_frame_id: "odom"`，所以 `/pcl_pose` 发布的是 `map -> odom` 位姿，不是 `map -> base_footprint` 机器人位姿。因此用 `/pcl_pose` 代替 TF 查询获取 NDT 原始 `map->odom` 是成立的。

相关配置位于安装后的 `lidar_localization_ros2` 参数文件：

```yaml
global_frame_id: "map"
odom_frame_id: "odom"
base_frame_id: "odom"
```

**残余问题**

代码记录了 `latest_pcl_pose_time`，但没有在 `_is_healthy()` 或 `_enter_transitioning()` 中校验 freshness。

风险场景：

1. NDT status 新到达，`fitness_score < healthy_error_threshold` 且 `has_converged=True`。
2. `/pcl_pose` 因生命周期、QoS、发布节奏或 NDT reject/republish 机制没有同步更新。
3. fusion 使用旧的 `latest_pcl_map_odom` 通过 pose gate 或作为 transition 目标。

**影响**

- 可能出现“新 status + 旧 pose”的组合。
- pose gate 可能错误通过或错误拒绝。
- transition 目标可能落后于 NDT 当前真实输出。

**建议优化**

在 `_is_healthy()` 中增加 `/pcl_pose` 新鲜度检查：

```python
pcl_age = time.monotonic() - self.latest_pcl_pose_time
if pcl_age > 0.5:
    return False
```

更稳的做法是把 NDT status 的 `stamp_sec` 和 `/pcl_pose.header.stamp` 对齐，要求两者时间差在一个可接受窗口内。

---

### P0-2: SC bridge 绕过 fusion 自行 recovery

**结论**: 主问题已基本解决。

**修复代码出处**

- `src/humanoid_navigation2/humanoid_navigation2/scancontext_to_initialpose.py`
- 新增参数，约第 140-145 行

```python
self.enable_runtime_auto_recovery = bool(
    self.declare_parameter("enable_runtime_auto_recovery", True).value
)
```

- `timer_callback()` 区分启动期和运行期，约第 205-213 行

```python
if startup:
    recovery_needed = localization_missing or external_recovery
else:
    if self.enable_runtime_auto_recovery:
        recovery_needed = (localization_stale or localization_missing
                           or external_recovery)
    else:
        recovery_needed = external_recovery
```

- `navigation2_fusion_sc.launch.py` 中关闭运行期自动 recovery，约第 472-474 行

```python
'enable_runtime_auto_recovery': False,
```

**效果**

启动阶段：

- 仍允许 localization missing 触发 SC initialpose。

运行期：

- `/pcl_pose` stale 不再自行触发 SC recovery。
- 只有 fusion 发出 `/localization/recovery_requests` 后，SC bridge 才进入 recovery。

**残余问题**

启动阶段仍会根据 `localization_missing` 自动触发，这符合当前设计。但如果启动阶段超过 30s 后还没有完成初始化，行为会切换为运行期逻辑，此时若没有 external request，SC bridge 不会继续自动尝试。

**建议优化**

确认实际启动耗时。如果 SC/GICP 地图加载或首帧云等待经常超过 30s，可以：

- 延长 `startup_duration_sec`。
- 或引入更明确的 startup complete 条件，而不是只用 wall-clock 30s。

---

### P2: fusion_status JSON 格式

**结论**: 已解决。

**修复代码出处**

- `src/humanoid_navigation2/humanoid_navigation2/localization_odom_fusion.py`
- `_publish_fusion_status()`，约第 1187-1189 行

```python
import json as _json
msg.data = _json.dumps(status, ensure_ascii=False)
self.fusion_status_pub.publish(msg)
```

- `src/humanoid_navigation/humanoid_navigation/navigation_state_manager_fusion.py`
- `_on_fusion_status()`，约第 315-323 行

```python
import ast, json as _json
try:
    data = _json.loads(msg.data)
except (ValueError, _json.JSONDecodeError):
    data = ast.literal_eval(msg.data) if isinstance(msg.data, str) else _json.loads(msg.data)
```

**效果**

- 新消息为标准 JSON。
- APP 层仍兼容旧的 Python dict repr。

---

## 三、仍存在的风险点

### 1. `/pcl_pose` 缺少 freshness 校验

**优先级**: P0/P1 之间，建议尽快修。

**代码出处**

- `src/humanoid_navigation2/humanoid_navigation2/localization_odom_fusion.py`
- `latest_pcl_pose_time` 只写入，不读取：
  - 初始化约第 322 行
  - 更新约第 420 行

**问题说明**

`latest_pcl_pose_time` 没有参与任何状态判断。`_is_healthy()` 只验证 NDT status 的 error/converged，然后使用最近一次 `/pcl_pose` 做 pose gate。

这比上一版的 TF 污染问题轻，但仍可能导致状态机使用过期 NDT 位姿。

**建议**

新增参数：

```python
max_pcl_pose_age_sec = 0.5
```

在 `_is_healthy()` 和 `_enter_transitioning()` 中检查：

```python
if self.latest_pcl_map_odom is None:
    return False
if time.monotonic() - self.latest_pcl_pose_time > self.max_pcl_pose_age_sec:
    return False
```

---

### 2. fusion 对 `localization_recovered` 仍然过度信任

**优先级**: P1，文档已标注延后。

**代码出处**

- `src/humanoid_navigation2/humanoid_navigation2/localization_odom_fusion.py`
- `_on_recovery_status()`，约第 457-463 行

```python
if event_type == 'localization_recovered':
    self._reset_state()
    self.state = FusionState.HEALTHY
    self._recovery_in_progress = False
    self._publish_fusion_status()
```

**问题说明**

SC bridge 会等待 3 帧 NDT accepted，但 fusion 收到 recovered 后没有自己复核：

- NDT status 是否新鲜。
- `/pcl_pose` 是否新鲜。
- NDT error 是否低于恢复阈值。
- 当前 `map->odom` 是否稳定。

**建议**

增加 `RECOVERING` 或 `VERIFYING` 状态：

1. 收到 `localization_recovered` 后进入 VERIFYING。
2. 连续若干帧验证 NDT status 和 `/pcl_pose` freshness。
3. 通过后再切 HEALTHY。

---

### 3. SC bridge 仍缺运行期静止/压零保护

**优先级**: P1，文档已标注延后。

**代码出处**

- `src/humanoid_navigation2/humanoid_navigation2/scancontext_to_initialpose.py`
- 当前 recovery 主要依赖 trigger + `/initialpose` + NDT accepted 验证。

**问题说明**

相比 HDL bridge，SC bridge 仍缺少：

- runtime recovery 前等待机器人静止。
- recovery 过程中持续发布 `/cmd_vel=0`。
- NDT correction translation/yaw 限幅。
- map->odom stale 校验。

**当前缓解**

修复 P0-2 后，运行期 SC recovery 只有 fusion LOST 请求才触发；APP 状态管理器会在 LOST/recovery 过程中暂停导航。

**剩余风险**

APP 层暂停和 recovery 节点本身压零不是同一个安全层。若状态管理器异常、取消目标延迟或底盘仍保留旧速度，SC bridge 没有底层兜底。

**建议**

从 HDL bridge 迁移最小保护集：

1. recovery 前采样 `/odom` 或 TF，确认静止。
2. recovery 期间以 10Hz 发布 `/cmd_vel=0`。
3. NDT 验收除 `state == accepted` 外，增加 `correction_translation/correction_yaw/fitness_score` 限制。

---

### 4. `min_publish_interval_sec=12` 仍可能压掉外部 recovery 候选

**优先级**: P2，文档已标注延后。

**代码出处**

- `src/humanoid_navigation2/humanoid_navigation2/scancontext_to_initialpose.py`
- `best_pose_callback()`，约第 343-346 行

```python
if now - self.last_publish_time < self.min_publish_interval_sec:
    return
```

**问题说明**

该限流对所有 best pose 生效。如果刚刚发布过 startup initialpose，随后 fusion LOST 触发外部 recovery，新的 best pose 可能被 12s 间隔抑制。

**建议**

外部 recovery 请求或 global recovery 成功时绕过该限制，或者只对“同一候选位姿”限流。

---

### 5. launch 注释仍有语义冲突

**优先级**: P2。

**代码出处**

- `src/humanoid_navigation2/launch/navigation2_fusion_sc.launch.py`
- 约第 403-405 行

```python
# │  一个经过 odom gate/GICP gate 的 /initialpose。NDT 独占发布 map->odom。     │
# │  fusion 节点是唯一 map->odom 发布者，统一管理 HEALTHY/DEGRADED/LOST 状态。  │
```

**问题说明**

这两句互相冲突。实际行为是：

- HEALTHY 时 fusion 不发布，让 NDT 发布 `map->odom`。
- DEGRADED/TRANSITIONING 时 fusion 发布 `map->odom`。
- LOST 时 fusion 不发布，等待 recovery。

**建议**

改成：

```text
NDT 在 HEALTHY 时发布正式 map->odom；
fusion 在 DEGRADED/TRANSITIONING 时临时覆盖发布冻结/过渡 map->odom；
NDT 原始输出通过 /pcl_pose 提供给 fusion 做恢复判断。
```

---

### 6. 报告描述与 launch 参数不一致：global recovery confidence gate

**优先级**: P2。

**代码出处**

- 报告前文描述有 candidate confidence gate。
- `src/humanoid_navigation2/launch/navigation2_fusion_sc.launch.py`
- 约第 442 行

```python
'global_recovery_enable_candidate_confidence_gate': False,
```

**问题说明**

普通 conservative trigger 仍启用 candidate confidence gate：

```python
'enable_candidate_confidence_gate': True,
```

但 global recovery 模式关闭了 confidence gate。若这是基于实测为了提高召回率而有意关闭，需要在报告中明确说明；否则读者会误以为 global recovery 仍具备完整 5 层 gate。

**2026-05-26 16:29 更新**

该项已按实机错位日志修正。开机 SC 接受 `x=0.323, y=-1.173, yaw=-104.4deg` 后，NDT 以 `error=0.1186` 误判健康；人工 RViz 正确初值为 `x=6.229, y=10.813, yaw=90.7deg`。这证明关闭 global confidence gate 后，仅靠 NDT accepted 不足以过滤相似走廊错位。

当前已改为:

```python
'global_recovery_enable_candidate_confidence_gate': True,
'global_recovery_gicp_fitness_threshold': 0.09,
'global_recovery_required_consistent_results': 4,
'global_recovery_consistency_window': 8,
'global_recovery_consistency_xy_tolerance': 0.5,
'global_recovery_consistency_yaw_tolerance': 0.25,
```

**建议**

在报告中拆开描述：

- 普通 recovery: SC distance -> odom consistency -> candidate confidence -> GICP -> refined odom。
- global recovery: SC distance -> candidate confidence -> GICP -> 多帧一致性；如果候选歧义，宁可继续等待或人工给初值，不自动接受。

---

## 四、已做静态检查

### Python 编译检查

命令：

```bash
python3 -m py_compile \
  src/humanoid_navigation2/humanoid_navigation2/localization_odom_fusion.py \
  src/humanoid_navigation2/humanoid_navigation2/scancontext_to_initialpose.py \
  src/humanoid_navigation/humanoid_navigation/navigation_state_manager_fusion.py
```

结果：通过。

### Launch AST 检查

命令：

```bash
python3 -c "import ast, pathlib; [ast.parse(pathlib.Path(p).read_text()) for p in ['src/humanoid_navigation2/launch/navigation2_fusion_sc.launch.py','src/humanoid_navigation/launch/navigation_fusion_sc.launch.py']] ; print('launch ast ok')"
```

结果：

```text
launch ast ok
```

---

## 五、建议下一步优化顺序

### 优先级 1

补 `/pcl_pose` freshness 校验。  
这是当前 P0-1 修复后的主要残余风险，改动小、收益高。

### 优先级 2

修正 launch 注释和报告中 global recovery gate 描述。  
这不会改变行为，但能避免后续维护误判架构。

### 优先级 3

给 fusion 增加 recovery VERIFYING 阶段。  
这会提高安全性，但需要更细的状态机设计。

### 优先级 4

给 SC bridge 补 runtime 静止/压零/NDT correction 限幅。  
这是实机安全层面的增强，建议在下一轮实机验证前规划。

### 优先级 5

优化 `min_publish_interval_sec` 的作用范围。  
把全局 12s 限流改成只抑制重复候选，外部 recovery 请求不被压制。

---

## 六、复审结论

当前修复后，系统已经从“状态机可能被 TF 污染和 SC 自行 recovery 破坏”的状态，推进到“主状态机基本可控”的状态。

仍建议在实机验证前至少补上 `/pcl_pose` freshness 校验，因为这直接关系到 DEGRADED 自然恢复和 transition 目标是否可信。
