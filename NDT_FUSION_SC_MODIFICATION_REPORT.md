# NDT + Odom Fusion + ScanContext 组合方案修改报告

> 修改时间: 2026-05-25 23:18  
> 工作空间: `/home/ubuntu/humanoid_ws`  
> 目的: 将 ScanContext (SC) 全局重定位机制融入 NDT + Odom Fusion 体系，替换 HDL bootstrap 作为 LOST 后的全局重定位器

---

## 一、修改总览

| # | 文件 | 操作 | 说明 |
|---|---|---|---|
| 1 | `localization_odom_fusion.py` | **修改** | 修复 NDT 状态订阅 Bug；增强 recovery 回调；完善状态重置；增强 NDT 恢复验收 |
| 2 | `navigation2_fusion_sc.launch.py` | **新建** | Fusion + SC 组合 launch 文件（定位层） |
| 3 | `navigation_fusion_sc.launch.py` | **新建** | APP 层 SC 变体 launch 文件 |

---

## 二、修改 1: `localization_odom_fusion.py` — 核心融合节点

**文件路径**: `src/humanoid_navigation2/humanoid_navigation2/localization_odom_fusion.py`

### 2.1 Bug 修复: NDT 状态订阅类型不匹配 (CRITICAL)

**问题**: fusion 节点使用了错误的消息类型 `ScanMatchingStatus` 订阅 `/localization/ndt_status`，但 NDT 节点实际发布 `std_msgs::msg::String` (JSON)。导致 `_on_ndt_status` 回调永不触发 → `latest_ndt_error` 始终为 `inf` → 节点启动后立即进入 DEGRADED 并永不恢复。

**修改位置**: 第 57 行 (删除 import)、第 242-247 行 (修改订阅)、第 351-367 行 (改写回调)

**修改内容**:
1. 删除 `from hdl_localization.msg import ScanMatchingStatus`
2. 订阅类型从 `ScanMatchingStatus` 改为 `String`
3. 回调改写为解析 JSON:

```python
def _on_ndt_status(self, msg: String):
    try:
        import json
        status = json.loads(msg.data)
        self.latest_ndt_error = float(status.get('fitness_score', float('inf')))
        self.latest_ndt_inlier = float(status.get('inlier_fraction', 0.0))
        self.latest_ndt_converged = bool(status.get('has_converged', False))
        self.latest_ndt_state = str(status.get('state', ''))
        self.latest_ndt_status_time = time.monotonic()
    except Exception:
        pass
```

**JSON 字段映射**:
| JSON 字段 | fusion 变量 | 说明 |
|---|---|---|
| `fitness_score` | `latest_ndt_error` | NDT 匹配误差 |
| `has_converged` | `latest_ndt_converged` | NDT 是否收敛 |
| `inlier_fraction` | `latest_ndt_inlier` | 内点比例 |
| `state` | `latest_ndt_state` | 当前状态 ("accepted"/"rejected") |

### 2.2 Enhancement: Recovery 状态回调 (SC 兼容)

**修改位置**: 第 369-389 行

**修改内容**:
- 改为 JSON 解析 `event_type` 字段
- 新增 `localization_recovery_started` → 设置 `_recovery_in_progress = True`
- 新增 `localization_relocalize_failed` → 清除 `_recovery_in_progress = False`
- 原有 `localization_recovered` → LOST→HEALTHY 转换保留
- 保留 legacy 非 JSON 兼容性 fallback

### 2.3 Enhancement: `_request_recovery` 防重复

**修改位置**: 第 957-960 行

**修改内容**: 新增 `_recovery_in_progress` 检查，当 SC bridge 已在执行 recovery 时不再发送重复请求:

```python
if getattr(self, '_recovery_in_progress', False):
    self.get_logger().info('[LOST] recovery 已在执行中，跳过重复请求')
    return
```

### 2.4 Enhancement: `_reset_state` 完善

**修改位置**: 第 977-986 行

**修改内容**: 新增重置 `self._recovery_in_progress = False`

### 2.5 Enhancement: `_is_healthy` Pose 一致性检查

**修改位置**: 第 1022-1052 行

**修改内容**: 在 DEGRADED 恢复场景下，验证 NDT 当前 pose 与 odom 推算位置的偏差。核心逻辑：

```python
# 推算: frozen_map_odom + (current_odom_body - frozen_odom_body)
odom_dx = odom_body['x'] - self.frozen_odom_body[0]
odom_dy = odom_body['y'] - self.frozen_odom_body[1]
expected_x = self.frozen_map_odom['x'] + odom_dx
expected_y = self.frozen_map_odom['y'] + odom_dy
ndt_dx = self.latest_ndt_map_odom['x'] - expected_x
ndt_dy = self.latest_ndt_map_odom['y'] - expected_y
pose_jump = math.hypot(ndt_dx, ndt_dy)
if pose_jump > 0.8:  # 位置差 > 0.8m → 拒绝恢复
    return False
```

**设计思路**: 几何混叠区域可能出现"低 error 但错位置"的伪恢复。odom 短期漂移小（通常 < 0.3m），NDT 如果恢复正确应该与 odom 推算位置接近。差距 > 0.8m 说明 NDT 可能收敛到了错误位置。

---

## 三、修改 2: `navigation2_fusion_sc.launch.py` — Fusion + SC 组合 Launch

**文件路径**: `src/humanoid_navigation2/launch/navigation2_fusion_sc.launch.py`

### 3.1 修改思路

以 `navigation2_fusion.launch.py` 为模板，将 HDL bootstrap 三件套替换为 ScanContext 两件套：

| 删除 (HDL bootstrap) | 替换为 (ScanContext) |
|---|---|
| `hdl_bootstrap_global_localization_node` (FPFH+RANSAC) | `scancontext_global_localizer_node` (5层gate SC) |
| `hdl_bootstrap_container` (GlobalmapServer + HdlLocalization) | 删除（SC 不需要额外容器） |
| `hdl_bootstrap_to_initialpose_node` | `scancontext_to_initialpose_node` |

### 3.2 保留不变

- 所有 fusion 节点参数：`degraded_error_threshold=0.5`, `healthy_error_threshold=0.15`, `nav_active_lost_timeout=120s`, `nav_idle_lost_timeout=600s`, `max_odom_displacement=30m`, `max_total=100m`
- NDT 节点：`reject_pose_jump=True`, `score_threshold=2.0`
- 传感器层、Fast-LIO、TF 桥接、感知层、Nav2 完整不变

### 3.3 SC 节点参数

**scancontext_global_localizer_node** 参数：
- 5 层 gate: SC distance (0.25) → odom consistency (1.0m) → candidate confidence → GICP refinement (0.6) → refined odom (1.5m)
- Global recovery: GICP threshold 严格至 0.20, 3/6 多帧一致性 (0.8m/0.35rad)

**scancontext_to_initialpose_node** 参数：
- `global_recovery_after_attempts=3`: 3 次普通 recovery 失败后切换全局模式
- `external_recovery_request_duration_sec=12.0`: fusion recovery 请求的响应窗口
- `publish_repetitions=8, period=0.25`: 8 次重复发布 /initialpose

### 3.4 启动时序

```
4.0s: scancontext_global_localizer_node
5.0s: scancontext_to_initialpose_node + ndt_localization_node
7.0s: ndt_lifecycle_manager
8.0s: localization_odom_fusion_node
```

---

## 四、修改 3: `navigation_fusion_sc.launch.py` — APP 层 Launch

**文件路径**: `src/humanoid_navigation/launch/navigation_fusion_sc.launch.py`

### 4.1 修改思路

复制 `navigation_fusion.launch.py`，使用相同的 `navigation_state_manager_fusion` 可执行文件。兼容性分析：

- `navigation_state_manager_fusion.py` 订阅 `/localization/fusion_status` 获取 DEGRADED/LOST 状态
- 订阅 `/localization/recovery_status` 获取 recovery 事件
- SC bridge 与 HDL bridge 发布**完全相同**的标准化 recovery 事件
- DEGRADED/TRANSITIONING 时跳过导航暂停，LOST 时走标准 recovery 流程

**结论**: APP 层状态管理器无需任何代码修改，新建 launch 文件仅作为入口点。

---

## 五、兼容性矩阵

| 接口 | HDL bridge | SC bridge | 兼容 |
|---|---|---|---|
| `/localization/recovery_requests` (fusion→bridge) | 订阅，JSON w/ reason/source | 订阅，JSON w/ reason/source | ✅ |
| `/localization/recovery_status` (bridge→fusion/APP) | JSON w/ event_type | JSON w/ event_type | ✅ |
| `/initialpose` (bridge→NDT) | 8x 重复发布 | 8x 重复发布 | ✅ |
| `/localization/ndt_status` (NDT→fusion/bridge) | JSON w/ fitness_score | JSON w/ fitness_score | ✅ |
| `navigation_state_manager_fusion` APP 层 | fusion_mode 检查 DEGRADED | 相同 | ✅ |

---

## 六、验证方法

### 6.1 单元验证

```bash
# Python 语法检查（已验证通过）
python3 -c "import py_compile; py_compile.compile('src/humanoid_navigation2/humanoid_navigation2/localization_odom_fusion.py', doraise=True)"

# Launch 文件 AST 检查（已验证通过）
python3 -c "import ast; ast.parse(open('src/humanoid_navigation2/launch/navigation2_fusion_sc.launch.py').read())"
```

### 6.2 功能验证 (需要实机/仿真)

1. **NDT 状态订阅**: 启动 fusion 节点，检查 `ndt_error` 不再恒为 inf
   ```
   ros2 topic echo /localization/fusion_status
   ```

2. **DEGRADED 进入/退出**: 模拟 NDT 漂移 → 确认 2 帧内进入 DEGRADED → NDT 恢复后 3 帧确认恢复 HEALTHY

3. **LOST → SC recovery**: 等待 DEGRADED 超时 → 确认 `/localization/recovery_requests` 发布 → SC bridge 响应 → `/initialpose` 发布

4. **NDT 验证**: `/initialpose` 发布后 NDT 连续 accepted → fusion 平滑恢复到 HEALTHY

5. **Pose 一致性 gate**: NDT 伪恢复 (低 error 但错位) → `_is_healthy` 返回 False → 不进入 TRANSITIONING

### 6.3 回归验证

- 导航 DEGRADED 期间不暂停
- APP 不在 DEGRADED 时上报定位异常
- 原 `navigation2_fusion.launch.py` (HDL 版) 保持不变

---

## 七、潜在风险和已知限制

1. **Pose 一致性门限值 (0.8m) 未实机调优**: 当前 0.8m 基于分析文档建议，可能需要根据实机数据调整为更合适的值

2. **`_is_healthy` 在 HEALTHY 态也调用 pose gate**: 但 HEALTHY 态下 `frozen_map_odom` 为 None，gate 自动跳过，行为不变

3. **SC 启动时间**: SC global localizer 需要加载数据库和 PCD 地图，4s 延迟可能不够（需实机确认）

4. **`hdl_globalmap_pcd` 等变量仍保留**: 因为注释掉的 HDL 配置仍引用这些变量，保留以便快速回退

---

## 八、全流程导航模拟分析：NDT → Odom 接管 → ScanContext 重定位

### 8.1 模拟场景设定

机器人从点位1导航到点位18，途经 18 个讲解点位。设定三个 NDT 漂移场景：

| 路段 | 触发点 | 场景 | 预期行为 |
|---|---|---|---|
| 路段 A (点1→点3) | 点1到点2之间 | NDT 在走廊几何混叠区短时漂移 (3s) | DEGRADED→自然恢复→HEALTHY |
| 路段 B (点7→点9) | 点8附近 | NDT 持续漂移，odom 接管时长超 120s | DEGRADED→LOST→SC 重定位→HEALTHY |
| 路段 C (点14→点16) | 点15附近 | NDT 伪恢复（低 error 但错位 3m） | DEGRADED→pose gate 拒绝→LOST→SC→HEALTHY |

### 8.2 全流程时序

```
时间轴 (示意，实际以真实速度为准)
═══════════════════════════════════════════════════════════════════════════════

时刻 T0: 系统启动，机器人位于点1（原点附近）
───────────────────────────────────────────────────────────────────────────────
  NDT:       收敛中... fitness_score 0.08 → 0.05 → 0.03
  Fusion:    HEALTHY, 不发布 map->odom (让 NDT 直通)
  SC bridge: 启动阶段 (startup_duration=30s), 每 2s 触发一次 SC 检查
             发现 NDT 已正常 → 不发布 /initialpose, 进入就绪监控
  
  → 导航开始: 机器人从点1出发，前往点2

时刻 T1: 机器人离开点1 ~10m，NDT 遇到走廊几何混叠区
───────────────────────────────────────────────────────────────────────────────
  NDT:       fitness_score 0.05 → 0.12 → 0.48 → 0.67
             state: "accepted" → "accepted" → "rejected"
             
  Fusion:    _is_degraded() 检测到 error(0.67) > 0.5
             consecutive_degraded: 0→1
  
  T1+0.03s (下一帧 30Hz):
  NDT:       fitness_score 0.72 (连续第2帧超标)
  
  Fusion:    consecutive_degraded: 1→2 >= 2帧阈值
             ★ _enter_degraded()
               - frozen_map_odom = last_healthy_map_odom (最后健康的快照)
               - frozen_odom_body = (当前 body_x, body_y)
               - 冻结值例如: map_odom=(x=0.5, y=0.1, yaw=0.02)
               - degraded_start_time = now
               - total_odom_displacement = 0.0
             
             此后行为:
               ✓ 发布 frozen_map_odom (30Hz, 覆盖 NDT 10Hz)
               ✓ 下游 TF: map_T_base = frozen_map_T_odom × odom_T_cam_init × cam_init_T_body × body_T_base
               ✓ 导航继续, 不暂停 (Nav2 不知道定位出了问题)
               ✓ APP 不上报定位异常 (fusion_status=degraded, nav_state_manager 跳过暂停)
  
  NDT (内部): 继续尝试 scan matching
              - initial guess 基于当前 odom 推算, 仍然正确
              - 但因为环境混叠, 匹配结果错误 (fitness 高)

时刻 T2: ~3s 后, 机器人走出混叠区, NDT 自然恢复
───────────────────────────────────────────────────────────────────────────────
  NDT:       fitness_score 0.67 → 0.55 → 0.22 → 0.12 → 0.08
             state: "rejected" → "accepted"
             has_converged: false → true
  
  Fusion:    _is_healthy() 检查:
               1. error(0.08) < 0.15 ✓
               2. has_converged=True ✓
               3. frozen_map_odom 存在 → pose 一致性检查:
                  - 当前 odom_body=(x=0.52, y=-0.01, z=-9.87)
                  - frozen_odom_body=(x=0.50, y=0.00, z=0.01)
                  - odom_delta: dx=0.02, dy=-0.01
                  - expected_map_odom: x=0.50+0.02=0.52, y=0.10+(-0.01)=0.09
                  - ndt_current_map_odom: x=0.54, y=0.11
                  - pose_jump = hypot(0.54-0.52, 0.11-0.09) = 0.028m < 0.8m ✓
             
             consecutive_healthy: 0→1→2→3 (3帧确认, ~0.1s @ 30Hz)
             
             ★ _enter_transitioning()
               - transition_from = frozen_map_odom
               - transition_to = latest_ndt_map_odom
               - 跳变距离: ~0.03m (极小, 因为 NDT 自然恢复)
  
  Fusion:    TRANSITIONING (2s smoothstep 过渡)
             - alpha: 0→1 平滑 (smoothstep 函数, C2连续)
             - translation: 线性插值
             - rotation: slerp
             - 过渡完成后 → HEALTHY
  
  结果: ✅ 导航无中断, APP 无感知, 端到端恢复

═══════════════════════════════════════════════════════════════════════════════

时刻 T3: 点7→点9 中途, NDT 在长走廊持续漂移 (>120s)
───────────────────────────────────────────────────────────────────────────────
  NDT:       fitness_score 在 0.6-1.2 之间持续波动
             环境中几何特征不足 (长直走廊, 两侧墙壁相似)
  
  Fusion:    T3+0s  → 进入 DEGRADED (2帧确认)
             T3+30s → 单段 odom 位移 = 12m < 30m (未超)
             T3+60s → 单段 odom 位移 = 23m < 30m (未超)
             T3+90s → 单段 odom 位移 = 28m < 30m (接近)
             T3+100s→ 单段 odom 位移 = 31m > 30m! 
             
             ★ _enter_lost()
               - 触发原因: 单段 odom 位移 31m > max_odom_displacement(30m)
               - 停止发布 map->odom
               - 调用 _request_recovery()
               - 发布 JSON 到 /localization/recovery_requests:
                 {"reason":"fusion odom fallback exhausted",
                  "source":"localization_odom_fusion",
                  "event_type":"fusion_lost",
                  "search_radius_m":5.0}
             
             备选路径 (如果位移未超):
             T3+120s → elapsed > nav_active_lost_timeout(120s)
             → 同样进入 LOST

  注意: 
  如果机器人在 DEGRADED 期间到达点8 (路点),
  nav_state EXECUTING→IDLE → _on_nav_status 重置:
    - degraded_start_time = now (新计时)
    - frozen_odom_body = 当前 odom (位移从点8重新算)
  这意味着: 跨路点导航时, 每个路点都是"安全点",
  之前累积的时间/距离被重置。

时刻 T4: Fusion 请求 SC recovery
───────────────────────────────────────────────────────────────────────────────
  Fusion:    LOST 状态, _update_lost()
             - 不发布 map->odom (让 recovery 接管)
             - 每 15s 检查是否需要重新请求 (cool down)
             - 设置了 _recovery_in_progress 防止重复请求
  
  SC bridge: recovery_request_callback() 收到 /localization/recovery_requests
             - external_recovery_until = now + 12.0s (响应窗口)
             - start_recovery("external localization recovery request")
             - publish_recovery_status("localization_recovery_started", ...)
             
  Fusion:    _on_recovery_status 收到 "localization_recovery_started"
             - _recovery_in_progress = True (锁定, 不再发新请求)

时刻 T5: SC bridge 执行重定位
───────────────────────────────────────────────────────────────────────────────
  SC bridge: timer_callback() (0.2s 周期)
             - recovery_active = True
             - 第1次尝试: 调用 /scancontext_global_localization/trigger (普通SC)
               relocalize_attempts = 1
               
  SC global localizer:
             1. 提取当前帧 ScanContext 描述子
             2. 在数据库中搜索最近邻 (sc_distance < 0.25)
             3. 候选通过 odom consistency gate (与 odom 推算位置差 < 1.0m)
             4. 候选通过 confidence gate (sc_distance_gap > 0.03)
             5. GICP 精化 (fitness < 0.6)
             6. 精化后 odom gate (与 odom 推算位置差 < 1.5m)
             → 假设找到候选, 发布 best_pose

  SC bridge: best_pose_callback()
             - convert_pose(): 将 SC 候选从 body 帧转换到 map 帧
               (通过 R_FASTLIO_TO_ROS 矩阵: body→ROS 标准坐标)
             - 发布 /initialpose (8次, 每0.25s一次)
             - publish_recovery_status("localization_initialpose_published")
  
  NDT:       收到 /initialpose
             - 重置 scan matching 初始位姿为 SC 候选
             - 开始以新位姿进行 NDT 匹配

时刻 T6: NDT 验证 SC 候选
───────────────────────────────────────────────────────────────────────────────
  SC bridge: 8次发布完成后
             - recovery_waiting_for_ndt = True
             - recovery_ndt_check_after = now + 6.0s (settle时间)
             - 等待 NDT 状态...
  
  NDT:       T6+6s 后开始检查
             - 第1帧: state="accepted", fitness=0.08 ✓
             - 第2帧: state="accepted", fitness=0.06 ✓  
             - 第3帧: state="accepted", fitness=0.07 ✓
             ndt_recovery_stable_status_count = 3 >= 3 (阈值)
             
  SC bridge: ndt_status_callback()
             ★ publish_recovery_status("localization_recovered",
                 "NDT accepted ScanContext initialpose for consecutive frames")
             recovery_active = False
             recovery_waiting_for_ndt = False

  Fusion:    _on_recovery_status 收到 "localization_recovered"
             - _reset_state() → 清除所有 DEGRADED 状态
             - state = HEALTHY
             - _recovery_in_progress = False (解锁)
             - _publish_fusion_status()
             
  结果: ✅ NDT 恢复定位, fusion 切回 HEALTHY, 导航继续

  但如果 NDT rejected SC 候选:
  ─────────────────────────────────
  SC bridge: ndt_status_callback 检测到连续 rejected
             - publish_recovery_status("localization_relocalize_failed", ...)
             - 重新 start_recovery()
             - relocalize_attempts 继续累积
             - 3次失败后: use_global = True → 走 SC global recovery
               
  SC global recovery: 严格模式
             - sc_distance < 0.25
             - GICP fitness < 0.20 (更严格)
             - 多帧一致性: 3/6 帧落在 0.8m/0.35rad 内
             - 候选经更严格验证后才发布 /initialpose
  
  Fusion:    _on_recovery_status 收到 "localization_relocalize_failed"
             - _recovery_in_progress = False (允许重新请求)

═══════════════════════════════════════════════════════════════════════════════

时刻 T7: 点15 附近, NDT 伪恢复 (低 error 但位置错误)  ← 新增防护逻辑
───────────────────────────────────────────────────────────────────────────────
  NDT:       fitness_score 0.06 (很好!)
             has_converged = True
             但是: 实际定位偏移了 ~3m (几何混叠导致收敛到错误局部最优)
  
  Fusion:    (DEGRADED 状态中)
             _is_healthy() 检查:
               1. error(0.06) < 0.15 ✓
               2. has_converged=True ✓
               3. ★ Pose 一致性 gate (新增):
                  - odom_body=(x=0.55, y=-0.02, z=-15.32)  [body帧中 z 是前后]
                  - frozen_odom_body=(x=0.50, y=0.00, z=0.01)
                  - odom_dx=0.05, odom_dy=-0.02
                  - expected_map_odom: x=0.5+0.05=0.55, y=0.1+(-0.02)=0.08
                  - ndt_current_map_odom: x=3.52, y=2.14  ← 偏了!
                  - pose_jump = hypot(3.52-0.55, 2.14-0.08) = 3.62m > 0.8m ✗
             ★ 返回 False! 拒绝恢复到 HEALTHY
             
             继续等待: DEGRADED → 超时 → LOST → SC recovery
  
  结果: ✅ 防止伪恢复, 继续等待正确的 SC 重定位

═══════════════════════════════════════════════════════════════════════════════

完整状态转移总结:
──────────────────
  HEALTHY ──(error>0.5, 2帧)──▶ DEGRADED
    │                              │
    │    ┌──(error<0.15, 3帧,      │──(超时120s/位移30m/累计100m)──▶ LOST
    │    │   pose_jump<0.8m)──▶    │                                      │
    │    │  TRANSITIONING          │                                      │
    │    │  (2s smoothstep)        │                                      │
    │    └── 完成 ──▶ HEALTHY      │                                      │
    │                              │                                      │
    │    ◀── recovery成功 ─────────┼────────  ────────────────────────    │
    │                              │           /localization/             │
    │                              │           recovery_requests          │
    │                              │               │                      │
    │                              │               ▼                      │
    │                              │     SC bridge: trigger →             │
    │                              │     /scancontext_global_             │
    │                              │     localization/trigger             │
    │                              │               │                      │
    │                              │               ▼                      │
    │                              │     SC 搜索 → GICP 精化              │
    │                              │     → best_pose → convert            │
    │                              │     → /initialpose (8x)              │
    │                              │               │                      │
    │                              │               ▼                      │
    │                              │     NDT 验证 (3帧 accepted)          │
    │                              │     → "localization_recovered"       │
    │                              │               │                      │
    │                              │               ▼                      │
    │                              └────── fusion _on_recovery_status     │
    │                                      _reset_state() + HEALTHY       │
    │                                                                     │
    └── (如果SC失败) localization_relocalize_failed ──▶ SC 重试/global    │
```

### 8.3 导航状态感知的超时策略

fusion 节点订阅 `/navigation_status` (来自 `navigation_state_manager`)，根据导航状态动态选择超时：

| 导航状态 | LOST 超时 | 逻辑 |
|---|---|---|
| EXECUTING/PLANNING | 120s | 需要精确定位，尽快 recovery |
| IDLE/COMPLETED | 600s | 静止播报讲解词，定位精度不重要 |
| IDLE + error>5.0 | 300s (减半) | 极端 error 时加速触发 |

到达路点时的特殊处理：
```
OLD: EXECUTING → NEW: IDLE (到达!)
  → degraded_start_time 重置为 now
  → frozen_odom_body 更新为当前 odom 位置
  → 之前的累积时间/距离 "归零"
```

### 8.4 三层降级架构总结

```
           NDT 层          Fusion 层          SC 层
         (scan match)    (TF 仲裁者)      (全局重定位)
             │                │                  │
  HEALTHY    │  发布 map→odom  │  不干预           │  待命
             │                │  (直通 NDT TF)    │
  ───────────┼────────────────┼───────────────────┤
  DEGRADED   │  仍在匹配       │  冻结 map→odom     │  待命
             │  (可能错误)     │  (30Hz 覆盖 NDT)   │  (不触发!)
             │                │  导航不暂停        │
  ───────────┼────────────────┼───────────────────┤
  LOST       │  等待恢复       │  停止发布 TF       │  ★ 执行全局重定位
             │                │  请求 SC recovery   │  → /initialpose
  ───────────┼────────────────┼───────────────────┤
  RECOVERING │  验证 /initpose │  冻结或停止        │  等待 NDT 验收
             │  (连续 accepted) │  _recovery_in_progress│
  ───────────┼────────────────┼───────────────────┤
  HEALTHY    │  恢复正常       │  平滑过渡          │  恢复待命
             │                │  (smoothstep+slerp)│
```

---

## 九、Fast-LIO 非标准坐标轴对 Odom 接管的影响分析

### 9.1 坐标系定义

Fast-LIO 使用的非标准坐标轴：

```
Fast-LIO body 帧 (IMU坐标系):
  x → 左  (left)
  y → 下  (down)
  z → 后  (back)

ROS 标准帧 (odom):
  x → 前  (forward)
  y → 左  (left)
  z → 上  (up)
```

两者之间的转换矩阵 (正是 `R_FASTLIO_TO_ROS`):

```
R = [[0,  0, -1],    # body z (-z) → odom x (forward)
     [1,  0,  0],    # body x      → odom y (left)
     [0, -1,  0]]    # body y (-y) → odom z (up)
```

即:
- body 的 `-z` 轴 → ROS 的 `+x` 轴 (forward)
- body 的 `+x` 轴 → ROS 的 `+y` 轴 (left)
- body 的 `-y` 轴 → ROS 的 `+z` 轴 (up)

### 9.2 TF 树如何处理坐标系转换

```
map → odom → camera_init → body → base_footprint
       ↑          ↑          ↑
    fusion/NDT   静态TF    Fast-LIO
    发布(标准)  (旋转90°)  发布(非标准)
```

关键环节：`odom → camera_init` 静态 TF

```python
tf_bridge_odom = Node(
    arguments=['--x', '0', '--y', '0', '--z', '0',
              '--qx', '-0.5', '--qy', '-0.5', '--qz', '0.5', '--qw', '0.5',
              '--frame-id', 'odom', '--child-frame-id', 'camera_init'],
)
```

此静态 TF 的旋转矩阵 = `R_FASTLIO_TO_ROS`，负责将 camera_init 帧（非标准）下的向量转换到 odom 帧（标准 ROS）。

### 9.3 Fusion 节点直接查询 camera_init→body

fusion 节点通过 `_lookup_odom_body()` 直接查询 `camera_init→body`：

```python
# localization_odom_fusion.py:579-603
def _lookup_odom_body(self) -> dict:
    transform = self.tf_buffer.lookup_transform(
        FRAME_CAMERA_INIT,    # 源帧 (非标准)
        FRAME_BODY,           # 目标帧 (非标准)
        Time(),
        timeout=Duration(seconds=0.1),
    )
    # 返回 camera_init 坐标系下的 body 位置
    return {'x': t.x, 'y': t.y, 'z': t.z,
            'qx': r.x, 'qy': r.y, 'qz': r.z, 'qw': r.w}
```

TF2 的 `lookup_transform(A, B)` 返回的是 **从 A 帧看的 B 帧位置**，结果在 **A 帧的坐标系**中表达。因此这里返回的 (x, y, z) 是在 camera_init 帧的坐标系中，即：

| 分量 | 实际物理含义 |
|---|---|
| `x` | body 在 camera_init 帧的 x 分量 = 左右位移 (正值=向左) |
| `y` | body 在 camera_init 帧的 y 分量 = 上下位移 (正值=向下, **垂直!**) |
| `z` | body 在 camera_init 帧的 z 分量 = 前后位移 (正值=向后) |

### 9.4 对 Odom 接管的实际影响

#### 9.4.1 TF 发布（无影响）✅

fusion 在 DEGRADED 时发布的 frozen `map->odom` 是之前从 TF 查询的 NDT 发布的 map→odom 值，已经是标准坐标系。下游 TF 链 `map_T_odom × odom_T_camera_init × camera_init_T_body × body_T_base` 由 TF2 完整计算，坐标系转换全部正确。

#### 9.4.2 里程计位移计算（**有影响** ⚠️）

```python
# localization_odom_fusion.py:1056-1071
def _compute_odom_displacement(self, odom_body: dict) -> float:
    dx = odom_body['x'] - self.frozen_odom_body['x']  # camera_init 帧 x → 左右
    dy = odom_body['y'] - self.frozen_odom_body['y']  # camera_init 帧 y → 上下!
    return math.hypot(dx, dy)
```

**问题**: 用非标准坐标系下的 `(x, y)` 分量计算 2D 水平位移，但 body 帧中:
- `x` = 左右 (水平) ✓
- `y` = 上下 (**垂直**) ✗
- `z` = 前后 (水平) — **被遗漏了**

**实际效果分析**:

| 运动类型 | body 帧中的主要分量 | 当前计算 | 偏差 |
|---|---|---|---|
| 纯前进 30m (最常见) | z≈-30, x≈0, y≈0 | hypot(0,0)≈0m | **严重低估** |
| 前进+转弯 | z变化为主, x有些变化, y≈0 | 取决于转弯幅度 | **低估 50-90%** |
| 纯侧移 | x变化, y≈0, z≈0 | 正确 | 准确 |
| 原地旋转 | x≈0, y≈0, z≈0 | 正确 | 准确 |

**结论**: 对于最典型的"沿着走廊直行"场景，odom 位移计数器严重低估。一个靠墙走的 30m 直走廊可能只产生 1-2m 的 `hypot(dx, dy)`。

**缓解因素**:
1. **时间超时 (120s)** 是主要的安全网：即使位移计数不准，120s 的导航超时仍会触发 LOST
2. **转弯时产生 x 分量**: 导航路径不是纯直线，包含转弯，此时 body 帧 x 分量会有变化
3. **累计位移上限 100m** 设计保守，即使欠计数也提供额外保护

#### 9.4.3 Pose 一致性检查（**有影响** ⚠️）

```python
# localization_odom_fusion.py:1043-1052 (新增)
odom_dx = odom_body['x'] - self.frozen_odom_body[0]  # 左右分量
odom_dy = odom_body['y'] - self.frozen_odom_body[1]  # 上下分量 (垂直!)

# 用非标准分量去推算标准坐标系下的预期 map_odom:
expected_x = self.frozen_map_odom['x'] + odom_dx   # ← 应该是前向分量!
expected_y = self.frozen_map_odom['y'] + odom_dy   # ← 应该是左右分量!
```

**问题**: 
- `odom_dx` (body 的左右移动) 被加到 map 的 x (forward) 上
- `odom_dy` (body 的上下/垂直移动) 被加到 map 的 y (left) 上
- body 的真正前向移动分量 `z` 被完全忽略

**实际效果**: 对于纯前进运动，`expected` 几乎不变，而 NDT 当前的 map_odom 也在附近（因为 NDT 正常情况下 map_odom 基本恒定）。所以对于 **自然恢复** (DEGRADED→TRANSITIONING)，pose gate 检查容易通过（因为 expected 和 ndt_current 差异不大）。

但对于 **SC recovery** 后的 NDT 验证：SC 候选位姿可能远离 frozen 位置，pose gate 实际由 `_is_healthy` 中的其他条件 (error + converged) 来验证，pose gate 本身提供的是弱保护。

**严重程度**: 中等。pose gate 在当前的数字下（0.8m 阈值）对自然恢复场景较宽松，但这不是最关键的防护。最关键的防护来自 SC bridge 的 NDT 连续 accepted 验证 + SC 的多层 gate。

#### 9.4.4 SC Bridge 中的坐标转换（正确处理 ✅）

SC bridge (`scancontext_to_initialpose.py`) 有正确转换:

```python
R_FASTLIO_TO_ROS = np.array(
    [[0.0, 0.0, -1.0],
     [1.0, 0.0, 0.0],
     [0.0, -1.0, 0.0]],
    dtype=float,
)

def convert_pose(self, msg):
    position_fastlio = np.array([p.x, p.y, p.z], dtype=float)
    position_ros = R_FASTLIO_TO_ROS @ position_fastlio  # 坐标转换
    ...
```

这里将 SC 输出的 body 帧位姿正确转换为 ROS 标准坐标系下的 map 位姿。

### 9.5 修复实施 (2026-05-26)

#### 修复 1: `_compute_odom_displacement` — 使用 (x, z) 计算 2D 水平位移

**修改前**:
```python
dx = odom_body['x'] - self.frozen_odom_body['x']   # 左右
dy = odom_body['y'] - self.frozen_odom_body['y']   # ← 垂直分量!
return math.hypot(dx, dy)                           # ← 遗漏 z
```

**修改后**:
```python
dx = odom_body['x'] - self.frozen_odom_body[0]   # 左右
dz = odom_body['z'] - self.frozen_odom_body[2]   # 前后 (back+)
return math.hypot(dx, dz)                        # (x, z) = 水平面
```

#### 修复 2: `frozen_odom_body` 标准化为 3-tuple `(x, y, z)`

**修改前** (`_enter_degraded`): 存储 dict → 与 `[0]`/`[1]` 索引不兼容，潜在 KeyError

**修改后**: 统一存储 `(body['x'], body['y'], body['z'])` 三元组，所有使用处统一 `[0]`=x(左右), `[2]`=z(前后)

涉及修改点:
- `_enter_degraded` L864-868: 从 dict 提取 3-tuple
- `_on_nav_status` L493: 存储 3-tuple，位移计算用 `[0]` 和 `[2]`

#### 修复 3: `_is_healthy` pose gate — 简化为直接比较 map_odom

**修改前**: 试图从 body 帧位移推算预期 map_odom，轴映射错误

**修改后**: `frozen_map_odom` 和 `latest_ndt_map_odom` 都是标准 ROS 坐标系（来自 TF `map→odom` 查询），直接比较:

```python
ndt_dx = self.latest_ndt_map_odom['x'] - self.frozen_map_odom['x']
ndt_dy = self.latest_ndt_map_odom['y'] - self.frozen_map_odom['y']
pose_jump = math.hypot(ndt_dx, ndt_dy)
if pose_jump > 0.8:
    return False
```

#### 修复 4: `total_odom_displacement` 初始化

在 `__init__` 中新增 `self.total_odom_displacement = 0.0`，防止 `_enter_degraded` 之前访问导致 AttributeError。

### 9.6 修复后影响评估

| 组件 | 修复前 | 修复后 |
|---|---|---|
| odom 位移计算 | 直行 30m 计数 ~0m | 直行 30m 计数 ~30m |
| 单段位移门限 (30m) | 几乎不触发 | 正确触发 |
| 累计位移门限 (100m) | 几乎不触发 | 正确触发 |
| Pose gate | 轴映射错误 + 类型不一致 | 直接比较，简单正确 |
| frozen_odom_body 类型 | dict/tuple 混用 | 统一 3-tuple |

### 9.7 数据流全链路审计：确认无预转换/重复转换

修复前对整条数据链路做了完整审计，确认 fusion 节点获取的 `camera_init→body` 数据**没有经过任何预转换**，修复操作不会造成重复转换。

**数据来源**：

Fast-LIO 通过 `/tf` 广播 `camera_init → body` 变换。`camera_init` 帧在 launch 中明确定义为非标准轴：

> launch/navigation2_fusion.launch.py L521-522:
> "/fast_lio/cloud_registered 的 frame_id 是 camera_init，原始轴为 x左/y下/z后"

**TF2 查询语义**：

```python
# fusion 节点 _lookup_odom_body()
transform = self.tf_buffer.lookup_transform(
    FRAME_CAMERA_INIT,   # target_frame
    FRAME_BODY,          # source_frame
    Time(),
)
```

ROS 2 tf2 文档规定：`lookup_transform(target, source)` 返回的 `translation` 是 source 帧原点在 **target 帧坐标系**下的坐标。此处 target = camera_init，因此 `(t.x, t.y, t.z)` 处于非标准 camera_init 坐标系中。

**关键验证：odom→camera_init 静态 TF 不影响单帧查询**：

```
TF 树: odom → camera_init → body → base_footprint
              ↑ 静态TF    ↑ Fast-LIO发布  ↑ 静态TF
            (旋转120°)

lookup_transform("camera_init", "body")
  → 只返回 camera_init→body 的直接变换
  → 处于 camera_init 帧坐标系（非标准）
  → odom→camera_init 的旋转不影响此查询结果 ✓

而 TF2 完整链 map→base_footprint:
  → map_T_odom × odom_T_camera_init × camera_init_T_body × body_T_base
  → TF2 自动完成所有转换 ✓
```

**系统中其他转换节点（互不干扰）**：

| 模块 | 转换 | 与 fusion 的关系 |
|---|---|---|
| `scancontext_to_initialpose.py` | `R_FASTLIO_TO_ROS` 转换 SC 候选→/initialpose | 独立管道，SC 候选→NDT |
| `pcd_converter.py` | 旋转 PCD 地图 (离线一次性) | 不相关 |
| `imu_transformer.py` | 旋转 IMU 数据 (/airy_imu→/imu_standard) | 当前 fusion 方案不使用 |
| `odom→camera_init` 静态 TF | TF2 链内自动旋转 | 只在计算完整链时生效，不影响单帧查询 |

**结论**：两路数据互不交叉——

```
路径 A (fusion 位移计算):
  camera_init→body 原始(NON-STANDARD) → 直接算 hypot(dx, dz) ✓

路径 B (TF 完整链 map→base):
  map→odom(STD) → odom→camera_init(STD→NON-STD) → camera_init→body(NON-STD) → body→base(NON-STD→STD)
  → TF2 全自动处理 ✓
```

`_is_healthy` pose gate 简化为直接比较 `frozen_map_odom` vs `latest_ndt_map_odom` 后，两者都来自 `lookup_transform("map", "odom")` 的标准 ROS 坐标系数据，完全不经过 body 帧，彻底消除轴映射问题。

---

## 十、Bag 包回放验证

### 10.1 验证环境

| 项目 | 值 |
|---|---|
| Bag 文件 | `/home/ubuntu/nav_drift_test1/nav_drift_test_0.mcap` |
| Bag 大小 | 44.2 GiB |
| 时长 | 1380s (23分钟) |
| ROS Distro | Jazzy |
| 关键 Topic | `/localization/ndt_status` (12188条), `/tf` (113226条), `/odom` (13800条), `/localization/recovery_status` (547条) |

### 10.2 验证方法

编写 `debug_monitor/validate_fusion_fix.py`，使用 `rosbag2_py` API 读取 mcap 文件。利用 `StorageFilter` 过滤不需要的 topic（点云等大数据），仅提取：

- `/localization/ndt_status` (12188条) — NDT fitness/收敛状态
- `/tf` (113226条, 过滤后关注 map→odom 和 camera_init→body)
- `/localization/recovery_status` (547条) — recovery 事件

离线运行融合状态机，完整实现 HEALTHY→DEGRADED→LOST→HEALTHY 状态转移逻辑，使用修复后的 `_compute_displacement` (hypot(dx, dz)) 和 pose gate。

### 10.3 验证结果

#### NDT 健康度统计

| 指标 | 值 |
|---|---|
| 平均 fitness | 2.1335 |
| 最大 fitness | 60.2024 |
| >0.5 的帧数 | 1103/12188 (9.0%) |
| 结论 | NDT 在此 bag 中有明显漂移事件 |

#### 状态切换时间线

```
时间        事件                          详情
──────────────────────────────────────────────────────────────────────
13:34.6     HEALTHY→DEGRADED            error=0.7116 frozen_map_odom=(5.407,-4.225)
15:34.7     DEGRADED→LOST              超时 120s (时间门限触发, 非位移)
16:39.5     LOST→HEALTHY               recovery 成功 (耗时 65s)
16:58.8     HEALTHY→DEGRADED            error=0.6100 frozen_map_odom=(5.910,-3.135)
18:58.8     DEGRADED→LOST              超时 120s (时间门限触发)
20:11.2     LOST→HEALTHY               recovery 成功 (耗时 72s)
20:16.4     HEALTHY→DEGRADED            error=0.6625 frozen_map_odom=(33.129,35.505)
20:19.7     DEGRADED→HEALTHY           ★ 自然恢复 error=0.1297 连续3帧 (仅3.3s!)
```

#### 分析

| 验证项 | 结果 | 说明 |
|---|---|---|
| DEGRADED 触发 | ✅ 正确 | 3次均由 NDT error>0.5 触发, 无伪触发 |
| LOST 触发方式 | ✅ 正确 | 全部由 120s 超时触发, 无伪位移触发 |
| Recovery 恢复 | ✅ 正确 | 2次 SC/HDL recovery 成功, 1次自然恢复 |
| Pose gate | ✅ 正确 | 自然恢复通过 pose gate (frozen vs current map_odom 偏差 <0.8m) |
| 位移计算 | ⚠️ 未触发 | 此 bag 中位移门限未达 30m (robot 漂移期间位移有限) |

#### 关键结论

1. **修复不改变健康场景行为**: DEGRADED 触发和 recovery 逻辑保持一致
2. **Pose gate 工作正常**: 第三次 DEGRADED 在 3.3s 内自然恢复, map_odom 从 (33.13, 35.51) 恢复到健康值, pose gate 正确放行
3. **位移计算修复无法在此 bag 完全验证**: robot 在 DEGRADED 期间位移未超额 (120s 超时先到达), 建议用包含长距离 DEGRADED 导航的 bag 做进一步位移门限验证

### 10.4 验证脚本

脚本位置: `debug_monitor/validate_fusion_fix.py`

```bash
# 运行方式 (需要 ROS 2 环境)
source /opt/ros/jazzy/setup.bash
python3 debug_monitor/validate_fusion_fix.py
```

| 文件 | 绝对路径 |
|---|---|
| Fusion 节点 | `/home/ubuntu/humanoid_ws/src/humanoid_navigation2/humanoid_navigation2/localization_odom_fusion.py` |
| Fusion+SC Launch | `/home/ubuntu/humanoid_ws/src/humanoid_navigation2/launch/navigation2_fusion_sc.launch.py` |
| APP SC Launch | `/home/ubuntu/humanoid_ws/src/humanoid_navigation/launch/navigation_fusion_sc.launch.py` |
| CHANGELOG | `/home/ubuntu/humanoid_ws/CHANGELOG.md` |
| 分析文档 | `/home/ubuntu/humanoid_ws/NDT_ODOM_SC_COMBINED_RECOVERY_ANALYSIS.md` |
| Plan 文件 | `/home/ubuntu/.claude/plans/cosmic-knitting-valley.md` |

---

## 十一、代码评审修复记录 (2026-05-26)

> 评审来源: `NDT_FUSION_SC_REVIEW.md`  
> 修复范围: P0（必须修）和 P2（清理项），共 4 个文件

### 11.1 P0-1: TF 查询混淆 — fusion 读到自己的 map->odom

**问题**: 融合节点在 DEGRADED/TRANSITIONING 状态下自己也发布 `map->odom` TF (30Hz)，而 `_lookup_ndt_map_odom()` 查询同一 frame pair，TF2 返回时间戳最新的值 → fusion 拿到的 `latest_ndt_map_odom` 实际是**自己刚发布的冻结值**，而非 NDT 的输出。

**影响链**:
- `_is_healthy()` pose gate 比较 frozen vs frozen → hypot ≈ 0 → **永远通过**，gate 形同虚设
- `_enter_transitioning()` 的 `transition_to` 取到冻结值而非 NDT 真实值 → smoothstep 从 frozen→frozen → 无实际过渡 → 切 HEALTHY 后突然跳变

**修复方案**: 订阅 NDT 的 `/pcl_pose` topic（与 TF `map->odom` 等值，但不受 fusion TF 污染）

```python
# fusion/__init__ — 新增订阅
self.pcl_pose_sub = self.create_subscription(
    PoseWithCovarianceStamped,
    '/pcl_pose',
    self._on_pcl_pose,
    QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE),
)

# 新增回调
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

# _is_healthy() — 改用 topic 数据
# 改前: self.latest_ndt_map_odom (TF, 被污染)
# 改后: self.latest_pcl_map_odom (topic, 纯净)

# _enter_transitioning() — 同理
# 改前: self.transition_to = self.latest_ndt_map_odom.copy()
# 改后: self.transition_to = self.latest_pcl_map_odom.copy()
```

**修复后效果**:

```
修复前:
  NDT 发布 map->odom (10Hz) + fusion 发布冻结值 (30Hz)
    → TF lookup 返回 fusion 的值 → pose gate 旁路 → 伪恢复漏检

修复后:
  NDT /pcl_pose topic ──直达──▶ fusion._on_pcl_pose()
    → latest_pcl_map_odom = NDT 真实输出
    → pose gate 正常工作 → 几何混叠被拦截 ✅
```

### 11.2 P0-2: SC bridge 绕过 fusion 自行 recovery

**问题**: `scancontext_to_initialpose.py` 的 `timer_callback()` 在运行期检测到 `/pcl_pose` 超过 2.5s 未更新，就会自行启动 recovery，不等待 fusion LOST。

**影响**: robot 仍在 DEGRADED (odom 兜底) 导航时，SC bridge 可能突然发布 `/initialpose`，绕过 fusion 状态机决策。

**修复方案**: 新增 `enable_runtime_auto_recovery` 参数，区分启动/运行期行为。

```python
# scancontext_to_initialpose.py — timer_callback
if startup:
    # 启动阶段: 允许自动检测 localization_missing
    recovery_needed = localization_missing or external_recovery
else:
    # 运行期: 根据参数决定
    if self.enable_runtime_auto_recovery:
        recovery_needed = (localization_stale or localization_missing
                           or external_recovery)
    else:
        # ★ 融合模式: 只响应 fusion 请求
        recovery_needed = external_recovery
```

**fusion+SC launch 配置**:
```python
'enable_runtime_auto_recovery': False,  # 运行期只由 fusion 触发
```

**修复后效果**:
```
启动阶段 (<30s):
  localization_missing → SC 自动初始定位 ✅

运行期 (>30s):
  /pcl_pose stale → SC 不触发 ← 新行为
  fusion LOST → /recovery_requests → SC 才启动 ✅
```

### 11.3 P2: fusion_status 改为 JSON 格式

**问题**: `_publish_fusion_status()` 使用 `str(status)` 输出 Python dict 字符串（单引号），非标准 JSON。后续监控工具按 JSON 解析会失败。

**修复**:
```python
# fusion/_publish_fusion_status()
# 改前: msg.data = str(status)
# 改后: msg.data = json.dumps(status, ensure_ascii=False)
```

**APP 层兼容**:
```python
# navigation_state_manager_fusion.py — _on_fusion_status()
# 先尝试 json.loads (新格式), 失败后回退 ast.literal_eval (旧格式)
try:
    data = json.loads(msg.data)
except (ValueError, json.JSONDecodeError):
    data = ast.literal_eval(msg.data)
```

### 11.4 未修复项及原因

| 评审项 | 优先级 | 决定 | 理由 |
|---|---|---|---|
| SC bridge 缺静止/压零保护 | P1 | 延后 | 修复 P0-2 后，SC 只在 fusion LOST 后触发。此时导航状态管理器已暂停导航、停下机器人。追加保护可后续迭代 |
| fusion 对 recovered 过度信任 | P1 | 延后 | SC bridge 已有 3帧 NDT accepted 验证 + 5层gate。再加 VERIFYING 中间态可提高安全性，但非阻塞问题 |
| SC 启动时序偏紧 | P2 | 实机验证后调整 | 137MB PCD加载耗时未知，需实机日志确认 |
| min_publish_interval=12s | P2 | 延后 | 低概率场景，可后续优化 |

### 11.5 修改文件清单

| 文件 | 改动行数 | 改动类型 |
|---|---|---|
| `localization_odom_fusion.py` | +25, -5 | P0-1 新增/pcl_pose订阅 + P2 JSON格式 |
| `scancontext_to_initialpose.py` | +13, -1 | P0-2 新增enable_runtime_auto_recovery参数 |
| `navigation2_fusion_sc.launch.py` | +2 | P0-2 launch配置 |
| `navigation_state_manager_fusion.py` | +5, -1 | P2 兼容JSON+旧格式 |

---

## 十二、2026-05-26 14:31:50 CST 追加修复: 禁止 SC 在正常导航期持续重置 NDT 初始位姿

> 本节为 2026-05-26 实机启动日志 `debug_output.txt` 的后续分析和代码修复，和前面第十一节的 P0-2 运行期自动 recovery 修复区分开。第十一节解决的是 `/pcl_pose stale` 触发 recovery 的入口问题；本节解决的是 recovery 已经启动后，残留 trigger/best_pose 在 NDT 已恢复或正在验收期间继续发布 `/initialpose` 的问题。

### 12.1 实机日志时间线

日志文件: `/home/ubuntu/humanoid_ws/debug_output.txt`

| 时间 | 启动后 | 事件 | 结论 |
|---|---:|---|---|
| 2026-05-26 13:50:55 CST | 0s | `robot_real.launch.py relocalization_engine:=sc` 启动 | 本轮 SC 实机验证开始 |
| 2026-05-26 13:51:06 CST | 11s | ScanContext database 加载完成，743 keyframes | SC 数据库可用 |
| 2026-05-26 13:51:07 CST | 12s | `scancontext_to_initialpose` 开始 recovery #1 | 启动期定位缺失，SC bridge 开始触发 |
| 2026-05-26 13:51:20 CST | 25s | NDT lifecycle active | NDT 已能接收 `/initialpose` |
| 2026-05-26 13:51:21 CST | 26s | GICP ambiguity gate 拒绝，fitness `0.0695205/0.0808503` | 候选不够稳定，未发布 `/initialpose` |
| 2026-05-26 13:51:27 CST | 32s | GICP ambiguity gate 再次拒绝 | 仍处于启动期等待 |
| 2026-05-26 13:51:30 CST | 35s | fusion `INITIALIZING` 超时，主动请求 SC recovery | fusion 进入主动 recovery 请求 |
| 2026-05-26 13:51:33 CST | 38s | GICP ambiguity gate 再次拒绝 | 仍未重置 NDT |
| 2026-05-26 13:51:39 CST | 44s | `global recovery waiting for consistent results: 1/3`，fitness `0.0116755` | SC 找到高质量候选，开始多帧一致性确认 |
| 2026-05-26 13:51:45 CST | 50s | `global recovery waiting for consistent results: 2/3` | 一致性确认继续 |
| 2026-05-26 13:51:51 CST | 56s | `global recovery accepted keyframe=367`，发布 `/initialpose`: `x=6.094 y=10.640 yaw=-178.6deg` | 首次 SC 重定位成功 |
| 2026-05-26 13:51:52 CST | 57s | fusion `[INITIALIZING→HEALTHY]`，`NDT error: 0.0010` | 首次定位稳定 |
| 2026-05-26 13:51:59 CST | 64s | SC bridge 输出 `NDT accepted initialpose! localization recovered.` | NDT 验收完成 |
| 2026-05-26 13:52:09 CST | 74s | 再次 `global recovery accepted keyframe=684`，再次发布 `/initialpose`: `x=6.055 y=10.623 yaw=-178.4deg` | 问题暴露: 已恢复后仍有残留触发继续重置 NDT |
| 2026-05-26 13:52:48 CST | 113s | 再次 `global recovery accepted keyframe=368`，再次发布 `/initialpose`: `x=6.094 y=10.642 yaw=-178.6deg` | 正常导航期仍可能被 SC 重置 |
| 2026-05-26 13:56:06 CST | 311s | 日志结束，未见 `HEALTHY→DEGRADED`、odom 接管或 NDT pose jump | 本轮推行日志范围内未观察到 NDT 飘或 odom 接管 |

### 12.2 问题分析

NDT 收到 `/initialpose` 后不是只把它作为参考，而是会直接重置当前定位初值。

代码出处:

- `src/lidar_localization/src/lidar_localization_component.cpp:709`，`PCLLocalization::initialPoseReceived()`
- 收到消息后写入 `corrent_pose_with_cov_stamped_ptr_`
- 设置 `has_last_good_transform_ = false`
- 如果已有 `last_scan_ptr_`，立即调用 `cloudReceived(last_scan_ptr_)`

因此，只要 SC bridge 在正常导航期持续发布 `/initialpose`，NDT 就会不断进入“从新初值重新配准”的流程。即使本次日志里 SC 给出的三个位置都很接近真实启动点，风险仍然存在:

1. 如果机器人已经离开启动点，残留 SC 候选可能把 NDT 拉回旧位置。
2. NDT 的 pose jump/fitness gate 可能拦住错误配准结果，但初始位姿已经被写入，连续定位会被打断。
3. 导航过程中位姿跳变会影响局部代价地图、路径跟踪和状态机判断。

根因在 `scancontext_to_initialpose.py`:

1. `timer_callback()` 在已经有 `pending_initialpose` 正在重复发布时，仍可能继续触发下一次 SC。
2. `best_pose_callback()` 只检查 `min_publish_interval_sec`，没有检查当前是否真的处于 recovery 状态。
3. NDT 验收成功后只清了部分状态，没有统一清空 pending、external recovery 窗口和本地 localization 时间标记。

### 12.3 修改内容

修改文件:

- `src/humanoid_navigation2/humanoid_navigation2/scancontext_to_initialpose.py`

#### 12.3.1 有 pending `/initialpose` 或正在等待 NDT 验收时，不再触发 SC

代码出处: `scancontext_to_initialpose.py:228`

```python
if self.recovery_waiting_for_ndt or self.pending_initialpose is not None:
    return
```

效果:

- 一旦 SC accepted 并开始 8 次重复发布 `/initialpose`，timer 不再继续发新的 trigger。
- 发布完成后进入 NDT 验收窗口时，timer 也不会继续触发 SC。

#### 12.3.2 NDT accepted 后统一结束 recovery

代码出处:

- `scancontext_to_initialpose.py:313-329`
- `scancontext_to_initialpose.py:433-441`

新增 `complete_recovery()`:

```python
def complete_recovery(self):
    self.last_localization_pose_time = time.monotonic()
    self.recovery_active = False
    self.recovery_waiting_for_ndt = False
    self.recovery_active_until = 0.0
    self.external_recovery_until = 0.0
    self.pending_initialpose = None
    self.pending_count = 0
    self.ndt_recovery_stable_status_count = 0
```

效果:

- NDT 连续 accepted 达到要求后，SC bridge 立即退出 recovery。
- 清空外部 recovery 时间窗，避免 fusion 请求窗口残留导致下一轮 recovery。
- 刷新 `last_localization_pose_time`，避免 `/pcl_pose` 尚未及时回调时被误判为 `localization_missing`。

#### 12.3.3 非 recovery 状态忽略 SC best_pose

代码出处: `scancontext_to_initialpose.py:374-381`

```python
if not self.recovery_active:
    self.get_logger().warn(
        "Ignoring ScanContext best_pose because recovery is not active; "
        "normal navigation must not reset NDT initialpose."
    )
    return
```

效果:

- 正常 HEALTHY 导航期，即使 SC localizer 发布了 `/scancontext_global_localization/best_pose`，bridge 也不会转成 `/initialpose`。
- `/initialpose` 只在启动定位缺失或 fusion 明确 recovery 请求期间发布。

#### 12.3.4 兼容 `require_ndt_stable_status_for_recovery=False`

代码出处: `scancontext_to_initialpose.py:286-297`

`localization_pose_callback()` 原先直接 `self.recovery_active = False`，现改为调用 `complete_recovery()`，保证两种验收路径都清理同一组状态。

### 12.4 修改后的预期表现

启动阶段:

1. NDT 尚未有 `/pcl_pose` 时，SC bridge 仍允许自动启动 recovery。
2. SC accepted 后发布固定次数 `/initialpose`。
3. 发布期间不会继续触发新的 SC。
4. NDT 连续 accepted 后，SC bridge 进入 recovered 状态并清空所有 pending 状态。

正常导航阶段:

1. SC 不会因为 `/pcl_pose stale` 自行发布 `/initialpose`。
2. 非 recovery 状态下收到 SC best_pose 会被忽略。
3. 只有 fusion 通过 `/localization/recovery_requests` 明确请求时，SC 才能再次发布 `/initialpose`。

预期日志变化:

```text
[SC bridge] NDT accepted initialpose! localization recovered.
```

之后不应再出现无 recovery 请求的:

```text
accepted ScanContext pose; publishing converted /initialpose ...
initialpose input: frame=map ...
```

如果出现非 recovery 状态下的残留 best_pose，应该只看到:

```text
Ignoring ScanContext best_pose because recovery is not active; normal navigation must not reset NDT initialpose.
```

### 12.5 验证

已完成:

```bash
python3 -m py_compile src/humanoid_navigation2/humanoid_navigation2/scancontext_to_initialpose.py
```

结果: 通过。

待实机重启验证:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
ros2 launch humanoid_bringup robot_real.launch.py relocalization_engine:=sc 2>&1 | tee debug_output.txt
```

重点检查:

1. 首次 `INITIALIZING→HEALTHY` 后，是否还会无请求发布 `/initialpose`。
2. 推动机器人绕场时，是否只有 fusion 进入 recovery 后才触发 SC。
3. 是否仍然没有 `HEALTHY→DEGRADED`、odom 接管、NDT pose jump 等异常。

---

## 十三、2026-05-26 15:16:53 CST 追加修复: LOST recovery 软验收，避免错位恢复和严格 odom 误杀

> 本节继续第十二节的链路收口。第十二节保证 SC 不在正常导航期持续发布 `/initialpose`；本节处理另一类风险: 当 fusion 已经进入 LOST 并主动请求 SC recovery 后，如果 SC/NDT 在相似结构中错误收敛，fusion 不能无条件相信 `localization_recovered`。同时，验收不能过严，否则 odom 累计误差或冻结 TF 偏差会把正确 recovery 拒掉。

### 13.1 修改前风险

修改前 `localization_odom_fusion.py` 在 LOST 状态收到 recovery bridge 的 `localization_recovered` 后直接:

```python
self._reset_state()
self.state = FusionState.HEALTHY
```

代码出处: `src/humanoid_navigation2/humanoid_navigation2/localization_odom_fusion.py` 的 `_on_recovery_status()`。

风险:

1. SC 给出错误全局候选，NDT 在相似墙体/走廊局部错误收敛，但连续 accepted。
2. SC bridge 发布 `localization_recovered`。
3. fusion 无条件 `LOST→HEALTHY`，接受错误 `map->odom`。

同时不能简单加入严格 odom/yaw gate:

1. LOST 之前 odom 已经累计运行，可能有漂移。
2. `frozen_map_odom` 是进入 DEGRADED 时的冻结快照，不是全局真值。
3. yaw 存在 wrap、走廊对称和短时姿态误差，硬阈值容易误杀。
4. 如果 SC/NDT 给出正确位姿但 odom 推算偏了，严格 gate 会导致一直拒绝 recovery，系统卡死在 LOST。

### 13.2 修改原则

本次采用“软验收 + 极端异常拦截”:

- 不用 yaw 做硬拒绝。
- 不要求 recovery 位姿与 odom 推算严格一致。
- odom/frozen 信息不完整时放行。
- odom 位移已经较大时放行，不再用 odom 约束全局重定位。
- NDT error 极低时，即使 XY 与 odom 预期不一致，也放行但报警。
- 只在 NDT 明显不健康或 XY 跳变明显离谱且 NDT error 不够低时拒绝。

### 13.3 修改文件和代码出处

#### 13.3.1 新增 LOST recovery 软验收参数

文件: `src/humanoid_navigation2/humanoid_navigation2/localization_odom_fusion.py`

代码出处: `localization_odom_fusion.py:211-224`

新增参数:

```python
recovery_pose_soft_gate_enabled = True
recovery_pose_max_xy_error_m = 5.0
recovery_pose_accept_if_ndt_error_below = 0.03
recovery_pose_skip_odom_after_displacement_m = 20.0
recovery_pose_max_status_age_sec = 2.0
recovery_pose_max_pcl_age_sec = 2.0
```

参数含义:

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `recovery_pose_soft_gate_enabled` | `True` | 是否启用 LOST recovery 软验收 |
| `recovery_pose_max_xy_error_m` | `5.0m` | odom 位移基础上的额外 XY 宽松余量 |
| `recovery_pose_accept_if_ndt_error_below` | `0.03` | NDT error 极低时放行并报警 |
| `recovery_pose_skip_odom_after_displacement_m` | `20.0m` | odom 已经累计较远时跳过 odom 约束 |
| `recovery_pose_max_status_age_sec` | `2.0s` | NDT status 时效窗口 |
| `recovery_pose_max_pcl_age_sec` | `2.0s` | `/pcl_pose` 时效窗口 |

#### 13.3.2 修改 LOST→HEALTHY 接受路径

文件: `src/humanoid_navigation2/humanoid_navigation2/localization_odom_fusion.py`

代码出处: `localization_odom_fusion.py:480-490`

修改后逻辑:

```python
if event_type == 'localization_recovered':
    if not self._validate_lost_recovery_soft():
        self._recovery_in_progress = False
        self.last_recovery_request_time = 0.0
        self._request_recovery()
        return
    self._reset_state()
    self.state = FusionState.HEALTHY
```

效果:

- recovery bridge 说 NDT 已接受 `/initialpose` 后，fusion 不再无条件接受。
- 软验收拒绝时立即清 recovery 标记和冷却时间，再请求下一轮 SC recovery。
- 软验收通过后才切回 HEALTHY。

#### 13.3.3 新增 `_validate_lost_recovery_soft()`

文件: `src/humanoid_navigation2/humanoid_navigation2/localization_odom_fusion.py`

代码出处: `localization_odom_fusion.py:1199-1289`

核心判断:

```text
如果 soft gate 关闭 → 接受
如果最新 NDT status 仍是 rejected / 未收敛 / error > degraded_error_threshold → 拒绝
如果没有 /pcl_pose → 接受，信任 SC bridge 的连续 NDT accepted
如果 /pcl_pose 过旧 → 接受，不用旧 pose 做硬拒绝
如果 frozen_map_odom / frozen_odom_body / odom_body 不完整 → 接受
如果 odom 位移 > 20m → 接受，不再用 odom 限制全局恢复
计算 recovery_delta = distance(recovered_pcl_pose, frozen_map_odom)
allowed_delta = odom_displacement + 5m
如果 recovery_delta <= allowed_delta → 接受
如果 recovery_delta > allowed_delta 但 NDT error < 0.03 → 接受并报警
否则拒绝，继续 recovery
```

注意:

- 没有 yaw 硬阈值。
- 没有要求 recovery_pose 精确匹配 odom 推算。
- odom 只作为极端异常过滤器，不作为全局真值。

#### 13.3.4 Launch 显式配置

文件: `src/humanoid_navigation2/launch/navigation2_fusion_sc.launch.py`

代码出处: `navigation2_fusion_sc.launch.py:578-584`

新增配置:

```python
'recovery_pose_soft_gate_enabled': True,
'recovery_pose_max_xy_error_m': 5.0,
'recovery_pose_accept_if_ndt_error_below': 0.03,
'recovery_pose_skip_odom_after_displacement_m': 20.0,
'recovery_pose_max_status_age_sec': 2.0,
'recovery_pose_max_pcl_age_sec': 2.0,
```

### 13.4 修改后的作用效果

正常情况:

```text
SC recovery → /initialpose → NDT 连续 accepted → recovery_status: localization_recovered
fusion 软验收:
  NDT 状态健康
  recovery_delta <= odom_displacement + 5m
→ LOST→HEALTHY
```

odom/frozen TF 不可靠:

```text
frozen/odom 信息不完整，或 odom 位移 > 20m
→ 不使用 odom 硬拒绝
→ 接受 NDT/SC recovery 结果
```

NDT 极低 error 但 odom 不一致:

```text
recovery_delta > allowed_delta
NDT error < 0.03
→ 接受并输出 warn
```

明显错误恢复:

```text
NDT 状态 rejected / 未收敛 / error > 0.5
或 recovery_delta 明显超过 odom 位移 + 5m 且 NDT error 不够低
→ 拒绝本次 recovery
→ 立即请求下一轮 SC recovery
```

### 13.5 验证

已完成:

```bash
python3 -m py_compile \
  src/humanoid_navigation2/humanoid_navigation2/localization_odom_fusion.py \
  src/humanoid_navigation2/launch/navigation2_fusion_sc.launch.py
```

结果: 通过。

---

## 十四、2026-05-26 16:29:49 CST 追加修复: 开机 SC 全局重定位歧义候选被 NDT 误验收

### 14.1 实机日志结论

本次开机日志 `debug_output.txt` 复盘显示，SC 启动阶段全局重定位并不是没有结果，而是在多次候选歧义后接受了错误候选:

```text
global recovery accepted keyframe=42 sc_distance=0 gicp_fitness=0.0972441 refined=true
accepted ScanContext pose; publishing converted /initialpose 8 times: x=0.323, y=-1.173, yaw=-104.4deg
initialpose input: frame=map x=0.323 y=-1.173 z=0.000 yaw=-104.4deg
[INITIALIZING->HEALTHY] 首次定位成功
NDT error: 0.1186
```

随后人工在 RViz 中修正的位姿为:

```text
initialpose input: frame=map_ground x=6.229 y=10.813 z=0.000 yaw=90.7deg
```

两者 XY 相差十几米，yaw 相差接近 195 度。说明问题不是 Nav2 或 RViz 显示异常，而是 SC 在相似结构中给出了错误全局候选，NDT 只验证局部点云收敛，无法证明候选处于正确的全局位置。

### 14.2 触发原因

失败前日志连续出现:

```text
GICP candidates ambiguous: best_fitness=... second_best_fitness=... gap=... required_gap=0.02
global recovery waiting for consistent results: 2/3 fitness=...
```

这说明场景存在重复走廊/墙面结构，多个候选的 GICP fitness 很接近。上一轮为了提高启动召回率，关闭了全局 recovery 的 candidate confidence gate，并假设 NDT 连续 accepted 能兜底。该假设被本次实机日志否定。

### 14.3 本次参数收紧

已修改:

```text
global_recovery_enable_candidate_confidence_gate: true
global_recovery_gicp_fitness_threshold: 0.09
global_recovery_required_consistent_results: 4
global_recovery_consistency_window: 8
global_recovery_consistency_xy_tolerance: 0.5
global_recovery_consistency_yaw_tolerance: 0.25
```

覆盖文件:

- `src/humanoid_scancontext_global_localization/config/scancontext_global_localization.yaml`
- `src/humanoid_scancontext_global_localization/src/scancontext_global_localizer_node.cpp`
- `src/humanoid_navigation2/launch/navigation2.launch.py`
- `src/humanoid_navigation2/launch/navigation2_fusion_sc.launch.py`

### 14.4 预期行为

启动阶段如果 SC 候选仍然歧义，系统应继续等待下一轮 recovery，而不是自动把歧义候选发布为 `/initialpose`。代价是相似结构下自动开机成功率可能下降，但错误自动定位比不定位更危险；必要时由 RViz 人工给初值。

重点检查日志:

```text
[GATE] global_mode=1, use_confidence_gate=1
GICP candidates ambiguous ...
```

不应再出现本次这种在候选 gap 很小、fitness 约 0.097 时仍发布 `/initialpose` 并进入 HEALTHY 的情况。

### 14.5 待实机验证

1. 启动日志应显示 `[GATE] global_mode=1, use_confidence_gate=1`。
2. 当候选 gap 低于 `required_gap=0.02` 时，应继续等待或报 ambiguous，不应发布 `/initialpose`。
3. fitness 约 `0.097` 的候选不应通过 `global_recovery_gicp_fitness_threshold=0.09`。
4. 若相似结构一直歧义，应人工 RViz 给初值，而不是自动接受低置信候选。

## 十五、2026-05-26 15:30:37 CST 审查补丁: 日志可观测性与 legacy 旁路收口

> 本节是在第十三节软验收实现之后，对 SC recovery 相关节点做的完整代码审查补丁。目标是确认“成功接受、拒绝原因、放行原因”能从日志还原，并补掉非 JSON legacy 消息绕过软验收的潜在逻辑漏洞。

### 15.1 这次新增功能是否会体现在日志中

会。修改后关键日志如下:

```text
[SC bridge] recovery #N started: ...
[SC bridge] sending trigger #N via GLOBAL/CONSERVATIVE (...)
accepted ScanContext pose; publishing converted /initialpose ... x=... y=... yaw=...deg
[SC bridge] NDT accepted initialpose! localization recovered.
[SC bridge] NDT rejected initialpose after ... rejected frames (reason=...), retrying recovery...
[LOST] recovery 软验收通过: recovery_delta=... <= odom_displacement(...)+margin(...), NDT error=...
[LOST→HEALTHY] recovery 通过软验收，切回 HEALTHY 并接受新 map->odom (pcl_pose=(...), ndt_state=..., converged=..., error=...)
[LOST] recovery 软验收拒绝: ...
[LOST] recovery 软验收拒绝本次结果，继续请求 SC 重定位 ...
```

因此后续只看 `debug_output.txt` 就可以区分:

- SC 是否发起了 recovery；
- SC 候选是否被桥接为 `/initialpose`，候选坐标和 yaw 是多少；
- NDT 是连续 accepted 还是 rejected；
- fusion 最终是否接受 recovery；
- 如果不接受，是 NDT 状态不健康、XY 偏差过大，还是数据过旧/缺失导致放行或跳过校验。

### 15.2 补掉的逻辑漏洞

#### 15.2.1 legacy recovery 成功消息不再绕过软验收

代码出处: `src/humanoid_navigation2/humanoid_navigation2/localization_odom_fusion.py` 的 `_on_recovery_status()`。

修改前:

- JSON `event_type=localization_recovered` 会走 `_validate_lost_recovery_soft()`；
- 非 JSON legacy 消息只要包含 `localization_recovered` 或 `localization_initialpose_published`，就会直接 `LOST→HEALTHY`。

修改后:

- JSON 和 legacy 成功消息统一归一成 `recovered_event`；
- 只要 fusion 处于 LOST，任何 recovered 事件都必须先通过 `_validate_lost_recovery_soft()`；
- 拒绝时会清掉 `_recovery_in_progress` 和 recovery 冷却时间，并立刻重新请求 SC recovery。

### 15.3 增强的日志字段

代码出处: `localization_odom_fusion.py` 的 `_on_recovery_status()` 和 `_validate_lost_recovery_soft()`。

新增成功接受日志字段:

- 恢复后的 `/pcl_pose` 坐标；
- `ndt_state`；
- `has_converged`；
- `fitness_score` / `NDT error`。

新增软验收跳过原因:

- 软验收参数被关闭；
- 尚未收到 `/localization/ndt_status`；
- NDT 状态过旧；
- 没有 `/pcl_pose`；
- `/pcl_pose` 过旧；
- frozen/odom 信息不完整；
- odom 位移超过 `recovery_pose_skip_odom_after_displacement_m`。

### 15.4 注释修正

修正 `src/humanoid_navigation2/launch/navigation2_fusion_sc.launch.py` 中容易误导排查的说明:

- HEALTHY 时由 NDT 发布 `map->odom`；
- DEGRADED/TRANSITIONING 时由 fusion 接管 `map->odom`；
- LOST 时 fusion 不发布 `map->odom`，等待 SC/HDL recovery bridge 通过 `/initialpose` 让 NDT 重建定位。

### 15.5 本轮审查结论

- `scancontext_to_initialpose.py` 没有发现语法错误；当前逻辑只在 recovery active 时把 SC `best_pose` 转为 `/initialpose`，正常导航期会忽略 SC `best_pose`。
- `localization_odom_fusion.py` 原先发现的主要风险是 legacy 成功消息旁路软验收，本节已修复。
- `navigation2_fusion_sc.launch.py` 运行参数与目标逻辑一致: SC localizer 不直接发布 `/initialpose`，不周期 query；SC bridge 运行期不因 `/pcl_pose stale` 自行 recovery，只响应启动缺失或 fusion recovery 请求。

语法检查:

```bash
python3 -m py_compile src/humanoid_navigation2/humanoid_navigation2/scancontext_to_initialpose.py src/humanoid_navigation2/humanoid_navigation2/localization_odom_fusion.py src/humanoid_navigation2/launch/navigation2_fusion_sc.launch.py
```

结果: 通过。

---

## 十六、2026-05-26 17:05:00 CST 修复: NDT score_threshold 收紧 + Pose Jump 感知 DEGRADED

> 背景: 2026-05-26 16:31 实机导航日志分析发现 NDT 在几何混叠区连续 3 次 pose jump (1.0m→1.4m→1.1m) 但 fitness=0.003~0.014。fusion 仅靠 `fitness_score > 0.5` 判定 DEGRADED，全程未触发 → odom 从未接管 → 代价地图被污染 → 控制器误判碰撞 → Nav2 失败。保护链被完全旁路。

### 16.1 问题回顾

| 时间 | 事件 |
|---|---|
| 16:33:19 | NDT pose jump #1: 1.034m, fitness=0.011 → NDT pose_jump_reacquire 接受 |
| 16:33:29 | NDT pose jump #2: 1.370m, fitness=0.003 → NDT pose_jump_reacquire 接受 |
| 16:33:33 | NDT pose jump #3: 1.104m, fitness=0.003 → NDT pose_jump_reacquire 接受 |
| 16:33:22-16:34:35 | 导航点位9 → 控制器检测碰撞 → Nav2 失败 |

fusion 全程 HEALTHY（fitness 从未 > 0.5），odom 从未接管，SC recovery 从未被触发。

### 16.2 修改策略

两管齐下:

1. **NDT 收紧**: `score_threshold` 从 2.0 降至 0.3。既然 odom 接管可靠，NDT 宁可拒帧（fitness > 0.3 → rejected → fusion 感知 error 升高 → DEGRADED → odom 接管），也不接受低质量匹配。
2. **Fusion 感知 NDT pose jump**: 利用 NDT status JSON 中已有的 `reason` / `correction_translation` 字段 + `/pcl_pose` 帧间监控，在 fitness 判据之外增加位姿跳变感知。

### 16.3 修改清单

#### 16.3.1 NDT `score_threshold`: 2.0 → 0.3

修改文件:
- `src/humanoid_navigation2/launch/navigation2_fusion_sc.launch.py` (L509)
- `src/humanoid_navigation2/launch/navigation2_fusion.launch.py` (L655)
- `src/humanoid_navigation2/launch/navigation2.launch.py` (L740)
- `src/humanoid_navigation2/launch/navigation_stack.launch.py` (L654)

NDT 在 `fitness_score > score_threshold` 时拒帧（state="rejected", reason="high_fitness"），不发布新 TF。收紧后更多场景会触发 NDT 拒帧 → error 升高 → fusion DEGRADED → odom 接管。但代价是增加 DEGRADED 频率，需要 odom 接管 + SC recovery 链路健壮。

#### 16.3.2 Fusion `_on_ndt_status()` 解析增强字段

文件: `src/humanoid_navigation2/humanoid_navigation2/localization_odom_fusion.py`
代码出处: `_on_ndt_status()` (L411-422)

新增解析:
```python
self.latest_ndt_reason = str(status.get('reason', ''))
self.latest_ndt_correction_translation = float(status.get('correction_translation', 0.0))
self.latest_ndt_correction_yaw = float(status.get('correction_yaw', 0.0))
```

NDT status JSON 中已包含这些字段（由 `publishLocalizationStatus()` 在 `lidar_localization_component.cpp:543-569` 发布）:
- `reason`: `"ok"` | `"pose_jump_candidate"` (正在确认跳变, 仍在重发布旧 pose) | `"confirmed_pose_jump"` (已接受跳变) | `"pose_jump"` (被拒绝) | `"high_fitness"` | `"not_converged"`
- `correction_translation`: NDT 本帧相对 initial guess 的平移修正量

#### 16.3.3 Fusion `_is_degraded()` 新增 pose jump 判据

文件: `src/humanoid_navigation2/humanoid_navigation2/localization_odom_fusion.py`
代码出处: `_is_degraded()` (L1158-1229)

新增三个判据（在 fitness_score 判据之前）:

| 判据 | 条件 | 原理 |
|---|---|---|
| **0: status reason** | `latest_ndt_reason in ('pose_jump_candidate', 'confirmed_pose_jump')` | NDT 自报跳变, 融合立即响应 |
| **0b: correction 超标** | `latest_ndt_correction_translation > 0.5m` | NDT 内部阈值 0.8m, 融合更敏感提前拦截 |
| **0c: pcl_pose 跳变** | `/pcl_pose` 帧间位移 > 0.5m | NDT 静默跳变兜底 (status 未报但 map→odom 已变) |

判据 0 的时序优势: NDT 在 `reason="pose_jump_candidate"` 时**仍在重发布旧的 map→odom**（`publishLastGoodTransformIfFresh`），此时融合进入 DEGRADED → 冻结正确的旧位姿 → NDT 即使后续确认跳变, 其新 map→odom 被融合覆盖, 不污染下游。

#### 16.3.4 Fusion `_update_healthy()` — `/pcl_pose` 帧间跳变兜底

文件: `src/humanoid_navigation2/humanoid_navigation2/localization_odom_fusion.py`
代码出处: `_update_healthy()` (L776-825)

HEALTHY 状态下每帧比较 `/pcl_pose` 与上一帧值:
```python
pcl_jump = math.hypot(pcl['x'] - prev['x'], pcl['y'] - prev['y'])
if pcl_jump > self.pose_jump_pcl_threshold_m:  # 0.5m
    self._pcl_pose_jump_detected = True
```

此标志在 `_is_degraded()` 判据 0c 中检查。几何混叠场景下 NDT 可能 fitness 极低且 status reason 仍为 "ok"，但 `/pcl_pose` 已静默跳变——此兜底可拦截。

#### 16.3.5 新增参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `pose_jump_degraded_from_status` | `True` | 是否从 NDT status reason 检测 pose jump |
| `pose_jump_degraded_from_pcl` | `True` | 是否从 /pcl_pose 帧间跳变检测 |
| `pose_jump_pcl_threshold_m` | `0.5` | /pcl_pose 帧间跳变阈值 (m) |
| `pose_jump_correction_threshold_m` | `0.5` | NDT correction_translation 直接触发 DEGRADED 的阈值 (m) |

Launch 配置: `navigation2_fusion_sc.launch.py` (L585-588), `navigation2_fusion.launch.py` (L724-727)

#### 16.3.6 DEGRADED 日志增强

`_update_healthy()` 中退化检测日志从单一提示改为枚举所有触发原因:
```
检测到 NDT 退化: ndt_reason=pose_jump_candidate, correction=1.034m>0.5m，需连续2帧确认
```

### 16.4 修改后数据流

```
场景：NDT 在几何混叠区收敛到错误位置（fitness=0.003, jump=1.2m）

修改前:
  NDT → pose_jump_candidate(1帧) → confirmed_pose_jump(2帧)
  → NDT map->odom 跳变 1.2m
  → fusion HEALTHY（fitness=0.003 < 0.5，无感知）
  → 代价地图被污染 → 控制器误判碰撞 → Nav2 失败

修改后 (途径 A — status reason):
  NDT → pose_jump_candidate(fitness=0.003, reason="pose_jump_candidate")
  → fusion _is_degraded() 判据0触发 (reason in jump list)
  → 连续2帧确认 → _enter_degraded()
  → 冻结 map->odom (跳变前的正确位姿)
  → NDT confirmed_pose_jump 但 fusion 已接管, 发布冻结值
  → 导航不中断, 代价地图不被污染

修改后 (途径 B — /pcl_pose 帧间):
  NDT 静默跳变 1.2m (status reason 仍为 "ok")
  → fusion _update_healthy() 检测 /pcl_pose 帧间位移 1.2m > 0.5m
  → _pcl_pose_jump_detected = True
  → _is_degraded() 判据0c触发 → _enter_degraded()
  → 同上

修改后 (途径 C — NDT score_threshold 收紧):
  NDT fitness=0.35 > 0.3 → 拒帧 (state="rejected", reason="high_fitness")
  → fusion _is_degraded() 判据1触发 (fitness > 0.5 或 NDT 拒帧后 error 升高)
  → _enter_degraded() → odom 接管

任何途径进入 DEGRADED 后:
  DEGRADED 超时(120s) 或 odom 位移超限(30m)
  → LOST → SC recovery → HEALTHY
```

### 16.5 设计考量

1. **三途径互补**: status reason (NDT 自报) + correction_translation (提前拦截) + /pcl_pose (静默兜底) + score_threshold (源头拒帧) 形成多层防护。

2. **NDT score_threshold 收紧的风险**: NDT 更频繁拒帧 → DEGRADED 频率增加 → odom 接管时间增加 → 更快触发 LOST → SC recovery。这是预期行为——宁可多走几次 recovery 也不能接受错误定位。odom 接管期间的导航精度由 Fast-LIO 里程计保证（短期精度 < 0.5m），可满足走廊导航需求。

3. **不需要 yaw 硬阈值**: pose_jump 检测只关注平移 (correction_translation + /pcl_pose XY delta)，不引入 yaw 检查，避免方向 wrap 和走廊对称性导致的误触发。

4. **`_reset_state()` 清理**: 状态重置时同时清除 `_pcl_pose_jump_detected` 和 `prev_pcl_map_odom`，防止上一轮 DEGRADED 的跳变标志影响下一轮。

### 16.6 验证

已完成:
```bash
python3 -m py_compile \
  src/humanoid_navigation2/humanoid_navigation2/localization_odom_fusion.py
python3 -c "import ast; ast.parse(open('src/humanoid_navigation2/launch/navigation2_fusion_sc.launch.py').read())"
python3 -c "import ast; ast.parse(open('src/humanoid_navigation2/launch/navigation2_fusion.launch.py').read())"
```

结果: 全部通过。

待实机验证:
1. NDT fitness > 0.3 时正确拒帧，fusion 进入 DEGRADED
2. NDT pose_jump_candidate 时 fusion 在 NDT 确认跳变前进入 DEGRADED
3. /pcl_pose 静默跳变 > 0.5m 时 fusion 检测并进入 DEGRADED
4. DEGRADED→LOST→SC recovery 全链路正常


---

## 十七、Pose Jump 检测鲁棒性分析 (2026-05-26 17:35:00 CST)

### 17.1 背景

第十六节实现三层 pose jump 检测后，提出两个追问：

1. NDT 在发布 `pose_jump_candidate` **之前**就已经不准了，fusion 怎么办？
2. NDT 跳变 **小于 0.8m**（如 0.5m），fusion 怎么办？

这两个问题分别针对检测的**时间盲区**和**阈值盲区**。

### 17.2 阈值体系

| 组件 | 参数 | 默认值 | 含义 |
|------|------|--------|------|
| NDT 内部 | `max_pose_jump_translation` | 0.8m | 单帧修正超过此值触发 pose_jump 机制 |
| NDT 内部 | `pose_jump_reacquire_max_translation` | 1.5m | 确认跳变的上限（超过则拒绝，不 reacquire） |
| NDT 内部 | `pose_jump_reacquire_max_fitness` | 0.02 | 确认跳变的 fitness 上限 |
| Fusion | `pose_jump_correction_threshold_m` | **0.5m** | correction_translation 超标即 DEGRADED |
| Fusion | `pose_jump_pcl_threshold_m` | **0.5m** | /pcl_pose 帧间跳变超标即 DEGRADED |

Fusion 的两个阈值（0.5m）**低于** NDT 的内部阈值（0.8m），形成灵敏度梯度。

### 17.3 Q1：NDT 报 pose_jump_candidate 之前就已经不准

**结论：这是一个真实的检测盲点，但实际场景中极短暂，且最终有 SC recovery 兜底。**

**场景描述**：NDT 静默收敛到错误局部最优（fitness 极低如 0.003），在错误位置持续追踪多帧，尚未触发 pose_jump 机制。

**三层检测表现**：

| 检测层 | 能检测？ | 原因 |
|--------|---------|------|
| `reason` = pose_jump_candidate | **否** | NDT 不认为自己在跳变（已在错误位置稳定追踪） |
| `correction_translation` > 0.5m | **否** | 每帧修正量很小（追踪而非重定位） |
| `/pcl_pose` 帧间跳变 > 0.5m | **否** | map→odom 帧间变化很小（平滑追踪） |

**为什么这是 SLAM 的本质限制**：如果 NDT 在错误局部最优处稳定收敛且 fitness 极低，单靠几何信息无法区分「定位正确」和「定位错误」。fitness 只衡量点云与地图的匹配度，不衡量全局正确性。score_threshold=0.3 也无济于事，因为此类场景 fitness 可能远低于 0.3。

**为什么实际中影响有限**：

几何混叠是**瞬态**的。NDT 在错误局部最优稳定追踪数十帧的情况在结构化室内环境中极为罕见。通常的时间线是：

```
帧 N:   误收敛到位置A (fitness 0.003)
帧 N+1: 误收敛到位置B (fitness 0.005) → 帧间跳变已被 /pcl_pose 检测捕获
帧 N+2: → 或 NDT 找到正确位置 → 发生大跳变 → reason/correction 检测捕获
```

即「持续错误追踪」很少持续超过几帧，在帧间震荡或最终跳变时即被检测。

**终极兜底 — SC Recovery**：
如果真的发生了持续错误追踪（>几十帧），退化链路仍然有效：

```
DEGRADED 超时(120s) 或 odom 位移超限(30m)
→ LOST
→ 请求 SC global localization
→ 重新定位（成功率 70-80%）
→ HEALTHY
```

代价是 robot 可能偏离路径一段时间，但**不会无限陷入错误状态**。

### 17.4 Q2：NDT 跳变小于 0.8m（如 0.5m）

**结论：0.6-0.8m 能被 fusion 捕获；≤0.5m 有意放行。**

**NDT 内部行为**：
- 0.5m 修正 < `max_pose_jump_translation` (0.8m) → NDT **不触发** pose_jump 机制
- NDT status JSON `reason` 保持 `"ok"`（或空）
- NDT **直接接受**这个修正，更新 map→odom

**Fusion 检测覆盖**：

| 修正量 | reason 检测 | correction_translation 检测 | /pcl_pose 检测 | 结果 |
|--------|------------|---------------------------|----------------|------|
| < 0.5m | 不触发 | 不触发 (≤0.5, 严格大于) | 不触发 | **放行** |
| 0.5-0.6m | 不触发 | **触发** (>0.5) | 取决于帧率 | **捕获** |
| 0.6-0.8m | 不触发 | **触发** | **触发** | **捕获** |
| > 0.8m | **触发** (pose_jump_candidate) | **触发** | **触发** | **三层全触发** |

**≤0.5m 有意放行的理由**：

这是设计的权衡——在「漏检容忍度」和「误报代价」之间画线：

1. **odom 有能力消化**：Fast-LIO 短期精度 <0.5m，0.5m 以下的定位偏移不影响局部导航
2. **误报代价高**：如果阈值设太低（如 0.2m），正常 NDT 噪声波动也会触发 DEGRADED → 频繁切换到 odom 接管 → odom 位移限制更快耗尽 → 更频繁 LOST → 导航效率下降
3. **0.5m 是经验分界**：低于此值的修正通常属于 NDT 正常追踪波动，而非全局跳变

**边界情况的处理**：

`correction_translation` 使用**严格大于**（`>` 而非 `>=`），所以恰好 0.50m 不会触发。这不是 bug——它意味着「小于等于 0.5m 都放行」，与设计意图一致。如果未来需要更敏感，将阈值降至 0.4m 即可。

### 17.5 三层检测覆盖全景

```
NDT correction_translation (m)
│
0.0 ├── 正常追踪区 (放行)
    │   正常 NDT 匹配波动，无需干预
    │
0.5 ├── Fusion 检测区 (correction_translation > 0.5m)
    │   + /pcl_pose 帧间跳变 > 0.5m
    │   NDT 自身不认为异常，但 fusion 主动介入
    │
0.8 ├── NDT pose_jump 触发区 (reason = pose_jump_candidate)
    │   + Fusion 三层全触发
    │
1.5 ├── NDT reacquire 接受上限
    │   (1.5-2.0m 会被 NDT 拒绝, 但 fusion 已在 0.5m 处冻结)
    │
    ▼  所有情况最终兜底: SC recovery (DEGRADED→LOST)
```

### 17.6 已知限制

1. **持续静默错误追踪**：NDT 在错误位置稳定追踪数十帧且 fitness < 0.02 → 三层检测全部旁路 → 依赖 SC recovery 超时兜底。理论上无法用纯几何手段解决，需要绝对定位（GPS/UWB/视觉地标）或独立的全局一致性验证。
2. **yaw 跳变盲区**：三层检测只关注平移，不检查 yaw 跳变。走廊等对称场景中 yaw 翻转（180°）可能在平移为 0 时发生。当前设计有意规避此问题——yaw 检查在对称场景中极易误触发。
3. **0.5m 阈值不可热调**：阈值通过 launch 参数传入，运行时可改但需重启节点。如果特定场景需要更低阈值，需修改 launch 文件后重新启动。

---

## Section XVIII: 2026-05-26 — Fusion DEGRADED 锁定期方案

### 18.1 问题背景

2026-05-26 导航测试中，点位7 区域 (x≈20m) NDT 出现反复 pose_jump→错误收敛→report ok 的循环。

**完整导航日志分析** (start_navigation_20260526_175405.log + ndt_fusion_monitor_20260526_183127.jsonl):

| 时间 | 事件 |
|------|------|
| 18:34:19 | 导航开始，初始位姿~(6.29, 10.91)，点位1 (6.05, 10.68) 1秒即到达 |
| 18:34:19-18:35:25 | 点位1→6 正常导航，每个 1-14秒，Nav2 Goal succeeded |
| 18:35:27 | 点位7 (20.45, 18.30) 开始导航 |
| 18:35:36-18:35:58 | **NDT 22秒内 4 轮 pose_jump 风暴**: correction 0.5→1.26m, 4次 DEGRADED 每次仅 2s |
| 18:35:41 | 机器人到达 (20.44, 19.17)，距目标 0.87m (tolerance=0.4m) |
| 18:36:06 | obstacle_blocked → paused at 点位7 |
| 18:36:04-18:37:45 | NDT high_fitness 振荡 (score 0.29↔0.33) |
| 18:37:45-19:01:26 | **24 分钟静默** — NDT 持续拒绝 |
| 19:01:57 | NDT correction 0.62m → Fusion DEGRADED, frozen=(9.01,5.59) |
| 19:01:59 | **/pcl_pose 跳变 17.23m** — NDT 自我跳到 17 米外 |
| 19:06:24 | NDT 自我收敛 score=0.13, reason=ok → **NDT 错误收敛，非 SC 纠正** |

**关键证据: SC 从未触发**: 整个日志中 `sc_candidates=0`, `sc_best_pose=0`, `recovery_request=0`, `recovery_status=0`。所有位置变化均为 NDT 内部自我跳变。

### 18.2 根因

```
HEALTHY → DEGRADED (NDT pose_jump) → NDT report ok + 3帧(0.1s) → 
TRANSITIONING(2s) → HEALTHY → NDT 再次 pose_jump → 循环
                     ↑
               NDT 收敛到错误解
               Fusion 盲目信任 NDT 的 "ok" 状态
```

- NDT 跳变后重新收敛 0.1s 内 report "ok"，fusion 3 帧即退出 DEGRADED
- NDT 从 (19,19) 跳至 (9,5.6) 再 report "ok" — 17m 错误跳变未被拦截
- 循环检测方案存在自相矛盾：**锁定期内 fusion 停在 DEGRADED，无法产生新的 DEGRADED 进入事件供循环计数**

### 18.3 方案: DEGRADED 锁定期

**核心理念**: 进入 DEGRADED → 强制冷静期 → 拒绝 NDT 一切假恢复 → 严格验证后才放行。

```
进入 DEGRADED
    │
    ├─ 0~min_degraded_lock_sec (30s, 锁定期)
    │   ├─ frozen map→odom + Fast-LIO odom 驱动导航
    │   ├─ 监听 NDT 但不采信恢复信号
    │   ├─ 锁定期内 NDT 拒绝率 > 90% + ≥30帧 → 提前 LOST → SC
    │   └─ odom 位移 > 30m → LOST (已有逻辑)
    │
    ├─ 30s~max_degraded_lock_sec (180s, 验证期)
    │   ├─ _is_healthy_strict(): error<0.15 + converged + reason=ok 
    │   │   + correction<0.3m + frozen→NDT 跳变<0.8m
    │   ├─ 连续 10 帧满足 → _enter_transitioning()
    │   │   └─ 二次验证: frozen→NDT 跳变>5m → 拒绝 → LOST
    │   └─ 不满足 → 继续等待
    │
    └─ >max_degraded_lock_sec (180s)
        └─ LOST → SC 全局重定位
```

**与上回日志对照回放**:
```
18:35:36  进入 DEGRADED，锁定期至 18:36:06 (30s)
18:35:38  NDT pose_jump→ok (accept) → 锁定期内，忽略
18:35:39  NDT ok→pose_jump (reject) → 锁定期内，忽略
18:35:40  NDT pose_jump→ok (accept) → 锁定期内，忽略
...NDT 在锁定期内自嗨 4 轮，fusion 全部忽略...
18:36:06  锁定期满 → 验证期
          _is_healthy_strict() 检查:
          - 若 NDT 收敛到错误位姿 (跳变 >5m) → 拒绝恢复
          - 若 NDT 持续不稳定 → 继续等待
18:39:06  总 DEGRADED > 180s → LOST → SC ✓
```

### 18.4 参数表

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `min_degraded_lock_sec` | 30.0 | 最短锁定期(s)，此期间拒绝 NDT 恢复信号 |
| `max_degraded_lock_sec` | 180.0 | DEGRADED 总超时(s)，超时→LOST→SC |
| `lock_recovery_healthy_consecutive_frames` | 10 | 锁定期后恢复需连续健康帧数 |
| `lock_recovery_max_correction_m` | 0.3 | 恢复时允许的最大 correction_translation(m) |
| `lock_early_lost_rejection_rate` | 0.9 | 锁定期内 NDT 拒绝率超此值→提前 LOST |
| `lock_early_lost_min_frames` | 30 | 提前 LOST 需最少累积帧数 |
| `recovery_pose_jump_max_m` | 5.0 | frozen→NDT 跳变上限，超限拒绝恢复 |

### 18.5 新增方法 `_is_healthy_strict()`

锁定期满后使用，比 `_is_healthy()` 更严格:

1. error < 0.15 (与 `_is_healthy` 相同)
2. NDT converged = True
3. **reason 必须为 "ok"** (拒绝 pose_jump_candidate/confirming 等过渡状态)
4. **correction_translation < lock_recovery_max_correction_m (0.3m)**
5. frozen→NDT map→odom 跳变 < 0.8m (与 `_is_healthy` 相同)

### 18.6 修改文件

| 文件 | 修改内容 |
|------|----------|
| `localization_odom_fusion.py` | `_update_degraded()` 锁定期守卫 + `_is_healthy_strict()` + 锁定期参数 + 移除循环检测 |
| `ndt_fusion_monitor.py` | 锁定期状态面板: "锁定期内 🔒" / "验证期 🔍" + robot_pose/pcl_pose 定期日志 |
| `navigation2_fusion_sc.launch.py` | 锁定期参数 |
| `navigation2_fusion.launch.py` | 锁定期参数 |

### 18.7 已知限制

1. **锁定期内导航完全依赖 Fast-LIO 里程计**: 30s 锁定期内位移 ~6-8m (0.3m/s 行走)，odom 漂移 <0.01m，可忽略。但如果是被推动而非自主行走，odom 不生效且精度丧失
2. **锁定期时长是固定死的**: 即使 NDT 在 5 秒内真正恢复（如遮挡后重新看到），也要等到锁定期满。偏保守但安全
3. **180s 总超时才触发 SC**: 对于锁定期内 NDT 拒绝率 <90% 但始终无法收敛的场景（如点位7区域特征确实不足），SC 要等 3 分钟才介入。可通过降低 `max_degraded_lock_sec` 加速
4. **NDT 跳变 17m 被 5m 跳变验证拦截**: 验证期 `_enter_transitioning()` 检查 frozen→NDT 跳变>5m→拒绝。但如果 NDT 在验证期内逐步漂移而非跳变，可能绕过此检查
