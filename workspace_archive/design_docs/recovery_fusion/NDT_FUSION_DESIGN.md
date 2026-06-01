# NDT定位 + 里程计融合方案 设计与实现文档

> 创建时间: 2026-05-25
> 作者: Claude Opus 4.7 + ubuntu

---

## 一、问题背景

### 1.1 现象
机器人在导航到点位14附近（map位置约(8.9, 19.2)，odom位置约(18.7, -0.7)）时，NDT定位反复漂移。`robot_realpose` (map->base_footprint) 会突然跳回原点附近(-0.2, 1.9)，与实际位置相差15-20m。

### 1.2 根因链
```
该位置几何感知混叠 (激光扫描与地图中另一位置高度相似)
    ↓
NDT matching_error 从 0.09 飙升到 1.0~9.4
    ↓
score_threshold=0.30 → 扫描被拒绝 → 3秒后丢定位
    ↓
recovery 触发 → hard_gate=false → 接受21m外的错误候选
    ↓
trusted pose 被污染 → 后续 recovery 全部基于错误 prior
    ↓
191次 recovery 全部失败 / 恶性循环
```

---

## 二、整体方案

### 2.1 核心思路

**当 NDT 漂移时，不打断导航，而是冻结 map->odom 变换，让 Fast-LIO 里程计驱动机器人继续运动，等 NDT 自然恢复后平滑切回。**

```
NDT 健康:  map_T_base = NDT输出的map_T_odom × 里程计(odom_T_body) × 静态(body_T_base)
NDT 漂移:  map_T_base = 冻结的map_T_odom × 里程计(odom_T_body) × 静态(body_T_base)
                                ↑ 唯一的区别：map->odom 不更新了
```

### 2.2 为什么 odom 短期可靠

| 指标 | Fast-LIO 性能 | 120s/30m 窗口内 |
|------|-------------|----------------|
| 平移漂移 | <0.5cm/m | **<15cm** |
| 旋转漂移 | <0.1°/s | **<12°** |
| 静止漂移 | 0 | **0** |
| 13.5圈累积 | ~5.4° | **单次spin<1°** |

### 2.3 两条路架构

```
路径1 (原逻辑, 不修改任何原文件):
  NDT漂移 → hdl_bootstrap触发recovery → 暂停导航 → 全局重定位 → 上报APP
  启动: ros2 launch humanoid_navigation2 navigation_stack.launch.py
        ros2 launch humanoid_navigation navigation.launch.py

路径2 (融合模式, 新建文件实现):
  NDT漂移 → fusion节点冻结map->odom → 导航继续 → odom兜底 → 不上报APP
  NDT恢复 → 平滑过渡切回
  odom兜不住 → LOST → 触发HDL全局重定位(保留)
  启动: ros2 launch humanoid_navigation2 navigation2_fusion.launch.py
        ros2 launch humanoid_navigation navigation_fusion.launch.py
```

---

## 三、文件改动清单

### 3.1 新建文件

| 文件 | 位置 | 行数 | 作用 |
|------|------|------|------|
| `localization_odom_fusion.py` | `src/humanoid_navigation2/humanoid_navigation2/` | ~800行 | **核心融合节点** |
| `navigation2_fusion.launch.py` | `src/humanoid_navigation2/launch/` | ~1100行 | 融合模式导航栈启动文件 |
| `navigation_fusion.launch.py` | `src/humanoid_navigation/launch/` | ~70行 | 融合模式应用层启动文件 |
| `navigation_state_manager_fusion.py` | `src/humanoid_navigation/humanoid_navigation/` | ~2790行 | 融合模式导航状态管理器 |

### 3.2 修改文件

| 文件 | 改动 | 行数 |
|------|------|------|
| `src/humanoid_navigation2/launch/navigation_stack.launch.py` | 恢复 `score_threshold=2.0`、`hard_gate=True`、`max_xy=4.0` | ~5行 |
| `src/humanoid_navigation2/humanoid_navigation2/hdl_bootstrap_to_initialpose.py` | 新增 `trusted_pose_max_map_odom_displacement` 参数和位移检查 | ~15行 |
| `src/humanoid_navigation2/setup.py` | 注册 `localization_odom_fusion` 入口点 | +1行 |
| `src/humanoid_navigation/setup.py` | 注册 `navigation_state_manager_fusion` 入口点 | +1行 |

### 3.3 完全未修改的原文件

- `navigation_stack.launch.py` — 路径1继续使用
- `navigation.launch.py` — 路径1继续使用
- `navigation_state_manager_recoverable.py` — 路径1继续使用
- `hdl_bootstrap_to_initialpose.py` — 原逻辑不受影响
- `lidar_localization_component.cpp` — 不改

---

## 四、核心组件详解

### 4.1 localization_odom_fusion (融合节点)

**文件**: `src/humanoid_navigation2/humanoid_navigation2/localization_odom_fusion.py`

#### 状态机

```
                    matching_error > 0.5 (连续2帧)
        ┌─────────────────────────────────────────────┐
        │                                             │
        ▼                                             │
   ┌─────────┐                              ┌─────────────────┐
   │ HEALTHY │                              │    DEGRADED     │
   │         │                              │                 │
   │ 直通NDT │                              │ 冻结map->odom   │
   │ 更新    │                              │ odom传播位姿     │
   │ snapshot│                              │ 更新诊断信息     │
   └────┬────┘                              └───────┬─────────┘
        │                                           │
        │     matching_error < 0.15 (连续3帧)        │
        │     + 平滑过渡 2s                          │
        │                                           │
        │         ┌─────────────────┐               │
        └────────▶│ TRANSITIONING   │◀──────────────┘
                  │ smoothstep插值  │
                  └────────┬────────┘
                           │
                           ▼ (超时/位移过大)
                  ┌─────────────────┐
                  │      LOST       │
                  │ 停止发布TF      │
                  │ 等待recovery    │
                  └────────┬────────┘
                           │ recovery成功
                           ▼
                      HEALTHY
```

#### 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `degraded_error_threshold` | 0.5 | 进入 DEGRADED 的 NDT error 阈值 |
| `healthy_error_threshold` | 0.15 | 恢复 HEALTHY 的 NDT error 阈值 |
| `healthy_consecutive_frames` | 3 | 恢复需要连续健康帧数 |
| `degraded_consecutive_frames` | 2 | 退化需要连续退化帧数 |
| `max_odom_displacement_m` | 30.0 | 最大odom位移，超过进入LOST |
| `nav_active_lost_timeout_sec` | 120.0 | 导航中LOST超时 |
| `nav_idle_lost_timeout_sec` | 600.0 | 静止时LOST超时（10分钟够播报） |
| `nav_idle_extreme_error` | 5.0 | 静止时极端error阈值 |
| `transition_duration_sec` | 2.0 | 平滑过渡时间 |

#### 导航感知超时逻辑

```
DEGRADED 期间，根据导航状态选择不同 LOST 超时:

  导航中 (EXECUTING/PLANNING):
    → 超时 120s (需要准确定位)
    → odom位移>30m: 立即LOST

  静止播报 (IDLE/COMPLETED/PAUSED):
    NDT error < 5.0:
      → 超时 600s (10分钟, 等播报完)
    NDT error > 5.0:
      → 超时 300s (NDT彻底挂了, 该恢复了)
```

### 4.2 navigation_state_manager_fusion (状态管理器)

**文件**: `src/humanoid_navigation/humanoid_navigation/navigation_state_manager_fusion.py`

与原版唯一区别在 `handle_localization_recovery_started()`:

```python
# 融合版新增逻辑 (在原版暂停导航之前):
fusion_state = self._get_fusion_state()
if fusion_state in ("DEGRADED", "TRANSITIONING"):
    # fusion节点正在用odom兜底 → 不暂停 → 不上报APP
    return

if fusion_state == "LOST":
    # fusion兜不住了 → 走原版recovery逻辑
    pass
# 否则: 走原版逻辑 (暂停导航 + 上报APP)
```

### 4.3 hdl_bootstrap_to_initialpose 改动

**文件**: `src/humanoid_navigation2/humanoid_navigation2/hdl_bootstrap_to_initialpose.py`

改动两处:

1. **recovery prior 防污染**:
```python
# 新增参数
self.trusted_pose_max_map_odom_displacement = 3.0  # 默认3m

# 在 update_trusted_pose_if_healthy() 中:
# 计算新旧 map->odom 的位移差
map_odom_delta_xy = |new_map_odom - last_map_odom|
if map_odom_delta_xy > 3.0:
    reject update  # 防止漂移后的错误位姿污染recovery prior
```

2. **融合模式下禁用自动recovery** (通过launch参数):
```python
# navigation2_fusion.launch.py 中:
'ndt_failure_triggers_recovery': False  # 让fusion决定何时recovery
'hdl_divergence_triggers_recovery': False
```

### 4.4 navigation_stack.launch.py 改动

**文件**: `src/humanoid_navigation2/launch/navigation_stack.launch.py`

| 参数 | 改前 | 改后 | 原因 |
|------|------|------|------|
| `score_threshold` | 0.30 | **2.0** | 容忍fitness 1.0+噪声帧，不丢定位 |
| `global_localization_recovery_prior_hard_gate` | False | **True** | recovery不接受远方错误候选 |
| `global_localization_recovery_prior_max_xy` | 3.0 | **4.0** | 配合hard gate放宽prior半径 |

---

## 五、完整逻辑流程

### 5.1 正常导航 (NDT健康)

```
1. HDL bootstrap → 全局重定位 → 发布 /initialpose → NDT初始化
2. NDT 持续做 scan-to-map 匹配 → 发布 map->odom TF
3. fusion 节点处于 HEALTHY 状态 → 不干预 → 更新健康快照
4. 导航正常进行 → APP正常上报
```

### 5.2 NDT漂移 (融合模式)

```
1. NDT matching_error > 0.5 (连续2帧)
      ↓
2. fusion: HEALTHY → DEGRADED
      - 冻结 map->odom (使用最后的健康快照)
      - 发布 frozen_map_odom 作为 map->odom TF (30Hz, 覆盖NDT的10Hz)
      - 发布 fusion_status={"state":"DEGRADED"}
      ↓
3. hdl_bootstrap: ndt_failure_triggers_recovery=false → 不触发recovery
      ↓
4. nav_state_manager_fusion: 收到 DEGRADED → 忽略 recovery_started
      - 导航不暂停
      - APP不上报"定位异常"
      ↓
5. 机器人继续走:
      - controller_server (odom帧) → 局部跟踪正常
      - planner_server (map帧) → 全局路径有<15cm误差
      - local_costmap → 障碍物感知正常
      - 收到新点位 → 正常规划路径 → 走过去
      ↓
6. NDT 恢复: matching_error < 0.15 (连续3帧)
      - fusion: DEGRADED → TRANSITIONING
      - smoothstep 平滑插值 2s → frozen_map_odom → ndt_current_map_odom
      - fusion: TRANSITIONING → HEALTHY
      - 恢复正常 NDT 定位
```

### 5.3 NDT无法恢复 (兜底失败)

```
1. DEGRADED 超时 (导航中120s / 静止时600s) 或 odom位移>30m
      ↓
2. fusion: DEGRADED → LOST
      - 停止发布 map->odom TF
      - 发布 fusion_status={"state":"LOST"}
      ↓
3. nav_state_manager_fusion: 收到 LOST → 走原逻辑
      - 暂停导航
      - 上报APP"定位异常"
      ↓
4. hdl_bootstrap: 触发 HDL 全局重定位
      - 成功 → 发布新 /initialpose → NDT 重新初始化 → HEALTHY
      - 失败 → 重试
```

### 5.4 到达点位后静止播报 (关键场景)

```
1. 机器人到达点位 → 导航状态: EXECUTING → COMPLETED → IDLE
      ↓
2. APP 开始播报讲解词
      ↓
3. 此时 NDT 漂移:
      - fusion: HEALTHY → DEGRADED
      - 导航状态是 IDLE → LOST超时 = 600s (不是120s!)
      - 机器人不动 → odom不漂 → 定位保持稳定
      ↓
4. 播报完成 → APP发下一个点位:
      - 导航状态: IDLE → EXECUTING
      - LOST超时变回 120s (正常)
      - 机器人走到下一个点位
      ↓
5. 如果 NDT 在此期间恢复:
      - fusion 自动切回 HEALTHY
      - 整个过程对 APP 完全透明
```

---

## 六、TF树与坐标系

### 6.1 完整TF链

```
map ──fusion/NDT──▶ odom ──ident──▶ camera_init ──FastLIO──▶ body ──static──▶ base_footprint
       ↑ 融合节点         ↑ 静态TF      ↑ 非标坐标系       ↑ 静态TF旋转
       发布(不变)          (identity)    (x左y下z后)        (0.5,0.5,-0.5,0.5)
                                                        转换为标准ROS坐标系
```

### 6.2 融合节点与坐标系

融合节点只发布 `map->odom` TF。**完全不感知**非标坐标系：
- Fast-LIO 的 `camera_init->body` 非标旋转由 `body->base_footprint` 静态TF处理
- odom 位移计算使用 `hypot(dx, dy)`，旋转不变量
- 非标坐标系对融合节点**零影响**

### 6.3 DEGRADED期间各节点影响

| 节点 | 工作帧 | 是否受影响 | 说明 |
|------|--------|-----------|------|
| local_costmap | odom | ✗ 不受影响 | 障碍物感知正常 |
| controller_server | odom | ✗ 不受影响 | 局部路径跟踪正常 |
| planner_server | map | ~ 微影响 | 全局路径 <15cm 误差 |
| periodic_clearing | base_footprint | ✗ 不受影响 | 清除扫描正常 |
| 避障 | odom | ✗ 不受影响 | 完全正常 |

---

## 七、Bag验证结果

### 7.1 验证环境
- Bag: `nav_drift_test2` (49GB, 1547s)
- 36个路点，机器人实际到达25个
- 总旋转4869° (13.5圈)，多次spin-in-place

### 7.2 验证结论

| 验证项 | 结果 |
|--------|------|
| 状态机切换 | ✓ 17次转换，全部正确 |
| HEALTHY→DEGRADED | ✓ error>0.5后2帧内切换 |
| DEGRADED→TRANSITIONING | ✓ error<0.15连续3帧 |
| TRANSITIONING→DEGRADED | ✓ NDT又变差时正确中断过渡 |
| DEGRADED→LOST | ✓ 超时正确触发 |
| LOST→HEALTHY | ✓ recovery成功正确恢复 |
| 状态分布 | HEALTHY 71% / DEGRADED 14% / LOST 14% / TRANSITIONING 1% |

### 7.3 局限
此bag中机器人在漂移时已停止移动（recovery提前打断了导航），无法完全验证"融合节点让机器人继续走"的效果。需在真实导航中测试。

---

## 八、部署与验证

### 8.1 启动方式

```bash
# 路径1 (原逻辑)
ros2 launch humanoid_bringup robot_real.launch.py

# 路径2 (融合模式) - 修改 robot_real.launch.py 中两行:
#   navigation_stack.launch.py → navigation2_fusion.launch.py
#   navigation.launch.py      → navigation_fusion.launch.py
```

### 8.2 运行时监控

```bash
# 查看融合状态
ros2 topic echo /localization/fusion_status

# 查看NDT状态
ros2 topic echo /localization/ndt_status | grep matching_error

# 查看导航状态
ros2 topic echo /navigation_status | grep state

# 查看map->odom跳变
ros2 topic echo /tf | grep "map.*odom"
```

### 8.3 验证检查表

| # | 检查项 | 通过标准 |
|---|--------|----------|
| 1 | fusion节点启动 | `ros2 node list \| grep fusion` 有输出 |
| 2 | fusion_status发布 | topic持续输出JSON |
| 3 | HEALTHY→DEGRADED | error>0.5后2帧内切换 |
| 4 | 导航不暂停 | DEGRADED期间无 PAUSED 事件 |
| 5 | APP不上报 | 无"定位异常"ack消息 |
| 6 | DEGRADED→HEALTHY | error<0.15后3帧开始过渡 |
| 7 | 静止超时 | IDLE状态下超时>120s |
| 8 | LOST恢复 | LOST→recovery→HEALTHY |
| 9 | 原版路径不变 | 无回归 |

---

## 九、关键设计细节

### 9.1 导航感知 LOST 超时

DEGRADED 期间，根据导航状态选择不同的超时阈值：

| 机器人状态 | LOST 超时 | 原因 |
|-----------|----------|------|
| 导航中 (EXECUTING/PLANNING) | **120s** | 需要准确定位 |
| 静止播报 (IDLE/COMPLETED/PAUSED) | **600s** | 不需要定位，等播报完 |
| 静止 + NDT error > 5.0 | **300s** | NDT彻底挂了，该恢复了 |

融合节点订阅 `/navigation_status`（由 navigation_state_manager 发布），实时感知导航状态。

### 9.2 到达路点重置计时器

每次到达路点（nav_state: EXECUTING→IDLE），**同时重置超时计时器和 odom 位移计数器**：

```
点位2 → 点位3:
  走了 40s / 8m → 到达点位3
  → EXECUTING→IDLE → degraded_start_time = now (归零)
  → frozen_odom_body = 当前 odom 位置 (位移归零)
  → 播报 90s: 使用 idle 超时(600s), 从0开始算, 90/600=15%

点位3 → 点位4:
  走了 30s / 6m → 30/120=25%, 6/30=20% → OK
  → 到达点位4 → 再次归零
```

原理：静止时 odom 零漂移，且知道自己在路点位置，之前的累积误差可以归零。

### 9.3 三层位移保护

| 层级 | 限制 | 重置条件 | 作用 |
|------|------|---------|------|
| 单段位移 | **30m** | 每路点重置 | 防止单段走太远 |
| 累计位移 | **100m** | **永不重置** | 防止跨路点累积漂移 |
| odom 漂移率 | ~0.5cm/m | - | 100m → ~50cm 地图误差 |

```
点位2→3:  8m,  单段✓, 累计8m
点位3→4:  10m, 单段✓, 累计18m
...
点位12→13: 8m,  单段✓, 累计98m
点位13→14: 8m,  单段✓, 累计106m → LOST! ← 100m累计触发
```

Fast-LIO 漂移率约 0.5cm/m，100m 累计位移 → 约 50cm 地图误差，是导航可接受的极限。

### 9.4 LOST 触发 Recovery 机制

当融合节点进入 LOST（超时或位移过大），**主动请求 HDL 全局重定位**：

```
fusion LOST
    ↓
fusion._request_recovery() → 发布 /localization/recovery_requests (JSON)
    ↓
hdl_bootstrap.recovery_request_callback() 收到消息
    ↓
prepare_external_prior_from_request() → start_recovery()
    ↓
清除HDL缓存 → 等待静止 → 全局重定位 (/relocalize_with_prior_checked)
    ↓
匹配成功 → 发布 /initialpose → NDT 重新初始化
    ↓
fusion._on_recovery_status() 检测到 recovery 成功 → LOST → HEALTHY
```

如果第一次 recovery 失败，fusion 每 **15秒**（`recovery_request_cooldown_sec`）重新请求一次。

### 9.5 多次接管支持

融合节点支持**无限次** HEALTHY ↔ DEGRADED 切换。每次进入 DEGRADED 都是全新开始：

```python
def _enter_degraded(self):
    self.frozen_map_odom = last_healthy_map_odom.copy()  # 新的快照(NDT恢复后的正确值)
    self.frozen_odom_body = last_healthy_odom_body.copy()  # 新的参考点
    self.degraded_start_time = time.monotonic()  # 新计时器
    self.total_odom_displacement = 0.0  # 累计位移清零
```

关键是：**NDT 恢复后 map->odom 被重新校正，odom 的累积误差被 NDT 归零**。

### 9.6 NDT 漂移检测速度

从 bag 数据实测：NDT 在一帧内（0.1s）从 error=0.09 跳到 error=1.75。

```
检测链:
  帧N:   error=0.09  last_healthy 更新 ← 这一刻的快照
  帧N+1: error=1.75  degraded计数=1
  帧N+2: error=1.05  degraded计数=2 → DEGRADED!
  
  总延迟: 0.2s (2帧)
  机器人移动: ~2cm (@0.1m/s)
  快照年龄: <0.3s → frozen_map_odom 误差 <2cm
```

### 9.7 五层防线

| 防线 | 位置 | 参数 | 作用 |
|------|------|------|------|
| ① 跳变拒帧 | lidar_localization | `reject_pose_jump=True` | NDT跳>0.8m→拒帧→error升高 |
| ② 高fitness拒帧 | lidar_localization | `score_threshold=2.0` | fitness>2.0→拒绝 |
| ③ err>0.5冻结 | fusion节点 | `degraded_error_threshold=0.5` | error>0.5→DEGRADED |
| ④ recovery硬门 | hdl_bootstrap | `hard_gate=True` | recovery候选在prior 4m外→拒绝 |
| ⑤ prior防污染 | hdl_bootstrap | `trusted_pose_max_map_odom_displacement=3.0` | map->odom跳>3m→拒绝更新 |

**关键**: `reject_pose_jump` 在融合模式下必须为 **True**。如果为 False，NDT 可能以低 error 跳到错误位姿（感知混叠），fusion 的 err>0.5 检测完全抓不到。修正后的链路：

```
NDT 跳到错误位置 → reject_pose_jump=True → 拒帧
    → error 升高 → fusion 检测到 → DEGRADED
    → 冻结正确的 map->odom → 安全
```

---

## 十、路径1 vs 路径2 参数差异

| 参数 | 路径1 (原) | 路径2 (fusion) | 说明 |
|------|-----------|----------------|------|
| `score_threshold` | 2.0 | 2.0 | 相同 |
| `reject_pose_jump` | **False** | **True** | fusion需要拒帧来检测漂移 |
| `hard_gate` | True | True | 相同 |
| `ndt_failure_triggers_recovery` | True | **False** | fusion自己决定何时recovery |
| `hdl_divergence_triggers_recovery` | False | False | 相同 |
| 状态管理器 | `recoverable` | `fusion` | fusion版抑制DEGRADED告警 |
| fusion节点 | 不启动 | 启动 | - |

---

## 十一、Git提交记录

```
d4e7ab5 fix: reset odom displacement counter on waypoint arrival
852be60 fix: add cumulative odom displacement limit (100m, never resets)
a5df8f1 fix: enable reject_pose_jump in fusion mode launch file
6d961fa fix: fusion LOST now actively triggers HDL relocalization via recovery_requests
f672615 fix: reset DEGRADED timeout when robot arrives at waypoint
0a0fa4f feat: add navigation-aware LOST timeout to fusion node
9cad3e5 feat: add two-path architecture - fusion mode with odom fallback
80d2a96 chore: checkpoint before fusion two-path implementation
ebd551f fix: localization drift at hallway corner - harden recovery and fix NDT threshold
```
