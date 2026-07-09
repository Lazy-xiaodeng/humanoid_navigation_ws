# 定位架构简化方案：去 Fusion 节点

## 1. 现状与问题

### 当前架构

```
NDT (lidar_localization)  ──map→odom TF──▶  Fusion (localization_odom_fusion)
       │                                        │
       └──ndt_status (JSON)──▶                  │  5态状态机:
                                                │  INITIALIZING → HEALTHY
                                                │  HEALTHY → DEGRADED → LOST
                                                │  DEGRADED → TRANSITIONING → HEALTHY
                                                │
                                                ├──fusion_status──▶  nav_state_manager
                                                │                    BLOCKED_STATES:
                                                │                    DEGRADED/TRANSITIONING/LOST
                                                │
                                                └──recovery_request─▶  hdl_bootstrap / SC
                                                  (frozen_tf_chain)
```

### 问题

| # | 问题 | 影响 |
|---|---|---|
| 1 | Fusion 1992 行，14 次 commit，是最大的 bug 来源 | 维护成本极高 |
| 2 | Fusion 和 NDT 同时发布 map→odom TF → 竞态覆盖 | prior 污染 |
| 3 | Fusion 状态机与 nav_state_manager 状态机互相耦合 | 状态不一致风险 |
| 4 | Fusion HEALTHY 时崩溃 → nav_state_manager 无感知 | 本次测试的根因 |
| 5 | 两套 recovery 路径 (fusion LOST vs nav 失败) 逻辑重复 | 维护负担 |
| 6 | 10+ ROS 参数仅用于 tuning fusion 行为 | 调参困难 |

### 实际上 fusion 做的事，NDT 和 nav_state_manager 自己就能做

| Fusion 职责 | 替代方案 |
|---|---|
| 监控 NDT error/correction/jump → DEGRADED | NDT 自己检测 pose_jump，发 ndt_status；nav_state_manager 订阅 |
| 冻结 map→odom TF | NDT `republish_last_good_tf_on_failure: true` |
| 发 recovery 请求 + prior | nav_state_manager 已有的 `request_navigation_context_recovery_for_localization()` |
| 软验收 recovery 结果 | nav_state_manager 检查 NDT fitness < 阈值 + TF 可用 |
| inlier=0 虚假健康检测 | 迁移到 nav_state_manager (~20行) |

---

## 2. 目标架构

```
                        ┌──────────────┐
                        │   HDL / SC   │
                        │  全局重定位   │
                        └──────┬───────┘
                               │ /initialpose
                               ▼
┌─────────────────────────────────────────────────────┐
│               NDT (lidar_localization)               │
│                                                     │
│  正常: 发布 map→odom TF + ndt_status (reason="ok")   │
│  跳变: reason="pose_jump" → 停止发布新 TF             │
│        保持发布 last_good TF (republish_on_failure)   │
│  恢复: 收到 /initialpose → initialpose_relax (4s)    │
│        → 恢复匹配 → 重新发布 TF                       │
└──────────────────────┬──────────────────────────────┘
                       │ ndt_status (JSON)
                       ▼
┌─────────────────────────────────────────────────────┐
│            nav_state_manager                        │
│                                                     │
│  订阅 ndt_status:                                   │
│    reason="pose_jump" (连续N帧) → PAUSE + zero cmd   │
│    inlier=0 + fitness<0.01 × 30s → recovery         │
│    TF 超时 (3s 无更新) → recovery                   │
│                                                     │
│  Recovery:                                          │
│    waypoint_path prior (前→当前→下)                   │
│    → HDL with prior → 失败 → full global            │
│    → 循环直到 NDT fitness < 0.3 + TF 恢复            │
└─────────────────────────────────────────────────────┘
```

**节点数: 3→2** (去掉 fusion)
**状态机: 5态→0态** (NDT 自监控, nav_state 只管 PAUSE/RESUME)

---

## 3. 涉及文件清单

### 3.1 删除

| 文件 | 行数 | 说明 |
|---|---|---|
| `src/humanoid_navigation2/humanoid_navigation2/localization_odom_fusion.py` | 1992 | 整个 fusion 节点 |

### 3.2 修改

| 文件 | 改动范围 | 说明 |
|---|---|---|
| `src/humanoid_navigation/humanoid_navigation/navigation_state_manager_fusion.py` | ~300行 删/改 + ~50行 新增 | 去掉 fusion_status 订阅, 新增 ndt_status 直接监听 |
| `src/humanoid_navigation2/humanoid_navigation2/hdl_bootstrap_to_initialpose.py` | ~20行 删 | 清理 fusion 专用的 recovery 路径, P0-3 保留 |
| `src/humanoid_navigation2/launch/navigation2_fusion_sc_v2.launch.py` | ~20行 删 | 去掉 fusion 节点启动 + 参数 |
| `src/humanoid_navigation2/launch/navigation2_fusion_sc.launch.py` | ~10行 删 | 同上 (v1 launch) |
| `src/lidar_localization/src/lidar_localization_component.cpp` | ~10行 删 | 去掉 fusion_status_timeout 参数 |
| `src/lidar_localization/include/lidar_localization/lidar_localization_component.hpp` | ~3行 删 | 去掉对应成员变量 |
| `src/lidar_localization/param/localization.yaml` | ~5行 改 | `republish_last_good_tf_on_failure: true`, 删 fusion 参数 |

### 3.3 不改

| 文件 | 说明 |
|---|---|
| `scancontext_to_initialpose.py` | 不涉及 fusion, 无需修改 |
| `scancontext_global_localization/` | 不涉及 |
| `hdl_localization/` (C++ nodelet) | 不涉及, P1-2 TTL 保留 |
| `hdl_global_localization_node` | 不涉及 |
| `nav2_params_xy_yaw.yaml` | 不涉及 |
| `robot_real.launch.py` | 不涉及 (只改子 launch) |

---

## 4. 简化后的各阶段逻辑

### 4.1 启动定位

```
1. HDL bootstrap 全局搜索 → /initialpose
2. NDT 收到 /initialpose → initialpose_relax 4s
3. NDT 收敛 → 发布 map→odom TF + ndt_status{state:"accepted", reason:"ok"}
4. nav_state_manager 检测到 TF 可用 → 允许导航
5. 用户发导航 → 正常执行
```

**超时处理**: 如果 30s 内 NDT 无 TF → nav_state_manager 触发 HDL global recovery

### 4.2 导航中 NDT 跳变

```
1. NDT 检测 pose_jump:
   - translation > max_pose_jump_translation (0.4m) 
   - 或 yaw > max_pose_jump_yaw (0.3rad)
   → ndt_status{reason:"pose_jump_candidate"}

2. nav_state_manager 检测 reason != "ok" 连续 2 帧:
   → PAUSE + zero cmd_vel (30Hz)
   → cancel_navigation()
   → 等 robot 静止 (odom 位移 < 0.05m × 2s)

3. NDT 侧:
   - 停止发布新 map→odom TF
   - 保持发布 last_good TF (republish_last_good_tf_on_failure=true)
   - 继续尝试 scan matching（可能 fitness 很高, state="rejected"）

4. nav_state_manager 发 recovery 请求:
   - prior = waypoint_path_prior(前一点→当前点→下一点→投影)
   - source = "navigation_context_segment"
   - radius = 5.0m (默认)
```

### 4.3 重定位期间

```
1. HDL 收到 recovery 请求 (带 prior):
   - with prior: 限制候选在 prior 附近 5m
   - prior 过期 (>8s): fallback full global
   - 找到候选 → NDT fine-match → /initialpose

2. NDT 收到 /initialpose:
   - initialpose_relax 4s
   - 开始匹配 → 发布新 TF

3. nav_state_manager 监控恢复:
   - 等待 ndt_status state="accepted" + fitness < 0.3 连续 3 帧
   - → 恢复成功 → resume 导航

4. 如果 HDL 失败:
   - hdl_bootstrap 返回失败 → nav_state_manager 冷却 5s 后重试
   - 每次重试可扩大 prior radius (+2m per retry, cap 10m)
   - 超过 5 次 → 退回 full global (无 prior)
```

### 4.4 inlier=0 虚假健康检测

```
NDT 可能 fitness 极低 (0.003) + converged=True 但 inlier=0
→ "看起来健康, 实际在错误位置"

nav_state_manager 监控:
  if inlier == 0 AND fitness < 0.01 AND ndt_state == "accepted":
      inlier_zero_timer += dt
      if inlier_zero_timer > 30s:
          → 视为定位异常 → recovery
  else:
      inlier_zero_timer = 0
```

### 4.5 NDT 自身恢复（无需 recovery）

```
NDT 已有 pose_jump_reacquire 机制:
  pose_jump 后连续 N 帧 fitness < 0.08 + xy_tolerance < 0.5m
  → NDT 自行恢复 → reason 恢复 "ok"

此时 nav_state_manager 无需干预:
  reason 恢复 "ok" + TF 正常 → 取消 PAUSE → 恢复导航
```

---

## 5. 风险点

| # | 风险 | 等级 | 缓解 |
|---|---|---|---|
| 1 | NDT 静默漂移 (inlier=0, fitness 极低) 不触发 pose_jump | 中 | inlier=0 计时器 30s → recovery |
| 2 | NDT last_good TF 持续发布, costmap 基于错误位置更新 | 低 | robot 已停止 (PAUSED), costmap 不变化 |
| 3 | NDT TF 恢复后 robot_map 跳变 | 低 | robot 静止时跳变可接受, 新 goal 重新规划 |
| 4 | nav_state_manager 新增逻辑增加复杂度 | 低 | ~50 行新增, 远小于 fusion 的 1992 行 |
| 5 | 去掉 fusion 后丢失 TRANSITIONING 平滑过渡 | 极低 | robot 静止时直接切换, 不需要平滑 |
| 6 | NDT 假 pose_jump (正常运动产生的位姿变化) | 低 | 阈值 0.4m/0.3rad 对正常帧间运动已足够 |

### 5.1 当前实现审查新增风险

| # | 风险 | 等级 | 缓解 |
|---|---|---|---|
| 7 | NDT 拒绝跳变时 reason 是 `pose_jump`, 但 nav_state_manager 只监听 `pose_jump_candidate`/`confirmed_pose_jump` | 高 | degraded reason 必须包含 `pose_jump`; `confirmed_pose_jump` 应视为 NDT 自恢复/接受结果 |
| 8 | inlier=0 判据依赖 `inlier_fraction`, 但 NDT status 当前未发布该字段 | 高 | NDT 增加 `inlier_fraction`/`corr_count`, 或先禁用该判据 |
| 9 | launch 中 `republish_last_good_tf_on_failure` 仍可能被覆盖为 false | 高 | 所有入口统一参数: 要么启用 last_good TF, 要么文档和恢复逻辑改成不依赖冻结 TF |
| 10 | NDT 自身恢复时 nav_state_manager 可能不会 resume | 中 | `_handle_ndt_recovered()` 设置 `localization_resume_pending` 和 `localization_recovered_at`, 或只由 recovery_status 统一恢复 |
| 11 | prior 失败后没有真正切换 full global, 可能进入失败冷却 | 高 | hdl_bootstrap 消费 `allow_full_global_fallback`, prior 尝试失败后自动切到无 prior 全局搜索 |
| 12 | 旧 launch 仍可能启动 fusion | 中 | 清理/废弃所有仍引用 `localization_odom_fusion` 的 launch, 保证 map->odom 单一发布源 |

### 5.2 异常恢复链路必须满足的时序

```
NDT rejected/pose_jump
  → nav_state_manager 立即进入 PAUSED + LOCALIZATION_RECOVERY
  → 30Hz zero cmd_vel + cancel Nav2
  → 等机器人静止
  → 发布 navigation_context prior
  → HDL/SC 带 prior 搜索
  → prior 失败则 full global 搜索
  → /initialpose
  → NDT initialpose_relax
  → NDT accepted + TF fresh 连续验收
  → localization_recovered
  → resume 当前 waypoint 或执行 pending 请求
```

以上链路中任一环节缺失, 都会出现“停了但不恢复”、“恢复了但不续航”、“prior 失败后卡住”或“旧 fusion 抢 TF”的问题。

---

## 6. 优缺点

### 优点

- **代码量**: -1992 行删除, +~50 行新增, 净减少 ~1900 行
- **状态机**: 5 态 → 0 态 (NDT self-monitoring 替代)
- **TF 发布**: 单一来源 (NDT), 消除竞态
- **监控链路**: nav_state 直接看 ndt_status, 无中间层
- **Recovery 路径**: 统一为 nav_state → HDL/SC, 消除双重路径
- **崩溃容错**: NDT 崩溃 → TF 消失 → nav_state 3s 内检测 (融合心跳 5s+ 才检测)
- **调参**: 去掉 10+ fusion 参数, NDT 阈值 + nav_state 冷却时间即可

### 缺点

- nav_state_manager 职责增加 (~350 行 → ~400 行)
- 失去 fusion 的 TRANSITIONING 平滑过渡 (可接受, robot 静止时不需要)
- 失去 fusion 的锁定期机制 (NDT 自身的 initialpose_relax 替代)
- 需要 1-2 次实机测试验证新逻辑

---

## 7. 实施步骤

### Step 1: NDT 侧改动
1. `localization.yaml`: `republish_last_good_tf_on_failure: false → true`
2. 删除 `allow_ndt_tf_when_fusion_timeout` 参数
3. 删除 (C++) `fusion_status_timeout` 参数

### Step 2: nav_state_manager 改动
1. 删除 `_on_fusion_status()` 回调 + `BLOCKED_STATES` 变量 + `_check_fusion_status_timeout()` 定时器
2. 删除 `_handle_localization_blocked()` / `_handle_fusion_recovered()` 
3. 新增 `_on_ndt_status()` 回调:
   - 解析 ndt_status JSON
   - 检测 pose_jump (reason != "ok" 连续 2 帧) → PAUSE
   - 检测 inlier=0 (inlier==0 + fitness<0.01 × 30s) → recovery
   - 检测 TF 超时 → recovery
4. 新增 `_check_ndt_recovery()` 定时器:
   - 检测 recovery 是否成功 (fitness < 0.3 + TF 可用 连续 3 帧)
   - → resume 导航
5. `request_navigation_context_recovery_for_localization()` 保留并作为唯一 recovery 路径
6. 删除 `enforce_localization_stop` 定时器中的 fusion_status 检查 (保留 PAUSED + LOCALIZATION_RECOVERY 检查)

### Step 3: launch 文件改动
1. 删除 `localization_odom_fusion` Node 启动
2. 删除 fusion 相关参数传递

### Step 4: hdl_bootstrap 清理
1. 删除 fusion 专用的 recovery 请求路径 (保留通用路径 + P0-3)

### Step 5: 测试验证
1. 冷启动定位 → 导航
2. NDT 跳变 (模拟) → 暂停 → recovery → 恢复
3. inlier=0 检测 (长时间导航) → recovery
4. TF 超时 → recovery
