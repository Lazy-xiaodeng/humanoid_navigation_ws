# NDT + Odom 接管 + ScanContext 重定位组合方案分析

> 创建时间: 2026-05-25  
> 工作空间: `/home/ubuntu/humanoid_ws`  
> 目的: 分析将 odom 短期接管方案与 ScanContext context 重定位方案融合后的可行性、风险和推荐落地路径

---

## 一、目标方案

用户期望的最终逻辑是：

```text
NDT 正常定位维护 TF 并导航
    ↓
NDT 漂移 / 匹配异常
    ↓
odom 接管，冻结 map->odom，继续导航，不暂停，不上报 APP
    ↓
NDT 持续尝试匹配
    ├─ NDT 自然恢复
    │     ↓
    │   NDT 重新接管，odom 退出接管
    │
    └─ NDT 超过时间/距离阈值仍未恢复
          ↓
        判定 LOST / 恢复失败
          ↓
        odom/fusion 发布全局重定位请求
          ↓
        ScanContext 收到请求后开始重定位
          ↓
        ScanContext 发布初始位姿给 NDT
          ↓
        NDT 验证
          ├─ 验证通过: NDT 接管，导航恢复
          └─ 验证失败: ScanContext 继续重定位直到恢复
```

这是一套三层降级架构：

| 层级 | 职责 | 目标 |
|---|---|---|
| 正常层 | NDT 维护 `map->odom` | 正常定位和导航 |
| 短期兜底层 | odom 接管，冻结 `map->odom` | NDT 短时漂移时不中断导航 |
| 全局恢复层 | ScanContext 重定位，NDT 验证 | odom 兜底失败后恢复全局定位 |

---

## 二、总体可行性结论

这套组合方案**架构上可行**，并且比“NDT 一漂就立刻全局重定位”更合理。

核心优势：

1. NDT 短时不稳定时，不立刻暂停导航。
2. Fast-LIO odom 在短时间、短距离内通常比错误 NDT 跳变更可信。
3. 只有 odom 兜底失败后才触发 ScanContext，减少全局误匹配机会。
4. ScanContext 只提供 `/initialpose`，最终仍由 NDT 验证，避免单次候选直接污染 TF。

但这套方案不能简单把现有 fusion、NDT、ScanContext 三套逻辑并行打开。关键前提是：

```text
必须明确 map->odom 的唯一发布权
必须明确状态机的单一仲裁者
必须明确 NDT 恢复和 ScanContext 恢复的验收条件
```

如果这些边界没有统一，会出现 TF 竞争、重复 recovery、错误恢复、状态振荡等问题。

---

## 三、推荐最终架构

### 3.1 模块职责

| 模块 | 推荐职责 |
|---|---|
| NDT | 做 scan-to-map 匹配，输出 pose/status，不直接决定系统恢复状态 |
| Odom / Fast-LIO | 在短期 DEGRADED 阶段提供连续运动 |
| Fusion 节点 | 系统定位状态机仲裁者，最好独占发布 `map->odom` |
| ScanContext | 全局候选生成器，只在 recovery 请求后工作 |
| scancontext_to_initialpose | 将 SC 候选转换为 `/initialpose`，等待 NDT 验证 |
| APP 状态管理器 | DEGRADED 不上报，LOST/RECOVERING 才上报或暂停 |

### 3.2 理想 TF 所有权

最推荐：

```text
fusion 节点是唯一 map->odom 发布者
```

具体含义：

```text
HEALTHY:
    fusion 接收 NDT pose / map->odom 候选
    fusion 转发为正式 map->odom

DEGRADED_ODOM_HOLD:
    fusion 冻结最后健康的 map->odom
    odom->base 继续由 Fast-LIO 传播

RECOVERING:
    fusion 继续冻结或停止发布，等待恢复结果

RECOVERED_TRANSITION:
    fusion 平滑从冻结 map->odom 过渡到 NDT 验证后的 map->odom
```

不推荐：

```text
NDT 和 fusion 同时发布 map->odom
```

原因：

- tf2 会看到同一 parent/child 的多源变换
- 30Hz 覆盖 10Hz 不是严格仲裁
- 时间戳和缓存可能导致局部跳变
- 恢复时 NDT、fusion、ScanContext 间接同时影响 TF

---

## 四、推荐状态机

不建议只用 `HEALTHY / DEGRADED / LOST` 三态。组合方案更适合拆成以下状态：

```text
HEALTHY
    NDT 正常，fusion 转发 NDT 定位

DEGRADED_ODOM_HOLD
    NDT 异常，冻结 map->odom，odom 继续传播 base

NDT_REACQUIRE_CHECK
    NDT 看似恢复，进入连续验证阶段

LOST_REQUEST_RECOVERY
    odom 兜底超过时间/距离阈值，发布 recovery request

SC_RECOVERING
    ScanContext 正在尝试普通/全局重定位

NDT_VALIDATE_INITIALPOSE
    ScanContext 已发布 /initialpose，等待 NDT 验证

RECOVERED_TRANSITION
    NDT 验证通过，fusion 平滑切回 HEALTHY
```

### 4.1 状态转移

```text
HEALTHY
  └─ NDT error > degraded_threshold 连续 N 帧
       → DEGRADED_ODOM_HOLD

DEGRADED_ODOM_HOLD
  ├─ NDT 满足恢复候选条件
  │    → NDT_REACQUIRE_CHECK
  ├─ odom 位移 > max_odom_displacement
  │    → LOST_REQUEST_RECOVERY
  ├─ DEGRADED 时间 > max_degraded_duration
  │    → LOST_REQUEST_RECOVERY
  └─ 累计 odom 位移 > max_total_odom_displacement
       → LOST_REQUEST_RECOVERY

NDT_REACQUIRE_CHECK
  ├─ 连续 accepted 且与 odom 预测一致
  │    → RECOVERED_TRANSITION
  └─ 任一验证失败
       → DEGRADED_ODOM_HOLD

LOST_REQUEST_RECOVERY
  └─ 发布 /localization/recovery_requests
       → SC_RECOVERING

SC_RECOVERING
  ├─ ScanContext 找到候选并发布 /initialpose
  │    → NDT_VALIDATE_INITIALPOSE
  └─ 超时或失败
       → 继续 SC_RECOVERING

NDT_VALIDATE_INITIALPOSE
  ├─ NDT 连续 accepted
  │    → RECOVERED_TRANSITION
  └─ NDT rejected
       → SC_RECOVERING

RECOVERED_TRANSITION
  └─ 平滑过渡完成
       → HEALTHY
```

---

## 五、关键验收条件

### 5.1 NDT 漂移判定

只看 `matching_error` 不够，建议组合判断：

| 条件 | 建议 |
|---|---|
| `matching_error > 0.5` | 连续 2 帧进入 DEGRADED |
| `has_converged=false` | 如果 error 同时异常，进入 DEGRADED |
| pose jump | 超过 0.8m / 0.45rad 拒绝 |
| inlier fraction | 低于阈值时禁止恢复 HEALTHY |

### 5.2 NDT 自然恢复判定

不能只看 `matching_error < 0.15`。建议至少满足：

```text
matching_error < healthy_threshold
has_converged = true
inlier_fraction 足够高
连续 accepted >= 3 帧
NDT pose 与 frozen_map_odom + current_odom 推算位置差 < 阈值
NDT yaw 与 odom 推算 yaw 差 < 阈值
```

原因：几何混叠区域可能出现“低 error 但错位置”的伪恢复。

### 5.3 ScanContext 恢复验收

ScanContext 不能直接恢复系统，只能给 NDT 初始位姿。

必须保留：

```text
SC candidate
    ↓
GICP / 多帧一致性
    ↓
/initialpose
    ↓
NDT 连续 accepted
    ↓
与 odom/frozen pose 一致性检查
    ↓
fusion 才允许进入 RECOVERED_TRANSITION
```

如果 NDT rejected：

```text
清空 pending initialpose
发布 localization_relocalize_failed
继续 ScanContext recovery
```

---

## 六、主要风险和弊端

### 6.1 TF 发布权冲突

这是最大风险。

当前已有代码中：

- NDT 会发布 `map->odom`
- fusion DEGRADED 时也会发布 `map->odom`
- ScanContext 通过 `/initialpose` 间接让 NDT 更新 `map->odom`

如果不重构，可能出现：

```text
NDT 发布错误 map->odom
fusion 发布冻结 map->odom
tf2 缓存收到两个来源
下游查询到的 TF 随时间戳跳变
Nav2 / costmap / RViz 出现瞬移
```

规避方案：

```text
fusion 独占发布 map->odom
NDT 只提供候选 pose/status
或者至少在 DEGRADED/RECOVERING 时禁用 NDT TF 生效
```

### 6.2 NDT 不一定自然恢复

DEGRADED 期间，NDT 继续匹配是必要的，但不能假设它一定会恢复。

可能情况：

- NDT initial guess 已经被历史状态污染
- 局部几何混叠导致一直收敛到错误位置
- error 降低但 pose 是错的
- NDT 短暂 accepted 后马上 rejected

所以需要 `NDT_REACQUIRE_CHECK` 独立状态，而不是直接 DEGRADED -> HEALTHY。

### 6.3 odom 兜底不能无限相信

Fast-LIO 短期可靠，但不是全局真值。

风险：

- 长距离累积平移漂移
- 多次原地旋转导致 yaw 漂移
- 走廊中 yaw 小误差会放大全局路径误差
- planner 在 map 下工作，误差过大时路径可能偏离真实环境

建议保留：

| 限制 | 示例 |
|---|---:|
| 单段 odom 位移 | 30m |
| DEGRADED 导航中超时 | 120s |
| 静止播报超时 | 600s |
| 累计 odom 位移 | 100m |

### 6.4 APP 不上报会掩盖故障

DEGRADED 不上报 APP 是合理的，但系统内部必须可见。

建议：

```text
DEGRADED:
    不暂停导航
    不向 APP 上报定位异常
    但发布 /localization/fusion_status
    内部日志 warning

LOST / SC_RECOVERING:
    暂停或降速
    可以向 APP 上报定位恢复中
```

### 6.5 ScanContext 当前成功率还不够高

基于 `nav_drift_test2` 的离线验证：

| 指标 | 结果 |
|---|---:|
| ScanContext 直接 accepted | 64.8% |
| 随机 12 点漂移仿真，1m NDT 接受阈值 | 66.7% |
| 随机 12 点漂移仿真，2m NDT 接受阈值 | 75.0% |

明显风险点：

| 点位 | 风险 |
|---|---|
| 点位4 | 可能跳到点位24 附近 |
| 点位6 | 可能跳到点位9 附近 |
| 点位9 | 恢复误差仍偏大 |
| 点位2 | 2m 阈值可过，但候选接近点位3，可信度一般 |

所以 ScanContext 适合做 LOST 后恢复器，但不能无条件信任。

### 6.6 状态振荡风险

可能出现：

```text
DEGRADED
  → NDT 短暂 accepted
  → HEALTHY
  → 立即 DEGRADED

SC_RECOVERING
  → 发布 /initialpose
  → NDT rejected
  → 再次 SC_RECOVERING

NDT_VALIDATE_INITIALPOSE
  → NDT accepted 但位置错误
  → 错误进入 HEALTHY
```

需要通过连续帧、冷却时间、最小 publish interval、pose 一致性 gate 限制。

---

## 七、推荐落地顺序

### 7.1 第一阶段: 明确 TF 所有权

目标：

```text
fusion 成为唯一 map->odom 发布者
```

推荐改造：

1. NDT 输出 pose/status，但不直接发最终 TF；或提供开关禁用 TF。
2. fusion 在 HEALTHY 时转发 NDT 位姿。
3. fusion 在 DEGRADED 时冻结 `map->odom`。
4. fusion 在 RECOVERING 时保持冻结或按策略停止发布。

验收：

```text
ros2 topic echo /tf
确认 map->odom 只有一个发布源
```

### 7.2 第二阶段: 完善 DEGRADED 兜底

目标：

```text
NDT 异常后导航不停，APP 不上报，odom 短期接管
```

需要验证：

- DEGRADED 进入是否及时
- `map->base` 是否连续
- local_costmap/controller 是否稳定
- APP 是否没有收到定位异常
- 超过阈值是否进入 LOST

### 7.3 第三阶段: 接入 ScanContext recovery

目标：

```text
fusion LOST 后发布 /localization/recovery_requests
scancontext_to_initialpose 收到后开始 SC recovery
```

需要保证：

- DEGRADED 阶段不触发 SC
- LOST 后才触发 SC
- SC 不发布 TF
- SC 只发布 `/initialpose`
- NDT 验证通过后 fusion 才恢复

### 7.4 第四阶段: 做完整 bag 验证

需要录一包真实开启组合方案的 bag，至少验证：

| 项 | 通过标准 |
|---|---|
| TF 连续性 | `map->base` 无异常大跳 |
| DEGRADED 接管 | NDT 漂移后 2 帧左右进入 |
| 导航不中断 | DEGRADED 中无 PAUSED |
| APP 行为 | DEGRADED 不上报，LOST/RECOVERING 可上报 |
| LOST 触发 | 超时/超距后发布 request |
| SC 恢复 | 发布 `/initialpose` |
| NDT 验证 | 连续 accepted 后恢复 |
| 误恢复率 | 不接受点位4/6/9 的错误候选 |

---

## 八、当前代码状态判断

### 8.1 已有基础

当前工作空间已经具备大部分组件：

| 组件 | 状态 |
|---|---|
| NDT 定位 | 已有 |
| ScanContext sidecar | 已有 |
| scancontext_to_initialpose | 已有 |
| `/localization/recovery_requests` | 已有 |
| `/localization/recovery_status` | 已有 |
| odom fusion 节点 | 已有，已修复语法错误 |
| navigation_fusion launch | 已有 |

### 8.2 尚未满足的关键前提

| 问题 | 当前风险 |
|---|---|
| `map->odom` 发布权 | NDT 和 fusion 可能同时发布 |
| NDT 自然恢复验收 | 需要加强一致性判断 |
| SC 恢复验收 | 需要由 fusion 统一确认后切状态 |
| APP 状态抑制 | DEGRADED/LOST 边界要明确 |
| 实机组合 bag | 还没有真实开启组合方案验证 |

---

## 九、最终建议

这套方案可以作为最终主线继续推进，但不要直接把三套逻辑并行打开。

推荐最终策略：

```text
1. fusion 统一仲裁定位状态
2. fusion 独占 map->odom
3. NDT 只作为局部 scan matching 结果来源
4. odom 只在 DEGRADED 中短期接管
5. ScanContext 只在 LOST 后作为全局恢复候选来源
6. NDT 验证通过后 fusion 平滑恢复 HEALTHY
```

不推荐：

```text
NDT、fusion 同时发布 map->odom
DEGRADED 阶段立即触发 ScanContext
ScanContext 单次候选直接恢复 HEALTHY
只看 matching_error 判断 NDT 恢复
长时间 odom 接管但不上报任何内部状态
```

如果按推荐顺序落地，这条路线是可行的；如果直接拼接，主要风险会集中在 TF 竞争、误恢复和状态振荡。
