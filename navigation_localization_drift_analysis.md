# 导航定位漂移分析报告

## 结论

本次在点位6附近触发暂停时，机器人实际位姿与点位6坐标基本一致。问题不是“点位6先验给错”，而是 recovery 流程中先验在真正候选选择时过期失效，系统退化成全局重定位，并选中了远离点位6的候选。随后错误候选污染了 `frozen_tf_chain`，后续 recovery 又围绕错误 prior 继续搜索，导致定位没有纠正回来。

## 关键文件

- 日志文件：`/home/ubuntu/humanoid_ws/debug_output1.txt`
- 路点文件：`/home/ubuntu/humanoid_ws/data/dynamic_waypoints.json`

## 点位6坐标核对

`dynamic_waypoints.json` 中存在两套同名点位：

- 旧点位6：`id=917`，坐标 `(-4.398, -5.754, 0)`。
- 本次日志实际使用的点位6：`id=941`，坐标 `(14.889, 18.359, 0)`。

日志中 Nav2 到点位6时的目标是：

```text
Begin navigating from current location (12.19, 17.97) to (14.89, 18.36)
```

因此本次运行对应的是 `id=941` 这套点位。点位5为 `(12.052, 18.129, 0)`，点位6为 `(14.889, 18.359, 0)`。

## 时间线

### 14:31:11 左右，开始导航到点位6

日志显示：

```text
开始导航到路点: 点位6 (1/1)
Begin navigating from current location (12.19, 17.97) to (14.89, 18.36)
```

这说明系统认为机器人从点位5附近出发，目标为点位6。

### 14:31:22，点位6附近触发定位退化并自动暂停

关键日志：

```text
Holding NDT pose jump candidate for confirmation: translation=0.417 normal_limit=0.400
检测到 NDT 退化: ndt_reason=pose_jump_candidate, correction=0.417m>0.35m
[HEALTHY→DEGRADED] 冻结 map->odom
冻结值: (5.669, 11.349, yaw=1.47rad)
[FUSION] 定位进入 DEGRADED 状态, 暂停导航 + zero cmd hold
```

此时 Nav2 已经接近点位6。后续 LOST 时 recovery prior 为：

```text
prior: x=15.15, y=18.23, radius=2.1m, source=frozen_tf_chain
```

该 prior 距点位6 `(14.889, 18.359)` 约 0.29m，说明暂停时机器人确实在点位6附近。

### 14:31:32，进入 LOST 并触发 recovery

关键日志：

```text
[DEGRADED→LOST] 触发 recovery
最后健康 map->odom: (5.669, 11.349)
已发送 recovery 请求 ... prior: x=15.15, y=18.23
定位异常后请求 HDL 使用导航上下文重定位: prior=navigation_context_segment, prev=点位5, current=点位6
```

这里有两类先验：

- `frozen_tf_chain`：点位6附近，`x=15.147, y=18.230`。
- `navigation_context_segment`：根据点位5到点位6的导航上下文生成。

但是导航上下文请求被忽略了：

```text
ignore duplicate localization recovery request while relocalization is already pending
```

也就是说，更合理的“点位5到点位6线段先验”没有真正接管这次 recovery。

### 14:31:36 到 14:31:39，先验过期，HDL 退回全局候选

HDL 开始带 prior 重定位：

```text
calling HDL /relocalize_with_prior_checked attempt 1
external recovery prior received: pose=(15.1475, 18.2299, yaw=0), max_xy_override=5
```

但候选计算完成时：

```text
ignore stale external recovery prior: age=3.12285s > 3s
recovery prior requested but no usable prior is available; using full global candidates
```

这是本次跑飞的关键点。先验是对的，但 TTL 只有 3 秒，HDL 全局候选计算耗时超过 3 秒，导致先验在筛选阶段被判定 stale。系统于是改用全局候选，不再限制在点位6附近。

随后选择了全局最优候选：

```text
Best global localization candidate[2] fitness=0.0521842 ... pose=(3.26753, 12.4484, yaw=-0.605588)
prepared A initial pose from HDL bootstrap/recovery: map->odom=(5.423, 0.833, yaw=2.600)
initialpose input: frame=map x=5.423 y=0.833
```

该结果明显不在点位6附近，属于错误重定位候选。

### 14:31:49，错误结果被软验收拒绝，但状态已经被污染

日志：

```text
recovery_delta=9.72m > allowed=5.11m, odom_displacement=0.11m, NDT error=0.1083
recovery 软验收拒绝本次结果，继续请求 SC 重定位
```

软验收拒绝了这次结果，这是合理的。但之后系统继续使用 `frozen_tf_chain` 派生 prior，且 pending/recovery 状态没有干净回滚到点位6上下文。

### 后续，prior 逐步偏到点位4/5附近

后面日志出现：

```text
prior: x=10.58, y=18.27, radius=2.0m, source=frozen_tf_chain
```

这个位置更接近点位4/点位5区域，而不是点位6。说明先前错误 recovery 结果或错误 TF 链已经污染了后续 frozen prior。之后系统就围绕错误区域继续重定位，导致用户看到“跑到点位4、5那边”。

## 为什么“带着先验”仍然跑偏

根因是先验没有在候选选择阶段生效：

1. 点位6先验本身是正确的。
2. HDL 查询耗时超过了先验 TTL：`age=3.12285s > 3s`。
3. 代码判定 prior stale 后，直接使用 full global candidates。
4. 全局候选里选中了远离点位6的位置。
5. 导航上下文先验请求由于已有 recovery pending 被忽略，没能替换过期 prior。
6. 后续 frozen prior 被错误结果污染，重定位开始围绕点位4/5附近反复尝试。

## 直接风险点

- `external recovery prior` 的有效期 3 秒太短，无法覆盖一次完整 HDL 全局候选计算。
- `navigation_context_segment` 请求在已有 recovery pending 时被直接忽略，无法刷新 prior。
- 先验过期后自动退回 full global candidates，缺少“必须使用先验，否则本次失败”的保护。
- `NDT error` 低但 `inlier=0.000` 长期存在，说明匹配质量指标不一致，容易产生虚假健康。
- 软验收拒绝错误候选后，后续 prior 仍可能被错误 TF/frozen pose 污染。

## 建议修复

1. 延长 recovery prior TTL。

   建议至少覆盖一次 HDL 查询耗时，例如从 3 秒提高到 8-10 秒，或者让 prior 在一次 service call 生命周期内保持有效。

2. 已有 recovery pending 时允许刷新 prior。

   `navigation_context_segment` 比旧的 `frozen_tf_chain` 更符合业务上下文。已有 pending 时不应直接丢弃，应允许更新当前 recovery 的 prior。

3. 对带 prior 的 recovery 增加硬约束。

   如果请求是 `/relocalize_with_prior_checked`，prior 过期或不可用时，不应自动退回 full global candidates。更安全的行为是返回失败，让上层重新发 fresh prior。

4. recovery 软验收拒绝后清理错误候选影响。

   软验收拒绝说明本次候选不可接受，应避免它污染后续 `frozen_tf_chain` 或 recovery prior。

5. 修复 NDT 健康判据。

   需要排查为什么 `NDT error` 很低时 `inlier=0.000`。在 inlier 长期为 0 的情况下，不应仅凭低 error 判定可恢复或健康。

## 一句话总结

点位6暂停时机器人大概率就在点位6附近；跑到点位4/5附近不是先验坐标错，而是先验在 HDL 候选选择前过期失效，系统退回全局重定位并选错候选，之后错误 TF/frozen prior 又把后续 recovery 带偏。
