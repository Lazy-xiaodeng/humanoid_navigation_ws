# 多帧 NDT 离线回放模拟报告

> 结论版本: filtered `/tf` replay  
> 生成时间: 2026-05-31 00:45 CST  
> 回放方式: 1.0x 完整回放，过滤 bag 中旧 `map->odom`，只让 NDT 自己发布新的 `map->odom`

---

## 1. 一句话结论

多帧 NDT 在这三个 bag 上**不是稳定无漂的方案**。它大多数时间能正常跟踪，但三个包都出现了低 fitness 下的大平移跳变候选；现有兜底机制确实会拦截大量跳变、重发 last-good TF，并在 last-good 过期后进入等待重定位状态。  

因此答案是:

| 问题 | 结论 |
|---|---|
| 多帧 NDT 在同环境会不会飘? | 会。三包均出现 `pose_jump`，test19 后段最明显。 |
| 飘了会不会被兜底? | 会。`pose_jump` gate、`rotation_guard`、last-good TF 都生效。 |
| 会不会暂停导航? | 本次只启动 NDT 回放，没有启动状态管理器，所以不会真实发布 `navigation_auto_paused`。但 NDT 已多次进入需要重定位的状态，完整链路下应触发暂停/恢复流程。 |
| 会不会触发重定位? | NDT 日志中已出现 `last good TF is too old, waiting for relocalization`。若启动 recovery bridge/state manager，应进入重定位链路。 |

---

## 2. 测试设计

### 2.1 为什么要过滤 `/tf`

第一次直接回放 bag 时，bag 内旧定位节点录下的 `map->odom` 会和 NDT 新发布的 `map->odom` 混在一起，导致 TF 跳变统计被污染。

最终采用的干净回放链路:

```text
ros2 bag play /tf  ->  /tf_bag_raw
                         |
                         v
                 filter out map->odom
                         |
                         v
                       /tf  -> NDT 使用 Fast-LIO TF

NDT 自己发布 map->odom -> /tf -> report recorder
```

### 2.2 NDT 参数

| 参数 | 值 |
|---|---:|
| `multi_frame_matching_enabled` | `true` |
| `multi_frame_window_sec` | `1.2s` |
| `multi_frame_max_frames` | `12` |
| `multi_frame_voxel_leaf_size` | `0.20m` |
| `rotation_guard_enabled` | `true` |
| `max_pose_jump_translation` | `0.50m` |
| `score_threshold` | `0.3` |
| `pose_jump_reacquire_required_frames` | `4` |

### 2.3 重要边界

这是离线影子回放，不是闭环实机复跑。它验证的是同一批点云、TF、导航状态输入下，多帧 NDT 会输出什么，以及保护策略会如何响应。  
它不能证明如果当时真的用多帧 NDT，机器人会走出完全相同的轨迹，因为 Nav2 控制闭环会被新的定位输出改变。

---

## 3. 总览

| Bag | 时长 | NDT帧 | Accepted | Rejected | Confirming | 最大NDT TF跳变 | pose_jump拒绝 | rotation_guard | last-good过期等待重定位 | 结论 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| `navtest0` | 682.5s | 3104 | 69.7% | 29.6% | 21 | 0.837m | 407 | 0 | 66 | 不适合直接作为稳定定位源 |
| `nav_drift_test17` | 1201.3s | 5150 | 94.5% | 3.2% | 119 | 1.225m | 164 | 64 | 3 | 大部分可跟踪，但会跳变 |
| `nav_drift_test19` | 536.1s | 2456 | 88.7% | 8.1% | 78 | 1.091m | 199 | 58 | 6 | 后段出现明显漂移/恢复窗口 |

---

## 4. 关键指标对比

### 4.1 匹配质量

| Bag | fitness p50 | fitness p90 | fitness p99 | accepted fitness p99 | 说明 |
|---|---:|---:|---:|---:|---|
| `navtest0` | 0.0035 | 0.6827 | 2.2820 | 0.0962 | 大量高 fitness，被拒绝较多 |
| `nav_drift_test17` | 0.0018 | 0.0051 | 0.1614 | 0.0845 | 整体匹配质量最好 |
| `nav_drift_test19` | 0.0020 | 0.0567 | 0.1241 | 0.1074 | 后段退化明显 |

### 4.2 correction 平移

| Bag | correction p50 | correction p90 | correction p99 | max correction | accepted max |
|---|---:|---:|---:|---:|---:|
| `navtest0` | 0.008m | 1.713m | 3.515m | 4.021m | 0.837m |
| `nav_drift_test17` | 0.009m | 0.091m | 1.003m | 1.549m | 1.225m |
| `nav_drift_test19` | 0.016m | 0.743m | 2.214m | 2.732m | 1.091m |

### 4.3 多帧实际使用情况

| Bag | 多帧 p50 | 多帧 p90 | 多帧 max | 点数 p50 | 点数 p90 |
|---|---:|---:|---:|---:|---:|
| `navtest0` | 5帧 | 6帧 | 11帧 | 6972 | 11354 |
| `nav_drift_test17` | 4帧 | 6帧 | 11帧 | 7178 | 10164 |
| `nav_drift_test19` | 5帧 | 7帧 | 12帧 | 7941 | 12653 |

---

## 5. 分包分析

### 5.1 `navtest0`

```text
bag: /home/ubuntu/nav_drift_test/navtest0
NDT JSONL: debug_monitor/multiframe_ndt_filtered_navtest0_20260530/navtest0/ndt_status.jsonl
```

| 指标 | 数值 |
|---|---:|
| NDT状态帧 | 3104 |
| accepted | 2164 |
| rejected | 919 |
| high_fitness | 512 |
| pose_jump | 407 |
| confirmed_pose_jump | 3 |
| rotation_guard | 0 |
| last-good过期等待重定位 | 66 |

**判断**

- 这个包没有 `/cmd_vel`，所以 rotation guard 无法靠速度触发。
- 多帧 NDT 出现大量 `high_fitness` 和 `pose_jump`，说明环境或初始/TF条件下定位并不稳定。
- 最大 correction 达到 4.021m，被 pose_jump gate 拦截。
- accepted 最大 TF 跳变仍有 0.837m，说明 confirmed/reacquire 放行仍可能带来较大定位修正。

**最大异常**

| 时间戳 | 状态 | 原因 | correction | yaw | fitness |
|---:|---|---|---:|---:|---:|
| 1780123235.647 | rejected | pose_jump | 4.021m | 0.161rad | 0.0293 |
| 1780123235.147 | rejected | pose_jump | 4.018m | 0.157rad | 0.0352 |
| 1780123236.547 | rejected | pose_jump | 4.017m | 0.166rad | 0.0282 |

### 5.2 `nav_drift_test17`

```text
bag: /home/ubuntu/nav_drift_test/nav_drift_test17
NDT JSONL: debug_monitor/multiframe_ndt_filtered_test17_20260530/nav_drift_test17/ndt_status.jsonl
```

| 指标 | 数值 |
|---|---:|
| NDT状态帧 | 5150 |
| accepted | 4867 |
| rejected | 164 |
| pose_jump | 164 |
| pose_jump_candidate | 55 |
| confirmed_pose_jump | 12 |
| rotation_guard | 64 |
| last-good过期等待重定位 | 3 |

**判断**

- 整体接受率 94.5%，多帧 NDT 大部分时间能跟住。
- 但低 fitness 大跳变仍然存在，最大 correction 1.549m。
- rotation guard 有 64 次 hold/settle，说明导航/旋转窗口内确实拦住了一部分跳变。
- last-good 过期等待重定位出现 3 次，代表兜底不是无限维持，超时后需要 recovery bridge 接管。

**最大异常**

| 时间戳 | 状态 | 原因 | correction | yaw | fitness |
|---:|---|---|---:|---:|---:|
| 1780140439.478 | rejected | pose_jump | 1.549m | 0.059rad | 0.0526 |
| 1780140439.573 | rejected | pose_jump | 1.544m | 0.059rad | 0.0535 |
| 1780140462.374 | rejected | pose_jump | 1.512m | 0.092rad | 0.0151 |

### 5.3 `nav_drift_test19`

```text
bag: /home/ubuntu/nav_drift_test/nav_drift_test19
NDT JSONL: debug_monitor/multiframe_ndt_filtered_test19_20260530/nav_drift_test19/ndt_status.jsonl
```

| 指标 | 数值 |
|---|---:|
| NDT状态帧 | 2456 |
| accepted | 2179 |
| rejected | 199 |
| pose_jump | 199 |
| pose_jump_candidate | 20 |
| confirmed_pose_jump | 5 |
| rotation_guard | 58 |
| last-good过期等待重定位 | 6 |

**判断**

- test19 是三个包里最能说明问题的包之一：接受率 88.7%，pose_jump 199 次。
- 后段出现连续 0.5m 到 2.7m correction，被 pose_jump gate 大量拒绝。
- rotation guard 多次拦截，日志中明确出现 last-good TF 重发。
- 也出现 `Localization rejected scans ... last good TF is too old, waiting for relocalization`，说明如果完整 recovery 链路在线，应进入暂停导航/重定位流程。

**最大异常**

| 时间戳 | 状态 | 原因 | correction | yaw | fitness |
|---:|---|---|---:|---:|---:|
| 1780149206.352 | rejected | pose_jump | 2.732m | 0.129rad | 0.0269 |
| 1780149207.557 | rejected | pose_jump | 2.692m | 0.127rad | 0.0519 |
| 1780149210.952 | confirming | rotation_guard_hold | 2.684m | 0.124rad | 0.0675 |

---

## 6. 对你关心问题的直接回答

### 6.1 多帧 NDT 在同样环境表现怎么样?

表现分裂:

- test17 大多数时间表现不错，接受率 94.5%。
- test19 明显退化，接受率降到 88.7%，pose_jump 199 次。
- navtest0 的 NDT 表现最差，拒绝率 29.6%。

所以多帧 NDT 不是“换上就稳”的定位主方案。

### 6.2 会不会飘?

会。证据:

- `navtest0`: max correction 4.021m。
- `nav_drift_test17`: max correction 1.549m。
- `nav_drift_test19`: max correction 2.732m。

这些不是普通小修正，而是足以造成导航错位的大跳变。

### 6.3 飘了会不会兜底?

会，而且多层兜底都生效:

- `pose_jump` gate 拒绝大平移跳变。
- `rotation_guard_hold/settle` 在旋转/settle 窗口拦截跳变。
- last-good TF 被重发，避免 TF 树立刻断掉。
- last-good 超时后不再强撑，进入等待重定位状态。

但兜底不是完美的:

- `confirmed_pose_jump` 仍会放行部分大修正。
- accepted max TF jump 在三个包分别达到 0.837m、1.225m、1.091m。
- 这说明目前阈值/确认策略仍可能允许较大定位重捕获，对导航闭环有冲击。

### 6.4 会不会暂停导航并触发重定位?

本次没有启动 `navigation_state_manager` 和 recovery bridge，所以不会真实看到 `navigation_auto_paused` 事件。  

但从 NDT 侧看，触发条件已经出现:

| Bag | `last good TF is too old, waiting for relocalization` |
|---|---:|
| `navtest0` | 66 |
| `nav_drift_test17` | 3 |
| `nav_drift_test19` | 6 |

如果完整导航链路在线，合理预期是:

1. NDT 连续拒绝或 last-good 超时。
2. 状态管理器发现定位恢复状态。
3. 导航自动暂停。
4. recovery bridge 调用 ScanContext/HDL 重定位。
5. NDT 通过 `/initialpose` 重新验收后恢复导航。

---

## 7. 工程判断

### 7.1 当前多帧实现的关键风险

当前 `buildMultiFrameSource()` 会合并最近多帧并体素降采样，但历史帧没有运动补偿。机器人移动或转弯时，历史帧和当前帧存在空间拖影，多帧有时会增强几何约束，有时会让退化匹配更自信。

这能解释为什么一些错误匹配 fitness 很低，但 correction 很大。

### 7.2 是否建议直接切回多帧 NDT 导航?

不建议直接切回作为主定位。更合理的策略是:

1. 保留当前新定位节点作为主链路。
2. 多帧 NDT 作为影子诊断或辅助验收。
3. 若要让 NDT 上线，先实现历史帧运动补偿。
4. 收紧 `confirmed_pose_jump` 放行条件，避免 0.8m 到 1.2m 的 accepted TF jump。
5. 完整回放时同时启动状态管理器和 recovery bridge，验证真实 `navigation_auto_paused` 与自动恢复链路。

---

## 8. 输出文件

| 内容 | 路径 |
|---|---|
| 回放脚本 | `tools/multiframe_ndt_bag_replay_report.py` |
| navtest0 分包报告 | `debug_monitor/multiframe_ndt_filtered_navtest0_20260530/multiframe_ndt_replay_report.md` |
| test17 分包报告 | `debug_monitor/multiframe_ndt_filtered_test17_20260530/multiframe_ndt_replay_report.md` |
| test19 分包报告 | `debug_monitor/multiframe_ndt_filtered_test19_20260530/multiframe_ndt_replay_report.md` |
| 本总报告 | `debug_monitor/multiframe_ndt_filtered_final_20260530/report.md` |

