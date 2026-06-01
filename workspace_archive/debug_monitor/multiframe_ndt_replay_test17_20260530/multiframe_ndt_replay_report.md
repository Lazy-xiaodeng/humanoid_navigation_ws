# 多帧 NDT Bag 回放模拟报告

- 生成时间: `2026-05-30 23:44:09`
- 回放倍率: `1.0x`
- 每包截断: `未截断，完整回放`
- 输入: `/fast_lio/cloud_registered` + `/tf` + `/tf_static` + `/navigation/status` + `/cmd_vel` + `/odom`
- NDT 模式: `multi_frame_matching_enabled=true`, `window=1.2s`, `max_frames=12`, `rotation_guard_enabled=true`

> 说明: 这是离线影子回放。它能验证同一批传感器数据下多帧 NDT 的匹配、拒绝、旋转保护和 TF 输出表现；不能完全替代实机闭环，因为 Nav2 控制不会根据新的 NDT 输出重新生成真实运动。

## 总览

| Bag | NDT帧 | 接受率 | 拒绝率 | 最大TF跳变 | pose_jump | rotation_guard | 多帧p50/p90 | 结论 |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| nav_drift_test17 | 7597 | 97.8% | 0.3% | 4.093m | 20 | 102 | 6/7 | 触发跳变拒绝 |

## 关键判断

- `nav_drift_test17`: 接受 7432/7597 帧，最大 map->odom TF 跳变 4.093m / 0.124rad，pose_jump=20，rotation_guard=102。

## 分包详情

### nav_drift_test17

- Bag: `/home/ubuntu/nav_drift_test/nav_drift_test17`
- 原始 NDT JSONL: `debug_monitor/multiframe_ndt_replay_test17_20260530/nav_drift_test17/ndt_status.jsonl`
- 初始 map->odom: x=0.006, y=0.007, z=0.000, yaw=-0.165rad
- NDT 时间跨度: `1201.9s`

| 指标 | 数值 |
|---|---:|
| NDT状态帧 | 7597 |
| accepted | 7432 (97.8%) |
| rejected | 20 (0.3%) |
| confirming | 145 |
| high_fitness拒绝 | 0 |
| pose_jump拒绝 | 20 |
| pose_jump候选确认中 | 43 |
| confirmed_pose_jump放行 | 11 |
| rotation_guard hold/settle | 102 |
| Fast-LIO delta guess应用 | 7140 |
| map->odom变化次数 | 16115 |
| map->odom大变化(>=5cm或>=0.03rad) | 11818 |
| 最大map->odom平移跳变 | 4.093m |
| 最大map->odom yaw跳变 | 0.124rad |

**分布**

- fitness: p50 0.0019, p90 0.0044, p99 0.0281, max 0.1814
- accepted fitness: p50 0.0019, p90 0.0042, p99 0.0135, max 0.1731
- correction translation: p50 0.007, p90 0.041, p99 1.038, max 3.099m
- accepted correction translation: p50 0.007, p90 0.034, p99 0.176, max 1.480m
- multi-frame source frames: p50 6, p90 7, p99 10, max 12 帧
- multi-frame source points: p50 7246, p90 11126, p99 16098, max 19284 点

**拒绝/确认原因 Top 10**

| reason | count |
|---|---:|
| `ok` | 7421 |
| `rotation_guard_settle` | 64 |
| `pose_jump_candidate` | 43 |
| `rotation_guard_hold` | 38 |
| `pose_jump` | 20 |
| `confirmed_pose_jump` | 11 |

**最大 correction 事件 Top 12**

| t | state | reason | correction | yaw | fitness | source_frames | guard |
|---:|---|---|---:|---:|---:|---:|---|
| 1780140447.478 | `rejected` | `pose_jump` | 3.099 | 0.070 | 0.0058 | 6 |  |
| 1780140447.373 | `rejected` | `pose_jump` | 2.873 | 0.062 | 0.0075 | 6 |  |
| 1780140447.673 | `rejected` | `pose_jump` | 2.864 | 0.061 | 0.0068 | 5 |  |
| 1780140447.173 | `confirming` | `rotation_guard_settle` | 2.505 | 0.050 | 0.0134 | 7 | Y |
| 1780140447.073 | `confirming` | `rotation_guard_settle` | 2.446 | 0.049 | 0.0147 | 6 | Y |
| 1780140447.973 | `rejected` | `pose_jump` | 2.413 | 0.044 | 0.0084 | 4 |  |
| 1780140448.273 | `rejected` | `pose_jump` | 2.398 | 0.044 | 0.0088 | 5 |  |
| 1780140448.073 | `rejected` | `pose_jump` | 2.333 | 0.042 | 0.0095 | 5 |  |
| 1780140434.478 | `confirming` | `rotation_guard_settle` | 2.179 | 0.075 | 0.0018 | 6 | Y |
| 1780140448.479 | `rejected` | `pose_jump` | 2.019 | 0.031 | 0.0129 | 5 |  |
| 1780140434.373 | `confirming` | `rotation_guard_settle` | 1.960 | 0.068 | 0.0022 | 7 | Y |
| 1780140449.173 | `rejected` | `pose_jump` | 1.754 | 0.019 | 0.0122 | 5 |  |

## 结论边界

- 本报告验证的是定位节点和保护策略在录制输入上的表现。
- `navigation_auto_paused` 和真正重定位闭环需要同时启动导航状态管理器与重定位 bridge；本次 NDT 回放结果可作为是否会触发这些链路的依据。
- 当前代码的多帧合并会累积最近帧并体素降采样，但没有对历史帧做运动补偿；机器人移动较快或转弯时，多帧可能增强几何约束，也可能引入轻微拖影。
