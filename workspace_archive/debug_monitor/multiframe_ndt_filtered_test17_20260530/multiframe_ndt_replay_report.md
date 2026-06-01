# 多帧 NDT Bag 回放模拟报告

- 生成时间: `2026-05-31 00:27:59`
- 回放倍率: `1.0x`
- 每包截断: `未截断，完整回放`
- 输入: `/fast_lio/cloud_registered` + `/tf` + `/tf_static` + `/navigation/status` + `/cmd_vel` + `/odom`
- NDT 模式: `multi_frame_matching_enabled=true`, `window=1.2s`, `max_frames=12`, `rotation_guard_enabled=true`

> 说明: 这是离线影子回放。它能验证同一批传感器数据下多帧 NDT 的匹配、拒绝、旋转保护和 TF 输出表现；不能完全替代实机闭环，因为 Nav2 控制不会根据新的 NDT 输出重新生成真实运动。

## 总览

| Bag | NDT帧 | 接受率 | 拒绝率 | 最大TF跳变 | pose_jump | rotation_guard | 多帧p50/p90 | 结论 |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| nav_drift_test17 | 5150 | 94.5% | 3.2% | 1.225m | 164 | 64 | 4/6 | 触发跳变拒绝 |

## 关键判断

- `nav_drift_test17`: 接受 4867/5150 帧，最大 map->odom TF 跳变 1.225m / 0.061rad，pose_jump=164，rotation_guard=64。

## 分包详情

### nav_drift_test17

- Bag: `/home/ubuntu/nav_drift_test/nav_drift_test17`
- 原始 NDT JSONL: `debug_monitor/multiframe_ndt_filtered_test17_20260530/nav_drift_test17/ndt_status.jsonl`
- 初始 map->odom: x=0.006, y=0.007, z=0.000, yaw=-0.165rad
- NDT 时间跨度: `1201.3s`

| 指标 | 数值 |
|---|---:|
| NDT状态帧 | 5150 |
| accepted | 4867 (94.5%) |
| rejected | 164 (3.2%) |
| confirming | 119 |
| high_fitness拒绝 | 0 |
| pose_jump拒绝 | 164 |
| pose_jump候选确认中 | 55 |
| confirmed_pose_jump放行 | 12 |
| rotation_guard hold/settle | 64 |
| Fast-LIO delta guess应用 | 4033 |
| map->odom变化次数 | 4866 |
| map->odom大变化(>=5cm或>=0.03rad) | 444 |
| 最大map->odom平移跳变 | 1.225m |
| 最大map->odom yaw跳变 | 0.061rad |

**分布**

- fitness: p50 0.0018, p90 0.0051, p99 0.1614, max 0.2034
- accepted fitness: p50 0.0017, p90 0.0041, p99 0.0845, max 0.1894
- correction translation: p50 0.009, p90 0.091, p99 1.003, max 1.549m
- accepted correction translation: p50 0.008, p90 0.046, p99 0.284, max 1.225m
- multi-frame source frames: p50 4, p90 6, p99 8, max 11 帧
- multi-frame source points: p50 7178, p90 10164, p99 15652, max 19393 点

**拒绝/确认原因 Top 10**

| reason | count |
|---|---:|
| `ok` | 4855 |
| `pose_jump` | 164 |
| `pose_jump_candidate` | 55 |
| `rotation_guard_hold` | 35 |
| `rotation_guard_settle` | 29 |
| `confirmed_pose_jump` | 12 |

**最大 correction 事件 Top 12**

| t | state | reason | correction | yaw | fitness | source_frames | guard |
|---:|---|---|---:|---:|---:|---:|---|
| 1780140439.478 | `rejected` | `pose_jump` | 1.549 | 0.059 | 0.0526 | 7 |  |
| 1780140439.573 | `rejected` | `pose_jump` | 1.544 | 0.059 | 0.0535 | 7 |  |
| 1780140462.374 | `rejected` | `pose_jump` | 1.512 | 0.092 | 0.0151 | 6 |  |
| 1780140548.480 | `rejected` | `pose_jump` | 1.512 | 0.054 | 0.0030 | 1 |  |
| 1780140548.575 | `rejected` | `pose_jump` | 1.503 | 0.054 | 0.0031 | 2 |  |
| 1780140443.873 | `confirming` | `rotation_guard_hold` | 1.471 | 0.064 | 0.1172 | 8 | Y |
| 1780140462.174 | `confirming` | `pose_jump_candidate` | 1.435 | 0.086 | 0.0161 | 5 |  |
| 1780140443.973 | `confirming` | `rotation_guard_hold` | 1.422 | 0.066 | 0.1200 | 8 | Y |
| 1780140444.274 | `confirming` | `rotation_guard_hold` | 1.420 | 0.067 | 0.1214 | 7 | Y |
| 1780140548.675 | `confirming` | `pose_jump_candidate` | 1.413 | 0.051 | 0.0032 | 2 |  |
| 1780140439.373 | `rejected` | `pose_jump` | 1.351 | 0.052 | 0.0529 | 6 |  |
| 1780140548.875 | `confirming` | `pose_jump_candidate` | 1.344 | 0.048 | 0.0031 | 3 |  |

## 结论边界

- 本报告验证的是定位节点和保护策略在录制输入上的表现。
- `navigation_auto_paused` 和真正重定位闭环需要同时启动导航状态管理器与重定位 bridge；本次 NDT 回放结果可作为是否会触发这些链路的依据。
- 当前代码的多帧合并会累积最近帧并体素降采样，但没有对历史帧做运动补偿；机器人移动较快或转弯时，多帧可能增强几何约束，也可能引入轻微拖影。
