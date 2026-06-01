# 多帧 NDT Bag 回放模拟报告

- 生成时间: `2026-05-30 23:53:43`
- 回放倍率: `1.0x`
- 每包截断: `未截断，完整回放`
- 输入: `/fast_lio/cloud_registered` + `/tf` + `/tf_static` + `/navigation/status` + `/cmd_vel` + `/odom`
- NDT 模式: `multi_frame_matching_enabled=true`, `window=1.2s`, `max_frames=12`, `rotation_guard_enabled=true`

> 说明: 这是离线影子回放。它能验证同一批传感器数据下多帧 NDT 的匹配、拒绝、旋转保护和 TF 输出表现；不能完全替代实机闭环，因为 Nav2 控制不会根据新的 NDT 输出重新生成真实运动。

## 总览

| Bag | NDT帧 | 接受率 | 拒绝率 | 最大TF跳变 | pose_jump | rotation_guard | 多帧p50/p90 | 结论 |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| nav_drift_test19 | 2950 | 86.0% | 9.5% | 6.396m | 280 | 116 | 6/8 | 触发跳变拒绝 |

## 关键判断

- `nav_drift_test19`: 接受 2538/2950 帧，最大 map->odom TF 跳变 6.396m / 0.181rad，pose_jump=280，rotation_guard=116。

## 分包详情

### nav_drift_test19

- Bag: `/home/ubuntu/nav_drift_test/nav_drift_test19`
- 原始 NDT JSONL: `debug_monitor/multiframe_ndt_replay_test19_20260530/nav_drift_test19/ndt_status.jsonl`
- 初始 map->odom: x=0.004, y=0.064, z=0.000, yaw=-0.136rad
- NDT 时间跨度: `537.5s`

| 指标 | 数值 |
|---|---:|
| NDT状态帧 | 2950 |
| accepted | 2538 (86.0%) |
| rejected | 280 (9.5%) |
| confirming | 132 |
| high_fitness拒绝 | 0 |
| pose_jump拒绝 | 280 |
| pose_jump候选确认中 | 16 |
| confirmed_pose_jump放行 | 5 |
| rotation_guard hold/settle | 116 |
| Fast-LIO delta guess应用 | 2383 |
| map->odom变化次数 | 5878 |
| map->odom大变化(>=5cm或>=0.03rad) | 5206 |
| 最大map->odom平移跳变 | 6.396m |
| 最大map->odom yaw跳变 | 0.181rad |

**分布**

- fitness: p50 0.0019, p90 0.0082, p99 0.1075, max 0.2156
- accepted fitness: p50 0.0018, p90 0.0053, p99 0.1081, max 0.2156
- correction translation: p50 0.013, p90 1.055, p99 2.968, max 4.456m
- accepted correction translation: p50 0.010, p90 0.069, p99 0.330, max 1.055m
- multi-frame source frames: p50 6, p90 8, p99 11, max 12 帧
- multi-frame source points: p50 7995, p90 12765, p99 17521, max 19426 点

**拒绝/确认原因 Top 10**

| reason | count |
|---|---:|
| `ok` | 2533 |
| `pose_jump` | 280 |
| `rotation_guard_hold` | 59 |
| `rotation_guard_settle` | 57 |
| `pose_jump_candidate` | 16 |
| `confirmed_pose_jump` | 5 |

**最大 correction 事件 Top 12**

| t | state | reason | correction | yaw | fitness | source_frames | guard |
|---:|---|---|---:|---:|---:|---:|---|
| 1780149198.557 | `rejected` | `pose_jump` | 4.456 | 0.133 | 0.0039 | 4 |  |
| 1780149198.752 | `rejected` | `pose_jump` | 4.438 | 0.133 | 0.0040 | 5 |  |
| 1780149198.652 | `rejected` | `pose_jump` | 4.408 | 0.132 | 0.0041 | 4 |  |
| 1780149198.852 | `rejected` | `pose_jump` | 4.371 | 0.130 | 0.0041 | 5 |  |
| 1780149198.952 | `rejected` | `pose_jump` | 4.316 | 0.129 | 0.0042 | 6 |  |
| 1780149199.053 | `rejected` | `pose_jump` | 4.285 | 0.128 | 0.0039 | 5 |  |
| 1780149198.152 | `rejected` | `pose_jump` | 3.904 | 0.113 | 0.0138 | 3 |  |
| 1780149204.557 | `confirming` | `rotation_guard_hold` | 3.868 | 0.116 | 0.0043 | 3 | Y |
| 1780149204.652 | `confirming` | `rotation_guard_hold` | 3.798 | 0.113 | 0.0044 | 4 | Y |
| 1780149204.352 | `confirming` | `rotation_guard_hold` | 3.793 | 0.113 | 0.0051 | 3 | Y |
| 1780149204.852 | `confirming` | `rotation_guard_hold` | 3.691 | 0.109 | 0.0044 | 6 | Y |
| 1780149204.752 | `confirming` | `rotation_guard_hold` | 3.587 | 0.106 | 0.0051 | 5 | Y |

## 结论边界

- 本报告验证的是定位节点和保护策略在录制输入上的表现。
- `navigation_auto_paused` 和真正重定位闭环需要同时启动导航状态管理器与重定位 bridge；本次 NDT 回放结果可作为是否会触发这些链路的依据。
- 当前代码的多帧合并会累积最近帧并体素降采样，但没有对历史帧做运动补偿；机器人移动较快或转弯时，多帧可能增强几何约束，也可能引入轻微拖影。
