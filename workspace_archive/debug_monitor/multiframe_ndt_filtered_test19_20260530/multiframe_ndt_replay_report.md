# 多帧 NDT Bag 回放模拟报告

- 生成时间: `2026-05-31 00:37:35`
- 回放倍率: `1.0x`
- 每包截断: `未截断，完整回放`
- 输入: `/fast_lio/cloud_registered` + `/tf` + `/tf_static` + `/navigation/status` + `/cmd_vel` + `/odom`
- NDT 模式: `multi_frame_matching_enabled=true`, `window=1.2s`, `max_frames=12`, `rotation_guard_enabled=true`

> 说明: 这是离线影子回放。它能验证同一批传感器数据下多帧 NDT 的匹配、拒绝、旋转保护和 TF 输出表现；不能完全替代实机闭环，因为 Nav2 控制不会根据新的 NDT 输出重新生成真实运动。

## 总览

| Bag | NDT帧 | 接受率 | 拒绝率 | 最大TF跳变 | pose_jump | rotation_guard | 多帧p50/p90 | 结论 |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| nav_drift_test19 | 2456 | 88.7% | 8.1% | 1.091m | 199 | 58 | 5/7 | 触发跳变拒绝 |

## 关键判断

- `nav_drift_test19`: 接受 2179/2456 帧，最大 map->odom TF 跳变 1.091m / 0.087rad，pose_jump=199，rotation_guard=58。

## 分包详情

### nav_drift_test19

- Bag: `/home/ubuntu/nav_drift_test/nav_drift_test19`
- 原始 NDT JSONL: `debug_monitor/multiframe_ndt_filtered_test19_20260530/nav_drift_test19/ndt_status.jsonl`
- 初始 map->odom: x=0.004, y=0.064, z=0.000, yaw=-0.136rad
- NDT 时间跨度: `536.1s`

| 指标 | 数值 |
|---|---:|
| NDT状态帧 | 2456 |
| accepted | 2179 (88.7%) |
| rejected | 199 (8.1%) |
| confirming | 78 |
| high_fitness拒绝 | 0 |
| pose_jump拒绝 | 199 |
| pose_jump候选确认中 | 20 |
| confirmed_pose_jump放行 | 5 |
| rotation_guard hold/settle | 58 |
| Fast-LIO delta guess应用 | 1889 |
| map->odom变化次数 | 2178 |
| map->odom大变化(>=5cm或>=0.03rad) | 370 |
| 最大map->odom平移跳变 | 1.091m |
| 最大map->odom yaw跳变 | 0.087rad |

**分布**

- fitness: p50 0.0020, p90 0.0567, p99 0.1241, max 0.1885
- accepted fitness: p50 0.0018, p90 0.0051, p99 0.1074, max 0.1778
- correction translation: p50 0.016, p90 0.743, p99 2.214, max 2.732m
- accepted correction translation: p50 0.012, p90 0.084, p99 0.357, max 1.091m
- multi-frame source frames: p50 5, p90 7, p99 10, max 12 帧
- multi-frame source points: p50 7941, p90 12653, p99 16268, max 18839 点

**拒绝/确认原因 Top 10**

| reason | count |
|---|---:|
| `ok` | 2174 |
| `pose_jump` | 199 |
| `rotation_guard_hold` | 30 |
| `rotation_guard_settle` | 28 |
| `pose_jump_candidate` | 20 |
| `confirmed_pose_jump` | 5 |

**最大 correction 事件 Top 12**

| t | state | reason | correction | yaw | fitness | source_frames | guard |
|---:|---|---|---:|---:|---:|---:|---|
| 1780149206.352 | `rejected` | `pose_jump` | 2.732 | 0.129 | 0.0269 | 8 |  |
| 1780149207.557 | `rejected` | `pose_jump` | 2.692 | 0.127 | 0.0519 | 7 |  |
| 1780149210.952 | `confirming` | `rotation_guard_hold` | 2.684 | 0.124 | 0.0675 | 6 | Y |
| 1780149206.252 | `rejected` | `pose_jump` | 2.678 | 0.125 | 0.0334 | 8 |  |
| 1780149207.352 | `rejected` | `pose_jump` | 2.659 | 0.131 | 0.0260 | 7 |  |
| 1780149205.952 | `rejected` | `pose_jump` | 2.648 | 0.123 | 0.0384 | 8 |  |
| 1780149206.053 | `rejected` | `pose_jump` | 2.628 | 0.123 | 0.0395 | 8 |  |
| 1780149208.352 | `rejected` | `pose_jump` | 2.628 | 0.128 | 0.0416 | 5 |  |
| 1780149207.752 | `rejected` | `pose_jump` | 2.621 | 0.126 | 0.0505 | 7 |  |
| 1780149210.752 | `confirming` | `rotation_guard_hold` | 2.618 | 0.123 | 0.0639 | 6 | Y |
| 1780149211.052 | `confirming` | `rotation_guard_hold` | 2.609 | 0.122 | 0.0639 | 7 | Y |
| 1780149207.452 | `rejected` | `pose_jump` | 2.569 | 0.132 | 0.0160 | 7 |  |

## 结论边界

- 本报告验证的是定位节点和保护策略在录制输入上的表现。
- `navigation_auto_paused` 和真正重定位闭环需要同时启动导航状态管理器与重定位 bridge；本次 NDT 回放结果可作为是否会触发这些链路的依据。
- 当前代码的多帧合并会累积最近帧并体素降采样，但没有对历史帧做运动补偿；机器人移动较快或转弯时，多帧可能增强几何约束，也可能引入轻微拖影。
