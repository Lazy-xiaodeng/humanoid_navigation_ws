# 多帧 NDT Bag 回放模拟报告

- 生成时间: `2026-05-31 00:07:21`
- 回放倍率: `1.0x`
- 每包截断: `未截断，完整回放`
- 输入: `/fast_lio/cloud_registered` + `/tf` + `/tf_static` + `/navigation/status` + `/cmd_vel` + `/odom`
- NDT 模式: `multi_frame_matching_enabled=true`, `window=1.2s`, `max_frames=12`, `rotation_guard_enabled=true`

> 说明: 这是离线影子回放。它能验证同一批传感器数据下多帧 NDT 的匹配、拒绝、旋转保护和 TF 输出表现；不能完全替代实机闭环，因为 Nav2 控制不会根据新的 NDT 输出重新生成真实运动。

## 总览

| Bag | NDT帧 | 接受率 | 拒绝率 | 最大TF跳变 | pose_jump | rotation_guard | 多帧p50/p90 | 结论 |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| navtest0 | 3104 | 69.7% | 29.6% | 0.837m | 407 | 0 | 5/6 | 触发跳变拒绝 |

## 关键判断

- `navtest0`: 接受 2164/3104 帧，最大 map->odom TF 跳变 0.837m / 0.051rad，pose_jump=407，rotation_guard=0。

## 分包详情

### navtest0

- Bag: `/home/ubuntu/nav_drift_test/navtest0`
- 原始 NDT JSONL: `debug_monitor/multiframe_ndt_filtered_navtest0_20260530/navtest0/ndt_status.jsonl`
- 初始 map->odom: x=0.153, y=0.079, z=0.000, yaw=-0.190rad
- NDT 时间跨度: `682.5s`

| 指标 | 数值 |
|---|---:|
| NDT状态帧 | 3104 |
| accepted | 2164 (69.7%) |
| rejected | 919 (29.6%) |
| confirming | 21 |
| high_fitness拒绝 | 512 |
| pose_jump拒绝 | 407 |
| pose_jump候选确认中 | 21 |
| confirmed_pose_jump放行 | 3 |
| rotation_guard hold/settle | 0 |
| Fast-LIO delta guess应用 | 1886 |
| map->odom变化次数 | 2155 |
| map->odom大变化(>=5cm或>=0.03rad) | 259 |
| 最大map->odom平移跳变 | 0.837m |
| 最大map->odom yaw跳变 | 0.051rad |

**分布**

- fitness: p50 0.0035, p90 0.6827, p99 2.2820, max 2.7521
- accepted fitness: p50 0.0016, p90 0.0072, p99 0.0962, max 0.1895
- correction translation: p50 0.008, p90 1.713, p99 3.515, max 4.021m
- accepted correction translation: p50 0.008, p90 0.061, p99 0.380, max 0.837m
- multi-frame source frames: p50 5, p90 6, p99 9, max 11 帧
- multi-frame source points: p50 6972, p90 11354, p99 14904, max 18369 点

**拒绝/确认原因 Top 10**

| reason | count |
|---|---:|
| `ok` | 2161 |
| `high_fitness` | 512 |
| `pose_jump` | 407 |
| `pose_jump_candidate` | 21 |
| `confirmed_pose_jump` | 3 |

**最大 correction 事件 Top 12**

| t | state | reason | correction | yaw | fitness | source_frames | guard |
|---:|---|---|---:|---:|---:|---:|---|
| 1780123235.647 | `rejected` | `pose_jump` | 4.021 | 0.161 | 0.0293 | 5 |  |
| 1780123235.147 | `rejected` | `pose_jump` | 4.018 | 0.157 | 0.0352 | 6 |  |
| 1780123236.547 | `rejected` | `pose_jump` | 4.017 | 0.166 | 0.0282 | 6 |  |
| 1780123235.347 | `rejected` | `pose_jump` | 4.006 | 0.159 | 0.0290 | 5 |  |
| 1780123251.347 | `rejected` | `pose_jump` | 3.952 | 0.166 | 0.0706 | 5 |  |
| 1780123250.347 | `rejected` | `pose_jump` | 3.943 | 0.176 | 0.0731 | 6 |  |
| 1780123235.447 | `rejected` | `pose_jump` | 3.912 | 0.156 | 0.0288 | 5 |  |
| 1780123250.847 | `rejected` | `pose_jump` | 3.889 | 0.166 | 0.0772 | 6 |  |
| 1780123236.247 | `rejected` | `pose_jump` | 3.889 | 0.159 | 0.0288 | 5 |  |
| 1780123236.647 | `rejected` | `pose_jump` | 3.834 | 0.160 | 0.0287 | 5 |  |
| 1780123236.447 | `rejected` | `pose_jump` | 3.802 | 0.157 | 0.0302 | 5 |  |
| 1780123234.747 | `rejected` | `pose_jump` | 3.785 | 0.144 | 0.0418 | 8 |  |

## 结论边界

- 本报告验证的是定位节点和保护策略在录制输入上的表现。
- `navigation_auto_paused` 和真正重定位闭环需要同时启动导航状态管理器与重定位 bridge；本次 NDT 回放结果可作为是否会触发这些链路的依据。
- 当前代码的多帧合并会累积最近帧并体素降采样，但没有对历史帧做运动补偿；机器人移动较快或转弯时，多帧可能增强几何约束，也可能引入轻微拖影。
