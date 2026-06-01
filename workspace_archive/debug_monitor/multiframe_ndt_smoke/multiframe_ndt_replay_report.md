# 多帧 NDT Bag 回放模拟报告

- 生成时间: `2026-05-30 23:11:08`
- 回放倍率: `1.0x`
- 每包截断: `20.0s`
- 输入: `/fast_lio/cloud_registered` + `/tf` + `/tf_static` + `/navigation/status` + `/cmd_vel` + `/odom`
- NDT 模式: `multi_frame_matching_enabled=true`, `window=1.2s`, `max_frames=12`, `rotation_guard_enabled=true`

> 说明: 这是离线影子回放。它能验证同一批传感器数据下多帧 NDT 的匹配、拒绝、旋转保护和 TF 输出表现；不能完全替代实机闭环，因为 Nav2 控制不会根据新的 NDT 输出重新生成真实运动。

## 总览

| Bag | NDT帧 | 接受率 | 拒绝率 | 最大TF跳变 | pose_jump | rotation_guard | 多帧p50/p90 | 结论 |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| nav_drift_test19 | 127 | 100.0% | 0.0% | 0.075m | 0 | 0 | 6/6 | 整体稳定 |

## 关键判断

- `nav_drift_test19`: 接受 127/127 帧，最大 map->odom TF 跳变 0.075m / 0.006rad，pose_jump=0，rotation_guard=0。

## 分包详情

### nav_drift_test19

- Bag: `/home/ubuntu/nav_drift_test/nav_drift_test19`
- 原始 NDT JSONL: `debug_monitor/multiframe_ndt_smoke/nav_drift_test19/ndt_status.jsonl`
- 初始 map->odom: x=0.004, y=0.064, z=0.000, yaw=-0.136rad
- NDT 时间跨度: `18.8s`

| 指标 | 数值 |
|---|---:|
| NDT状态帧 | 127 |
| accepted | 127 (100.0%) |
| rejected | 0 (0.0%) |
| confirming | 0 |
| high_fitness拒绝 | 0 |
| pose_jump拒绝 | 0 |
| pose_jump候选确认中 | 0 |
| confirmed_pose_jump放行 | 0 |
| rotation_guard hold/settle | 0 |
| Fast-LIO delta guess应用 | 122 |
| map->odom变化次数 | 272 |
| map->odom大变化(>=5cm或>=0.03rad) | 252 |
| 最大map->odom平移跳变 | 0.075m |
| 最大map->odom yaw跳变 | 0.006rad |

**分布**

- fitness: p50 0.0007, p90 0.0007, p99 0.0007, max 0.0007
- accepted fitness: p50 0.0007, p90 0.0007, p99 0.0007, max 0.0007
- correction translation: p50 0.006, p90 0.008, p99 0.010, max 0.065m
- accepted correction translation: p50 0.006, p90 0.008, p99 0.010, max 0.065m
- multi-frame source frames: p50 6, p90 6, p99 7, max 7 帧
- multi-frame source points: p50 5681, p90 5741, p99 7467, max 7482 点

**拒绝/确认原因 Top 10**

| reason | count |
|---|---:|
| `ok` | 127 |

**最大 correction 事件 Top 12**

| t | state | reason | correction | yaw | fitness | source_frames | guard |
|---:|---|---|---:|---:|---:|---:|---|
| 1780148792.745 | `accepted` | `ok` | 0.065 | 0.005 | 0.0007 | 1 |  |
| 1780148793.245 | `accepted` | `ok` | 0.010 | 0.001 | 0.0007 | 3 |  |
| 1780148796.245 | `accepted` | `ok` | 0.010 | 0.001 | 0.0007 | 6 |  |
| 1780148799.346 | `accepted` | `ok` | 0.010 | 0.001 | 0.0007 | 6 |  |
| 1780148807.645 | `accepted` | `ok` | 0.009 | 0.001 | 0.0007 | 5 |  |
| 1780148810.845 | `accepted` | `ok` | 0.009 | 0.001 | 0.0007 | 4 |  |
| 1780148799.845 | `accepted` | `ok` | 0.009 | 0.001 | 0.0007 | 7 |  |
| 1780148804.845 | `accepted` | `ok` | 0.009 | 0.001 | 0.0007 | 6 |  |
| 1780148804.347 | `accepted` | `ok` | 0.009 | 0.001 | 0.0007 | 4 |  |
| 1780148809.645 | `accepted` | `ok` | 0.009 | 0.001 | 0.0007 | 6 |  |
| 1780148798.848 | `accepted` | `ok` | 0.009 | 0.001 | 0.0007 | 6 |  |
| 1780148810.646 | `accepted` | `ok` | 0.009 | 0.001 | 0.0007 | 6 |  |

## 结论边界

- 本报告验证的是定位节点和保护策略在录制输入上的表现。
- `navigation_auto_paused` 和真正重定位闭环需要同时启动导航状态管理器与重定位 bridge；本次 NDT 回放结果可作为是否会触发这些链路的依据。
- 当前代码的多帧合并会累积最近帧并体素降采样，但没有对历史帧做运动补偿；机器人移动较快或转弯时，多帧可能增强几何约束，也可能引入轻微拖影。
