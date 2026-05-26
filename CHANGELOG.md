# 变更记录

## [2026-05-26 23:10:00] TF冲突修复: DEGRADED期间禁止NDT重发旧TF

- **修改文件**:
  - `src/humanoid_navigation2/launch/navigation2_fusion_sc.launch.py`

- **根因**: NDT C++ 节点在拒帧时通过 `publishLastGoodTransformIfFresh()` 重发旧 map->odom TF (最多3秒前)，与 fusion DEGRADED 冻结的 map->odom TF 同时发布。两个发布者竞争，下游节点按最新时间戳取 TF，在"旧位姿"和"冻结位姿"之间反复横跳，导致 Nav2 控制器位姿跳动、反复重算路径。

- **修改内容**:
  - `republish_last_good_tf_on_failure`: true → false (NDT拒帧时不再重发旧TF)
  - `max_last_good_tf_age_sec`: 3.0 → 0.5 (旧TF有效期降至0.5秒，双重保险)

- **效果**: DEGRADED 期间仅 fusion 发布冻结 map->odom TF，无冲突。HEALTHY 期间 NDT 正常发布当前 TF（不受影响）。启动到导航的 TF 链完全不变。

## [2026-05-26 22:45:00] test3 导航失控修复: 点位8-9 NDT漂移阈值收紧 + TF冲突分析

- **修改文件**:
  - `src/humanoid_navigation2/launch/navigation2_fusion_sc.launch.py`

- **根因**: test3 导航点位8→9 区域，NDT 在几何退化区产出 0.5-0.77m 系统性错误修正，单帧低于 0.8m 阈值被接受，累积导致位姿漂移 3m+。同时 NDT C++ 节点在 fusion DEGRADED 状态仍持续发布 map->odom TF，与 fusion 冻结的 TF 产生时间戳竞争（30Hz vs 10Hz），下游节点位姿来回跳动，Nav2 控制器反复重算路径，robot 右转失控。

- **修改内容**:
  - `max_pose_jump_translation`: 0.80 → 0.40 (拦截 test3 中的 0.5-0.77m 致命区间)
  - `max_pose_jump_yaw`: 0.45 → 0.30
  - `pose_jump_reacquire_max_translation`: 2.00 → 0.80 (降低重定位触发门槛，原来太高永不触发)
  - `pose_jump_reacquire_max_fitness`: 0.10 → 0.08
  - `pose_jump_reacquire_max_yaw`: 0.45 → 0.30
  - `pose_jump_correction_threshold_m`: 0.50 → 0.35 (Fusion DEGRADED 更早触发)
  - NDT 网格分辨率: 确认已在上次 commit 调至 0.5m，无需再次修改

- **效果预期**:
  - test3 的 32 帧 0.4-0.8m 漂移修正全部被拦截
  - Fusion 在修正 >0.35m 时提前进入 DEGRADED 冻结 TF
  - 重定位触发条件从 (连续2帧>2.0m) 降到 (连续2帧>0.8m)，实际可生效
  - 风险: 急转弯时合法修正 0.3-0.4m 可能被误拒 (从 test3 数据看健康期修正<0.01m，风险低)

## [2026-05-26 20:50:00] P0+P1: NDT跳变/融合链路修复 — 长廊定位漂移根治

- **修改文件**:
  - `src/humanoid_navigation2/humanoid_navigation2/localization_odom_fusion.py`
  - `src/humanoid_navigation2/launch/navigation2_fusion.launch.py`
  - `src/lidar_localization/param/localization.yaml`

- **背景**: 2026-05-26 20:20 导航测试，点位7-12区域 NDT 频繁跳变（0.8-1.8m），fusion DEGRADED 持续180s后 LOST，恢复时接受 3.26m 位姿跳变导致定位崩溃。根因分析见下方五条链路。

- **P0-1: 收紧 pose_jump_reacquire 参数** (`navigation2_fusion.launch.py`):
  - `pose_jump_reacquire_max_translation`: 2.00→0.50 (长廊中单帧真实运动不超过0.5m)
  - `pose_jump_reacquire_max_yaw`: 0.45→0.20
  - `pose_jump_reacquire_max_fitness`: 0.10→0.05
  - `pose_jump_reacquire_required_frames`: 2→5 (需要更多帧确认非偶然错误收敛)
  - `pose_jump_reacquire_xy_tolerance`: 0.50→0.30
  - `pose_jump_reacquire_yaw_tolerance`: 0.25→0.15

- **P0-2: inlier=0 虚假健康检测** (`localization_odom_fusion.py`):
  - 新增参数 `inlier_zero_degraded_early_lost_sec`(30s) + `inlier_zero_error_ceiling`(0.01)
  - 锁定期后在 `_update_degraded` 增加条件1.5: inlier=0 + error<0.01 持续30s → 虚假健康 → 加速 LOST
  - 原理: 长廊几何混叠下NDT收敛到错误位置后fitness极低但inlier始终为0

- **P0-3: DEGRADED 静止检测定时器重置** (`localization_odom_fusion.py`):
  - 新增基于里程计位移的静止检测: odom位移<0.1m持续5s → 自动重置 `degraded_start_time`
  - 替代不可靠的 nav_state 变更检测（快速连续导航时状态转换可能漏过）
  - 新增参数 `_odom_stationary_threshold_m`(0.1), `_odom_stationary_duration_sec`(5.0)

- **P1-4: 收紧 LOST recovery 软验收** (`navigation2_fusion.launch.py`):
  - `recovery_pose_max_xy_error_m`: 5.0→2.0 (显式覆盖默认值)
  - `recovery_pose_accept_if_ndt_error_below`: 0.03→0.01
  - `recovery_pose_skip_odom_after_displacement_m`: 20→10
  - `max_degraded_lock_sec`: 180→60 (缩短DEGRADED超时减少odom累积漂移)

- **P1-5: NDT匹配参数优化** (`localization.yaml`):
  - `ndt_resolution`: 1.0→0.5 (更细网格增强长廊方向约束)
  - `ndt_max_iterations`: 35→50 (更细网格需要更多迭代收敛)
  - `score_threshold`: 2.0→1.0 (收紧匹配质量要求)
  - `voxel_leaf_size`: 0.2→0.15 (配合resolution保持1/3比例)

- **根因总结**:
  1. 长廊几何混叠 → NDT收敛到错误局部极小值 (fitness 0.0009但位姿偏移3m)
  2. pose_jump_reacquire 2帧确认+2m容差 → 错误收敛被确认
  3. DEGRADED 定时器依赖 nav_state 变更重置 → 快速导航中漏过
  4. inlier=0 异常信号被忽略 → 虚假健康持续3分钟
  5. LOST 软验收 5m+odom_displacement → 3.26m跳变被放行

## [2026-05-26 20:15:00] Fusion DEGRADED 锁定期方案 (替代循环检测)

- **修改文件**:
  - `src/humanoid_navigation2/humanoid_navigation2/localization_odom_fusion.py`
  - `src/humanoid_navigation2/humanoid_navigation2/ndt_fusion_monitor.py`
  - `src/humanoid_navigation2/launch/navigation2_fusion_sc.launch.py`
  - `src/humanoid_navigation2/launch/navigation2_fusion.launch.py`

- **背景**: 2026-05-26 导航测试中，点位7区域 NDT 反复 pose_jump→错误收敛→report ok→fusion 退出 DEGRADED 的循环（22s内4次）。日志分析发现
  NDT 在 19:01:59 自我跳变 17.23m 后收敛到错误位姿 (9.01,5.59)，机器人物理位置在 (19,19) 附近。全程 SC 从未触发（0条SC事件）。

- **循环检测方案废弃原因**: 锁定期内 fusion 一直停在 DEGRADED，不会产生新的 DEGRADED 进入事件 → 循环计数无法递增 → 自相矛盾

- **锁定期方案** (`_update_degraded` + `_is_healthy_strict`):
  - 进入 DEGRADED → 强制 `min_degraded_lock_sec`(30s) 锁定期 → 拒绝 NDT 一切恢复信号
  - 锁定期内: frozen map→odom + Fast-LIO odom 驱动导航，NDT 拒绝率 >90% + ≥30帧 → 提前 LOST
  - 锁定期满 → 验证期: `_is_healthy_strict()` 检查 error<0.15 + converged + reason=ok + correction<0.3m + 跳变<0.8m
  - 连续 `lock_recovery_healthy_consecutive_frames`(10) 帧满足 → TRANSITIONING → HEALTHY
  - TRANSITIONING 二次验证: frozen→NDT 跳变 >`recovery_pose_jump_max_m`(5m) → 拒绝 → LOST
  - 总超时 `max_degraded_lock_sec`(180s) → LOST → SC

- **新增参数**: `min_degraded_lock_sec`(30), `max_degraded_lock_sec`(180), `lock_recovery_healthy_consecutive_frames`(10), `lock_recovery_max_correction_m`(0.3), `lock_early_lost_rejection_rate`(0.9), `lock_early_lost_min_frames`(30), `recovery_pose_jump_max_m`(5.0)

- **监控增强** (`ndt_fusion_monitor.py`):
  - Fusion DEGRADED 面板: "锁定期内 🔒" / "验证期 🔍" + "拒绝NDT恢复" / "严格验证中"
  - robot_pose/pcl_pose 定期快照日志（每5s到JSONL）

- **分析文档**: Section XVIII 追加至 `NDT_FUSION_SC_MODIFICATION_REPORT.md`

## [2026-05-26 17:05:00] NDT score_threshold 收紧 2.0→0.3 + fusion 新增 NDT pose jump 感知 DEGRADED

- **修改文件**:
  - `src/humanoid_navigation2/humanoid_navigation2/localization_odom_fusion.py` (+55/-3)
  - `src/humanoid_navigation2/launch/navigation2_fusion_sc.launch.py` (+5/-1)
  - `src/humanoid_navigation2/launch/navigation2_fusion.launch.py` (+5/-1)
  - `src/humanoid_navigation2/launch/navigation2.launch.py` (±1)
  - `src/humanoid_navigation2/launch/navigation_stack.launch.py` (±1)
  - `NDT_FUSION_SC_MODIFICATION_REPORT.md` (新增第十六节)
- **修改内容**:
  - **NDT 收紧**: 所有 launch 的 `score_threshold` 从 2.0 降至 0.3。odom 接管链可靠，NDT 宁可拒帧也不接受低质量匹配。
  - **fusion `_on_ndt_status()`**: 新增解析 `reason` / `correction_translation` / `correction_yaw` 字段（NDT status JSON 已含，之前未利用）
  - **fusion `_is_degraded()`**: 新增三个 pose jump 判据（status reason / correction_translation 超标 / pcl_pose 帧间跳变），在 fitness_score 判据之前执行
  - **fusion `_update_healthy()`**: 新增 `/pcl_pose` 帧间跳变检测兜底 (阈值 0.5m)
  - **fusion 新增参数**: `pose_jump_degraded_from_status` / `pose_jump_degraded_from_pcl` / `pose_jump_pcl_threshold_m` / `pose_jump_correction_threshold_m`
  - **DEGRADED 日志增强**: 枚举所有触发原因 (ndt_reason / correction / pcl_pose_jump / error / not_converged)
- **修改原因**: 2026-05-26 16:31 实机导航日志分析：NDT 连续 3 次 pose jump (1.0m→1.4m→1.1m) 但 fitness=0.003~0.014，fusion 全程 HEALTHY（fitness 从未 > 0.5），odom 未接管 → 代价地图被污染 → 控制器误判碰撞 → Nav2 失败。根源是几何混叠场景下 NDT 的 fitness 与全局正确性脱钩，仅靠 fitness_score 的 DEGRADED 检测被完全旁路。
- **影响范围**: fusion DEGRADED 触发更灵敏（pose jump 感知 + NDT 更频繁拒帧），DEGRADED 频率预期增加，但 odom 接管 + SC recovery 链路已就绪，导航连续性不受影响。

## [2026-05-26 17:35:00] Pose Jump 检测鲁棒性分析 — 时间盲区与阈值盲区评估

- **修改文件**:
  - `NDT_FUSION_SC_MODIFICATION_REPORT.md` (新增第十七节)
- **分析内容**:
  - **Q1 时间盲区**：NDT 在发布 pose_jump_candidate 之前已不准 → 三层检测全部旁路（持续错误追踪时 reason/correction/pcl_pose 均不触发）。这是 SLAM 本质限制（fitness 不衡量全局正确性），但实际中几何混叠是瞬态的，极少持续数十帧。终极兜底靠 SC recovery。
  - **Q2 阈值盲区**：NDT 跳变 <0.8m 时 NDT 不触发 pose_jump 机制。fusion 通过 correction_translation(0.5m) 和 /pcl_pose(0.5m) 两个更低阈值覆盖 0.5-0.8m 区间。≤0.5m 有意放行（odom 可消化，误报代价高）。
  - **阈值体系表**：NDT max_pose_jump_translation(0.8m) / Fusion correction_translation(0.5m) / Fusion pcl_pose(0.5m) 形成灵敏度梯度
  - **已知限制**：持续静默错误追踪 / yaw 跳变盲区 / 阈值不可热调

## [2026-05-26 16:29:49] 收紧 SC 开机全局重定位门限，防止相似走廊错位初始化

- **修改文件**:
  - `src/humanoid_scancontext_global_localization/config/scancontext_global_localization.yaml`
  - `src/humanoid_scancontext_global_localization/src/scancontext_global_localizer_node.cpp`
  - `src/humanoid_navigation2/launch/navigation2.launch.py`
  - `src/humanoid_navigation2/launch/navigation2_fusion_sc.launch.py`
- **修改内容**:
  - `global_recovery_enable_candidate_confidence_gate`: `false` -> `true`
  - `global_recovery_gicp_fitness_threshold`: `0.13/0.20` -> `0.09`
  - `global_recovery_required_consistent_results`: `3` -> `4`
  - `global_recovery_consistency_window`: `6` -> `8`
  - `global_recovery_consistency_xy_tolerance`: `0.8` -> `0.5`
  - `global_recovery_consistency_yaw_tolerance`: `0.35` -> `0.25`
- **修改原因**: 2026-05-26 开机日志显示 SC 在候选长期歧义后接受 `x=0.323, y=-1.173, yaw=-104.4deg`，NDT 以 `error=0.1186` 判定首次定位成功；实际 RViz 手动修正位姿为 `x=6.229, y=10.813, yaw=90.7deg`。这证明“关闭 ambiguity gate + 仅靠 NDT 连续 accepted 兜底”不能保证全局位姿正确。
- **影响范围**: 启动阶段 SC 全局重定位会更保守；相似结构下宁可继续等待/要求人工初值，也不自动接受歧义候选。

## [2026-05-26 04:00:00] 硬改 C++ 默认值 — ambiguity gate 参数未通过 YAML/launch 加载

- **修改文件**: `src/humanoid_scancontext_global_localization/src/scancontext_global_localizer_node.cpp`
- **修改内容**:
  - `declare_parameter<bool>("global_recovery_enable_candidate_confidence_gate", ...)` 默认值: `true` → `false`
  - 成员变量初始化 `{true}` → `{false}`
- **修改原因**: YAML 文件和 launch 文件 dict 均设置了 `global_recovery_enable_candidate_confidence_gate: false`，但运行时 ambiguity gate 仍然拒绝所有候选（连续 3 次启动测试，每次 16-17 次 trigger 全部被拒），说明 ROS 2 参数覆盖未生效。直接改 C++ 硬编码默认值是唯一可靠的方式
- **影响范围**: SC 全局重定位 ambiguity gate 在全局模式下真正禁用

## [2026-05-26 03:45:00] 修复 SC ambiguity gate 参数未生效 — 直接修改 YAML 源文件

- **修改文件**: `src/humanoid_scancontext_global_localization/config/scancontext_global_localization.yaml`
- **修改内容**: `global_recovery_enable_candidate_confidence_gate`: true → false
- **修改原因**: launch 文件 dict 参数未能覆盖 YAML 文件中的值（ROS 2 参数加载顺序问题），导致全局模式下 ambiguity gate 仍在拒绝所有 SC 候选。日志显示 16 次 trigger 全部被 ambiguity gate 拒绝。直接修改 YAML 源文件确保参数生效
- **影响范围**: SC 全局重定位启动成功率

## [2026-05-26 03:30:00] 修复 SC 启动/运行阶段策略分离 — odom gate 和 ambiguity gate

- **修改文件**: `src/humanoid_navigation2/humanoid_navigation2/scancontext_to_initialpose.py`
- **修改内容**:
  - **启动 vs 运行阶段策略分离**: `timer_callback()` 中 `use_global` 逻辑改为:
    - **启动阶段** (`startup=True`): 始终使用全局 recovery 模式，跳过 odom consistency gate
    - **运行阶段** (`startup=False`): 保持原有策略——conservative 优先 (odom gate 验证)，3次失败后渐进升级 global
  - `global_recovery_after_attempts` 保持为 3（运行时 escalation 阈值）
- **修改原因**: (1) 启动阶段里程计在原点 (0,0,0)，SC 候选位姿在 camera_init 录制帧中可能有数米偏差，odom gate 在此时无参考价值；(2) 运行阶段机器人已移动，里程计有实际参考意义，odom gate 能有效过滤 SC 的离谱候选；(3) 一刀切禁用 odom gate 会损害运行时恢复的安全性
- **影响范围**: SC 重定位启动成功率 + 运行时恢复安全性

- **修改文件**: `src/humanoid_navigation2/launch/navigation2_fusion_sc.launch.py`
- **修改内容**:
  - `global_recovery_enable_candidate_confidence_gate`: True → False（全局模式下不拒绝感知混叠候选，由 NDT 的 3 帧连续接受做最终验证）
  - `global_recovery_after_attempts`: 保持 3（运行时 escalation 阈值）
- **影响范围**: SC 全局重定位的 ambiguity 容忍度

- **修改文件**: `src/humanoid_navigation2/launch/navigation2.launch.py`
- **修改内容**: 上述参数同步修改
- **影响范围**: SC-only 导航 launch 配置

## [2026-05-26 03:00:00] 修复 SyntaxError 导致 fusion 节点崩溃 + SC bridge 诊断日志增强

- **修改文件**: `src/humanoid_navigation2/humanoid_navigation2/localization_odom_fusion.py`
- **修改内容**:
  - **SyntaxError 修复**: `_update_initializing()` 的 docstring 和字符串中误用了 Unicode 弯引号 `"` `"` (U+201C/U+201D)，Python 无法解析导致 fusion 节点启动即崩溃 (exit code 1)。批量替换为 ASCII 引号
- **修改原因**: 上一次修改时中文输入法引入了弯引号，导致整个 fusion 节点不可用，SC 重定位即使成功也没有 fusion 来管理 HEALTHY/DEGRADED 状态
- **影响范围**: fusion 节点可正常启动

- **修改文件**: `src/humanoid_navigation2/humanoid_navigation2/scancontext_to_initialpose.py`
- **修改内容**:
  - **诊断日志增强**: `start_recovery()` 新增 INFO 日志（recovery 编号+原因），`trigger_done()` 新增 INFO/WARN 日志（触发成功/失败及 SC 的 message），`ndt_status_callback()` 在 NDT 接受/拒绝 initialpose 时新增 INFO/WARN 日志
- **修改原因**: 原代码在 `start_recovery`、`trigger_done`、NDT 状态处理等关键节点无 INFO 级别日志，无法从 log 判断 SC 匹配流程走到了哪一步
- **影响范围**: 诊断能力提升，运行时日志量轻微增加

## [2026-05-26 02:15:00] 修复开机启动阶段重定位死锁问题

- **修改文件**: `src/humanoid_navigation2/humanoid_navigation2/scancontext_to_initialpose.py`
- **修改内容**:
  - **Bug 1 修复 — `localization_missing` 被错误抑制**: 第205-213行，`enable_runtime_auto_recovery=False`（融合模式）时，`localization_missing`（NDT 从未发布过 `/pcl_pose`，即启动从未成功）被和 `localization_stale`（运行时丢定位）一同抑制。修复后 `localization_missing` 永远触发 recovery，不论 `enable_runtime_auto_recovery` 为何值
- **修改原因**: `localization_missing` 和 `localization_stale` 本质不同——前者意味启动阶段从未建立过定位，后者意味运行时 NDT 丢定位。将两者等同处理导致死锁: SC bridge 等 fusion 发 recovery 请求 → fusion INITIALIZING 等 NDT 变健康 → NDT 等 SC bridge 发 /initialpose → 三方死锁，30s 启动窗口过后系统永久卡死
- **影响范围**: 开机重定位成功率

- **修改文件**: `src/humanoid_navigation2/humanoid_navigation2/localization_odom_fusion.py`
- **修改内容**:
  - **Bug 2 修复 — INITIALIZING 无超时无 recovery 机制**: 新增 `init_timeout_sec` 参数（默认 20s），在 `_update_initializing()` 中超时后主动调用 `_request_recovery()` 向 SC bridge 请求全局重定位
  - `_request_recovery()` 日志标签从硬编码 `[LOST]` 改为动态 `[INITIALIZING]/[LOST]`
- **修改原因**: 原逻辑 fusion 在 INITIALIZING 状态被动等待 NDT 健康，从不发送 recovery 请求。LOST 可以发 recovery 但 INITIALIZING 永远到不了 LOST（DEGRADED 需要从 HEALTHY 进入）。加上 Bug 1 联动，30s 后系统彻底死锁。修复后 INITIALIZING 超时主动请求 recovery，即使 SC bridge 的 startup 窗口已过也能打破死锁
- **影响范围**: fusion 状态机启动逻辑、与 SC bridge 的 recovery 握手协议

- **修改文件**: `src/humanoid_navigation2/launch/navigation2_fusion_sc.launch.py`
- **修改内容**: fusion 节点新增 `init_timeout_sec: 20.0` 参数
- **影响范围**: fusion+SC 组合 launch 配置

- **修改文件**: `src/humanoid_navigation2/launch/navigation2_fusion.launch.py`
- **修改内容**: fusion 节点新增 `init_timeout_sec: 20.0` 参数
- **影响范围**: fusion+HDL 组合 launch 配置

- **修改文件**: `src/humanoid_navigation2/launch/navigation_stack.launch.py`
- **修改内容**: fusion 节点新增 `init_timeout_sec: 20.0` 参数
- **影响范围**: 原始导航 launch 配置（路径1）

## [2026-05-26 01:30:00] 修复代码评审发现的 P0/P2 问题

- **修改文件**: `src/humanoid_navigation2/humanoid_navigation2/localization_odom_fusion.py`
- **修改内容**:
  - **P0-1 TF查询混淆修复**: 新增 `/pcl_pose` topic 订阅 (`_on_pcl_pose` 回调)，在 `_is_healthy()` pose gate 和 `_enter_transitioning()` 中使用 `latest_pcl_map_odom` 替代 `latest_ndt_map_odom`。修复前 DEGRADED 期间 TF 查询会拿到 fusion 自己发布的 map->odom，导致 pose gate 形同虚设、smoothstep 过渡目标错误
  - **P2 JSON格式修复**: `_publish_fusion_status()` 改用 `json.dumps(status, ensure_ascii=False)` 替代 `str(status)`
- **修改原因**: 代码评审发现融合节点在 DEGRADED/TRANSITIONING 状态下通过 TF 查询 `map->odom` 时会读到自己发布的值（30Hz覆盖NDT的10Hz），而非NDT的真实输出。改用 `/pcl_pose` topic 直达 NDT 数据，绕过 TF 污染
- **影响范围**: NDT恢复判定准确性、DEGRADED→TRANSITIONING平滑过渡质量、fusion_status格式兼容性

- **修改文件**: `src/humanoid_navigation2/humanoid_navigation2/scancontext_to_initialpose.py`
- **修改内容**:
  - **P0-2 SC bridge绕过修复**: 新增参数 `enable_runtime_auto_recovery` (默认 True 保持兼容)。`timer_callback()` 中区分启动/运行期：启动期照常检测 `localization_missing`；运行期若 `enable_runtime_auto_recovery=False`，只响应 fusion 的 `/localization/recovery_requests`，不自行根据 `/pcl_pose` stale 触发 recovery
- **修改原因**: 原逻辑在运行期检测到 `/pcl_pose` 停止更新 2.5s 就会自行启动 recovery，绕过 fusion 的 LOST 决策。融合模式下 recovery 应由 fusion 统一仲裁
- **影响范围**: SC bridge recovery 触发逻辑

- **修改文件**: `src/humanoid_navigation2/launch/navigation2_fusion_sc.launch.py`
- **修改内容**: SC bridge 参数新增 `enable_runtime_auto_recovery: False`
- **影响范围**: fusion+SC launch 配置

- **修改文件**: `src/humanoid_navigation/humanoid_navigation/navigation_state_manager_fusion.py`
- **修改内容**: `_on_fusion_status()` 解析逻辑改为先尝试 `json.loads`（新格式），失败后回退 `ast.literal_eval`（旧格式）
- **影响范围**: 向后兼容两种 fusion_status 格式

## [2026-05-26 00:38:53] 修复 odom 非标准坐标轴位移计算 bug

- **修改文件**: `src/humanoid_navigation2/humanoid_navigation2/localization_odom_fusion.py`
- **修改内容**:
  - **`_compute_odom_displacement`**: 位移计算从 `hypot(dx, dy)` 改为 `hypot(dx, dz)`——camera_init 帧中 (x=左右, z=前后) 才是 2D 水平面，y 是垂直轴
  - **`frozen_odom_body` 标准化**: 统一为 `(x, y, z)` 3-tuple，消除 dict/tuple 类型混用导致的潜在 KeyError
  - **`_is_healthy` pose gate 简化**: 去掉 body 帧坐标推算，直接比较 `frozen_map_odom` vs `latest_ndt_map_odom`（两者都是标准 ROS 坐标系），消除轴映射错误
  - **`_on_nav_status` 路点重置**: 位移计算改用 `(x, z)`，存储改用 3-tuple
  - **`__init__`**: 新增 `total_odom_displacement = 0.0` 初始化
- **修改原因**: Fast-LIO 的 camera_init 帧是非标准坐标轴 (x=左, y=下, z=后)，原代码用 `(x, y)` 计算水平位移遗漏了前后分量 z。直行 30m 时位移计数器只读 ~0m，单段/累计位移门限形同虚设
- **影响范围**: odom 位移计算准确性、DEGRADED→LOST 转换时机

## [2026-05-25 23:18:48] NDT + Odom Fusion + ScanContext 组合方案集成

- **修改文件**: `src/humanoid_navigation2/humanoid_navigation2/localization_odom_fusion.py`
- **修改内容**:
  - **Bug 修复**: 删除无效的 `from hdl_localization.msg import ScanMatchingStatus` 导入，将 `/localization/ndt_status` 订阅改为 `std_msgs.msg.String` (JSON)，修复 NDT 状态回调永不触发的致命 bug
  - **`_on_ndt_status` 回调重写**: 解析 JSON 字段 `fitness_score`、`has_converged`、`inlier_fraction`、`state`
  - **`_on_recovery_status` 回调增强**: 解析 JSON event_type，处理 `localization_recovery_started`(设标志)、`localization_relocalize_failed`(清标志)、`localization_recovered`(LOST→HEALTHY)
  - **新增 `_recovery_in_progress` 标志**: 防止 fusion 在 SC recovery 进行中重复发送 recovery 请求
  - **`_request_recovery` 增强**: 检查 `_recovery_in_progress` 标志，recovery 已在进行时跳过
  - **`_reset_state` 完善**: 重置 `_recovery_in_progress = False`
  - **`_is_healthy` 增强**: 新增 NDT pose 与 odom 推算位置一致性检查 (pose_jump > 0.8m 时拒绝恢复)，防止几何混叠区域的伪恢复
- **修改原因**: (1) NDT 发布 JSON 格式状态而非 `ScanMatchingStatus` 消息，fusion 节点回调永不触发导致 `latest_ndt_error` 恒为 inf，节点启动后立即永久 DEGRADED；(2) 原有 recovery 流程与 HDL bootstrap 绑定，需兼容 ScanContext bridge 的标准 recovery 事件协议；(3) 几何混叠区域可能出现低 error 但错位的伪恢复
- **影响范围**: fusion 定位状态机核心逻辑、NDT 恢复验收条件

- **新增文件**: `src/humanoid_navigation2/launch/navigation2_fusion_sc.launch.py`
- **新增内容**: NDT + Odom Fusion + ScanContext 组合 launch 文件，以 `navigation2_fusion.launch.py` 为模板：
  - 移除 HDL bootstrap 全部三个节点 (hdl_global_localization_node, hdl_bootstrap_container, hdl_bootstrap_to_initialpose_node)
  - 接入 ScanContext 两个节点 (scancontext_global_localizer_node + scancontext_to_initialpose_node)
  - 保留所有 fusion 节点参数（导航感知超时、odom 位移上限等）
  - 保留 NDT 节点 (reject_pose_jump=True, score_threshold=2.0)
- **修改原因**: 将 SC 全局重定位机制融入 fusion 体系，替换 HDL bootstrap (FPFH+RANSAC) 作为 LOST 后的全局重定位器
- **影响范围**: 定位层 launch 配置，新增第三种定位方案组合

- **新增文件**: `src/humanoid_navigation/launch/navigation_fusion_sc.launch.py`
- **新增内容**: APP 层 Fusion + SC 变体 launch 文件，使用 `navigation_state_manager_fusion` 作为可执行文件（与 HDL 版本相同，通过 `/localization/recovery_status` 标准接口与 SC bridge 兼容）
- **修改原因**: 提供与 fusion + SC 定位层对应的 APP 层 launch 入口
- **影响范围**: APP 层 launch 配置

## [2026-05-25 18:10:00] 定位漂移修复 - Recovery 加固 + NDT 阈值回滚

- **修改文件**: `src/humanoid_navigation2/launch/navigation_stack.launch.py`
- **修改内容**:
  - `global_localization_recovery_prior_hard_gate`: False → True (拒绝接收远离 prior 的全局定位候选)
  - `global_localization_recovery_prior_max_xy`: 3.0 → 4.0 (放宽 prior 半径)
  - `score_threshold`: 0.30 → 2.0 (回滚 NDT 匹配阈值，容忍 fitness 1.0+ 的噪声帧)
- **修改原因**: NDT 在 odom~(18.7,-0.7) 位置产生感知混叠，fitness 1.0-1.8 被 0.30 阈值拒绝，触发 recovery；recovery 因 soft gate 接受了 21m 外的错误候选，导致恶性循环。回滚 2.0 让系统容忍噪声帧维持跟踪，hard_gate 兜底防止 recovery 接受错误位姿。
- **影响范围**: NDT 定位容错性、recovery 行为

- **修改文件**: `src/humanoid_navigation2/humanoid_navigation2/hdl_bootstrap_to_initialpose.py`
- **修改内容**:
  - 新增参数 `trusted_pose_max_map_odom_displacement` (默认 3.0m)
  - `update_trusted_pose_if_healthy()` 新增 map->odom 位移检查，位移 >3m 时拒绝更新 trusted pose
- **修改原因**: 防止 NDT 漂移后的错误位姿污染 recovery prior，从源头阻断恶性循环
- **影响范围**: recovery prior 更新逻辑

- **新增文件**: `bag_waypoint_analysis.py`, `bag_final_analysis.py` — bag 包离线分析工具

## [2026-05-25 18:40:00] 新增 NDT定位 + 里程计融合节点

- **新增文件**: `src/humanoid_navigation2/humanoid_navigation2/localization_odom_fusion.py`
- **修改内容**:
  - 实现完整的 `LocalizationOdomFusion` 节点（~600 行，含详细中文注释）
  - 状态机: HEALTHY → DEGRADED → TRANSITIONING → HEALTHY（NDT 恢复）/ LOST（超时）
  - HEALTHY 时不干预 NDT，DEGRADED 时冻结 map->odom 让 Fast-LIO 里程计传播位姿
  - 平滑过渡: smoothstep + slerp 插值，避免位姿瞬移
  - 振荡保护: 连续帧确认机制（需连续2帧确认退化，连续3帧确认恢复）
  - 超时保护: 冻结超过 120s 或 odom 位移 >30m 时进入 LOST 触发 recovery
  - 诊断发布: /localization/fusion_status 和 /localization/fusion_odom_displacement
- **修改原因**: 当 NDT 因环境混叠漂移时，用里程计短期精度填补 NDT 局部不稳定，给 NDT 争取恢复时间的同时保持机器人正确运动
- **影响范围**: 定位系统容错性、导航连续性

- **修改文件**: `src/humanoid_navigation2/setup.py`
- **修改内容**: 注册 `localization_odom_fusion` 入口点
- **修改原因**: 将融合节点注册为 ROS2 console script

- **修改文件**: `src/humanoid_navigation2/launch/navigation_stack.launch.py`
- **修改内容**:
  - 新增 `localization_odom_fusion_node` 节点定义（含完整参数配置）
  - 新增 `TimerAction(period=8.0, actions=[localization_odom_fusion_node])`
  - 融合节点在 NDT 定位（period=5.0）和 lifecycle（period=7.0）之后启动
- **修改原因**: 集成融合节点到导航启动流程
- **影响范围**: 导航启动顺序

## [2026-05-25 20:15:00] 两条路架构 - 融合模式独立路径

- **新建文件**: `src/humanoid_navigation2/launch/navigation2_fusion.launch.py`
- **修改内容**: 复制自 navigation_stack.launch.py，改动:
  - hdl_bootstrap `ndt_failure_triggers_recovery`: True → **False** (不让hdl_bootstrap自动触发recovery)
  - hdl_bootstrap `hdl_divergence_triggers_recovery`: False → False (保持)
  - 保留 fusion 节点和所有其他配置
- **修改原因**: 融合模式下 fusion 节点决定何时 recovery，hdl_bootstrap 不应争抢控制权

- **新建文件**: `src/humanoid_navigation/launch/navigation_fusion.launch.py`
- **修改内容**: 复制自 navigation.launch.py，只改可执行文件名为 `navigation_state_manager_fusion`
- **修改原因**: 融合模式需使用专用的状态管理器

- **新建文件**: `src/humanoid_navigation/humanoid_navigation/navigation_state_manager_fusion.py`
- **修改内容**: 复制自原版 ~2700行, 只改 ~40行:
  - 类名改为 `NavigationStateManagerFusion`
  - 新增 `/localization/fusion_status` 订阅
  - `handle_localization_recovery_started()` 检查 fusion 状态: DEGRADED/TRANSITIONING → 跳过暂停和上报; LOST → 走原版 recovery 流程
  - 入口改为 `main_fusion()`
- **修改原因**: NDT 漂移时 fusion 节点用 odom 兜底，导航不应暂停，APP不应收到定位异常

- **修改文件**: `src/humanoid_navigation/setup.py` → 注册 `navigation_state_manager_fusion` 入口点

- **原文件完全不动**:
  - `navigation_stack.launch.py` — 路径1继续使用
  - `navigation.launch.py` — 路径1继续使用
  - `navigation_state_manager_recoverable.py` — 路径1继续使用

- **切换方式**:
  - 路径1: `ros2 launch humanoid_navigation2 navigation_stack.launch.py` + `ros2 launch humanoid_navigation navigation.launch.py`
  - 路径2: `ros2 launch humanoid_navigation2 navigation2_fusion.launch.py` + `ros2 launch humanoid_navigation navigation_fusion.launch.py`

## [2026-05-25 18:45:00] 工作规则更新

- **记忆文件**: `feedback_git_and_docs.md` — 修改前 git commit、修改后记录 CHANGELOG.md（精确到秒）
