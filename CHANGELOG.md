# 变更记录

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

## [2026-05-25 18:45:00] 工作规则更新

- **记忆文件**: `feedback_git_and_docs.md` — 修改前 git commit、修改后记录 CHANGELOG.md（精确到秒）
