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
