# 全局重定位工程交接

本文档用于在另一台机器开启新 Codex 会话时快速恢复工程上下文，不代替源码、Git 历史或原始会话。

## 工作约束

- 工作区：`humanoid_ws`
- 分支：`feature/route-task-through-v1`
- 只修改 Todesk 工作区，不修改 `/home/ubuntu/humanoid_ws`
- `map->odom` 只能由 Bridge 发布，任何重定位节点不得直接发布
- 成功定义：全局候选发布、RO trusted commit、Bridge 接受、TF 连续稳定且误差不超过 `0.20m`
- 统计分别给出冷启动和定位恢复，以及 `5cm/10cm/20cm` 成功数
- 宁可安全拒绝，不能误接受

## 正式链路

```text
定位不可信或冷启动
  -> 路线暂停并缓存目标
  -> 默认全局层
  -> SC/SOLiD 精度层
  -> trajectory 裁决
  -> 可选多初值连续跟踪兜底
  -> 全局候选
  -> RO trusted commit 和精匹配
  -> Bridge 接受并更新 map->odom
  -> 定位可信确认
  -> Nav2 ready
  -> 恢复缓存路线
```

多初值连续跟踪的正式参数入口为 `enable_multi_seed_recovery`。它只在前层失败后触发，
不在后台持续运行。最终是否默认开启，以正式 YAML 和最新验证记录为准。

## 验证重点

1. 使用本目录 `stress_suite.example.yaml` 配置所有外部 bag。
2. 每个 bag 至少三轮随机 100 点，seed 不重复。
3. 全部导航点分别验证冷启动与恢复。
4. 记录未召回、算法拒绝、RO 拒绝、Bridge 拒绝、TF 不稳定五类失败原因。
5. 最终结果交给 `summarize_final_chain.py`，不要把离线候选成功率当成系统成功率。
6. 对正常导航 bag 检查误停车、误触发重定位、路线缓存和恢复是否符合预期。

## 给新会话的开场文案

```text
请先阅读 tools/global_relocalization_stress/README.md、
tools/global_relocalization_stress/CONVERSATION_HANDOFF.md，以及
src/humanoid_global_relocalization_runtime/docs/ 下最新修改记录。
当前工作分支是 feature/route-task-through-v1。所有操作只在当前 Todesk 工作区进行。
先检查 git 状态和正式 YAML，不要覆盖未提交改动。后续压测以
“候选发布 -> RO trusted commit -> Bridge 接受 -> TF 连续稳定且误差 <= 0.20m”
为唯一成功口径，并分别报告冷启动/恢复的 5cm、10cm、20cm 成功率、耗时和资源。
```

## 会话迁移

Codex CLI 会话通常保存在 `CODEX_HOME`（默认 `~/.codex`）下的 `sessions/` JSONL 和状态数据库中。
完整迁移前应先正常退出 Codex，再通过加密介质迁移会话状态；不要把它们放进 Git。
新机器安装兼容版本、准备同一路径源码后，可使用：

```bash
codex resume <SESSION_ID> -C /path/to/humanoid_ws
```

直接复制单个 JSONL 不保证会话索引、附件和状态都完整，因此工程恢复应同时依赖本交接文档和 Git。
