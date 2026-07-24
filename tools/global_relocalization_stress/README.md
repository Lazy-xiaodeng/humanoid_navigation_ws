# 全局重定位可移植压测工具

本目录提供跨机器统一入口，复用正式功能包中的完整算法评估器，覆盖：

- 多个 bag、每个 bag 多轮去重随机 100 点；
- 全部导航点位的冷启动与定位恢复；
- 协调器、RO、Bridge、路线暂停恢复的集成合同；
- 最终 `全局候选 -> RO trusted commit -> Bridge 接受 -> map-odom 更新 -> TF 连续稳定`
  成功率、精度、耗时、CPU 和内存统计。

压测工具不会发布 `map->odom`。正式系统仍只允许 Bridge 发布该 TF。

## 文件说明

| 文件 | 用途 |
|---|---|
| `run_stress_suite.py` | 统一检查环境并顺序执行所有压测 |
| `stress_suite.example.yaml` | 跨机器配置模板 |
| `summarize_final_chain.py` | 按完整链路口径统计最终成功率 |
| `final_chain_results.example.csv` | 最终链路结果字段示例 |
| `CONVERSATION_HANDOFF.md` | 给另一台机器上的 Codex 使用的工程交接 |

bag、地图、SOLiD/SC 数据库和运行结果通常较大，不进入 Git；由 YAML 指向外部路径。

## 新机器准备

```bash
git clone -b feature/route-task-through-v1 <仓库地址> humanoid_ws
cd humanoid_ws
cp tools/global_relocalization_stress/stress_suite.example.yaml \
  tools/global_relocalization_stress/stress_suite.yaml
```

编辑 `stress_suite.yaml` 中的 bag 路径、导航点文件和 ROS 版本。先构建工作空间：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

检查输入，不启动压测：

```bash
python3 tools/global_relocalization_stress/run_stress_suite.py --check-only
```

查看将要执行的完整命令：

```bash
python3 tools/global_relocalization_stress/run_stress_suite.py --dry-run
```

正式执行：

```bash
python3 tools/global_relocalization_stress/run_stress_suite.py
```

每个任务的标准输出写入 `output/logs/`，执行命令、返回码和耗时写入
`output/run_manifest.json`。随机点和导航点的详细算法指标保存在各自子目录。

## 成功口径

随机点与导航点离线结果用于区分“未召回、算法拒绝、精配准失败”。正式成功率必须使用
`summarize_final_chain.py`，同时满足：

1. 全局候选已发布；
2. RO 完成 trusted commit；
3. Bridge 接受并成为唯一 `map->odom` 发布者；
4. TF 连续稳定达到配置帧数；
5. 真值平移误差不超过 `0.20m`。

报告同时列出 `5cm/10cm/20cm` 成功数。Bridge 已接受但超过误差门槛的样本计入
`false_accept`，不能计作成功。

单独汇总已有结果：

```bash
python3 tools/global_relocalization_stress/summarize_final_chain.py \
  --input /data/results/bag44.csv \
  --input /data/results/bag45.csv \
  --output /data/results/summary.json \
  --max-error 0.20 \
  --stable-tf-frames 20
```

## 注意事项

- `validation_gates` 依赖历史离线产物，新机器首次运行保持关闭。
- 冷启动与恢复应分组统计，不能用候选发布率代替最终 TF 成功率。
- 各轮必须使用不同 seed，并保留 manifest，失败点才能复现。
- 不要将 bag、`output/`、Codex 会话、Token 或 `~/.codex/auth.json` 提交到仓库。
