# humanoid_global_localization

人形机器人全局定位功能包，基于多分辨率 NDT (Normal Distributions Transform) 网格搜索实现。

## 功能特性

- **任意位置启动**：通过多分辨率 NDT 网格搜索，在整个已建图区域内自动寻找初始位姿
- **稳定跟踪**：细分辨率 NDT + 指数平滑滤波，定位结果平滑不跳变
- **自动重定位**：跟踪丢失后在最后已知位姿附近自动搜索恢复
- **即插即用**：发布 `map→odom` TF 和 `/pcl_pose` 话题，与现有 TF 树和 Nav2 导航栈完全兼容

## 算法原理

```
启动后（无初始位姿）：
  ┌─────────────────────────────────────────┐
  │ 第一阶段：全局搜索                        │
  │  1. ROI 内以 1m 间距生成候选位姿         │
  │  2. 粗 NDT (3m 分辨率) 快速筛选          │
  │  3. 取 top-5 候选，中 NDT (1.5m) 精化   │
  │  4. 最佳候选 → 初始位姿                   │
  └─────────────────────────────────────────┘
                      ↓
  ┌─────────────────────────────────────────┐
  │ 第二阶段：持续跟踪                        │
  │  1. 每帧细 NDT (1m 分辨率) 匹配          │
  │  2. 指数平滑滤波                          │
  │  3. 发布 map→odom TF                     │
  └─────────────────────────────────────────┘
                      ↓ (连续失败)
  ┌─────────────────────────────────────────┐
  │ 第三阶段：局部重定位                      │
  │  在最后已知位姿 3m 范围内搜索恢复         │
  └─────────────────────────────────────────┘
```

### 与 hdl_localization 的对比

| 特性 | hdl_localization | 本节点 |
|------|-----------------|--------|
| 算法框架 | UKF + NDT | 多分辨率 NDT + 指数平滑 |
| 全局定位 | 独立模块 (分支定界) | 内建网格搜索 |
| ROS2 支持 | Humble 及以下 | **Jazzy 原生支持** |
| 外部依赖 | ndt_omp + fast_gicp | **仅 PCL** |
| 代码规模 | ~5000+ 行 | ~500 行 |
| 适用场景 | 大场景 (>50m²) | 中小场景 (<30m²) 最优 |
| 搜索速度 | 分支定界，大场景更快 | 网格搜索，中小场景相当 |

## 参数调优指南

### 1. 搜索范围 (全局搜索 ROI)

```yaml
search_x_min: -12.0
search_x_max:  12.0
search_y_min: -10.0
search_y_max:  10.0
search_step_xy: 1.0
```

| 参数 | 说明 | 调参建议 |
|------|------|---------|
| `search_x/y_min/max` | 搜索区域的 XY 边界 | 应略大于实际建图范围。例如地图覆盖 20m×15m，设置 ±12m × ±10m |
| `search_step_xy` | 网格步长 (米) | 越小搜索越精细但越慢。**1.0m 推荐**，环境特征少时可降到 0.5m |

**搜索耗时估算：**
```
候选数 = (range_x / step) × (range_y / step) × 4 朝向
耗时 ≈ 候选数 × 0.01 秒
```

示例：24m × 20m 区域，步长 1m → 24×20×4 = 1920 候选 → 约 20 秒

### 2. NDT 分辨率 (核心参数)

```yaml
coarse_ndt_resolution: 3.0    # 粗匹配分辨率 (米)
medium_ndt_resolution: 1.5    # 中匹配分辨率 (米)
fine_ndt_resolution: 1.0      # 细匹配分辨率 (米)
```

| 参数 | 作用 | 调大效果 | 调小效果 |
|------|------|---------|---------|
| `coarse_ndt_resolution` | 全局搜索阶段的分辨率 | 收敛范围更大，但可能漏掉正确位姿 | 搜索更精确，但变慢 |
| `medium_ndt_resolution` | top-K 精化阶段的分辨率 | 同上 | 同上 |
| `fine_ndt_resolution` | 跟踪阶段的分辨率 | 收敛更稳定，但精度下降 | 精度更高，但更容易不收敛 |

**推荐值：**
- 室内大厅/走廊：coarse=3.0, medium=1.5, fine=1.0
- 室外开阔场景：coarse=4.0, medium=2.0, fine=1.5
- 精细小场景：coarse=2.0, medium=1.0, fine=0.5

### 3. NDT 迭代次数

```yaml
coarse_ndt_iterations: 10
medium_ndt_iterations: 20
fine_ndt_iterations: 35
```

- 粗匹配迭代少（速度优先），细匹配迭代多（精度优先）
- 如果粗匹配经常漏掉正确位姿，增加 `coarse_ndt_iterations` 到 15
- 如果跟踪不稳定（频繁重定位），增加 `fine_ndt_iterations` 到 50

### 4. 匹配阈值

```yaml
score_threshold: 2.0                 # 跟踪模式阈值
relocalize_score_threshold: 5.0      # 全局/局部搜索阈值
consecutive_failures_for_reloc: 10   # 触发重定位的连续失败帧数
```

**fitness_score 含义**：匹配点云中每个点到其最近 NDT 网格中心的平均距离 (米)。越低越好。

| 症状 | 诊断 | 调整 |
|------|------|------|
| 定位频繁"丢失"（重定位触发太频繁） | `score_threshold` 太严 | 增大到 3.0 |
| 定位漂移但未触发重定位 | `score_threshold` 太松 | 减小到 1.5 |
| 全局搜索总是失败 | `relocalize_score_threshold` 太严 | 增大到 8.0 |
| 全局搜索返回错误位姿 | `relocalize_score_threshold` 太松 | 减小到 3.0 |
| 短暂遮挡导致重定位 | `consecutive_failures_for_reloc` 太小 | 增大到 20 |
| 已明显偏航仍未重定位 | `consecutive_failures_for_reloc` 太大 | 减小到 5 |

### 5. 位姿平滑

```yaml
smoothing_alpha: 0.3
```

| α 值 | 效果 | 适用场景 |
|------|------|---------|
| 0.0 | 无平滑，完全信任 NDT | NDT 非常稳定时 |
| 0.2~0.3 | 适度平滑，推荐值 | 大多数场景 |
| 0.5 | 强平滑，位姿变化很慢 | 机器人移动极慢时 |
| 0.8 | 几乎不更新 | 不推荐，会导致定位滞后 |

### 6. 点云滤波

```yaml
voxel_leaf_size: 0.2      # 体素降采样 (米)
scan_min_range: 1.0       # 近距离盲区 (米)
scan_max_range: 100.0     # 最大有效距离 (米)
```

- `voxel_leaf_size`：必须小于最小的 NDT 分辨率。通常设为 fine_ndt_resolution 的 1/5 ~ 1/2
- `scan_min_range`：过滤机器人自身遮挡点。人形机器人胸口雷达建议 1.0m
- `scan_max_range`：超过此距离的点不可靠，过滤。室内建议 30~60m

### 7. 重定位

```yaml
reloc_search_radius: 3.0
```

跟踪丢失后在最后已知位姿周围的搜索半径。太小可能搜不到，太大耗时增加。**3m 推荐**。

## 常见问题排查

### Q1: 全局搜索太慢 (>30秒)

减小搜索范围或增大步长：
```yaml
search_step_xy: 1.5      # 从 1.0 增大到 1.5
search_x_min: -8.0        # 缩小搜索范围
search_x_max: 8.0
```

或在 RViz 中使用 "2D Pose Estimate" 手动指定初始位姿，跳过全局搜索。

### Q2: 全局搜索总是失败 (best score > 阈值)

1. 确认机器人确实在已建图区域内
2. 检查环境是否发生了大变化 (家具移动、门开关等)
3. 增大 `relocalize_score_threshold` 到 8.0
4. 减小 `coarse_ndt_resolution` 到 2.0 (更精细搜索)

### Q3: 定位跳动/代价地图抖动

1. 减小 `smoothing_alpha` 到 0.2 (更强的平滑)
2. 确认 `map→odom` TF 发布频率与点云频率一致 (10Hz)
3. 检查 Fast-LIO 输出的点云是否稳定

### Q4: 靠近墙壁时定位丢失

1. 增大 `scan_min_range` (过滤更多近距离自身遮挡点)
2. 减小 `fine_ndt_resolution` (更精细的匹配)

### Q5: 转弯后定位偏移

这是人形机器人的固有问题 (旋转时产生平移)。解决方法：
1. 增大 `smoothing_alpha` 到 0.4 (更快跟踪实际位置)
2. 增大 `relocalize_score_threshold` (容忍更大的匹配误差)

## 话题接口

### 发布的话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/pcl_pose` | PoseWithCovarianceStamped | 定位结果 (map→odom 偏移量) |
| `/path` | Path | 机器人运动轨迹 |
| TF `map→odom` | TransformStamped | 核心 TF 变换 |

### 订阅的话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/fast_lio/cloud_registered` | PointCloud2 | Fast-LIO 实时点云 |
| `/initialpose` | PoseWithCovarianceStamped | RViz 初始位姿 |

## 启动方式

### 独立启动

```bash
ros2 launch humanoid_global_localization global_localization.launch.py
```

### 作为导航栈的一部分启动

```bash
ros2 launch humanoid_navigation2 navigation2.launch.py
```

导航栈 launch 文件已自动集成全局定位节点，在 Fast-LIO 启动 5 秒后自动启动。

## 使用流程

1. **启动导航栈** → 全局定位节点自动加载 PCD 地图
2. **首次启动 (无初始位姿)**：
   - 等待全局搜索完成 (终端会打印进度和最终结果)
   - 如果搜索失败，在 RViz 中使用 "2D Pose Estimate" 工具手动指定大致初始位姿
3. **后续启动 (已知位姿)**：
   - 在 RViz 中点击 "2D Pose Estimate" 指定机器人当前位置
   - 或在配置中设置 `set_initial_pose: true` 使用预设位姿
4. **运行中**：定位自动跟踪，丢失后自动重定位
