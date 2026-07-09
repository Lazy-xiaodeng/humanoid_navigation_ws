# Nav2 集成 FastDDS 共享内存优化说明

## ✅ 已完成的集成

已将 FastDDS 共享内存优化**直接集成**到你的 `navigation2.launch.py` 文件中。

---

## 🎯 集成的内容

### 1. 自动设置环境变量

在 launch 文件启动时，会自动设置以下三个环境变量：

```python
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
FASTRTPS_DEFAULT_PROFILES_FILE=~/.config/fastdds_shm.xml
RMW_FASTRTPS_USE_QOS_FROM_XML=1
```

### 2. 影响的节点

所有通过该 launch 文件启动的节点都会**自动使用共享内存**：

- ✅ `rslidar_sdk_node` - 雷达驱动（发布 `/airy_points`）
- ✅ `fast_lio_node` - Fast-LIO（订阅 `/airy_points`）
- ✅ `point_cloud_filter` - 点云滤波（订阅 `/airy_points`）
- ✅ `lidar_localization` - NDT 定位（订阅 `/fast_lio/cloud_registered`）
- ✅ `nav2` 所有节点（planner、controller 等）

### 3. 添加的参数

新增了一个 launch 参数，可以动态控制是否启用共享内存：

```bash
enable_fastdds_shm:=true   # 默认启用
enable_fastdds_shm:=false  # 禁用（用于对比测试）
```

---

## 🚀 使用方法

### 方式 1：正常启动（自动启用共享内存优化）

```bash
# 直接使用原来的命令，现在已经自动启用了共享内存优化
ros2 launch humanoid_navigation2 navigation2.launch.py use_sim_time:=false
```

### 方式 2：禁用共享内存（用于对比测试）

```bash
# 如果你想对比测试优化效果，可以禁用
ros2 launch humanoid_navigation2 navigation2.launch.py \
  use_sim_time:=false \
  enable_fastdds_shm:=false
```

---

## 🧪 验证共享内存是否生效

### 1. 启动系统

```bash
ros2 launch humanoid_navigation2 navigation2.launch.py use_sim_time:=false
```

### 2. 监控雷达频率（运行 5-10 分钟）

```bash
ros2 topic hz /airy_points
```

**预期效果**：
- ✅ average rate: **9.5-10 Hz**（稳定，不下降）
- ✅ std dev: **0.005-0.015s**（远低于之前的 0.21s）
- ✅ max 间隔: **0.1-0.2s**（远低于之前的 1.9s）

### 3. 检查共享内存文件

```bash
ls -lht /dev/shm/ | grep fastrtps
```

**预期输出**：
```
-rw-r--r-- 1 ubuntu ubuntu 201M 4月 24 10:00 fastrtps_xxxxxxxxxxxx
-rw-r--r-- 1 ubuntu ubuntu 201M 4月 24 10:00 fastrtps_yyyyyyyyyyyy
```

看到 **201M 大小的文件**说明共享内存已生效！

### 4. 检查环境变量

```bash
# 在启动后，新终端中检查
env | grep FASTRTPS
```

**预期输出**：
```
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
FASTRTPS_DEFAULT_PROFILES_FILE=/home/ubuntu/.config/fastdds_shm.xml
RMW_FASTRTPS_USE_QOS_FROM_XML=1
```

---

## 📊 优化效果对比

| 指标 | 优化前 | 优化后（预期） | 改善幅度 |
|------|--------|----------------|----------|
| average rate | 5.0 Hz（持续下降） | 9.5-10 Hz（稳定） | **+90%** |
| std dev | 0.21s（持续增长） | 0.005-0.015s | **-93%** |
| max 间隔 | 1.9s | 0.1-0.2s | **-89%** |
| 内存增长 | 持续增加 | 稳定 | **✓ 解决** |

---

## 🔍 技术细节

### 启动顺序

```
1. 设置 FastDDS 环境变量（自动）
2. 启动雷达驱动（使用共享内存发布 /airy_points）
3. 启动 Fast-LIO（通过共享内存订阅 /airy_points）
4. 启动点云滤波（通过共享内存订阅）
5. 启动 NDT 定位（通过共享内存订阅）
6. 启动 Nav2 导航（通过共享内存订阅）
```

所有节点都通过**共享内存**传递点云数据，零拷贝！

### 配置文件位置

- **FastDDS 配置**: `~/.config/fastdds_shm.xml`
- **Launch 文件**: `src/humanoid_navigation2/launch/navigation2.launch.py`

---

## ⚠️ 注意事项

### 1. 配置文件必须存在

确保 `~/.config/fastdds_shm.xml` 文件存在且配置正确。

**检查命令**：
```bash
ls -l ~/.config/fastdds_shm.xml
```

**如果不存在，创建它**：
```bash
# 文件已经在之前创建，如果丢失请重新创建
cat ~/.config/fastdds_shm.xml
```

### 2. 只影响本地节点

共享内存优化只在**同一台机器**上的节点间生效。

如果你的系统架构是：
- ✅ **单机运行**（所有节点在同一台机器）→ 效果最佳
- ⚠️ **多机分布式**（部分节点在其他机器）→ 只有本地节点受益

### 3. 内存占用

- 共享内存文件在 `/dev/shm/` 目录下
- 每个话题约 201MB
- 退出系统后**自动清理**

### 4. 异常退出清理

如果系统异常退出，共享内存文件可能残留：

```bash
# 清理残留的共享内存文件
rm -f /dev/shm/fastrtps_*
```

---

## 🐛 故障排查

### 问题 1：启动后频率仍然下降

**检查步骤**：

1. 确认配置文件存在
   ```bash
   ls -l ~/.config/fastdds_shm.xml
   ```

2. 检查环境变量是否设置
   ```bash
   # 在启动后的新终端中运行
   env | grep FASTRTPS
   ```

3. 查看 launch 输出日志
   ```bash
   ros2 launch humanoid_navigation2 navigation2.launch.py 2>&1 | grep -i "fastrtps\|shm\|shared"
   ```

**解决方法**：
- 确保配置文件路径正确
- 重新编译 workspace（虽然不需要，但可以排除缓存问题）
  ```bash
  colcon build --packages-select humanoid_navigation2
  ```

### 问题 2：看不到 /dev/shm 下的共享内存文件

**可能原因**：
- 配置文件未正确加载
- RMW 实现不是 FastDDS

**检查命令**：
```bash
# 确认 RMW 实现
echo $RMW_IMPLEMENTATION
# 应该输出：rmw_fastrtps_cpp
```

**解决方法**：
- 确保系统使用 FastDDS 作为 RMW 实现
- 检查配置文件 XML 语法是否正确

### 问题 3：想临时禁用共享内存

**方法 1：通过 launch 参数**
```bash
ros2 launch humanoid_navigation2 navigation2.launch.py \
  use_sim_time:=false \
  enable_fastdds_shm:=false
```

**方法 2：临时重命名配置文件**
```bash
mv ~/.config/fastdds_shm.xml ~/.config/fastdds_shm.xml.bak
# 启动系统
ros2 launch humanoid_navigation2 navigation2.launch.py use_sim_time:=false
# 恢复
mv ~/.config/fastdds_shm.xml.bak ~/.config/fastdds_shm.xml
```

---

## 📝 对比测试方法

如果你想验证优化效果，可以进行对比测试：

### 测试 1：使用共享内存（默认）

```bash
# 启动系统
ros2 launch humanoid_navigation2 navigation2.launch.py use_sim_time:=false

# 在另一个终端监控频率（运行 5 分钟）
ros2 topic hz /airy_points > /tmp/hz_with_shm.txt
```

### 测试 2：不使用共享内存

```bash
# 启动系统（禁用共享内存）
ros2 launch humanoid_navigation2 navigation2.launch.py \
  use_sim_time:=false \
  enable_fastdds_shm:=false

# 在另一个终端监控频率（运行 5 分钟）
ros2 topic hz /airy_points > /tmp/hz_without_shm.txt
```

### 对比结果

```bash
# 对比最后几行的统计数据
tail -20 /tmp/hz_with_shm.txt
tail -20 /tmp/hz_without_shm.txt
```

---

## 🎯 总结

✅ **你现在只需要**：

```bash
# 正常启动即可，共享内存优化已自动启用！
ros2 launch humanoid_navigation2 navigation2.launch.py use_sim_time:=false
```

✅ **无需额外操作**：
- 不需要手动设置环境变量
- 不需要修改其他 launch 文件
- 不需要重新编译（只需启动时自动生效）

✅ **如果想禁用**：
```bash
ros2 launch humanoid_navigation2 navigation2.launch.py \
  use_sim_time:=false \
  enable_fastdds_shm:=false
```

---

## 🔗 相关文件

- **Launch 文件**: `src/humanoid_navigation2/launch/navigation2.launch.py`
- **FastDDS 配置**: `~/.config/fastdds_shm.xml`
- **详细说明**: `~/humanoid_ws/FastDDS共享内存优化方案.md`
- **官方文档**: https://robosense.feishu.cn/wiki/KBr3wQgqZiLLrNk4pTdcNZfpnld
