# RoboSense Airy 雷达频率优化方案

## 🔍 问题根源分析

### 当前配置问题

**config.yaml 当前配置**:
```yaml
split_frame_mode: 1        # 按角度分帧（默认）
start_angle: 0             # 0-360度完整扫描
end_angle: 360
split_angle: 0             # 在0度位置分帧
```

### 为什么只有3Hz？

**RoboSense Airy 96线是固态面阵雷达**，工作原理与机械旋转雷达不同：

1. **机械雷达**（如RS-LiDAR-32）:
   - 持续360度旋转
   - 每旋转一圈（10Hz）产生一帧
   - 角度跨越0度时分帧

2. **固态面阵雷达**（Airy 96线）:
   - **不是旋转扫描**，而是同时发射多个激光束
   - 完成一次"帧"扫描需要等待所有光束完成
   - 硬件扫描频率约**10Hz**（理论值）

### 代码分析

从 `decoder_RSAIRY.hpp` 可以看到：

```cpp
// 每个block的持续时间
float blk_ts = 111.080f;  // 微秒

// 默认RPS（每秒转数）
rps_(10)  // 10Hz
```

**理论计算**:
- 如果使用**按角度分帧**，需要等待完整的360度扫描
- 固态雷达完成一次扫描约 **100ms**（10Hz）
- **但实际观测到300ms（3Hz）**，说明有额外延迟

---

## ⚡ 优化方案

### 方案1：使用固定Block数量分帧（推荐）

这是**最有效的优化方案**，可以强制雷达SDK每收到一定数量的block就立即分帧，而不是等待角度条件。

**修改 `config.yaml`**:

```yaml
lidar:
  - driver:
      # ★★★ 关键修改 ★★★
      split_frame_mode: 3          # 使用自定义block数量分帧
      num_blks_split: 50           # 每50个block分一帧（可调）
      
      # 其他保持不变
      lidar_type: RSAIRY
      msop_port: 6699
      difop_port: 7788
      # ... 其他参数
```

**如何计算 `num_blks_split`**:

```
目标帧率 = 10Hz
每秒block数量 = 1秒 / BLOCK_DURATION = 1 / 0.000111080 ≈ 9000 blocks/秒

num_blks_split = 每秒block数 / 目标帧率
               = 9000 / 10
               = 900 blocks
```

**但是！** 这需要根据实际情况调整：

| num_blks_split | 预计帧率 | 说明 |
|----------------|----------|------|
| 900 | ~10Hz | 理论值（如果雷达真的输出这么快）|
| 300 | ~10Hz | 如果雷达本身只有10Hz输出 |
| **150-200** | **5-7Hz** | **推荐起始值，实际测试后调整** |

**建议测试步骤**:

1. 先设置为 `num_blks_split: 200`
2. 运行 `rostopic hz /airy_points`
3. 如果频率提高，继续减小这个值
4. 如果点云不完整（有缺失），增大这个值

---

### 方案2：检查DIFOP包中的RPM设置

雷达可能从DIFOP包中读取RPM设置，如果RPM被设置为600（10Hz），但实际硬件运行在更低频率：

**查看SDK日志**:
```bash
# 启动雷达SDK时查看输出
ros2 launch rslidar_sdk start.py | grep -i "rps\|rpm\|blks_per_frame"
```

如果看到类似输出：
```
rps:            3
blks_per_frame: 3000
```

说明RPS只有3（180RPM），这就是问题所在！

---

### 方案3：使用ROS 2工具诊断

```bash
# 1. 查看原始话题频率
rostopic hz /airy_points

# 2. 查看点云大小（点数）
rostopic echo /airy_points --no-arr | grep -A 2 "fields\|data"

# 3. 查看是否有丢包
ros2 topic hz /rslidar_packets  # 如果有这个话题
```

---

### 方案4：启用SDK详细日志

修改 `config.yaml` 或启动参数，启用debug日志：

```yaml
common:
  msg_source: 1
  send_point_cloud_ros: true
  # 添加日志级别（如果SDK支持）
  log_level: debug
```

或者通过环境变量：
```bash
export RCUTILS_CONSOLE_OUTPUT=1
export RS_DRIVER_LOG_LEVEL=debug
```

---

## 🎯 推荐测试方案

### 步骤1：修改config.yaml

```yaml
common:
  msg_source: 1
  send_packet_ros: false
  send_point_cloud_ros: true

lidar:
  - driver:
      lidar_type: RSAIRY
      msop_port: 6699
      difop_port: 7788
      imu_port: 6688
      
      # ★★★ 优化配置 ★★★
      split_frame_mode: 3          # 自定义block数量分帧
      num_blks_split: 200          # 初始值，后续可调整
      
      min_distance: 0.1
      max_distance: 60
      use_lidar_clock: false
      dense_points: false
      ts_first_point: true
      
      # 角度范围保持不变
      start_angle: 0
      end_angle: 360

    ros:
      ros_frame_id: rslidar
      ros_send_point_cloud_topic: /airy_points
      ros_queue_length: 100
```

### 步骤2：重新编译（如果需要）

```bash
cd ~/humanoid_ws
# 如果只是修改config.yaml，不需要重新编译
# 直接重启即可
```

### 步骤3：测试频率

```bash
# 启动雷达
ros2 launch rslidar_sdk start.py

# 在另一个终端测试频率
rostopic hz /airy_points

# 同时查看Fast-LIO输出
rostopic hz /fast_lio/cloud_registered
```

### 步骤4：调整参数

根据测试结果调整 `num_blks_split`:

- **频率太低** → 减小 `num_blks_split`
- **点云缺失** → 增大 `num_blks_split`
- **点云重叠/混乱** → 增大 `num_blks_split`

---

## 📊 预期效果

| 配置 | 预计频率 | 点云完整性 |
|------|----------|------------|
| 当前（split_frame_mode=1）| 3Hz | 完整 |
| num_blks_split=300 | 8-10Hz | 可能不完整 |
| num_blks_split=200 | 10-15Hz | 需要测试 |
| num_blks_split=100 | 15-20Hz | 可能碎片化 |

---

## ⚠️ 注意事项

1. **不要设置过小的 num_blks_split**
   - 太小会导致点云碎片化
   - 每帧点数太少，影响Fast-LIO和NDT定位

2. **检查点云质量**
   - 提高频率后，在RViz中检查点云是否完整
   - 是否有缺失或重叠

3. **Fast-LIO可能需要调整**
   - 如果频率大幅提高，Fast-LIO的 `num_sub_cloud` 参数可能需要调整
   - 点云滤波的队列大小可能需要增加

4. **网络带宽**
   - 更高的帧率意味着更大的网络带宽需求
   - 确保千兆网卡正常工作

---

## 🔧 如果优化后仍然只有3Hz

可能的原因：

1. **雷达硬件限制**
   - Airy 96线的硬件扫描频率就是3Hz
   - 查阅官方技术手册确认

2. **网络丢包**
   - UDP丢包导致帧无法正确组装
   - 使用 `tcpdump` 或 `wireshark` 检查

3. **固件版本**
   - 雷达固件可能有bug
   - 联系RoboSense技术支持确认

4. **安装模式问题**
   - Airy有不同的安装模式（normal/side/M）
   - 不同的安装模式可能影响扫描频率
   - 查看 `install_mode` 参数

---

## 📞 联系RoboSense技术支持

如果以上方案都无效，建议：

1. 提供雷达序列号和固件版本
2. 询问Airy 96线的**最大输出频率**
3. 确认是否有特殊的分帧配置要求
4. 询问是否有推荐的ROS 2配置示例
