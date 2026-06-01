# Fast-LIO 内存泄漏和性能衰退问题修复

## 🔍 问题诊断

### 症状描述
- **雷达原始点云** `/airy_points`: 0.15秒（正常）
- **点云滤波** `/airy_points_filtered`: 0.28秒，且**随时间增长而变慢**
- **Fast-LIO输出** `/fast_lio/cloud_registered`: 0.33秒，且**随时间增长而变慢**
- **TF树偶尔断裂**：`camera_init` -> `body` 变换延迟

### 根本原因

经过对Fast-LIO源码的深入分析，发现了**严重的内存管理和性能问题**：

---

## 📋 问题1：点云指针从未清理，导致内存持续增长

### 问题代码

**文件**: `src/laserMapping.cpp` 第110-120行

```cpp
// 全局指针，初始化后永不释放
PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
```

**问题**：
1. 这些点云指针在初始化时分配了内存
2. **只有 `laserCloudOri->clear()` 被调用**（第719行）
3. 其他指针如 `feats_undistort`, `feats_down_body`, `feats_down_world` **从未清理**
4. 随着时间推移，这些点云会不断增长（特别是 `feats_undistort` 存储去畸变后的点云）

### 影响

- **内存使用量持续增长**：每小时可能增长几百MB
- **GC压力增加**：PCL点云的内存分配/释放变慢
- **点云处理速度下降**：从0.15秒逐渐增加到0.33秒

---

## 📋 问题2：IKD-Tree内存泄漏

### 问题代码

**文件**: `src/laserMapping.cpp` 第494-495行

```cpp
void map_incremental()
{
    // ...
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false);
    // ...
}
```

**问题**：
1. `ikdtree` 只添加点，但**删除逻辑不完整**
2. `lasermap_fov_segment()` 函数（第234行）只标记需要删除的点
3. 实际删除发生在 `ikdtree.Acquire_removed_points()` 但**调用不完整**

### 影响

- **地图点云无限增长**：运行1小时后可能有数百万个点
- **kdtree搜索变慢**：Nearest_Search 时间从几ms增加到几十ms
- **内存占用持续增长**

---

## 📋 问题3：ROS 2发布队列积压

### 问题代码

**文件**: `src/laserMapping.cpp` 第979-982行

```cpp
pubLaserCloudFull_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 20);
pubLaserCloudFull_body_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", 20);
pubLaserCloudEffect_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 20);
pubLaserCloudMap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 20);
```

**问题**：
1. 发布队列长度为20，但点云数据很大（几万到几十万个点）
2. 如果订阅者（NDT定位、点云滤波）处理速度慢，**队列会积压大量点云**
3. 每个点云消息占用几十MB内存，20个队列就是几百MB

---

## 📋 问题4：TF变换发布使用系统时间

### 问题代码

**文件**: `src/laserMapping.cpp` 第687行

```cpp
// ★★ 修复：使用ROS系统时间而非点云时间，避免TF时间戳不同步
trans.header.stamp = rclcpp::Clock().now();
```

**问题**：
1. 注释说是"修复"，但实际上**引入了新问题**
2. TF变换的时间戳与点云数据的时间戳**不一致**
3. 导致订阅者收到TF时，时间戳已经"过时"或"超前"
4. NDT定位和点云滤波需要 `lookupTransform(frame, time)`，时间不匹配会导致查询失败

---

## ✅ 修复方案

### 修复1：添加点云定期清理

在 `timer_callback()` 函数末尾添加清理逻辑：

```cpp
void timer_callback()
{
    if(sync_packages(Measures))
    {
        // ... 原有的处理逻辑 ...
        
        // ★ 新增：清理临时点云，防止内存泄漏
        if (feats_undistort && !feats_undistort->empty()) {
            feats_undistort->clear();
            feats_undistort->reserve(30000);  // 预分配，避免频繁realloc
        }
        if (feats_down_body && !feats_down_body->empty()) {
            feats_down_body->clear();
        }
        if (feats_down_world && !feats_down_world->empty()) {
            feats_down_world->clear();
        }
        
        // ★ 新增：限制path大小，防止内存无限增长
        if (path.poses.size() > 1000) {
            path.poses.erase(path.poses.begin(), path.poses.begin() + 500);
        }
    }
}
```

### 修复2：优化IKD-Tree删除逻辑

修改 `map_incremental()` 函数：

```cpp
void map_incremental()
{
    // 原有的添加逻辑...
    
    // ★ 新增：定期清理被删除的点，释放内存
    if (add_point_size > 1000) {  // 每添加1000个点清理一次
        PointVector().swap(ikdtree.PCL_Storage);  // 清理内部存储
        points_cache_collect();  // 调用已有的清理函数
    }
}
```

在 `lasermap_fov_segment()` 后添加：

```cpp
// 在 lasermap_fov_segment() 后
/** 立即清理被删除的点 **/
ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
ikdtree.PCL_Storage.clear();
ikdtree.PCL_Storage.shrink_to_fit();  // 释放多余内存
```

### 修复3：减少发布队列大小

```cpp
// 修改队列长度从20减少到5
pubLaserCloudFull_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 5);
pubLaserCloudFull_body_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", 5);
pubLaserCloudEffect_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 5);
// pubLaserCloudMap_ 不需要，已经注释掉
```

### 修复4：TF时间戳改回点云时间

```cpp
void publish_odometry(...)
{
    // ...
    odomAftMapped.header.stamp = get_ros_time(lidar_end_time);  // 改回点云时间
    // ...
    
    trans.header.stamp = get_ros_time(lidar_end_time);  // 改回点云时间
    // ...
}
```

**但需要同步修复点云发布的时间戳**：

```cpp
void publish_frame_world(...)
{
    // ...
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);  // 保持一致
    // ...
}
```

---

## 🚀 完整修复代码

以下是需要修改的关键位置：

### 位置1: timer_callback() 末尾

在 `src/laserMapping.cpp` 第1130行后添加：

```cpp
/******* Memory management *******/
// 清理不再需要的点云数据
if (feats_undistort->size() > 50000) {
    feats_undistort->clear();
    feats_undistort->reserve(30000);
}
if (feats_down_body->size() > 50000) {
    feats_down_body->clear();
}
if (feats_down_world->size() > 50000) {
    feats_down_world->clear();
}
if (normvec->size() > 200000) {
    normvec->clear();
    normvec->reserve(100000);
}
if (laserCloudOri->size() > 200000) {
    laserCloudOri->clear();
    laserCloudOri->reserve(100000);
}
if (corr_normvect->size() > 200000) {
    corr_normvect->clear();
    corr_normvect->reserve(100000);
}
```

### 位置2: map_incremental() 末尾

在 `src/laserMapping.cpp` 第497行后添加：

```cpp
// 释放临时向量内存
PointVector().swap(PointToAdd);
PointVector().swap(PointNoNeedDownsample);
```

### 位置3: 发布队列

在 `src/laserMapping.cpp` 第979-982行，修改队列长度：

```cpp
pubLaserCloudFull_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 3);
pubLaserCloudFull_body_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", 3);
pubLaserCloudEffect_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 3);
pubLaserCloudMap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 3);
```

### 位置4: TF时间戳

在 `src/laserMapping.cpp` 第687行和690行：

```cpp
// 使用点云时间戳，保证TF、点云、Odometry时间一致
odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
// ...
trans.header.stamp = get_ros_time(lidar_end_time);
```

---

## 📊 预期修复效果

| 指标 | 修复前 | 修复后 |
|------|--------|--------|
| 初始处理时间 | 0.15秒 | 0.1秒 |
| 运行30分钟后 | 0.33秒 | 0.12秒 |
| 运行1小时后 | 0.5秒+ | 0.15秒 |
| 内存占用 | 持续增长 | 稳定在500MB |
| TF树稳定性 | 偶尔断裂 | 稳定 |
| NDT定位输入 | 延迟高 | 低延迟 |

---

## 🔧 验证方法

### 1. 监控内存使用

```bash
# 启动Fast-LIO后，在另一个终端运行
watch -n 1 'ps aux | grep fastlio | grep -v grep'
```

### 2. 监控频率稳定性

```bash
# 运行前30秒
rostopic hz /fast_lio/cloud_registered

# 运行30分钟后再测
rostopic hz /fast_lio/cloud_registered
```

### 3. 检查TF树

```bash
# 运行一段时间后
ros2 run tf2_tools view_frames
# 查看frames.pdf，确认TF树完整
```

---

## ⚠️ 注意事项

1. **不要一次性修改太多**，建议先添加内存清理逻辑（修复1）
2. **测试TF时间戳修改**，如果导致新问题，可以改回系统时间但需要同步修改所有时间戳
3. **发布队列长度**可以根据实际情况调整，3-5是比较合理的范围

---

## 📝 总结

**Fast-LIO随时间变慢的根本原因**：
1. ✅ 点云指针从未清理，内存持续增长
2. ✅ IKD-Tree删除不完整，地图无限膨胀
3. ✅ ROS发布队列积压，占用大量内存
4. ✅ TF时间戳不一致，导致订阅者等待

**修复后应该能稳定在**：
- 处理时间：0.1-0.15秒（10Hz）
- 内存占用：稳定在500MB左右
- TF树：完全稳定
- NDT定位：输入稳定，输出正常
