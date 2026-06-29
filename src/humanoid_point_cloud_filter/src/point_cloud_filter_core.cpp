/**
 * @file point_cloud_filter_core.cpp
 * @brief 点云滤波核心算法实现
 * 
 * 包含四种滤波算法：
 * 1. 统计离群点移除（SOR）
 * 2. 高度连续性检查
 * 3. 密度检查
 * 4. 体素下采样
 * 
 * 支持两种实现：
 * - 如果有 SYCL：使用 GPU 加速
 * - 如果没有 SYCL：使用 CPU 多线程（OpenMP）
 */

#include "humanoid_point_cloud_filter/point_cloud_filter_core.hpp"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/centroid.h>
#include <chrono>
#include <unordered_map>
#include <cmath>

#ifdef USE_SYCL
#include "humanoid_point_cloud_filter/sycl_kernels.hpp"
#endif

#ifdef _OPENMP
#include <omp.h>
#endif

namespace humanoid_point_cloud_filter
{

/**
 * @brief 构造函数 - 初始化配置和状态
 */
PointCloudFilterCore::PointCloudFilterCore(const FilterConfig & config)
: config_(config),
  is_moving_(false),
  first_frame_(true)
{
  motion_history_.reserve(config_.motion_history_size);
  
  // 设置OpenMP线程数
  if (config_.num_threads > 0) {
    #ifdef _OPENMP
    omp_set_num_threads(config_.num_threads);
    #endif
  }
}

/**
 * @brief 主滤波接口 - 按顺序执行所有滤波步骤
 * 
 * @param input 输入点云
 * @param timings 输出各步骤耗时
 * @return 滤波后的点云
 */
pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudFilterCore::filter(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & input,
  FilterTimings & timings)
{
  auto t_total_start = std::chrono::high_resolution_clock::now();
  timings.motion_ms = 0.0;
  timings.sor_ms = 0.0;
  timings.height_ms = 0.0;
  timings.density_ms = 0.0;
  timings.pre_voxel_ms = 0.0;
  timings.voxel_ms = 0.0;
  timings.total_ms = 0.0;
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = input;
  
  // ===== 运动检测 =====
  if (config_.enable_motion_detection) {
    auto t_motion_start = std::chrono::high_resolution_clock::now();
    is_moving_ = detectMotion(cloud);
    auto t_motion_end = std::chrono::high_resolution_clock::now();
    timings.motion_ms =
      std::chrono::duration<double, std::milli>(t_motion_end - t_motion_start).count();
  } else {
    timings.motion_ms = 0.0;
  }
  
  // 根据运动状态选择滤波参数
  double sor_std_ratio = is_moving_ ? config_.sor_std_ratio_moving : config_.sor_std_ratio;
  double height_threshold = is_moving_ ? config_.height_diff_threshold_moving : config_.height_diff_threshold;
  int min_density = is_moving_ ? config_.min_density_points_moving : config_.min_density_points;
  bool combined_sor_height_ran = false;

  if (config_.filter_mode == "voxel_density") {
    if (cloud->size() > 0) {
      cloud = voxelDownsample(cloud, config_.voxel_leaf_size, timings.voxel_ms);
      if (cloud->empty()) {
        return cloud;
      }
    }

    if (cloud->size() > static_cast<size_t>(min_density)) {
      cloud = voxelDensityOutlierFilter(
        cloud, config_.voxel_leaf_size, min_density, timings.density_ms);
      if (cloud->empty()) {
        return cloud;
      }
    }

    if (config_.enable_height_continuity && cloud->size() > 5) {
      cloud = heightContinuityFilter(cloud, height_threshold, timings.height_ms);
      if (cloud->empty()) {
        return cloud;
      }
    }

    auto t_total_end = std::chrono::high_resolution_clock::now();
    timings.total_ms = std::chrono::duration<double, std::milli>(t_total_end - t_total_start).count();
    return cloud;
  }

  // 可选实验 1：把最终 voxel 提前到 KNN 类滤波之前，后面不再重复 voxel。
  // 可选实验 2：额外做轻量 pre-voxel，最终仍保留末端 voxel。
  if (config_.voxel_before_filters && cloud->size() > 0) {
    cloud = voxelDownsample(cloud, config_.voxel_leaf_size, timings.voxel_ms);
    if (cloud->empty()) {
      return cloud;
    }
  } else if (config_.enable_pre_voxel_for_filter && cloud->size() > 0) {
    cloud = voxelDownsample(cloud, config_.pre_voxel_leaf_size, timings.pre_voxel_ms);
    if (cloud->empty()) {
      return cloud;
    }
  }
  
  if (
    config_.combine_sor_height_kdtree &&
    config_.enable_sor &&
    config_.enable_height_continuity &&
    cloud->size() > static_cast<size_t>(config_.sor_k) &&
    cloud->size() > 5)
  {
    cloud = combinedSorHeightFilter(
      cloud, config_.sor_k, sor_std_ratio, height_threshold, timings.sor_ms, timings.height_ms);
    combined_sor_height_ran = true;
    if (cloud->empty()) {
      return cloud;
    }
  }
  // ===== 第 1 步：统计离群点移除（SOR）=====
  else if (config_.enable_sor && cloud->size() > static_cast<size_t>(config_.sor_k)) {
    cloud = statisticalOutlierRemoval(cloud, config_.sor_k, sor_std_ratio, timings.sor_ms);
    if (cloud->empty()) {
      return cloud;
    }
  } else {
    timings.sor_ms = 0.0;
  }
  
  // ===== 第 2 步：高度连续性检查 =====
  if (
    !combined_sor_height_ran &&
    config_.enable_height_continuity &&
    cloud->size() > 5)
  {
    cloud = heightContinuityFilter(cloud, height_threshold, timings.height_ms);
    if (cloud->empty()) {
      return cloud;
    }
  } else if (!combined_sor_height_ran) {
    timings.height_ms = 0.0;
  }
  
  // ===== 第 3 步：密度检查 =====
  if (config_.enable_density && cloud->size() > static_cast<size_t>(min_density)) {
    cloud = densityFilter(cloud, config_.density_radius, min_density, timings.density_ms);
    if (cloud->empty()) {
      return cloud;
    }
  } else {
    timings.density_ms = 0.0;
  }
  
  // ===== 第 4 步：体素下采样 =====
  if (!config_.voxel_before_filters && cloud->size() > 0) {
    cloud = voxelDownsample(cloud, config_.voxel_leaf_size, timings.voxel_ms);
  } else {
    if (!config_.voxel_before_filters) {
      timings.voxel_ms = 0.0;
    }
  }
  
  // ===== 计算总耗时 =====
  auto t_total_end = std::chrono::high_resolution_clock::now();
  timings.total_ms = std::chrono::duration<double, std::milli>(t_total_end - t_total_start).count();
  
  return cloud;
}

/**
 * @brief 运动检测 - 通过点云质心变化判断机器人是否在运动
 * 
 * @param cloud 输入点云
 * @return true 表示在运动，false 表示静止
 */
bool PointCloudFilterCore::detectMotion(const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud)
{
  if (cloud->empty()) {
    return false;
  }
  
  // 计算点云质心
  Eigen::Vector4d centroid_4d;
  pcl::compute3DCentroid(*cloud, centroid_4d);
  Eigen::Vector3d current_centroid = centroid_4d.head<3>();
  
  if (first_frame_) {
    prev_centroid_ = current_centroid;
    first_frame_ = false;
    return false;
  }
  
  // 计算质心移动距离
  double motion_distance = (current_centroid - prev_centroid_).norm();
  
  // 更新运动历史
  motion_history_.push_back(motion_distance > config_.motion_threshold);
  if (motion_history_.size() > static_cast<size_t>(config_.motion_history_size)) {
    motion_history_.erase(motion_history_.begin());
  }
  
  // 判断是否在运动（最近几帧中大多数都在运动）
  int moving_count = 0;
  for (bool moving : motion_history_) {
    if (moving) moving_count++;
  }
  bool is_moving = moving_count > static_cast<int>(motion_history_.size()) / 2;
  
  prev_centroid_ = current_centroid;
  
  return is_moving;
}

/**
 * @brief 统计离群点移除（SOR）
 * 
 * 原理：
 * 1. 对每个点，找到其 k 个最近邻
 * 2. 计算该点到这 k 个邻近点的平均距离
 * 3. 计算所有点的平均距离的均值和标准差
 * 4. 如果某个点的平均距离超过 (均值 + std_ratio * 标准差)，则认为是离群点
 * 
 * @param input 输入点云
 * @param k 邻近点数量
 * @param std_ratio 标准差倍数（越小越严格）
 * @param time_ms 输出耗时（毫秒）
 * @return 过滤后的点云
 */
pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudFilterCore::statisticalOutlierRemoval(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & input,
  int k,
  double std_ratio,
  double & time_ms)
{
  auto t_start = std::chrono::high_resolution_clock::now();
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>);
  
  if (input->size() < static_cast<size_t>(k)) {
    *output = *input;
    time_ms = 0.0;
    return output;
  }
  
#ifdef USE_SYCL
  // ===== GPU 加速版本（SYCL）=====
  
  // 转换为 Eigen 向量数组
  std::vector<Eigen::Vector3f> points;
  points.reserve(input->size());
  for (const auto & p : input->points) {
    points.emplace_back(p.x, p.y, p.z);
  }
  
  // 调用 SYCL kernel
  std::vector<bool> inlier_mask = sycl_kernels::statisticalOutlierRemovalGPU(points, k, std_ratio);
  
  // 根据 mask 过滤点云
  output->reserve(input->size());
  for (size_t i = 0; i < input->size(); ++i) {
    if (inlier_mask[i]) {
      output->push_back(input->points[i]);
    }
  }
  
#else
  // ===== CPU 多线程版本（OpenMP）=====
  
  // 构建 KD-Tree
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud(input);
  
  // 计算每个点到其 k 个邻近点的平均距离
  std::vector<double> mean_distances(input->size());
  
  #pragma omp parallel
  {
    std::vector<int> indices(k + 1);
    std::vector<float> distances(k + 1);

    #pragma omp for
    for (size_t i = 0; i < input->size(); ++i) {
      indices.resize(k + 1);
      distances.resize(k + 1);

      kdtree.nearestKSearch(input->points[i], k + 1, indices, distances);

      // 计算平均距离（排除自己，即第一个点）
      double sum = 0.0;
      for (size_t j = 1; j < distances.size(); ++j) {
        sum += std::sqrt(distances[j]);
      }
      mean_distances[i] = sum / k;
    }
  }
  
  // 计算全局均值和标准差
  double global_mean = 0.0;
  #pragma omp parallel for reduction(+:global_mean)
  for (size_t i = 0; i < mean_distances.size(); ++i) {
    global_mean += mean_distances[i];
  }
  global_mean /= mean_distances.size();
  
  double global_std = 0.0;
  #pragma omp parallel for reduction(+:global_std)
  for (size_t i = 0; i < mean_distances.size(); ++i) {
    const double diff = mean_distances[i] - global_mean;
    global_std += diff * diff;
  }
  global_std = std::sqrt(global_std / mean_distances.size());
  
  // 判断离群点
  double threshold = global_mean + std_ratio * global_std;
  
  output->reserve(input->size());
  for (size_t i = 0; i < input->size(); ++i) {
    if (mean_distances[i] < threshold) {
      output->push_back(input->points[i]);
    }
  }
  
#endif
  
  auto t_end = std::chrono::high_resolution_clock::now();
  time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
  
  return output;
}

/**
 * @brief 合并 SOR 和高度连续性检查，共用一棵 KDTree。
 *
 * 原始实现会先对输入点云建树做 SOR，再对 SOR 输出重新建树做高度连续性。
 * 这里用同一棵输入点云 KDTree 先计算 SOR mask，再计算 height mask。
 * 实验版使用原始点云上的 6 近邻做高度检查，最终输出同时满足 SOR
 * 和高度连续性 mask。这样速度更快，但邻域和原始“先 SOR 再重建树”
 * 语义略有差异，所以必须通过 bag 对比后再打开。
 */
pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudFilterCore::combinedSorHeightFilter(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & input,
  int k,
  double std_ratio,
  double height_threshold,
  double & sor_time_ms,
  double & height_time_ms)
{
  auto t_sor_start = std::chrono::high_resolution_clock::now();

  pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>);
  if (input->size() < static_cast<size_t>(k) || input->size() < 6) {
    *output = *input;
    sor_time_ms = 0.0;
    height_time_ms = 0.0;
    return output;
  }

  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud(input);

  std::vector<double> mean_distances(input->size());

  #pragma omp parallel
  {
    std::vector<int> indices(k + 1);
    std::vector<float> distances(k + 1);

    #pragma omp for
    for (size_t i = 0; i < input->size(); ++i) {
      indices.resize(k + 1);
      distances.resize(k + 1);

      kdtree.nearestKSearch(input->points[i], k + 1, indices, distances);

      double sum = 0.0;
      for (size_t j = 1; j < distances.size(); ++j) {
        sum += std::sqrt(distances[j]);
      }
      mean_distances[i] = sum / k;
    }
  }

  double global_mean = 0.0;
  #pragma omp parallel for reduction(+:global_mean)
  for (size_t i = 0; i < mean_distances.size(); ++i) {
    global_mean += mean_distances[i];
  }
  global_mean /= mean_distances.size();

  double global_std = 0.0;
  #pragma omp parallel for reduction(+:global_std)
  for (size_t i = 0; i < mean_distances.size(); ++i) {
    const double diff = mean_distances[i] - global_mean;
    global_std += diff * diff;
  }
  global_std = std::sqrt(global_std / mean_distances.size());

  const double threshold = global_mean + std_ratio * global_std;
  std::vector<bool> sor_inlier_mask(input->size(), true);
  for (size_t i = 0; i < input->size(); ++i) {
    sor_inlier_mask[i] = mean_distances[i] < threshold;
  }

  auto t_sor_end = std::chrono::high_resolution_clock::now();
  sor_time_ms = std::chrono::duration<double, std::milli>(t_sor_end - t_sor_start).count();

  auto t_height_start = std::chrono::high_resolution_clock::now();
  std::vector<bool> height_inlier_mask(input->size(), true);
  const int search_neighbors = std::min<int>(6, static_cast<int>(input->size()));

  #pragma omp parallel
  {
    std::vector<int> indices(search_neighbors);
    std::vector<float> distances(search_neighbors);

    #pragma omp for
    for (size_t i = 0; i < input->size(); ++i) {
      if (!sor_inlier_mask[i]) {
        height_inlier_mask[i] = false;
        continue;
      }

      indices.resize(search_neighbors);
      distances.resize(search_neighbors);
      int found = kdtree.nearestKSearch(input->points[i], search_neighbors, indices, distances);

      if (found < 2) {
        continue;
      }

      const float current_z = input->points[i].z;
      int valid_neighbors = 0;
      int height_diff_count = 0;

      for (int j = 1; j < found; ++j) {
        const int neighbor_index = indices[j];
        if (neighbor_index < 0 || neighbor_index >= static_cast<int>(input->size())) {
          continue;
        }

        const float neighbor_z = input->points[neighbor_index].z;
        const float height_diff = std::abs(current_z - neighbor_z);
        valid_neighbors++;
        if (height_diff > height_threshold) {
          height_diff_count++;
        }
      }

      if (valid_neighbors > 0 && height_diff_count == valid_neighbors) {
        height_inlier_mask[i] = false;
      }
    }
  }

  output->reserve(input->size());
  for (size_t i = 0; i < input->size(); ++i) {
    if (sor_inlier_mask[i] && height_inlier_mask[i]) {
      output->push_back(input->points[i]);
    }
  }

  auto t_height_end = std::chrono::high_resolution_clock::now();
  height_time_ms = std::chrono::duration<double, std::milli>(t_height_end - t_height_start).count();

  return output;
}

/**
 * @brief 高度连续性检查
 * 
 * 原理：
 * 1. 对每个点，找到其周围的邻近点
 * 2. 计算该点与邻近点的高度差
 * 3. 如果所有邻近点的高度差都超过阈值，则认为是噪点（孤立的高点或低点）
 * 
 * 这个滤波器可以去除：
 * - 激光打到飞虫、灰尘产生的孤立点
 * - 地面反射产生的异常高点
 * - 传感器噪声产生的孤立低点
 * 
 * @param input 输入点云
 * @param threshold 高度差阈值（米）
 * @param time_ms 输出耗时（毫秒）
 * @return 过滤后的点云
 */
pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudFilterCore::heightContinuityFilter(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & input,
  double threshold,
  double & time_ms)
{
  auto t_start = std::chrono::high_resolution_clock::now();
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>);
  
  if (input->size() < 5) {
    *output = *input;
    time_ms = 0.0;
    return output;
  }
  
#ifdef USE_SYCL
  // ===== GPU 加速版本（SYCL）=====
  
  // 转换为 Eigen 向量数组
  std::vector<Eigen::Vector3f> points;
  points.reserve(input->size());
  for (const auto & p : input->points) {
    points.emplace_back(p.x, p.y, p.z);
  }
  
  // 调用 SYCL kernel（搜索半径设为 0.2 米）
  std::vector<bool> inlier_mask = sycl_kernels::heightContinuityFilterGPU(points, threshold, 0.2);
  
  // 根据 mask 过滤点云
  output->reserve(input->size());
  for (size_t i = 0; i < input->size(); ++i) {
    if (inlier_mask[i]) {
      output->push_back(input->points[i]);
    }
  }
  
#else
  // ===== CPU 多线程版本（OpenMP）=====
  
  // 构建 KD-Tree
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud(input);
  
  // 判断每个点是否保留
  std::vector<bool> inlier_mask(input->size(), true);
  
  constexpr int search_neighbors = 6;
  #pragma omp parallel
  {
    std::vector<int> indices(search_neighbors);  // 查询 6 个最近邻（包括自己）
    std::vector<float> distances(search_neighbors);

    #pragma omp for
    for (size_t i = 0; i < input->size(); ++i) {
      indices.resize(search_neighbors);
      distances.resize(search_neighbors);

      int found = kdtree.nearestKSearch(input->points[i], search_neighbors, indices, distances);

      if (found < 2) {
        // 如果连邻近点都找不到，保留该点
        continue;
      }

      // 计算与邻近点的高度差
      float current_z = input->points[i].z;
      int valid_neighbors = 0;
      int height_diff_count = 0;

      for (int j = 1; j < found; ++j) {  // 跳过自己（第一个点）
        if (indices[j] < 0) break;

        float neighbor_z = input->points[indices[j]].z;
        float height_diff = std::abs(current_z - neighbor_z);

        valid_neighbors++;
        if (height_diff > threshold) {
          height_diff_count++;
        }
      }

      // 如果所有邻近点的高度差都超过阈值，标记为噪点
      if (valid_neighbors > 0 && height_diff_count == valid_neighbors) {
        inlier_mask[i] = false;
      }
    }
  }
  
  // 根据 mask 过滤点云
  output->reserve(input->size());
  for (size_t i = 0; i < input->size(); ++i) {
    if (inlier_mask[i]) {
      output->push_back(input->points[i]);
    }
  }
  
#endif
  
  auto t_end = std::chrono::high_resolution_clock::now();
  time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
  
  return output;
}

/**
 * @brief 密度检查
 * 
 * 原理：
 * 1. 对每个点，统计其周围半径 radius 内的点数
 * 2. 如果点数少于 min_points，则认为是孤立点
 * 
 * 这个滤波器可以去除：
 * - 稀疏的噪点
 * - 远距离的不可靠点
 * - 边缘的不完整扫描点
 * 
 * @param input 输入点云
 * @param radius 检查半径（米）
 * @param min_points 最小点数
 * @param time_ms 输出耗时（毫秒）
 * @return 过滤后的点云
 */
pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudFilterCore::densityFilter(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & input,
  double radius,
  int min_points,
  double & time_ms)
{
  auto t_start = std::chrono::high_resolution_clock::now();
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>);
  
  if (input->size() < static_cast<size_t>(min_points)) {
    *output = *input;
    time_ms = 0.0;
    return output;
  }
  
#ifdef USE_SYCL
  // ===== GPU 加速版本（SYCL）=====
  
  // 转换为 Eigen 向量数组
  std::vector<Eigen::Vector3f> points;
  points.reserve(input->size());
  for (const auto & p : input->points) {
    points.emplace_back(p.x, p.y, p.z);
  }
  
  // 调用 SYCL kernel
  std::vector<bool> inlier_mask = sycl_kernels::densityFilterGPU(points, radius, min_points);
  
  // 根据 mask 过滤点云
  output->reserve(input->size());
  for (size_t i = 0; i < input->size(); ++i) {
    if (inlier_mask[i]) {
      output->push_back(input->points[i]);
    }
  }
  
#else
  // ===== CPU 多线程版本（OpenMP）=====
  
  // 构建 KD-Tree
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud(input);
  
  // 统计每个点周围的点数
  std::vector<int> neighbor_counts(input->size());
  
  #pragma omp parallel for
  for (size_t i = 0; i < input->size(); ++i) {
    std::vector<int> indices;
    std::vector<float> distances;
    
    // 半径搜索
    int found = kdtree.radiusSearch(input->points[i], radius, indices, distances);
    neighbor_counts[i] = found;
  }
  
  // 根据密度过滤
  output->reserve(input->size());
  for (size_t i = 0; i < input->size(); ++i) {
    if (neighbor_counts[i] >= min_points) {
      output->push_back(input->points[i]);
    }
  }
  
#endif
  
  auto t_end = std::chrono::high_resolution_clock::now();
  time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
  
  return output;
}

/**
 * @brief 体素邻域密度离群点过滤
 *
 * 这是 Autoware voxel/polar voxel outlier 思路的轻量实验版：
 * 先在体素级别统计占用，再用 3x3x3 邻域内的占用体素数量判断点是否稀疏。
 * 它避免了逐点 KDTree 半径/近邻搜索，适合实时链路先做低成本噪点过滤。
 */
pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudFilterCore::voxelDensityOutlierFilter(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & input,
  double leaf_size,
  int min_neighbor_voxels,
  double & time_ms)
{
  auto t_start = std::chrono::high_resolution_clock::now();

  pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>);
  if (input->empty()) {
    time_ms = 0.0;
    return output;
  }

  struct VoxelIndex {
    int x, y, z;

    bool operator==(const VoxelIndex & other) const {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct VoxelIndexHash {
    std::size_t operator()(const VoxelIndex & v) const {
      std::size_t seed = 0;
      seed ^= std::hash<int>()(v.x) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      seed ^= std::hash<int>()(v.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      seed ^= std::hash<int>()(v.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      return seed;
    }
  };

  auto toVoxel = [leaf_size](const pcl::PointXYZI & point) {
    VoxelIndex idx;
    idx.x = static_cast<int>(std::floor(point.x / leaf_size));
    idx.y = static_cast<int>(std::floor(point.y / leaf_size));
    idx.z = static_cast<int>(std::floor(point.z / leaf_size));
    return idx;
  };

  std::vector<VoxelIndex> point_voxels;
  point_voxels.reserve(input->size());

  std::unordered_map<VoxelIndex, int, VoxelIndexHash> occupancy;
  occupancy.reserve(input->size());

  for (const auto & point : input->points) {
    const VoxelIndex idx = toVoxel(point);
    point_voxels.push_back(idx);
    occupancy[idx] += 1;
  }

  const int min_required = std::max(1, min_neighbor_voxels);
  output->reserve(input->size());

  for (size_t i = 0; i < input->size(); ++i) {
    const VoxelIndex & center = point_voxels[i];
    int occupied_neighbors = 0;

    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          VoxelIndex neighbor{center.x + dx, center.y + dy, center.z + dz};
          if (occupancy.find(neighbor) != occupancy.end()) {
            occupied_neighbors++;
            if (occupied_neighbors >= min_required) {
              output->push_back(input->points[i]);
              dx = 2;
              dy = 2;
              break;
            }
          }
        }
      }
    }
  }

  auto t_end = std::chrono::high_resolution_clock::now();
  time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

  return output;
}

/**
 * @brief 体素下采样
 * 
 * 原理：
 * 1. 将 3D 空间划分为大小为 leaf_size 的体素网格
 * 2. 对于每个体素，只保留一个代表点（这里保留质心）
 * 3. 这样可以大幅减少点云数量，同时保持整体形状
 * 
 * 优点：
 * - 减少后续处理的计算量
 * - 使点云分布更均匀
 * - 减少内存占用
 * 
 * @param input 输入点云
 * @param leaf_size 体素大小（米）
 * @param time_ms 输出耗时（毫秒）
 * @return 下采样后的点云
 */
pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudFilterCore::voxelDownsample(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & input,
  double leaf_size,
  double & time_ms)
{
  auto t_start = std::chrono::high_resolution_clock::now();
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>);
  
  if (input->empty()) {
    time_ms = 0.0;
    return output;
  }
  
  // ===== 体素哈希表实现（CPU 版本，已经很快）=====
  
  // 定义体素索引的哈希函数
  struct VoxelIndex {
    int x, y, z;
    
    bool operator==(const VoxelIndex & other) const {
      return x == other.x && y == other.y && z == other.z;
    }
  };
  
  struct VoxelIndexHash {
    std::size_t operator()(const VoxelIndex & v) const {
      // 使用简单的哈希组合
      return std::hash<int>()(v.x) ^ 
             (std::hash<int>()(v.y) << 1) ^ 
             (std::hash<int>()(v.z) << 2);
    }
  };
  
  // 体素数据：累加点的坐标和强度，以及点数
  struct VoxelData {
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;
    float sum_intensity = 0.0f;
    int count = 0;
  };
  
  // 体素哈希表
  std::unordered_map<VoxelIndex, VoxelData, VoxelIndexHash> voxel_map;
  voxel_map.reserve(input->size() / 4);  // 预估下采样后点数
  
  // 将点分配到体素
  for (const auto & point : input->points) {
    // 计算体素索引
    VoxelIndex idx;
    idx.x = static_cast<int>(std::floor(point.x / leaf_size));
    idx.y = static_cast<int>(std::floor(point.y / leaf_size));
    idx.z = static_cast<int>(std::floor(point.z / leaf_size));
    
    // 累加到对应体素
    VoxelData & voxel = voxel_map[idx];
    voxel.sum_x += point.x;
    voxel.sum_y += point.y;
    voxel.sum_z += point.z;
    voxel.sum_intensity += point.intensity;
    voxel.count++;
  }
  
  // 计算每个体素的质心作为代表点
  output->reserve(voxel_map.size());
  for (const auto & pair : voxel_map) {
    const VoxelData & voxel = pair.second;
    
    pcl::PointXYZI centroid;
    centroid.x = voxel.sum_x / voxel.count;
    centroid.y = voxel.sum_y / voxel.count;
    centroid.z = voxel.sum_z / voxel.count;
    centroid.intensity = voxel.sum_intensity / voxel.count;
    
    output->push_back(centroid);
  }
  
  auto t_end = std::chrono::high_resolution_clock::now();
  time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
  
  return output;
}

}  // namespace humanoid_point_cloud_filter
