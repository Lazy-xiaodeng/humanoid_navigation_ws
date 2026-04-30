/**
 * @file sycl_kernels.cpp
 * @brief SYCL GPU 加速内核实现
 * 
 * 使用 Intel oneAPI SYCL 实现点云滤波的 GPU 加速
 * 
 * 包含三个 GPU 加速函数：
 * 1. statisticalOutlierRemovalGPU - SOR 滤波
 * 2. heightContinuityFilterGPU - 高度连续性检查
 * 3. densityFilterGPU - 密度检查
 * 
 * 注意：这个文件只在定义了 USE_SYCL 时才会编译
 */

#ifdef USE_SYCL

#include "humanoid_point_cloud_filter/sycl_kernels.hpp"
#include <sycl/sycl.hpp>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>

namespace humanoid_point_cloud_filter
{
namespace sycl_kernels
{

/**
 * @brief 在 GPU 上计算两点之间的欧氏距离
 */
inline float distance3D(const sycl::float3 & a, const sycl::float3 & b)
{
  float dx = a.x() - b.x();
  float dy = a.y() - b.y();
  float dz = a.z() - b.z();
  return sycl::sqrt(dx * dx + dy * dy + dz * dz);
}

/**
 * @brief SOR 滤波 GPU 加速版本
 * 
 * 实现策略：
 * 1. 将点云数据传输到 GPU
 * 2. 每个 GPU 线程处理一个点
 * 3. 对每个点，暴力搜索 k 个最近邻（适合中小规模点云）
 * 4. 计算平均距离
 * 5. 在 CPU 上计算全局统计量并判断离群点
 * 
 * 注意：对于大规模点云（>10万点），建议使用更高级的 GPU KD-Tree
 * 
 * @param points 输入点云（Eigen 向量数组）
 * @param k 邻近点数量
 * @param std_ratio 标准差倍数
 * @return 每个点是否为内点的 mask
 */
std::vector<bool> statisticalOutlierRemovalGPU(
  const std::vector<Eigen::Vector3f> & points,
  int k,
  double std_ratio)
{
  const size_t n_points = points.size();
  std::vector<bool> inlier_mask(n_points, true);
  
  if (n_points < static_cast<size_t>(k)) {
    return inlier_mask;
  }
  
  try {
    // ===== 创建 SYCL 队列（自动选择 GPU）=====
    sycl::queue q(sycl::gpu_selector_v);
    
    // 打印设备信息（仅第一次）
    static bool first_run = true;
    if (first_run) {
      auto device = q.get_device();
      auto device_name = device.get_info<sycl::info::device::name>();
      std::cout << "[SYCL] 使用设备: " << device_name << std::endl;
      first_run = false;
    }
    
    // ===== 准备数据：转换为 SYCL 可用的格式 =====
    std::vector<sycl::float3> points_sycl(n_points);
    for (size_t i = 0; i < n_points; ++i) {
      points_sycl[i] = sycl::float3(points[i].x(), points[i].y(), points[i].z());
    }
    
    // ===== 分配 GPU 内存 =====
    sycl::buffer<sycl::float3, 1> buf_points(points_sycl.data(), sycl::range<1>(n_points));
    sycl::buffer<float, 1> buf_mean_distances{sycl::range<1>(n_points)};  // 使用花括号避免 vexing parse
    
    // ===== 提交 GPU 计算任务 =====
    q.submit([&](sycl::handler & h) {
      // 访问器
      auto acc_points = buf_points.get_access<sycl::access::mode::read>(h);
      auto acc_mean_dist = buf_mean_distances.get_access<sycl::access::mode::write>(h);
      
      // 并行计算每个点的平均邻近距离
      h.parallel_for(sycl::range<1>(n_points), [=](sycl::id<1> idx) {
        size_t i = idx[0];
        sycl::float3 current_point = acc_points[i];
        
        // 找到 k 个最近邻（暴力搜索）
        // 注意：这里使用简化的实现，适合中小规模点云
        // 对于大规模点云，应该使用 GPU KD-Tree
        
        // 限制 k 的最大值（避免栈溢出）
        const int k_max = 50;
        int k_actual = (k < k_max) ? k : k_max;
        
        // 存储最近的 k 个距离（使用插入排序维护）
        float nearest_distances[50];
        for (int j = 0; j < k_actual; ++j) {
          nearest_distances[j] = 1e10f;  // 初始化为很大的值
        }
        
        // 遍历所有点，找到最近的 k 个
        for (size_t j = 0; j < n_points; ++j) {
          if (i == j) continue;  // 跳过自己
          
          float dist = distance3D(current_point, acc_points[j]);
          
          // 如果这个距离比当前最远的近邻还小，插入到数组中
          if (dist < nearest_distances[k_actual - 1]) {
            // 找到插入位置
            int insert_pos = k_actual - 1;
            for (int m = 0; m < k_actual - 1; ++m) {
              if (dist < nearest_distances[m]) {
                insert_pos = m;
                break;
              }
            }
            
            // 后移元素
            for (int m = k_actual - 1; m > insert_pos; --m) {
              nearest_distances[m] = nearest_distances[m - 1];
            }
            
            // 插入新距离
            nearest_distances[insert_pos] = dist;
          }
        }
        
        // 计算平均距离
        float sum = 0.0f;
        for (int j = 0; j < k_actual; ++j) {
          sum += nearest_distances[j];
        }
        acc_mean_dist[i] = sum / k_actual;
      });
    }).wait();  // 等待 GPU 计算完成
    
    // ===== 从 GPU 读回平均距离 =====
    std::vector<float> mean_distances(n_points);
    {
      auto acc = buf_mean_distances.get_access<sycl::access::mode::read>();
      for (size_t i = 0; i < n_points; ++i) {
        mean_distances[i] = acc[i];
      }
    }
    
    // ===== 在 CPU 上计算全局统计量 =====
    double global_mean = 0.0;
    for (float d : mean_distances) {
      global_mean += d;
    }
    global_mean /= n_points;
    
    double global_std = 0.0;
    for (float d : mean_distances) {
      global_std += (d - global_mean) * (d - global_mean);
    }
    global_std = std::sqrt(global_std / n_points);
    
    // ===== 判断离群点 =====
    double threshold = global_mean + std_ratio * global_std;
    for (size_t i = 0; i < n_points; ++i) {
      inlier_mask[i] = (mean_distances[i] < threshold);
    }
    
  } catch (const sycl::exception & e) {
    // std::cerr << "[SYCL 错误] SOR 滤波失败: " << e.what() << std::endl;
    // 发生错误时，保留所有点
    std::fill(inlier_mask.begin(), inlier_mask.end(), true);
  }
  
  return inlier_mask;
}

/**
 * @brief 高度连续性检查 GPU 加速版本
 * 
 * 实现策略：
 * 1. 将点云数据传输到 GPU
 * 2. 每个 GPU 线程处理一个点
 * 3. 在指定半径内搜索邻近点
 * 4. 计算与邻近点的高度差
 * 5. 如果所有邻近点的高度差都超过阈值，标记为噪点
 * 
 * @param points 输入点云（Eigen 向量数组）
 * @param threshold 高度差阈值（米）
 * @param search_radius 搜索半径（米）
 * @return 每个点是否为内点的 mask
 */
std::vector<bool> heightContinuityFilterGPU(
  const std::vector<Eigen::Vector3f> & points,
  double threshold,
  double search_radius)
{
  const size_t n_points = points.size();
  std::vector<bool> inlier_mask(n_points, true);
  
  if (n_points < 5) {
    return inlier_mask;
  }
  
  try {
    // ===== 创建 SYCL 队列 =====
    sycl::queue q(sycl::gpu_selector_v);
    
    // ===== 准备数据 =====
    std::vector<sycl::float3> points_sycl(n_points);
    for (size_t i = 0; i < n_points; ++i) {
      points_sycl[i] = sycl::float3(points[i].x(), points[i].y(), points[i].z());
    }
    
    // ===== 分配 GPU 内存 =====
    sycl::buffer<sycl::float3, 1> buf_points(points_sycl.data(), sycl::range<1>(n_points));
    sycl::buffer<int, 1> buf_inlier{sycl::range<1>(n_points)};  // 使用花括号避免 vexing parse
    
    float threshold_f = static_cast<float>(threshold);
    float radius_f = static_cast<float>(search_radius);
    
    // ===== 提交 GPU 计算任务 =====
    q.submit([&](sycl::handler & h) {
      auto acc_points = buf_points.get_access<sycl::access::mode::read>(h);
      auto acc_inlier = buf_inlier.get_access<sycl::access::mode::write>(h);
      
      h.parallel_for(sycl::range<1>(n_points), [=](sycl::id<1> idx) {
        size_t i = idx[0];
        sycl::float3 current_point = acc_points[i];
        float current_z = current_point.z();
        
        // 在半径内搜索邻近点
        int valid_neighbors = 0;
        int height_diff_count = 0;
        
        for (size_t j = 0; j < n_points; ++j) {
          if (i == j) continue;
          
          sycl::float3 neighbor = acc_points[j];
          float dist = distance3D(current_point, neighbor);
          
          if (dist < radius_f) {
            valid_neighbors++;
            float height_diff = sycl::fabs(current_z - neighbor.z());
            
            if (height_diff > threshold_f) {
              height_diff_count++;
            }
          }
        }
        
        // 如果所有邻近点的高度差都超过阈值，标记为噪点
        if (valid_neighbors > 0 && height_diff_count == valid_neighbors) {
          acc_inlier[i] = 0;  // 噪点
        } else {
          acc_inlier[i] = 1;  // 内点
        }
      });
    }).wait();
    
    // ===== 从 GPU 读回结果 =====
    {
      auto acc = buf_inlier.get_access<sycl::access::mode::read>();
      for (size_t i = 0; i < n_points; ++i) {
        inlier_mask[i] = (acc[i] == 1);
      }
    }
    
  } catch (const sycl::exception & e) {
    // std::cerr << "[SYCL 错误] 高度连续性检查失败: " << e.what() << std::endl;
    std::fill(inlier_mask.begin(), inlier_mask.end(), true);
  }
  
  return inlier_mask;
}

/**
 * @brief 密度检查 GPU 加速版本
 * 
 * 实现策略：
 * 1. 将点云数据传输到 GPU
 * 2. 每个 GPU 线程处理一个点
 * 3. 统计半径内的点数
 * 4. 如果点数少于阈值，标记为孤立点
 * 
 * @param points 输入点云（Eigen 向量数组）
 * @param radius 检查半径（米）
 * @param min_points 最小点数
 * @return 每个点是否为内点的 mask
 */
std::vector<bool> densityFilterGPU(
  const std::vector<Eigen::Vector3f> & points,
  double radius,
  int min_points)
{
  const size_t n_points = points.size();
  std::vector<bool> inlier_mask(n_points, true);
  
  if (n_points < static_cast<size_t>(min_points)) {
    return inlier_mask;
  }
  
  try {
    // ===== 创建 SYCL 队列 =====
    sycl::queue q(sycl::gpu_selector_v);
    
    // ===== 准备数据 =====
    std::vector<sycl::float3> points_sycl(n_points);
    for (size_t i = 0; i < n_points; ++i) {
      points_sycl[i] = sycl::float3(points[i].x(), points[i].y(), points[i].z());
    }
    
    // ===== 分配 GPU 内存 =====
    sycl::buffer<sycl::float3, 1> buf_points(points_sycl.data(), sycl::range<1>(n_points));
    sycl::buffer<int, 1> buf_neighbor_counts{sycl::range<1>(n_points)};  // 使用花括号避免 vexing parse
    
    float radius_f = static_cast<float>(radius);
    
    // ===== 提交 GPU 计算任务 =====
    q.submit([&](sycl::handler & h) {
      auto acc_points = buf_points.get_access<sycl::access::mode::read>(h);
      auto acc_counts = buf_neighbor_counts.get_access<sycl::access::mode::write>(h);
      
      h.parallel_for(sycl::range<1>(n_points), [=](sycl::id<1> idx) {
        size_t i = idx[0];
        sycl::float3 current_point = acc_points[i];
        
        // 统计半径内的点数
        int count = 0;
        for (size_t j = 0; j < n_points; ++j) {
          float dist = distance3D(current_point, acc_points[j]);
          if (dist < radius_f) {
            count++;
          }
        }
        
        acc_counts[i] = count;
      });
    }).wait();
    
    // ===== 从 GPU 读回结果 =====
    {
      auto acc = buf_neighbor_counts.get_access<sycl::access::mode::read>();
      for (size_t i = 0; i < n_points; ++i) {
        inlier_mask[i] = (acc[i] >= min_points);
      }
    }
    
  } catch (const sycl::exception & e) {
    // std::cerr << "[SYCL 错误] 密度检查失败: " << e.what() << std::endl;
    std::fill(inlier_mask.begin(), inlier_mask.end(), true);
  }
  
  return inlier_mask;
}

}  // namespace sycl_kernels
}  // namespace humanoid_point_cloud_filter

#endif  // USE_SYCL