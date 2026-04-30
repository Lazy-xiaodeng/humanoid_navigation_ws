#ifndef HUMANOID_POINT_CLOUD_FILTER__SYCL_KERNELS_HPP_
#define HUMANOID_POINT_CLOUD_FILTER__SYCL_KERNELS_HPP_

#ifdef USE_SYCL

#include <vector>
#include <Eigen/Dense>

namespace humanoid_point_cloud_filter
{
namespace sycl_kernels
{

// SOR 滤波 GPU 加速
std::vector<bool> statisticalOutlierRemovalGPU(
  const std::vector<Eigen::Vector3f> & points,
  int k,
  double std_ratio);

// 高度连续性检查 GPU 加速
std::vector<bool> heightContinuityFilterGPU(
  const std::vector<Eigen::Vector3f> & points,
  double threshold,
  double search_radius = 0.2);

// 密度检查 GPU 加速
std::vector<bool> densityFilterGPU(
  const std::vector<Eigen::Vector3f> & points,
  double radius,
  int min_points);

}  // namespace sycl_kernels
}  // namespace humanoid_point_cloud_filter

#endif  // USE_SYCL

#endif  // HUMANOID_POINT_CLOUD_FILTER__SYCL_KERNELS_HPP_