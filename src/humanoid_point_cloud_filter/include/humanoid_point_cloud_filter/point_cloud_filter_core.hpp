#ifndef HUMANOID_POINT_CLOUD_FILTER__POINT_CLOUD_FILTER_CORE_HPP_
#define HUMANOID_POINT_CLOUD_FILTER__POINT_CLOUD_FILTER_CORE_HPP_

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <memory>

namespace humanoid_point_cloud_filter  
{

struct FilterConfig
{
  // 体素下采样
  double voxel_leaf_size;

  // SOR 参数
  int sor_k;
  double sor_std_ratio;
  double sor_std_ratio_moving;

  // 高度连续性
  double height_diff_threshold;
  double height_diff_threshold_moving;

  // 密度检查
  double density_radius;
  int min_density_points;
  int min_density_points_moving;

  // 运动检测
  double motion_threshold;
  int motion_history_size;

  // 开关
  bool enable_sor;
  bool enable_height_continuity;
  bool enable_density;
  bool enable_motion_detection;

  // 多线程配置
  int num_threads;  // 0表示使用OpenMP默认线程数
};

struct FilterTimings
{
  double sor_ms;
  double height_ms;
  double density_ms;
  double voxel_ms;
  double total_ms;
};

class PointCloudFilterCore
{
public:
  explicit PointCloudFilterCore(const FilterConfig & config);

  pcl::PointCloud<pcl::PointXYZI>::Ptr filter(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr & input,
    FilterTimings & timings);

  bool detectMotion(const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud);

  bool isMoving() const { return is_moving_; }

private:
  pcl::PointCloud<pcl::PointXYZI>::Ptr statisticalOutlierRemoval(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr & input,
    int k, double std_ratio, double & time_ms);

  pcl::PointCloud<pcl::PointXYZI>::Ptr heightContinuityFilter(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr & input,
    double threshold, double & time_ms);

  pcl::PointCloud<pcl::PointXYZI>::Ptr densityFilter(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr & input,
    double radius, int min_points, double & time_ms);

  pcl::PointCloud<pcl::PointXYZI>::Ptr voxelDownsample(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr & input,
    double leaf_size, double & time_ms);

  FilterConfig config_;

  // 运动检测状态
  Eigen::Vector3d prev_centroid_;
  std::vector<bool> motion_history_;
  bool is_moving_;
  bool first_frame_;
};

}  // namespace humanoid_point_cloud_filter

#endif  // HUMANOID_POINT_CLOUD_FILTER__POINT_CLOUD_FILTER_CORE_HPP_