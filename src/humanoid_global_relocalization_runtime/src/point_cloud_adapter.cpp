/*
 * point_cloud_adapter.cpp
 *
 * 文件作用：
 *   1. 实现 PCD 读写和点云预处理。
 *   2. 实现 Fast-LIO raw body 到 base_footprint 标准轴的转换。
 *   3. 提供矩阵/点云转换工具，供 3D-BBS 和 GICP 共用。
 */

#include "humanoid_global_relocalization_runtime/point_cloud_adapter.hpp"

#include <cmath>
#include <filesystem>
#include <stdexcept>

#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

namespace humanoid_global_relocalization
{
namespace
{

CloudPtr remove_nan(const CloudPtr & input)
{
  CloudPtr output(new Cloud);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*input, *output, indices);
  return output;
}

CloudPtr voxel_downsample(const CloudPtr & input, double leaf_size)
{
  if (leaf_size <= 0.0) {
    return input;
  }
  CloudPtr output(new Cloud);
  pcl::VoxelGrid<Point> voxel;
  voxel.setInputCloud(input);
  voxel.setLeafSize(static_cast<float>(leaf_size), static_cast<float>(leaf_size), static_cast<float>(leaf_size));
  voxel.filter(*output);
  return output;
}

}  // namespace

CloudPtr load_pcd_xyz(const std::string & path)
{
  CloudPtr cloud(new Cloud);
  if (path.empty()) {
    throw std::runtime_error("pcd path is empty");
  }
  if (pcl::io::loadPCDFile<Point>(path, *cloud) != 0) {
    throw std::runtime_error("failed to load pcd: " + path);
  }
  return remove_nan(cloud);
}

void save_pcd_xyz(const std::string & path, const Cloud & cloud)
{
  const auto parent = std::filesystem::path(path).parent_path();
  if (!parent.empty()) {
    std::filesystem::create_directories(parent);
  }
  if (pcl::io::savePCDFileBinary(path, cloud) != 0) {
    throw std::runtime_error("failed to save pcd: " + path);
  }
}

CloudPtr preprocess_map_cloud(const CloudPtr & cloud, const PreprocessConfig & config)
{
  CloudPtr current = remove_nan(cloud);

  // 地图 Z 裁剪放在体素降采样前，能够先减少进入 VoxelGrid 的点数。
  if (config.enable_map_z_crop) {
    CloudPtr cropped(new Cloud);
    pcl::PassThrough<Point> pass;
    pass.setInputCloud(current);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(static_cast<float>(config.map_min_z), static_cast<float>(config.map_max_z));
    pass.filter(*cropped);
    current = cropped;
  }

  return voxel_downsample(current, config.map_leaf_size);
}

CloudPtr preprocess_scan_cloud(const CloudPtr & cloud, const PreprocessConfig & config)
{
  CloudPtr current(new Cloud);
  current->reserve(cloud->size());

  // scan 距离裁剪按欧氏距离执行，目的是去掉机器人自身附近点和远处稀疏噪点。
  const double min_r2 = config.min_scan_range * config.min_scan_range;
  const double max_r2 = config.max_scan_range * config.max_scan_range;
  for (const auto & point : cloud->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    const double r2 = point.x * point.x + point.y * point.y + point.z * point.z;
    if (r2 < min_r2 || r2 > max_r2) {
      continue;
    }
    // 可选 scan 高度裁剪用于削弱地面、机身低矮噪声和天花板平面在 BBS 评分中的影响。
    // 对全局重定位来说，墙面、门框、立柱等中高处结构通常比大片地面更有区分度。
    if (config.enable_scan_z_crop &&
      (point.z < config.scan_min_z || point.z > config.scan_max_z))
    {
      continue;
    }
    current->push_back(point);
  }
  current->width = current->size();
  current->height = 1;
  current->is_dense = true;

  return voxel_downsample(current, config.scan_leaf_size);
}

CloudPtr transform_cloud(const CloudPtr & cloud, const Eigen::Matrix4d & transform)
{
  CloudPtr output(new Cloud);
  pcl::transformPointCloud(*cloud, *output, transform.cast<float>());
  return output;
}

CloudPtr convert_raw_body_to_base_cloud(const CloudPtr & raw_body_cloud, const FrameConfig & config)
{
  if (!config.convert_raw_body_to_base) {
    return raw_body_cloud;
  }

  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform(0, 2) = -1.0;
  transform(1, 0) = 1.0;
  transform(2, 1) = -1.0;
  transform(0, 0) = 0.0;
  transform(1, 1) = 0.0;
  transform(2, 2) = 0.0;
  transform.block<3, 1>(0, 3) = config.raw_body_to_base_xyz;

  return transform_cloud(raw_body_cloud, transform);
}

std::vector<Eigen::Vector3d> cloud_to_eigen_points(const CloudPtr & cloud)
{
  std::vector<Eigen::Vector3d> points;
  points.reserve(cloud->size());
  for (const auto & point : cloud->points) {
    points.emplace_back(point.x, point.y, point.z);
  }
  return points;
}

Eigen::Matrix4d xyzrpy_to_matrix(const std::vector<double> & xyzrpy)
{
  if (xyzrpy.size() != 6) {
    throw std::runtime_error("xyzrpy vector must contain 6 values");
  }
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  const Eigen::AngleAxisd roll(xyzrpy[3], Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch(xyzrpy[4], Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw(xyzrpy[5], Eigen::Vector3d::UnitZ());
  matrix.block<3, 3>(0, 0) = (yaw * pitch * roll).toRotationMatrix();
  matrix(0, 3) = xyzrpy[0];
  matrix(1, 3) = xyzrpy[1];
  matrix(2, 3) = xyzrpy[2];
  return matrix;
}

double yaw_from_matrix(const Eigen::Matrix4d & matrix)
{
  return std::atan2(matrix(1, 0), matrix(0, 0));
}

}  // namespace humanoid_global_relocalization
