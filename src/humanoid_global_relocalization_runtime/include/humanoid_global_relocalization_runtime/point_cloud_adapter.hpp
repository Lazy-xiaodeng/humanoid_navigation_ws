#pragma once

/*
 * point_cloud_adapter.hpp
 *
 * 文件作用：
 *   1. 负责把不同来源的点云统一转换成 base_footprint 标准轴 scan。
 *   2. 提供 PCD 读写、距离裁剪、Z 裁剪、体素降采样等验证阶段通用操作。
 *   3. 把坐标系细节隔离在 adapter 层，避免 3D-BBS 搜索后端直接关心 Fast-LIO 历史轴系。
 *
 * 坐标约定：
 *   - Fast-LIO raw/body：x 左、y 下、z 后。
 *   - ROS base_footprint：x 前、y 左、z 上。
 *   - 默认转换为 base = R * raw + t，其中 R 完成轴变换，t 是 raw body 原点在 base 下的位置。
 */

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "humanoid_global_relocalization_runtime/config.hpp"

namespace humanoid_global_relocalization
{

using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;
using CloudPtr = Cloud::Ptr;

CloudPtr load_pcd_xyz(const std::string & path);
void save_pcd_xyz(const std::string & path, const Cloud & cloud);
CloudPtr preprocess_map_cloud(const CloudPtr & cloud, const PreprocessConfig & config);
CloudPtr preprocess_scan_cloud(const CloudPtr & cloud, const PreprocessConfig & config);
CloudPtr transform_cloud(const CloudPtr & cloud, const Eigen::Matrix4d & transform);
CloudPtr convert_raw_body_to_base_cloud(const CloudPtr & raw_body_cloud, const FrameConfig & config);
std::vector<Eigen::Vector3d> cloud_to_eigen_points(const CloudPtr & cloud);
Eigen::Matrix4d xyzrpy_to_matrix(const std::vector<double> & xyzrpy);
double yaw_from_matrix(const Eigen::Matrix4d & matrix);

}  // namespace humanoid_global_relocalization
