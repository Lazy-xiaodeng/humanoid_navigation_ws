#pragma once

/*
 * solid.hpp
 *
 * 文件作用：
 *   1. 定义 SOLiD RAH 描述子、数据库关键帧和查询结果。
 *   2. 提供描述子计算、数据库读写和候选航向生成接口。
 *   3. SOLiD 只负责候选召回，结果必须继续经过 GICP 与 temporal/trajectory 发布门控。
 */

#include <string>
#include <vector>

#include <Eigen/Dense>

#include "humanoid_global_relocalization_runtime/config.hpp"
#include "humanoid_global_relocalization_runtime/point_cloud_adapter.hpp"

namespace humanoid_global_relocalization
{

struct SolidDescriptor
{
  std::vector<float> range;
  std::vector<float> angle;
};

struct SolidEntry
{
  int cloud_index{-1};
  std::string source_id;
  Eigen::Matrix4d map_to_base{Eigen::Matrix4d::Identity()};
  SolidDescriptor descriptor;
  std::string cloud_path;
  CloudPtr cloud;
};

struct SolidMatch
{
  int rank{0};
  int cloud_index{-1};
  std::string source_id;
  double similarity{0.0};
  double relative_yaw_deg{0.0};
  Eigen::Matrix4d map_to_base{Eigen::Matrix4d::Identity()};
};

SolidDescriptor compute_solid_descriptor(const CloudPtr & scan_cloud, const SolidConfig & config);
CloudPtr prepare_solid_registration_cloud(const CloudPtr & scan_cloud, const SolidConfig & config);

std::vector<SolidMatch> query_solid_database(
  const SolidDescriptor & query,
  const std::vector<SolidEntry> & database,
  int current_cloud_index,
  const SolidConfig & config);

std::vector<SolidEntry> load_solid_database(const std::string & path);
void save_solid_database(const std::string & path, const std::vector<SolidEntry> & database);
Eigen::Matrix4d apply_solid_heading(const Eigen::Matrix4d & keyframe_pose, double relative_yaw_deg);

}  // namespace humanoid_global_relocalization
