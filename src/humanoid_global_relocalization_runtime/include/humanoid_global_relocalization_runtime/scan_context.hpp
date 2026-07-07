#pragma once

/*
 * scan_context.hpp
 *
 * 文件作用：
 *   1. 定义轻量 Scan Context 描述子，用于全局重定位冷启动候选召回。
 *   2. 把当前 LiDAR scan 编码成“距离环 x 方位扇区”的占据/高度矩阵。
 *   3. 对描述子做循环列移位匹配，得到相似 keyframe 和粗略 yaw 差。
 *   4. 为 3D-BBS/GICP 提供额外 map->base 初值，减少重复结构里漏掉正确候选的概率。
 *
 * 设计边界：
 *   - 本模块只负责候选召回，不直接决定是否发布恢复位姿。
 *   - 描述子匹配结果必须继续经过 scan-to-map 精配准和门控验证。
 *   - 当前实现使用手工描述子，便于 C++ 运行态部署；后续可替换 STD/LiDAR Iris 等更强描述子。
 */

#include <vector>
#include <string>

#include <Eigen/Dense>

#include "humanoid_global_relocalization_runtime/config.hpp"
#include "humanoid_global_relocalization_runtime/point_cloud_adapter.hpp"

namespace humanoid_global_relocalization
{

struct ScanContextDescriptor
{
  int rings{0};
  int sectors{0};
  std::vector<float> values;
};

struct ScanContextEntry
{
  int cloud_index{-1};
  Eigen::Matrix4d map_to_base{Eigen::Matrix4d::Identity()};
  ScanContextDescriptor descriptor;
};

struct ScanContextMatch
{
  int rank{0};
  int cloud_index{-1};
  double distance{0.0};
  int shift{0};
  Eigen::Matrix4d map_to_base{Eigen::Matrix4d::Identity()};
};

ScanContextDescriptor compute_scan_context_descriptor(
  const CloudPtr & scan_cloud,
  const ScanContextConfig & config);

std::pair<double, int> scan_context_distance(
  const ScanContextDescriptor & query,
  const ScanContextDescriptor & candidate);

std::vector<ScanContextMatch> query_scan_context_database(
  const ScanContextDescriptor & query,
  const std::vector<ScanContextEntry> & database,
  int current_cloud_index,
  const ScanContextConfig & config);

std::vector<ScanContextEntry> load_scan_context_database(const std::string & path);

void save_scan_context_database(
  const std::string & path,
  const std::vector<ScanContextEntry> & database);

Eigen::Matrix4d apply_scan_context_yaw_shift(
  const Eigen::Matrix4d & keyframe_pose,
  int shift,
  double yaw_offset_deg,
  const ScanContextConfig & config);

}  // namespace humanoid_global_relocalization
