#pragma once

/*
 * simple_bbs3d.hpp
 *
 * 文件作用：
 *   1. 提供验证用的 CPU 版 3D-BBS 搜索后端。
 *   2. 输入地图点和当前 scan 点，输出 top-K 粗定位候选。
 *   3. 使用多分辨率 voxel map 和 priority queue 做 branch-and-bound，尽量贴近 3D-BBS 的核心思想。
 *
 * 重要说明：
 *   - 本实现优先服务工程验证：坐标链路、参数影响、资源统计和多 bag 对比。
 *   - 后续如果直接链接官方 KOKIAOKI/3d_bbs CPU/GPU 库，外层接口保持不变。
 *   - 当前候选得分是 scan 点落入地图 voxel 的数量，score_ratio=score/scan_points。
 */

#include <chrono>
#include <cstdint>
#include <memory>
#include <queue>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>

#include "humanoid_global_relocalization_runtime/config.hpp"

namespace humanoid_global_relocalization
{

namespace cpu_bbs3d_vendor
{
class Holder;
}

struct BbsCandidate
{
  Eigen::Matrix4d pose{Eigen::Matrix4d::Identity()};
  int score{0};
  double score_ratio{0.0};
  bool pre_refined{false};
  bool solid_primary{false};
  double refinement_fitness{0.0};
  double selection_score{0.0};
};

struct BbsResult
{
  bool localized{false};
  bool timed_out{false};
  double build_index_ms{0.0};
  double search_ms{0.0};
  std::vector<BbsCandidate> candidates;
};

class SimpleBbs3d
{
public:
  explicit SimpleBbs3d(BbsConfig config);
  ~SimpleBbs3d();

  void build_map_index(const std::vector<Eigen::Vector3d> & map_points);
  BbsResult localize(const std::vector<Eigen::Vector3d> & scan_points) const;

private:
  struct VoxelKey
  {
    int x{0};
    int y{0};
    int z{0};

    bool operator==(const VoxelKey & other) const
    {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct VoxelKeyHash
  {
    std::size_t operator()(const VoxelKey & key) const;
  };

  struct AngularInfo
  {
    Eigen::Vector3i num_division{1, 1, 1};
    Eigen::Vector3d rpy_res{0.0, 0.0, 0.0};
    Eigen::Vector3d min_rpy{0.0, 0.0, 0.0};
  };

  struct Branch
  {
    int score{0};
    int level{0};
    int tx{0};
    int ty{0};
    int tz{0};
    int roll{0};
    int pitch{0};
    int yaw{0};

    bool operator<(const Branch & other) const
    {
      return score < other.score;
    }
  };

  std::vector<double> voxel_resolutions() const;
  std::vector<AngularInfo> calculate_angular_info(const std::vector<Eigen::Vector3d> & scan_points) const;
  std::vector<Branch> create_initial_branches(const AngularInfo & angular_info) const;
  std::vector<Branch> branch_children(const Branch & parent, const AngularInfo & child_angular_info) const;
  Eigen::Matrix4d branch_to_matrix(const Branch & branch, double trans_res, const AngularInfo & angular_info) const;
  int score_branch(const Branch & branch, int level, const AngularInfo & angular_info, const std::vector<Eigen::Vector3d> & scan_points) const;
  void push_top_candidate(std::vector<BbsCandidate> & candidates, const BbsCandidate & candidate, int top_k) const;

  BbsConfig config_;
  std::unique_ptr<cpu_bbs3d_vendor::Holder> official_backend_;
  std::vector<std::unordered_set<VoxelKey, VoxelKeyHash>> voxel_maps_;
  std::vector<double> voxel_resolutions_;
};

}  // namespace humanoid_global_relocalization
