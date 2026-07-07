/*
 * simple_bbs3d.cpp
 *
 * 文件作用：
 *   1. 实现验证使用的 CPU 版 3D-BBS。
 *   2. 使用多分辨率 voxel map 对候选位姿评分，并通过分支定界减少完整枚举成本。
 *   3. 输出 top-K 粗位姿候选，供后续 GICP 精配准和人工/RViz 检查。
 *
 * 算法说明：
 *   - 地图点云被构造成多个分辨率的 voxel set，level 越高分辨率越粗。
 *   - 搜索从最高 level 的大 voxel 和粗角度开始。
 *   - 每个分支的 score 是当前 scan 变换后命中地图 voxel 的点数。
 *   - priority_queue 总是优先展开高分分支；低于当前 top-K 最低叶子分数的分支会被剪枝。
 *   - 到达 level=0 时得到一个叶子候选，转成连续位姿矩阵保存。
 */

#include "humanoid_global_relocalization_runtime/simple_bbs3d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <cpu_bbs3d/bbs3d.hpp>

namespace humanoid_global_relocalization
{
namespace cpu_bbs3d_vendor
{
class Holder
{
public:
  // 官方 3D-BBS CPU 后端对象。放在 Holder 里是为了避免在 public header 中直接暴露第三方类型，
  // 后续切换 GPU 后端或替换库版本时，外层接口不需要变化。
  std::unique_ptr<cpu::BBS3D> bbs;
};
}  // namespace cpu_bbs3d_vendor

namespace
{

double clamp_cosine(double value)
{
  return std::max(-1.0, std::min(1.0, value));
}

}  // namespace

std::size_t SimpleBbs3d::VoxelKeyHash::operator()(const VoxelKey & key) const
{
  // 使用常见的三维整数哈希质数混合。这里不要求密码学安全，只需要在 voxel 坐标上分布均匀。
  return static_cast<std::size_t>(
    (key.x * 73856093) ^ (key.y * 19349663) ^ (key.z * 83492791));
}

SimpleBbs3d::SimpleBbs3d(BbsConfig config)
: config_(std::move(config))
{
  voxel_resolutions_ = voxel_resolutions();
  official_backend_ = std::make_unique<cpu_bbs3d_vendor::Holder>();
}

SimpleBbs3d::~SimpleBbs3d() = default;

std::vector<double> SimpleBbs3d::voxel_resolutions() const
{
  std::vector<double> resolutions;
  resolutions.reserve(static_cast<std::size_t>(config_.max_level + 1));
  double res = config_.min_level_res;
  for (int level = 0; level <= config_.max_level; ++level) {
    resolutions.push_back(res);
    res *= config_.voxel_expansion_rate;
  }
  return resolutions;
}

void SimpleBbs3d::build_map_index(const std::vector<Eigen::Vector3d> & map_points)
{
  // 正式使用官方 3D-BBS CPU 后端构建多分辨率 voxel map。
  // 下面保留本文件自带的 voxel_maps_ 构建，作为 /fallback 数据结构和后续 top-K 扩展参考。
  official_backend_->bbs = std::make_unique<cpu::BBS3D>();
  official_backend_->bbs->set_voxel_expantion_rate(config_.voxel_expansion_rate);
  official_backend_->bbs->set_tar_points(map_points, config_.min_level_res, config_.max_level);
  official_backend_->bbs->set_trans_search_range(config_.search_min_xyz, config_.search_max_xyz);
  official_backend_->bbs->set_angular_search_range(config_.search_min_rpy, config_.search_max_rpy);
  official_backend_->bbs->set_score_threshold_percentage(config_.score_threshold_percentage);
  official_backend_->bbs->set_top_k(config_.top_k);
  official_backend_->bbs->set_num_threads(config_.num_threads);
  if (config_.timeout_msec > 0) {
    official_backend_->bbs->enable_timeout();
    official_backend_->bbs->set_timeout_duration_in_msec(config_.timeout_msec);
  } else {
    official_backend_->bbs->disable_timeout();
  }

  voxel_maps_.clear();
  voxel_maps_.resize(static_cast<std::size_t>(config_.max_level + 1));

  // 每一层单独构造 unordered_set。粗层 voxel 数少，用于快速给分支打上界分；
  // 细层 voxel 多，用于最终 leaf 候选评分。
  for (int level = 0; level <= config_.max_level; ++level) {
    const double inv_res = 1.0 / voxel_resolutions_[static_cast<std::size_t>(level)];
    auto & voxel_set = voxel_maps_[static_cast<std::size_t>(level)];
    voxel_set.reserve(map_points.size());
    for (const auto & point : map_points) {
      VoxelKey key;
      key.x = static_cast<int>(std::floor(point.x() * inv_res));
      key.y = static_cast<int>(std::floor(point.y() * inv_res));
      key.z = static_cast<int>(std::floor(point.z() * inv_res));
      voxel_set.insert(key);
    }
  }
}

std::vector<SimpleBbs3d::AngularInfo> SimpleBbs3d::calculate_angular_info(
  const std::vector<Eigen::Vector3d> & scan_points) const
{
  std::vector<AngularInfo> infos(static_cast<std::size_t>(config_.max_level + 1));

  // 3D-BBS 用 scan 最大半径和当前 voxel 分辨率推导角度分辨率：
  // 粗 voxel 允许更大的角度误差，细 voxel 才需要更细的角度划分。
  double max_norm = 0.0;
  for (const auto & point : scan_points) {
    max_norm = std::max(max_norm, point.norm());
  }
  max_norm = std::max(max_norm, 1e-6);

  for (int level = config_.max_level; level >= 0; --level) {
    const double res = voxel_resolutions_[static_cast<std::size_t>(level)];
    const double cosine = 1.0 - (res * res / (max_norm * max_norm)) * 0.5;
    const double ori_res = std::acos(clamp_cosine(cosine));

    Eigen::Vector3d candidate_res;
    for (int axis = 0; axis < 3; ++axis) {
      const double range = std::abs(config_.search_max_rpy[axis] - config_.search_min_rpy[axis]);
      candidate_res[axis] = ori_res <= range ? ori_res : 0.0;
    }

    Eigen::Vector3d piece_range;
    if (level == config_.max_level) {
      piece_range = config_.search_max_rpy - config_.search_min_rpy;
    } else {
      const auto & parent = infos[static_cast<std::size_t>(level + 1)];
      for (int axis = 0; axis < 3; ++axis) {
        piece_range[axis] = parent.rpy_res[axis] != 0.0 ?
          parent.rpy_res[axis] :
          config_.search_max_rpy[axis] - config_.search_min_rpy[axis];
      }
    }

    AngularInfo info;
    for (int axis = 0; axis < 3; ++axis) {
      info.num_division[axis] = candidate_res[axis] == 0.0 ?
        1 :
        std::max(1, static_cast<int>(std::ceil(piece_range[axis] / candidate_res[axis])));
      info.rpy_res[axis] = info.num_division[axis] == 1 ?
        0.0 :
        piece_range[axis] / static_cast<double>(info.num_division[axis]);
      info.min_rpy[axis] = info.rpy_res[axis] == 0.0 ? 0.0 : config_.search_min_rpy[axis];
    }
    infos[static_cast<std::size_t>(level)] = info;
  }

  return infos;
}

std::vector<SimpleBbs3d::Branch> SimpleBbs3d::create_initial_branches(const AngularInfo & angular_info) const
{
  const int level = config_.max_level;
  const double res = voxel_resolutions_[static_cast<std::size_t>(level)];

  const int min_x = static_cast<int>(std::floor(config_.search_min_xyz.x() / res));
  const int max_x = static_cast<int>(std::ceil(config_.search_max_xyz.x() / res));
  const int min_y = static_cast<int>(std::floor(config_.search_min_xyz.y() / res));
  const int max_y = static_cast<int>(std::ceil(config_.search_max_xyz.y() / res));
  const int min_z = static_cast<int>(std::floor(config_.search_min_xyz.z() / res));
  const int max_z = static_cast<int>(std::ceil(config_.search_max_xyz.z() / res));

  std::vector<Branch> branches;
  for (int x = min_x; x <= max_x; ++x) {
    for (int y = min_y; y <= max_y; ++y) {
      for (int z = min_z; z <= max_z; ++z) {
        for (int roll = 0; roll < angular_info.num_division.x(); ++roll) {
          for (int pitch = 0; pitch < angular_info.num_division.y(); ++pitch) {
            for (int yaw = 0; yaw < angular_info.num_division.z(); ++yaw) {
              branches.push_back(Branch{0, level, x, y, z, roll, pitch, yaw});
            }
          }
        }
      }
    }
  }
  return branches;
}

std::vector<SimpleBbs3d::Branch> SimpleBbs3d::branch_children(
  const Branch & parent, const AngularInfo & child_angular_info) const
{
  const int rate = std::max(1, static_cast<int>(std::round(config_.voxel_expansion_rate)));
  const int child_level = parent.level - 1;
  std::vector<Branch> children;

  for (int ix = 0; ix < rate; ++ix) {
    for (int iy = 0; iy < rate; ++iy) {
      for (int iz = 0; iz < rate; ++iz) {
        for (int roll = 0; roll < child_angular_info.num_division.x(); ++roll) {
          for (int pitch = 0; pitch < child_angular_info.num_division.y(); ++pitch) {
            for (int yaw = 0; yaw < child_angular_info.num_division.z(); ++yaw) {
              Branch child;
              child.level = child_level;
              child.tx = parent.tx * rate + ix;
              child.ty = parent.ty * rate + iy;
              child.tz = parent.tz * rate + iz;
              child.roll = parent.roll * child_angular_info.num_division.x() + roll;
              child.pitch = parent.pitch * child_angular_info.num_division.y() + pitch;
              child.yaw = parent.yaw * child_angular_info.num_division.z() + yaw;
              children.push_back(child);
            }
          }
        }
      }
    }
  }
  return children;
}

Eigen::Matrix4d SimpleBbs3d::branch_to_matrix(
  const Branch & branch, double trans_res, const AngularInfo & angular_info) const
{
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  matrix(0, 3) = static_cast<double>(branch.tx) * trans_res;
  matrix(1, 3) = static_cast<double>(branch.ty) * trans_res;
  matrix(2, 3) = static_cast<double>(branch.tz) * trans_res;

  const double roll = static_cast<double>(branch.roll) * angular_info.rpy_res.x() + angular_info.min_rpy.x();
  const double pitch = static_cast<double>(branch.pitch) * angular_info.rpy_res.y() + angular_info.min_rpy.y();
  const double yaw = static_cast<double>(branch.yaw) * angular_info.rpy_res.z() + angular_info.min_rpy.z();
  const Eigen::AngleAxisd roll_axis(roll, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch_axis(pitch, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw_axis(yaw, Eigen::Vector3d::UnitZ());
  matrix.block<3, 3>(0, 0) = (yaw_axis * pitch_axis * roll_axis).toRotationMatrix();
  return matrix;
}

int SimpleBbs3d::score_branch(
  const Branch & branch,
  int level,
  const AngularInfo & angular_info,
  const std::vector<Eigen::Vector3d> & scan_points) const
{
  const auto & voxel_set = voxel_maps_[static_cast<std::size_t>(level)];
  const double res = voxel_resolutions_[static_cast<std::size_t>(level)];
  const double inv_res = 1.0 / res;
  const Eigen::Matrix4d transform = branch_to_matrix(branch, res, angular_info);

  int score = 0;
  for (const auto & point : scan_points) {
    const Eigen::Vector4d transformed = transform * Eigen::Vector4d(point.x(), point.y(), point.z(), 1.0);
    VoxelKey key;
    key.x = static_cast<int>(std::floor(transformed.x() * inv_res));
    key.y = static_cast<int>(std::floor(transformed.y() * inv_res));
    key.z = static_cast<int>(std::floor(transformed.z() * inv_res));
    if (voxel_set.find(key) != voxel_set.end()) {
      ++score;
    }
  }
  return score;
}

void SimpleBbs3d::push_top_candidate(
  std::vector<BbsCandidate> & candidates,
  const BbsCandidate & candidate,
  int top_k) const
{
  candidates.push_back(candidate);
  std::sort(candidates.begin(), candidates.end(), [](const BbsCandidate & lhs, const BbsCandidate & rhs) {
    return lhs.score > rhs.score;
  });
  if (static_cast<int>(candidates.size()) > top_k) {
    candidates.resize(static_cast<std::size_t>(top_k));
  }
}

BbsResult SimpleBbs3d::localize(const std::vector<Eigen::Vector3d> & scan_points) const
{
  BbsResult result;
  if (scan_points.empty() || voxel_maps_.empty()) {
    return result;
  }

  if (official_backend_ && official_backend_->bbs) {
    // 这里直接调用 vendored 官方 3D-BBS CPU 搜索。我们在 vendored 后端里补了 top-K leaf 队列导出，
    // 因此外层可以同时拿到 best pose 和若干高分候选，便于 RViz 检查重复走廊/相似区域的歧义。
    official_backend_->bbs->set_src_points(scan_points);
    official_backend_->bbs->localize();
    result.search_ms = official_backend_->bbs->get_elapsed_time();
    result.timed_out = official_backend_->bbs->has_timed_out();
    result.localized = official_backend_->bbs->has_localized();
    if (result.localized) {
      const auto poses = official_backend_->bbs->get_top_poses();
      const auto scores = official_backend_->bbs->get_top_scores();
      const double inv_scan_points = scan_points.empty() ? 0.0 : 1.0 / static_cast<double>(scan_points.size());

      for (std::size_t i = 0; i < poses.size() && i < scores.size(); ++i) {
        BbsCandidate candidate;
        candidate.pose = poses[i];
        candidate.score = scores[i];
        candidate.score_ratio = static_cast<double>(scores[i]) * inv_scan_points;
        result.candidates.push_back(candidate);
      }

      // 防御性兜底：如果第三方后端报告 localized 但 top-K 容器为空，仍保留 best pose，避免上层崩溃。
      if (result.candidates.empty()) {
        BbsCandidate candidate;
        candidate.pose = official_backend_->bbs->get_global_pose();
        candidate.score = official_backend_->bbs->get_best_score();
        candidate.score_ratio = official_backend_->bbs->get_best_score_percentage();
        result.candidates.push_back(candidate);
      }
    }
    return result;
  }

  const auto start = std::chrono::steady_clock::now();
  const auto deadline = start + std::chrono::milliseconds(config_.timeout_msec);
  const bool use_timeout = config_.timeout_msec > 0;
  const int score_threshold =
    static_cast<int>(std::floor(static_cast<double>(scan_points.size()) * config_.score_threshold_percentage));

  const auto angular_infos = calculate_angular_info(scan_points);
  auto initial_branches = create_initial_branches(angular_infos[static_cast<std::size_t>(config_.max_level)]);

  // 初始分支数量可能较大，使用 OpenMP 可以显著降低第一层评分耗时。
#pragma omp parallel for num_threads(config_.num_threads)
  for (int i = 0; i < static_cast<int>(initial_branches.size()); ++i) {
    initial_branches[static_cast<std::size_t>(i)].score = score_branch(
      initial_branches[static_cast<std::size_t>(i)],
      config_.max_level,
      angular_infos[static_cast<std::size_t>(config_.max_level)],
      scan_points);
  }

  std::priority_queue<Branch> queue(initial_branches.begin(), initial_branches.end());
  int best_leaf_score = score_threshold;

  while (!queue.empty()) {
    if (use_timeout && std::chrono::steady_clock::now() > deadline) {
      result.timed_out = true;
      break;
    }

    Branch branch = queue.top();
    queue.pop();

    // 如果分支上界都低于当前最差可保留 leaf，就没有继续展开价值。
    if (branch.score < best_leaf_score) {
      continue;
    }

    if (branch.level == 0) {
      BbsCandidate candidate;
      candidate.pose = branch_to_matrix(branch, voxel_resolutions_[0], angular_infos[0]);
      candidate.score = branch.score;
      candidate.score_ratio = static_cast<double>(branch.score) / static_cast<double>(scan_points.size());
      push_top_candidate(result.candidates, candidate, std::max(1, config_.top_k));
      best_leaf_score = std::max(best_leaf_score, result.candidates.back().score);
      continue;
    }

    const int child_level = branch.level - 1;
    auto children = branch_children(branch, angular_infos[static_cast<std::size_t>(child_level)]);

#pragma omp parallel for num_threads(config_.num_threads)
    for (int i = 0; i < static_cast<int>(children.size()); ++i) {
      children[static_cast<std::size_t>(i)].score = score_branch(
        children[static_cast<std::size_t>(i)],
        child_level,
        angular_infos[static_cast<std::size_t>(child_level)],
        scan_points);
    }

    for (const auto & child : children) {
      if (child.score >= best_leaf_score) {
        queue.push(child);
      }
    }
  }

  const auto end = std::chrono::steady_clock::now();
  result.search_ms = std::chrono::duration<double, std::milli>(end - start).count();
  result.localized = !result.candidates.empty() && !result.timed_out;
  return result;
}

}  // namespace humanoid_global_relocalization
