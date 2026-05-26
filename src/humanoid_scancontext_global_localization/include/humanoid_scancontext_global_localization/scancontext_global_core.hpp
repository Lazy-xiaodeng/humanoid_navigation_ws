#ifndef HUMANOID_SCANCONTEXT_GLOBAL_LOCALIZATION_SCANCONTEXT_GLOBAL_CORE_HPP_
#define HUMANOID_SCANCONTEXT_GLOBAL_LOCALIZATION_SCANCONTEXT_GLOBAL_CORE_HPP_

#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace humanoid_scancontext_global_localization {

class ScanContextGlobal {
public:
  struct Config {
    int num_sectors{90};
    int num_rings{24};
    double max_range{60.0};
    double min_range{0.8};
    double lateral_search_ratio{0.1};
    int horizontal_axis_1{0};
    int horizontal_axis_2{2};
    int vertical_axis{1};
    double vertical_sign{-1.0};
  };

  struct KeyFrame {
    int id{-1};
    Eigen::Matrix4f pose{Eigen::Matrix4f::Identity()};
    Eigen::MatrixXf descriptor;
    Eigen::VectorXf ring_key;
    Eigen::VectorXf sector_key;
  };

  struct Candidate {
    int keyframe_index{-1};
    int keyframe_id{-1};
    float distance{std::numeric_limits<float>::max()};
    int yaw_offset{0};
    float yaw_rad{0.0f};
    Eigen::Matrix4f initial_pose{Eigen::Matrix4f::Identity()};
  };

  ScanContextGlobal();
  explicit ScanContextGlobal(Config config);

  Eigen::MatrixXf computeDescriptor(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud) const;
  Eigen::VectorXf computeRingKey(const Eigen::MatrixXf& descriptor) const;
  Eigen::VectorXf computeSectorKey(const Eigen::MatrixXf& descriptor) const;
  std::pair<float, int> distanceAndYaw(const Eigen::MatrixXf& query, const Eigen::MatrixXf& reference) const;

  bool loadDatabase(const std::string& path);
  bool saveDatabase(const std::string& path) const;
  void addKeyFrame(const KeyFrame& keyframe);
  std::vector<Candidate> search(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud,
    int top_k,
    int ring_candidates,
    float distance_threshold) const;

  bool empty() const { return keyframes_.empty(); }
  size_t size() const { return keyframes_.size(); }
  const Config& config() const { return config_; }
  const KeyFrame& keyframe(size_t index) const { return keyframes_.at(index); }

private:
  Eigen::MatrixXf circularShiftColumns(const Eigen::MatrixXf& matrix, int shift) const;
  float columnCosineDistance(const Eigen::MatrixXf& query, const Eigen::MatrixXf& reference) const;
  Eigen::Matrix4f yawCorrection(float yaw_rad) const;

  Config config_;
  std::vector<KeyFrame> keyframes_;
};

}  // namespace humanoid_scancontext_global_localization

#endif  // HUMANOID_SCANCONTEXT_GLOBAL_LOCALIZATION_SCANCONTEXT_GLOBAL_CORE_HPP_
