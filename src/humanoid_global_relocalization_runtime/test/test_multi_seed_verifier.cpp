#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "humanoid_global_relocalization_runtime/multi_seed_verifier.hpp"

namespace hgr = humanoid_global_relocalization;

namespace
{

Eigen::Matrix4d pose(double x, double y, double yaw)
{
  Eigen::Matrix4d value = Eigen::Matrix4d::Identity();
  value(0, 0) = std::cos(yaw);
  value(0, 1) = -std::sin(yaw);
  value(1, 0) = std::sin(yaw);
  value(1, 1) = std::cos(yaw);
  value(0, 3) = x;
  value(1, 3) = y;
  return value;
}

hgr::BbsCandidate candidate(const std::string & source, const Eigen::Matrix4d & value)
{
  hgr::BbsCandidate result;
  result.source = source;
  result.pose = value;
  return result;
}

hgr::MultiSeedRecoveryConfig test_config()
{
  hgr::MultiSeedRecoveryConfig config;
  config.enable = true;
  config.worker_threads = 2;
  config.tracking_window_frames = 12;
  config.tracking_min_valid_frames = 9;
  return config;
}

hgr::MultiSeedTrack stable_track(
  int id,
  int cluster,
  int rank,
  double map_odom_x)
{
  hgr::MultiSeedTrack track;
  track.track_id = id;
  track.cluster_id = cluster;
  track.candidate_rank = rank;
  track.source = "bbs3d";
  for (int index = 0; index < 12; ++index) {
    hgr::MultiSeedTrackSample sample;
    sample.valid = true;
    sample.fitness = 0.96;
    sample.rmse_m = 0.06;
    sample.correction_xy_m = 0.03;
    sample.map_to_odom = pose(map_odom_x + 0.002 * (index % 2), 0.02, 0.01);
    sample.pose = sample.map_to_odom;
    track.samples.push_back(sample);
  }
  return track;
}

bool expect(bool condition, const std::string & message)
{
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
  }
  return condition;
}

}  // namespace

int main()
{
  bool passed = true;
  auto config = test_config();

  {
    hgr::MultiSeedInitializationInput input;
    input.map_cloud = hgr::CloudPtr(new hgr::Cloud);
    input.scan_cloud = hgr::CloudPtr(new hgr::Cloud);
    input.refine.method = hgr::RefineMethod::None;
    input.candidates = {
      candidate("bbs3d", pose(5.00, 2.00, 0.10)),
      candidate("bbs3d", pose(5.08, 2.03, 0.12)),
      candidate("solid", pose(-8.0, 3.0, 1.0))};
    const auto initialized = hgr::initialize_multi_seed_tracks(input, config);
    passed &= expect(initialized.tracks.size() == 2U, "same-source candidate pair should create two tracks");
    passed &= expect(initialized.retained_clusters == 1, "singleton source must not create a track cluster");
  }

  {
    std::vector<hgr::MultiSeedTrack> tracks{
      stable_track(1, 1, 2, 0.10),
      stable_track(2, 1, 5, 0.14)};
    const auto output = hgr::evaluate_multi_seed_tracks(tracks, Eigen::Matrix4d::Identity(), config);
    passed &= expect(output.accepted, "two stable agreeing tracks should be accepted");
    passed &= expect(output.qualified_tracks == 2, "accepted result should report two agreeing tracks");
    passed &= expect(output.valid_frames == 12, "rolling window should report twelve valid frames");
  }

  {
    std::vector<hgr::MultiSeedTrack> tracks{stable_track(1, 1, 2, 0.10)};
    const auto output = hgr::evaluate_multi_seed_tracks(tracks, Eigen::Matrix4d::Identity(), config);
    passed &= expect(!output.accepted, "one stable track must not be sufficient");
  }

  {
    std::vector<hgr::MultiSeedTrack> tracks{
      stable_track(1, 1, 2, 0.10),
      stable_track(2, 1, 5, 0.30)};
    const auto output = hgr::evaluate_multi_seed_tracks(tracks, Eigen::Matrix4d::Identity(), config);
    passed &= expect(!output.accepted, "separated tracks in one cluster must be rejected");
  }

  if (passed) {
    std::cout << "multi-seed rolling tracker tests passed\n";
    return 0;
  }
  return 1;
}
