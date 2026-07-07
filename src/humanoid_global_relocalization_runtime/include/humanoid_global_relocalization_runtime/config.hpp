#pragma once

/*
 * config.hpp
 *
 * 文件作用：
 *   1. 定义全局重定位运行层使用的全部配置结构。
 *   2. 把 YAML 中的参数转换为强类型 C++ 字段，避免后续算法代码到处用字符串索引。
 *   3. 统一保存输入话题、坐标转换、点云预处理、3D-BBS、精配准、恢复门控和运行态输出参数。
 *
 * 设计说明：
 *   - 这些结构只描述运行层核心需要的参数，不直接依赖 ROS 参数服务器。
 *   - 在线节点和离线回归工具复用同一份配置结构，防止同名参数含义不一致。
 *   - 任何新增参数都应先加到这里，再加到 YAML，并在 YAML 中补充中文说明。
 */

#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace humanoid_global_relocalization
{

enum class InputMode
{
  RegisteredWorld,
  Body
};

enum class RefineMethod
{
  None,
  Icp,
  Gicp,
  Ndt
};

struct InputConfig
{
  InputMode mode{InputMode::RegisteredWorld};
  std::string registered_world_topic{"/fast_lio/cloud_registered"};
  std::string body_topic{"/cloud_registered_body"};
  std::string odom_topic{"/odom"};
  std::string single_scan_pcd_path;
  std::vector<std::string> bag_paths;
  int bag_start_frame_skip{0};
  int max_bag_frames{20};
  int bag_frame_stride{10};
  std::vector<int> bag_sample_frame_indices;
  double odom_time_tolerance_sec{0.05};
  std::string map_path;
  std::string map_dir;
  std::string map_file_name;
  std::vector<std::string> map_candidates;
  std::string output_dir;
};

struct FrameConfig
{
  bool convert_raw_body_to_base{true};
  Eigen::Vector3d raw_body_to_base_xyz{0.072, -0.004, 1.215};
  bool force_2d_output{true};
};

struct PreprocessConfig
{
  double map_leaf_size{0.20};
  double scan_leaf_size{0.20};
  double min_scan_range{0.60};
  double max_scan_range{30.0};
  bool enable_map_z_crop{false};
  double map_min_z{-1.0};
  double map_max_z{2.5};
  bool enable_scan_z_crop{false};
  double scan_min_z{0.2};
  double scan_max_z{2.5};
};

struct BbsConfig
{
  double min_level_res{0.50};
  int max_level{5};
  double voxel_expansion_rate{2.0};
  int num_threads{4};
  double score_threshold_percentage{0.35};
  int timeout_msec{0};
  Eigen::Vector3d search_min_xyz{-30.0, -30.0, -1.0};
  Eigen::Vector3d search_max_xyz{30.0, 30.0, 2.0};
  Eigen::Vector3d search_min_rpy{-0.05, -0.05, 0.0};
  Eigen::Vector3d search_max_rpy{0.05, 0.05, 6.283185307};
  int top_k{10};
};

struct Bbs2dConfig
{
  bool enable{false};
  double base_resolution{0.20};
  std::vector<int> pyramid_factors{8, 4, 2, 1};
  double yaw_step_deg{5.0};
  std::vector<double> height_bands{0.2, 0.8, 0.8, 1.5, 1.5, 2.2};
  double distance_tolerance_m{0.45};
  int scan_point_step{1};
  int level_keep_candidates{12000};
  int top_k{80};
  double nms_xy_m{2.0};
  double nms_yaw_deg{30.0};
  int fuse_prefix_3d_candidates{10};
  double duplicate_xy_gate_m{0.30};
  double duplicate_yaw_gate_deg{5.0};
};

struct RefineConfig
{
  RefineMethod method{RefineMethod::Gicp};
  std::vector<RefineMethod> methods_for_sweep{RefineMethod::None, RefineMethod::Icp, RefineMethod::Gicp, RefineMethod::Ndt};
  bool refine_top_k{true};
  int max_refine_candidates{5};
  int max_iterations{40};
  double max_correspondence_distance{1.0};
  double transformation_epsilon{0.001};
  double euclidean_fitness_epsilon{0.001};
  double ndt_resolution{1.0};
  double ndt_step_size{0.1};
  double ndt_outlier_ratio{0.55};

  // 兼容旧 YAML 字段：如果没有 refine_method，仍可用 refine_with_gicp 控制是否启用 GICP。
  bool refine_with_gicp{true};
  int gicp_max_iterations{40};
  double gicp_max_correspondence_distance{1.0};
  double gicp_transformation_epsilon{0.001};
};

struct EvaluationConfig
{
  std::optional<Eigen::Matrix4d> reference_pose;
  bool use_bag_reference_pose{true};
  std::string reference_pose_topic{"/robot_realpose"};
  double reference_time_tolerance_sec{0.05};
  bool enable_reference_sanity_check{true};
  double reference_max_abs_xy_m{100.0};
  double reference_max_abs_z_m{10.0};
  double success_translation_thresh{0.50};
  double success_yaw_thresh_deg{10.0};
  bool save_aligned_cloud{true};
  std::string metrics_csv_name{"global_relocalization_metrics.csv"};
  std::string candidates_csv_name{"global_relocalization_candidates.csv"};
  std::string summary_csv_name{"global_relocalization_summary.csv"};
};

struct TemporalConsistencyConfig
{
  bool enable{true};
  std::string scenario_name{"arbitrary_start_no_prior"};
  std::string csv_name{"global_relocalization_temporal_consistency.csv"};
  std::string decision_csv_name{"global_relocalization_temporal_decisions.csv"};
  bool enable_trajectory_likelihood{true};
  std::string trajectory_likelihood_csv_name{"global_relocalization_trajectory_likelihood.csv"};
  int window_before{4};
  int window_after{0};
  double xy_gate_m{1.0};
  double yaw_gate_deg{12.0};
  int online_min_support_frames{2};
  double online_max_refine_fitness{0.12};
  int online_max_history_frames{10};
  bool single_frame_high_confidence_fallback_enable{true};
  double single_frame_high_confidence_max_fitness{0.02};
  int trajectory_max_candidates{30};
  std::vector<int> trajectory_center_frame_indices;
  double trajectory_voxel_size{0.35};
  int trajectory_neighbor_radius{1};
  double trajectory_min_overlap_ratio{0.18};
  double trajectory_min_average_overlap{0.95};
  double trajectory_min_margin{0.03};
  bool trajectory_single_agreement_fallback_enable{true};
  double trajectory_single_agreement_max_fitness{0.04};
  double trajectory_single_agreement_max_xy_m{1.0};
  double trajectory_single_agreement_max_yaw_deg{3.0};
  double trajectory_single_agreement_min_overlap{0.80};
  double trajectory_single_agreement_min_margin{0.005};
  bool trajectory_refine_enable{false};
  int trajectory_refine_top_n{5};
  double trajectory_refine_max_fitness{0.12};
  double trajectory_refine_min_fitness_margin{0.005};
};

struct ScanContextConfig
{
  bool enable{false};
  std::string database_path;
  std::string save_database_path;
  int keyframe_stride{80};
  int top_k{20};
  int exclude_index_radius{120};
  int rings{20};
  int sectors{60};
  double max_radius{20.0};
  double min_range{0.80};
  double min_z{0.2};
  double max_z{2.5};
  bool use_shift_yaw{true};
  std::vector<double> yaw_offsets_deg{-10.0, 0.0, 10.0};
  double duplicate_xy_gate_m{0.30};
  double duplicate_yaw_gate_deg{5.0};
};

struct EvaluationScenario
{
  std::string name{"arbitrary_start_no_prior"};
  Eigen::Vector3d prior_xyz_offset{0.0, 0.0, 0.0};
  Eigen::Vector3d prior_rpy_offset{0.0, 0.0, 0.0};
};

struct RuntimeOutputConfig
{
  bool enable_relocalization{false};
  double relocalization_min_period_sec{1.0};
  std::string candidate_pose_array_topic{"/global_relocalization/candidates"};
  std::string candidate_marker_topic{"/global_relocalization/candidate_markers"};
  std::string aligned_cloud_topic{"/global_relocalization/aligned_scan"};
  std::string recovery_pose_topic{"/global_relocalization/recovery_pose"};
  std::string recovery_map_odom_topic{"/global_relocalization/recovery_map_to_odom"};
  std::string recovery_status_topic{"/global_relocalization/recovery_status"};
};

struct RuntimeConfig
{
  InputConfig input;
  FrameConfig frames;
  PreprocessConfig preprocess;
  BbsConfig bbs;
  Bbs2dConfig bbs2d;
  RefineConfig refine;
  EvaluationConfig evaluation;
  TemporalConsistencyConfig temporal;
  ScanContextConfig scan_context;
  std::vector<EvaluationScenario> scenarios;
  RuntimeOutputConfig output;
};

RuntimeConfig load_config_file(const std::string & path);
std::string to_string(InputMode mode);
std::string to_string(RefineMethod method);

}  // namespace humanoid_global_relocalization
