/*
 * config_loader.cpp
 *
 * 文件作用：
 *   1. 从 YAML 文件读取 humanoid_global_relocalization_runtime 的运行参数。
 *   2. 将 YAML 字符串、数组、布尔值转换为 config.hpp 中的强类型结构。
 *   3. 对容易写错的输入模式、向量长度等做基本校验，尽早暴露配置问题。
 */

#include "humanoid_global_relocalization_runtime/config.hpp"

#include <cmath>
#include <filesystem>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace humanoid_global_relocalization
{
namespace
{

YAML::Node parameter_root(const YAML::Node & root)
{
  // ROS 2 参数 YAML 通常是 node_name.ros__parameters 两层结构。运行态主配置使用
  // global_relocalization_node；评估配置可能使用 global_relocalization_eval，因此这里兼容两者。
  if (root["global_relocalization_node"] && root["global_relocalization_node"]["ros__parameters"]) {
    return root["global_relocalization_node"]["ros__parameters"];
  }
  if (root["global_relocalization_eval"] && root["global_relocalization_eval"]["ros__parameters"]) {
    return root["global_relocalization_eval"]["ros__parameters"];
  }
  if (root["ros__parameters"]) {
    return root["ros__parameters"];
  }
  return root;
}

Eigen::Vector3d read_vector3(const YAML::Node & node, const std::string & key, const Eigen::Vector3d & default_value)
{
  if (!node[key]) {
    return default_value;
  }
  const auto values = node[key].as<std::vector<double>>();
  if (values.size() != 3) {
    throw std::runtime_error("parameter " + key + " must contain exactly 3 values");
  }
  return Eigen::Vector3d(values[0], values[1], values[2]);
}

template <typename T>
T read_value(const YAML::Node & node, const std::string & key, const T & default_value)
{
  if (!node[key]) {
    return default_value;
  }
  return node[key].as<T>();
}

Eigen::Matrix4d rpy_to_matrix(const Eigen::Vector3d & xyz, const Eigen::Vector3d & rpy)
{
  // 参考位姿采用常见 xyz + roll/pitch/yaw 表达。旋转顺序与 ROS 中 yaw-pitch-roll 组合一致：
  // 先绕 X roll，再绕 Y pitch，最后绕 Z yaw。
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  const Eigen::AngleAxisd roll(rpy.x(), Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch(rpy.y(), Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw(rpy.z(), Eigen::Vector3d::UnitZ());
  matrix.block<3, 3>(0, 0) = (yaw * pitch * roll).toRotationMatrix();
  matrix.block<3, 1>(0, 3) = xyz;
  return matrix;
}

EvaluationScenario read_scenario(const YAML::Node & node, std::size_t index)
{
  // 场景配置用于评估不同恢复入口状态。3D-BBS 全局搜索不使用这个先验，
  // 因此这里不改变算法输入，只把偏差写进指标供统计筛选。
  EvaluationScenario scenario;
  scenario.name = read_value<std::string>(
    node, "name", "scenario_" + std::to_string(index));

  const auto offset = node["offset_xyzrpy"] ?
    node["offset_xyzrpy"].as<std::vector<double>>() :
    std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  if (offset.size() != 6) {
    throw std::runtime_error("simulated_relocalization_cases offset_xyzrpy must contain exactly 6 values");
  }

  scenario.prior_xyz_offset = Eigen::Vector3d(offset[0], offset[1], offset[2]);
  scenario.prior_rpy_offset = Eigen::Vector3d(offset[3], offset[4], offset[5]);
  return scenario;
}

RefineMethod parse_refine_method(const std::string & value)
{
  if (value == "none") {
    return RefineMethod::None;
  }
  if (value == "icp") {
    return RefineMethod::Icp;
  }
  if (value == "gicp") {
    return RefineMethod::Gicp;
  }
  if (value == "ndt") {
    return RefineMethod::Ndt;
  }
  throw std::runtime_error("refine_method must be none/icp/gicp/ndt, got: " + value);
}

std::vector<RefineMethod> read_refine_methods(const YAML::Node & node, const std::vector<RefineMethod> & defaults)
{
  if (!node["refine_methods_for_sweep"]) {
    return defaults;
  }
  const auto values = node["refine_methods_for_sweep"].as<std::vector<std::string>>();
  std::vector<RefineMethod> methods;
  methods.reserve(values.size());
  for (const auto & value : values) {
    methods.push_back(parse_refine_method(value));
  }
  return methods.empty() ? defaults : methods;
}

std::string compose_map_path(
  const std::string & map_path,
  const std::string & map_dir,
  const std::string & map_file_name)
{
  if (!map_path.empty()) {
    return map_path;
  }
  if (!map_dir.empty() && !map_file_name.empty()) {
    return (std::filesystem::path(map_dir) / map_file_name).string();
  }
  return map_path;
}

std::string resolve_map_candidate(const std::string & candidate, const std::string & map_dir)
{
  // map_candidates 允许写绝对路径，也允许只写文件名。只写文件名时，自动拼接 map_dir，
  // 这样同一批地图放在一个目录下做 sweep 会更清爽。
  if (candidate.empty()) {
    return candidate;
  }
  const std::filesystem::path path(candidate);
  if (path.is_absolute() || map_dir.empty()) {
    return candidate;
  }
  return (std::filesystem::path(map_dir) / path).string();
}

}  // namespace

std::string to_string(InputMode mode)
{
  return mode == InputMode::RegisteredWorld ? "registered_world" : "body";
}

std::string to_string(RefineMethod method)
{
  switch (method) {
    case RefineMethod::None:
      return "none";
    case RefineMethod::Icp:
      return "icp";
    case RefineMethod::Gicp:
      return "gicp";
    case RefineMethod::Ndt:
      return "ndt";
  }
  return "unknown";
}

RuntimeConfig load_config_file(const std::string & path)
{
  const YAML::Node root = YAML::LoadFile(path);
  const YAML::Node p = parameter_root(root);

  RuntimeConfig config;

  // 输入模式决定 adapter 如何解释点云。这里严格校验字符串，避免拼写错误导致误用默认模式。
  const std::string mode = read_value<std::string>(p, "input_mode", "registered_world");
  if (mode == "registered_world") {
    config.input.mode = InputMode::RegisteredWorld;
  } else if (mode == "body") {
    config.input.mode = InputMode::Body;
  } else {
    throw std::runtime_error("input_mode must be registered_world or body, got: " + mode);
  }

  config.input.registered_world_topic =
    read_value<std::string>(p, "registered_world_topic", config.input.registered_world_topic);
  config.input.body_topic = read_value<std::string>(p, "body_topic", config.input.body_topic);
  config.input.odom_topic = read_value<std::string>(p, "odom_topic", config.input.odom_topic);
  config.input.single_scan_pcd_path =
    read_value<std::string>(p, "single_scan_pcd_path", config.input.single_scan_pcd_path);
  if (p["bag_paths"]) {
    config.input.bag_paths = p["bag_paths"].as<std::vector<std::string>>();
  }
  config.input.bag_start_frame_skip =
    read_value<int>(p, "bag_start_frame_skip", config.input.bag_start_frame_skip);
  config.input.max_bag_frames = read_value<int>(p, "max_bag_frames", config.input.max_bag_frames);
  config.input.bag_frame_stride = read_value<int>(p, "bag_frame_stride", config.input.bag_frame_stride);
  if (p["bag_sample_frame_indices"]) {
    config.input.bag_sample_frame_indices = p["bag_sample_frame_indices"].as<std::vector<int>>();
  }
  config.input.odom_time_tolerance_sec =
    read_value<double>(p, "odom_time_tolerance_sec", config.input.odom_time_tolerance_sec);
  config.input.map_path = read_value<std::string>(p, "map_path", config.input.map_path);
  config.input.map_dir = read_value<std::string>(p, "map_dir", config.input.map_dir);
  config.input.map_file_name = read_value<std::string>(p, "map_file_name", config.input.map_file_name);
  if (p["map_candidates"]) {
    config.input.map_candidates = p["map_candidates"].as<std::vector<std::string>>();
  }
  for (auto & candidate : config.input.map_candidates) {
    candidate = resolve_map_candidate(candidate, config.input.map_dir);
  }
  config.input.map_path = compose_map_path(
    config.input.map_path, config.input.map_dir, config.input.map_file_name);
  if (config.input.map_candidates.empty() && !config.input.map_path.empty()) {
    config.input.map_candidates.push_back(config.input.map_path);
  }
  config.input.output_dir = read_value<std::string>(p, "output_dir", config.input.output_dir);

  config.frames.convert_raw_body_to_base =
    read_value<bool>(p, "convert_raw_body_to_base", config.frames.convert_raw_body_to_base);
  config.frames.raw_body_to_base_xyz =
    read_vector3(p, "raw_body_to_base_xyz", config.frames.raw_body_to_base_xyz);
  config.frames.force_2d_output = read_value<bool>(p, "force_2d_output", config.frames.force_2d_output);

  config.preprocess.map_leaf_size = read_value<double>(p, "map_leaf_size", config.preprocess.map_leaf_size);
  config.preprocess.scan_leaf_size = read_value<double>(p, "scan_leaf_size", config.preprocess.scan_leaf_size);
  config.preprocess.min_scan_range = read_value<double>(p, "min_scan_range", config.preprocess.min_scan_range);
  config.preprocess.max_scan_range = read_value<double>(p, "max_scan_range", config.preprocess.max_scan_range);
  config.preprocess.enable_map_z_crop =
    read_value<bool>(p, "enable_map_z_crop", config.preprocess.enable_map_z_crop);
  config.preprocess.map_min_z = read_value<double>(p, "map_min_z", config.preprocess.map_min_z);
  config.preprocess.map_max_z = read_value<double>(p, "map_max_z", config.preprocess.map_max_z);
  config.preprocess.enable_scan_z_crop =
    read_value<bool>(p, "enable_scan_z_crop", config.preprocess.enable_scan_z_crop);
  config.preprocess.scan_min_z = read_value<double>(p, "scan_min_z", config.preprocess.scan_min_z);
  config.preprocess.scan_max_z = read_value<double>(p, "scan_max_z", config.preprocess.scan_max_z);

  config.bbs.min_level_res = read_value<double>(p, "bbs_min_level_res", config.bbs.min_level_res);
  config.bbs.max_level = read_value<int>(p, "bbs_max_level", config.bbs.max_level);
  config.bbs.voxel_expansion_rate =
    read_value<double>(p, "bbs_voxel_expansion_rate", config.bbs.voxel_expansion_rate);
  config.bbs.num_threads = read_value<int>(p, "bbs_num_threads", config.bbs.num_threads);
  config.bbs.score_threshold_percentage =
    read_value<double>(p, "bbs_score_threshold_percentage", config.bbs.score_threshold_percentage);
  config.bbs.timeout_msec = read_value<int>(p, "bbs_timeout_msec", config.bbs.timeout_msec);
  config.bbs.search_min_xyz = read_vector3(p, "search_min_xyz", config.bbs.search_min_xyz);
  config.bbs.search_max_xyz = read_vector3(p, "search_max_xyz", config.bbs.search_max_xyz);
  config.bbs.search_min_rpy = read_vector3(p, "search_min_rpy", config.bbs.search_min_rpy);
  config.bbs.search_max_rpy = read_vector3(p, "search_max_rpy", config.bbs.search_max_rpy);
  config.bbs.top_k = read_value<int>(p, "top_k", config.bbs.top_k);

  config.bbs2d.enable = read_value<bool>(p, "enable_bbs2d_recall", config.bbs2d.enable);
  config.bbs2d.base_resolution =
    read_value<double>(p, "bbs2d_base_resolution", config.bbs2d.base_resolution);
  if (p["bbs2d_pyramid_factors"]) {
    config.bbs2d.pyramid_factors = p["bbs2d_pyramid_factors"].as<std::vector<int>>();
  }
  config.bbs2d.yaw_step_deg =
    read_value<double>(p, "bbs2d_yaw_step_deg", config.bbs2d.yaw_step_deg);
  if (p["bbs2d_height_bands"]) {
    config.bbs2d.height_bands = p["bbs2d_height_bands"].as<std::vector<double>>();
  }
  config.bbs2d.distance_tolerance_m =
    read_value<double>(p, "bbs2d_distance_tolerance_m", config.bbs2d.distance_tolerance_m);
  config.bbs2d.scan_point_step =
    read_value<int>(p, "bbs2d_scan_point_step", config.bbs2d.scan_point_step);
  config.bbs2d.level_keep_candidates =
    read_value<int>(p, "bbs2d_level_keep_candidates", config.bbs2d.level_keep_candidates);
  config.bbs2d.top_k = read_value<int>(p, "bbs2d_top_k", config.bbs2d.top_k);
  config.bbs2d.nms_xy_m = read_value<double>(p, "bbs2d_nms_xy_m", config.bbs2d.nms_xy_m);
  config.bbs2d.nms_yaw_deg =
    read_value<double>(p, "bbs2d_nms_yaw_deg", config.bbs2d.nms_yaw_deg);
  config.bbs2d.fuse_prefix_3d_candidates =
    read_value<int>(p, "bbs2d_fuse_prefix_3d_candidates", config.bbs2d.fuse_prefix_3d_candidates);
  config.bbs2d.duplicate_xy_gate_m =
    read_value<double>(p, "bbs2d_duplicate_xy_gate_m", config.bbs2d.duplicate_xy_gate_m);
  config.bbs2d.duplicate_yaw_gate_deg =
    read_value<double>(p, "bbs2d_duplicate_yaw_gate_deg", config.bbs2d.duplicate_yaw_gate_deg);

  config.refine.refine_with_gicp = read_value<bool>(p, "refine_with_gicp", config.refine.refine_with_gicp);
  const std::string default_refine_method = config.refine.refine_with_gicp ? "gicp" : "none";
  config.refine.method = parse_refine_method(
    read_value<std::string>(p, "refine_method", default_refine_method));
  config.refine.methods_for_sweep = read_refine_methods(p, config.refine.methods_for_sweep);
  config.refine.refine_top_k = read_value<bool>(p, "refine_top_k", config.refine.refine_top_k);
  config.refine.max_refine_candidates =
    read_value<int>(p, "max_refine_candidates", config.refine.max_refine_candidates);
  config.refine.max_iterations = read_value<int>(p, "refine_max_iterations", config.refine.max_iterations);
  config.refine.max_correspondence_distance =
    read_value<double>(p, "refine_max_correspondence_distance", config.refine.max_correspondence_distance);
  config.refine.transformation_epsilon =
    read_value<double>(p, "refine_transformation_epsilon", config.refine.transformation_epsilon);
  config.refine.euclidean_fitness_epsilon =
    read_value<double>(p, "refine_euclidean_fitness_epsilon", config.refine.euclidean_fitness_epsilon);
  config.refine.ndt_resolution = read_value<double>(p, "ndt_resolution", config.refine.ndt_resolution);
  config.refine.ndt_step_size = read_value<double>(p, "ndt_step_size", config.refine.ndt_step_size);
  config.refine.ndt_outlier_ratio = read_value<double>(p, "ndt_outlier_ratio", config.refine.ndt_outlier_ratio);
  config.refine.gicp_max_iterations =
    read_value<int>(p, "gicp_max_iterations", config.refine.gicp_max_iterations);
  config.refine.gicp_max_correspondence_distance =
    read_value<double>(p, "gicp_max_correspondence_distance", config.refine.gicp_max_correspondence_distance);
  config.refine.gicp_transformation_epsilon =
    read_value<double>(p, "gicp_transformation_epsilon", config.refine.gicp_transformation_epsilon);
  if (!p["refine_max_iterations"] && p["gicp_max_iterations"]) {
    config.refine.max_iterations = config.refine.gicp_max_iterations;
  }
  if (!p["refine_max_correspondence_distance"] && p["gicp_max_correspondence_distance"]) {
    config.refine.max_correspondence_distance = config.refine.gicp_max_correspondence_distance;
  }
  if (!p["refine_transformation_epsilon"] && p["gicp_transformation_epsilon"]) {
    config.refine.transformation_epsilon = config.refine.gicp_transformation_epsilon;
  }

  if (p["reference_pose_xyzrpy"]) {
    const auto values = p["reference_pose_xyzrpy"].as<std::vector<double>>();
    if (!values.empty()) {
      if (values.size() != 6) {
        throw std::runtime_error("reference_pose_xyzrpy must be empty or contain 6 values");
      }
      config.evaluation.reference_pose = rpy_to_matrix(
        Eigen::Vector3d(values[0], values[1], values[2]),
        Eigen::Vector3d(values[3], values[4], values[5]));
    }
  }
  config.evaluation.use_bag_reference_pose =
    read_value<bool>(p, "use_bag_reference_pose", config.evaluation.use_bag_reference_pose);
  config.evaluation.reference_pose_topic =
    read_value<std::string>(p, "reference_pose_topic", config.evaluation.reference_pose_topic);
  config.evaluation.reference_time_tolerance_sec =
    read_value<double>(p, "reference_time_tolerance_sec", config.evaluation.reference_time_tolerance_sec);
  config.evaluation.enable_reference_sanity_check =
    read_value<bool>(p, "enable_reference_sanity_check", config.evaluation.enable_reference_sanity_check);
  config.evaluation.reference_max_abs_xy_m =
    read_value<double>(p, "reference_max_abs_xy_m", config.evaluation.reference_max_abs_xy_m);
  config.evaluation.reference_max_abs_z_m =
    read_value<double>(p, "reference_max_abs_z_m", config.evaluation.reference_max_abs_z_m);
  config.evaluation.success_translation_thresh =
    read_value<double>(p, "success_translation_thresh", config.evaluation.success_translation_thresh);
  config.evaluation.success_yaw_thresh_deg =
    read_value<double>(p, "success_yaw_thresh_deg", config.evaluation.success_yaw_thresh_deg);
  config.evaluation.save_aligned_cloud =
    read_value<bool>(p, "save_aligned_cloud", config.evaluation.save_aligned_cloud);
  config.evaluation.metrics_csv_name =
    read_value<std::string>(p, "metrics_csv_name", config.evaluation.metrics_csv_name);
  config.evaluation.candidates_csv_name =
    read_value<std::string>(p, "candidates_csv_name", config.evaluation.candidates_csv_name);
  config.evaluation.summary_csv_name =
    read_value<std::string>(p, "summary_csv_name", config.evaluation.summary_csv_name);

  config.temporal.enable =
    read_value<bool>(p, "enable_temporal_consistency", config.temporal.enable);
  config.temporal.scenario_name =
    read_value<std::string>(p, "temporal_consistency_scenario_name", config.temporal.scenario_name);
  config.temporal.csv_name =
    read_value<std::string>(p, "temporal_consistency_csv_name", config.temporal.csv_name);
  config.temporal.decision_csv_name =
    read_value<std::string>(p, "temporal_decision_csv_name", config.temporal.decision_csv_name);
  config.temporal.enable_trajectory_likelihood =
    read_value<bool>(p, "enable_trajectory_likelihood", config.temporal.enable_trajectory_likelihood);
  config.temporal.trajectory_likelihood_csv_name =
    read_value<std::string>(p, "trajectory_likelihood_csv_name", config.temporal.trajectory_likelihood_csv_name);
  config.temporal.window_before =
    read_value<int>(p, "temporal_consistency_window_before", config.temporal.window_before);
  config.temporal.window_after =
    read_value<int>(p, "temporal_consistency_window_after", config.temporal.window_after);
  config.temporal.reset_gap_sec =
    read_value<double>(p, "temporal_consistency_reset_gap_sec", config.temporal.reset_gap_sec);
  config.temporal.xy_gate_m =
    read_value<double>(p, "temporal_consistency_xy_gate_m", config.temporal.xy_gate_m);
  config.temporal.yaw_gate_deg =
    read_value<double>(p, "temporal_consistency_yaw_gate_deg", config.temporal.yaw_gate_deg);
  config.temporal.online_min_support_frames =
    read_value<int>(p, "temporal_consistency_online_min_support_frames", config.temporal.online_min_support_frames);
  config.temporal.online_max_refine_fitness =
    read_value<double>(p, "temporal_consistency_online_max_refine_fitness", config.temporal.online_max_refine_fitness);
  config.temporal.solid_primary_relaxed_gate_enable = read_value<bool>(
    p, "solid_primary_relaxed_gate_enable", config.temporal.solid_primary_relaxed_gate_enable);
  config.temporal.solid_primary_max_refine_fitness = read_value<double>(
    p, "solid_primary_max_refine_fitness", config.temporal.solid_primary_max_refine_fitness);
  config.temporal.online_max_history_frames =
    read_value<int>(p, "temporal_consistency_online_max_history_frames", config.temporal.online_max_history_frames);
  config.temporal.single_frame_high_confidence_fallback_enable =
    read_value<bool>(
    p,
    "single_frame_high_confidence_fallback_enable",
    config.temporal.single_frame_high_confidence_fallback_enable);
  config.temporal.single_frame_high_confidence_max_fitness =
    read_value<double>(
    p,
    "single_frame_high_confidence_max_fitness",
    config.temporal.single_frame_high_confidence_max_fitness);
  config.temporal.trajectory_max_candidates =
    read_value<int>(p, "trajectory_likelihood_max_candidates", config.temporal.trajectory_max_candidates);
  if (p["trajectory_likelihood_center_frame_indices"]) {
    config.temporal.trajectory_center_frame_indices =
      p["trajectory_likelihood_center_frame_indices"].as<std::vector<int>>();
  }
  config.temporal.trajectory_voxel_size =
    read_value<double>(p, "trajectory_likelihood_voxel_size", config.temporal.trajectory_voxel_size);
  config.temporal.trajectory_neighbor_radius =
    read_value<int>(p, "trajectory_likelihood_neighbor_radius", config.temporal.trajectory_neighbor_radius);
  config.temporal.trajectory_min_overlap_ratio =
    read_value<double>(p, "trajectory_likelihood_min_overlap_ratio", config.temporal.trajectory_min_overlap_ratio);
  config.temporal.trajectory_min_average_overlap =
    read_value<double>(
    p,
    "trajectory_likelihood_min_average_overlap",
    config.temporal.trajectory_min_average_overlap);
  config.temporal.trajectory_min_margin =
    read_value<double>(p, "trajectory_likelihood_min_margin", config.temporal.trajectory_min_margin);
  config.temporal.trajectory_single_agreement_fallback_enable =
    read_value<bool>(
    p,
    "trajectory_single_agreement_fallback_enable",
    config.temporal.trajectory_single_agreement_fallback_enable);
  config.temporal.trajectory_single_agreement_max_fitness =
    read_value<double>(
    p,
    "trajectory_single_agreement_max_fitness",
    config.temporal.trajectory_single_agreement_max_fitness);
  config.temporal.trajectory_single_agreement_max_xy_m =
    read_value<double>(
    p,
    "trajectory_single_agreement_max_xy_m",
    config.temporal.trajectory_single_agreement_max_xy_m);
  config.temporal.trajectory_single_agreement_max_yaw_deg =
    read_value<double>(
    p,
    "trajectory_single_agreement_max_yaw_deg",
    config.temporal.trajectory_single_agreement_max_yaw_deg);
  config.temporal.trajectory_single_agreement_min_overlap =
    read_value<double>(
    p,
    "trajectory_single_agreement_min_overlap",
    config.temporal.trajectory_single_agreement_min_overlap);
  config.temporal.trajectory_single_agreement_min_margin =
    read_value<double>(
    p,
    "trajectory_single_agreement_min_margin",
    config.temporal.trajectory_single_agreement_min_margin);
  config.temporal.trajectory_refine_enable =
    read_value<bool>(p, "trajectory_refine_enable", config.temporal.trajectory_refine_enable);
  config.temporal.trajectory_refine_top_n =
    read_value<int>(p, "trajectory_refine_top_n", config.temporal.trajectory_refine_top_n);
  config.temporal.trajectory_refine_max_fitness =
    read_value<double>(p, "trajectory_refine_max_fitness", config.temporal.trajectory_refine_max_fitness);
  config.temporal.trajectory_refine_min_fitness_margin =
    read_value<double>(p, "trajectory_refine_min_fitness_margin", config.temporal.trajectory_refine_min_fitness_margin);
  config.temporal.trajectory_refine_cluster_enable =
    read_value<bool>(p, "trajectory_refine_cluster_enable", config.temporal.trajectory_refine_cluster_enable);
  config.temporal.trajectory_refine_cluster_xy_m =
    read_value<double>(p, "trajectory_refine_cluster_xy_m", config.temporal.trajectory_refine_cluster_xy_m);
  config.temporal.trajectory_refine_cluster_yaw_deg =
    read_value<double>(p, "trajectory_refine_cluster_yaw_deg", config.temporal.trajectory_refine_cluster_yaw_deg);
  config.temporal.trajectory_refine_min_cluster_size =
    read_value<int>(p, "trajectory_refine_min_cluster_size", config.temporal.trajectory_refine_min_cluster_size);

  config.scan_context.enable =
    read_value<bool>(p, "enable_scan_context_recall", config.scan_context.enable);
  config.scan_context.database_path =
    read_value<std::string>(p, "scan_context_database_path", config.scan_context.database_path);
  config.scan_context.save_database_path =
    read_value<std::string>(p, "scan_context_save_database_path", config.scan_context.save_database_path);
  config.scan_context.keyframe_stride =
    read_value<int>(p, "scan_context_keyframe_stride", config.scan_context.keyframe_stride);
  config.scan_context.top_k =
    read_value<int>(p, "scan_context_top_k", config.scan_context.top_k);
  config.scan_context.exclude_index_radius =
    read_value<int>(p, "scan_context_exclude_index_radius", config.scan_context.exclude_index_radius);
  config.scan_context.rings =
    read_value<int>(p, "scan_context_rings", config.scan_context.rings);
  config.scan_context.sectors =
    read_value<int>(p, "scan_context_sectors", config.scan_context.sectors);
  config.scan_context.max_radius =
    read_value<double>(p, "scan_context_max_radius", config.scan_context.max_radius);
  config.scan_context.min_range =
    read_value<double>(p, "scan_context_min_range", config.scan_context.min_range);
  config.scan_context.min_z =
    read_value<double>(p, "scan_context_min_z", config.scan_context.min_z);
  config.scan_context.max_z =
    read_value<double>(p, "scan_context_max_z", config.scan_context.max_z);
  config.scan_context.use_shift_yaw =
    read_value<bool>(p, "scan_context_use_shift_yaw", config.scan_context.use_shift_yaw);
  if (p["scan_context_yaw_offsets_deg"]) {
    config.scan_context.yaw_offsets_deg = p["scan_context_yaw_offsets_deg"].as<std::vector<double>>();
  }
  config.scan_context.duplicate_xy_gate_m =
    read_value<double>(p, "scan_context_duplicate_xy_gate_m", config.scan_context.duplicate_xy_gate_m);
  config.scan_context.duplicate_yaw_gate_deg =
    read_value<double>(p, "scan_context_duplicate_yaw_gate_deg", config.scan_context.duplicate_yaw_gate_deg);

  config.solid.enable = read_value<bool>(p, "enable_solid_recall", config.solid.enable);
  config.solid.database_path =
    read_value<std::string>(p, "solid_database_path", config.solid.database_path);
  config.solid.save_database_path =
    read_value<std::string>(p, "solid_save_database_path", config.solid.save_database_path);
  config.solid.keyframe_stride =
    read_value<int>(p, "solid_keyframe_stride", config.solid.keyframe_stride);
  config.solid.top_k = read_value<int>(p, "solid_top_k", config.solid.top_k);
  config.solid.top_k_per_source =
    read_value<int>(p, "solid_top_k_per_source", config.solid.top_k_per_source);
  config.solid.exclude_index_radius =
    read_value<int>(p, "solid_exclude_index_radius", config.solid.exclude_index_radius);
  config.solid.ranges = read_value<int>(p, "solid_ranges", config.solid.ranges);
  config.solid.angles = read_value<int>(p, "solid_angles", config.solid.angles);
  config.solid.heights = read_value<int>(p, "solid_heights", config.solid.heights);
  config.solid.min_range = read_value<double>(p, "solid_min_range", config.solid.min_range);
  config.solid.max_range = read_value<double>(p, "solid_max_range", config.solid.max_range);
  config.solid.fov_down_deg =
    read_value<double>(p, "solid_fov_down_deg", config.solid.fov_down_deg);
  config.solid.fov_up_deg = read_value<double>(p, "solid_fov_up_deg", config.solid.fov_up_deg);
  config.solid.sensor_height =
    read_value<double>(p, "solid_sensor_height", config.solid.sensor_height);
  config.solid.voxel_size = read_value<double>(p, "solid_voxel_size", config.solid.voxel_size);
  config.solid.registration_voxel_size =
    read_value<double>(p, "solid_registration_voxel_size", config.solid.registration_voxel_size);
  config.solid.registration_max_correspondence_m = read_value<double>(
    p, "solid_registration_max_correspondence_m", config.solid.registration_max_correspondence_m);
  config.solid.registration_max_iterations = read_value<int>(
    p, "solid_registration_max_iterations", config.solid.registration_max_iterations);
  config.solid.similarity_min =
    read_value<double>(p, "solid_similarity_min", config.solid.similarity_min);
  config.solid.bbs_prefix_candidates =
    read_value<int>(p, "solid_bbs_prefix_candidates", config.solid.bbs_prefix_candidates);
  config.solid.duplicate_xy_gate_m =
    read_value<double>(p, "solid_duplicate_xy_gate_m", config.solid.duplicate_xy_gate_m);
  config.solid.duplicate_yaw_gate_deg =
    read_value<double>(p, "solid_duplicate_yaw_gate_deg", config.solid.duplicate_yaw_gate_deg);

  config.precision_recovery.enable =
    read_value<bool>(p, "enable_precision_recovery_layer", config.precision_recovery.enable);
  config.precision_recovery.trigger_on_default_reject =
    read_value<bool>(
    p,
    "precision_layer_trigger_on_default_reject",
    config.precision_recovery.trigger_on_default_reject);
  config.precision_recovery.trigger_on_weak_accept =
    read_value<bool>(
    p,
    "precision_layer_trigger_on_weak_accept",
    config.precision_recovery.trigger_on_weak_accept);
  config.precision_recovery.min_default_reject_frames =
    read_value<int>(
    p,
    "precision_layer_min_default_reject_frames",
    config.precision_recovery.min_default_reject_frames);
  config.precision_recovery.attempt_frames =
    read_value<int>(p, "precision_layer_attempt_frames", config.precision_recovery.attempt_frames);
  config.precision_recovery.cooldown_sec =
    read_value<double>(p, "precision_layer_cooldown_sec", config.precision_recovery.cooldown_sec);
  config.precision_recovery.scan_leaf_size =
    read_value<double>(p, "precision_layer_scan_leaf_size", config.precision_recovery.scan_leaf_size);
  config.precision_recovery.max_refine_candidates =
    read_value<int>(
    p,
    "precision_layer_max_refine_candidates",
    config.precision_recovery.max_refine_candidates);
  config.precision_recovery.enable_scan_context_recall =
    read_value<bool>(
    p,
    "precision_layer_enable_scan_context_recall",
    config.precision_recovery.enable_scan_context_recall);
  config.precision_recovery.enable_bbs2d_recall =
    read_value<bool>(
    p,
    "precision_layer_enable_bbs2d_recall",
    config.precision_recovery.enable_bbs2d_recall);
  config.precision_recovery.scan_context_database_path =
    read_value<std::string>(
    p,
    "precision_layer_scan_context_database_path",
    config.precision_recovery.scan_context_database_path);
  config.precision_recovery.trigger_risk_score =
    read_value<int>(
    p,
    "precision_layer_trigger_risk_score",
    config.precision_recovery.trigger_risk_score);
  config.precision_recovery.weak_support_frames =
    read_value<int>(
    p,
    "precision_layer_risk_weak_support_frames",
    config.precision_recovery.weak_support_frames);
  config.precision_recovery.bad_support_frames =
    read_value<int>(
    p,
    "precision_layer_risk_bad_support_frames",
    config.precision_recovery.bad_support_frames);
  config.precision_recovery.weak_selected_rank =
    read_value<int>(
    p,
    "precision_layer_risk_weak_selected_rank",
    config.precision_recovery.weak_selected_rank);
  config.precision_recovery.bad_selected_rank =
    read_value<int>(
    p,
    "precision_layer_risk_bad_selected_rank",
    config.precision_recovery.bad_selected_rank);
  config.precision_recovery.weak_refine_fitness =
    read_value<double>(
    p,
    "precision_layer_risk_weak_refine_fitness",
    config.precision_recovery.weak_refine_fitness);
  config.precision_recovery.bad_refine_fitness =
    read_value<double>(
    p,
    "precision_layer_risk_bad_refine_fitness",
    config.precision_recovery.bad_refine_fitness);
  config.precision_recovery.weak_seed_disagreement_xy_m =
    read_value<double>(
    p,
    "precision_layer_risk_weak_seed_disagreement_xy_m",
    config.precision_recovery.weak_seed_disagreement_xy_m);
  config.precision_recovery.bad_seed_disagreement_xy_m =
    read_value<double>(
    p,
    "precision_layer_risk_bad_seed_disagreement_xy_m",
    config.precision_recovery.bad_seed_disagreement_xy_m);
  config.precision_recovery.weak_seed_disagreement_yaw_deg =
    read_value<double>(
    p,
    "precision_layer_risk_weak_seed_disagreement_yaw_deg",
    config.precision_recovery.weak_seed_disagreement_yaw_deg);
  config.precision_recovery.bad_seed_disagreement_yaw_deg =
    read_value<double>(
    p,
    "precision_layer_risk_bad_seed_disagreement_yaw_deg",
    config.precision_recovery.bad_seed_disagreement_yaw_deg);
  if (config.precision_recovery.scan_context_database_path.empty()) {
    config.precision_recovery.scan_context_database_path = config.scan_context.database_path;
  }

  if (p["simulated_relocalization_cases"]) {
    config.scenarios.clear();
    const auto cases = p["simulated_relocalization_cases"];
    if (!cases.IsSequence()) {
      throw std::runtime_error("simulated_relocalization_cases must be a YAML sequence");
    }
    for (std::size_t i = 0; i < cases.size(); ++i) {
      config.scenarios.push_back(read_scenario(cases[i], i));
    }
  }
  if (config.scenarios.empty()) {
    // 即使配置没有提供场景，也保留一个“任意点启动/无可靠先验”的默认场景，保证指标字段稳定。
    config.scenarios.push_back(EvaluationScenario{});
  }

  config.output.enable_relocalization =
    read_value<bool>(p, "enable_relocalization", config.output.enable_relocalization);
  config.output.relocalization_min_period_sec =
    read_value<double>(p, "relocalization_min_period_sec", config.output.relocalization_min_period_sec);
  config.output.candidate_pose_array_topic =
    read_value<std::string>(p, "candidate_pose_array_topic", config.output.candidate_pose_array_topic);
  config.output.candidate_marker_topic =
    read_value<std::string>(p, "candidate_marker_topic", config.output.candidate_marker_topic);
  config.output.aligned_cloud_topic =
    read_value<std::string>(p, "aligned_cloud_topic", config.output.aligned_cloud_topic);
  config.output.recovery_pose_topic =
    read_value<std::string>(p, "recovery_pose_topic", config.output.recovery_pose_topic);
  config.output.recovery_map_odom_topic =
    read_value<std::string>(p, "recovery_map_odom_topic", config.output.recovery_map_odom_topic);
  config.output.recovery_status_topic =
    read_value<std::string>(p, "recovery_status_topic", config.output.recovery_status_topic);

  return config;
}

}  // namespace humanoid_global_relocalization
