#include "humanoid_scancontext_global_localization/scancontext_global_core.hpp"

#include <chrono>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace humanoid_scancontext_global_localization {
namespace {
geometry_msgs::msg::Pose poseFromMatrix(const Eigen::Matrix4f& transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform(0, 3);
  pose.position.y = transform(1, 3);
  pose.position.z = transform(2, 3);

  Eigen::Matrix3f rotation = transform.block<3, 3>(0, 0);
  Eigen::Quaternionf q(rotation);
  q.normalize();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

double yawFromMatrix(const Eigen::Matrix4f& transform)
{
  const auto pose = poseFromMatrix(transform);
  tf2::Quaternion q;
  tf2::fromMsg(pose.orientation, q);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

Eigen::Matrix4f fastlioToRosMatrix(const Eigen::Matrix4f& transform)
{
  Eigen::Matrix4f converted = Eigen::Matrix4f::Identity();
  Eigen::Matrix3f basis;
  basis << 0.0f, 0.0f, -1.0f,
           1.0f, 0.0f, 0.0f,
           0.0f, -1.0f, 0.0f;
  converted.block<3, 3>(0, 0) =
    basis * transform.block<3, 3>(0, 0) * basis.transpose();
  converted.block<3, 1>(0, 3) = basis * transform.block<3, 1>(0, 3);
  converted(2, 3) = 0.0f;
  return converted;
}

Eigen::Matrix4f odomToMatrix(const nav_msgs::msg::Odometry& msg)
{
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  const auto& p = msg.pose.pose.position;
  const auto& q_msg = msg.pose.pose.orientation;
  Eigen::Quaternionf q(
    static_cast<float>(q_msg.w),
    static_cast<float>(q_msg.x),
    static_cast<float>(q_msg.y),
    static_cast<float>(q_msg.z));
  q.normalize();
  transform.block<3, 3>(0, 0) = q.toRotationMatrix();
  transform(0, 3) = static_cast<float>(p.x);
  transform(1, 3) = static_cast<float>(p.y);
  transform(2, 3) = static_cast<float>(p.z);
  return transform;
}
}  // namespace

class ScanContextGlobalLocalizerNode : public rclcpp::Node {
public:
  explicit ScanContextGlobalLocalizerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("scancontext_global_localizer", options)
  {
    declareParameters();
    loadDatabase();
    loadMapIfRequested();
    setupRos();
    RCLCPP_INFO(get_logger(), "ScanContext sidecar global localizer is ready. publish_initialpose=%s",
      publish_initialpose_ ? "true" : "false");
  }

private:
  enum class QueryMode {
    Conservative,
    GlobalRecovery,
  };

  struct GicpCandidateDebug
  {
    int keyframe_id{-1};
    double fitness{std::numeric_limits<double>::infinity()};
    Eigen::Matrix4f initial_pose{Eigen::Matrix4f::Identity()};
    Eigen::Matrix4f final_pose{Eigen::Matrix4f::Identity()};
  };

  struct GlobalRecoveryObservation {
    Eigen::Matrix4f pose{Eigen::Matrix4f::Identity()};
    double fitness{std::numeric_limits<double>::max()};
    double stamp_sec{0.0};
  };

  void declareParameters()
  {
    database_path_ = declare_parameter<std::string>("database_path", "");
    pcd_map_path_ = declare_parameter<std::string>("pcd_map_path", "");
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/fast_lio/cloud_registered");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    cloud_frame_mode_ = declare_parameter<std::string>("cloud_frame_mode", "registered");
    map_frame_ = declare_parameter<std::string>("map_frame", "camera_init");
    status_topic_ = declare_parameter<std::string>(
      "status_topic", "/scancontext_global_localization/candidates");
    best_pose_topic_ = declare_parameter<std::string>(
      "best_pose_topic", "/scancontext_global_localization/best_pose");
    initialpose_topic_ = declare_parameter<std::string>("initialpose_topic", "/initialpose");

    sc_config_.num_sectors = declare_parameter<int>("num_sectors", 90);
    sc_config_.num_rings = declare_parameter<int>("num_rings", 24);
    sc_config_.max_range = declare_parameter<double>("max_range", 60.0);
    sc_config_.min_range = declare_parameter<double>("min_range", 0.8);
    sc_config_.lateral_search_ratio = declare_parameter<double>("lateral_search_ratio", 0.1);
    sc_config_.horizontal_axis_1 = declare_parameter<int>("horizontal_axis_1", 0);
    sc_config_.horizontal_axis_2 = declare_parameter<int>("horizontal_axis_2", 2);
    sc_config_.vertical_axis = declare_parameter<int>("vertical_axis", 1);
    sc_config_.vertical_sign = declare_parameter<double>("vertical_sign", -1.0);

    top_k_ = declare_parameter<int>("top_k", 5);
    ring_candidates_ = declare_parameter<int>("ring_candidates", 30);
    global_recovery_top_k_ = declare_parameter<int>("global_recovery_top_k", top_k_);
    global_recovery_ring_candidates_ =
      declare_parameter<int>("global_recovery_ring_candidates", ring_candidates_);
    sc_distance_threshold_ = declare_parameter<double>("sc_distance_threshold", 0.25);
    enable_odom_consistency_gate_ = declare_parameter<bool>("enable_odom_consistency_gate", true);
    max_odom_consistency_distance_ = declare_parameter<double>("max_odom_consistency_distance", 1.0);
    max_cloud_odom_time_diff_sec_ = declare_parameter<double>("max_cloud_odom_time_diff_sec", 0.35);
    enable_candidate_confidence_gate_ = declare_parameter<bool>("enable_candidate_confidence_gate", true);
    min_sc_distance_gap_ = declare_parameter<double>("min_sc_distance_gap", 0.03);
    max_ambiguous_candidate_distance_ = declare_parameter<double>("max_ambiguous_candidate_distance", 2.0);
    max_refined_odom_consistency_distance_ = declare_parameter<double>("max_refined_odom_consistency_distance", 1.5);
    voxel_leaf_size_ = declare_parameter<double>("voxel_leaf_size", 0.25);
    min_cloud_points_ = declare_parameter<int>("min_cloud_points", 800);
    query_on_cloud_ = declare_parameter<bool>("query_on_cloud", false);
    query_period_sec_ = declare_parameter<double>("query_period_sec", 0.0);
    publish_initialpose_ = declare_parameter<bool>("publish_initialpose", false);

    enable_gicp_refinement_ = declare_parameter<bool>("enable_gicp_refinement", true);
    gicp_max_iterations_ = declare_parameter<int>("gicp_max_iterations", 50);
    gicp_max_correspondence_distance_ = declare_parameter<double>("gicp_max_correspondence_distance", 2.0);
    gicp_fitness_threshold_ = declare_parameter<double>("gicp_fitness_threshold", 0.6);

    global_recovery_service_ = declare_parameter<std::string>(
      "global_recovery_service", "/scancontext_global_localization/trigger_global");
    global_recovery_sc_distance_threshold_ =
      declare_parameter<double>("global_recovery_sc_distance_threshold", sc_distance_threshold_);
    global_recovery_gicp_fitness_threshold_ =
      declare_parameter<double>("global_recovery_gicp_fitness_threshold", 0.09);
    global_recovery_min_gicp_fitness_gap_ =
      declare_parameter<double>("global_recovery_min_gicp_fitness_gap", 0.02);
    global_recovery_same_solution_xy_tolerance_ =
      declare_parameter<double>("global_recovery_same_solution_xy_tolerance", 0.5);
    global_recovery_same_solution_yaw_tolerance_ =
      declare_parameter<double>("global_recovery_same_solution_yaw_tolerance", 0.35);
    global_recovery_required_consistent_results_ =
      declare_parameter<int>("global_recovery_required_consistent_results", 4);
    global_recovery_consistency_window_ =
      declare_parameter<int>("global_recovery_consistency_window", 8);
    global_recovery_consistency_xy_tolerance_ =
      declare_parameter<double>("global_recovery_consistency_xy_tolerance", 0.5);
    global_recovery_consistency_yaw_tolerance_ =
      declare_parameter<double>("global_recovery_consistency_yaw_tolerance", 0.25);
    global_recovery_observation_max_age_sec_ =
      declare_parameter<double>("global_recovery_observation_max_age_sec", 20.0);
    global_recovery_enable_candidate_confidence_gate_ =
      declare_parameter<bool>("global_recovery_enable_candidate_confidence_gate", true);
    global_recovery_max_refined_odom_consistency_distance_ =
      declare_parameter<double>("global_recovery_max_refined_odom_consistency_distance", 0.0);

    scan_context_ = std::make_unique<ScanContextGlobal>(sc_config_);

    if (cloud_frame_mode_ != "registered" && cloud_frame_mode_ != "local") {
      RCLCPP_WARN(get_logger(),
        "Unknown cloud_frame_mode '%s'; falling back to registered.", cloud_frame_mode_.c_str());
      cloud_frame_mode_ = "registered";
    }
  }

  void loadDatabase()
  {
    if (database_path_.empty()) {
      RCLCPP_WARN(get_logger(), "database_path is empty; trigger requests will fail until configured.");
      return;
    }
    if (!scan_context_->loadDatabase(database_path_)) {
      RCLCPP_ERROR(get_logger(), "Failed to load Scan Context database: %s", database_path_.c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "Loaded Scan Context database: %zu keyframes from %s",
      scan_context_->size(), database_path_.c_str());
  }

  void loadMapIfRequested()
  {
    if (!enable_gicp_refinement_ || pcd_map_path_.empty()) {
      enable_gicp_refinement_ = false;
      RCLCPP_INFO(get_logger(), "GICP refinement disabled; pcd_map_path is empty or disabled.");
      return;
    }

    global_map_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    if (pcl::io::loadPCDFile(pcd_map_path_, *global_map_) != 0) {
      RCLCPP_ERROR(get_logger(), "Failed to load PCD map: %s; disabling GICP refinement.",
        pcd_map_path_.c_str());
      enable_gicp_refinement_ = false;
      return;
    }

    global_map_downsampled_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    downsample(global_map_, global_map_downsampled_);
    RCLCPP_INFO(get_logger(), "Loaded PCD map: %zu points, downsampled to %zu",
      global_map_->size(), global_map_downsampled_->size());
  }

  void setupRos()
  {
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ScanContextGlobalLocalizerNode::cloudCallback, this, std::placeholders::_1));

    if (cloud_frame_mode_ == "registered") {
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ScanContextGlobalLocalizerNode::odomCallback, this, std::placeholders::_1));
    }

    candidates_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);
    best_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(best_pose_topic_, 10);
    initialpose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(initialpose_topic_, 10);

    trigger_srv_ = create_service<std_srvs::srv::Trigger>(
      "/scancontext_global_localization/trigger",
      std::bind(&ScanContextGlobalLocalizerNode::triggerCallback, this, std::placeholders::_1, std::placeholders::_2));
    trigger_global_srv_ = create_service<std_srvs::srv::Trigger>(
      global_recovery_service_,
      std::bind(&ScanContextGlobalLocalizerNode::triggerGlobalCallback, this, std::placeholders::_1, std::placeholders::_2));

    if (query_period_sec_ > 0.0) {
      timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(query_period_sec_)),
        [this]() { (void)this->runQueryFromLatestCloud(QueryMode::Conservative); });
    }
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    latest_cloud_msg_ = msg;
    latest_cloud_pose_valid_ = false;
    if (cloud_frame_mode_ == "registered") {
      if (latest_odom_msg_) {
        const double time_diff = std::abs(
          rclcpp::Time(msg->header.stamp).seconds() -
          rclcpp::Time(latest_odom_msg_->header.stamp).seconds());
        if (max_cloud_odom_time_diff_sec_ <= 0.0 || time_diff <= max_cloud_odom_time_diff_sec_) {
          latest_cloud_pose_ = odomToMatrix(*latest_odom_msg_);
          latest_cloud_pose_valid_ = true;
        } else {
          std::ostringstream oss;
          oss << "latest odom is too far from cloud: dt=" << time_diff
              << "s > " << max_cloud_odom_time_diff_sec_ << "s";
          last_result_message_ = oss.str();
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "%s", last_result_message_.c_str());
        }
      }
    } else {
      latest_cloud_pose_ = Eigen::Matrix4f::Identity();
      latest_cloud_pose_valid_ = true;
    }
    if (query_on_cloud_) {
      runQueryFromLatestCloud(QueryMode::Conservative);
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_odom_msg_ = msg;
  }

  void triggerCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    RCLCPP_INFO(get_logger(), "[TRIGGER] conservative mode called");
    const bool ok = runQueryFromLatestCloud(QueryMode::Conservative);
    response->success = ok;
    response->message = last_result_message_;
  }

  void triggerGlobalCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    RCLCPP_INFO(get_logger(), "[TRIGGER] global mode called");
    const bool ok = runQueryFromLatestCloud(QueryMode::GlobalRecovery);
    response->success = ok;
    response->message = last_result_message_;
  }

  bool runQueryFromLatestCloud(QueryMode mode = QueryMode::Conservative)
  {
    if (!latest_cloud_msg_) {
      last_result_message_ = "no point cloud received yet";
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "%s", last_result_message_.c_str());
      return false;
    }
    if (!scan_context_ || scan_context_->empty()) {
      last_result_message_ = "Scan Context database is empty";
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "%s", last_result_message_.c_str());
      return false;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*latest_cloud_msg_, *cloud);
    if (static_cast<int>(cloud->size()) < min_cloud_points_) {
      std::ostringstream oss;
      oss << "not enough points: " << cloud->size() << " < " << min_cloud_points_;
      last_result_message_ = oss.str();
      return false;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr descriptor_cloud = prepareDescriptorCloud(cloud);
    if (!descriptor_cloud) {
      return false;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
    downsample(descriptor_cloud, filtered);

    const bool global_mode = mode == QueryMode::GlobalRecovery;
    const double sc_threshold =
      global_mode ? global_recovery_sc_distance_threshold_ : sc_distance_threshold_;
    const int query_top_k = global_mode ? global_recovery_top_k_ : top_k_;
    const int query_ring_candidates =
      global_mode ? global_recovery_ring_candidates_ : ring_candidates_;
    auto candidates = scan_context_->search(
      filtered, query_top_k, query_ring_candidates, static_cast<float>(sc_threshold));
    if (candidates.empty()) {
      publishCandidatesJson({}, false, Eigen::Matrix4f::Identity(), 0.0, mode);
      last_result_message_ = "no Scan Context candidate passed threshold";
      return false;
    }

    if (!global_mode) {
      candidates = filterCandidatesByOdomGate(candidates);
      if (candidates.empty()) {
        publishCandidatesJson({}, false, latest_cloud_pose_, 0.0, mode);
        last_result_message_ = "Scan Context candidates rejected by odom consistency gate";
        return false;
      }
    }

    const bool use_confidence_gate = candidateConfidenceGateEnabled(mode);
    RCLCPP_INFO(get_logger(),
      "[GATE] global_mode=%d, use_confidence_gate=%d, "
      "global_recov_enable=%d, conservative_enable=%d",
      int(global_mode), int(use_confidence_gate),
      int(global_recovery_enable_candidate_confidence_gate_),
      int(enable_candidate_confidence_gate_));
    if (use_confidence_gate && !passesCandidateConfidenceGate(candidates)) {
      last_result_message_ = "Scan Context candidates rejected by ambiguity gate";
      publishCandidatesJson(candidates, false, candidates.front().initial_pose, 0.0, mode);
      return false;
    }

    Eigen::Matrix4f best_pose = candidates.front().initial_pose;
    double best_fitness = -1.0;
    bool refined = false;

    if (enable_gicp_refinement_) {
      const double fitness_threshold =
        global_mode ? global_recovery_gicp_fitness_threshold_ : gicp_fitness_threshold_;
      refined = refineWithGicp(filtered, candidates, best_pose, best_fitness, fitness_threshold);
      if (!refined) {
        last_result_message_ = "Scan Context candidates found, but GICP refinement failed gate";
        publishCandidatesJson(candidates, false, candidates.front().initial_pose, best_fitness, mode);
        return false;
      }
      if (global_mode && !passesGicpAmbiguityGate(best_fitness)) {
        publishCandidatesJson(candidates, false, best_pose, best_fitness, mode);
        return false;
      }
    }

    if (!passesRefinedOdomGate(best_pose, mode)) {
      last_result_message_ = "refined pose rejected by odom consistency gate";
      publishCandidatesJson(candidates, false, best_pose, best_fitness, mode);
      return false;
    }

    if (global_mode && !passesGlobalRecoveryConsistency(best_pose, best_fitness)) {
      publishCandidatesJson(candidates, false, best_pose, best_fitness, mode);
      return false;
    }

    publishBestPose(best_pose, best_fitness, refined);
    publishCandidatesJson(candidates, true, best_pose, best_fitness, mode);

    std::ostringstream oss;
    oss << (global_mode ? "global recovery accepted keyframe=" : "accepted keyframe=")
        << (last_gicp_best_keyframe_id_ >= 0 ? last_gicp_best_keyframe_id_ : candidates.front().keyframe_id)
        << " sc_distance=" << candidates.front().distance
        << " gicp_fitness=" << best_fitness
        << " refined=" << (refined ? "true" : "false");
    last_result_message_ = oss.str();
    RCLCPP_INFO(get_logger(), "%s", last_result_message_.c_str());
    return true;
  }

  bool candidateConfidenceGateEnabled(QueryMode mode) const
  {
    if (mode == QueryMode::GlobalRecovery) {
      // Startup/global recovery intentionally searches the full map, where
      // repeated corridor geometry can produce several equally good Scan
      // Context candidates. Correctness is enforced by GICP refinement,
      // GICP ambiguity gating, repeated global-recovery consistency, and
      // downstream NDT validation.
      return false;
    }
    return enable_candidate_confidence_gate_;
  }

  std::vector<ScanContextGlobal::Candidate> filterCandidatesByOdomGate(
    const std::vector<ScanContextGlobal::Candidate>& candidates) const
  {
    if (!enable_odom_consistency_gate_ || cloud_frame_mode_ != "registered" || !latest_cloud_pose_valid_) {
      return candidates;
    }

    std::vector<ScanContextGlobal::Candidate> filtered;
    filtered.reserve(candidates.size());
    const int a = sc_config_.horizontal_axis_1;
    const int b = sc_config_.horizontal_axis_2;
    if (a < 0 || a > 2 || b < 0 || b > 2 || a == b) {
      return candidates;
    }

    for (const auto& candidate : candidates) {
      const double da = candidate.initial_pose(a, 3) - latest_cloud_pose_(a, 3);
      const double db = candidate.initial_pose(b, 3) - latest_cloud_pose_(b, 3);
      const double distance = std::hypot(da, db);
      if (distance <= max_odom_consistency_distance_) {
        filtered.push_back(candidate);
      }
    }
    return filtered;
  }

  double horizontalDistance(const Eigen::Matrix4f& lhs, const Eigen::Matrix4f& rhs) const
  {
    const int a = sc_config_.horizontal_axis_1;
    const int b = sc_config_.horizontal_axis_2;
    if (a < 0 || a > 2 || b < 0 || b > 2 || a == b) {
      return 0.0;
    }
    return std::hypot(lhs(a, 3) - rhs(a, 3), lhs(b, 3) - rhs(b, 3));
  }

  bool passesCandidateConfidenceGate(const std::vector<ScanContextGlobal::Candidate>& candidates) const
  {
    if (candidates.size() < 2) {
      return true;
    }

    const double best_distance = candidates.front().distance;
    for (size_t i = 1; i < candidates.size(); ++i) {
      const double distance_gap = static_cast<double>(candidates[i].distance) - best_distance;
      if (distance_gap >= min_sc_distance_gap_) {
        break;
      }
      if (horizontalDistance(candidates.front().initial_pose, candidates[i].initial_pose) >
          max_ambiguous_candidate_distance_) {
        return false;
      }
    }
    return true;
  }

  double yawDistance(double lhs, double rhs) const
  {
    double diff = std::fmod(lhs - rhs + M_PI, 2.0 * M_PI);
    if (diff < 0.0) {
      diff += 2.0 * M_PI;
    }
    return std::abs(diff - M_PI);
  }

  bool passesRefinedOdomGate(const Eigen::Matrix4f& pose, QueryMode mode) const
  {
    if (!enable_odom_consistency_gate_ || cloud_frame_mode_ != "registered" || !latest_cloud_pose_valid_) {
      return true;
    }
    if (mode == QueryMode::GlobalRecovery) {
      if (global_recovery_max_refined_odom_consistency_distance_ <= 0.0) {
        return true;
      }
      return horizontalDistance(pose, latest_cloud_pose_) <=
        global_recovery_max_refined_odom_consistency_distance_;
    }
    if (max_refined_odom_consistency_distance_ <= 0.0) {
      return true;
    }
    return horizontalDistance(pose, latest_cloud_pose_) <= max_refined_odom_consistency_distance_;
  }

  bool passesGlobalRecoveryConsistency(const Eigen::Matrix4f& pose, double fitness)
  {
    const double stamp = now().seconds();
    global_recovery_observations_.push_back({pose, fitness, stamp});
    while (global_recovery_observations_.size() >
           static_cast<size_t>(std::max(1, global_recovery_consistency_window_))) {
      global_recovery_observations_.erase(global_recovery_observations_.begin());
    }
    global_recovery_observations_.erase(
      std::remove_if(
        global_recovery_observations_.begin(), global_recovery_observations_.end(),
        [&](const GlobalRecoveryObservation& observation) {
          return global_recovery_observation_max_age_sec_ > 0.0 &&
            stamp - observation.stamp_sec > global_recovery_observation_max_age_sec_;
        }),
      global_recovery_observations_.end());

    const double yaw = yawFromMatrix(pose);
    int consistent = 0;
    double best_fitness_in_cluster = std::numeric_limits<double>::max();
    for (const auto& observation : global_recovery_observations_) {
      if (horizontalDistance(pose, observation.pose) <= global_recovery_consistency_xy_tolerance_ &&
          yawDistance(yaw, yawFromMatrix(observation.pose)) <= global_recovery_consistency_yaw_tolerance_) {
        ++consistent;
        best_fitness_in_cluster = std::min(best_fitness_in_cluster, observation.fitness);
      }
    }

    if (consistent < std::max(1, global_recovery_required_consistent_results_)) {
      std::ostringstream oss;
      oss << "global recovery waiting for consistent results: " << consistent << "/"
          << global_recovery_required_consistent_results_
          << " fitness=" << fitness;
      last_result_message_ = oss.str();
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "%s", last_result_message_.c_str());
      return false;
    }

    std::ostringstream oss;
    oss << "global recovery consistency passed: " << consistent << "/"
        << global_recovery_required_consistent_results_
        << " best_cluster_fitness=" << best_fitness_in_cluster;
    last_result_message_ = oss.str();
    global_recovery_observations_.clear();
    return true;
  }

  bool passesGicpAmbiguityGate(double best_fitness)
  {
    if (global_recovery_min_gicp_fitness_gap_ <= 0.0 ||
        !std::isfinite(last_gicp_second_best_fitness_)) {
      return true;
    }
    const double gap = last_gicp_second_best_fitness_ - best_fitness;
    if (gap >= global_recovery_min_gicp_fitness_gap_) {
      return true;
    }

    std::ostringstream oss;
    oss << "GICP candidates ambiguous: best_fitness=" << best_fitness
        << " second_best_fitness=" << last_gicp_second_best_fitness_
        << " gap=" << gap
        << " required_gap=" << global_recovery_min_gicp_fitness_gap_;
    last_result_message_ = oss.str();
    RCLCPP_WARN(get_logger(), "%s", last_result_message_.c_str());
    return false;
  }

  bool isSameGicpSolution(const Eigen::Matrix4f& lhs, const Eigen::Matrix4f& rhs) const
  {
    return horizontalDistance(lhs, rhs) <= global_recovery_same_solution_xy_tolerance_ &&
      yawDistance(yawFromMatrix(lhs), yawFromMatrix(rhs)) <=
      global_recovery_same_solution_yaw_tolerance_;
  }

  void updateSecondBestDistinctGicpFitness(const Eigen::Matrix4f& best_pose)
  {
    last_gicp_second_best_fitness_ = std::numeric_limits<double>::infinity();
    for (const auto& debug : last_gicp_candidates_) {
      if (debug.keyframe_id == last_gicp_best_keyframe_id_) {
        continue;
      }
      if (isSameGicpSolution(best_pose, debug.final_pose)) {
        continue;
      }
      last_gicp_second_best_fitness_ =
        std::min(last_gicp_second_best_fitness_, debug.fitness);
    }
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr prepareDescriptorCloud(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud)
  {
    if (cloud_frame_mode_ == "local") {
      return pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>(*cloud));
    }

    if (!latest_cloud_pose_valid_) {
      last_result_message_ = "no odom pose captured for latest registered cloud";
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "%s", last_result_message_.c_str());
      return nullptr;
    }

    const Eigen::Matrix3f rotation_inv = latest_cloud_pose_.block<3, 3>(0, 0).transpose();
    const Eigen::Vector3f translation = latest_cloud_pose_.block<3, 1>(0, 3);

    pcl::PointCloud<pcl::PointXYZI>::Ptr local(new pcl::PointCloud<pcl::PointXYZI>());
    local->reserve(cloud->size());
    local->header = cloud->header;
    for (const auto& point : cloud->points) {
      const Eigen::Vector3f world(point.x, point.y, point.z);
      const Eigen::Vector3f body = rotation_inv * (world - translation);
      pcl::PointXYZI out = point;
      out.x = body.x();
      out.y = body.y();
      out.z = body.z();
      local->push_back(out);
    }
    local->width = static_cast<uint32_t>(local->size());
    local->height = 1;
    local->is_dense = cloud->is_dense;
    return local;
  }

  bool refineWithGicp(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& source,
    const std::vector<ScanContextGlobal::Candidate>& candidates,
    Eigen::Matrix4f& best_pose,
    double& best_fitness,
    double fitness_threshold)
  {
    if (!global_map_downsampled_ || global_map_downsampled_->empty()) {
      return false;
    }

    bool found = false;
    best_fitness = std::numeric_limits<double>::max();
    last_gicp_second_best_fitness_ = std::numeric_limits<double>::infinity();
    last_gicp_best_keyframe_id_ = -1;
    last_gicp_candidates_.clear();

    for (const auto& candidate : candidates) {
      pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> gicp;
      gicp.setMaximumIterations(gicp_max_iterations_);
      gicp.setMaxCorrespondenceDistance(gicp_max_correspondence_distance_);
      gicp.setInputSource(source);
      gicp.setInputTarget(global_map_downsampled_);

      pcl::PointCloud<pcl::PointXYZI> aligned;
      gicp.align(aligned, candidate.initial_pose);
      if (!gicp.hasConverged()) {
        continue;
      }

      const double fitness = gicp.getFitnessScore(gicp_max_correspondence_distance_);
      last_gicp_candidates_.push_back(
        {candidate.keyframe_id, fitness, candidate.initial_pose, gicp.getFinalTransformation()});
      if (fitness < best_fitness) {
        best_fitness = fitness;
        best_pose = gicp.getFinalTransformation();
        last_gicp_best_keyframe_id_ = candidate.keyframe_id;
        found = true;
      }
    }

    if (found) {
      updateSecondBestDistinctGicpFitness(best_pose);
    }
    return found && best_fitness <= fitness_threshold;
  }

  void downsample(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& input,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& output) const
  {
    pcl::VoxelGrid<pcl::PointXYZI> voxel;
    voxel.setInputCloud(input);
    voxel.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    voxel.filter(*output);
  }

  void publishBestPose(const Eigen::Matrix4f& pose, double fitness, bool refined)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = map_frame_;
    msg.pose.pose = poseFromMatrix(pose);
    const double covariance_xy = refined ? std::max(0.05, fitness) : 1.0;
    msg.pose.covariance[0] = covariance_xy;
    msg.pose.covariance[7] = covariance_xy;
    msg.pose.covariance[35] = refined ? 0.05 : 0.5;
    best_pose_pub_->publish(msg);
    if (publish_initialpose_) {
      initialpose_pub_->publish(msg);
    }
  }

  void publishCandidatesJson(
    const std::vector<ScanContextGlobal::Candidate>& candidates,
    bool accepted,
    const Eigen::Matrix4f& best_pose,
    double best_fitness,
    QueryMode mode)
  {
    const bool global_mode = mode == QueryMode::GlobalRecovery;
    std::ostringstream json;
    json << "{\"stamp\":" << now().seconds()
         << ",\"mode\":\"" << (global_mode ? "global_recovery" : "conservative") << "\""
         << ",\"cloud_frame_mode\":\"" << cloud_frame_mode_ << "\""
         << ",\"odom_gate_enabled\":" << (enable_odom_consistency_gate_ ? "true" : "false")
         << ",\"max_odom_consistency_distance\":"
         << (global_mode ? global_recovery_max_refined_odom_consistency_distance_ : max_odom_consistency_distance_)
         << ",\"candidate_confidence_gate_enabled\":"
         << (candidateConfidenceGateEnabled(mode) ? "true" : "false")
         << ",\"configured_global_candidate_confidence_gate\":"
         << (global_recovery_enable_candidate_confidence_gate_ ? "true" : "false")
         << ",\"min_sc_distance_gap\":" << min_sc_distance_gap_
         << ",\"global_recovery_consistency_count\":" << global_recovery_observations_.size()
         << ",\"accepted\":" << (accepted ? "true" : "false")
         << ",\"failure_reason\":\"" << last_result_message_ << "\""
         << ",\"gicp_fitness\":" << best_fitness
         << ",\"gicp_second_best_fitness\":" << last_gicp_second_best_fitness_
         << ",\"gicp_best_keyframe_id\":" << last_gicp_best_keyframe_id_
         << ",\"best_pose\":{\"x\":" << best_pose(0, 3)
         << ",\"y\":" << best_pose(1, 3)
         << ",\"z\":" << best_pose(2, 3)
         << ",\"yaw_deg\":" << yawFromMatrix(best_pose) * 180.0 / M_PI << "}";
    const Eigen::Matrix4f best_pose_ros = fastlioToRosMatrix(best_pose);
    json << ",\"best_pose_ros\":{\"x\":" << best_pose_ros(0, 3)
         << ",\"y\":" << best_pose_ros(1, 3)
         << ",\"z\":" << best_pose_ros(2, 3)
         << ",\"yaw_deg\":" << yawFromMatrix(best_pose_ros) * 180.0 / M_PI << "}"
         << ",\"gicp_candidates\":[";
    for (size_t i = 0; i < last_gicp_candidates_.size(); ++i) {
      const auto& debug = last_gicp_candidates_[i];
      if (i > 0) {
        json << ",";
      }
      const Eigen::Matrix4f final_ros = fastlioToRosMatrix(debug.final_pose);
      json << "{\"keyframe_id\":" << debug.keyframe_id
           << ",\"fitness\":" << debug.fitness
           << ",\"initial_pose\":{\"x\":" << debug.initial_pose(0, 3)
           << ",\"y\":" << debug.initial_pose(1, 3)
           << ",\"z\":" << debug.initial_pose(2, 3)
           << ",\"yaw_deg\":" << yawFromMatrix(debug.initial_pose) * 180.0 / M_PI << "}"
           << ",\"final_pose\":{\"x\":" << debug.final_pose(0, 3)
           << ",\"y\":" << debug.final_pose(1, 3)
           << ",\"z\":" << debug.final_pose(2, 3)
           << ",\"yaw_deg\":" << yawFromMatrix(debug.final_pose) * 180.0 / M_PI << "}"
           << ",\"final_pose_ros\":{\"x\":" << final_ros(0, 3)
           << ",\"y\":" << final_ros(1, 3)
           << ",\"z\":" << final_ros(2, 3)
           << ",\"yaw_deg\":" << yawFromMatrix(final_ros) * 180.0 / M_PI << "}}";
    }
    json << "]"
         << ",\"candidates\":[";
    for (size_t i = 0; i < candidates.size(); ++i) {
      const auto& candidate = candidates[i];
      if (i > 0) {
        json << ",";
      }
      json << "{\"keyframe_id\":" << candidate.keyframe_id
           << ",\"distance\":" << candidate.distance
           << ",\"yaw_deg\":" << candidate.yaw_rad * 180.0 / M_PI
           << ",\"x\":" << candidate.initial_pose(0, 3)
           << ",\"y\":" << candidate.initial_pose(1, 3) << "}";
    }
    json << "]}";

    std_msgs::msg::String msg;
    msg.data = json.str();
    candidates_pub_->publish(msg);
  }

  std::string database_path_;
  std::string pcd_map_path_;
  std::string cloud_topic_;
  std::string odom_topic_;
  std::string cloud_frame_mode_;
  std::string map_frame_;
  std::string status_topic_;
  std::string best_pose_topic_;
  std::string initialpose_topic_;
  std::string global_recovery_service_;
  std::string last_result_message_;

  ScanContextGlobal::Config sc_config_;
  std::unique_ptr<ScanContextGlobal> scan_context_;

  int top_k_{5};
  int ring_candidates_{30};
  int global_recovery_top_k_{5};
  int global_recovery_ring_candidates_{30};
  int min_cloud_points_{800};
  int gicp_max_iterations_{50};
  double sc_distance_threshold_{0.25};
  double max_odom_consistency_distance_{1.0};
  double max_cloud_odom_time_diff_sec_{0.35};
  double min_sc_distance_gap_{0.03};
  double max_ambiguous_candidate_distance_{2.0};
  double max_refined_odom_consistency_distance_{1.5};
  double voxel_leaf_size_{0.25};
  double query_period_sec_{0.0};
  double gicp_max_correspondence_distance_{2.0};
  double gicp_fitness_threshold_{0.6};
  double global_recovery_sc_distance_threshold_{0.25};
  double global_recovery_gicp_fitness_threshold_{0.09};
  double global_recovery_min_gicp_fitness_gap_{0.02};
  double global_recovery_same_solution_xy_tolerance_{0.5};
  double global_recovery_same_solution_yaw_tolerance_{0.35};
  double global_recovery_consistency_xy_tolerance_{0.5};
  double global_recovery_consistency_yaw_tolerance_{0.25};
  double global_recovery_observation_max_age_sec_{20.0};
  double global_recovery_max_refined_odom_consistency_distance_{0.0};
  int global_recovery_required_consistent_results_{4};
  int global_recovery_consistency_window_{8};
  bool query_on_cloud_{false};
  bool publish_initialpose_{false};
  bool enable_gicp_refinement_{true};
  bool enable_odom_consistency_gate_{true};
  bool enable_candidate_confidence_gate_{true};
  bool global_recovery_enable_candidate_confidence_gate_{true};
  double last_gicp_second_best_fitness_{std::numeric_limits<double>::infinity()};
  int last_gicp_best_keyframe_id_{-1};
  std::vector<GicpCandidateDebug> last_gicp_candidates_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr global_map_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr global_map_downsampled_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_msg_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_msg_;
  Eigen::Matrix4f latest_cloud_pose_{Eigen::Matrix4f::Identity()};
  bool latest_cloud_pose_valid_{false};
  std::vector<GlobalRecoveryObservation> global_recovery_observations_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr candidates_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr best_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_global_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace humanoid_scancontext_global_localization

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<
    humanoid_scancontext_global_localization::ScanContextGlobalLocalizerNode>());
  rclcpp::shutdown();
  return 0;
}
