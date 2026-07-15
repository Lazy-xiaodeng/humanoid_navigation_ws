/*
 * prior_map_odom_bridge_cpp.cpp
 *
 * 文件作用：
 * 1. 接收外部全局定位 pose/odom，并结合 Fast-LIO odom 缓存计算 map -> odom。
 * 2. 独占发布 map -> odom TF，避免多个定位节点抢同一条全局 TF。
 * 3. 发布 /localization/prior_map_odom_bridge_status，给路线运行层判断定位健康和是否允许启动/恢复导航。
 * 4. 包含跳变保护、连续帧一致性、导航中大跳门控和 SpinToPose 保护等定位稳定性逻辑。
 *
 * 上游：RoboSense 定位、Fast-LIO /odom、/navigation/status、RViz initialpose adapter。
 * 下游：TF 树、Nav2、路线运行层和地图切换门控。
 */

#include <algorithm>
#include <cmath>
#include <deque>
#include <memory>
#include <optional>
#include <sstream>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace
{
using Matrix4 = Eigen::Matrix4d;

double stamp_to_sec(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

bool is_zero_stamp(const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec == 0 && stamp.nanosec == 0;
}

double normalize_angle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

Matrix4 identity()
{
  return Matrix4::Identity();
}

Eigen::Quaterniond normalized_quaternion(double x, double y, double z, double w)
{
  Eigen::Quaterniond q(w, x, y, z);
  if (q.norm() < 1e-12) {
    return Eigen::Quaterniond::Identity();
  }
  q.normalize();
  return q;
}

Matrix4 pose_to_matrix(const geometry_msgs::msg::Pose & pose)
{
  Matrix4 matrix = identity();
  const auto q = normalized_quaternion(
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  matrix.block<3, 3>(0, 0) = q.toRotationMatrix();
  matrix(0, 3) = pose.position.x;
  matrix(1, 3) = pose.position.y;
  matrix(2, 3) = pose.position.z;
  return matrix;
}

Matrix4 odom_to_matrix(const nav_msgs::msg::Odometry & odom)
{
  return pose_to_matrix(odom.pose.pose);
}

Matrix4 transform_to_matrix(const geometry_msgs::msg::TransformStamped & tf)
{
  Matrix4 matrix = identity();
  const auto q = normalized_quaternion(
    tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z,
    tf.transform.rotation.w);
  matrix.block<3, 3>(0, 0) = q.toRotationMatrix();
  matrix(0, 3) = tf.transform.translation.x;
  matrix(1, 3) = tf.transform.translation.y;
  matrix(2, 3) = tf.transform.translation.z;
  return matrix;
}

double yaw_from_matrix(const Matrix4 & matrix)
{
  return std::atan2(matrix(1, 0), matrix(0, 0));
}

std::pair<double, double> correction_delta(const Matrix4 & candidate, const Matrix4 & accepted)
{
  const double dx = candidate(0, 3) - accepted(0, 3);
  const double dy = candidate(1, 3) - accepted(1, 3);
  const double xy = std::hypot(dx, dy);
  const double yaw = std::abs(normalize_angle(yaw_from_matrix(candidate) - yaw_from_matrix(accepted)));
  return {xy, yaw};
}

std::string fixed3(double value)
{
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out.precision(3);
  out << value;
  return out.str();
}

std::string fixed2(double value)
{
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out.precision(2);
  out << value;
  return out.str();
}
}  // namespace

class PriorMapOdomBridgeCpp : public rclcpp::Node
{
public:
  PriorMapOdomBridgeCpp()
  : Node("prior_map_odom_bridge"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    localized_frame_ = declare_parameter<std::string>("localized_frame", "base_footprint");

    prior_pose_topic_ = declare_parameter<std::string>("prior_pose_topic", "/prior_localization/pose");
    prior_pose_with_covariance_topic_ = declare_parameter<std::string>(
      "prior_pose_with_covariance_topic", "/prior_localization/pose_with_covariance");
    prior_odom_topic_ = declare_parameter<std::string>("prior_odom_topic", "/prior_localization/odom");
    global_recovery_map_to_odom_topic_ = declare_parameter<std::string>(
      "global_recovery_map_to_odom_topic", "/localization/global_recovery_map_to_odom");
    global_recovery_timeout_sec_ = declare_parameter<double>("global_recovery_timeout_sec", 2.5);
    confidence_topic_ = declare_parameter<std::string>("confidence_topic", "/prior_localization/confidence");
    require_confidence_ = declare_parameter<bool>("require_confidence", false);
    min_confidence_ = declare_parameter<double>("min_confidence", 0.5);
    confidence_timeout_sec_ = declare_parameter<double>("confidence_timeout_sec", 1.5);

    publish_rate_ = declare_parameter<double>("publish_rate", 30.0);
    tf_lookup_timeout_sec_ = declare_parameter<double>("tf_lookup_timeout_sec", 0.08);
    pose_timeout_sec_ = declare_parameter<double>("pose_timeout_sec", 0.8);
    accept_zero_stamp_ = declare_parameter<bool>("accept_zero_stamp", true);
    allow_initial_pose_ = declare_parameter<bool>("allow_initial_pose", true);
    origin_seed_radius_ = declare_parameter<double>("origin_seed_radius", 0.30);

    use_odom_cache_ = declare_parameter<bool>("use_odom_cache", true);
    odom_cache_topic_ = declare_parameter<std::string>(
      "odom_cache_topic", "/prior_localization/open3d_input_odom");
    odom_cache_duration_sec_ = declare_parameter<double>("odom_cache_duration_sec", 5.0);
    odom_interpolation_max_gap_sec_ = declare_parameter<double>("odom_interpolation_max_gap_sec", 0.25);
    odom_lookup_tolerance_sec_ = declare_parameter<double>("odom_lookup_tolerance_sec", 0.03);
    odom_future_wait_sec_ = declare_parameter<double>("odom_future_wait_sec", 0.20);
    fallback_to_tf_lookup_ = declare_parameter<bool>("fallback_to_tf_lookup", true);

    max_small_correction_translation_ = declare_parameter<double>("max_small_correction_translation", 0.25);
    max_small_correction_yaw_ = declare_parameter<double>("max_small_correction_yaw", 0.12);
    max_large_correction_translation_ = declare_parameter<double>("max_large_correction_translation", 3.0);
    max_large_correction_yaw_ = declare_parameter<double>("max_large_correction_yaw", 1.2);
    required_consistent_frames_ = declare_parameter<int>("required_consistent_frames", 5);
    consistency_translation_tolerance_ = declare_parameter<double>("consistency_translation_tolerance", 0.25);
    consistency_yaw_tolerance_ = declare_parameter<double>("consistency_yaw_tolerance", 0.10);

    jump_protection_mode_ = declare_parameter<std::string>("jump_protection_mode", "off");
    std::transform(
      jump_protection_mode_.begin(), jump_protection_mode_.end(), jump_protection_mode_.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (
      jump_protection_mode_ != "off" && jump_protection_mode_ != "monitor" &&
      jump_protection_mode_ != "protect")
    {
      RCLCPP_WARN(get_logger(), "unknown jump_protection_mode=%s, fallback to off", jump_protection_mode_.c_str());
      jump_protection_mode_ = "off";
    }

    nav_medium_correction_translation_ = declare_parameter<double>("nav_medium_correction_translation", 0.50);
    nav_medium_correction_yaw_ = declare_parameter<double>("nav_medium_correction_yaw", 0.20);
    nav_medium_required_frames_ = declare_parameter<int>("nav_medium_required_frames", 5);
    nav_large_correction_translation_ = declare_parameter<double>("nav_large_correction_translation", 0.50);
    nav_large_correction_yaw_ = declare_parameter<double>("nav_large_correction_yaw", 0.20);
    allow_nav_large_jump_ = declare_parameter<bool>("allow_nav_large_jump", false);
    idle_large_correction_translation_ = declare_parameter<double>("idle_large_correction_translation", 1.00);
    idle_large_correction_yaw_ = declare_parameter<double>("idle_large_correction_yaw", 0.35);
    idle_large_required_frames_ = declare_parameter<int>("idle_large_required_frames", 5);
    allow_idle_large_jump_ = declare_parameter<bool>("allow_idle_large_jump", true);
    hard_reject_translation_ = declare_parameter<double>("hard_reject_translation", 1.00);
    hard_reject_yaw_ = declare_parameter<double>("hard_reject_yaw", 0.50);
    large_jump_degraded_after_sec_ = declare_parameter<double>("large_jump_degraded_after_sec", 3.0);

    enable_spin_to_pose_guard_ = declare_parameter<bool>("enable_spin_to_pose_guard", true);
    navigation_status_topic_ = declare_parameter<std::string>("navigation_status_topic", "/navigation/status");
    spin_to_pose_guard_settle_sec_ = declare_parameter<double>("spin_to_pose_guard_settle_sec", 1.2);
    spin_to_pose_guard_max_duration_sec_ = declare_parameter<double>("spin_to_pose_guard_max_duration_sec", 8.0);
    force_2d_ = declare_parameter<bool>("force_2d", true);
    force_z_ = declare_parameter<double>("force_z", 0.0);

    status_pub_ = create_publisher<std_msgs::msg::String>("/localization/prior_map_odom_bridge_status", 10);
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      prior_pose_topic_, 10,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        handle_candidate_pose(msg->header, msg->pose);
      });
    pose_cov_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      prior_pose_with_covariance_topic_, 10,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        handle_candidate_pose(msg->header, msg->pose.pose);
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      prior_odom_topic_, 10,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        handle_candidate_pose(msg->header, msg->pose.pose);
      });
    global_recovery_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      global_recovery_map_to_odom_topic_, 10,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        handle_global_recovery_map_to_odom(*msg);
      });
    odom_cache_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_cache_topic_, 100,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) { on_odom_cache(*msg); });
    confidence_sub_ = create_subscription<std_msgs::msg::Float32>(
      confidence_topic_, 10,
      [this](std_msgs::msg::Float32::SharedPtr msg) {
        latest_confidence_ = msg->data;
        latest_confidence_time_ = now();
      });
    navigation_status_sub_ = create_subscription<std_msgs::msg::String>(
      navigation_status_topic_, 10,
      [this](std_msgs::msg::String::SharedPtr msg) { on_navigation_status(msg->data); });

    if (publish_rate_ <= 0.0) {
      publish_rate_ = 30.0;
    }
    publish_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_),
      [this]() { publish_last_tf(); });

    RCLCPP_INFO(
      get_logger(),
      "prior_map_odom_bridge_cpp started: input=%s, %s, or %s, localized_frame=%s, publishing %s->%s, use_odom_cache=%s, odom_cache_topic=%s",
      prior_pose_topic_.c_str(), prior_pose_with_covariance_topic_.c_str(), prior_odom_topic_.c_str(),
      localized_frame_.c_str(), map_frame_.c_str(), odom_frame_.c_str(),
      use_odom_cache_ ? "true" : "false", odom_cache_topic_.c_str());
  }

private:
  struct OdomEntry
  {
    double stamp{};
    Matrix4 matrix{identity()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  };

  struct PendingOdomCandidate
  {
    double target{};
    rclcpp::Time lookup_time;
    Matrix4 map_to_localized{identity()};
    rclcpp::Time queued_at;
  };

  void on_odom_cache(const nav_msgs::msg::Odometry & msg)
  {
    if (!use_odom_cache_) {
      return;
    }
    if (!msg.header.frame_id.empty() && msg.header.frame_id != odom_frame_) {
      publish_status("REJECTED odom_cache_wrong_frame frame_id=" + msg.header.frame_id + " expected=" + odom_frame_);
      return;
    }
    if (!msg.child_frame_id.empty() && msg.child_frame_id != localized_frame_) {
      publish_status(
        "REJECTED odom_cache_wrong_child child_frame_id=" + msg.child_frame_id + " expected=" + localized_frame_);
      return;
    }
    const double stamp = stamp_to_sec(msg.header.stamp);
    if (stamp <= 0.0) {
      return;
    }
    OdomEntry entry;
    entry.stamp = stamp;
    entry.matrix = odom_to_matrix(msg);
    entry.orientation = normalized_quaternion(
      msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
      msg.pose.pose.orientation.w);

    if (!odom_cache_.empty() && stamp < odom_cache_.back().stamp) {
      auto it = std::find_if(
        odom_cache_.begin(), odom_cache_.end(),
        [stamp](const OdomEntry & cached) { return stamp < cached.stamp; });
      odom_cache_.insert(it, entry);
    } else {
      odom_cache_.push_back(entry);
    }

    const double cutoff = stamp - odom_cache_duration_sec_;
    while (!odom_cache_.empty() && odom_cache_.front().stamp < cutoff) {
      odom_cache_.pop_front();
    }
    process_pending_odom_candidates();
  }

  void on_navigation_status(const std::string & data)
  {
    navigation_active_ = navigation_status_is_active(data);
    if (!enable_spin_to_pose_guard_) {
      return;
    }
    const bool turning = navigation_status_is_turning(data);
    const auto now_time = now();
    if (turning) {
      enter_spin_guard(now_time, "navigation_status");
      spin_guard_turning_ = true;
      return;
    }
    if (spin_guard_turning_) {
      spin_guard_turning_ = false;
      spin_guard_settle_until_ =
        now_time + rclcpp::Duration::from_seconds(spin_to_pose_guard_settle_sec_);
      publish_status("SPIN_GUARD settling sec=" + fixed2(spin_to_pose_guard_settle_sec_));
    }
  }

  bool navigation_status_is_turning(const std::string & data) const
  {
    const std::string upper = to_upper(data);
    const std::string lower = to_lower(data);
    const bool active =
      lower.find("executing") != std::string::npos || lower.find("planning") != std::string::npos ||
      lower.find("running") != std::string::npos || lower.find("active") != std::string::npos ||
      upper.find("NAVIGATING") != std::string::npos;
    return active && upper.find("TURNING") != std::string::npos;
  }

  bool navigation_status_is_active(const std::string & data) const
  {
    const std::string upper = to_upper(data);
    const std::string lower = to_lower(data);
    if (
      upper.find("COMPLETED") != std::string::npos || upper.find("SUCCEEDED") != std::string::npos ||
      upper.find("IDLE") != std::string::npos)
    {
      return false;
    }
    return lower.find("executing") != std::string::npos || lower.find("planning") != std::string::npos ||
           lower.find("running") != std::string::npos || lower.find("active") != std::string::npos ||
           upper.find("NAVIGATING") != std::string::npos || upper.find("PLANNING") != std::string::npos ||
           upper.find("CONTROLLING") != std::string::npos || upper.find("TURNING") != std::string::npos;
  }

  std::optional<Matrix4> lookup_odom_to_localized_matrix(const rclcpp::Time & lookup_time, bool zero_stamp)
  {
    last_odom_cache_failure_.clear();
    if (use_odom_cache_) {
      const auto cached = lookup_odom_cache_matrix(lookup_time, zero_stamp);
      if (cached) {
        return cached;
      }
      if (last_odom_cache_failure_ == "future") {
        return std::nullopt;
      }
      if (!fallback_to_tf_lookup_) {
        return std::nullopt;
      }
    }

    try {
      const rclcpp::Time tf_lookup_time = zero_stamp ? rclcpp::Time(0, 0, get_clock()->get_clock_type()) : lookup_time;
      const auto tf = tf_buffer_.lookupTransform(
        odom_frame_, localized_frame_, tf_lookup_time,
        tf2::durationFromSec(tf_lookup_timeout_sec_));
      return transform_to_matrix(tf);
    } catch (const tf2::TransformException & exc) {
      publish_status(
        "REJECTED missing_tf " + odom_frame_ + "->" + localized_frame_ + ": " + std::string(exc.what()));
      return std::nullopt;
    }
  }

  std::optional<Matrix4> lookup_odom_cache_matrix(const rclcpp::Time & lookup_time, bool zero_stamp)
  {
    if (odom_cache_.empty()) {
      last_odom_cache_failure_ = "empty";
      publish_status("REJECTED odom_cache_empty");
      return std::nullopt;
    }
    if (zero_stamp) {
      return odom_cache_.back().matrix;
    }

    const double target = lookup_time.seconds();
    const double first_time = odom_cache_.front().stamp;
    const double last_time = odom_cache_.back().stamp;
    if (target < first_time) {
      if (first_time - target <= odom_lookup_tolerance_sec_) {
        return odom_cache_.front().matrix;
      }
      last_odom_cache_failure_ = "too_old";
      publish_status(
        "REJECTED odom_cache_too_old target=" + fixed3(target) + " first=" + fixed3(first_time));
      return std::nullopt;
    }
    if (target > last_time) {
      if (target - last_time <= odom_lookup_tolerance_sec_) {
        return odom_cache_.back().matrix;
      }
      last_odom_cache_failure_ = "future";
      return std::nullopt;
    }

    std::optional<OdomEntry> previous;
    std::optional<OdomEntry> following;
    for (const auto & entry : odom_cache_) {
      if (entry.stamp <= target) {
        previous = entry;
      }
      if (entry.stamp >= target) {
        following = entry;
        break;
      }
    }
    if (!previous || !following) {
      last_odom_cache_failure_ = "lookup_failed";
      publish_status("REJECTED odom_cache_lookup_failed");
      return std::nullopt;
    }
    if (std::abs(target - previous->stamp) <= 1e-6) {
      return previous->matrix;
    }
    if (std::abs(following->stamp - target) <= 1e-6) {
      return following->matrix;
    }

    const double gap = following->stamp - previous->stamp;
    if (gap <= 1e-9 || gap > odom_interpolation_max_gap_sec_) {
      last_odom_cache_failure_ = "gap";
      publish_status(
        "REJECTED odom_cache_gap gap=" + fixed3(gap) + "s max=" + fixed3(odom_interpolation_max_gap_sec_) + "s");
      return std::nullopt;
    }

    const double ratio = (target - previous->stamp) / gap;
    Matrix4 interpolated = identity();
    interpolated.block<3, 1>(0, 3) =
      previous->matrix.block<3, 1>(0, 3) +
      ratio * (following->matrix.block<3, 1>(0, 3) - previous->matrix.block<3, 1>(0, 3));
    interpolated.block<3, 3>(0, 0) = previous->orientation.slerp(ratio, following->orientation).toRotationMatrix();
    return interpolated;
  }

  bool queue_pending_odom_candidate(const rclcpp::Time & lookup_time, const Matrix4 & map_to_localized)
  {
    if (!use_odom_cache_ || odom_cache_.empty()) {
      return false;
    }
    const double target = lookup_time.seconds();
    const double latest = odom_cache_.back().stamp;
    const double future_gap = target - latest;
    if (future_gap <= 0.0 || future_gap > odom_future_wait_sec_) {
      return false;
    }
    PendingOdomCandidate item;
    item.target = target;
    item.lookup_time = lookup_time;
    item.map_to_localized = map_to_localized;
    item.queued_at = now();
    pending_odom_candidates_.push_back(item);
    while (pending_odom_candidates_.size() > 10) {
      pending_odom_candidates_.pop_front();
    }
    publish_status(
      "PENDING wait_odom_cache gap=" + fixed3(future_gap) + "s max=" + fixed3(odom_future_wait_sec_) + "s");
    return true;
  }

  void process_pending_odom_candidates()
  {
    if (pending_odom_candidates_.empty() || odom_cache_.empty()) {
      return;
    }
    const auto now_time = now();
    const double latest = odom_cache_.back().stamp;
    std::deque<PendingOdomCandidate> remaining;
    while (!pending_odom_candidates_.empty()) {
      const auto item = pending_odom_candidates_.front();
      pending_odom_candidates_.pop_front();
      const double queued_age = (now_time - item.queued_at).seconds();
      if (queued_age > odom_future_wait_sec_ + 0.10) {
        publish_status("REJECTED odom_cache_pending_timeout age=" + fixed3(queued_age) + "s");
        continue;
      }
      if (latest + odom_lookup_tolerance_sec_ < item.target) {
        remaining.push_back(item);
        continue;
      }
      const auto odom_to_localized = lookup_odom_cache_matrix(item.lookup_time, false);
      if (!odom_to_localized) {
        if (last_odom_cache_failure_ == "future") {
          remaining.push_back(item);
        }
        continue;
      }
      Matrix4 candidate = item.map_to_localized * odom_to_localized->inverse();
      candidate = apply_output_constraints(candidate);
      evaluate_candidate(candidate);
    }
    pending_odom_candidates_ = remaining;
  }

  void handle_candidate_pose(const std_msgs::msg::Header & header, const geometry_msgs::msg::Pose & pose)
  {
    if (!header.frame_id.empty() && header.frame_id != map_frame_) {
      publish_status("REJECTED wrong_frame frame_id=" + header.frame_id + " expected=" + map_frame_);
      return;
    }
    if (!confidence_is_acceptable()) {
      return;
    }

    const bool zero_stamp = is_zero_stamp(header.stamp);
    rclcpp::Time lookup_time(0, 0, get_clock()->get_clock_type());
    if (zero_stamp && accept_zero_stamp_) {
      lookup_time = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    } else {
      lookup_time = rclcpp::Time(header.stamp, get_clock()->get_clock_type());
      const double pose_age = (now() - lookup_time).seconds();
      if (pose_age < -0.2 || pose_age > pose_timeout_sec_) {
        publish_status("REJECTED stale_pose age=" + fixed3(pose_age) + "s");
        return;
      }
    }

    const Matrix4 map_to_localized = pose_to_matrix(pose);
    const auto odom_to_localized = lookup_odom_to_localized_matrix(lookup_time, zero_stamp);
    if (!odom_to_localized) {
      if (!zero_stamp && last_odom_cache_failure_ == "future" && queue_pending_odom_candidate(lookup_time, map_to_localized)) {
        return;
      }
      return;
    }
    Matrix4 candidate = map_to_localized * odom_to_localized->inverse();
    candidate = apply_output_constraints(candidate);
    evaluate_candidate(candidate);
  }

  void handle_global_recovery_map_to_odom(const geometry_msgs::msg::PoseStamped & msg)
  {
    if (!msg.header.frame_id.empty() && msg.header.frame_id != map_frame_) {
      publish_status(
        "REJECTED global_recovery_wrong_frame frame_id=" + msg.header.frame_id +
        " expected=" + map_frame_);
      return;
    }
    if (!is_zero_stamp(msg.header.stamp)) {
      const rclcpp::Time stamp(msg.header.stamp, get_clock()->get_clock_type());
      const double age = (now() - stamp).seconds();
      if (age < -0.2 || age > global_recovery_timeout_sec_) {
        publish_status("REJECTED global_recovery_stale age=" + fixed3(age) + "s");
        return;
      }
    }
    Matrix4 candidate = apply_output_constraints(pose_to_matrix(msg.pose));
    pending_large_candidate_.reset();
    pending_large_count_ = 0;
    large_jump_hold_start_time_.reset();
    accept_candidate(candidate, "global_recovery");
  }

  bool confidence_is_acceptable()
  {
    if (!require_confidence_) {
      return true;
    }
    if (!latest_confidence_ || !latest_confidence_time_) {
      publish_status("REJECTED no_confidence");
      return false;
    }
    const double age = (now() - *latest_confidence_time_).seconds();
    if (age > confidence_timeout_sec_) {
      publish_status("REJECTED stale_confidence age=" + fixed3(age) + "s");
      return false;
    }
    if (*latest_confidence_ < min_confidence_) {
      publish_status(
        "REJECTED low_confidence value=" + fixed3(*latest_confidence_) + " min=" + fixed3(min_confidence_));
      return false;
    }
    return true;
  }

  Matrix4 apply_output_constraints(const Matrix4 & transform) const
  {
    if (!force_2d_) {
      return transform;
    }
    Matrix4 constrained = identity();
    const double yaw = yaw_from_matrix(transform);
    constrained(0, 0) = std::cos(yaw);
    constrained(0, 1) = -std::sin(yaw);
    constrained(1, 0) = std::sin(yaw);
    constrained(1, 1) = std::cos(yaw);
    constrained(0, 3) = transform(0, 3);
    constrained(1, 3) = transform(1, 3);
    constrained(2, 3) = force_z_;
    return constrained;
  }

  void evaluate_candidate(const Matrix4 & candidate)
  {
    if (spin_to_pose_guard_is_active()) {
      pending_large_candidate_.reset();
      pending_large_count_ = 0;
      large_jump_hold_start_time_.reset();
      publish_status("REJECTED spin_to_pose_freeze_tf phase=" + spin_guard_phase());
      return;
    }
    if (!accepted_map_to_odom_) {
      if (allow_initial_pose_) {
        accept_candidate(candidate, "initial_pose");
      } else {
        publish_status("REJECTED no_initial_pose");
      }
      return;
    }
    const auto [delta_xy, delta_yaw] = correction_delta(candidate, *accepted_map_to_odom_);
    if (jump_protection_mode_ == "protect") {
      evaluate_candidate_protected(candidate, delta_xy, delta_yaw);
      return;
    }
    if (jump_protection_mode_ == "monitor") {
      publish_jump_protection_monitor(delta_xy, delta_yaw);
    }
    evaluate_candidate_legacy(candidate, delta_xy, delta_yaw);
  }

  void evaluate_candidate_legacy(const Matrix4 & candidate, double delta_xy, double delta_yaw)
  {
    if (delta_xy <= max_small_correction_translation_ && delta_yaw <= max_small_correction_yaw_) {
      pending_large_candidate_.reset();
      pending_large_count_ = 0;
      large_jump_hold_start_time_.reset();
      accept_candidate(candidate, "small_correction dx=" + fixed3(delta_xy) + " yaw=" + fixed3(delta_yaw));
      return;
    }
    if (delta_xy > max_large_correction_translation_ || delta_yaw > max_large_correction_yaw_) {
      pending_large_candidate_.reset();
      pending_large_count_ = 0;
      large_jump_hold_start_time_.reset();
      publish_status("REJECTED too_large dx=" + fixed3(delta_xy) + " yaw=" + fixed3(delta_yaw));
      return;
    }
    if (!pending_large_candidate_) {
      pending_large_candidate_ = candidate;
      pending_large_count_ = 1;
      publish_status("PENDING large_correction count=1 dx=" + fixed3(delta_xy) + " yaw=" + fixed3(delta_yaw));
      return;
    }
    const auto [consistent_xy, consistent_yaw] = correction_delta(candidate, *pending_large_candidate_);
    if (consistent_xy <= consistency_translation_tolerance_ && consistent_yaw <= consistency_yaw_tolerance_) {
      pending_large_count_++;
      pending_large_candidate_ = candidate;
    } else {
      pending_large_candidate_ = candidate;
      pending_large_count_ = 1;
      publish_status(
        "PENDING reset_large_candidate spread_xy=" + fixed3(consistent_xy) +
        " spread_yaw=" + fixed3(consistent_yaw));
      return;
    }
    if (pending_large_count_ >= required_consistent_frames_) {
      accept_candidate(candidate, "confirmed_large_correction count=" + std::to_string(pending_large_count_));
      pending_large_candidate_.reset();
      pending_large_count_ = 0;
      large_jump_hold_start_time_.reset();
    } else {
      publish_status(
        "PENDING large_correction count=" + std::to_string(pending_large_count_) + "/" +
        std::to_string(required_consistent_frames_));
    }
  }

  void evaluate_candidate_protected(const Matrix4 & candidate, double delta_xy, double delta_yaw)
  {
    if (delta_xy <= max_small_correction_translation_ && delta_yaw <= max_small_correction_yaw_) {
      pending_large_candidate_.reset();
      pending_large_count_ = 0;
      large_jump_hold_start_time_.reset();
      accept_candidate(candidate, "small_correction dx=" + fixed3(delta_xy) + " yaw=" + fixed3(delta_yaw));
      return;
    }
    if (delta_xy > max_large_correction_translation_ || delta_yaw > max_large_correction_yaw_) {
      pending_large_candidate_.reset();
      pending_large_count_ = 0;
      update_large_jump_hold("too_large dx=" + fixed3(delta_xy) + " yaw=" + fixed3(delta_yaw));
      return;
    }
    if (navigation_active_) {
      if (delta_xy <= nav_medium_correction_translation_ && delta_yaw <= nav_medium_correction_yaw_) {
        if (update_pending_candidate(candidate, nav_medium_required_frames_, "nav_medium_correction", delta_xy, delta_yaw)) {
          accept_candidate(candidate, "confirmed_nav_medium_correction count=" + std::to_string(pending_large_count_));
          pending_large_candidate_.reset();
          pending_large_count_ = 0;
          large_jump_hold_start_time_.reset();
        }
        return;
      }
      if (allow_nav_large_jump_) {
        if (update_pending_candidate(candidate, required_consistent_frames_, "nav_large_correction", delta_xy, delta_yaw)) {
          accept_candidate(candidate, "confirmed_nav_large_correction count=" + std::to_string(pending_large_count_));
          pending_large_candidate_.reset();
          pending_large_count_ = 0;
          large_jump_hold_start_time_.reset();
        }
        return;
      }
      pending_large_candidate_.reset();
      pending_large_count_ = 0;
      update_large_jump_hold("nav_large_correction dx=" + fixed3(delta_xy) + " yaw=" + fixed3(delta_yaw));
      return;
    }
    if (delta_xy > hard_reject_translation_ || delta_yaw > hard_reject_yaw_) {
      pending_large_candidate_.reset();
      pending_large_count_ = 0;
      update_large_jump_hold("idle_hard_reject dx=" + fixed3(delta_xy) + " yaw=" + fixed3(delta_yaw));
      return;
    }
    if (
      allow_idle_large_jump_ && delta_xy <= idle_large_correction_translation_ &&
      delta_yaw <= idle_large_correction_yaw_)
    {
      if (update_pending_candidate(candidate, idle_large_required_frames_, "idle_large_correction", delta_xy, delta_yaw)) {
        accept_candidate(candidate, "confirmed_idle_large_correction count=" + std::to_string(pending_large_count_));
        pending_large_candidate_.reset();
        pending_large_count_ = 0;
        large_jump_hold_start_time_.reset();
      }
      return;
    }
    pending_large_candidate_.reset();
    pending_large_count_ = 0;
    update_large_jump_hold("idle_large_not_allowed dx=" + fixed3(delta_xy) + " yaw=" + fixed3(delta_yaw));
  }

  bool update_pending_candidate(
    const Matrix4 & candidate, int required_frames, const std::string & label, double delta_xy, double delta_yaw)
  {
    large_jump_hold_start_time_.reset();
    required_frames = std::max(1, required_frames);
    if (!pending_large_candidate_) {
      pending_large_candidate_ = candidate;
      pending_large_count_ = 1;
      publish_status(
        "PENDING " + label + " count=1/" + std::to_string(required_frames) +
        " dx=" + fixed3(delta_xy) + " yaw=" + fixed3(delta_yaw));
      return pending_large_count_ >= required_frames;
    }
    const auto [consistent_xy, consistent_yaw] = correction_delta(candidate, *pending_large_candidate_);
    if (consistent_xy <= consistency_translation_tolerance_ && consistent_yaw <= consistency_yaw_tolerance_) {
      pending_large_count_++;
      pending_large_candidate_ = candidate;
    } else {
      pending_large_candidate_ = candidate;
      pending_large_count_ = 1;
      publish_status(
        "PENDING reset_" + label + " spread_xy=" + fixed3(consistent_xy) +
        " spread_yaw=" + fixed3(consistent_yaw));
      return false;
    }
    if (pending_large_count_ >= required_frames) {
      return true;
    }
    publish_status(
      "PENDING " + label + " count=" + std::to_string(pending_large_count_) + "/" +
      std::to_string(required_frames) + " dx=" + fixed3(delta_xy) + " yaw=" + fixed3(delta_yaw));
    return false;
  }

  void update_large_jump_hold(const std::string & reason)
  {
    const auto now_time = now();
    if (!large_jump_hold_start_time_) {
      large_jump_hold_start_time_ = now_time;
      large_jump_hold_reason_ = reason;
      publish_status("HOLD large_jump " + reason);
      return;
    }
    const double hold_age = (now_time - *large_jump_hold_start_time_).seconds();
    if (hold_age >= large_jump_degraded_after_sec_) {
      publish_status("DEGRADED large_jump_hold age=" + fixed2(hold_age) + "s reason=" + large_jump_hold_reason_);
    } else {
      publish_status("HOLD large_jump age=" + fixed2(hold_age) + "s reason=" + large_jump_hold_reason_);
    }
  }

  void publish_jump_protection_monitor(double delta_xy, double delta_yaw)
  {
    if (delta_xy <= max_small_correction_translation_ && delta_yaw <= max_small_correction_yaw_) {
      return;
    }
    if (navigation_active_) {
      if (delta_xy <= nav_medium_correction_translation_ && delta_yaw <= nav_medium_correction_yaw_) {
        publish_status("WOULD_PENDING nav_medium_correction dx=" + fixed3(delta_xy) + " yaw=" + fixed3(delta_yaw));
      } else {
        publish_status("WOULD_HOLD nav_large_correction dx=" + fixed3(delta_xy) + " yaw=" + fixed3(delta_yaw));
      }
      return;
    }
    if (delta_xy > hard_reject_translation_ || delta_yaw > hard_reject_yaw_) {
      publish_status("WOULD_HOLD idle_hard_reject dx=" + fixed3(delta_xy) + " yaw=" + fixed3(delta_yaw));
    } else {
      publish_status("WOULD_PENDING idle_large_correction dx=" + fixed3(delta_xy) + " yaw=" + fixed3(delta_yaw));
    }
  }

  void enter_spin_guard(const rclcpp::Time & now_time, const std::string & source)
  {
    if (!spin_guard_active_) {
      spin_guard_start_time_ = now_time;
      publish_status("SPIN_GUARD entered source=" + source);
    }
    spin_guard_active_ = true;
    spin_guard_source_ = source;
    spin_guard_settle_until_ =
      now_time + rclcpp::Duration::from_seconds(spin_to_pose_guard_settle_sec_);
  }

  bool spin_to_pose_guard_is_active()
  {
    if (!enable_spin_to_pose_guard_ || !spin_guard_active_) {
      return false;
    }
    const auto now_time = now();
    if (spin_guard_start_time_) {
      const double age = (now_time - *spin_guard_start_time_).seconds();
      if (age < 0.0 || age > spin_to_pose_guard_max_duration_sec_) {
        spin_guard_active_ = false;
        spin_guard_turning_ = false;
        spin_guard_source_.clear();
        publish_status("SPIN_GUARD expired age=" + fixed2(age) + "s");
        return false;
      }
    }
    if (spin_guard_settle_until_ && now_time > *spin_guard_settle_until_) {
      spin_guard_active_ = false;
      spin_guard_turning_ = false;
      spin_guard_source_.clear();
      publish_status("SPIN_GUARD settled");
      return false;
    }
    return true;
  }

  std::string spin_guard_phase() const
  {
    return spin_guard_turning_ ? "turning" : "settle";
  }

  void accept_candidate(const Matrix4 & candidate, const std::string & reason)
  {
    accepted_map_to_odom_ = candidate;
    last_accept_time_ = now();
    const double xy = std::hypot(candidate(0, 3), candidate(1, 3));
    const double yaw = yaw_from_matrix(candidate);
    std::string status_reason = reason;
    if (reason == "initial_pose" && xy <= origin_seed_radius_) {
      status_reason += " origin_seeded=true origin_seed_radius=" + fixed3(origin_seed_radius_);
    }
    publish_status("ACCEPTED " + status_reason + " map_odom_xy_norm=" + fixed3(xy) + " yaw=" + fixed3(yaw));
  }

  void publish_last_tf()
  {
    if (!accepted_map_to_odom_) {
      publish_status("WAITING no_accepted_map_to_odom");
      return;
    }
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = map_frame_;
    msg.child_frame_id = odom_frame_;
    msg.transform.translation.x = (*accepted_map_to_odom_)(0, 3);
    msg.transform.translation.y = (*accepted_map_to_odom_)(1, 3);
    msg.transform.translation.z = (*accepted_map_to_odom_)(2, 3);
    Eigen::Matrix3d rotation = accepted_map_to_odom_->block<3, 3>(0, 0);
    Eigen::Quaterniond q(rotation);
    q.normalize();
    msg.transform.rotation.x = q.x();
    msg.transform.rotation.y = q.y();
    msg.transform.rotation.z = q.z();
    msg.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(msg);
  }

  void publish_status(const std::string & text)
  {
    std_msgs::msg::String msg;
    msg.data = text;
    status_pub_->publish(msg);
    if (text.rfind("REJECTED", 0) == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "%s", text.c_str());
    } else if (text.rfind("ACCEPTED small_correction", 0) == 0) {
      RCLCPP_DEBUG(get_logger(), "%s", text.c_str());
    } else if (text.rfind("ACCEPTED", 0) == 0) {
      RCLCPP_INFO(get_logger(), "%s", text.c_str());
    } else {
      RCLCPP_DEBUG(get_logger(), "%s", text.c_str());
    }
  }

  static std::string to_upper(std::string text)
  {
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    return text;
  }

  static std::string to_lower(std::string text)
  {
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return text;
  }

  std::string map_frame_;
  std::string odom_frame_;
  std::string localized_frame_;
  std::string prior_pose_topic_;
  std::string prior_pose_with_covariance_topic_;
  std::string prior_odom_topic_;
  std::string global_recovery_map_to_odom_topic_;
  std::string confidence_topic_;
  std::string odom_cache_topic_;
  std::string jump_protection_mode_;
  std::string navigation_status_topic_;

  bool require_confidence_{false};
  bool accept_zero_stamp_{true};
  bool allow_initial_pose_{true};
  bool use_odom_cache_{true};
  bool fallback_to_tf_lookup_{true};
  bool allow_nav_large_jump_{false};
  bool allow_idle_large_jump_{true};
  bool enable_spin_to_pose_guard_{true};
  bool force_2d_{true};
  bool navigation_active_{false};
  bool spin_guard_active_{false};
  bool spin_guard_turning_{false};

  double min_confidence_{0.5};
  double confidence_timeout_sec_{1.5};
  double publish_rate_{30.0};
  double tf_lookup_timeout_sec_{0.08};
  double pose_timeout_sec_{0.8};
  double global_recovery_timeout_sec_{2.5};
  double origin_seed_radius_{0.30};
  double odom_cache_duration_sec_{5.0};
  double odom_interpolation_max_gap_sec_{0.25};
  double odom_lookup_tolerance_sec_{0.03};
  double odom_future_wait_sec_{0.20};
  double max_small_correction_translation_{0.25};
  double max_small_correction_yaw_{0.12};
  double max_large_correction_translation_{3.0};
  double max_large_correction_yaw_{1.2};
  double consistency_translation_tolerance_{0.25};
  double consistency_yaw_tolerance_{0.10};
  double nav_medium_correction_translation_{0.50};
  double nav_medium_correction_yaw_{0.20};
  double nav_large_correction_translation_{0.50};
  double nav_large_correction_yaw_{0.20};
  double idle_large_correction_translation_{1.00};
  double idle_large_correction_yaw_{0.35};
  double hard_reject_translation_{1.00};
  double hard_reject_yaw_{0.50};
  double large_jump_degraded_after_sec_{3.0};
  double spin_to_pose_guard_settle_sec_{1.2};
  double spin_to_pose_guard_max_duration_sec_{8.0};
  double force_z_{0.0};

  int required_consistent_frames_{5};
  int nav_medium_required_frames_{5};
  int idle_large_required_frames_{5};
  int pending_large_count_{0};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr global_recovery_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_cache_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr confidence_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr navigation_status_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::optional<Matrix4> accepted_map_to_odom_;
  std::optional<Matrix4> pending_large_candidate_;
  std::optional<double> latest_confidence_;
  std::optional<rclcpp::Time> latest_confidence_time_;
  std::optional<rclcpp::Time> last_accept_time_;
  std::optional<rclcpp::Time> large_jump_hold_start_time_;
  std::optional<rclcpp::Time> spin_guard_start_time_;
  std::optional<rclcpp::Time> spin_guard_settle_until_;
  std::string large_jump_hold_reason_;
  std::string spin_guard_source_;
  std::string last_odom_cache_failure_;
  std::deque<OdomEntry> odom_cache_;
  std::deque<PendingOdomCandidate> pending_odom_candidates_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PriorMapOdomBridgeCpp>());
  rclcpp::shutdown();
  return 0;
}
