#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{
struct Point
{
  float x{};
  float y{};
  float z{};
};

struct Bounds
{
  bool valid{false};
  float min_x{0.0F};
  float max_x{0.0F};
  float min_y{0.0F};
  float max_y{0.0F};
  float min_z{0.0F};
  float max_z{0.0F};
};

struct VoxelKey
{
  int x{};
  int y{};
  int z{};

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const
  {
    const auto hx = std::hash<int>{}(key.x * 73856093);
    const auto hy = std::hash<int>{}(key.y * 19349663);
    const auto hz = std::hash<int>{}(key.z * 83492791);
    return hx ^ (hy << 1U) ^ (hz << 2U);
  }
};

std::string fixed(double value, int precision = 3)
{
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out.precision(precision);
  out << value;
  return out.str();
}

Eigen::Matrix3f rpy_to_matrix(float roll, float pitch, float yaw)
{
  const Eigen::AngleAxisf rx(roll, Eigen::Vector3f::UnitX());
  const Eigen::AngleAxisf ry(pitch, Eigen::Vector3f::UnitY());
  const Eigen::AngleAxisf rz(yaw, Eigen::Vector3f::UnitZ());
  return (rz * ry * rx).toRotationMatrix();
}

Eigen::Vector3f normalize_or(const Eigen::Vector3f & vector, const Eigen::Vector3f & fallback)
{
  const float norm = vector.norm();
  if (norm < 1e-6F) {
    return fallback;
  }
  return vector / norm;
}

Eigen::Matrix3f rotation_between_vectors(Eigen::Vector3f source, Eigen::Vector3f target)
{
  source = normalize_or(source, Eigen::Vector3f::UnitZ());
  target = normalize_or(target, Eigen::Vector3f::UnitZ());
  const float dot = std::clamp(source.dot(target), -1.0F, 1.0F);
  if (dot > 0.9999F) {
    return Eigen::Matrix3f::Identity();
  }
  if (dot < -0.9999F) {
    Eigen::Vector3f axis = source.cross(Eigen::Vector3f::UnitX());
    if (axis.norm() < 1e-6F) {
      axis = source.cross(Eigen::Vector3f::UnitY());
    }
    axis.normalize();
    return Eigen::AngleAxisf(static_cast<float>(M_PI), axis).toRotationMatrix();
  }
  Eigen::Vector3f axis = source.cross(target);
  axis.normalize();
  const float angle = std::acos(dot);
  return Eigen::AngleAxisf(angle, axis).toRotationMatrix();
}

Bounds compute_bounds(const std::vector<Point> & points)
{
  Bounds bounds;
  if (points.empty()) {
    return bounds;
  }
  bounds.valid = true;
  bounds.min_x = bounds.max_x = points.front().x;
  bounds.min_y = bounds.max_y = points.front().y;
  bounds.min_z = bounds.max_z = points.front().z;
  for (const auto & p : points) {
    bounds.min_x = std::min(bounds.min_x, p.x);
    bounds.max_x = std::max(bounds.max_x, p.x);
    bounds.min_y = std::min(bounds.min_y, p.y);
    bounds.max_y = std::max(bounds.max_y, p.y);
    bounds.min_z = std::min(bounds.min_z, p.z);
    bounds.max_z = std::max(bounds.max_z, p.z);
  }
  return bounds;
}

std::string bounds_json(const Bounds & bounds)
{
  if (!bounds.valid) {
    return "null";
  }
  std::ostringstream out;
  out << "{\"min_x\":" << fixed(bounds.min_x) << ",\"max_x\":" << fixed(bounds.max_x)
      << ",\"min_y\":" << fixed(bounds.min_y) << ",\"max_y\":" << fixed(bounds.max_y)
      << ",\"min_z\":" << fixed(bounds.min_z) << ",\"max_z\":" << fixed(bounds.max_z) << "}";
  return out.str();
}
}  // namespace

class RoiObstacleDetectorCpp : public rclcpp::Node
{
public:
  RoiObstacleDetectorCpp()
  : Node("roi_obstacle_detector"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameters();
    load_parameters();

    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, sensor_qos,
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { cloud_callback(*msg); });

    if (enable_imu_leveling_ || transform_mode_ == "imu") {
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, rclcpp::QoS(50),
        [this](sensor_msgs::msg::Imu::SharedPtr msg) { imu_callback(*msg); });
    }

    has_obstacle_pub_ = create_publisher<std_msgs::msg::Bool>(has_obstacle_topic_, 10);
    debug_pub_ = create_publisher<std_msgs::msg::String>(debug_topic_, 10);
    roi_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(roi_cloud_topic_, sensor_qos);

    RCLCPP_INFO(
      get_logger(),
      "roi_obstacle_detector_cpp started: input=%s target_frame=%s transform_mode=%s publish_roi_cloud=%s",
      input_topic_.c_str(), target_frame_.c_str(), transform_mode_.c_str(),
      publish_roi_cloud_ ? "true" : "false");
  }

private:
  void declare_parameters()
  {
    declare_parameter<std::string>("input_topic", "/airy_points");
    declare_parameter<std::string>("target_frame", "base_footprint");
    declare_parameter<std::string>("source_frame_override", "");
    declare_parameter<std::string>("transform_mode", "tf");
    declare_parameter<bool>("fallback_to_manual", false);
    declare_parameter<double>("tf_timeout_sec", 0.05);
    declare_parameter<std::vector<double>>("manual_translation_xyz", {0.0, 0.0, 0.0});
    declare_parameter<std::vector<double>>("manual_rotation_rpy", {0.0, 0.0, 0.0});
    declare_parameter<std::vector<double>>("manual_rotation_matrix", std::vector<double>{});
    declare_parameter<bool>("enable_imu_leveling", false);
    declare_parameter<std::string>("imu_topic", "/airy_imu");
    declare_parameter<std::string>("imu_mode", "accel");
    declare_parameter<std::string>("imu_filter_mode", "lowpass");
    declare_parameter<std::vector<double>>("source_to_imu_rotation_matrix", std::vector<double>{});
    declare_parameter<double>("imu_timeout_sec", 0.3);
    declare_parameter<double>("imu_accel_alpha", 0.05);
    declare_parameter<bool>("imu_accel_is_up", true);
    declare_parameter<double>("imu_expected_accel_norm", 9.80665);
    declare_parameter<double>("imu_accel_gate_sigma", 1.5);
    declare_parameter<double>("imu_max_gyro_dt_sec", 0.05);
    declare_parameter<std::vector<double>>("imu_gyro_bias_xyz", {0.0, 0.0, 0.0});
    declare_parameter<bool>("estimate_gyro_bias", false);
    declare_parameter<double>("gyro_bias_estimation_sec", 2.0);
    declare_parameter<int>("gyro_bias_min_samples", 100);
    declare_parameter<std::vector<double>>("level_up_axis", {0.0, 0.0, 1.0});
    declare_parameter<std::vector<double>>("level_reference_axis_source", std::vector<double>{});

    declare_parameter<bool>("enable_ground_z_compensation", true);
    declare_parameter<double>("ground_roi_min_x", 0.25);
    declare_parameter<double>("ground_roi_max_x", 3.0);
    declare_parameter<double>("ground_roi_min_y", -1.2);
    declare_parameter<double>("ground_roi_max_y", 1.2);
    declare_parameter<double>("ground_roi_min_z", -0.45);
    declare_parameter<double>("ground_roi_max_z", 0.35);
    declare_parameter<double>("ground_z_percentile", 35.0);
    declare_parameter<int>("ground_min_points", 200);
    declare_parameter<double>("ground_filter_alpha", 0.35);
    declare_parameter<double>("ground_max_step", 0.08);
    declare_parameter<double>("ground_z_offset", 0.0);

    declare_parameter<double>("roi_min_x", 0.25);
    declare_parameter<double>("roi_max_x", 2.0);
    declare_parameter<double>("roi_min_y", -0.45);
    declare_parameter<double>("roi_max_y", 0.45);
    declare_parameter<double>("roi_min_z", 0.05);
    declare_parameter<double>("roi_max_z", 1.2);
    declare_parameter<double>("voxel_leaf_size", 0.05);
    declare_parameter<int>("min_points", 10);
    declare_parameter<bool>("use_clustering", true);
    declare_parameter<int>("cluster_min_size", 10);
    declare_parameter<int>("cluster_connectivity", 26);
    declare_parameter<int>("trigger_frames", 2);
    declare_parameter<int>("clear_frames", 4);

    declare_parameter<std::string>("has_obstacle_topic", "/front_obstacle/has_obstacle");
    declare_parameter<std::string>("debug_topic", "/front_obstacle/debug");
    declare_parameter<std::string>("roi_cloud_topic", "/front_obstacle/roi_cloud");
    declare_parameter<bool>("publish_roi_cloud", true);
    declare_parameter<bool>("publish_debug", true);
    declare_parameter<double>("debug_publish_period_sec", 0.5);
    declare_parameter<bool>("log_input_stats", true);
  }

  void load_parameters()
  {
    input_topic_ = get_parameter("input_topic").as_string();
    target_frame_ = get_parameter("target_frame").as_string();
    source_frame_override_ = get_parameter("source_frame_override").as_string();
    transform_mode_ = get_parameter("transform_mode").as_string();
    std::transform(transform_mode_.begin(), transform_mode_.end(), transform_mode_.begin(), ::tolower);
    fallback_to_manual_ = get_parameter("fallback_to_manual").as_bool();
    tf_timeout_sec_ = get_parameter("tf_timeout_sec").as_double();
    manual_translation_ = vector3_param("manual_translation_xyz", Eigen::Vector3f::Zero());
    manual_rotation_ = load_manual_rotation();
    enable_imu_leveling_ = get_parameter("enable_imu_leveling").as_bool();
    imu_topic_ = get_parameter("imu_topic").as_string();
    imu_mode_ = get_parameter("imu_mode").as_string();
    imu_filter_mode_ = get_parameter("imu_filter_mode").as_string();
    source_to_imu_rotation_ = matrix3_param("source_to_imu_rotation_matrix", Eigen::Matrix3f::Identity());
    imu_to_source_rotation_ = source_to_imu_rotation_.transpose();
    imu_timeout_sec_ = get_parameter("imu_timeout_sec").as_double();
    imu_accel_alpha_ = get_parameter("imu_accel_alpha").as_double();
    imu_accel_is_up_ = get_parameter("imu_accel_is_up").as_bool();
    imu_expected_accel_norm_ = get_parameter("imu_expected_accel_norm").as_double();
    imu_accel_gate_sigma_ = get_parameter("imu_accel_gate_sigma").as_double();
    imu_max_gyro_dt_sec_ = get_parameter("imu_max_gyro_dt_sec").as_double();
    configured_gyro_bias_ = vector3_param("imu_gyro_bias_xyz", Eigen::Vector3f::Zero());
    estimate_gyro_bias_ = get_parameter("estimate_gyro_bias").as_bool();
    gyro_bias_estimation_sec_ = get_parameter("gyro_bias_estimation_sec").as_double();
    gyro_bias_min_samples_ = get_parameter("gyro_bias_min_samples").as_int();
    level_up_axis_ = normalize_or(vector3_param("level_up_axis", Eigen::Vector3f::UnitZ()), Eigen::Vector3f::UnitZ());
    level_reference_axis_source_ = load_level_reference_axis_source();

    enable_ground_z_compensation_ = get_parameter("enable_ground_z_compensation").as_bool();
    ground_roi_ = {
      static_cast<float>(get_parameter("ground_roi_min_x").as_double()),
      static_cast<float>(get_parameter("ground_roi_max_x").as_double()),
      static_cast<float>(get_parameter("ground_roi_min_y").as_double()),
      static_cast<float>(get_parameter("ground_roi_max_y").as_double()),
      static_cast<float>(get_parameter("ground_roi_min_z").as_double()),
      static_cast<float>(get_parameter("ground_roi_max_z").as_double())};
    ground_z_percentile_ = get_parameter("ground_z_percentile").as_double();
    ground_min_points_ = get_parameter("ground_min_points").as_int();
    ground_filter_alpha_ = get_parameter("ground_filter_alpha").as_double();
    ground_max_step_ = get_parameter("ground_max_step").as_double();
    ground_z_offset_ = get_parameter("ground_z_offset").as_double();

    roi_ = {
      static_cast<float>(get_parameter("roi_min_x").as_double()),
      static_cast<float>(get_parameter("roi_max_x").as_double()),
      static_cast<float>(get_parameter("roi_min_y").as_double()),
      static_cast<float>(get_parameter("roi_max_y").as_double()),
      static_cast<float>(get_parameter("roi_min_z").as_double()),
      static_cast<float>(get_parameter("roi_max_z").as_double())};
    voxel_leaf_size_ = get_parameter("voxel_leaf_size").as_double();
    min_points_ = get_parameter("min_points").as_int();
    use_clustering_ = get_parameter("use_clustering").as_bool();
    cluster_min_size_ = get_parameter("cluster_min_size").as_int();
    cluster_connectivity_ = get_parameter("cluster_connectivity").as_int();
    trigger_frames_ = std::max(1, static_cast<int>(get_parameter("trigger_frames").as_int()));
    clear_frames_ = std::max(1, static_cast<int>(get_parameter("clear_frames").as_int()));
    has_obstacle_topic_ = get_parameter("has_obstacle_topic").as_string();
    debug_topic_ = get_parameter("debug_topic").as_string();
    roi_cloud_topic_ = get_parameter("roi_cloud_topic").as_string();
    publish_roi_cloud_ = get_parameter("publish_roi_cloud").as_bool();
    publish_debug_ = get_parameter("publish_debug").as_bool();
    debug_publish_period_sec_ = get_parameter("debug_publish_period_sec").as_double();
    log_input_stats_ = get_parameter("log_input_stats").as_bool();
  }

  Eigen::Vector3f vector3_param(const std::string & name, const Eigen::Vector3f & fallback)
  {
    const auto values = get_parameter(name).as_double_array();
    if (values.size() != 3U) {
      return fallback;
    }
    return Eigen::Vector3f(values[0], values[1], values[2]);
  }

  Eigen::Matrix3f matrix3_param(const std::string & name, const Eigen::Matrix3f & fallback)
  {
    const auto values = get_parameter(name).as_double_array();
    if (values.size() != 9U) {
      return fallback;
    }
    Eigen::Matrix3f matrix;
    matrix << static_cast<float>(values[0]), static_cast<float>(values[1]), static_cast<float>(values[2]),
      static_cast<float>(values[3]), static_cast<float>(values[4]), static_cast<float>(values[5]),
      static_cast<float>(values[6]), static_cast<float>(values[7]), static_cast<float>(values[8]);
    return matrix;
  }

  Eigen::Matrix3f load_manual_rotation()
  {
    const auto matrix_values = get_parameter("manual_rotation_matrix").as_double_array();
    if (matrix_values.size() == 9U) {
      return matrix3_param("manual_rotation_matrix", Eigen::Matrix3f::Identity());
    }
    const auto rpy = get_parameter("manual_rotation_rpy").as_double_array();
    if (rpy.size() == 3U) {
      return rpy_to_matrix(static_cast<float>(rpy[0]), static_cast<float>(rpy[1]), static_cast<float>(rpy[2]));
    }
    return Eigen::Matrix3f::Identity();
  }

  Eigen::Vector3f load_level_reference_axis_source()
  {
    const auto configured = get_parameter("level_reference_axis_source").as_double_array();
    if (configured.size() == 3U) {
      return normalize_or(Eigen::Vector3f(configured[0], configured[1], configured[2]), Eigen::Vector3f::UnitZ());
    }
    return normalize_or(manual_rotation_.transpose() * level_up_axis_, Eigen::Vector3f::UnitZ());
  }

  void imu_callback(const sensor_msgs::msg::Imu & msg)
  {
    latest_imu_time_ = rclcpp::Time(msg.header.stamp, get_clock()->get_clock_type());
    latest_accel_ = Eigen::Vector3f(
      msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
    latest_orientation_ = Eigen::Quaternionf(
      msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
    if (latest_orientation_->norm() < 1e-6F) {
      latest_orientation_.reset();
    } else {
      latest_orientation_->normalize();
    }
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2 & msg)
  {
    const auto start = std::chrono::steady_clock::now();
    std::vector<Point> points = read_xyz(msg);
    const Bounds input_bounds = compute_bounds(points);
    if (log_input_stats_) {
      maybe_log_input_stats(points.size(), input_bounds, msg.header.frame_id);
    }

    std::string output_frame = target_frame_;
    Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();
    Eigen::Vector3f translation = Eigen::Vector3f::Zero();
    if (!get_transform(msg, rotation, translation, output_frame)) {
      publish_bool(false);
      maybe_publish_debug(points.size(), 0, 0, 0, false, output_frame, msg.header.frame_id, input_bounds,
        Bounds{}, Bounds{}, Bounds{}, 0.0, 0, elapsed_ms(start));
      return;
    }

    transform_points(points, rotation, translation);
    Bounds pre_ground_bounds = compute_bounds(points);
    double ground_z = 0.0;
    int ground_candidates = 0;
    if (enable_ground_z_compensation_) {
      apply_ground_z_compensation(points, ground_z, ground_candidates);
    }

    Bounds target_bounds = compute_bounds(points);
    std::vector<Point> roi_points = crop(points, roi_);
    const auto raw_roi_count = roi_points.size();
    auto voxel_points = voxel_downsample(roi_points);
    const auto voxel_count = voxel_points.size();
    int max_cluster_size = 0;
    bool detected_now = false;
    if (voxel_count >= static_cast<std::size_t>(min_points_)) {
      if (use_clustering_) {
        max_cluster_size = find_max_cluster_size(voxel_points);
        detected_now = max_cluster_size >= cluster_min_size_;
      } else {
        max_cluster_size = static_cast<int>(voxel_count);
        detected_now = true;
      }
    }

    const bool state = update_hysteresis(detected_now);
    publish_bool(state);
    if (publish_roi_cloud_) {
      publish_cloud(voxel_points, output_frame, msg.header.stamp);
    }
    maybe_publish_debug(points.size(), raw_roi_count, voxel_count, max_cluster_size, detected_now, output_frame,
      msg.header.frame_id, input_bounds, pre_ground_bounds, target_bounds, compute_bounds(voxel_points), ground_z,
      ground_candidates, elapsed_ms(start));
  }

  std::vector<Point> read_xyz(const sensor_msgs::msg::PointCloud2 & msg) const
  {
    std::vector<Point> points;
    points.reserve(static_cast<std::size_t>(msg.width) * static_cast<std::size_t>(msg.height));
    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        const float x = *iter_x;
        const float y = *iter_y;
        const float z = *iter_z;
        if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
          points.push_back({x, y, z});
        }
      }
    } catch (const std::runtime_error & exc) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "PointCloud2 xyz read failed: %s", exc.what());
    }
    return points;
  }

  bool get_transform(
    const sensor_msgs::msg::PointCloud2 & msg, Eigen::Matrix3f & rotation, Eigen::Vector3f & translation,
    std::string & output_frame)
  {
    if (transform_mode_ == "none") {
      output_frame = msg.header.frame_id.empty() ? target_frame_ : msg.header.frame_id;
      return true;
    }
    if (transform_mode_ == "manual" || transform_mode_ == "imu") {
      rotation = manual_rotation_;
      translation = manual_translation_;
      if (enable_imu_leveling_ || transform_mode_ == "imu") {
        rotation = manual_rotation_ * imu_level_rotation();
      }
      output_frame = target_frame_;
      return true;
    }
    if (transform_mode_ == "tf") {
      const std::string source_frame = source_frame_override_.empty() ? msg.header.frame_id : source_frame_override_;
      try {
        const auto tf = tf_buffer_.lookupTransform(
          target_frame_, source_frame, msg.header.stamp, tf2::durationFromSec(tf_timeout_sec_));
        const Eigen::Quaternionf q(
          tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z);
        rotation = q.normalized().toRotationMatrix();
        translation = Eigen::Vector3f(
          tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
        output_frame = target_frame_;
        return true;
      } catch (const tf2::TransformException & exc) {
        if (!fallback_to_manual_) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "ROI TF lookup failed: %s", exc.what());
          return false;
        }
        rotation = manual_rotation_;
        translation = manual_translation_;
        output_frame = target_frame_;
        return true;
      }
    }
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "unknown transform_mode=%s", transform_mode_.c_str());
    return false;
  }

  Eigen::Matrix3f imu_level_rotation()
  {
    if (!latest_imu_time_) {
      return Eigen::Matrix3f::Identity();
    }
    if ((now() - *latest_imu_time_).seconds() > imu_timeout_sec_) {
      return Eigen::Matrix3f::Identity();
    }
    if (imu_mode_ == "orientation" && latest_orientation_) {
      const Eigen::Vector3f up_in_imu = latest_orientation_->toRotationMatrix().transpose() * level_up_axis_;
      const Eigen::Vector3f up_in_source = imu_to_source_rotation_ * up_in_imu;
      return rotation_between_vectors(level_reference_axis_source_, up_in_source);
    }
    if (!latest_accel_) {
      return Eigen::Matrix3f::Identity();
    }
    const float accel_norm = latest_accel_->norm();
    const float max_error = static_cast<float>(imu_expected_accel_norm_ * imu_accel_gate_sigma_);
    if (std::abs(accel_norm - static_cast<float>(imu_expected_accel_norm_)) > max_error) {
      return Eigen::Matrix3f::Identity();
    }
    Eigen::Vector3f up_in_imu = normalize_or(*latest_accel_, Eigen::Vector3f::UnitZ());
    if (!imu_accel_is_up_) {
      up_in_imu = -up_in_imu;
    }
    Eigen::Vector3f up_in_source = imu_to_source_rotation_ * up_in_imu;
    if (!filtered_accel_up_) {
      filtered_accel_up_ = up_in_source;
    } else {
      const float alpha = std::clamp(static_cast<float>(imu_accel_alpha_), 0.0F, 1.0F);
      filtered_accel_up_ = normalize_or((1.0F - alpha) * *filtered_accel_up_ + alpha * up_in_source, up_in_source);
    }
    return rotation_between_vectors(level_reference_axis_source_, *filtered_accel_up_);
  }

  void transform_points(std::vector<Point> & points, const Eigen::Matrix3f & rotation, const Eigen::Vector3f & translation)
  {
    if (points.empty()) {
      return;
    }
    for (auto & p : points) {
      const Eigen::Vector3f transformed = rotation * Eigen::Vector3f(p.x, p.y, p.z) + translation;
      p.x = transformed.x();
      p.y = transformed.y();
      p.z = transformed.z();
    }
  }

  struct Roi
  {
    float min_x{};
    float max_x{};
    float min_y{};
    float max_y{};
    float min_z{};
    float max_z{};
  };

  std::vector<Point> crop(const std::vector<Point> & points, const Roi & roi) const
  {
    std::vector<Point> result;
    result.reserve(points.size());
    for (const auto & p : points) {
      if (
        p.x >= roi.min_x && p.x <= roi.max_x && p.y >= roi.min_y && p.y <= roi.max_y &&
        p.z >= roi.min_z && p.z <= roi.max_z)
      {
        result.push_back(p);
      }
    }
    return result;
  }

  void apply_ground_z_compensation(std::vector<Point> & points, double & ground_z, int & candidate_count)
  {
    std::vector<float> ground_values;
    ground_values.reserve(points.size());
    for (const auto & p : points) {
      if (
        p.x >= ground_roi_.min_x && p.x <= ground_roi_.max_x &&
        p.y >= ground_roi_.min_y && p.y <= ground_roi_.max_y &&
        p.z >= ground_roi_.min_z && p.z <= ground_roi_.max_z)
      {
        ground_values.push_back(p.z);
      }
    }
    candidate_count = static_cast<int>(ground_values.size());
    if (candidate_count < ground_min_points_) {
      return;
    }
    const double percentile = std::clamp(ground_z_percentile_, 0.0, 100.0) / 100.0;
    const auto index = static_cast<std::size_t>(percentile * static_cast<double>(ground_values.size() - 1));
    std::nth_element(ground_values.begin(), ground_values.begin() + index, ground_values.end());
    double measured = static_cast<double>(ground_values[index]) + ground_z_offset_;
    if (!filtered_ground_z_) {
      filtered_ground_z_ = measured;
    } else {
      const double delta = std::clamp(measured - *filtered_ground_z_, -ground_max_step_, ground_max_step_);
      filtered_ground_z_ = *filtered_ground_z_ + ground_filter_alpha_ * delta;
    }
    ground_z = *filtered_ground_z_;
    for (auto & p : points) {
      p.z = static_cast<float>(static_cast<double>(p.z) - ground_z);
    }
  }

  VoxelKey voxel_key(const Point & p) const
  {
    const double leaf = std::max(1e-6, voxel_leaf_size_);
    return {
      static_cast<int>(std::floor(static_cast<double>(p.x) / leaf)),
      static_cast<int>(std::floor(static_cast<double>(p.y) / leaf)),
      static_cast<int>(std::floor(static_cast<double>(p.z) / leaf))};
  }

  std::vector<Point> voxel_downsample(const std::vector<Point> & points) const
  {
    if (voxel_leaf_size_ <= 0.0 || points.empty()) {
      return points;
    }
    std::unordered_set<VoxelKey, VoxelKeyHash> seen;
    seen.reserve(points.size());
    std::vector<Point> result;
    result.reserve(points.size());
    for (const auto & p : points) {
      const auto key = voxel_key(p);
      if (seen.insert(key).second) {
        result.push_back(p);
      }
    }
    return result;
  }

  std::vector<VoxelKey> neighbor_offsets() const
  {
    std::vector<VoxelKey> offsets;
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          const int manhattan = std::abs(dx) + std::abs(dy) + std::abs(dz);
          if (cluster_connectivity_ == 6 && manhattan != 1) {
            continue;
          }
          if (cluster_connectivity_ == 18 && manhattan > 2) {
            continue;
          }
          offsets.push_back({dx, dy, dz});
        }
      }
    }
    return offsets;
  }

  int find_max_cluster_size(const std::vector<Point> & points) const
  {
    if (points.empty()) {
      return 0;
    }
    std::unordered_map<VoxelKey, int, VoxelKeyHash> voxel_counts;
    voxel_counts.reserve(points.size());
    for (const auto & p : points) {
      voxel_counts[voxel_key(p)]++;
    }
    std::unordered_set<VoxelKey, VoxelKeyHash> visited;
    visited.reserve(voxel_counts.size());
    const auto offsets = neighbor_offsets();
    int max_cluster = 0;
    for (const auto & item : voxel_counts) {
      if (visited.find(item.first) != visited.end()) {
        continue;
      }
      int cluster_size = 0;
      std::queue<VoxelKey> queue;
      queue.push(item.first);
      visited.insert(item.first);
      while (!queue.empty()) {
        const auto key = queue.front();
        queue.pop();
        cluster_size += voxel_counts.at(key);
        for (const auto & offset : offsets) {
          VoxelKey neighbor{key.x + offset.x, key.y + offset.y, key.z + offset.z};
          if (visited.find(neighbor) != visited.end()) {
            continue;
          }
          if (voxel_counts.find(neighbor) == voxel_counts.end()) {
            continue;
          }
          visited.insert(neighbor);
          queue.push(neighbor);
        }
      }
      max_cluster = std::max(max_cluster, cluster_size);
    }
    return max_cluster;
  }

  bool update_hysteresis(bool detected_now)
  {
    if (detected_now) {
      hit_streak_++;
      clear_streak_ = 0;
      if (hit_streak_ >= trigger_frames_) {
        obstacle_state_ = true;
      }
    } else {
      clear_streak_++;
      hit_streak_ = 0;
      if (clear_streak_ >= clear_frames_) {
        obstacle_state_ = false;
      }
    }
    return obstacle_state_;
  }

  void publish_bool(bool state)
  {
    std_msgs::msg::Bool msg;
    msg.data = state;
    has_obstacle_pub_->publish(msg);
  }

  void publish_cloud(
    const std::vector<Point> & points, const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = frame_id;
    cloud.header.stamp = stamp;
    cloud.height = 1;
    cloud.width = static_cast<std::uint32_t>(points.size());
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    for (const auto & p : points) {
      *iter_x = p.x;
      *iter_y = p.y;
      *iter_z = p.z;
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }
    roi_cloud_pub_->publish(cloud);
  }

  double elapsed_ms(const std::chrono::steady_clock::time_point & start) const
  {
    const auto elapsed = std::chrono::steady_clock::now() - start;
    return std::chrono::duration<double, std::milli>(elapsed).count();
  }

  void maybe_log_input_stats(std::size_t count, const Bounds & bounds, const std::string & frame_id)
  {
    const auto now_time = now();
    if (last_input_log_time_ && (now_time - *last_input_log_time_).seconds() < 2.0) {
      return;
    }
    last_input_log_time_ = now_time;
    RCLCPP_INFO(
      get_logger(), "input cloud frame=%s count=%zu bounds=%s", frame_id.c_str(), count,
      bounds_json(bounds).c_str());
  }

  void maybe_publish_debug(
    std::size_t input_count, std::size_t raw_roi_count, std::size_t voxel_count, int max_cluster_size,
    bool detected_now, const std::string & output_frame, const std::string & source_frame,
    const Bounds & input_bounds, const Bounds & pre_ground_bounds, const Bounds & target_bounds,
    const Bounds & roi_bounds, double ground_z, int ground_candidate_count, double elapsed_ms_value)
  {
    if (!publish_debug_) {
      return;
    }
    const auto now_time = now();
    if (last_debug_publish_time_ && (now_time - *last_debug_publish_time_).seconds() < debug_publish_period_sec_) {
      return;
    }
    last_debug_publish_time_ = now_time;
    std::ostringstream out;
    out << "{\"input_count\":" << input_count
        << ",\"raw_roi_count\":" << raw_roi_count
        << ",\"voxel_count\":" << voxel_count
        << ",\"max_cluster_size\":" << max_cluster_size
        << ",\"detected_now\":" << (detected_now ? "true" : "false")
        << ",\"state\":" << (obstacle_state_ ? "true" : "false")
        << ",\"hit_streak\":" << hit_streak_
        << ",\"clear_streak\":" << clear_streak_
        << ",\"output_frame\":\"" << output_frame
        << "\",\"source_frame\":\"" << source_frame
        << "\",\"input_bounds\":" << bounds_json(input_bounds)
        << ",\"pre_ground_bounds\":" << bounds_json(pre_ground_bounds)
        << ",\"target_bounds\":" << bounds_json(target_bounds)
        << ",\"roi_bounds\":" << bounds_json(roi_bounds)
        << ",\"ground_z\":" << fixed(ground_z)
        << ",\"ground_candidate_count\":" << ground_candidate_count
        << ",\"elapsed_ms\":" << fixed(elapsed_ms_value, 2) << "}";
    std_msgs::msg::String msg;
    msg.data = out.str();
    debug_pub_->publish(msg);
  }

  std::string input_topic_;
  std::string target_frame_;
  std::string source_frame_override_;
  std::string transform_mode_;
  std::string imu_topic_;
  std::string imu_mode_;
  std::string imu_filter_mode_;
  std::string has_obstacle_topic_;
  std::string debug_topic_;
  std::string roi_cloud_topic_;

  bool fallback_to_manual_{false};
  bool enable_imu_leveling_{false};
  bool imu_accel_is_up_{true};
  bool estimate_gyro_bias_{false};
  bool enable_ground_z_compensation_{true};
  bool use_clustering_{true};
  bool publish_roi_cloud_{true};
  bool publish_debug_{true};
  bool log_input_stats_{true};
  bool obstacle_state_{false};

  double tf_timeout_sec_{0.05};
  double imu_timeout_sec_{0.3};
  double imu_accel_alpha_{0.05};
  double imu_expected_accel_norm_{9.80665};
  double imu_accel_gate_sigma_{1.5};
  double imu_max_gyro_dt_sec_{0.05};
  double gyro_bias_estimation_sec_{2.0};
  double ground_z_percentile_{35.0};
  double ground_filter_alpha_{0.35};
  double ground_max_step_{0.08};
  double ground_z_offset_{0.0};
  double voxel_leaf_size_{0.05};
  double debug_publish_period_sec_{0.5};

  int gyro_bias_min_samples_{100};
  int ground_min_points_{200};
  int min_points_{10};
  int cluster_min_size_{10};
  int cluster_connectivity_{26};
  int trigger_frames_{2};
  int clear_frames_{4};
  int hit_streak_{0};
  int clear_streak_{0};

  Roi ground_roi_;
  Roi roi_;
  Eigen::Vector3f manual_translation_{Eigen::Vector3f::Zero()};
  Eigen::Vector3f configured_gyro_bias_{Eigen::Vector3f::Zero()};
  Eigen::Vector3f level_up_axis_{Eigen::Vector3f::UnitZ()};
  Eigen::Vector3f level_reference_axis_source_{Eigen::Vector3f::UnitZ()};
  Eigen::Matrix3f manual_rotation_{Eigen::Matrix3f::Identity()};
  Eigen::Matrix3f source_to_imu_rotation_{Eigen::Matrix3f::Identity()};
  Eigen::Matrix3f imu_to_source_rotation_{Eigen::Matrix3f::Identity()};
  std::optional<Eigen::Vector3f> latest_accel_;
  std::optional<Eigen::Vector3f> filtered_accel_up_;
  std::optional<Eigen::Quaternionf> latest_orientation_;
  std::optional<double> filtered_ground_z_;
  std::optional<rclcpp::Time> latest_imu_time_;
  std::optional<rclcpp::Time> last_debug_publish_time_;
  std::optional<rclcpp::Time> last_input_log_time_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr has_obstacle_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr roi_cloud_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoiObstacleDetectorCpp>());
  rclcpp::shutdown();
  return 0;
}
