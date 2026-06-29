/*
 * periodic_clearing_3d_publisher.cpp
 *
 * 文件作用：
 * 1. 根据输入点云节流触发，周期性生成一组 3D clearing points。
 * 2. 将 clearing cloud 发布给 Nav2 costmap 点云层，用于清理残留障碍。
 * 3. 通过角度、距离和高度层参数控制清障覆盖范围。
 *
 * 上游：点云滤波或雷达点云。
 * 下游：Nav2 local/global costmap 的 PointCloud2 observation/clearing layer。
 */

#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

class PeriodicClearing3DPublisher : public rclcpp::Node
{
public:
  PeriodicClearing3DPublisher()
  : Node("periodic_clearing_3d_publisher")
  {
    input_points_topic_ = declare_parameter<std::string>(
      "input_points_topic", "/fast_lio/cloud_registered");
    output_clearing_cloud_topic_ = declare_parameter<std::string>(
      "output_clearing_cloud_topic", "/clearing_cloud_3d");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_footprint");
    angle_min_deg_ = declare_parameter<double>("angle_min_deg", -180.0);
    angle_max_deg_ = declare_parameter<double>("angle_max_deg", 180.0);
    angle_increment_deg_ = declare_parameter<double>("angle_increment_deg", 1.0);
    clearing_range_min_ = declare_parameter<double>("clearing_range_min", 0.30);
    clearing_range_max_ = declare_parameter<double>("clearing_range_max", 5.0);
    clearing_range_step_ = declare_parameter<double>("clearing_range_step", 0.50);
    clearing_height_layers_ = declare_parameter<std::vector<double>>(
      "clearing_height_layers",
      std::vector<double>{0.05, 0.08, 0.10, 0.30, 0.50, 0.70, 0.90, 1.10, 1.30, 1.50, 1.70});
    max_publish_rate_hz_ = declare_parameter<double>("max_publish_rate_hz", 2.0);
    debug_ = declare_parameter<bool>("debug", false);
    qos_depth_ = declare_parameter<int>("qos_depth", 5);
    if (qos_depth_ <= 0) {
      qos_depth_ = 5;
    }

    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(qos_depth_)))
      .best_effort()
      .durability_volatile();

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_points_topic_, sensor_qos,
      std::bind(&PeriodicClearing3DPublisher::cloud_callback, this, std::placeholders::_1));
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_clearing_cloud_topic_, sensor_qos);

    build_clearing_cloud_template();

    RCLCPP_INFO(get_logger(), "periodic_clearing_3d_publisher started.");
    RCLCPP_INFO(get_logger(), "input_points_topic        : %s", input_points_topic_.c_str());
    RCLCPP_INFO(get_logger(), "output_clearing_cloud     : %s", output_clearing_cloud_topic_.c_str());
    RCLCPP_INFO(get_logger(), "target_frame              : %s", target_frame_.c_str());
    RCLCPP_INFO(get_logger(), "angle_min_deg             : %.1f", angle_min_deg_);
    RCLCPP_INFO(get_logger(), "angle_max_deg             : %.1f", angle_max_deg_);
    RCLCPP_INFO(get_logger(), "angle_increment_deg       : %.1f", angle_increment_deg_);
    RCLCPP_INFO(get_logger(), "bin_count                 : %zu", bin_count_);
    RCLCPP_INFO(
      get_logger(), "clearing_range            : %.2f ~ %.2f, step=%.2f",
      clearing_range_min_, clearing_range_max_, clearing_range_step_);
    RCLCPP_INFO(get_logger(), "max_publish_rate_hz       : %.1f", max_publish_rate_hz_);
    RCLCPP_INFO(get_logger(), "precomputed clearing_points : %u", cloud_template_.width);
  }

private:
  void build_clearing_cloud_template()
  {
    if (angle_increment_deg_ <= 0.0 || clearing_range_step_ <= 0.0) {
      throw std::runtime_error("angle_increment_deg and clearing_range_step must be > 0");
    }

    const double angle_min = angle_min_deg_ * M_PI / 180.0;
    const double angle_max = angle_max_deg_ * M_PI / 180.0;
    const double angle_increment = angle_increment_deg_ * M_PI / 180.0;
    bin_count_ = static_cast<size_t>(std::floor((angle_max - angle_min) / angle_increment)) + 1;
    if (bin_count_ == 0) {
      throw std::runtime_error("bin_count <= 0");
    }

    std::vector<float> points;
    for (size_t i = 0; i < bin_count_; ++i) {
      const double angle = angle_min + static_cast<double>(i) * angle_increment;
      const float cos_a = static_cast<float>(std::cos(angle));
      const float sin_a = static_cast<float>(std::sin(angle));
      for (double rr = clearing_range_min_; rr <= clearing_range_max_ + 1e-6; rr += clearing_range_step_) {
        const float x = static_cast<float>(rr) * cos_a;
        const float y = static_cast<float>(rr) * sin_a;
        for (double z : clearing_height_layers_) {
          points.push_back(x);
          points.push_back(y);
          points.push_back(static_cast<float>(z));
        }
      }
    }

    cloud_template_.header.frame_id = target_frame_;
    cloud_template_.height = 1;
    cloud_template_.width = static_cast<uint32_t>(points.size() / 3);
    cloud_template_.is_bigendian = false;
    cloud_template_.is_dense = true;
    cloud_template_.point_step = 12;
    cloud_template_.row_step = cloud_template_.point_step * cloud_template_.width;
    cloud_template_.fields.resize(3);
    set_field(cloud_template_.fields[0], "x", 0);
    set_field(cloud_template_.fields[1], "y", 4);
    set_field(cloud_template_.fields[2], "z", 8);
    cloud_template_.data.resize(points.size() * sizeof(float));
    std::memcpy(cloud_template_.data.data(), points.data(), cloud_template_.data.size());
  }

  static void set_field(sensor_msgs::msg::PointField & field, const std::string & name, uint32_t offset)
  {
    field.name = name;
    field.offset = offset;
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
  }

  bool should_publish_now()
  {
    if (max_publish_rate_hz_ <= 0.0) {
      return true;
    }
    const auto now_time = now();
    const auto min_dt = rclcpp::Duration::from_seconds(1.0 / max_publish_rate_hz_);
    if (last_publish_time_.nanoseconds() == 0 || now_time - last_publish_time_ >= min_dt) {
      last_publish_time_ = now_time;
      return true;
    }
    return false;
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr)
  {
    if (!should_publish_now()) {
      return;
    }
    if (cloud_template_.width == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "clearing_points 为空，跳过发布");
      return;
    }

    auto msg = cloud_template_;
    msg.header.stamp = now();
    pub_->publish(msg);
    if (debug_) {
      RCLCPP_INFO(get_logger(), "3D brute clearing published: clearing_points=%u", msg.width);
    }
  }

  std::string input_points_topic_;
  std::string output_clearing_cloud_topic_;
  std::string target_frame_;
  double angle_min_deg_{-180.0};
  double angle_max_deg_{180.0};
  double angle_increment_deg_{1.0};
  double clearing_range_min_{0.30};
  double clearing_range_max_{5.0};
  double clearing_range_step_{0.50};
  std::vector<double> clearing_height_layers_;
  double max_publish_rate_hz_{2.0};
  bool debug_{false};
  int qos_depth_{5};
  size_t bin_count_{0};
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};
  sensor_msgs::msg::PointCloud2 cloud_template_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PeriodicClearing3DPublisher>());
  rclcpp::shutdown();
  return 0;
}
