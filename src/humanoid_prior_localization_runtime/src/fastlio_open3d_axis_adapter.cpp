/*
 * fastlio_open3d_axis_adapter.cpp
 *
 * 文件作用：
 * 1. 将 Fast-LIO 原始 /odom 与 /fast_lio/cloud_registered 转换为 Open3D 先验地图定位使用的标准轴数据。
 * 2. 发布 /prior_localization/open3d_input_odom，作为 global_localization_node 的里程计初值。
 * 3. 发布 /prior_localization/open3d_input_cloud，作为 global_localization_node 的点云配准输入。
 * 4. 可发布 odom->prior_open3d_base TF，供 prior_map_odom_bridge_cpp 在 odom cache 不足时兜底查询。
 *
 * 上游节点：
 * - fast_lio_node：发布 /odom 和 /fast_lio/cloud_registered。
 *
 * 下游节点：
 * - global_localization_node：使用标准轴 odom/点云做 Open3D 配准。
 * - prior_map_odom_bridge_cpp：使用标准轴 odom cache 对齐定位时间戳。
 *
 * 注意：
 * - 这里只做坐标轴和点云字段转换，不做全局定位和滤波。
 * - initial_body_to_base_translation_raw 必须和 Fast-LIO/body 到导航 base 的外参保持一致。
 */

#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace
{

Eigen::Vector3d parameter_vector3(
  rclcpp::Node & node,
  const std::string & name,
  const std::vector<double> & default_value)
{
  const auto value = node.declare_parameter<std::vector<double>>(name, default_value);
  if (value.size() != 3) {
    RCLCPP_WARN(
      node.get_logger(),
      "parameter %s must contain exactly 3 values; using default [%.3f, %.3f, %.3f]",
      name.c_str(), default_value[0], default_value[1], default_value[2]);
    return Eigen::Vector3d(default_value[0], default_value[1], default_value[2]);
  }
  return Eigen::Vector3d(value[0], value[1], value[2]);
}

}  // namespace

class FastLioOpen3dAxisAdapter : public rclcpp::Node
{
public:
  explicit FastLioOpen3dAxisAdapter(const rclcpp::NodeOptions & options)
  : Node("fastlio_open3d_axis_adapter", options)
  {
    raw_odom_topic_ = declare_parameter<std::string>("raw_odom_topic", "/odom");
    raw_cloud_topic_ = declare_parameter<std::string>("raw_cloud_topic", "/fast_lio/cloud_registered");
    output_odom_topic_ = declare_parameter<std::string>(
      "output_odom_topic", "/prior_localization/open3d_input_odom");
    output_cloud_topic_ = declare_parameter<std::string>(
      "output_cloud_topic", "/prior_localization/open3d_input_cloud");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    output_base_frame_ = declare_parameter<std::string>("output_base_frame", "prior_open3d_base");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);

    // Kept for compatibility with older bag-test launch files. The current adapter does not use
    // map_origin_*; the initial_body_to_base_translation_raw parameter defines the cloud origin.
    declare_parameter<double>("map_origin_x", -18.5);
    declare_parameter<double>("map_origin_y", -10.5);
    declare_parameter<double>("map_origin_z", 0.0);

    raw_to_std_ <<
      0.0, 0.0, -1.0,
      1.0, 0.0, 0.0,
      0.0, -1.0, 0.0;

    body_to_base_rotation_raw_ <<
      0.0, 1.0, 0.0,
      0.0, 0.0, -1.0,
      -1.0, 0.0, 0.0;

    initial_body_to_base_translation_raw_ = parameter_vector3(
      *this, "initial_body_to_base_translation_raw", {0.004, 1.215, 0.072});

    rclcpp::QoS input_cloud_qos(rclcpp::KeepLast(3));
    input_cloud_qos.best_effort();
    rclcpp::QoS output_cloud_qos(rclcpp::KeepLast(3));
    output_cloud_qos.reliable();

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, 30);
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, output_cloud_qos);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      raw_odom_topic_, 50,
      std::bind(&FastLioOpen3dAxisAdapter::onRawOdom, this, std::placeholders::_1));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      raw_cloud_topic_, input_cloud_qos,
      std::bind(&FastLioOpen3dAxisAdapter::onRawCloud, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "fastlio_open3d_axis_adapter started: %s->%s, %s->%s, tf=%s->%s",
      raw_odom_topic_.c_str(), output_odom_topic_.c_str(),
      raw_cloud_topic_.c_str(), output_cloud_topic_.c_str(),
      odom_frame_.c_str(), output_base_frame_.c_str());
  }

private:
  void onRawOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto & pose = msg->pose.pose;
    Eigen::Quaterniond raw_quat(
      pose.orientation.w,
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z);
    if (raw_quat.norm() == 0.0) {
      raw_quat = Eigen::Quaterniond::Identity();
    } else {
      raw_quat.normalize();
    }

    const Eigen::Matrix3d raw_rotation = raw_quat.toRotationMatrix();
    const Eigen::Vector3d raw_translation(
      pose.position.x,
      pose.position.y,
      pose.position.z);

    const Eigen::Vector3d base_translation_raw =
      raw_translation + raw_rotation * initial_body_to_base_translation_raw_;
    const Eigen::Vector3d base_translation_std =
      raw_to_std_ * (base_translation_raw - initial_body_to_base_translation_raw_);
    const Eigen::Matrix3d base_rotation_std =
      raw_to_std_ * raw_rotation * body_to_base_rotation_raw_;

    Eigen::Quaterniond base_quat(base_rotation_std);
    base_quat.normalize();

    nav_msgs::msg::Odometry out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = odom_frame_;
    out.child_frame_id = output_base_frame_;
    out.pose.pose.position.x = base_translation_std.x();
    out.pose.pose.position.y = base_translation_std.y();
    out.pose.pose.position.z = base_translation_std.z();
    out.pose.pose.orientation.x = base_quat.x();
    out.pose.pose.orientation.y = base_quat.y();
    out.pose.pose.orientation.z = base_quat.z();
    out.pose.pose.orientation.w = base_quat.w();
    out.twist = msg->twist;
    odom_pub_->publish(out);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header = out.header;
      tf_msg.child_frame_id = output_base_frame_;
      tf_msg.transform.translation.x = out.pose.pose.position.x;
      tf_msg.transform.translation.y = out.pose.pose.position.y;
      tf_msg.transform.translation.z = out.pose.pose.position.z;
      tf_msg.transform.rotation = out.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf_msg);
    }
  }

  void onRawCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::vector<float> xyz;
    xyz.reserve(static_cast<std::size_t>(msg->width) * static_cast<std::size_t>(msg->height) * 3U);

    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        const float x = *iter_x;
        const float y = *iter_y;
        const float z = *iter_z;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          continue;
        }

        xyz.push_back(static_cast<float>(-(static_cast<double>(z) - initial_body_to_base_translation_raw_.z())));
        xyz.push_back(static_cast<float>(static_cast<double>(x) - initial_body_to_base_translation_raw_.x()));
        xyz.push_back(static_cast<float>(-(static_cast<double>(y) - initial_body_to_base_translation_raw_.y())));
      }
    } catch (const std::exception & exc) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "failed to read raw cloud: %s", exc.what());
      return;
    }

    if (xyz.empty()) {
      return;
    }

    sensor_msgs::msg::PointCloud2 out;
    out.header = msg->header;
    out.header.frame_id = odom_frame_;

    sensor_msgs::PointCloud2Modifier modifier(out);
    modifier.setPointCloud2Fields(
      3,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(xyz.size() / 3U);
    out.is_dense = true;

    sensor_msgs::PointCloud2Iterator<float> out_x(out, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(out, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(out, "z");
    for (std::size_t i = 0; i < xyz.size(); i += 3U, ++out_x, ++out_y, ++out_z) {
      *out_x = xyz[i];
      *out_y = xyz[i + 1U];
      *out_z = xyz[i + 2U];
    }

    cloud_pub_->publish(out);
  }

  std::string raw_odom_topic_;
  std::string raw_cloud_topic_;
  std::string output_odom_topic_;
  std::string output_cloud_topic_;
  std::string odom_frame_;
  std::string output_base_frame_;
  bool publish_tf_{true};

  Eigen::Matrix3d raw_to_std_{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d body_to_base_rotation_raw_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d initial_body_to_base_translation_raw_{0.004, 1.215, 0.072};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(false);
  rclcpp::spin(std::make_shared<FastLioOpen3dAxisAdapter>(options));
  rclcpp::shutdown();
  return 0;
}
