/*
 * rviz_initialpose_adapter.cpp
 *
 * 文件作用：
 * 1. 订阅 RViz 的 /initialpose。
 * 2. 将 map_ground 下的初始位姿转换到 map 坐标系。
 * 3. 同时转发给 prior_map_odom_bridge 和 RoboSense 定位初始化入口，避免手动重定位后被旧定位状态拉回。
 *
 * 上游：RViz 2D Pose Estimate 或 APP 等价初始位姿输入。
 * 下游：/prior_localization/pose_with_covariance 与 /prior_localization/manual_initialpose。
 */

#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{
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

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  const double half = 0.5 * yaw;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(half);
  q.w = std::cos(half);
  return q;
}
}  // namespace

class RvizInitialposeAdapter : public rclcpp::Node
{
public:
  RvizInitialposeAdapter()
  : Node("rviz_initialpose_adapter"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/initialpose");
    bridge_pose_topic_ = declare_parameter<std::string>(
      "bridge_pose_topic", "/prior_localization/pose_with_covariance");
    robosense_pose_topic_ = declare_parameter<std::string>(
      "robosense_pose_topic", "/prior_localization/manual_initialpose");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    map_ground_frame_ = declare_parameter<std::string>("map_ground_frame", "map_ground");
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.2);

    bridge_pose_pub_ =
      create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(bridge_pose_topic_, 10);
    robosense_pose_pub_ =
      create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(robosense_pose_topic_, 10);
    initialpose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      input_topic_, 10,
      std::bind(&RvizInitialposeAdapter::initialpose_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "RViz initial pose adapter started: %s -> %s, %s",
      input_topic_.c_str(), bridge_pose_topic_.c_str(), robosense_pose_topic_.c_str());
  }

private:
  void initialpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    auto converted = convert_to_map(*msg);
    if (!converted) {
      return;
    }

    converted->header.stamp = now();
    bridge_pose_pub_->publish(*converted);
    robosense_pose_pub_->publish(*converted);

    const auto & pose = converted->pose.pose.position;
    const double yaw_deg = yaw_from_quaternion(converted->pose.pose.orientation) * 180.0 / M_PI;
    RCLCPP_INFO(
      get_logger(), "forwarded manual initial pose in %s: x=%.3f y=%.3f yaw=%.1fdeg",
      map_frame_.c_str(), pose.x, pose.y, yaw_deg);
  }

  std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> convert_to_map(
    const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
  {
    const std::string frame_id = msg.header.frame_id.empty() ? map_ground_frame_ : msg.header.frame_id;
    if (frame_id == map_frame_) {
      return msg;
    }

    if (frame_id != map_ground_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "ignore initial pose in unsupported frame %s; expected %s or %s",
        frame_id.c_str(), map_ground_frame_.c_str(), map_frame_.c_str());
      return std::nullopt;
    }

    tf2::TimePoint stamp = tf2::TimePointZero;
    if (msg.header.stamp.sec != 0 || msg.header.stamp.nanosec != 0) {
      stamp = tf2_ros::fromMsg(msg.header.stamp);
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
        map_frame_, map_ground_frame_, stamp, tf2::durationFromSec(tf_timeout_sec_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "waiting for TF %s->%s before using manual initial pose: %s",
        map_frame_.c_str(), map_ground_frame_.c_str(), ex.what());
      return std::nullopt;
    }

    const double tf_yaw = yaw_from_quaternion(transform.transform.rotation);
    const double pose_yaw = yaw_from_quaternion(msg.pose.pose.orientation);
    const double out_yaw = normalize_angle(tf_yaw + pose_yaw);
    const double px = msg.pose.pose.position.x;
    const double py = msg.pose.pose.position.y;
    const double cos_yaw = std::cos(tf_yaw);
    const double sin_yaw = std::sin(tf_yaw);

    geometry_msgs::msg::PoseWithCovarianceStamped converted;
    converted.header = msg.header;
    converted.header.frame_id = map_frame_;
    converted.pose.covariance = msg.pose.covariance;
    converted.pose.pose.position.x = transform.transform.translation.x + cos_yaw * px - sin_yaw * py;
    converted.pose.pose.position.y = transform.transform.translation.y + sin_yaw * px + cos_yaw * py;
    converted.pose.pose.position.z = transform.transform.translation.z + msg.pose.pose.position.z;
    converted.pose.pose.orientation = quaternion_from_yaw(out_yaw);
    return converted;
  }

  std::string input_topic_;
  std::string bridge_pose_topic_;
  std::string robosense_pose_topic_;
  std::string map_frame_;
  std::string map_ground_frame_;
  double tf_timeout_sec_{0.2};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr bridge_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr robosense_pose_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RvizInitialposeAdapter>());
  rclcpp::shutdown();
  return 0;
}
