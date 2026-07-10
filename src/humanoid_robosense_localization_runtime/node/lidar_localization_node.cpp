/******************************************************************************
Copyright 2025 RoboSense Technology Co., Ltd

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
 *****************************************************************************/
 
#include "lidar_localization.h"
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <map>
#include <mutex>
#include <sstream>
#include <string>

#ifdef ROS1 
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#endif

#ifdef ROS2
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>
#endif

std::shared_ptr<LidarLocalization> lidar_localization_ptr(nullptr);
#ifdef ROS1 
ros::Publisher lidar_pose_pub;
#endif

#ifdef ROS2 
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr lidar_pose_pub;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_cache_pub;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr localization_status_pub;
std::shared_ptr<tf2_ros::TransformBroadcaster> map_odom_tf_broadcaster;
#endif

std::mutex tf_rel_pose_mutex;
std::map<double, Pose> tf_rel_poses_map;
std::string map_frame_id = "map";
std::string odom_frame_id = "odom";
std::string base_frame_id = "base_footprint";
bool publish_map_to_odom_tf = true;
double tf_odom_cache_duration_sec = 5.0;
double tf_max_interpolation_gap_sec = 0.25;
std::mutex raw_odom_mutex;
std::map<double, Pose> raw_odom_map;
bool convert_fastlio_odom_to_base = false;
bool convert_registered_cloud_to_body = false;
double registered_cloud_odom_sync_tolerance_sec = 0.15;
bool publish_odom_cache = true;
std::string pose_topic = "/prior_localization/robosense_odom";
std::string odom_cache_topic = "/prior_localization/robosense_input_odom";
std::string abs_pose_topic = "/prior_localization/manual_initialpose";
std::string status_topic = "/prior_localization/robosense_status";
Eigen::Vector3d fastlio_odom_camera_xyz = Eigen::Vector3d::Zero();
Eigen::Quaterniond fastlio_odom_camera_q(0.5, -0.5, -0.5, 0.5);
Eigen::Vector3d fastlio_body_base_xyz(0.004, 1.215, 0.072);
Eigen::Quaterniond fastlio_body_base_q(0.5, 0.5, 0.5, -0.5);

namespace {

Eigen::Matrix4d makeTransform(const Eigen::Vector3d &xyz,
                              const Eigen::Quaterniond &q) {
  Eigen::Affine3d transform = Eigen::Translation3d(xyz) * q;
  return transform.matrix();
}

Eigen::Vector3d loadVector3(const YAML::Node &node,
                            const Eigen::Vector3d &default_value) {
  if (!node) {
    return default_value;
  }
  const auto values = node.as<std::vector<double>>();
  if (values.size() != 3) {
    return default_value;
  }
  return Eigen::Vector3d(values[0], values[1], values[2]);
}

Eigen::Quaterniond loadQuatXyzw(const YAML::Node &node,
                                const Eigen::Quaterniond &default_value) {
  if (!node) {
    return default_value;
  }
  const auto values = node.as<std::vector<double>>();
  if (values.size() != 4) {
    return default_value;
  }
  Eigen::Quaterniond q(values[3], values[0], values[1], values[2]);
  q.normalize();
  return q;
}

const char *statusName(STATUS status) {
  switch (status) {
  case STATUS::IDLE:
    return "IDLE";
  case STATUS::LOW_ACCURACY:
    return "LOW_ACCURACY";
  case STATUS::NORMAL:
    return "NORMAL";
  case STATUS::LOST:
    return "LOST";
  case STATUS::NO_ENOUGH_MAP:
    return "NO_ENOUGH_MAP";
  case STATUS::LOW_ACCURACY_RPZ:
    return "LOW_ACCURACY_RPZ";
  }
  return "UNKNOWN";
}

#ifdef ROS2
void publishLocalizationStatus(const Pose *pose, const std::string &event) {
  if (!localization_status_pub) {
    return;
  }

  const std::string status =
      pose ? std::string(statusName(pose->status_code)) : "UNKNOWN";
  const std::string source = pose ? pose->source : "";
  const double stamp = pose ? pose->timestamp : rclcpp::Clock().now().seconds();
  const double x = pose ? pose->xyz.x() : 0.0;
  const double y = pose ? pose->xyz.y() : 0.0;
  const double z = pose ? pose->xyz.z() : 0.0;
  const int status_code = pose ? static_cast<int>(pose->status_code) : -1;

  std_msgs::msg::String msg;
  std::ostringstream oss;
  oss << "{"
      << "\"event\":\"" << event << "\","
      << "\"status\":\"" << status << "\","
      << "\"status_code\":" << status_code << ","
      << "\"source\":\"" << source << "\","
      << "\"stamp\":" << std::fixed << std::setprecision(6) << stamp << ","
      << "\"x\":" << std::setprecision(4) << x << ","
      << "\"y\":" << std::setprecision(4) << y << ","
      << "\"z\":" << std::setprecision(4) << z
      << "}";
  msg.data = oss.str();
  localization_status_pub->publish(msg);
}
#endif

void loadNodeConfig(const YAML::Node &config_node) {
  if (config_node["publish_map_to_odom_tf"]) {
    publish_map_to_odom_tf = config_node["publish_map_to_odom_tf"].as<bool>();
  }
  if (config_node["map_frame_id"]) {
    map_frame_id = config_node["map_frame_id"].as<std::string>();
  }
  if (config_node["odom_frame_id"]) {
    odom_frame_id = config_node["odom_frame_id"].as<std::string>();
  }
  if (config_node["base_frame_id"]) {
    base_frame_id = config_node["base_frame_id"].as<std::string>();
  }
  if (config_node["tf_odom_cache_duration_sec"]) {
    tf_odom_cache_duration_sec =
        config_node["tf_odom_cache_duration_sec"].as<double>();
  }
  if (config_node["tf_max_interpolation_gap_sec"]) {
    tf_max_interpolation_gap_sec =
        config_node["tf_max_interpolation_gap_sec"].as<double>();
  }
  if (config_node["convert_fastlio_odom_to_base"]) {
    convert_fastlio_odom_to_base =
        config_node["convert_fastlio_odom_to_base"].as<bool>();
  }
  if (config_node["convert_registered_cloud_to_body"]) {
    convert_registered_cloud_to_body =
        config_node["convert_registered_cloud_to_body"].as<bool>();
  }
  if (config_node["registered_cloud_odom_sync_tolerance_sec"]) {
    registered_cloud_odom_sync_tolerance_sec =
        config_node["registered_cloud_odom_sync_tolerance_sec"].as<double>();
  }
  if (config_node["pose_topic"]) {
    pose_topic = config_node["pose_topic"].as<std::string>();
  }
  if (config_node["odom_cache_topic"]) {
    odom_cache_topic = config_node["odom_cache_topic"].as<std::string>();
  }
  if (config_node["abs_pose_topic"]) {
    abs_pose_topic = config_node["abs_pose_topic"].as<std::string>();
  }
  if (config_node["status_topic"]) {
    status_topic = config_node["status_topic"].as<std::string>();
  }
  if (config_node["publish_odom_cache"]) {
    publish_odom_cache = config_node["publish_odom_cache"].as<bool>();
  }
  fastlio_odom_camera_xyz =
      loadVector3(config_node["fastlio_odom_camera_xyz"],
                  fastlio_odom_camera_xyz);
  fastlio_odom_camera_q =
      loadQuatXyzw(config_node["fastlio_odom_camera_quat_xyzw"],
                   fastlio_odom_camera_q);
  fastlio_body_base_xyz =
      loadVector3(config_node["fastlio_body_base_xyz"],
                  fastlio_body_base_xyz);
  fastlio_body_base_q =
      loadQuatXyzw(config_node["fastlio_body_base_quat_xyzw"],
                   fastlio_body_base_q);
}

template <typename FieldT>
bool hasField(const std::vector<FieldT> &fields, const std::string &name) {
  for (const auto &field : fields) {
    if (field.name == name) {
      return true;
    }
  }
  return false;
}

void normalizePointTimestamps(pcl::PointCloud<RsPointXYZIRT>::Ptr &cloud,
                              double msg_time, bool has_timestamp_field) {
  if (cloud->points.empty()) {
    cloud->header.stamp = static_cast<std::uint64_t>(msg_time * 1e6);
    return;
  }

  bool timestamps_are_relative = false;
  if (has_timestamp_field) {
    for (const auto &pt : cloud->points) {
      if (std::isfinite(pt.timestamp) && pt.timestamp > 0.0) {
        timestamps_are_relative = pt.timestamp < 1e6;
        break;
      }
    }
  }

  for (auto &pt : cloud->points) {
    if (!has_timestamp_field || !std::isfinite(pt.timestamp) ||
        pt.timestamp <= 0.0) {
      pt.timestamp = msg_time;
    } else if (timestamps_are_relative) {
      pt.timestamp += msg_time;
    }
  }

  cloud->header.stamp =
      static_cast<std::uint64_t>(cloud->points.back().timestamp * 1e6);
}

void cacheRelPoseForTf(const Pose &pose) {
  std::lock_guard<std::mutex> lock(tf_rel_pose_mutex);
  tf_rel_poses_map[pose.timestamp] = pose;
  const double oldest_to_keep = pose.timestamp - tf_odom_cache_duration_sec;
  auto erase_end = tf_rel_poses_map.lower_bound(oldest_to_keep);
  tf_rel_poses_map.erase(tf_rel_poses_map.begin(), erase_end);
}

void cacheRawOdomPoseForCloud(const Pose &pose) {
  std::lock_guard<std::mutex> lock(raw_odom_mutex);
  raw_odom_map[pose.timestamp] = pose;
  const double oldest_to_keep = pose.timestamp - tf_odom_cache_duration_sec;
  auto erase_end = raw_odom_map.lower_bound(oldest_to_keep);
  raw_odom_map.erase(raw_odom_map.begin(), erase_end);
}

bool nearestCachedRawOdomPose(double timestamp, Pose &pose_at_t) {
  std::lock_guard<std::mutex> lock(raw_odom_mutex);
  if (raw_odom_map.empty()) {
    return false;
  }

  auto next = raw_odom_map.lower_bound(timestamp);
  auto best = raw_odom_map.end();
  double best_delta = std::numeric_limits<double>::max();

  if (next != raw_odom_map.end()) {
    best = next;
    best_delta = std::fabs(next->first - timestamp);
  }
  if (next != raw_odom_map.begin()) {
    auto prev = std::prev(next);
    const double prev_delta = std::fabs(prev->first - timestamp);
    if (prev_delta < best_delta) {
      best = prev;
      best_delta = prev_delta;
    }
  }

  if (best == raw_odom_map.end() ||
      best_delta > registered_cloud_odom_sync_tolerance_sec) {
    return false;
  }

  pose_at_t = best->second;
  return true;
}

bool interpolateCachedRawOdomPose(double timestamp, Pose &pose_at_t) {
  std::lock_guard<std::mutex> lock(raw_odom_mutex);
  if (raw_odom_map.size() < 2) {
    return false;
  }

  auto next = raw_odom_map.lower_bound(timestamp);
  if (next == raw_odom_map.end()) {
    auto last = std::prev(raw_odom_map.end());
    if (std::fabs(timestamp - last->first) <=
        registered_cloud_odom_sync_tolerance_sec) {
      pose_at_t = last->second;
      pose_at_t.timestamp = timestamp;
      return true;
    }
    return false;
  }

  if (next == raw_odom_map.begin()) {
    if (std::fabs(timestamp - next->first) <=
        registered_cloud_odom_sync_tolerance_sec) {
      pose_at_t = next->second;
      pose_at_t.timestamp = timestamp;
      return true;
    }
    return false;
  }

  if (std::fabs(timestamp - next->first) < 1e-6) {
    pose_at_t = next->second;
    pose_at_t.timestamp = timestamp;
    return true;
  }

  auto prev = std::prev(next);
  const double gap = next->first - prev->first;
  if (gap <= 0.0 || gap > registered_cloud_odom_sync_tolerance_sec) {
    return false;
  }

  const double ratio = (timestamp - prev->first) / gap;
  pose_at_t.timestamp = timestamp;
  pose_at_t.xyz = (1.0 - ratio) * prev->second.xyz + ratio * next->second.xyz;
  pose_at_t.q = prev->second.q.slerp(ratio, next->second.q);
  pose_at_t.q.normalize();
  pose_at_t.source = prev->second.source;
  return true;
}

bool interpolateCachedRelPose(double timestamp, Pose &pose_at_t) {
  std::lock_guard<std::mutex> lock(tf_rel_pose_mutex);
  if (tf_rel_poses_map.size() < 2) {
    return false;
  }

  auto next = tf_rel_poses_map.lower_bound(timestamp);
  if (next == tf_rel_poses_map.end()) {
    auto last = std::prev(tf_rel_poses_map.end());
    if (std::fabs(timestamp - last->first) <= tf_max_interpolation_gap_sec) {
      pose_at_t = last->second;
      pose_at_t.timestamp = timestamp;
      return true;
    }
    return false;
  }

  if (next == tf_rel_poses_map.begin()) {
    if (std::fabs(timestamp - next->first) <= tf_max_interpolation_gap_sec) {
      pose_at_t = next->second;
      pose_at_t.timestamp = timestamp;
      return true;
    }
    return false;
  }

  if (std::fabs(timestamp - next->first) < 1e-6) {
    pose_at_t = next->second;
    pose_at_t.timestamp = timestamp;
    return true;
  }

  auto prev = std::prev(next);
  const double gap = next->first - prev->first;
  if (gap <= 0.0 || gap > tf_max_interpolation_gap_sec) {
    return false;
  }

  const double ratio = (timestamp - prev->first) / gap;
  pose_at_t.timestamp = timestamp;
  pose_at_t.xyz = (1.0 - ratio) * prev->second.xyz + ratio * next->second.xyz;
  pose_at_t.q = prev->second.q.slerp(ratio, next->second.q);
  pose_at_t.q.normalize();
  return true;
}

Pose convertFastlioCameraOdomToBase(const Pose &camera_body_pose) {
  const Eigen::Matrix4d t_odom_camera =
      makeTransform(fastlio_odom_camera_xyz, fastlio_odom_camera_q);
  const Eigen::Matrix4d t_body_base =
      makeTransform(fastlio_body_base_xyz, fastlio_body_base_q);
  Pose base_pose(t_odom_camera * camera_body_pose.transform() * t_body_base,
                 camera_body_pose.timestamp);
  base_pose.source = camera_body_pose.source;
  return base_pose;
}

pcl::PointCloud<RsPointXYZIRT>::Ptr convertRegisteredCloudToBodyCloud(
    const pcl::PointCloud<pcl::PointXYZI> &registered_cloud, double msg_time) {
  Pose camera_body_pose;
  if (!interpolateCachedRawOdomPose(msg_time, camera_body_pose)) {
    static double last_warn_time = 0.0;
    if (msg_time - last_warn_time > 2.0) {
      std::cout << "drop registered cloud without synced /odom: "
                << std::fixed << msg_time << std::endl;
#ifdef ROS2
      publishLocalizationStatus(nullptr, "drop_registered_cloud_without_synced_odom");
#endif
      last_warn_time = msg_time;
    }
    return nullptr;
  }

  auto cloud = pcl::PointCloud<RsPointXYZIRT>::Ptr(
      new pcl::PointCloud<RsPointXYZIRT>);
  cloud->reserve(registered_cloud.points.size());
  const Eigen::Matrix3d r_body_camera =
      camera_body_pose.q.toRotationMatrix().transpose();

  for (const auto &pt : registered_cloud.points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) ||
        !std::isfinite(pt.z)) {
      continue;
    }
    const Eigen::Vector3d point_camera(pt.x, pt.y, pt.z);
    const Eigen::Vector3d point_body =
        r_body_camera * (point_camera - camera_body_pose.xyz);

    RsPointXYZIRT out_pt;
    out_pt.x = static_cast<float>(point_body.x());
    out_pt.y = static_cast<float>(point_body.y());
    out_pt.z = static_cast<float>(point_body.z());
    out_pt.intensity = pt.intensity;
    out_pt.ring = 0;
    out_pt.timestamp = msg_time;
    cloud->points.emplace_back(out_pt);
  }

  cloud->height = 1;
  cloud->width = cloud->points.size();
  cloud->is_dense = true;
  cloud->header.stamp = static_cast<std::uint64_t>(msg_time * 1e6);
  return cloud;
}

}  // namespace


#ifdef ROS1
Pose odom2pose(const nav_msgs::Odometry &odom) {
  Pose pose;
  pose.timestamp = odom.header.stamp.toSec();
  pose.xyz << odom.pose.pose.position.x, odom.pose.pose.position.y,
      odom.pose.pose.position.z;
  pose.q.x() = odom.pose.pose.orientation.x;
  pose.q.y() = odom.pose.pose.orientation.y;
  pose.q.z() = odom.pose.pose.orientation.z;
  pose.q.w() = odom.pose.pose.orientation.w;
  return pose;
}
#endif

#ifdef ROS2
Pose odom2pose(const nav_msgs::msg::Odometry::SharedPtr &odom) {
  Pose pose;
  double timestamp = odom->header.stamp.sec + odom->header.stamp.nanosec * 1e-9;
  pose.timestamp = timestamp;
  pose.xyz << odom->pose.pose.position.x, 
             odom->pose.pose.position.y, 
             odom->pose.pose.position.z;
  pose.q.x() = odom->pose.pose.orientation.x;
  pose.q.y() = odom->pose.pose.orientation.y;
  pose.q.z() = odom->pose.pose.orientation.z;
  pose.q.w() = odom->pose.pose.orientation.w;
  return pose;
}
#endif

#ifdef ROS2
Pose poseMsgToPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr &msg) {
  Pose pose;
  pose.timestamp =
      msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  pose.xyz << msg->pose.pose.position.x, msg->pose.pose.position.y,
      msg->pose.pose.position.z;
  pose.q.x() = msg->pose.pose.orientation.x;
  pose.q.y() = msg->pose.pose.orientation.y;
  pose.q.z() = msg->pose.pose.orientation.z;
  pose.q.w() = msg->pose.pose.orientation.w;
  pose.q.normalize();
  pose.source = "manual_initialpose";
  return pose;
}
#endif

#ifdef ROS1
void relPoseCallback(const nav_msgs::OdometryConstPtr &msg) {
  auto raw_pose = odom2pose(*msg);
  raw_pose.q.normalize();
  cacheRawOdomPoseForCloud(raw_pose);
  auto pose = convert_fastlio_odom_to_base
                  ? convertFastlioCameraOdomToBase(raw_pose)
                  : raw_pose;
  cacheRelPoseForTf(pose);
  lidar_localization_ptr->addRelPose(pose);
}

void lidarCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
  if (convert_registered_cloud_to_body) {
    pcl::PointCloud<pcl::PointXYZI> registered_cloud;
    pcl::fromROSMsg(*msg, registered_cloud);
    auto cloud =
        convertRegisteredCloudToBodyCloud(registered_cloud,
                                          msg->header.stamp.toSec());
    if (cloud && !cloud->points.empty()) {
      lidar_localization_ptr->addLidarData(cloud);
    }
    return;
  }

  pcl::PointCloud<RsPointXYZIRT>::Ptr cloud(new pcl::PointCloud<RsPointXYZIRT>);
  pcl::fromROSMsg(*msg, *cloud);
  normalizePointTimestamps(cloud, msg->header.stamp.toSec(),
                           hasField(msg->fields, "timestamp"));
  lidar_localization_ptr->addLidarData(cloud);
}
#endif

#ifdef ROS2
void relPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  auto raw_pose = odom2pose(msg);
  raw_pose.q.normalize();
  cacheRawOdomPoseForCloud(raw_pose);
  auto pose = convert_fastlio_odom_to_base
                  ? convertFastlioCameraOdomToBase(raw_pose)
                  : raw_pose;
  cacheRelPoseForTf(pose);
  lidar_localization_ptr->addRelPose(pose);
  if (publish_odom_cache && odom_cache_pub) {
    nav_msgs::msg::Odometry odom = *msg;
    odom.header.stamp = rclcpp::Time(static_cast<int64_t>(pose.timestamp * 1e9));
    odom.header.frame_id = odom_frame_id;
    odom.child_frame_id = base_frame_id;
    odom.pose.pose.position.x = pose.xyz.x();
    odom.pose.pose.position.y = pose.xyz.y();
    odom.pose.pose.position.z = pose.xyz.z();
    odom.pose.pose.orientation.x = pose.q.x();
    odom.pose.pose.orientation.y = pose.q.y();
    odom.pose.pose.orientation.z = pose.q.z();
    odom.pose.pose.orientation.w = pose.q.w();
    odom_cache_pub->publish(odom);
  }
}

void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  const double msg_time =
      msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  if (convert_registered_cloud_to_body) {
    pcl::PointCloud<pcl::PointXYZI> registered_cloud;
    pcl::fromROSMsg(*msg, registered_cloud);
    auto cloud = convertRegisteredCloudToBodyCloud(registered_cloud, msg_time);
    if (cloud && !cloud->points.empty()) {
      lidar_localization_ptr->addLidarData(cloud);
    }
    return;
  }

  pcl::PointCloud<RsPointXYZIRT>::Ptr cloud(new pcl::PointCloud<RsPointXYZIRT>);
  pcl::fromROSMsg(*msg, *cloud);
  normalizePointTimestamps(cloud, msg_time, hasField(msg->fields, "timestamp"));
  lidar_localization_ptr->addLidarData(cloud);
}

void absPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  if (!lidar_localization_ptr) {
    return;
  }

  if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_id) {
    RCLCPP_WARN(
        rclcpp::get_logger("robosense_lidar_localization_node"),
        "ignore manual pose in frame %s, expected %s",
        msg->header.frame_id.c_str(), map_frame_id.c_str());
    return;
  }

  Pose pose = poseMsgToPose(msg);
  if (pose.timestamp <= 0.0) {
    pose.timestamp = rclcpp::Clock().now().seconds();
  }
  pose.setStatus(STATUS::LOW_ACCURACY);
  lidar_localization_ptr->setManualPose(pose);
  publishLocalizationStatus(&pose, "manual_initialpose_applied");

  RCLCPP_INFO(
      rclcpp::get_logger("robosense_lidar_localization_node"),
      "applied manual initial pose on %s: x=%.3f y=%.3f z=%.3f",
      abs_pose_topic.c_str(), pose.xyz.x(), pose.xyz.y(), pose.xyz.z());
}
#endif

#ifdef ROS2
void publishMapToOdomTf(const Pose &map_base_pose) {
  if (!publish_map_to_odom_tf || !map_odom_tf_broadcaster) {
    return;
  }

  Pose odom_base_pose;
  if (!interpolateCachedRelPose(map_base_pose.timestamp, odom_base_pose)) {
    static double last_warn_time = 0.0;
    if (map_base_pose.timestamp - last_warn_time > 2.0) {
      std::cout << "skip map->odom TF: no synchronized odom pose at "
                << std::fixed << map_base_pose.timestamp << std::endl;
      last_warn_time = map_base_pose.timestamp;
    }
    return;
  }

  const Eigen::Matrix4d t_map_base = map_base_pose.transform();
  const Eigen::Matrix4d t_odom_base = odom_base_pose.transform();
  const Eigen::Matrix4d t_map_odom = t_map_base * t_odom_base.inverse();
  Eigen::Quaterniond q_map_odom(t_map_odom.block<3, 3>(0, 0));
  q_map_odom.normalize();

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = rclcpp::Time(
      static_cast<int64_t>(map_base_pose.timestamp * 1e9));
  tf_msg.header.frame_id = map_frame_id;
  tf_msg.child_frame_id = odom_frame_id;
  tf_msg.transform.translation.x = t_map_odom(0, 3);
  tf_msg.transform.translation.y = t_map_odom(1, 3);
  tf_msg.transform.translation.z = t_map_odom(2, 3);
  tf_msg.transform.rotation.x = q_map_odom.x();
  tf_msg.transform.rotation.y = q_map_odom.y();
  tf_msg.transform.rotation.z = q_map_odom.z();
  tf_msg.transform.rotation.w = q_map_odom.w();
  map_odom_tf_broadcaster->sendTransform(tf_msg);
}
#endif

int main(int argc, char **argv) {
#ifdef ROS1
  ros::init(argc, argv, "lidar_localization_node");
  ros::NodeHandle nh;

  nh->declare_parameter<std::string>(
      "config_file",
      std::string(PACKAGE_PATH) + "/config/robosense_lidar_localization.yaml");
  std::string config_file = nh->get_parameter("config_file").as_string();
  std::string lidar_topic = "";
  std::string rel_pose_topic = "";
  YAML::Node config_node;
  try {
    config_node = YAML::LoadFile(config_file);
    lidar_topic = config_node["lidar_topic"].as<std::string>();
    rel_pose_topic = config_node["rel_pose_topic"].as<std::string>();
    loadNodeConfig(config_node);
  } catch (...) {
    std::cout << "config file load failed!" << std::endl;
    return -1;
  }
  lidar_localization_ptr = std::make_shared<LidarLocalization>(config_node);
  lidar_pose_pub = nh.advertise<nav_msgs::Odometry>("/lidar_pose_xyz", 10);
  auto pose_func = [&](const Pose &pose) {
    nav_msgs::Odometry odom;
    Eigen::Vector3d xyz(pose.xyz.x(), pose.xyz.y(),
                        pose.xyz.z());
    odom.header.stamp = ros::Time(pose.timestamp);
    odom.header.frame_id = map_frame_id;
    odom.child_frame_id = base_frame_id;
    /////加入status
    odom.pose.covariance[0] = static_cast<double>(pose.status_code); // status
    odom.pose.pose.position.x = pose.xyz.x();
    odom.pose.pose.position.y = pose.xyz.y();
    odom.pose.pose.position.z = pose.xyz.z();
    odom.pose.pose.orientation.x = pose.q.x();
    odom.pose.pose.orientation.y = pose.q.y();
    odom.pose.pose.orientation.z = pose.q.z();
    odom.pose.pose.orientation.w = pose.q.w();
    lidar_pose_pub.publish(odom);
  };
  lidar_localization_ptr->registerCallback(pose_func);

  ros::Subscriber lidar_sub = nh.subscribe(lidar_topic, 10, lidarCallback);
  ros::Subscriber rel_pose_sub =
      nh.subscribe(rel_pose_topic, 100, relPoseCallback);

  ros::MultiThreadedSpinner spinner(3);
  spinner.spin();
  return 0;
#endif

#ifdef ROS2
  // 初始化 ROS2
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("robosense_lidar_localization_node");

  nh->declare_parameter<std::string>(
      "config_file",
      std::string(PACKAGE_PATH) + "/config/robosense_lidar_localization.yaml");
  std::string config_file = nh->get_parameter("config_file").as_string();

  std::string lidar_topic = "";
  std::string rel_pose_topic = "";
  YAML::Node config_node;
  try {
    config_node = YAML::LoadFile(config_file);
    lidar_topic = config_node["lidar_topic"].as<std::string>();
    rel_pose_topic = config_node["rel_pose_topic"].as<std::string>();
    loadNodeConfig(config_node);
  } catch (...) {
    std::cout << "config file load failed!" << std::endl;
    return -1;
  }

  lidar_localization_ptr = std::make_shared<LidarLocalization>(config_node);

  lidar_pose_pub = nh->create_publisher<nav_msgs::msg::Odometry>(pose_topic, 10);
  odom_cache_pub =
      nh->create_publisher<nav_msgs::msg::Odometry>(odom_cache_topic, 100);
  localization_status_pub =
      nh->create_publisher<std_msgs::msg::String>(status_topic, 10);
  map_odom_tf_broadcaster =
      std::make_shared<tf2_ros::TransformBroadcaster>(nh);
  auto pose_func = [&](const Pose &pose) {
    nav_msgs::msg::Odometry odom;
    Eigen::Vector3d xyz(pose.xyz.x(), pose.xyz.y(),
                        pose.xyz.z());
    odom.header.stamp = rclcpp::Time(pose.timestamp * 1e9); // 将秒转为纳秒
    odom.header.frame_id = map_frame_id;
    odom.child_frame_id = base_frame_id;
    /////加入status
    odom.pose.covariance[0] = static_cast<double>(pose.status_code); // status
    odom.pose.pose.position.x = pose.xyz.x();
    odom.pose.pose.position.y = pose.xyz.y();
    odom.pose.pose.position.z = pose.xyz.z();
    odom.pose.pose.orientation.x = pose.q.x();
    odom.pose.pose.orientation.y = pose.q.y();
    odom.pose.pose.orientation.z = pose.q.z();
    odom.pose.pose.orientation.w = pose.q.w();
    lidar_pose_pub->publish(odom);
    publishLocalizationStatus(&pose, "pose_update");
    publishMapToOdomTf(pose);
  };
  lidar_localization_ptr->registerCallback(pose_func);

  auto lidar_sub = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
    lidar_topic, 10, lidarCallback);

  auto rel_pose_sub = nh->create_subscription<nav_msgs::msg::Odometry>(
      rel_pose_topic, 100, relPoseCallback);
  auto abs_pose_sub =
      nh->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          abs_pose_topic, 10, absPoseCallback);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(nh);
  executor.spin();
  executor.remove_node(nh);
  map_odom_tf_broadcaster.reset();
  lidar_pose_pub.reset();
  localization_status_pub.reset();
  lidar_localization_ptr.reset();
  nh.reset();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;

#endif

}
