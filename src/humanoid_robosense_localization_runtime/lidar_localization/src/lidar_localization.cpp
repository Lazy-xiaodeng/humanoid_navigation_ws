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
#include <filesystem>

#ifdef ROS1 
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#endif

#ifdef ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#endif

LidarLocalization::LidarLocalization(const YAML::Node &config_node)
  : config_node_(config_node) {
  std::string lidar_topic = config_node_["lidar_topic"].as<std::string>();
  is_pub_cloud_ = config_node["is_pub_cloud"].as<bool>();
  is_pub_map_ = config_node["is_pub_map"].as<bool>();
  input_cloud_size_thr_ = config_node["input_cloud_size_thr"].as<size_t>();
  blind_distance_ = config_node["lidar_matcher"]["blind_distance"].as<double>();
  source_voxel_leaf_size_ = config_node_["source_voxel_leaf_size"]
                                ? config_node_["source_voxel_leaf_size"].as<double>()
                                : 0.15;
  debug_print_ = config_node_["debug_print"] ? config_node_["debug_print"].as<bool>() : false;

  std::vector<double> lidar_vehicle_xyz = config_node_["lidar_vehicle_xyz"].as<std::vector<double>>();
  std::vector<double> lidar_vehicle_rpy = config_node_["lidar_vehicle_rpy"].as<std::vector<double>>();
  auto rotation_lidar_vehicle_ = Eigen::AngleAxisd(lidar_vehicle_rpy[2], Eigen::Vector3d::UnitZ()) *
                                  Eigen::AngleAxisd(lidar_vehicle_rpy[1], Eigen::Vector3d::UnitY()) *
                                  Eigen::AngleAxisd(lidar_vehicle_rpy[0], Eigen::Vector3d::UnitX()).toRotationMatrix();
  T_base_lidar_.block<3, 3>(0, 0) = rotation_lidar_vehicle_;
  T_base_lidar_.block<3, 1>(0, 3) = Eigen::Vector3d(lidar_vehicle_xyz[0], lidar_vehicle_xyz[1], lidar_vehicle_xyz[2]);
  // init pose
  std::vector<double> init_xyz = config_node_["init_position"].as<std::vector<double>>();
  std::vector<double> init_rpy = config_node_["init_euler"].as<std::vector<double>>();
  if (config_node_["matching_check_residual_thresh"]) {
    MATCHING_CHECK_RESIDUAL_THRESH =
        config_node_["matching_check_residual_thresh"].as<double>();
  }
  if (config_node_["matching_check_valid_pair_ratio_thresh"]) {
    MATCHING_CHECK_VALID_PAIR_RATIO_THRESH =
        config_node_["matching_check_valid_pair_ratio_thresh"].as<double>();
  }
  if (config_node_["matching_check_prior_xy_thresh"]) {
    MATCHING_CHECK_PRIOR_XY_THRESH =
        config_node_["matching_check_prior_xy_thresh"].as<double>();
  }
  if (config_node_["matching_check_prior_z_thresh"]) {
    MATCHING_CHECK_PRIOR_Z_THRESH =
        config_node_["matching_check_prior_z_thresh"].as<double>();
  }
  if (config_node_["matching_check_prior_yaw_thresh_deg"]) {
    MATCHING_CHECK_PRIOR_YAW_THRESH_DEG =
        config_node_["matching_check_prior_yaw_thresh_deg"].as<double>();
  }
  init_orientation_ = Eigen::AngleAxisd(init_rpy[2], Eigen::Vector3d::UnitZ()) *
                      Eigen::AngleAxisd(init_rpy[1], Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(init_rpy[0], Eigen::Vector3d::UnitX());
  init_position_ = Eigen::Vector3d(init_xyz[0], init_xyz[1], init_xyz[2]);
  std::cout << "-------------------- Configuration --------------------"<<std::endl;
  std::cout << "is_pub_cloud_: " <<is_pub_cloud_ << std::endl;
  std::cout << "input_cloud_size_thr: " << input_cloud_size_thr_ << std::endl;
  std::cout << "blind_distance_: " << blind_distance_ << std::endl;
  std::cout << "source_voxel_leaf_size_: " << source_voxel_leaf_size_ << std::endl;
  std::cout << "debug_print_: " << debug_print_ << std::endl;
  std::cout << "matching_check_residual_thresh: "
            << MATCHING_CHECK_RESIDUAL_THRESH << std::endl;
  std::cout << "matching_check_valid_pair_ratio_thresh: "
            << MATCHING_CHECK_VALID_PAIR_RATIO_THRESH << std::endl;
  std::cout << "matching_check_prior_xy_thresh: "
            << MATCHING_CHECK_PRIOR_XY_THRESH << std::endl;
  std::cout << "matching_check_prior_z_thresh: "
            << MATCHING_CHECK_PRIOR_Z_THRESH << std::endl;
  std::cout << "matching_check_prior_yaw_thresh_deg: "
            << MATCHING_CHECK_PRIOR_YAW_THRESH_DEG << std::endl;
  std::cout << "-------------------- Calibration --------------------"<<std::endl;
  std::cout << "T_base_lidar_: \n" << T_base_lidar_ << std::endl;
  std::cout << "-------------------- Initialization --------------------"<<std::endl;
  std::cout << "init_position_: \n" << init_position_.transpose() << std::endl;
  std::cout << "init_orientation_: \n" << init_orientation_.matrix() << std::endl;
  std::cout << "-------------------------------------------------------"<<std::endl;

  bool init_map = initMap();
  status_ = STATUS::LOST;
#ifndef USE_EIGEN_OPTIMIZATION
  lidar_matcher_ = std::make_shared<LidarMatcherCeres>(config_node_["lidar_matcher"]);
#else
  lidar_matcher_ = std::make_shared<LidarMatcherEigen>(config_node_["lidar_matcher"]);
#endif
  yamlRead<int>(
      config_node_["lidar_matcher"], "recovery_pose_prior_release_normal_frames",
      recovery_pose_prior_release_normal_frames_, 6);
  recovery_pose_prior_release_normal_frames_ =
      std::max(1, recovery_pose_prior_release_normal_frames_);
}

// 添加相对位姿
void LidarLocalization::addRelPose(const Pose &pose) {
  rel_mutex_.lock();
  // 将位姿插入相对位姿映射中
  rel_poses_map_.insert(std::make_pair(pose.timestamp, pose));
  rel_mutex_.unlock();
}

void LidarLocalization::setManualPose(const Pose &pose) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  last_lidar_pose_ = pose;
  last_lidar_pose_.q.normalize();
  last_lidar_pose_.source = pose.source.empty() ? "manual_initialpose" : pose.source;
  status_ = STATUS::LOW_ACCURACY;
  last_lidar_pose_.setStatus(status_);
  init_position_ = pose.xyz;
  init_orientation_ = pose.q;
  if (last_lidar_pose_.source == "global_relocalization") {
    lidar_matcher_->setRecoveryAnchor(last_lidar_pose_);
    recovery_pose_prior_tracking_ = true;
    recovery_pose_prior_normal_frames_ = 0;
  } else {
    lidar_matcher_->clearRecoveryAnchor();
    recovery_pose_prior_tracking_ = false;
    recovery_pose_prior_normal_frames_ = 0;
  }
}

// 添加激光雷达数据: 主流程
void LidarLocalization::addLidarData(const pcl::PointCloud<RsPointXYZIRT>::Ptr &lidar_cloud) {
  double lidar_time = lidar_cloud->header.stamp * 1e-6;
  if (debug_print_) {
    std::cout << "lidar timestamp :" << std::fixed << lidar_time << std::endl;
  }
  // 变换到车体
  pcl::PointCloud<RsPointXYZIRT>::Ptr lidar_cloud_base(new pcl::PointCloud<RsPointXYZIRT>);
  pcl::transformPointCloud(*lidar_cloud, *lidar_cloud_base, T_base_lidar_);
  pcl::PointCloud<RsPointXYZIRT>::Ptr lidar_cloud_base_ptr(new pcl::PointCloud<RsPointXYZIRT>);
  //check input lidar_cloud_base->points.size()
  if (lidar_cloud_base->points.size() < input_cloud_size_thr_) {
    LERROR << "addLidarData : INPUT CLOUD SIZE SMALL TAHN 1000: " << lidar_cloud_base->points.size()
           << REND;
    return;
  }
  //////////////////////////////////////
  lidar_cloud_base_ptr->reserve(lidar_cloud_base->points.size());
  lidar_cloud_base_ptr->header = lidar_cloud_base->header;
  for (int i = 0; i < lidar_cloud_base->points.size(); i++) {
    auto pt = lidar_cloud_base->points[i];
    if (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z < blind_distance_*blind_distance_ || pt.x * pt.x + pt.y * pt.y + pt.z * pt.z > 900.0 || pt.z > 10) {
      continue;
    }
    lidar_cloud_base_ptr->points.emplace_back(pt);
  }
  Pose result_pose;
  result_pose.timestamp = lidar_time;

  // 判断是否存在相对位姿及时间戳是否可以匹配
  rel_mutex_.lock();
  if (rel_poses_map_.size() < 2 || lidar_time < rel_poses_map_.begin()->first) {
    if (debug_print_) {
      std::cout << "LiDAR earlier than first rel_tf, skip this scan" << std::endl;
    }
    rel_mutex_.unlock();
    return;
  }
  rel_mutex_.unlock();
  // 点云去畸变
  PointCloudT::Ptr undistorted_cloud(new PointCloudT);
  Eigen::Affine3d pose_undistort;
  if (!undistortPointCloud(lidar_cloud_base_ptr, undistorted_cloud, pose_undistort))
    return;
  // 去除nan点
  undistorted_cloud->is_dense = false;
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*undistorted_cloud, *undistorted_cloud, indices);
  undistorted_cloud->height = 1;
  undistorted_cloud->width = undistorted_cloud->points.size();
  PointCloudT::Ptr semantic_cloud(new PointCloudT);
  semanticFilter(undistorted_cloud, semantic_cloud);
  AlignInfo matcher_align_info;
  if (debug_print_) {
    std::cout << "semantic_cloud size: " << semantic_cloud->points.size() << std::endl;
  }
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  switch (status_) {
    case STATUS::IDLE:
    case STATUS::LOST: {
      // 初始化
      Pose init_lidar_pose;
      init_lidar_pose.xyz = init_position_;
      init_lidar_pose.q = init_orientation_;
      init_lidar_pose.timestamp = lidar_time;
      init_lidar_pose.source = "configured_initial_pose";
      bool ret_align =
          lidar_matcher_->align(semantic_cloud, kdtree_ptr_, map_cloud_ptr_,
                                init_lidar_pose, result_pose, lidar_time);
      if (!ret_align) {
        result_pose = init_lidar_pose;
        result_pose.source = "configured_initial_pose_unverified";
      } else {
        result_pose.source = "ro_initial_alignment";
      }
      if (debug_print_) {
        std::cout << "init_lidar_pose: \n" << result_pose.transform() << std::endl;
      }
      last_lidar_pose_ = result_pose;
      status_ = STATUS::LOW_ACCURACY;
      /////加入status
      result_pose.setStatus(status_);
      //////////
      break;
    }
    case STATUS::LOW_ACCURACY:
    case STATUS::LOW_ACCURACY_RPZ:
    case STATUS::NORMAL: {
      Pose init_pose;
      Pose origin_init_pose;
      //init_pose: 从last_lidar_pose_外推的位姿 origin_init_pose: 原始相对定位pose
      if (forwardPropagate(last_lidar_pose_, lidar_time, init_pose, origin_init_pose)) {
        RSTicToc time_c0{"cloud align"};
        time_c0.tic();
        bool ret_align = lidar_matcher_->align(
            semantic_cloud, kdtree_ptr_, map_cloud_ptr_, init_pose, result_pose,lidar_time);
        matcher_align_info = lidar_matcher_->getAlignInfo();
        double match_score = matcher_align_info.point_pair_distance_residual; ///残差
        double valid_pair_ratio = matcher_align_info.ceres_valid_pair_ratio;
        if (debug_print_) {
          std::cout << "valid_pair_ratio: " << valid_pair_ratio << std::endl;
        }
        bool ret_score = match_score < MATCHING_CHECK_RESIDUAL_THRESH;
        bool valid_pair_score =
            valid_pair_ratio > MATCHING_CHECK_VALID_PAIR_RATIO_THRESH;
        Eigen::Matrix4d delta_from_prior =
            init_pose.transform().inverse() * result_pose.transform();
        double prior_delta_xy =
            delta_from_prior.block<2, 1>(0, 3).norm();
        double prior_delta_z = std::fabs(delta_from_prior(2, 3));
        double prior_delta_yaw =
            std::fabs(std::atan2(delta_from_prior(1, 0),
                                 delta_from_prior(0, 0))) *
            R2D;
        bool prior_score = prior_delta_xy < MATCHING_CHECK_PRIOR_XY_THRESH &&
                           prior_delta_z < MATCHING_CHECK_PRIOR_Z_THRESH &&
                           prior_delta_yaw < MATCHING_CHECK_PRIOR_YAW_THRESH_DEG;
        time_c0.tocAndGetTime();
        if (debug_print_) {
          std::cout << "match_score: " << match_score
                    << " prior_delta_xy: " << prior_delta_xy
                    << " prior_delta_z: " << prior_delta_z
                    << " prior_delta_yaw: " << prior_delta_yaw << std::endl;
          std::cout << "ret_align: " << ret_align << " ret_score: " << ret_score
                    << " valid_pair_score: " << valid_pair_score
                    << " prior_score: " << prior_score << std::endl;
        }
        if (ret_align && ret_score && valid_pair_score && prior_score) {
          status_ = STATUS::NORMAL;
          result_pose.source = "ro_normal_match";
          result_pose.setStatus(status_);
          last_lidar_pose_ = result_pose;
          if (recovery_pose_prior_tracking_) {
            ++recovery_pose_prior_normal_frames_;
            if (recovery_pose_prior_normal_frames_ >=
                recovery_pose_prior_release_normal_frames_) {
              lidar_matcher_->clearRecoveryAnchor();
              recovery_pose_prior_tracking_ = false;
            }
          }
        } else {
          status_ = STATUS::LOW_ACCURACY;
          result_pose = init_pose;
          result_pose.source = "ro_low_accuracy_prediction";
          result_pose.setStatus(status_);
          last_lidar_pose_ = result_pose;
          recovery_pose_prior_normal_frames_ = 0;
        }
      }
      break;
    }
  }
#ifdef ROS1
  static ros::NodeHandle nh_;
  static ros::Publisher pub_cloud =
      nh_.advertise<sensor_msgs::PointCloud2>("/lidar_world_cloud", 1);
  sensor_msgs::PointCloud2 world_cloud_msg;
  PointCloudT::Ptr world_cloud_ptr(new PointCloudT());
  auto T = result_pose.transform();
  pcl::transformPointCloud(*semantic_cloud, *world_cloud_ptr, T.cast<float>());
  pcl::toROSMsg(*world_cloud_ptr, world_cloud_msg);
  world_cloud_msg.header.stamp = ros::Time::now();
  world_cloud_msg.header.frame_id = "rslidar";
  pub_cloud.publish(world_cloud_msg);
#endif

#ifdef ROS2
  if (is_pub_cloud_) {
    static auto nh_ = rclcpp::Node::make_shared("align_pc_pub_node");
    static auto pub_cloud = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_world_cloud", 1);
    sensor_msgs::msg::PointCloud2 world_cloud_msg;
    PointCloudT::Ptr world_cloud_ptr(new PointCloudT());
    auto T = result_pose.transform();
    pcl::transformPointCloud(*semantic_cloud, *world_cloud_ptr, T.cast<float>());
    pcl::toROSMsg(*world_cloud_ptr, world_cloud_msg);
    world_cloud_msg.header.stamp = rclcpp::Clock().now();
    world_cloud_msg.header.frame_id = "rslidar";
    pub_cloud->publish(world_cloud_msg);
  }
#endif

  for (const auto &cb : callbacks_) {
    cb(result_pose);
  }
}

bool LidarLocalization::initMap() {
  // 加载地图
  const auto configured_map_path = config_node_["map_path"].as<std::string>();
  std::filesystem::path map_path(configured_map_path);
  if (!map_path.is_absolute()) {
    map_path = std::filesystem::path(PACKAGE_PATH) / configured_map_path;
  }
  map_path_ = map_path.string();
  pcl::PointCloud<MapPointT>::Ptr map_ptr_rgb(new pcl::PointCloud<MapPointT>);
  if (pcl::io::loadPCDFile<MapPointT>(map_path_, *map_ptr_rgb) == -1) {
    std::cout << "Failed to load map: " << map_path_ << std::endl;
    return false;
  }
  std::cout << "Loaded map: " << map_path_ << std::endl;
  pcl::PointCloud<PointT>::Ptr map_ptr(new pcl::PointCloud<PointT>);
  for(int i = 0; i < map_ptr_rgb->points.size(); i++) {
    auto &pt = map_ptr_rgb->points[i];
    PointT p_tmp;
    p_tmp.x = pt.x;
    p_tmp.y = pt.y;
    p_tmp.z = pt.z;
    map_ptr->points.push_back(p_tmp);
  }

  if(is_pub_map_) {
#ifdef ROS1
    static ros::NodeHandle nh_;
    static ros::Publisher pub_map = nh_.advertise<sensor_msgs::PointCloud2>("/map_cloud", 1);
    std::this_thread::sleep_for(std::chrono::seconds(1)); // wait for node to be ready
    sensor_msgs::PointCloud2 map_cloud_msg;
    pcl::toROSMsg(*map_ptr_rgb, map_cloud_msg);
    map_cloud_msg.header.stamp = ros::Time::now();
    map_cloud_msg.header.frame_id = "rslidar";
    pub_map.publish(map_cloud_msg);
    std::cout << "ROS1 map published" << std::endl;
#endif
#ifdef ROS2
    static auto nh_ = rclcpp::Node::make_shared("map_pub_node");
    static auto pub_map = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("/map_cloud", 1);
    sensor_msgs::msg::PointCloud2 map_cloud_msg;
    pcl::toROSMsg(*map_ptr_rgb, map_cloud_msg);
    map_cloud_msg.header.stamp = rclcpp::Clock().now();
    map_cloud_msg.header.frame_id = "rslidar";
    pub_map->publish(map_cloud_msg);
    std::cout << "map published" << std::endl;
#endif
  }

  map_ptr->is_dense = false;
  map_ptr->height = 1;
  map_ptr->width = map_ptr->points.size();
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*map_ptr, *map_ptr, indices);
  // 地图下采样。默认保持原始接入行为，bag/调参时可通过 YAML 覆盖。
  const double map_voxel_leaf_size =
      config_node_["map_voxel_leaf_size"] ? config_node_["map_voxel_leaf_size"].as<double>() : 0.3;
  map_cloud_ptr_.reset(new pcl::PointCloud<PointT>());
  pcl::VoxelGrid<PointT> voxel_filter;
  voxel_filter.setInputCloud(map_ptr);
  voxel_filter.setLeafSize(map_voxel_leaf_size, map_voxel_leaf_size, map_voxel_leaf_size);
  voxel_filter.filter(*map_cloud_ptr_);
  map_cloud_ptr_->height = 1;
  map_cloud_ptr_->width = map_cloud_ptr_->points.size();
  // Calculate normals for map points
  pcl::NormalEstimation<PointT, PointT> normal_estimator;
  normal_estimator.setInputCloud(map_cloud_ptr_);
  pcl::search::KdTree<PointT>::Ptr normal_tree(new pcl::search::KdTree<PointT>());
  normal_estimator.setSearchMethod(normal_tree);
  normal_estimator.setKSearch(10); // Use 10 nearest neighbors
  normal_estimator.compute(*map_cloud_ptr_);
  // 构建地图KDtree
  kdtree_ptr_.reset(new pcl::KdTreeFLANN<PointT>());
  kdtree_ptr_->setInputCloud(map_cloud_ptr_);
  std::cout << "map voxel leaf size: " << map_voxel_leaf_size << std::endl;
  std::cout << "map size: " << map_cloud_ptr_->size() << std::endl;
  return true;
}

bool LidarLocalization::undistortPointCloud(
    const pcl::PointCloud<RsPointXYZIRT>::Ptr lidar_cloud,
    PointCloudT::Ptr &undistorted_cloud, Eigen::Affine3d &pose_undistort) {
  rel_mutex_.lock();
  if (rel_poses_map_.size() < 2) {
    if (debug_print_) {
      std::cout << "No enough relative pose, skip this scan" << std::endl;
    }
    rel_mutex_.unlock();
    return false;
  }

  Pose rel_tf_begin;
  Pose rel_tf_end;
  bool ret1 = interpolate(rel_poses_map_, lidar_cloud->points.front().timestamp,
                          rel_tf_begin);
  bool ret2 = interpolate(rel_poses_map_, lidar_cloud->points.back().timestamp,
                          rel_tf_end);
  rel_mutex_.unlock();

  Eigen::Affine3d dst = Eigen::Affine3d::Identity();
  if (ret1 && ret2) {
    pointsUndistort(lidar_cloud, undistorted_cloud, rel_tf_begin, rel_tf_end, 0,
                    dst, true);
    undistorted_cloud->header = lidar_cloud->header;
    pose_undistort = dst;
    return true;
  } else {
    return false;
  }
}

void LidarLocalization::semanticFilter(const PointCloudT::Ptr &undistorted_cloud, PointCloudT::Ptr &semantic_cloud) {
  for (int i = 0; i < undistorted_cloud->points.size(); ++i) {
    auto &pt = undistorted_cloud->points[i];
    pt.curvature = 1; // record as new points (current scan)
    if (pt.z < 0.2) {
      pt.intensity = -1; // ground
      if (pt.y > -10 && pt.y < 10 && pt.x > -50 && pt.x < 150) {
        pt.intensity = -2; // special ground
      }
    } else {
      if (pt.z > 1.5) {
        pt.intensity = 2;
      }
      else
        pt.intensity = 1; // non ground
    }
    semantic_cloud->points.push_back(pt);
  }

  semantic_cloud = voxelGridFilter(semantic_cloud, source_voxel_leaf_size_);
  semantic_cloud->header = undistorted_cloud->header;
  semantic_cloud->height = 1;
  semantic_cloud->width = semantic_cloud->points.size();
}

bool LidarLocalization::forwardPropagate(const Pose &last_pose, double t,
                                         Pose &pose_at_t,Pose& origin_pose_at_t) {
  if (last_pose.timestamp > t)
    return false;

  rel_mutex_.lock();
  Pose rel_pose_begin;
  if (!interpolate(rel_poses_map_, last_pose.timestamp, rel_pose_begin)) {
    if (debug_print_) {
      std::cout << "FP: failed getting rel_pose_begin" << std::endl;
    }
    rel_mutex_.unlock();
    return false;
  }

  Pose rel_pose_t;
  if (!interpolate(rel_poses_map_, t, rel_pose_t)) {
    if (debug_print_) {
      std::cout << "FP: failed getting rel_pose_t" << std::endl;
    }
    rel_mutex_.unlock();
    return false;
  }
  rel_mutex_.unlock();

  auto T_b = rel_pose_begin.transform();
  auto T_t = rel_pose_t.transform();
  auto T = T_b.inverse() * T_t;

  Eigen::Matrix4d T_pose_at_t = last_pose.transform() * T;
  pose_at_t = Pose(T_pose_at_t, t);
  pose_at_t.timestamp = t;
  origin_pose_at_t = rel_pose_t;
  origin_pose_at_t.timestamp = t;
  return true;
}

PointCloudT::Ptr LidarLocalization::voxelGridFilter(const PointCloudT::Ptr &cloud_ptr,
                                   float leaf_size) {
  pcl::VoxelGrid<PointT> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_grid_filter.setInputCloud(cloud_ptr);
  PointCloudT::Ptr filtered_ptr(new PointCloudT);
  voxel_grid_filter.filter(*filtered_ptr);
  return filtered_ptr;
}
