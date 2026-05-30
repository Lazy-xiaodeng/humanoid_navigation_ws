/**
 * @file lidar_localization_component.cpp
 * @brief 基于PCL的激光雷达定位ROS2组件实现文件
 * 
 * 本文件实现了使用PCL库进行激光雷达配准定位的核心功能
 * 支持NDT、GICP等多种配准算法，支持IMU去畸变、里程计预测等功能
 */

#include <cmath>
#include <sstream>

#include <lidar_localization/lidar_localization_component.hpp>
#include <pcl/common/transforms.h> // ★ 新增：用于点云坐标系转换的头文件

namespace
{
double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}
}  // namespace

/**
 * @brief 构造函数，声明所有ROS参数及其默认值
 * 
 * @param options ROS节点选项
 * 
 * 在此处声明的所有参数都可以在launch文件或yaml配置文件中覆盖
 * 参数分为几大类：
 * - 坐标系相关：global_frame_id, odom_frame_id, base_frame_id
 * - 配准算法相关：registration_method, score_threshold, ndt_*等
 * - 点云处理相关：voxel_leaf_size, scan_max_range, scan_min_range
 * - 初始位姿相关：set_initial_pose, initial_pose_*
 * - 传感器融合相关：use_odom, use_imu
 */
PCLLocalization::PCLLocalization(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("lidar_localization", options),  // 使用LifecycleNode支持生命周期管理
  clock_(RCL_ROS_TIME),  // 使用ROS时间
  tfbuffer_(std::make_shared<rclcpp::Clock>(clock_)),  // TF缓冲区，用于坐标变换查询
  tflistener_(tfbuffer_),  // TF监听器，用于接收坐标变换
  broadcaster_(this)  // TF广播器，用于发布坐标变换
{
  // ========== 坐标系参数 ==========
  declare_parameter("global_frame_id", "map");    // 全局坐标系ID，通常为"map"
  declare_parameter("odom_frame_id", "odom");     // 里程计坐标系ID
  declare_parameter("base_frame_id", "odom");     // 机器人基坐标系ID，通常设置为"odom"或"base_link"
  declare_parameter("initialpose_base_frame_id", "base_footprint");
  
  // ========== 配准方法参数 ==========
  declare_parameter("registration_method", "NDT");  // 配准方法：NDT, NDT_OMP, GICP, GICP_OMP
  declare_parameter("score_threshold", 2.0);        // 配准得分阈值（fitness score），超过此值认为配准不可靠
  declare_parameter("reject_pose_jump", true);
  declare_parameter("max_pose_jump_translation", 0.8);
  declare_parameter("max_pose_jump_yaw", 0.45);
  declare_parameter("initialpose_relax_duration_sec", 0.0);
  declare_parameter("initialpose_max_pose_jump_translation", 2.0);
  declare_parameter("initialpose_max_pose_jump_yaw", 1.0);
  declare_parameter("pose_jump_reacquire_enabled", false);
  declare_parameter("pose_jump_reacquire_max_translation", 1.5);
  declare_parameter("pose_jump_reacquire_max_yaw", 0.12);
  declare_parameter("pose_jump_reacquire_max_fitness", 0.02);
  declare_parameter("pose_jump_reacquire_required_frames", 2);
  declare_parameter("pose_jump_reacquire_xy_tolerance", 0.35);
  declare_parameter("pose_jump_reacquire_yaw_tolerance", 0.12);
  declare_parameter("candidate_confirmation_enabled", false);
  declare_parameter("candidate_confirmation_min_translation", 0.20);
  declare_parameter("candidate_confirmation_min_yaw", 0.08);
  declare_parameter("candidate_confirmation_max_fitness", 0.08);
  declare_parameter("candidate_confirmation_max_mean_corr_dist", 0.0);
  declare_parameter("candidate_confirmation_required_frames", 2);
  declare_parameter("candidate_confirmation_xy_tolerance", 0.20);
  declare_parameter("candidate_confirmation_yaw_tolerance", 0.06);
  declare_parameter("rotation_guard_enabled", false);
  declare_parameter("rotation_guard_use_cmd_vel_fallback", false);
  declare_parameter("rotation_guard_navigation_status_topic", "/navigation/status");
  declare_parameter("rotation_guard_angular_threshold", 0.20);
  declare_parameter("rotation_guard_linear_threshold", 0.05);
  declare_parameter("rotation_guard_settle_sec", 1.0);
  declare_parameter("rotation_guard_max_duration_sec", 8.0);
  declare_parameter("multi_frame_matching_enabled", false);
  declare_parameter("multi_frame_use_only_when_rotating", true);
  declare_parameter("multi_frame_window_sec", 0.6);
  declare_parameter("multi_frame_max_frames", 8);
  declare_parameter("multi_frame_voxel_leaf_size", 0.20);
  declare_parameter("multi_frame_max_points", 40000);
  declare_parameter("multi_frame_keyframe_filter_enabled", false);
  declare_parameter("multi_frame_keyframe_translation_threshold", 0.10);
  declare_parameter("multi_frame_keyframe_yaw_threshold", 0.052);
  declare_parameter("multi_frame_keyframe_max_interval_sec", 0.25);
  declare_parameter("ndt_resolution", 1.0);         // NDT算法的体素网格分辨率（米），控制NDT网格大小
  declare_parameter("ndt_step_size", 0.1);          // NDT算法的牛顿迭代步长，越大收敛越快但可能不稳定
  declare_parameter("ndt_max_iterations", 35);      // 配准算法最大迭代次数
  declare_parameter("ndt_num_threads", 4);          // OMP版本的线程数，>0时使用指定线程数，<=0时使用最大可用线程数
  declare_parameter("transform_epsilon", 0.01);     // 变换收敛阈值，两次迭代间变换小于此值认为收敛
  declare_parameter("ndt_outlier_ratio", 0.55);     // NDT离群点比率: 越低约束越强, 标准PCL=0.35; 0.55=更宽容
  declare_parameter("ndt_max_corr_dist", 0.0);      // NDT最大关联距离(m): 0=禁用; 设置2.0可剔除远距离错误关联
  declare_parameter("ndt_rotation_prior_enabled", false);  // 是否启用 roll/pitch 先验约束
  declare_parameter("ndt_rotation_prior_weight", 0.0);     // roll/pitch 先验权重: 10~20 推荐
  declare_parameter("ndt_rotation_prior_roll_pitch_only", true); // true=仅约束roll/pitch(yaw留给NDT)
  
  // ========== 点云滤波参数 ==========
  declare_parameter("voxel_leaf_size", 0.2);        // 体素滤波叶子大小（米），用于降采样，越大点越少
  
  // ========== 点云范围参数 ==========
  declare_parameter("scan_max_range", 100.0);       // 点云最大有效距离（米），超出此范围的点将被过滤
  declare_parameter("scan_min_range", 1.0);         // 点云最小有效距离（米），近距离盲区过滤
  declare_parameter("min_scan_points", 50);         // NDT匹配前最少有效点数，空点云/过少点云直接拒绝
  declare_parameter("scan_period", 0.1);            // 雷达扫描周期（秒），10Hz雷达为0.1，用于IMU去畸变
  
  // ========== 地图参数 ==========
  declare_parameter("use_pcd_map", true);          // 是否启动时加载PCD地图文件
  declare_parameter("map_path", "/home/ubuntu/humanoid_ws/src/humanoid_navigation2/pcd/hall.pcd");    // PCD地图文件路径
  
  // ========== 初始位姿参数 ==========
  declare_parameter("set_initial_pose", false);     // 是否启动时设置初始位姿
  declare_parameter("initial_pose_x", 0.0);         // 初始位置X（米）
  declare_parameter("initial_pose_y", 0.0);         // 初始位置Y（米）
  declare_parameter("initial_pose_z", 0.0);         // 初始位置Z（米）
  declare_parameter("initial_pose_qx", 0.0);        // 初始四元数X分量
  declare_parameter("initial_pose_qy", 0.0);        // 初始四元数Y分量
  declare_parameter("initial_pose_qz", 0.0);        // 初始四元数Z分量
  declare_parameter("initial_pose_qw", 1.0);        // 初始四元数W分量（默认无旋转）
  
  // ========== 传感器融合参数 ==========
  declare_parameter("use_odom", false);   // 是否使用里程计数据进行位姿预测
  declare_parameter("use_imu", false);    // 是否使用IMU数据进行点云去畸变
  declare_parameter("enable_debug", false);  // 是否启用调试信息输出

  // ========== 导航输出约束参数 ==========
  declare_parameter("force_2d_pose", false);
  declare_parameter("force_2d_fixed_z", true);
  declare_parameter("force_2d_z", 0.0);
  declare_parameter("republish_last_good_tf_on_failure", true);
  declare_parameter("max_last_good_tf_age_sec", 3.0);
  declare_parameter("localization_status_topic", "/localization/ndt_status");

  // ========== Fast-LIO delta guess 参数 (Plan B) ==========
  declare_parameter("use_fastlio_delta_guess", false);
  declare_parameter("fastlio_delta_guess_mode", "disabled");
  declare_parameter("fastlio_camera_frame", "camera_init");
  declare_parameter("fastlio_body_frame", "body");
  declare_parameter("tf_max_stamp_mismatch_sec", 0.2);
  declare_parameter("fastlio_max_delta_translation", 0.20);
  declare_parameter("fastlio_max_delta_yaw", 0.25);
  declare_parameter("fastlio_max_delta_dt", 0.50);
  declare_parameter("fastlio_max_dead_reckon_sec", 2.0);
}

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @brief 节点配置回调，在节点进入inactive状态前调用
 * 
 * 此阶段进行参数初始化、发布订阅创建、配准算法初始化等准备工作
 * 
 * @return CallbackReturn 返回SUCCESS表示配置成功
 */
CallbackReturn PCLLocalization::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  initializeParameters();     // 从参数服务器获取所有参数值
  initializePubSub();         // 创建ROS发布者和订阅者
  initializeRegistration();   // 初始化配准算法

  path_ptr_ = std::make_shared<nav_msgs::msg::Path>();  // 创建路径消息对象
  path_ptr_->header.frame_id = global_frame_id_;        // 设置路径消息的坐标系

  RCLCPP_INFO(get_logger(), "Configuring end");
  return CallbackReturn::SUCCESS;
}

/**
 * @brief 节点激活回调，在节点进入active状态前调用
 * 
 * 此阶段激活所有发布者，加载初始地图，设置初始位姿
 * 节点进入active状态后开始正常处理数据
 * 
 * @return CallbackReturn 返回SUCCESS表示激活成功
 */
CallbackReturn PCLLocalization::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // 激活所有发布者
  pose_pub_->on_activate();
  path_pub_->on_activate();
  initial_map_pub_->on_activate();
  status_pub_->on_activate();

  // 如果配置了启动时设置初始位姿，则发布初始位姿
  if (set_initial_pose_) {
    auto msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();

    msg->header.stamp = now();
    msg->header.frame_id = global_frame_id_;
    msg->pose.pose.position.x = initial_pose_x_;
    msg->pose.pose.position.y = initial_pose_y_;
    msg->pose.pose.position.z = initial_pose_z_;
    msg->pose.pose.orientation.x = initial_pose_qx_;
    msg->pose.pose.orientation.y = initial_pose_qy_;
    msg->pose.pose.orientation.z = initial_pose_qz_;
    msg->pose.pose.orientation.w = initial_pose_qw_;

    // 将初始位姿添加到路径消息中
    geometry_msgs::msg::PoseStamped::SharedPtr pose_stamped(new geometry_msgs::msg::PoseStamped);
    pose_stamped->header.stamp = msg->header.stamp;
    pose_stamped->header.frame_id = global_frame_id_;
    pose_stamped->pose = msg->pose.pose;
    path_ptr_->poses.push_back(*pose_stamped);

    initialPoseReceived(msg);  // 处理初始位姿
  }

  // 如果配置了使用PCD地图，则启动时加载地图
  if (use_pcd_map_) {
    // 从文件加载PCD地图
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(map_path_, *map_cloud_ptr);
    RCLCPP_INFO(get_logger(), "Map Size %ld", map_cloud_ptr->size());

    // ★★★ 核心修改 1：把 PCD 地图从 LiDAR 坐标系(Z朝后) 掰平到 ROS标准 坐标系(Z朝上) ★★★
    // 设定逆变换四元数，将倾斜的点云转平。注意 Eigen 的构造函数顺序是 (w, x, y, z) 
    Eigen::Quaternionf q_cam_to_ros(0.5, -0.5, -0.5, 0.5); 
    Eigen::Vector3f t_cam_to_ros(0.0, 0.0, 0.0);
    pcl::transformPointCloud(*map_cloud_ptr, *map_cloud_ptr, t_cam_to_ros, q_cam_to_ros);
    RCLCPP_INFO(get_logger(), "PCD Map transformed to ROS standard frame (Z-up)");
    // ★★★ 转换结束 ★★★

    // 将点云转换为ROS消息并发布
    sensor_msgs::msg::PointCloud2::SharedPtr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*map_cloud_ptr, *map_msg_ptr);
    map_msg_ptr->header.frame_id = global_frame_id_;
    initial_map_pub_->publish(*map_msg_ptr);
    RCLCPP_INFO(get_logger(), "Initial Map Published");

    // 根据配准方法类型决定是否对地图进行体素滤波
    // GICP类算法计算协方差较慢，对大地图建议先滤波
    if (registration_method_ == "GICP" || registration_method_ == "GICP_OMP") {
      pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
      voxel_grid_filter_.setInputCloud(map_cloud_ptr);
      voxel_grid_filter_.filter(*filtered_cloud_ptr);
      registration_->setInputTarget(filtered_cloud_ptr);
    } else {
      // NDT类算法使用原始地图即可
      registration_->setInputTarget(map_cloud_ptr);
    }

    map_recieved_ = true;  // 标记地图已加载
  }

  RCLCPP_INFO(get_logger(), "Activating end");
  return CallbackReturn::SUCCESS;
}

/**
 * @brief 节点停用回调，在节点从active状态转换时调用
 * 
 * 停用所有发布者，节点暂时不处理数据但仍保留配置
 * 
 * @return CallbackReturn 返回SUCCESS表示停用成功
 */
CallbackReturn PCLLocalization::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  pose_pub_->on_deactivate();
  path_pub_->on_deactivate();
  initial_map_pub_->on_deactivate();
  status_pub_->on_deactivate();

  RCLCPP_INFO(get_logger(), "Deactivating end");
  return CallbackReturn::SUCCESS;
}

/**
 * @brief 节点清理回调，在节点从inactive状态转换时调用
 * 
 * 释放所有资源，重置订阅者和发布者，节点回到unconfigured状态
 * 
 * @return CallbackReturn 返回SUCCESS表示清理成功
 */
CallbackReturn PCLLocalization::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning Up");
  initial_pose_sub_.reset();
  initial_map_pub_.reset();
  path_pub_.reset();
  pose_pub_.reset();
  status_pub_.reset();
  odom_sub_.reset();
  cloud_sub_.reset();
  imu_sub_.reset();
  cmd_vel_sub_.reset();
  navigation_status_sub_.reset();

  RCLCPP_INFO(get_logger(), "Cleaning Up end");
  return CallbackReturn::SUCCESS;
}

/**
 * @brief 节点关闭回调
 * 
 * 节点准备退出，进行最后的清理工作
 * 
 * @param state 当前生命周期状态
 * @return CallbackReturn 返回SUCCESS表示关闭成功
 */
CallbackReturn PCLLocalization::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Shutting Down from %s", state.label().c_str());

  return CallbackReturn::SUCCESS;
}

/**
 * @brief 节点错误处理回调
 * 
 * 节点进入error状态时调用，通常记录错误信息
 * 
 * @param state 当前生命周期状态
 * @return CallbackReturn 返回SUCCESS
 */
CallbackReturn PCLLocalization::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_FATAL(get_logger(), "Error Processing from %s", state.label().c_str());

  return CallbackReturn::SUCCESS;
}

/**
 * @brief 初始化参数，从参数服务器获取所有参数值
 * 
 * 在构造函数中declare_parameter只是声明参数及默认值
 * 实际参数值需要在节点配置后通过get_parameter获取
 */
void PCLLocalization::initializeParameters()
{
  RCLCPP_INFO(get_logger(), "initializeParameters");
  
  // 获取坐标系参数
  get_parameter("global_frame_id", global_frame_id_);  // 全局坐标系（地图坐标系）
  get_parameter("odom_frame_id", odom_frame_id_);      // 里程计坐标系
  get_parameter("base_frame_id", base_frame_id_);      // 机器人基坐标系
  get_parameter("initialpose_base_frame_id", initialpose_base_frame_id_);
  get_parameter("localization_status_topic", localization_status_topic_);
  
  // 获取配准算法参数
  get_parameter("registration_method", registration_method_);  // 配准方法名称
  get_parameter("score_threshold", score_threshold_);          // 配准得分阈值
  get_parameter("reject_pose_jump", reject_pose_jump_);
  get_parameter("max_pose_jump_translation", max_pose_jump_translation_);
  get_parameter("max_pose_jump_yaw", max_pose_jump_yaw_);
  get_parameter("initialpose_relax_duration_sec", initialpose_relax_duration_sec_);
  get_parameter("initialpose_max_pose_jump_translation", initialpose_max_pose_jump_translation_);
  get_parameter("initialpose_max_pose_jump_yaw", initialpose_max_pose_jump_yaw_);
  get_parameter("pose_jump_reacquire_enabled", pose_jump_reacquire_enabled_);
  get_parameter("pose_jump_reacquire_max_translation", pose_jump_reacquire_max_translation_);
  get_parameter("pose_jump_reacquire_max_yaw", pose_jump_reacquire_max_yaw_);
  get_parameter("pose_jump_reacquire_max_fitness", pose_jump_reacquire_max_fitness_);
  get_parameter("pose_jump_reacquire_required_frames", pose_jump_reacquire_required_frames_);
  get_parameter("pose_jump_reacquire_xy_tolerance", pose_jump_reacquire_xy_tolerance_);
  get_parameter("pose_jump_reacquire_yaw_tolerance", pose_jump_reacquire_yaw_tolerance_);
  get_parameter("candidate_confirmation_enabled", candidate_confirmation_enabled_);
  get_parameter("candidate_confirmation_min_translation", candidate_confirmation_min_translation_);
  get_parameter("candidate_confirmation_min_yaw", candidate_confirmation_min_yaw_);
  get_parameter("candidate_confirmation_max_fitness", candidate_confirmation_max_fitness_);
  get_parameter("candidate_confirmation_max_mean_corr_dist", candidate_confirmation_max_mean_corr_dist_);
  get_parameter("candidate_confirmation_required_frames", candidate_confirmation_required_frames_);
  get_parameter("candidate_confirmation_xy_tolerance", candidate_confirmation_xy_tolerance_);
  get_parameter("candidate_confirmation_yaw_tolerance", candidate_confirmation_yaw_tolerance_);
  get_parameter("rotation_guard_enabled", rotation_guard_enabled_);
  get_parameter("rotation_guard_use_cmd_vel_fallback", rotation_guard_use_cmd_vel_fallback_);
  get_parameter("rotation_guard_navigation_status_topic", rotation_guard_navigation_status_topic_);
  get_parameter("rotation_guard_angular_threshold", rotation_guard_angular_threshold_);
  get_parameter("rotation_guard_linear_threshold", rotation_guard_linear_threshold_);
  get_parameter("rotation_guard_settle_sec", rotation_guard_settle_sec_);
  get_parameter("rotation_guard_max_duration_sec", rotation_guard_max_duration_sec_);
  get_parameter("multi_frame_matching_enabled", multi_frame_matching_enabled_);
  get_parameter("multi_frame_use_only_when_rotating", multi_frame_use_only_when_rotating_);
  get_parameter("multi_frame_window_sec", multi_frame_window_sec_);
  get_parameter("multi_frame_max_frames", multi_frame_max_frames_);
  get_parameter("multi_frame_voxel_leaf_size", multi_frame_voxel_leaf_size_);
  get_parameter("multi_frame_max_points", multi_frame_max_points_);
  get_parameter("multi_frame_keyframe_filter_enabled", multi_frame_keyframe_filter_enabled_);
  get_parameter("multi_frame_keyframe_translation_threshold", multi_frame_keyframe_translation_threshold_);
  get_parameter("multi_frame_keyframe_yaw_threshold", multi_frame_keyframe_yaw_threshold_);
  get_parameter("multi_frame_keyframe_max_interval_sec", multi_frame_keyframe_max_interval_sec_);
  get_parameter("ndt_resolution", ndt_resolution_);     // NDT网格分辨率
  get_parameter("ndt_step_size", ndt_step_size_);       // NDT牛顿迭代步长
  get_parameter("ndt_num_threads", ndt_num_threads_);   // OMP线程数
  get_parameter("ndt_max_iterations", ndt_max_iterations_);  // 最大迭代次数
  get_parameter("transform_epsilon", transform_epsilon_);    // 变换收敛阈值
  
  // 获取点云处理参数
  get_parameter("voxel_leaf_size", voxel_leaf_size_);   // 体素滤波叶子大小
  get_parameter("ndt_outlier_ratio", ndt_outlier_ratio_);
  get_parameter("ndt_max_corr_dist", ndt_max_corr_dist_);
  get_parameter("ndt_rotation_prior_enabled", ndt_rotation_prior_enabled_);
  get_parameter("ndt_rotation_prior_weight", ndt_rotation_prior_weight_);
  get_parameter("ndt_rotation_prior_roll_pitch_only", ndt_rotation_prior_roll_pitch_only_);
  get_parameter("scan_max_range", scan_max_range_);     // 点云最大距离
  get_parameter("scan_min_range", scan_min_range_);     // 点云最小距离
  get_parameter("min_scan_points", min_scan_points_);   // NDT最少有效点数
  get_parameter("scan_period", scan_period_);           // 雷达扫描周期
  if (min_scan_points_ < 1) {
    RCLCPP_WARN(
      get_logger(),
      "min_scan_points=%d is invalid; clamping to 1.",
      min_scan_points_);
    min_scan_points_ = 1;
  }
  if (initialpose_relax_duration_sec_ < 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "initialpose_relax_duration_sec=%lf is invalid; clamping to 0.",
      initialpose_relax_duration_sec_);
    initialpose_relax_duration_sec_ = 0.0;
  }
  if (initialpose_max_pose_jump_translation_ < max_pose_jump_translation_) {
    RCLCPP_WARN(
      get_logger(),
      "initialpose_max_pose_jump_translation=%lf is below normal limit %lf; clamping to normal limit.",
      initialpose_max_pose_jump_translation_, max_pose_jump_translation_);
    initialpose_max_pose_jump_translation_ = max_pose_jump_translation_;
  }
  if (initialpose_max_pose_jump_yaw_ < max_pose_jump_yaw_) {
    RCLCPP_WARN(
      get_logger(),
      "initialpose_max_pose_jump_yaw=%lf is below normal limit %lf; clamping to normal limit.",
      initialpose_max_pose_jump_yaw_, max_pose_jump_yaw_);
    initialpose_max_pose_jump_yaw_ = max_pose_jump_yaw_;
  }
  pose_jump_reacquire_max_translation_ = std::max(
    max_pose_jump_translation_, pose_jump_reacquire_max_translation_);
  pose_jump_reacquire_max_yaw_ = std::max(0.0, pose_jump_reacquire_max_yaw_);
  pose_jump_reacquire_max_fitness_ = std::max(0.0, pose_jump_reacquire_max_fitness_);
  pose_jump_reacquire_required_frames_ = std::max(1, pose_jump_reacquire_required_frames_);
  pose_jump_reacquire_xy_tolerance_ = std::max(0.0, pose_jump_reacquire_xy_tolerance_);
  pose_jump_reacquire_yaw_tolerance_ = std::max(0.0, pose_jump_reacquire_yaw_tolerance_);
  candidate_confirmation_min_translation_ = std::max(0.0, candidate_confirmation_min_translation_);
  candidate_confirmation_min_yaw_ = std::max(0.0, candidate_confirmation_min_yaw_);
  candidate_confirmation_max_fitness_ = std::max(0.0, candidate_confirmation_max_fitness_);
  candidate_confirmation_max_mean_corr_dist_ = std::max(0.0, candidate_confirmation_max_mean_corr_dist_);
  candidate_confirmation_required_frames_ = std::max(1, candidate_confirmation_required_frames_);
  candidate_confirmation_xy_tolerance_ = std::max(0.0, candidate_confirmation_xy_tolerance_);
  candidate_confirmation_yaw_tolerance_ = std::max(0.0, candidate_confirmation_yaw_tolerance_);
  rotation_guard_angular_threshold_ = std::max(0.0, rotation_guard_angular_threshold_);
  rotation_guard_linear_threshold_ = std::max(0.0, rotation_guard_linear_threshold_);
  rotation_guard_settle_sec_ = std::max(0.0, rotation_guard_settle_sec_);
  rotation_guard_max_duration_sec_ = std::max(0.1, rotation_guard_max_duration_sec_);
  multi_frame_window_sec_ = std::max(0.0, multi_frame_window_sec_);
  multi_frame_max_frames_ = std::max(1, multi_frame_max_frames_);
  multi_frame_voxel_leaf_size_ = std::max(0.01, multi_frame_voxel_leaf_size_);
  multi_frame_max_points_ = std::max(1000, multi_frame_max_points_);
  multi_frame_keyframe_translation_threshold_ = std::max(0.0, multi_frame_keyframe_translation_threshold_);
  multi_frame_keyframe_yaw_threshold_ = std::max(0.0, multi_frame_keyframe_yaw_threshold_);
  multi_frame_keyframe_max_interval_sec_ = std::max(0.0, multi_frame_keyframe_max_interval_sec_);
  
  // 获取地图参数
  get_parameter("use_pcd_map", use_pcd_map_);  // 是否使用PCD地图
  get_parameter("map_path", map_path_);        // 地图路径
  
  // 获取初始位姿参数
  get_parameter("set_initial_pose", set_initial_pose_);  // 是否设置初始位姿
  get_parameter("initial_pose_x", initial_pose_x_);      // 初始X
  get_parameter("initial_pose_y", initial_pose_y_);      // 初始Y
  get_parameter("initial_pose_z", initial_pose_z_);      // 初始Z
  get_parameter("initial_pose_qx", initial_pose_qx_);    // 初始四元数X
  get_parameter("initial_pose_qy", initial_pose_qy_);    // 初始四元数Y
  get_parameter("initial_pose_qz", initial_pose_qz_);    // 初始四元数Z
  get_parameter("initial_pose_qw", initial_pose_qw_);    // 初始四元数W
  
  // 获取传感器融合参数
  get_parameter("use_odom", use_odom_);        // 是否使用里程计
  get_parameter("use_imu", use_imu_);          // 是否使用IMU
  get_parameter("enable_debug", enable_debug_);  // 是否启用调试
  get_parameter("force_2d_pose", force_2d_pose_);
  get_parameter("force_2d_fixed_z", force_2d_fixed_z_);
  get_parameter("force_2d_z", force_2d_z_);
  get_parameter("republish_last_good_tf_on_failure", republish_last_good_tf_on_failure_);
  get_parameter("max_last_good_tf_age_sec", max_last_good_tf_age_sec_);

  // Fast-LIO delta guess 参数 (Plan B)
  get_parameter("use_fastlio_delta_guess", use_fastlio_delta_guess_);
  get_parameter("fastlio_delta_guess_mode", fastlio_delta_guess_mode_);
  get_parameter("fastlio_camera_frame", fastlio_camera_frame_);
  get_parameter("fastlio_body_frame", fastlio_body_frame_);
  get_parameter("tf_max_stamp_mismatch_sec", tf_max_stamp_mismatch_sec_);
  get_parameter("fastlio_max_delta_translation", fastlio_max_delta_translation_);
  get_parameter("fastlio_max_delta_yaw", fastlio_max_delta_yaw_);
  get_parameter("fastlio_max_delta_dt", fastlio_max_delta_dt_);
  get_parameter("fastlio_max_dead_reckon_sec", fastlio_max_dead_reckon_sec_);
  if (use_fastlio_delta_guess_ && fastlio_delta_guess_mode_ == "disabled") {
    fastlio_delta_guess_mode_ = "map_body_to_map_odom";
    RCLCPP_WARN(
      get_logger(),
      "use_fastlio_delta_guess=true with fastlio_delta_guess_mode=disabled; using map_body_to_map_odom for backward compatibility.");
  }
  if (use_fastlio_delta_guess_ && fastlio_delta_guess_mode_ != "map_body_to_map_odom") {
    RCLCPP_WARN(
      get_logger(),
      "Unsupported fastlio_delta_guess_mode='%s'; disabling Fast-LIO delta guess.",
      fastlio_delta_guess_mode_.c_str());
    use_fastlio_delta_guess_ = false;
  }
  if (use_fastlio_delta_guess_) {
    has_prev_body_pose_ = false;
    has_prev_odom_body_pose_ = false;
    last_accept_time_ = this->now();
    RCLCPP_INFO(get_logger(),
      "Fast-LIO delta guess enabled: mode=%s camera=%s body=%s max_delta_trans=%.3f max_delta_yaw=%.3f "
      "max_delta_dt=%.3f dead_reckon=%.1fs",
      fastlio_delta_guess_mode_.c_str(), fastlio_camera_frame_.c_str(), fastlio_body_frame_.c_str(),
      fastlio_max_delta_translation_, fastlio_max_delta_yaw_, fastlio_max_delta_dt_,
      fastlio_max_dead_reckon_sec_);
  }

  // 打印参数值到日志，方便调试
  RCLCPP_INFO(get_logger(),"global_frame_id: %s", global_frame_id_.c_str());
  RCLCPP_INFO(get_logger(),"odom_frame_id: %s", odom_frame_id_.c_str());
  RCLCPP_INFO(get_logger(),"base_frame_id: %s", base_frame_id_.c_str());
  RCLCPP_INFO(get_logger(),"initialpose_base_frame_id: %s", initialpose_base_frame_id_.c_str());
  RCLCPP_INFO(get_logger(),"localization_status_topic: %s", localization_status_topic_.c_str());
  RCLCPP_INFO(get_logger(),"registration_method: %s", registration_method_.c_str());
  RCLCPP_INFO(get_logger(),"score_threshold: %lf", score_threshold_);
  RCLCPP_INFO(get_logger(),"reject_pose_jump: %d", reject_pose_jump_);
  RCLCPP_INFO(get_logger(),"max_pose_jump_translation: %lf", max_pose_jump_translation_);
  RCLCPP_INFO(get_logger(),"max_pose_jump_yaw: %lf", max_pose_jump_yaw_);
  RCLCPP_INFO(get_logger(),"initialpose_relax_duration_sec: %lf", initialpose_relax_duration_sec_);
  RCLCPP_INFO(get_logger(),"initialpose_max_pose_jump_translation: %lf", initialpose_max_pose_jump_translation_);
  RCLCPP_INFO(get_logger(),"initialpose_max_pose_jump_yaw: %lf", initialpose_max_pose_jump_yaw_);
  RCLCPP_INFO(get_logger(),"pose_jump_reacquire_enabled: %d", pose_jump_reacquire_enabled_);
  RCLCPP_INFO(get_logger(),"pose_jump_reacquire_max_translation: %lf", pose_jump_reacquire_max_translation_);
  RCLCPP_INFO(get_logger(),"pose_jump_reacquire_max_yaw: %lf", pose_jump_reacquire_max_yaw_);
  RCLCPP_INFO(get_logger(),"pose_jump_reacquire_max_fitness: %lf", pose_jump_reacquire_max_fitness_);
  RCLCPP_INFO(get_logger(),"pose_jump_reacquire_required_frames: %d", pose_jump_reacquire_required_frames_);
  RCLCPP_INFO(get_logger(),"pose_jump_reacquire_xy_tolerance: %lf", pose_jump_reacquire_xy_tolerance_);
  RCLCPP_INFO(get_logger(),"pose_jump_reacquire_yaw_tolerance: %lf", pose_jump_reacquire_yaw_tolerance_);
  RCLCPP_INFO(get_logger(),"candidate_confirmation_enabled: %d", candidate_confirmation_enabled_);
  RCLCPP_INFO(get_logger(),"candidate_confirmation_min_translation: %lf", candidate_confirmation_min_translation_);
  RCLCPP_INFO(get_logger(),"candidate_confirmation_min_yaw: %lf", candidate_confirmation_min_yaw_);
  RCLCPP_INFO(get_logger(),"candidate_confirmation_max_fitness: %lf", candidate_confirmation_max_fitness_);
  RCLCPP_INFO(get_logger(),"candidate_confirmation_max_mean_corr_dist: %lf", candidate_confirmation_max_mean_corr_dist_);
  RCLCPP_INFO(get_logger(),"candidate_confirmation_required_frames: %d", candidate_confirmation_required_frames_);
  RCLCPP_INFO(get_logger(),"candidate_confirmation_xy_tolerance: %lf", candidate_confirmation_xy_tolerance_);
  RCLCPP_INFO(get_logger(),"candidate_confirmation_yaw_tolerance: %lf", candidate_confirmation_yaw_tolerance_);
  RCLCPP_INFO(get_logger(),"rotation_guard_enabled: %d", rotation_guard_enabled_);
  RCLCPP_INFO(get_logger(),"rotation_guard_use_cmd_vel_fallback: %d", rotation_guard_use_cmd_vel_fallback_);
  RCLCPP_INFO(get_logger(),"rotation_guard_navigation_status_topic: %s", rotation_guard_navigation_status_topic_.c_str());
  RCLCPP_INFO(get_logger(),"rotation_guard_angular_threshold: %lf", rotation_guard_angular_threshold_);
  RCLCPP_INFO(get_logger(),"rotation_guard_linear_threshold: %lf", rotation_guard_linear_threshold_);
  RCLCPP_INFO(get_logger(),"rotation_guard_settle_sec: %lf", rotation_guard_settle_sec_);
  RCLCPP_INFO(get_logger(),"rotation_guard_max_duration_sec: %lf", rotation_guard_max_duration_sec_);
  RCLCPP_INFO(get_logger(),"multi_frame_matching_enabled: %d", multi_frame_matching_enabled_);
  RCLCPP_INFO(get_logger(),"multi_frame_use_only_when_rotating: %d", multi_frame_use_only_when_rotating_);
  RCLCPP_INFO(get_logger(),"multi_frame_window_sec: %lf", multi_frame_window_sec_);
  RCLCPP_INFO(get_logger(),"multi_frame_max_frames: %d", multi_frame_max_frames_);
  RCLCPP_INFO(get_logger(),"multi_frame_voxel_leaf_size: %lf", multi_frame_voxel_leaf_size_);
  RCLCPP_INFO(get_logger(),"multi_frame_max_points: %d", multi_frame_max_points_);
  RCLCPP_INFO(get_logger(),"multi_frame_keyframe_filter_enabled: %d", multi_frame_keyframe_filter_enabled_);
  RCLCPP_INFO(get_logger(),"multi_frame_keyframe_translation_threshold: %lf", multi_frame_keyframe_translation_threshold_);
  RCLCPP_INFO(get_logger(),"multi_frame_keyframe_yaw_threshold: %lf", multi_frame_keyframe_yaw_threshold_);
  RCLCPP_INFO(get_logger(),"multi_frame_keyframe_max_interval_sec: %lf", multi_frame_keyframe_max_interval_sec_);
  RCLCPP_INFO(get_logger(),"ndt_resolution: %lf", ndt_resolution_);
  RCLCPP_INFO(get_logger(),"ndt_step_size: %lf", ndt_step_size_);
  RCLCPP_INFO(get_logger(),"ndt_num_threads: %d", ndt_num_threads_);
  RCLCPP_INFO(get_logger(),"transform_epsilon: %lf", transform_epsilon_);
  RCLCPP_INFO(get_logger(),"voxel_leaf_size: %lf", voxel_leaf_size_);
  RCLCPP_INFO(get_logger(),"ndt_outlier_ratio: %lf", ndt_outlier_ratio_);
  RCLCPP_INFO(get_logger(),"ndt_max_corr_dist: %lf", ndt_max_corr_dist_);
  RCLCPP_INFO(get_logger(),"ndt_rotation_prior_enabled: %d", ndt_rotation_prior_enabled_);
  RCLCPP_INFO(get_logger(),"ndt_rotation_prior_weight: %lf", ndt_rotation_prior_weight_);
  RCLCPP_INFO(get_logger(),"ndt_rotation_prior_roll_pitch_only: %d", ndt_rotation_prior_roll_pitch_only_);
  RCLCPP_INFO(get_logger(),"scan_max_range: %lf", scan_max_range_);
  RCLCPP_INFO(get_logger(),"scan_min_range: %lf", scan_min_range_);
  RCLCPP_INFO(get_logger(),"min_scan_points: %d", min_scan_points_);
  RCLCPP_INFO(get_logger(),"scan_period: %lf", scan_period_);
  RCLCPP_INFO(get_logger(),"use_pcd_map: %d", use_pcd_map_);
  RCLCPP_INFO(get_logger(),"map_path: %s", map_path_.c_str());
  RCLCPP_INFO(get_logger(),"set_initial_pose: %d", set_initial_pose_);
  RCLCPP_INFO(get_logger(),"use_odom: %d", use_odom_);
  RCLCPP_INFO(get_logger(),"use_imu: %d", use_imu_);
  RCLCPP_INFO(get_logger(),"enable_debug: %d", enable_debug_);
  RCLCPP_INFO(get_logger(),"force_2d_pose: %d", force_2d_pose_);
  RCLCPP_INFO(get_logger(),"force_2d_fixed_z: %d", force_2d_fixed_z_);
  RCLCPP_INFO(get_logger(),"force_2d_z: %lf", force_2d_z_);
  RCLCPP_INFO(get_logger(),"republish_last_good_tf_on_failure: %d", republish_last_good_tf_on_failure_);
  RCLCPP_INFO(get_logger(),"max_last_good_tf_age_sec: %lf", max_last_good_tf_age_sec_);
}

void PCLLocalization::applyPlanarPoseConstraint(geometry_msgs::msg::Pose & pose) const
{
  if (!force_2d_pose_) {
    return;
  }

  tf2::Quaternion quat_tf;
  tf2::fromMsg(pose.orientation, quat_tf);

  double roll;
  double pitch;
  double yaw;
  tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);

  tf2::Quaternion yaw_only;
  yaw_only.setRPY(0.0, 0.0, yaw);
  yaw_only.normalize();

  if (force_2d_fixed_z_) {
    pose.position.z = force_2d_z_;
  }
  pose.orientation = tf2::toMsg(yaw_only);
}

Eigen::Matrix4f PCLLocalization::applyPlanarTransformConstraint(
  const Eigen::Matrix4f & transform) const
{
  if (!force_2d_pose_) {
    return transform;
  }

  const Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0).cast<double>();
  const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));

  Eigen::Matrix4f constrained = Eigen::Matrix4f::Identity();
  constrained.block<3, 3>(0, 0) =
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix().cast<float>();
  constrained(0, 3) = transform(0, 3);
  constrained(1, 3) = transform(1, 3);
  constrained(2, 3) = force_2d_fixed_z_ ? static_cast<float>(force_2d_z_) : transform(2, 3);
  return constrained;
}

bool PCLLocalization::initialPoseReacquireActive()
{
  if (!initialpose_reacquire_active_ || initialpose_relax_duration_sec_ <= 0.0) {
    return false;
  }

  const double age_sec = (this->now() - last_initialpose_time_).seconds();
  if (age_sec < 0.0 || age_sec > initialpose_relax_duration_sec_) {
    initialpose_reacquire_active_ = false;
    return false;
  }

  return true;
}

bool PCLLocalization::publishLastGoodTransformIfFresh(const char * reject_reason)
{
  if (initialPoseReacquireActive()) {
    return false;
  }

  if (!republish_last_good_tf_on_failure_ || !has_last_good_transform_) {
    return false;
  }

  const rclcpp::Time now = this->now();
  const double age_sec = (now - last_good_transform_time_).seconds();
  if (age_sec < 0.0 || age_sec > max_last_good_tf_age_sec_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Localization rejected scans for %.2f sec after %s; last good TF is too old, waiting for relocalization.",
      age_sec, reject_reason);
    return false;
  }

  // Phase 1: fusion DEGRADED/LOST/FUSION_TIMEOUT 时不重发旧 TF
  if (false) {
    return false;
  }

  auto transform_stamped = last_good_transform_;
  transform_stamped.header.stamp = now;
  broadcaster_.sendTransform(transform_stamped);

  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), 2000,
    "Republishing last good %s->%s TF for %.2f sec after %s.",
    transform_stamped.header.frame_id.c_str(), transform_stamped.child_frame_id.c_str(),
    age_sec, reject_reason);
  return true;
}

void PCLLocalization::resetCandidateConfirmation()
{
  candidate_confirmation_active_ = false;
  candidate_confirmation_count_ = 0;
}

void PCLLocalization::publishLocalizationStatus(
  const char * state,
  const char * reason,
  bool has_converged,
  double fitness_score,
  int filtered_points,
  const rclcpp::Time & stamp,
  double correction_translation,
  double correction_yaw)
{
  if (!status_pub_) {
    return;
  }

  const std::string state_text = state ? state : "unknown";
  const std::string reason_text = reason ? reason : "";
  if (state_text == "rejected") {
    consecutive_rejected_frames_ += 1;
  } else if (state_text == "accepted") {
    consecutive_rejected_frames_ = 0;
  }

  const bool initialpose_reacquiring = initialPoseReacquireActive();
  const double pose_jump_translation_limit =
    initialpose_reacquiring ? initialpose_max_pose_jump_translation_ : max_pose_jump_translation_;
  const double pose_jump_yaw_limit =
    initialpose_reacquiring ? initialpose_max_pose_jump_yaw_ : max_pose_jump_yaw_;

  std::ostringstream out;
  out.setf(std::ios::fixed);
  out.precision(6);
  out
    << "{"
    << "\"event_type\":\"ndt_localization_status\","
    << "\"state\":\"" << state_text << "\","
    << "\"reason\":\"" << reason_text << "\","
    << "\"stamp_sec\":" << stamp.seconds() << ","
    << "\"has_converged\":" << (has_converged ? "true" : "false") << ","
    << "\"fitness_score\":" << fitness_score << ","
    << "\"score_threshold\":" << score_threshold_ << ","
    << "\"correction_translation\":" << correction_translation << ","
    << "\"correction_yaw\":" << correction_yaw << ","
    << "\"max_pose_jump_translation\":" << max_pose_jump_translation_ << ","
    << "\"max_pose_jump_yaw\":" << max_pose_jump_yaw_ << ","
    << "\"initialpose_reacquiring\":" << (initialpose_reacquiring ? "true" : "false") << ","
    << "\"pose_jump_translation_limit\":" << pose_jump_translation_limit << ","
    << "\"pose_jump_yaw_limit\":" << pose_jump_yaw_limit << ","
    << "\"pose_jump_reacquire_enabled\":" << (pose_jump_reacquire_enabled_ ? "true" : "false") << ","
    << "\"pose_jump_reacquire_max_translation\":" << pose_jump_reacquire_max_translation_ << ","
    << "\"pose_jump_reacquire_max_yaw\":" << pose_jump_reacquire_max_yaw_ << ","
    << "\"pose_jump_reacquire_max_fitness\":" << pose_jump_reacquire_max_fitness_ << ","
    << "\"pose_jump_reacquire_required_frames\":" << pose_jump_reacquire_required_frames_ << ","
    << "\"pose_jump_candidate_count\":" << pose_jump_candidate_count_ << ","
    << "\"candidate_confirmation_enabled\":" << (candidate_confirmation_enabled_ ? "true" : "false") << ","
    << "\"candidate_confirmation_min_translation\":" << candidate_confirmation_min_translation_ << ","
    << "\"candidate_confirmation_min_yaw\":" << candidate_confirmation_min_yaw_ << ","
    << "\"candidate_confirmation_max_fitness\":" << candidate_confirmation_max_fitness_ << ","
    << "\"candidate_confirmation_max_mean_corr_dist\":" << candidate_confirmation_max_mean_corr_dist_ << ","
    << "\"candidate_confirmation_required_frames\":" << candidate_confirmation_required_frames_ << ","
    << "\"candidate_confirmation_count\":" << candidate_confirmation_count_ << ","
    << "\"filtered_points\":" << filtered_points << ","
    << "\"mean_corr_dist\":" << last_mean_corr_dist_ << ","   // NDT退化诊断关键指标
    << "\"ndt_outlier_ratio\":" << ndt_outlier_ratio_ << ","
    << "\"ndt_max_corr_dist\":" << ndt_max_corr_dist_ << ","
    << "\"ndt_rotation_prior_weight\":" << ndt_rotation_prior_weight_ << ","
    << "\"consecutive_rejected_frames\":" << consecutive_rejected_frames_ << ","
    << "\"fastlio_delta_applied\":" << (fastlio_delta_applied_ ? "true" : "false") << ","
    << "\"fastlio_delta_reject_reason\":\"" << fastlio_delta_reject_reason_ << "\","
    << "\"fastlio_dead_reckon_age_sec\":" << fastlio_dead_reckon_age_debug_ << ","
    << "\"fastlio_delta_translation\":" << fastlio_delta_translation_debug_ << ","
    << "\"fastlio_delta_yaw\":" << fastlio_delta_yaw_debug_ << ","
    << "\"fastlio_delta_guess_mode\":\"" << fastlio_delta_guess_mode_ << "\","
    << "\"fastlio_odom_body_used\":" << (fastlio_odom_body_used_debug_ ? "true" : "false") << ","
    << "\"fastlio_delta_dt_sec\":" << fastlio_delta_dt_debug_ << ","
    << "\"fastlio_map_odom_guess_shift\":" << fastlio_map_odom_guess_shift_debug_ << ","
    << "\"ndt_candidate_pose_valid\":" << (ndt_candidate_pose_valid_debug_ ? "true" : "false") << ","
    << "\"ndt_candidate_x\":" << ndt_candidate_x_debug_ << ","
    << "\"ndt_candidate_y\":" << ndt_candidate_y_debug_ << ","
    << "\"ndt_candidate_yaw\":" << ndt_candidate_yaw_debug_ << ","
    << "\"ndt_init_guess_x\":" << ndt_init_guess_x_debug_ << ","
    << "\"ndt_init_guess_y\":" << ndt_init_guess_y_debug_ << ","
    << "\"ndt_init_guess_yaw\":" << ndt_init_guess_yaw_debug_ << ","
    << "\"ndt_candidate_has_prev\":" << (ndt_prev_candidate_valid_debug_ ? "true" : "false") << ","
    << "\"ndt_candidate_delta_prev_xy\":" << ndt_candidate_delta_prev_xy_debug_ << ","
    << "\"ndt_candidate_delta_prev_yaw\":" << ndt_candidate_delta_prev_yaw_debug_ << ","
    << "\"ndt_candidate_delta_last_good_xy\":" << ndt_candidate_delta_last_good_xy_debug_ << ","
    << "\"ndt_candidate_delta_last_good_yaw\":" << ndt_candidate_delta_last_good_yaw_debug_ << ","
    << "\"rotation_guard_active\":" << (rotation_guard_active_debug_ ? "true" : "false") << ","
    << "\"rotation_guard_settle\":" << (rotation_guard_settle_debug_ ? "true" : "false") << ","
    << "\"rotation_guard_age_sec\":" << rotation_guard_age_debug_ << ","
    << "\"rotation_guard_source\":\"" << rotation_guard_source_debug_ << "\","
    << "\"rotation_guard_hold_count\":" << rotation_guard_hold_count_ << ","
    << "\"multi_frame_source_frames\":" << multi_frame_source_frames_debug_ << ","
    << "\"multi_frame_source_points\":" << multi_frame_source_points_debug_ << ","
    << "\"multi_frame_buffer_frames\":" << multi_frame_buffer_frames_debug_ << ","
    << "\"multi_frame_keyframe_filter_enabled\":" << (multi_frame_keyframe_filter_enabled_ ? "true" : "false") << ","
    << "\"multi_frame_skipped_frames\":" << multi_frame_skipped_frames_debug_ << ","
    << "\"ndt_align_duration_sec\":" << ndt_align_duration_sec_debug_
    << "}";

  std_msgs::msg::String msg;
  msg.data = out.str();
  status_pub_->publish(msg);
}

/**
 * @brief 初始化ROS发布者和订阅者
 * 
 * 创建所有ROS通信接口：
 * - 发布者：定位结果(pcl_pose)、路径(path)、初始地图(initial_map)
 * - 订阅者：初始位姿(initialpose)、地图(map)、里程计(odm)、点云(cloud)、IMU(imu)
 */
void PCLLocalization::initializePubSub()
{
  RCLCPP_INFO(get_logger(), "initializePubSub");

  // 创建定位结果发布者，使用transient_local确保新订阅者能收到最后一帧
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "pcl_pose",  // 话题名称
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  // 创建路径发布者
  path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "path",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  // 创建初始地图发布者
  initial_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "initial_map",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  status_pub_ = create_publisher<std_msgs::msg::String>(
    localization_status_topic_,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

  // 订阅初始位姿话题（通常在RViz中通过"2D Pose Estimate"设置）
  initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", rclcpp::SystemDefaultsQoS(),
    std::bind(&PCLLocalization::initialPoseReceived, this, std::placeholders::_1));

  // 订阅地图话题，使用transient_local确保能收到最后发布的地图
  map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&PCLLocalization::mapReceived, this, std::placeholders::_1));

  // 订阅里程计话题，用于位姿预测
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom", rclcpp::SensorDataQoS(),
    std::bind(&PCLLocalization::odomReceived, this, std::placeholders::_1));

  // 订阅点云话题，这是核心输入数据
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/fast_lio/cloud_registered", rclcpp::SensorDataQoS(),
    std::bind(&PCLLocalization::cloudReceived, this, std::placeholders::_1));

  // 订阅IMU话题，用于点云去畸变
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "imu", rclcpp::SensorDataQoS(),
    std::bind(&PCLLocalization::imuReceived, this, std::placeholders::_1));

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", rclcpp::SensorDataQoS(),
    std::bind(&PCLLocalization::cmdVelReceived, this, std::placeholders::_1));

  navigation_status_sub_ = create_subscription<std_msgs::msg::String>(
    rotation_guard_navigation_status_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
    std::bind(&PCLLocalization::navigationStatusReceived, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "initializePubSub end");
}

/**
 * @brief 初始化配准算法
 * 
 * 根据registration_method参数选择并配置相应的配准算法
 * 支持的算法：
 * - GICP: 通用迭代最近点（精确但较慢）
 * - NDT: 正态分布变换（适合大场景，较快）
 * - GICP_OMP/NDT_OMP: 多线程加速版本
 */
void PCLLocalization::initializeRegistration()
{
  RCLCPP_INFO(get_logger(), "initializeRegistration");

  // 根据参数选择配准算法
  if (registration_method_ == "GICP") {
    // 使用标准GICP算法，适合小场景或精确配准
    boost::shared_ptr<pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>> gicp(
      new pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
    gicp->setTransformationEpsilon(transform_epsilon_);  // 设置收敛阈值
    registration_ = gicp;
  }
  else if (registration_method_ == "NDT") {
    // 使用标准NDT算法，适合大场景定位
    boost::shared_ptr<pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>> ndt(
      new pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt->setStepSize(ndt_step_size_);        // 牛顿迭代步长
    ndt->setResolution(ndt_resolution_);     // NDT网格分辨率
    ndt->setTransformationEpsilon(transform_epsilon_);  // 收敛阈值
    ndt->setOulierRatio(ndt_outlier_ratio_); // 离群点比率
    registration_ = ndt;
  }
  else if (registration_method_ == "NDT_OMP") {
    // 使用多线程NDT算法，大幅提升速度
    pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt_omp(
      new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt_omp->setStepSize(ndt_step_size_);
    ndt_omp->setResolution(ndt_resolution_);
    ndt_omp->setTransformationEpsilon(transform_epsilon_);
    ndt_omp->setOulierRatio(ndt_outlier_ratio_);     // 离群点比率
    if (ndt_max_corr_dist_ > 0.0) {
      ndt_omp->setMaxCorrespondenceDistance(ndt_max_corr_dist_);  // 最大关联距离
    }
    // 设置线程数
    if (ndt_num_threads_ > 0) {
      ndt_omp->setNumThreads(ndt_num_threads_);  // 使用指定线程数
    } else {
      ndt_omp->setNumThreads(omp_get_max_threads());  // 使用最大可用线程
    }
    registration_ = ndt_omp;
  }
  else if (registration_method_ == "GICP_OMP") {
    // 使用多线程GICP算法
    pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>::Ptr gicp_omp(
      new pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
    gicp_omp->setTransformationEpsilon(transform_epsilon_);
    registration_ = gicp_omp;
  }
  else {
    // 无效的配准方法，退出程序
    RCLCPP_ERROR(get_logger(), "Invalid registration method.");
    exit(EXIT_FAILURE);
  }
  
  // 所有算法都需要设置最大迭代次数
  registration_->setMaximumIterations(ndt_max_iterations_);

  // 配置体素滤波器，用于点云降采样
  voxel_grid_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  RCLCPP_INFO(get_logger(), "initializeRegistration end");
}

/**
 * @brief 接收初始位姿消息回调
 * 
 * 通常由RViz的"2D Pose Estimate"工具触发
 * 收到初始位姿后，更新当前位姿并触发一次定位
 * 
 * @param msg 初始位姿消息
 */
void PCLLocalization::initialPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "initialPoseReceived");
  RCLCPP_INFO(
    get_logger(), "initialpose input: frame=%s x=%.3f y=%.3f z=%.3f yaw=%.1fdeg",
    msg->header.frame_id.c_str(),
    msg->pose.pose.position.x,
    msg->pose.pose.position.y,
    msg->pose.pose.position.z,
    tf2::getYaw(msg->pose.pose.orientation) * 180.0 / M_PI);

  auto initial_pose_msg =
    std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>(*msg);

  // RViz 的 2D Pose Estimate 会使用当前 Fixed Frame。导航界面通常使用
  // map_ground，因此这里把它转换到定位节点真正使用的 global_frame_id。
  if (msg->header.frame_id != global_frame_id_) {
    try {
      geometry_msgs::msg::PoseStamped pose_in;
      pose_in.header = msg->header;
      pose_in.pose = msg->pose.pose;

      const auto transform = tfbuffer_.lookupTransform(
        global_frame_id_, msg->header.frame_id, tf2::TimePointZero);

      geometry_msgs::msg::PoseStamped pose_out;
      tf2::doTransform(pose_in, pose_out, transform);

      initial_pose_msg->header.frame_id = global_frame_id_;
      initial_pose_msg->pose.pose = pose_out.pose;

      RCLCPP_INFO(
        get_logger(), "Transformed initialpose from %s to %s: x=%.3f y=%.3f z=%.3f yaw=%.1fdeg",
        msg->header.frame_id.c_str(), global_frame_id_.c_str(),
        initial_pose_msg->pose.pose.position.x,
        initial_pose_msg->pose.pose.position.y,
        initial_pose_msg->pose.pose.position.z,
        tf2::getYaw(initial_pose_msg->pose.pose.orientation) * 180.0 / M_PI);
    } catch (tf2::TransformException & ex) {
      // 当 map 帧尚未存在时 (NDT 还未发布 map->odom),
      // map_ground 帧的位姿在 x/y/yaw 上与 map 一致 (仅 z 投影到地面),
      // 直接接受初始位姿以避免"鸡生蛋"死锁。
      if (global_frame_id_ == "map" && msg->header.frame_id == "map_ground") {
        RCLCPP_WARN(
          this->get_logger(),
          "map frame not available yet, accepting map_ground initialpose directly "
          "(map_ground is aligned with map in xy+yaw): x=%.3f y=%.3f z=%.3f yaw=%.1fdeg",
          msg->pose.pose.position.x, msg->pose.pose.position.y,
          msg->pose.pose.position.z,
          tf2::getYaw(msg->pose.pose.orientation) * 180.0 / M_PI);
      } else {
        RCLCPP_WARN(
          this->get_logger(), "Failed to transform initialpose from %s to %s: %s",
          msg->header.frame_id.c_str(), global_frame_id_.c_str(), ex.what());
        return;
      }
    }
  }

  applyPlanarPoseConstraint(initial_pose_msg->pose.pose);

  try {
    const auto odom_to_base = tfbuffer_.lookupTransform(
      odom_frame_id_, initialpose_base_frame_id_, tf2::TimePointZero);

    Eigen::Affine3d T_map_base = Eigen::Affine3d::Identity();
    tf2::fromMsg(initial_pose_msg->pose.pose, T_map_base);
    const Eigen::Affine3d T_odom_base = tf2::transformToEigen(odom_to_base);
    const Eigen::Affine3d T_map_odom = T_map_base * T_odom_base.inverse();

    const auto map_odom_pose = tf2::toMsg(T_map_odom);
    RCLCPP_INFO(
      get_logger(),
      "Converted initialpose map->%s to internal map->%s: "
      "base=(%.3f, %.3f, yaw=%.1fdeg) odom_pose=(%.3f, %.3f, yaw=%.1fdeg)",
      initialpose_base_frame_id_.c_str(), odom_frame_id_.c_str(),
      initial_pose_msg->pose.pose.position.x,
      initial_pose_msg->pose.pose.position.y,
      tf2::getYaw(initial_pose_msg->pose.pose.orientation) * 180.0 / M_PI,
      map_odom_pose.position.x,
      map_odom_pose.position.y,
      tf2::getYaw(map_odom_pose.orientation) * 180.0 / M_PI);

    initial_pose_msg->pose.pose = map_odom_pose;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      get_logger(),
      "Failed to convert initialpose map->%s into map->%s using TF %s->%s: %s. "
      "Keeping the input pose as the internal NDT guess for compatibility.",
      initialpose_base_frame_id_.c_str(), odom_frame_id_.c_str(),
      odom_frame_id_.c_str(), initialpose_base_frame_id_.c_str(), ex.what());
  }

  RCLCPP_INFO(
    get_logger(), "initialpose stored as internal map->%s guess: frame=%s x=%.3f y=%.3f z=%.3f yaw=%.1fdeg",
    odom_frame_id_.c_str(),
    initial_pose_msg->header.frame_id.c_str(),
    initial_pose_msg->pose.pose.position.x,
    initial_pose_msg->pose.pose.position.y,
    initial_pose_msg->pose.pose.position.z,
    tf2::getYaw(initial_pose_msg->pose.pose.orientation) * 180.0 / M_PI);
  
  initial_pose_msg->header.stamp = this->now();
  initialpose_recieved_ = true;  // 标记已接收初始位姿
  corrent_pose_with_cov_stamped_ptr_ = initial_pose_msg;  // 保存当前位姿
  last_initialpose_time_ = this->now();
  initialpose_reacquire_active_ = initialpose_relax_duration_sec_ > 0.0;
  has_last_good_transform_ = false;
  consecutive_rejected_frames_ = 0;
  has_prev_body_pose_ = false;  // Fast-LIO delta guess: /initialpose 重置基准
  has_prev_odom_body_pose_ = false;
  has_last_ndt_candidate_debug_ = false;
  resetCandidateConfirmation();
  scan_frame_buffer_.clear();
  rotation_guard_active_ = false;
  rotation_guard_state_active_ = false;
  rotation_guard_rotating_now_ = false;
  rotation_guard_source_.clear();
  rotation_guard_hold_count_ = 0;
  last_accept_time_ = this->now();  // 同步重置 dead-reckon 计时
  pose_pub_->publish(*corrent_pose_with_cov_stamped_ptr_);  // 发布初始位姿

  // Phase 3: 不强制用旧帧点云匹配 — 等下一帧自然到达
  // 旧帧可能来自 DEGRADED 期间的污染点云, 强制匹配会导致 NDT 在错误位姿初始化
  // initialpose 已保存, cloudReceived 回调会在下一帧新点云到达时自然触发匹配
  if (last_scan_ptr_) {
    RCLCPP_INFO(get_logger(), "Initial pose stored, waiting for next scan frame for matching.");
  } else {
    RCLCPP_WARN(get_logger(), "No scan received yet, initial pose stored only.");
  }
  RCLCPP_INFO(get_logger(), "initialPoseReceived end");
}

/**
 * @brief 接收地图消息回调
 * 
 * 支持动态加载地图，收到地图后设置为配准目标
 * 
 * @param msg 地图点云消息
 */
void PCLLocalization::mapReceived(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "mapReceived");
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);

  // 检查坐标系是否匹配
  if (msg->header.frame_id != global_frame_id_) {
    RCLCPP_WARN(this->get_logger(), "map_frame_id does not match global_frame_id");
    return;
  }

  // 将ROS消息转换为PCL点云
  pcl::fromROSMsg(*msg, *map_cloud_ptr);

  // ★★★ 核心修改 2：把接收到的建图点云从 LiDAR 坐标系(Z朝后) 掰平到 ROS标准 坐标系(Z朝上) ★★★
  Eigen::Quaternionf q_cam_to_ros(0.5, -0.5, -0.5, 0.5);
  Eigen::Vector3f t_cam_to_ros(0.0, 0.0, 0.0);
  pcl::transformPointCloud(*map_cloud_ptr, *map_cloud_ptr, t_cam_to_ros, q_cam_to_ros);
  RCLCPP_INFO(get_logger(), "Received Map transformed to ROS standard frame (Z-up)");
  // ★★★ 转换结束 ★★★

  // 根据配准方法类型决定是否对地图进行滤波
  if (registration_method_ == "GICP" || registration_method_ == "GICP_OMP") {
    // GICP算法需要预先计算协方差，对大地图滤波可以加速
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    voxel_grid_filter_.setInputCloud(map_cloud_ptr);
    voxel_grid_filter_.filter(*filtered_cloud_ptr);
    registration_->setInputTarget(filtered_cloud_ptr);
  } else {
    // NDT算法直接使用原始地图
    registration_->setInputTarget(map_cloud_ptr);
  }

  map_recieved_ = true;  // 标记地图已接收
  has_prev_body_pose_ = false;  // Fast-LIO delta guess: 新地图重置基准
  has_prev_odom_body_pose_ = false;
  has_last_ndt_candidate_debug_ = false;
  resetCandidateConfirmation();
  scan_frame_buffer_.clear();
  rotation_guard_active_ = false;
  rotation_guard_state_active_ = false;
  rotation_guard_rotating_now_ = false;
  rotation_guard_source_.clear();
  rotation_guard_hold_count_ = 0;
  last_accept_time_ = this->now();
  RCLCPP_INFO(get_logger(), "mapReceived end");
}

/**
 * @brief 接收里程计消息回调
 * 
 * 使用里程计数据预测当前位姿，作为配准的初始猜测
 * 可以提高配准速度和稳定性
 * 
 * @param msg 里程计消息
 */
void PCLLocalization::odomReceived(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  if (!use_odom_) {return;}  // 如果未启用里程计，直接返回
  RCLCPP_INFO(get_logger(), "odomReceived");

  // 计算里程计消息的时间间隔
  double current_odom_received_time = msg->header.stamp.sec +
    msg->header.stamp.nanosec * 1e-9;
  double dt_odom = current_odom_received_time - last_odom_received_time_;
  last_odom_received_time_ = current_odom_received_time;
  
  // 检查时间间隔是否合理
  if (dt_odom > 1.0 /* [sec] */) {
    RCLCPP_WARN(this->get_logger(), "odom time interval is too large");
    return;
  }
  if (dt_odom < 0.0 /* [sec] */) {
    RCLCPP_WARN(this->get_logger(), "odom time interval is negative");
    return;
  }

  // 从当前位姿中提取欧拉角
  tf2::Quaternion previous_quat_tf;
  double roll, pitch, yaw;
  tf2::fromMsg(corrent_pose_with_cov_stamped_ptr_->pose.pose.orientation, previous_quat_tf);
  tf2::Matrix3x3(previous_quat_tf).getRPY(roll, pitch, yaw);

  // 根据角速度积分更新姿态
  roll += msg->twist.twist.angular.x * dt_odom;
  pitch += msg->twist.twist.angular.y * dt_odom;
  yaw += msg->twist.twist.angular.z * dt_odom;

  // 根据更新后的欧拉角构造四元数
  Eigen::Quaterniond quat_eig =
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

  geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig);

  // 根据线速度积分更新位置
  Eigen::Vector3d odom{
    msg->twist.twist.linear.x,
    msg->twist.twist.linear.y,
    msg->twist.twist.linear.z};
  Eigen::Vector3d delta_position = quat_eig.matrix() * dt_odom * odom;

  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x += delta_position.x();
  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.y += delta_position.y();
  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.z += delta_position.z();
  corrent_pose_with_cov_stamped_ptr_->pose.pose.orientation = quat_msg;
  applyPlanarPoseConstraint(corrent_pose_with_cov_stamped_ptr_->pose.pose);
}

/**
 * @brief 接收IMU消息回调
 * 
 * 将IMU数据转换到机器人基坐标系，用于点云去畸变
 * 机械式激光雷达在扫描过程中机器人运动会产生畸变
 * IMU数据可以用于补偿这种运动畸变
 * 
 * @param msg IMU消息
 */
void PCLLocalization::imuReceived(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  if (!use_imu_) {return;}  // 如果未启用IMU，直接返回

  sensor_msgs::msg::Imu tf_converted_imu;  // 转换后的IMU数据

  try {
    // 查询IMU坐标系到机器人基坐标系的变换
    const geometry_msgs::msg::TransformStamped transform = tfbuffer_.lookupTransform(
     base_frame_id_, msg->header.frame_id, tf2::TimePointZero);

    geometry_msgs::msg::Vector3Stamped angular_velocity, linear_acceleration, transformed_angular_velocity, transformed_linear_acceleration;
    geometry_msgs::msg::Quaternion  transformed_quaternion;

    // 准备角速度和加速度向量
    angular_velocity.header = msg->header;
    angular_velocity.vector = msg->angular_velocity;
    linear_acceleration.header = msg->header;
    linear_acceleration.vector = msg->linear_acceleration;

    // 使用TF将向量转换到基坐标系
    tf2::doTransform(angular_velocity, transformed_angular_velocity, transform);
    tf2::doTransform(linear_acceleration, transformed_linear_acceleration, transform);

    tf_converted_imu.angular_velocity = transformed_angular_velocity.vector;
    tf_converted_imu.linear_acceleration = transformed_linear_acceleration.vector;
    tf_converted_imu.orientation = transformed_quaternion;

  }
  catch (tf2::TransformException& ex)
  {
    std::cout << "Failed to lookup transform" << std::endl;
    RCLCPP_WARN(this->get_logger(), "Failed to lookup transform.");
    return;
  }

  // 提取IMU数据
  Eigen::Vector3f angular_velo{tf_converted_imu.angular_velocity.x, tf_converted_imu.angular_velocity.y,
    tf_converted_imu.angular_velocity.z};
  Eigen::Vector3f acc{tf_converted_imu.linear_acceleration.x, tf_converted_imu.linear_acceleration.y, tf_converted_imu.linear_acceleration.z};
  Eigen::Quaternionf quat{msg->orientation.w, msg->orientation.x, msg->orientation.y,
    msg->orientation.z};
  double imu_time = msg->header.stamp.sec +
    msg->header.stamp.nanosec * 1e-9;

  // 将IMU数据传入去畸变模块
  lidar_undistortion_.getImu(angular_velo, acc, quat, imu_time); // 去畸变处理
}

void PCLLocalization::enterRotationGuard(const char * source)
{
  if (!rotation_guard_enabled_) {
    return;
  }

  const rclcpp::Time now = this->now();
  if (!rotation_guard_active_) {
    rotation_guard_start_time_ = now;
    rotation_guard_hold_count_ = 0;
    RCLCPP_INFO(get_logger(), "NDT rotation guard entered by %s.", source ? source : "unknown");
  }
  rotation_guard_active_ = true;
  rotation_guard_source_ = source ? source : "unknown";
  rotation_guard_settle_until_ = now + rclcpp::Duration::from_seconds(rotation_guard_settle_sec_);
}

void PCLLocalization::cmdVelReceived(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  if (!rotation_guard_enabled_ || !rotation_guard_use_cmd_vel_fallback_ || !msg) {
    return;
  }

  last_cmd_vel_time_ = this->now();
  const double linear_xy = std::hypot(msg->linear.x, msg->linear.y);
  rotation_guard_rotating_now_ =
    std::abs(msg->angular.z) >= rotation_guard_angular_threshold_ &&
    linear_xy <= rotation_guard_linear_threshold_;

  if (rotation_guard_rotating_now_) {
    enterRotationGuard("cmd_vel");
  }
}

void PCLLocalization::navigationStatusReceived(const std_msgs::msg::String::ConstSharedPtr msg)
{
  if (!rotation_guard_enabled_ || !msg) {
    return;
  }

  const std::string & data = msg->data;
  const bool has_current_state = data.find("\"current_state\"") != std::string::npos;
  const bool navigation_active =
    !has_current_state ||
    data.find("\"current_state\":\"executing\"") != std::string::npos ||
    data.find("\"current_state\": \"executing\"") != std::string::npos ||
    data.find("\"current_state\":\"planning\"") != std::string::npos ||
    data.find("\"current_state\": \"planning\"") != std::string::npos;
  const bool turning =
    navigation_active &&
    (data.find("\"detailed_state\"") != std::string::npos ||
     data.find("\"current_detailed_state\"") != std::string::npos) &&
    data.find("TURNING") != std::string::npos;

  if (turning) {
    rotation_guard_state_active_ = true;
    rotation_guard_rotating_now_ = true;
    enterRotationGuard("navigation_status");
  } else if (rotation_guard_state_active_) {
    rotation_guard_state_active_ = false;
    rotation_guard_rotating_now_ = false;
    rotation_guard_settle_until_ =
      this->now() + rclcpp::Duration::from_seconds(rotation_guard_settle_sec_);
    RCLCPP_INFO(get_logger(), "NDT rotation guard state ended; entering settle window.");
  }
}

void PCLLocalization::updateRotationGuard()
{
  rotation_guard_active_debug_ = false;
  rotation_guard_settle_debug_ = false;
  rotation_guard_age_debug_ = 0.0;
  rotation_guard_source_debug_.clear();

  if (!rotation_guard_enabled_ || !rotation_guard_active_) {
    return;
  }

  const rclcpp::Time now = this->now();
  const double age_sec = (now - rotation_guard_start_time_).seconds();
  rotation_guard_age_debug_ = age_sec;

  if (age_sec < 0.0 || age_sec > rotation_guard_max_duration_sec_) {
    RCLCPP_WARN(
      get_logger(),
      "NDT rotation guard expired after %.2f sec; pose_jump checks are enabled again.",
      age_sec);
    rotation_guard_active_ = false;
    rotation_guard_state_active_ = false;
    rotation_guard_rotating_now_ = false;
    rotation_guard_hold_count_ = 0;
    return;
  }

  if (now > rotation_guard_settle_until_) {
    rotation_guard_active_ = false;
    rotation_guard_state_active_ = false;
    rotation_guard_rotating_now_ = false;
    rotation_guard_hold_count_ = 0;
    RCLCPP_INFO(get_logger(), "NDT rotation guard settled.");
    return;
  }

  rotation_guard_active_debug_ = true;
  rotation_guard_settle_debug_ = !rotation_guard_state_active_ && !rotation_guard_rotating_now_;
  rotation_guard_source_debug_ = rotation_guard_source_;
}

bool PCLLocalization::rotationGuardActive()
{
  updateRotationGuard();
  return rotation_guard_active_debug_;
}

bool PCLLocalization::rotationGuardInSettle()
{
  updateRotationGuard();
  return rotation_guard_settle_debug_;
}

bool PCLLocalization::lookupOdomBodyPlanarPose(
  const rclcpp::Time & stamp,
  double & x,
  double & y,
  double & yaw)
{
  auto extract_planar_pose = [&](const geometry_msgs::msg::TransformStamped & tf) {
    x = tf.transform.translation.x;
    y = tf.transform.translation.y;
    Eigen::Quaterniond q(
      tf.transform.rotation.w,
      tf.transform.rotation.x,
      tf.transform.rotation.y,
      tf.transform.rotation.z);
    q.normalize();
    const Eigen::Matrix3d rot = q.toRotationMatrix();
    yaw = std::atan2(rot(1, 0), rot(0, 0));
  };

  try {
    const auto tf_odom_body = tfbuffer_.lookupTransform(
      odom_frame_id_, fastlio_body_frame_, stamp);
    extract_planar_pose(tf_odom_body);
    return true;
  } catch (const tf2::TransformException &) {
  }

  try {
    const auto tf_odom_body = tfbuffer_.lookupTransform(
      odom_frame_id_, fastlio_body_frame_, tf2::TimePointZero);
    const rclcpp::Time tf_stamp = tf_odom_body.header.stamp;
    const double stamp_diff = std::abs((stamp - tf_stamp).seconds());
    if (stamp_diff > tf_max_stamp_mismatch_sec_) {
      return false;
    }
    extract_planar_pose(tf_odom_body);
    return true;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

bool PCLLocalization::shouldStoreMultiFrameKeyframe(const ScanFrame & frame) const
{
  if (!multi_frame_keyframe_filter_enabled_ || scan_frame_buffer_.empty()) {
    return true;
  }

  const ScanFrame & last_frame = scan_frame_buffer_.back();
  const double interval_sec = (frame.stamp - last_frame.stamp).seconds();
  if (multi_frame_keyframe_max_interval_sec_ > 0.0 &&
      interval_sec >= multi_frame_keyframe_max_interval_sec_) {
    return true;
  }

  if (!frame.has_odom_body_pose || !last_frame.has_odom_body_pose) {
    return true;
  }

  const double dx = frame.odom_body_x - last_frame.odom_body_x;
  const double dy = frame.odom_body_y - last_frame.odom_body_y;
  const double translation = std::hypot(dx, dy);
  const double yaw_delta = std::abs(normalizeAngle(frame.odom_body_yaw - last_frame.odom_body_yaw));

  return translation >= multi_frame_keyframe_translation_threshold_ ||
         yaw_delta >= multi_frame_keyframe_yaw_threshold_;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr PCLLocalization::buildMultiFrameSource(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & current_cloud,
  const rclcpp::Time & stamp,
  bool rotation_guard_active)
{
  multi_frame_source_frames_debug_ = 1;
  multi_frame_source_points_debug_ = current_cloud ? static_cast<int>(current_cloud->size()) : 0;

  if (!current_cloud) {
    return current_cloud;
  }

  if (!multi_frame_matching_enabled_ ||
      (multi_frame_use_only_when_rotating_ && !rotation_guard_active)) {
    scan_frame_buffer_.clear();
    multi_frame_buffer_frames_debug_ = 0;
    multi_frame_skipped_frames_debug_ = 0;
    return current_cloud;
  }

  ScanFrame frame;
  frame.stamp = stamp;
  frame.cloud.reset(new pcl::PointCloud<pcl::PointXYZI>(*current_cloud));
  frame.has_odom_body_pose = lookupOdomBodyPlanarPose(
    stamp, frame.odom_body_x, frame.odom_body_y, frame.odom_body_yaw);

  const bool store_current_frame = shouldStoreMultiFrameKeyframe(frame);
  if (store_current_frame) {
    scan_frame_buffer_.push_back(frame);
  } else {
    multi_frame_skipped_frames_debug_ += 1;
  }

  while (!scan_frame_buffer_.empty()) {
    const double age = (stamp - scan_frame_buffer_.front().stamp).seconds();
    if (age <= multi_frame_window_sec_ &&
        static_cast<int>(scan_frame_buffer_.size()) <= multi_frame_max_frames_) {
      break;
    }
    scan_frame_buffer_.pop_front();
  }
  multi_frame_buffer_frames_debug_ = static_cast<int>(scan_frame_buffer_.size());

  if (scan_frame_buffer_.size() <= 1 && store_current_frame) {
    return current_cloud;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr merged(new pcl::PointCloud<pcl::PointXYZI>());
  for (const auto & buffered_frame : scan_frame_buffer_) {
    if (buffered_frame.cloud) {
      *merged += *buffered_frame.cloud;
    }
  }
  if (!store_current_frame) {
    *merged += *current_cloud;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::VoxelGrid<pcl::PointXYZI> multi_frame_filter;
  multi_frame_filter.setLeafSize(
    multi_frame_voxel_leaf_size_, multi_frame_voxel_leaf_size_, multi_frame_voxel_leaf_size_);
  multi_frame_filter.setInputCloud(merged);
  multi_frame_filter.filter(*downsampled);

  if (static_cast<int>(downsampled->size()) > multi_frame_max_points_) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr capped(new pcl::PointCloud<pcl::PointXYZI>());
    capped->reserve(multi_frame_max_points_);
    const double stride =
      static_cast<double>(downsampled->size()) / static_cast<double>(multi_frame_max_points_);
    for (int i = 0; i < multi_frame_max_points_; ++i) {
      const size_t index = std::min(
        downsampled->size() - 1,
        static_cast<size_t>(std::floor(static_cast<double>(i) * stride)));
      capped->push_back((*downsampled)[index]);
    }
    downsampled = capped;
  }

  multi_frame_source_frames_debug_ =
    static_cast<int>(scan_frame_buffer_.size()) + (store_current_frame ? 0 : 1);
  multi_frame_source_points_debug_ = static_cast<int>(downsampled->size());
  return downsampled;
}

/**
 * @brief 接收点云消息回调（核心函数）
 *
 * 这是定位功能的核心处理函数，流程如下：
 * 1. 如果启用IMU，进行点云去畸变
 * 2. 对点云进行体素滤波降采样
 * 3. 根据距离范围过滤点
 * 4. 使用配准算法与地图对齐
 * 5. 发布定位结果、TF变换和路径
 * 
 * @param msg 点云消息
 */
void PCLLocalization::cloudReceived(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (!msg) {return;}
  last_scan_ptr_ = msg;  // 保存最新扫描，供手动 initialpose 后立即重定位

  // 检查是否已接收地图和初始位姿
  if (!map_recieved_ || !initialpose_recieved_) {return;}
  // RCLCPP_INFO(get_logger(), "cloudReceived");  // 已屏蔽：频繁输出，影响日志可读性
  
  // 将ROS消息转换为PCL点云
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *cloud_ptr);

  // ★★★ 核心修改 3：把实时扫描点云也一起掰平到 ROS标准 坐标系 (Z朝上) ★★★
  // 只有保证 source（实时点云）和 target（地图）同在正确的重力方向下，NDT 才正常
  Eigen::Quaternionf q_cam_to_ros(0.5, -0.5, -0.5, 0.5);
  Eigen::Vector3f t_cam_to_ros(0.0, 0.0, 0.0);
  pcl::transformPointCloud(*cloud_ptr, *cloud_ptr, t_cam_to_ros, q_cam_to_ros);
  // ★★★ 转换结束 ★★★

  // ========== 点云去畸变 ==========
  if (use_imu_) {
    // 如果启用IMU，使用IMU数据对点云进行运动畸变校正
    double received_time = msg->header.stamp.sec +
      msg->header.stamp.nanosec * 1e-9;
    lidar_undistortion_.adjustDistortion(cloud_ptr, received_time);
  }

  // ========== 点云滤波 ==========
  // 使用体素滤波器进行降采样，减少计算量
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  voxel_grid_filter_.setInputCloud(cloud_ptr);
  voxel_grid_filter_.filter(*filtered_cloud_ptr);

  // ========== 距离过滤 ==========
  // 根据配置的远近程范围过滤点云
  double r;
  pcl::PointCloud<pcl::PointXYZI> tmp;
  for (const auto & p : filtered_cloud_ptr->points) {
    r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));  // 计算点到原点的距离
    if (scan_min_range_ < r && r < scan_max_range_) {
      tmp.push_back(p);  // 保留有效范围内的点
    }
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_ptr(new pcl::PointCloud<pcl::PointXYZI>(tmp));

  if (static_cast<int>(tmp_ptr->size()) < min_scan_points_) {
    pose_jump_candidate_active_ = false;
    pose_jump_candidate_count_ = 0;
    resetCandidateConfirmation();
    const char * reason = tmp_ptr->empty() ? "empty_scan" : "too_few_scan_points";
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Rejecting NDT scan before registration: %s, effective_points=%zu < min_scan_points=%d "
      "(raw=%zu voxel=%zu). No pose or TF will be published for this scan.",
      reason, tmp_ptr->size(), min_scan_points_, cloud_ptr->size(), filtered_cloud_ptr->size());
    publishLocalizationStatus(
      "rejected", reason, false, -1.0,
      static_cast<int>(tmp_ptr->size()), rclcpp::Time(msg->header.stamp));
    return;
  }

  const bool rotation_guard_active = rotationGuardActive();
  const bool rotation_guard_settle = rotation_guard_settle_debug_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud_ptr =
    buildMultiFrameSource(tmp_ptr, rclcpp::Time(msg->header.stamp), rotation_guard_active);
  if (static_cast<int>(source_cloud_ptr->size()) < min_scan_points_) {
    pose_jump_candidate_active_ = false;
    pose_jump_candidate_count_ = 0;
    resetCandidateConfirmation();
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Rejecting NDT scan after multi-frame build: effective_points=%zu < min_scan_points=%d.",
      source_cloud_ptr->size(), min_scan_points_);
    publishLocalizationStatus(
      "rejected", "too_few_multiframe_points", false, -1.0,
      static_cast<int>(source_cloud_ptr->size()), rclcpp::Time(msg->header.stamp));
    return;
  }
  
  // 设置配准源点云
  registration_->setInputSource(source_cloud_ptr);

  // =====================================================================
  // Plan B: Fast-LIO delta guess — 用 camera_init→body TF 的位姿差推进 init_guess
  // 防止长间隙期间 corrent_pose 冻结导致的 pose_jump 死亡螺旋
  // =====================================================================
  fastlio_delta_applied_ = false;
  fastlio_delta_reject_reason_.clear();
  fastlio_delta_translation_debug_ = 0.0;
  fastlio_delta_yaw_debug_ = 0.0;
  fastlio_dead_reckon_age_debug_ = (this->now() - last_accept_time_).seconds();
  fastlio_odom_body_used_debug_ = false;
  fastlio_delta_dt_debug_ = 0.0;
  fastlio_map_odom_guess_shift_debug_ = 0.0;
  ndt_candidate_pose_valid_debug_ = false;
  ndt_prev_candidate_valid_debug_ = false;
  ndt_candidate_delta_prev_xy_debug_ = 0.0;
  ndt_candidate_delta_prev_yaw_debug_ = 0.0;
  ndt_candidate_delta_last_good_xy_debug_ = 0.0;
  ndt_candidate_delta_last_good_yaw_debug_ = 0.0;

  if (use_fastlio_delta_guess_ && corrent_pose_with_cov_stamped_ptr_) {
    rclcpp::Time cloud_stamp = msg->header.stamp;
    geometry_msgs::msg::TransformStamped tf_body_in_cam;
    geometry_msgs::msg::TransformStamped tf_odom_body_curr;
    bool tf_ok = false;
    bool odom_body_tf_ok = false;

    auto reset_fastlio_delta_baseline = [this]() {
      has_prev_body_pose_ = false;
      has_prev_odom_body_pose_ = false;
    };

    auto transform_to_affine = [](const geometry_msgs::msg::Transform & transform) {
      Eigen::Affine3d affine = Eigen::Affine3d::Identity();
      affine.translation() = Eigen::Vector3d(
        transform.translation.x, transform.translation.y, transform.translation.z);
      Eigen::Quaterniond q(
        transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z);
      q.normalize();
      affine.linear() = q.toRotationMatrix();
      return affine;
    };

    auto lookup_tf_with_fallback = [&](
      const std::string & target_frame,
      const std::string & source_frame,
      geometry_msgs::msg::TransformStamped & out,
      const char * prefix) -> bool {
      try {
        out = tfbuffer_.lookupTransform(target_frame, source_frame, cloud_stamp);
        return true;
      } catch (const tf2::TransformException& ex) {
        try {
          out = tfbuffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
          try {
            const rclcpp::Time tf_stamp = out.header.stamp;
            const double stamp_diff = std::abs((cloud_stamp - tf_stamp).seconds());
            if (stamp_diff > tf_max_stamp_mismatch_sec_) {
              fastlio_delta_reject_reason_ = std::string(prefix) + "_stamp_mismatch";
              return false;
            }
            return true;
          } catch (const std::exception& e) {
            fastlio_delta_reject_reason_ = std::string(prefix) + "_clock_mismatch";
            return false;
          }
        } catch (const tf2::TransformException& ex2) {
          fastlio_delta_reject_reason_ = std::string(prefix) + "_lookup_failed";
          return false;
        }
      }
    };

    // Step 1: 按点云时间戳查 TF
    tf_ok = lookup_tf_with_fallback(
      fastlio_camera_frame_, fastlio_body_frame_, tf_body_in_cam, "fastlio_tf");
    if (tf_ok) {
      odom_body_tf_ok = lookup_tf_with_fallback(
        odom_frame_id_, fastlio_body_frame_, tf_odom_body_curr, "odom_body_tf");
    }

    if (tf_ok && odom_body_tf_ok) {
      double body_x = tf_body_in_cam.transform.translation.x;
      double body_y = tf_body_in_cam.transform.translation.y;
      double body_z = tf_body_in_cam.transform.translation.z;
      double body_qx = tf_body_in_cam.transform.rotation.x;
      double body_qy = tf_body_in_cam.transform.rotation.y;
      double body_qz = tf_body_in_cam.transform.rotation.z;
      double body_qw = tf_body_in_cam.transform.rotation.w;

      Eigen::Affine3d T_odom_body_curr = transform_to_affine(tf_odom_body_curr.transform);
      fastlio_odom_body_used_debug_ = true;

      if (!has_prev_body_pose_ || !has_prev_odom_body_pose_) {
        // Step 2: 第一帧只建立基准
        prev_body_x_ = body_x;  prev_body_y_ = body_y;  prev_body_z_ = body_z;
        prev_body_qx_ = body_qx; prev_body_qy_ = body_qy;
        prev_body_qz_ = body_qz; prev_body_qw_ = body_qw;
        prev_cloud_stamp_ = cloud_stamp;
        prev_T_odom_body_ = T_odom_body_curr;
        has_prev_body_pose_ = true;
        has_prev_odom_body_pose_ = true;
        fastlio_delta_reject_reason_ = "no_prev_pose";
      } else if (fastlio_dead_reckon_age_debug_ > fastlio_max_dead_reckon_sec_) {
        // Step 3: dead reckoning 超时, 停止推进
        reset_fastlio_delta_baseline();
        fastlio_delta_reject_reason_ = "dead_reckon_timeout";
      } else {
        fastlio_delta_dt_debug_ = (cloud_stamp - prev_cloud_stamp_).seconds();
        if (fastlio_delta_dt_debug_ < 0.0 || fastlio_delta_dt_debug_ > fastlio_max_delta_dt_) {
          reset_fastlio_delta_baseline();
          fastlio_delta_reject_reason_ = "delta_dt_exceeded";
          fastlio_delta_translation_debug_ = 0.0;
          fastlio_delta_yaw_debug_ = 0.0;
        } else {

          // Step 4a: 直接使用 TF 链得到的 odom->body 前后帧差。
          // odom->camera_init 静态 TF 已经把 Fast-LIO 非标轴接到标准 odom 轴上，
          // 因此这里不能再对 odom->body 做手工轴变换。
          const Eigen::Affine3d T_delta_ros =
            prev_T_odom_body_.inverse() * T_odom_body_curr;

          const Eigen::Vector3d delta_translation = T_delta_ros.translation();
          const Eigen::Matrix3d delta_rot = T_delta_ros.rotation();
          const double delta_trans = std::hypot(delta_translation.x(), delta_translation.y());
          const double dyaw = std::atan2(delta_rot(1, 0), delta_rot(0, 0));

          // Step 4c: 单帧异常检测。超限直接重置基准，不推进。
          if (std::abs(delta_translation.x()) > fastlio_max_delta_translation_ ||
              std::abs(delta_translation.y()) > fastlio_max_delta_translation_ ||
              std::abs(dyaw) > fastlio_max_delta_yaw_) {
            // 先填 debug 值再重置基准, 否则实机排查时不知道超限了多少
            fastlio_delta_translation_debug_ = delta_trans;
            fastlio_delta_yaw_debug_ = dyaw;
            reset_fastlio_delta_baseline();
            fastlio_delta_reject_reason_ = "max_delta_exceeded";
          } else {
            // Step 4d: map_body_to_map_odom — 正确的 init_guess 预测
            // 公式: T_map_odom_guess = T_map_odom_prev * T_odom_body_prev * T_delta_ros * T_odom_body_curr^(-1)
            // 原理: 先由 body 运动 delta 预测 map->body, 再用当前 odom->body 反推 map->odom

            // T_map_odom_prev 从 corrent_pose 提取
            Eigen::Affine3d T_map_odom_prev;
            tf2::fromMsg(corrent_pose_with_cov_stamped_ptr_->pose.pose, T_map_odom_prev);

            Eigen::Affine3d T_map_odom_guess =
              T_map_odom_prev * prev_T_odom_body_ * T_delta_ros * T_odom_body_curr.inverse();

            fastlio_map_odom_guess_shift_debug_ = std::hypot(
              T_map_odom_guess.translation().x() - T_map_odom_prev.translation().x(),
              T_map_odom_guess.translation().y() - T_map_odom_prev.translation().y());

            corrent_pose_with_cov_stamped_ptr_->pose.pose = tf2::toMsg(T_map_odom_guess);
            applyPlanarPoseConstraint(corrent_pose_with_cov_stamped_ptr_->pose.pose);

            fastlio_delta_applied_ = true;
            fastlio_delta_translation_debug_ = delta_trans;
            fastlio_delta_yaw_debug_ = dyaw;
          }
        }
      }

      // Step 5: 存储本帧 body 位姿供下帧使用
      if (has_prev_body_pose_) {
        prev_body_x_ = body_x;  prev_body_y_ = body_y;  prev_body_z_ = body_z;
        prev_body_qx_ = body_qx; prev_body_qy_ = body_qy;
        prev_body_qz_ = body_qz; prev_body_qw_ = body_qw;
        prev_T_odom_body_ = T_odom_body_curr;
        prev_cloud_stamp_ = cloud_stamp;
      }
    } else if (!fastlio_delta_reject_reason_.empty()) {
      reset_fastlio_delta_baseline();
    }
  }

  // ========== 配准定位 ==========
  // 将当前位姿转换为矩阵，作为配准的初始猜测
  Eigen::Affine3d affine;
  tf2::fromMsg(corrent_pose_with_cov_stamped_ptr_->pose.pose, affine);
  Eigen::Matrix4f init_guess = affine.matrix().cast<float>();

  // NDT 旋转先验: 用当前位姿的 roll/pitch 约束 NDT 优化，减少退化区域漂移
  bool rotation_prior_was_set = false;
  if (ndt_rotation_prior_enabled_ && ndt_rotation_prior_weight_ > 0.0) {
    if (auto * ndt_omp_ptr =
          dynamic_cast<pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> *>(
            registration_.get())) {
      Eigen::Matrix3d rot_init = init_guess.block<3, 3>(0, 0).cast<double>();
      double roll = std::atan2(rot_init(2, 1), rot_init(2, 2));
      double pitch = std::asin(std::max(-1.0, std::min(1.0, -rot_init(2, 0))));
      double yaw = std::atan2(rot_init(1, 0), rot_init(0, 0));
      Eigen::Vector3d prior_rpy(roll, pitch, yaw);
      ndt_omp_ptr->setRotationPrior(
        prior_rpy, ndt_rotation_prior_weight_, ndt_rotation_prior_roll_pitch_only_);
      rotation_prior_was_set = true;
    }
  }

  // 执行配准算法
  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  rclcpp::Clock system_clock;
  rclcpp::Time time_align_start = system_clock.now();  // 记录开始时间
  registration_->align(*output_cloud, init_guess);
  rclcpp::Time time_align_end = system_clock.now();  // 记录结束时间
  ndt_align_duration_sec_debug_ = time_align_end.seconds() - time_align_start.seconds();

  // 提取 NDT_OMP 诊断信息 (退化监控用)
  // mean_corr_dist 是重要退化指标: <0.5=匹配紧密, >1.5=可能退化
  last_mean_corr_dist_ = -1.0;
  if (auto * ndt_diag =
        dynamic_cast<pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> *>(
          registration_.get())) {
    last_mean_corr_dist_ = ndt_diag->getLastMeanCorrespondenceDistance();
    if (rotation_prior_was_set) {
      ndt_diag->clearRotationPrior();
    }
  }

  // 检查配准结果
  bool has_converged = registration_->hasConverged();  // 是否收敛
  double fitness_score = registration_->getFitnessScore();  // 配准得分
  if (!has_converged) {
    pose_jump_candidate_active_ = false;
    pose_jump_candidate_count_ = 0;
    resetCandidateConfirmation();
    RCLCPP_WARN(get_logger(), "The registration didn't converge.");
    publishLocalizationStatus(
      "rejected", "not_converged", has_converged, fitness_score,
      static_cast<int>(tmp_ptr->size()), rclcpp::Time(msg->header.stamp));
    publishLastGoodTransformIfFresh("non-converged registration");
    return;  // 配准未收敛，放弃此次结果
  }
  if (fitness_score > score_threshold_) {
    pose_jump_candidate_active_ = false;
    pose_jump_candidate_count_ = 0;
    resetCandidateConfirmation();
    RCLCPP_WARN(get_logger(), "The fitness score is over %lf, skip this result.", score_threshold_);
    publishLocalizationStatus(
      "rejected", rotation_guard_active ? "rotation_guard_high_fitness" : "high_fitness",
      has_converged, fitness_score,
      static_cast<int>(source_cloud_ptr->size()), rclcpp::Time(msg->header.stamp));
    publishLastGoodTransformIfFresh("high fitness score");
    // ★ 匹配质量差时不发布新TF，避免位姿跳变
    return;  // 放弃此次结果，不发布不可靠的新TF变换
  }

  // 获取最终的变换矩阵
  Eigen::Matrix4f final_transformation = registration_->getFinalTransformation();
  const Eigen::Matrix4f navigation_transformation =
    applyPlanarTransformConstraint(final_transformation);

  const double correction_translation = std::hypot(
    static_cast<double>(navigation_transformation(0, 3) - init_guess(0, 3)),
    static_cast<double>(navigation_transformation(1, 3) - init_guess(1, 3)));
  const Eigen::Matrix3d init_rot = init_guess.block<3, 3>(0, 0).cast<double>();
  const Eigen::Matrix3d result_rot = navigation_transformation.block<3, 3>(0, 0).cast<double>();
  const double init_yaw = std::atan2(init_rot(1, 0), init_rot(0, 0));
  const double result_yaw = std::atan2(result_rot(1, 0), result_rot(0, 0));
  const double yaw_error = result_yaw - init_yaw;
  const double correction_yaw = std::abs(std::atan2(std::sin(yaw_error), std::cos(yaw_error)));
  const double candidate_x = static_cast<double>(navigation_transformation(0, 3));
  const double candidate_y = static_cast<double>(navigation_transformation(1, 3));

  ndt_candidate_pose_valid_debug_ = true;
  ndt_candidate_x_debug_ = candidate_x;
  ndt_candidate_y_debug_ = candidate_y;
  ndt_candidate_yaw_debug_ = result_yaw;
  ndt_init_guess_x_debug_ = static_cast<double>(init_guess(0, 3));
  ndt_init_guess_y_debug_ = static_cast<double>(init_guess(1, 3));
  ndt_init_guess_yaw_debug_ = init_yaw;
  ndt_prev_candidate_valid_debug_ = has_last_ndt_candidate_debug_;
  if (has_last_ndt_candidate_debug_) {
    ndt_candidate_delta_prev_xy_debug_ =
      std::hypot(candidate_x - last_ndt_candidate_x_debug_, candidate_y - last_ndt_candidate_y_debug_);
    const double prev_yaw_error = result_yaw - last_ndt_candidate_yaw_debug_;
    ndt_candidate_delta_prev_yaw_debug_ =
      std::abs(std::atan2(std::sin(prev_yaw_error), std::cos(prev_yaw_error)));
  }
  if (has_last_good_transform_) {
    const auto & last_t = last_good_transform_.transform.translation;
    const auto & last_q = last_good_transform_.transform.rotation;
    ndt_candidate_delta_last_good_xy_debug_ =
      std::hypot(candidate_x - last_t.x, candidate_y - last_t.y);
    const double last_good_yaw = tf2::getYaw(last_q);
    const double last_good_yaw_error = result_yaw - last_good_yaw;
    ndt_candidate_delta_last_good_yaw_debug_ =
      std::abs(std::atan2(std::sin(last_good_yaw_error), std::cos(last_good_yaw_error)));
  }
  last_ndt_candidate_x_debug_ = candidate_x;
  last_ndt_candidate_y_debug_ = candidate_y;
  last_ndt_candidate_yaw_debug_ = result_yaw;
  has_last_ndt_candidate_debug_ = true;

  const bool initialpose_reacquiring = initialPoseReacquireActive();
  const double pose_jump_translation_limit =
    initialpose_reacquiring ? initialpose_max_pose_jump_translation_ : max_pose_jump_translation_;
  const double pose_jump_yaw_limit =
    initialpose_reacquiring ? initialpose_max_pose_jump_yaw_ : max_pose_jump_yaw_;
  bool accepted_confirmed_pose_jump = false;
  const bool pose_jump_exceeded =
    reject_pose_jump_ && has_last_good_transform_ &&
    (correction_translation > pose_jump_translation_limit ||
     correction_yaw > pose_jump_yaw_limit);
  if (pose_jump_exceeded && rotation_guard_active && !initialpose_reacquiring) {
    rotation_guard_hold_count_ += 1;
    pose_jump_candidate_active_ = false;
    pose_jump_candidate_count_ = 0;
    resetCandidateConfirmation();
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 500,
      "Holding NDT pose jump during rotation guard%s: translation=%.3f limit=%.3f "
      "yaw=%.3f limit=%.3f fitness=%.3f holds=%d",
      rotation_guard_settle ? " settle" : "",
      correction_translation, pose_jump_translation_limit,
      correction_yaw, pose_jump_yaw_limit, fitness_score, rotation_guard_hold_count_);
    publishLocalizationStatus(
      "confirming",
      rotation_guard_settle ? "rotation_guard_settle" : "rotation_guard_hold",
      has_converged, fitness_score,
      static_cast<int>(source_cloud_ptr->size()), rclcpp::Time(msg->header.stamp),
      correction_translation, correction_yaw);
    publishLastGoodTransformIfFresh("rotation guard pose jump");
    return;
  }
  if (pose_jump_exceeded) {
    resetCandidateConfirmation();
    const bool can_confirm_pose_jump =
      pose_jump_reacquire_enabled_ &&
      !initialpose_reacquiring &&
      fitness_score <= pose_jump_reacquire_max_fitness_ &&
      correction_translation <= pose_jump_reacquire_max_translation_ &&
      correction_yaw <= pose_jump_reacquire_max_yaw_;

    if (can_confirm_pose_jump) {
      const double candidate_yaw = result_yaw;
      const double candidate_xy_delta = pose_jump_candidate_active_ ?
        std::hypot(candidate_x - pose_jump_candidate_x_, candidate_y - pose_jump_candidate_y_) :
        0.0;
      const double candidate_yaw_delta = pose_jump_candidate_active_ ?
        std::abs(std::atan2(
          std::sin(candidate_yaw - pose_jump_candidate_yaw_),
          std::cos(candidate_yaw - pose_jump_candidate_yaw_))) :
        0.0;
      const bool same_candidate =
        pose_jump_candidate_active_ &&
        candidate_xy_delta <= pose_jump_reacquire_xy_tolerance_ &&
        candidate_yaw_delta <= pose_jump_reacquire_yaw_tolerance_;

      if (!same_candidate) {
        pose_jump_candidate_count_ = 1;
        pose_jump_candidate_x_ = candidate_x;
        pose_jump_candidate_y_ = candidate_y;
        pose_jump_candidate_yaw_ = candidate_yaw;
        pose_jump_candidate_active_ = true;
      } else {
        pose_jump_candidate_count_ += 1;
      }

      if (pose_jump_candidate_count_ >= pose_jump_reacquire_required_frames_) {
        accepted_confirmed_pose_jump = true;
        RCLCPP_WARN(
          get_logger(),
          "Accepting confirmed NDT pose jump: translation=%.3f normal_limit=%.3f yaw=%.3f "
          "normal_limit=%.3f fitness=%.3f consistent_frames=%d/%d",
          correction_translation, pose_jump_translation_limit,
          correction_yaw, pose_jump_yaw_limit, fitness_score,
          pose_jump_candidate_count_, pose_jump_reacquire_required_frames_);
        pose_jump_candidate_active_ = false;
        pose_jump_candidate_count_ = 0;
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 500,
          "Holding NDT pose jump candidate for confirmation: translation=%.3f normal_limit=%.3f "
          "yaw=%.3f normal_limit=%.3f fitness=%.3f consistent_frames=%d/%d",
          correction_translation, pose_jump_translation_limit,
          correction_yaw, pose_jump_yaw_limit, fitness_score,
          pose_jump_candidate_count_, pose_jump_reacquire_required_frames_);
        publishLocalizationStatus(
          "confirming", "pose_jump_candidate", has_converged, fitness_score,
          static_cast<int>(source_cloud_ptr->size()), rclcpp::Time(msg->header.stamp),
          correction_translation, correction_yaw);
        publishLastGoodTransformIfFresh("pose jump candidate");
        return;
      }
    } else {
      pose_jump_candidate_active_ = false;
      pose_jump_candidate_count_ = 0;
    }
  } else {
    pose_jump_candidate_active_ = false;
    pose_jump_candidate_count_ = 0;
  }

  if (pose_jump_exceeded && !accepted_confirmed_pose_jump) {
    RCLCPP_WARN(
      get_logger(),
      "Rejecting NDT pose jump%s: translation=%.3f limit=%.3f yaw=%.3f limit=%.3f fitness=%.3f",
      initialpose_reacquiring ? " during initialpose reacquire" : "",
      correction_translation, pose_jump_translation_limit,
      correction_yaw, pose_jump_yaw_limit, fitness_score);
    publishLocalizationStatus(
      "rejected", "pose_jump", has_converged, fitness_score,
      static_cast<int>(source_cloud_ptr->size()), rclcpp::Time(msg->header.stamp),
      correction_translation, correction_yaw);
    publishLastGoodTransformIfFresh("pose jump");
    return;
  }

  if (
    !pose_jump_exceeded &&
    candidate_confirmation_enabled_ &&
    !initialpose_reacquiring &&
    has_last_good_transform_) {
    const bool translation_needs_confirmation =
      candidate_confirmation_min_translation_ > 0.0 &&
      correction_translation >= candidate_confirmation_min_translation_;
    const bool yaw_needs_confirmation =
      candidate_confirmation_min_yaw_ > 0.0 &&
      correction_yaw >= candidate_confirmation_min_yaw_;
    const bool mean_corr_rejected =
      candidate_confirmation_max_mean_corr_dist_ > 0.0 &&
      std::isfinite(last_mean_corr_dist_) &&
      last_mean_corr_dist_ > candidate_confirmation_max_mean_corr_dist_;
    const bool candidate_needs_confirmation =
      translation_needs_confirmation || yaw_needs_confirmation || mean_corr_rejected;

    if (candidate_needs_confirmation) {
      const bool fitness_ok =
        candidate_confirmation_max_fitness_ <= 0.0 ||
        fitness_score <= candidate_confirmation_max_fitness_;
      if (!fitness_ok || mean_corr_rejected) {
        resetCandidateConfirmation();
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 500,
          "Rejecting NDT candidate before publication: translation=%.3f trigger=%.3f "
          "yaw=%.3f trigger=%.3f fitness=%.3f max=%.3f mean_corr=%.3f max=%.3f",
          correction_translation, candidate_confirmation_min_translation_,
          correction_yaw, candidate_confirmation_min_yaw_,
          fitness_score, candidate_confirmation_max_fitness_,
          last_mean_corr_dist_, candidate_confirmation_max_mean_corr_dist_);
        publishLocalizationStatus(
          "rejected", "candidate_confirmation_low_quality", has_converged, fitness_score,
          static_cast<int>(source_cloud_ptr->size()), rclcpp::Time(msg->header.stamp),
          correction_translation, correction_yaw);
        publishLastGoodTransformIfFresh("low quality NDT candidate");
        return;
      }

      const double candidate_yaw = result_yaw;
      const double candidate_xy_delta = candidate_confirmation_active_ ?
        std::hypot(candidate_x - candidate_confirmation_x_, candidate_y - candidate_confirmation_y_) :
        0.0;
      const double candidate_yaw_delta = candidate_confirmation_active_ ?
        std::abs(std::atan2(
          std::sin(candidate_yaw - candidate_confirmation_yaw_),
          std::cos(candidate_yaw - candidate_confirmation_yaw_))) :
        0.0;
      const bool same_candidate =
        candidate_confirmation_active_ &&
        candidate_xy_delta <= candidate_confirmation_xy_tolerance_ &&
        candidate_yaw_delta <= candidate_confirmation_yaw_tolerance_;

      if (!same_candidate) {
        candidate_confirmation_count_ = 1;
        candidate_confirmation_x_ = candidate_x;
        candidate_confirmation_y_ = candidate_y;
        candidate_confirmation_yaw_ = candidate_yaw;
        candidate_confirmation_active_ = true;
      } else {
        candidate_confirmation_count_ += 1;
      }

      if (candidate_confirmation_count_ >= candidate_confirmation_required_frames_) {
        RCLCPP_WARN(
          get_logger(),
          "Accepting confirmed NDT candidate: translation=%.3f trigger=%.3f yaw=%.3f "
          "trigger=%.3f fitness=%.3f consistent_frames=%d/%d",
          correction_translation, candidate_confirmation_min_translation_,
          correction_yaw, candidate_confirmation_min_yaw_,
          fitness_score, candidate_confirmation_count_, candidate_confirmation_required_frames_);
        resetCandidateConfirmation();
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 500,
          "Holding NDT candidate for confirmation: translation=%.3f trigger=%.3f "
          "yaw=%.3f trigger=%.3f fitness=%.3f consistent_frames=%d/%d",
          correction_translation, candidate_confirmation_min_translation_,
          correction_yaw, candidate_confirmation_min_yaw_,
          fitness_score, candidate_confirmation_count_, candidate_confirmation_required_frames_);
        publishLocalizationStatus(
          "confirming", "candidate_confirmation", has_converged, fitness_score,
          static_cast<int>(source_cloud_ptr->size()), rclcpp::Time(msg->header.stamp),
          correction_translation, correction_yaw);
        publishLastGoodTransformIfFresh("NDT candidate confirmation");
        return;
      }
    } else {
      resetCandidateConfirmation();
    }
  } else if (!pose_jump_exceeded) {
    resetCandidateConfirmation();
  }

  // ★★★ 核心修改 4：已经删除了导致崩溃的坐标系矩阵连乘补正 ★★★
  // 因为现在输入的地图和点云都已经完全处在规范的 ROS 坐标系下，
  // 获取到的 final_transformation 本身就是准确纯粹的 map -> odom，不再需要多余补偿。

  // 从变换矩阵中提取旋转并转换为四元数
  Eigen::Matrix3d rot_mat = navigation_transformation.block<3, 3>(0, 0).cast<double>();  // 3x3旋转矩阵
  Eigen::Quaterniond quat_eig(rot_mat);  // 转换为四元数
  geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig);  // 转换为ROS消息

  // 更新位姿消息的时间戳和坐标系
  corrent_pose_with_cov_stamped_ptr_->header.stamp = msg->header.stamp;
  corrent_pose_with_cov_stamped_ptr_->header.frame_id = global_frame_id_;
  
  // 从变换矩阵中提取平移并更新位姿
  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x = static_cast<double>(navigation_transformation(0, 3));
  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.y = static_cast<double>(navigation_transformation(1, 3));
  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.z = static_cast<double>(navigation_transformation(2, 3));
  corrent_pose_with_cov_stamped_ptr_->pose.pose.orientation = quat_msg;
  
  // 发布定位结果
  last_accept_time_ = this->now();  // Fast-LIO delta guess: 重置 dead reckoning 计时
  pose_pub_->publish(*corrent_pose_with_cov_stamped_ptr_);
  publishLocalizationStatus(
    "accepted",
    accepted_confirmed_pose_jump ? "confirmed_pose_jump" : "ok",
    has_converged, fitness_score,
    static_cast<int>(source_cloud_ptr->size()), rclcpp::Time(msg->header.stamp),
    correction_translation, correction_yaw);

  // ========== 发布TF变换 ==========
  // 发布map到base_link的变换，供其他节点使用
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = this->now();
  transform_stamped.header.frame_id = global_frame_id_;      // 父坐标系
  transform_stamped.child_frame_id = base_frame_id_;         // 子坐标系（机器人基座）
  transform_stamped.transform.translation.x = static_cast<double>(navigation_transformation(0, 3));
  transform_stamped.transform.translation.y = static_cast<double>(navigation_transformation(1, 3));
  transform_stamped.transform.translation.z = static_cast<double>(navigation_transformation(2, 3));
  transform_stamped.transform.rotation = quat_msg;
  last_good_transform_ = transform_stamped;
  last_good_transform_time_ = this->now();
  has_last_good_transform_ = true;

  // Phase 1: fusion DEGRADED/LOST/FUSION_TIMEOUT 时抑制 TF
  if (true) {
    broadcaster_.sendTransform(transform_stamped);  // 广播TF变换
  }

  // ========== 更新并发布路径 ==========
  // 将当前位姿添加到路径中
  geometry_msgs::msg::PoseStamped::SharedPtr pose_stamped_ptr(new geometry_msgs::msg::PoseStamped);
  pose_stamped_ptr->header.stamp = msg->header.stamp;
  pose_stamped_ptr->header.frame_id = global_frame_id_;
  pose_stamped_ptr->pose = corrent_pose_with_cov_stamped_ptr_->pose.pose;
  path_ptr_->poses.push_back(*pose_stamped_ptr);  // 添加到路径
  path_pub_->publish(*path_ptr_);  // 发布路径

  // ========== 调试信息输出 ==========
  if (enable_debug_) {
    std::cout << "number of filtered cloud points: " << filtered_cloud_ptr->size() << std::endl;
    std::cout << "align time:" << time_align_end.seconds() - time_align_start.seconds() <<
      "[sec]" << std::endl;
    std::cout << "has converged: " << has_converged << std::endl;
    std::cout << "fitness score: " << fitness_score << std::endl;
    std::cout << "final transformation:" << std::endl;
    std::cout << final_transformation << std::endl;
    
    /* 计算旋转角度变化
     * 使用旋转矩阵的迹（trace）计算旋转角度
     * trace(RotationMatrix) = 2(cos(theta) + 1)
     */
    double init_cos_angle = 0.5 *
      (init_guess.coeff(0, 0) + init_guess.coeff(1, 1) + init_guess.coeff(2, 2) - 1);
    double cos_angle = 0.5 *
      (final_transformation.coeff(0,
      0) + final_transformation.coeff(1, 1) + final_transformation.coeff(2, 2) - 1);
    double init_angle = acos(init_cos_angle);
    double angle = acos(cos_angle);
    // 参考: https://twitter.com/Atsushi_twi/status/1185868416864808960
    double delta_angle = abs(atan2(sin(init_angle - angle), cos(init_angle - angle)));
    std::cout << "delta_angle:" << delta_angle * 180 / M_PI << "[deg]" << std::endl;
    std::cout << "-----------------------------------------------------" << std::endl;
  }
}
