/**
 * @file lidar_localization_component.hpp
 * @brief 基于PCL的激光雷达定位ROS2组件头文件
 * 
 * 本文件定义了PCLLocalization类，实现使用PCL库进行激光雷达配准定位的功能
 * 支持NDT、GICP等多种配准算法，支持IMU去畸变、里程计预测等功能
 * 
 * 依赖库：
 * - PCL: 点云处理库
 * - pcl_omp: PCL的多线程扩展
 * - ROS2: rclcpp, tf2, lifecycle等
 */

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

// PCL配准算法头文件
#include <pcl/registration/ndt.h>      // 正态分布变换（NDT）
#include <pcl/registration/gicp.h>     // 通用迭代最近点（GICP）

// TF2坐标变换库
#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl_conversions/pcl_conversions.h>  // PCL与ROS消息转换

// ROS2生命周期管理
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

// ROS2消息类型
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

// PCL多线程扩展
#include <pclomp/ndt_omp.h>
#include <pclomp/ndt_omp_impl.hpp>
#include <pclomp/voxel_grid_covariance_omp.h>
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>
#include <pclomp/gicp_omp.h>
#include <pclomp/gicp_omp_impl.hpp>

#include "lidar_localization/lidar_undistortion.hpp"  // 点云去畸变模块

using namespace std::chrono_literals;

/**
 * @brief 激光雷达定位ROS2生命周期组件
 * 
 * 该类实现了一个ROS2生命周期节点，支持通过NDT或GICP算法将激光雷达点云与地图配准
 * 从而实现精确定位。主要功能包括：
 * - 支持多种配准算法（NDT, NDT_OMP, GICP, GICP_OMP）
 * - 支持IMU点云去畸变
 * - 支持里程计位姿预测
 * - 支持动态地图加载和PCD文件加载
 * - 发布定位结果、TF变换和运动路径
 * 
 * 生命周期状态转换：
 * unconfigured -> configure -> inactive -> activate -> active -> deactivate -> inactive -> cleanup -> unconfigured
 */
class PCLLocalization : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * @brief 构造函数
   * @param options ROS节点选项
   */
  explicit PCLLocalization(const rclcpp::NodeOptions & options);

  /**
   * @brief 生命周期状态转换回调类型
   */
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  // ========== 生命周期回调函数 ==========
  /**
   * @brief 配置阶段回调，初始化参数、发布订阅、配准算法
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State &);
  
  /**
   * @brief 激活阶段回调，激活发布者，加载地图，设置初始位姿
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State &);
  
  /**
   * @brief 停用阶段回调，停用发布者
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
  
  /**
   * @brief 清理阶段回调，释放资源
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
  
  /**
   * @brief 关闭阶段回调
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);
  
  /**
   * @brief 错误处理回调
   */
  CallbackReturn on_error(const rclcpp_lifecycle::State & state);

  // ========== 初始化函数 ==========
  void initializeParameters();   ///< 初始化ROS参数
  void initializePubSub();      ///< 初始化发布者和订阅者
  void initializeRegistration(); ///< 初始化配准算法
  void applyPlanarPoseConstraint(geometry_msgs::msg::Pose & pose) const;
  Eigen::Matrix4f applyPlanarTransformConstraint(const Eigen::Matrix4f & transform) const;
  bool initialPoseReacquireActive();
  bool publishLastGoodTransformIfFresh(const char * reject_reason);
  void publishLocalizationStatus(
    const char * state,
    const char * reason,
    bool has_converged,
    double fitness_score,
    int filtered_points,
    const rclcpp::Time & stamp,
    double correction_translation = 0.0,
    double correction_yaw = 0.0);

  // ========== 消息回调函数 ==========
  void initialPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);  ///< 初始位姿回调
  void mapReceived(const sensor_msgs::msg::PointCloud2::SharedPtr msg);                          ///< 地图回调
  void odomReceived(const nav_msgs::msg::Odometry::ConstSharedPtr msg);                          ///< 里程计回调
  void imuReceived(const sensor_msgs::msg::Imu::ConstSharedPtr msg);                             ///< IMU回调
  void cloudReceived(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);                   ///< 点云回调（核心）

  // ========== 成员变量 ==========
  tf2_ros::TransformBroadcaster broadcaster_;  ///< TF变换广播器，发布坐标变换
  rclcpp::Clock clock_;                        ///< ROS时钟
  tf2_ros::Buffer tfbuffer_;                   ///< TF缓冲区，存储坐标变换
  tf2_ros::TransformListener tflistener_;      ///< TF监听器，接收坐标变换

  // 订阅者和发布者
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::ConstSharedPtr
    initial_pose_sub_;  ///< 初始位姿订阅者
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    pose_pub_;  ///< 定位结果发布者
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr
    path_pub_;  ///< 路径发布者
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    initial_map_pub_;  ///< 初始地图发布者
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr
    status_pub_;  ///< 定位质量状态发布者
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::ConstSharedPtr
    map_sub_;  ///< 地图订阅者
  rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr
    odom_sub_;  ///< 里程计订阅者
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::ConstSharedPtr
    cloud_sub_;  ///< 点云订阅者
  rclcpp::Subscription<sensor_msgs::msg::Imu>::ConstSharedPtr
    imu_sub_;  ///< IMU订阅者
  // 配准和滤波
  boost::shared_ptr<pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>> registration_;  ///< 配准算法对象
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter_;  ///< 体素滤波器，用于降采样

  // 状态数据
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr corrent_pose_with_cov_stamped_ptr_;  ///< 当前位姿
  nav_msgs::msg::Path::SharedPtr path_ptr_;            ///< 路径数据
  sensor_msgs::msg::PointCloud2::ConstSharedPtr last_scan_ptr_;  ///< 最后一次扫描数据
  geometry_msgs::msg::TransformStamped last_good_transform_;  ///< 最后一次可信定位TF

  bool map_recieved_{false};         ///< 标记是否已接收地图
  bool initialpose_recieved_{false}; ///< 标记是否已接收初始位姿
  bool has_last_good_transform_{false}; ///< 是否已有可信定位TF
  rclcpp::Time last_good_transform_time_{0, 0, RCL_ROS_TIME}; ///< 最后可信TF发布时间
  rclcpp::Time last_initialpose_time_{0, 0, RCL_ROS_TIME}; ///< 最近一次初始位姿时间
  bool initialpose_reacquire_active_{false}; ///< 初始位姿后是否处于NDT重新捕获窗口
  int consecutive_rejected_frames_{0}; ///< 连续被拒绝的扫描匹配帧数

  // Fast-LIO delta guess (Plan B): 用 camera_init→body TF 的位姿差推进 init_guess
  // 注意：启用后 corrent_pose_with_cov_stamped_ptr_ 不再是"当前可信定位"，而是内部 rolling guess
  bool use_fastlio_delta_guess_{false};
  std::string fastlio_camera_frame_{"camera_init"};
  std::string fastlio_body_frame_{"body"};
  double tf_max_stamp_mismatch_sec_{0.2};
  double fastlio_max_delta_translation_{0.20};
  double fastlio_max_delta_yaw_{0.25};
  double fastlio_max_dead_reckon_sec_{2.0};
  bool has_prev_body_pose_{false};
  double prev_body_x_{0.0}, prev_body_y_{0.0}, prev_body_z_{0.0};
  double prev_body_qx_{0.0}, prev_body_qy_{0.0}, prev_body_qz_{0.0}, prev_body_qw_{0.0};
  rclcpp::Time prev_cloud_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_accept_time_{0, 0, RCL_ROS_TIME};
  // Per-frame debug fields (set in cloudReceived, read by publishLocalizationStatus)
  bool fastlio_delta_applied_{false};
  std::string fastlio_delta_reject_reason_;
  double fastlio_delta_translation_debug_{0.0};
  double fastlio_delta_yaw_debug_{0.0};
  double fastlio_dead_reckon_age_debug_{0.0};


  // ========== ROS参数 ==========
  std::string global_frame_id_;   ///< 全局坐标系ID（如"map"）
  std::string odom_frame_id_;     ///< 里程计坐标系ID
  std::string base_frame_id_;     ///< 机器人基坐标系ID
  std::string localization_status_topic_; ///< 定位质量状态话题
  std::string registration_method_;  ///< 配准方法名称（NDT/GICP等）
  double scan_max_range_;         ///< 点云最大有效距离（米）
  double scan_min_range_;         ///< 点云最小有效距离（米）
  int min_scan_points_{50};       ///< NDT匹配前要求的最少有效点数，避免空点云发布坏TF
  double scan_period_;            ///< 雷达扫描周期（秒），用于IMU去畸变
  double score_threshold_;        ///< 配准得分阈值，超过认为不可靠
  bool reject_pose_jump_{true};    ///< 是否拒绝单帧位姿大跳变
  double max_pose_jump_translation_{0.8}; ///< 单帧最大允许平移修正
  double max_pose_jump_yaw_{0.45}; ///< 单帧最大允许航向修正
  double initialpose_relax_duration_sec_{0.0}; ///< 初始位姿后的宽松捕获窗口
  double initialpose_max_pose_jump_translation_{2.0}; ///< 捕获窗口内允许的最大平移修正
  double initialpose_max_pose_jump_yaw_{1.0}; ///< 捕获窗口内允许的最大航向修正
  bool pose_jump_reacquire_enabled_{false}; ///< 是否对高置信单帧跳变做连续确认后放行
  double pose_jump_reacquire_max_translation_{1.5}; ///< 连续确认模式允许的最大平移修正
  double pose_jump_reacquire_max_yaw_{0.12}; ///< 连续确认模式允许的最大航向修正
  double pose_jump_reacquire_max_fitness_{0.02}; ///< 连续确认模式要求的最大fitness
  int pose_jump_reacquire_required_frames_{2}; ///< 连续确认模式要求的一致帧数
  double pose_jump_reacquire_xy_tolerance_{0.35}; ///< 连续候选XY一致性阈值
  double pose_jump_reacquire_yaw_tolerance_{0.12}; ///< 连续候选yaw一致性阈值
  bool pose_jump_candidate_active_{false}; ///< 是否已有待确认的pose jump候选
  int pose_jump_candidate_count_{0}; ///< 当前pose jump候选连续确认帧数
  double pose_jump_candidate_x_{0.0}; ///< 待确认候选X
  double pose_jump_candidate_y_{0.0}; ///< 待确认候选Y
  double pose_jump_candidate_yaw_{0.0}; ///< 待确认候选yaw
  double last_mean_corr_dist_{-1.0}; ///< 最近一次NDT匹配的平均关联距离(用于退化诊断)
  int last_corr_count_{0}; ///< 最近一次NDT匹配的关联点对数
  double ndt_resolution_;         ///< NDT网格分辨率（米）
  double ndt_step_size_;          ///< NDT牛顿迭代步长
  double transform_epsilon_;      ///< 变换收敛阈值
  double voxel_leaf_size_;        ///< 体素滤波叶子大小（米）
  double ndt_outlier_ratio_{0.55};     ///< NDT离群点比率: 越高越"宽容"但曲面越平坦，标准PCL=0.35
  double ndt_max_corr_dist_{0.0};      ///< NDT最大关联距离(m): 超过此距的点-格配对跳过，0=禁用
  bool use_pcd_map_{false};       ///< 是否使用PCD地图文件
  std::string map_path_;          ///< PCD地图文件路径
  bool set_initial_pose_{false};  ///< 是否启动时设置初始位姿
  double initial_pose_x_;         ///< 初始位姿X
  double initial_pose_y_;         ///< 初始位姿Y
  double initial_pose_z_;         ///< 初始位姿Z
  double initial_pose_qx_;        ///< 初始四元数X
  double initial_pose_qy_;        ///< 初始四元数Y
  double initial_pose_qz_;        ///< 初始四元数Z
  double initial_pose_qw_;        ///< 初始四元数W

  bool use_odom_{false};          ///< 是否使用里程计
  double last_odom_received_time_; ///< 上次里程计时间戳
  bool use_imu_{false};           ///< 是否使用IMU
  bool ndt_rotation_prior_enabled_{false};   ///< 是否启用 NDT 旋转先验(roll/pitch约束)
  double ndt_rotation_prior_weight_{0.0};    ///< 旋转先验权重: 0=不约束, 10=强约束
  bool ndt_rotation_prior_roll_pitch_only_{true}; ///< 旋转先验仅约束roll/pitch(yaw留给NDT)
  bool enable_debug_{false};      ///< 是否启用调试输出
  bool force_2d_pose_{false};     ///< 是否将发布给导航的位姿约束到2D平面
  bool force_2d_fixed_z_{true};   ///< 2D约束时是否固定Z坐标
  double force_2d_z_{0.0};        ///< 2D约束时使用的固定Z坐标
  bool republish_last_good_tf_on_failure_{true}; ///< 定位短暂失败时是否重发最后可信TF
  double max_last_good_tf_age_sec_{3.0}; ///< 最后可信TF可重发的最大时长
  int ndt_num_threads_;           ///< OMP线程数
  int ndt_max_iterations_;        ///< 配准最大迭代次数

  // IMU去畸变模块
  LidarUndistortion lidar_undistortion_;  ///< 点云去畸变对象
};
