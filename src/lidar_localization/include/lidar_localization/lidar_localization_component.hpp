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

  bool map_recieved_{false};         ///< 标记是否已接收地图
  bool initialpose_recieved_{false}; ///< 标记是否已接收初始位姿

  // ========== ROS参数 ==========
  std::string global_frame_id_;   ///< 全局坐标系ID（如"map"）
  std::string odom_frame_id_;     ///< 里程计坐标系ID
  std::string base_frame_id_;     ///< 机器人基坐标系ID
  std::string registration_method_;  ///< 配准方法名称（NDT/GICP等）
  double scan_max_range_;         ///< 点云最大有效距离（米）
  double scan_min_range_;         ///< 点云最小有效距离（米）
  double scan_period_;            ///< 雷达扫描周期（秒），用于IMU去畸变
  double score_threshold_;        ///< 配准得分阈值，超过认为不可靠
  double ndt_resolution_;         ///< NDT网格分辨率（米）
  double ndt_step_size_;          ///< NDT牛顿迭代步长
  double transform_epsilon_;      ///< 变换收敛阈值
  double voxel_leaf_size_;        ///< 体素滤波叶子大小（米）
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
  bool enable_debug_{false};      ///< 是否启用调试输出

  int ndt_num_threads_;           ///< OMP线程数
  int ndt_max_iterations_;        ///< 配准最大迭代次数

  // IMU去畸变模块
  LidarUndistortion lidar_undistortion_;  ///< 点云去畸变对象
};
