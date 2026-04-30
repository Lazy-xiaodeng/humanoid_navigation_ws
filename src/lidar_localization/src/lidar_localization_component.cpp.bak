/**
 * @file lidar_localization_component.cpp
 * @brief 基于PCL的激光雷达定位ROS2组件实现文件
 * 
 * 本文件实现了使用PCL库进行激光雷达配准定位的核心功能
 * 支持NDT、GICP等多种配准算法，支持IMU去畸变、里程计预测等功能
 */

#include <lidar_localization/lidar_localization_component.hpp>
#include <pcl/common/transforms.h> // ★ 新增：用于点云坐标系转换的头文件

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
  
  // ========== 配准方法参数 ==========
  declare_parameter("registration_method", "NDT");  // 配准方法：NDT, NDT_OMP, GICP, GICP_OMP
  declare_parameter("score_threshold", 2.0);        // 配准得分阈值（fitness score），超过此值认为配准不可靠
  declare_parameter("ndt_resolution", 1.0);         // NDT算法的体素网格分辨率（米），控制NDT网格大小
  declare_parameter("ndt_step_size", 0.1);          // NDT算法的牛顿迭代步长，越大收敛越快但可能不稳定
  declare_parameter("ndt_max_iterations", 35);      // 配准算法最大迭代次数
  declare_parameter("ndt_num_threads", 4);          // OMP版本的线程数，>0时使用指定线程数，<=0时使用最大可用线程数
  declare_parameter("transform_epsilon", 0.01);     // 变换收敛阈值，两次迭代间变换小于此值认为收敛
  
  // ========== 点云滤波参数 ==========
  declare_parameter("voxel_leaf_size", 0.2);        // 体素滤波叶子大小（米），用于降采样，越大点越少
  
  // ========== 点云范围参数 ==========
  declare_parameter("scan_max_range", 100.0);       // 点云最大有效距离（米），超出此范围的点将被过滤
  declare_parameter("scan_min_range", 1.0);         // 点云最小有效距离（米），近距离盲区过滤
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
  odom_sub_.reset();
  cloud_sub_.reset();
  imu_sub_.reset();

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
  
  // 获取配准算法参数
  get_parameter("registration_method", registration_method_);  // 配准方法名称
  get_parameter("score_threshold", score_threshold_);          // 配准得分阈值
  get_parameter("ndt_resolution", ndt_resolution_);     // NDT网格分辨率
  get_parameter("ndt_step_size", ndt_step_size_);       // NDT牛顿迭代步长
  get_parameter("ndt_num_threads", ndt_num_threads_);   // OMP线程数
  get_parameter("ndt_max_iterations", ndt_max_iterations_);  // 最大迭代次数
  get_parameter("transform_epsilon", transform_epsilon_);    // 变换收敛阈值
  
  // 获取点云处理参数
  get_parameter("voxel_leaf_size", voxel_leaf_size_);   // 体素滤波叶子大小
  get_parameter("scan_max_range", scan_max_range_);     // 点云最大距离
  get_parameter("scan_min_range", scan_min_range_);     // 点云最小距离
  get_parameter("scan_period", scan_period_);           // 雷达扫描周期
  
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

  // 打印参数值到日志，方便调试
  RCLCPP_INFO(get_logger(),"global_frame_id: %s", global_frame_id_.c_str());
  RCLCPP_INFO(get_logger(),"odom_frame_id: %s", odom_frame_id_.c_str());
  RCLCPP_INFO(get_logger(),"base_frame_id: %s", base_frame_id_.c_str());
  RCLCPP_INFO(get_logger(),"registration_method: %s", registration_method_.c_str());
  RCLCPP_INFO(get_logger(),"ndt_resolution: %lf", ndt_resolution_);
  RCLCPP_INFO(get_logger(),"ndt_step_size: %lf", ndt_step_size_);
  RCLCPP_INFO(get_logger(),"ndt_num_threads: %d", ndt_num_threads_);
  RCLCPP_INFO(get_logger(),"transform_epsilon: %lf", transform_epsilon_);
  RCLCPP_INFO(get_logger(),"voxel_leaf_size: %lf", voxel_leaf_size_);
  RCLCPP_INFO(get_logger(),"scan_max_range: %lf", scan_max_range_);
  RCLCPP_INFO(get_logger(),"scan_min_range: %lf", scan_min_range_);
  RCLCPP_INFO(get_logger(),"scan_period: %lf", scan_period_);
  RCLCPP_INFO(get_logger(),"use_pcd_map: %d", use_pcd_map_);
  RCLCPP_INFO(get_logger(),"map_path: %s", map_path_.c_str());
  RCLCPP_INFO(get_logger(),"set_initial_pose: %d", set_initial_pose_);
  RCLCPP_INFO(get_logger(),"use_odom: %d", use_odom_);
  RCLCPP_INFO(get_logger(),"use_imu: %d", use_imu_);
  RCLCPP_INFO(get_logger(),"enable_debug: %d", enable_debug_);
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
    registration_ = ndt;
  }
  else if (registration_method_ == "NDT_OMP") {
    // 使用多线程NDT算法，大幅提升速度
    pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt_omp(
      new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt_omp->setStepSize(ndt_step_size_);
    ndt_omp->setResolution(ndt_resolution_);
    ndt_omp->setTransformationEpsilon(transform_epsilon_);
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
  
  // 检查坐标系是否匹配
  if (msg->header.frame_id != global_frame_id_) {
    RCLCPP_WARN(this->get_logger(), "initialpose_frame_id does not match global_frame_id");
    return;
  }
  
  initialpose_recieved_ = true;  // 标记已接收初始位姿
  corrent_pose_with_cov_stamped_ptr_ = msg;  // 保存当前位姿
  pose_pub_->publish(*corrent_pose_with_cov_stamped_ptr_);  // 发布初始位姿

  cloudReceived(last_scan_ptr_);  // 使用最后一帧点云进行首次定位
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
  
  // 设置配准源点云
  registration_->setInputSource(tmp_ptr);

  // ========== 配准定位 ==========
  // 将当前位姿转换为矩阵，作为配准的初始猜测
  Eigen::Affine3d affine;
  tf2::fromMsg(corrent_pose_with_cov_stamped_ptr_->pose.pose, affine);
  Eigen::Matrix4f init_guess = affine.matrix().cast<float>();

  // 执行配准算法
  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  rclcpp::Clock system_clock;
  rclcpp::Time time_align_start = system_clock.now();  // 记录开始时间
  registration_->align(*output_cloud, init_guess);
  rclcpp::Time time_align_end = system_clock.now();  // 记录结束时间

  // 检查配准结果
  bool has_converged = registration_->hasConverged();  // 是否收敛
  double fitness_score = registration_->getFitnessScore();  // 配准得分
  if (!has_converged) {
    RCLCPP_WARN(get_logger(), "The registration didn't converge.");
    return;  // 配准未收敛，放弃此次结果
  }
  if (fitness_score > score_threshold_) {
    RCLCPP_WARN(get_logger(), "The fitness score is over %lf, skip this result.", score_threshold_);
    // ★ 匹配质量差时不发布TF，避免位姿跳变
    return;  // 放弃此次结果，不发布不可靠的TF变换
  }

  // 获取最终的变换矩阵
  Eigen::Matrix4f final_transformation = registration_->getFinalTransformation();

  // ★★★ 核心修改 4：已经删除了导致崩溃的坐标系矩阵连乘补正 ★★★
  // 因为现在输入的地图和点云都已经完全处在规范的 ROS 坐标系下，
  // 获取到的 final_transformation 本身就是准确纯粹的 map -> odom，不再需要多余补偿。

  // 从变换矩阵中提取旋转并转换为四元数
  Eigen::Matrix3d rot_mat = final_transformation.block<3, 3>(0, 0).cast<double>();  // 3x3旋转矩阵
  Eigen::Quaterniond quat_eig(rot_mat);  // 转换为四元数
  geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig);  // 转换为ROS消息

  // 更新位姿消息的时间戳和坐标系
  corrent_pose_with_cov_stamped_ptr_->header.stamp = msg->header.stamp;
  corrent_pose_with_cov_stamped_ptr_->header.frame_id = global_frame_id_;
  
  // 从变换矩阵中提取平移并更新位姿
  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x = static_cast<double>(final_transformation(0, 3));
  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.y = static_cast<double>(final_transformation(1, 3));
  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.z = static_cast<double>(final_transformation(2, 3));
  corrent_pose_with_cov_stamped_ptr_->pose.pose.orientation = quat_msg;
  
  // 发布定位结果
  pose_pub_->publish(*corrent_pose_with_cov_stamped_ptr_);

  // ========== 发布TF变换 ==========
  // 发布map到base_link的变换，供其他节点使用
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = this->now();
  transform_stamped.header.frame_id = global_frame_id_;      // 父坐标系
  transform_stamped.child_frame_id = base_frame_id_;         // 子坐标系（机器人基座）
  transform_stamped.transform.translation.x = static_cast<double>(final_transformation(0, 3));
  transform_stamped.transform.translation.y = static_cast<double>(final_transformation(1, 3));
  transform_stamped.transform.translation.z = static_cast<double>(final_transformation(2, 3));
  transform_stamped.transform.rotation = quat_msg;
  broadcaster_.sendTransform(transform_stamped);  // 广播TF变换

  // ========== 更新并发布路径 ==========
  // 将当前位姿添加到路径中
  geometry_msgs::msg::PoseStamped::SharedPtr pose_stamped_ptr(new geometry_msgs::msg::PoseStamped);
  pose_stamped_ptr->header.stamp = msg->header.stamp;
  pose_stamped_ptr->header.frame_id = global_frame_id_;
  pose_stamped_ptr->pose = corrent_pose_with_cov_stamped_ptr_->pose.pose;
  path_ptr_->poses.push_back(*pose_stamped_ptr);  // 添加到路径
  path_pub_->publish(*path_ptr_);  // 发布路径

  last_scan_ptr_ = msg;  // 保存当前扫描，供下次使用

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
