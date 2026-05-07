/**
 * @file global_localization_node.cpp
 * @brief 人形机器人全局定位节点
 *
 * ============================================================================
 * 功能概述
 * ============================================================================
 * 本节点实现基于多分辨率 NDT (Normal Distributions Transform) 的全局定位功能，
 * 作为现有 lidar_localization 单分辨率 NDT 定位的升级替代方案。
 *
 * 核心特性：
 *   1. 全局初始化：通过多分辨率 NDT 网格搜索，在整张地图范围内自动找到
 *      机器人的初始位姿，无需人工指定。可在任意位置启动。
 *   2. 持续跟踪：使用细分辨率 NDT + 指数平滑滤波，稳定输出定位结果。
 *   3. 自动重定位：跟踪丢失后，在最后已知位姿附近自动搜索恢复。
 *   4. TF 发布：发布 map→odom 变换，与现有 TF 树完全兼容。
 *
 * ============================================================================
 * 算法流程
 * ============================================================================
 *
 * 启动后（无初始位姿时）：
 *   ┌─────────────────────────────────────────────────────────┐
 *   │ 第一阶段：全局搜索 (Global Search)                       │
 *   │  1. 在 ROI 范围内以 1m 间距生成候选位姿 (x, y, yaw)     │
 *   │  2. 每个候选位姿运行粗 NDT (3m 分辨率, 10次迭代)        │
 *   │  3. 按 fitness_score 排序，取 top-5 候选                │
 *   │  4. 对 top-5 用中 NDT (1.5m 分辨率, 20次迭代) 精化     │
 *   │  5. 取最佳候选作为初始位姿                              │
 *   └─────────────────────────────────────────────────────────┘
 *                            ↓
 *   ┌─────────────────────────────────────────────────────────┐
 *   │ 第二阶段：持续跟踪 (Tracking)                           │
 *   │  1. 每帧用上一帧位姿作为 NDT 初始猜测                   │
 *   │  2. 细 NDT (1m 分辨率, 35次迭代) 精确匹配               │
 *   │  3. 指数平滑滤波，抑制抖动                              │
 *   │  4. 发布 map→odom TF 和 /pcl_pose 话题                 │
 *   └─────────────────────────────────────────────────────────┘
 *                            ↓ (连续N帧匹配失败)
 *   ┌─────────────────────────────────────────────────────────┐
 *   │ 第三阶段：局部重定位 (Local Relocalization)              │
 *   │  1. 在最后已知位姿周围 3m 范围内搜索                    │
 *   │  2. 找到后恢复跟踪，找不到继续重试                      │
 *   └─────────────────────────────────────────────────────────┘
 *
 * 收到 /initialpose (RViz "2D Pose Estimate") 后：
 *   → 跳过全局搜索，直接以指定位姿为初始猜测
 *   → 粗 NDT 验证 → 中 NDT 精化 → 切换到跟踪模式
 *
 * ============================================================================
 * 坐标系处理
 * ============================================================================
 *
 * TF 树结构：
 *   map ──(本节点发布)──→ odom ──(静态)──→ camera_init ──(Fast-LIO)──→ body
 *     │                    │
 *     └──(静态 Z=-1.215)──→ map_ground    └──(静态)──→ odom_ground
 *
 * 输入点云坐标系说明：
 *   Fast-LIO 输出的 /fast_lio/cloud_registered 处于 camera_init 坐标系，
 *   该坐标系是非标准的（X左, Y下, Z后 或 X右, Y下, Z后，取决于安装方向）。
 *
 * 本节点内部处理：
 *   和现有 lidar_localization 一样，对地图和扫描点云都施加相同的旋转
 *   q_cam_to_ros(0.5, -0.5, -0.5, 0.5)，将其转换到 ROS 标准坐标系
 *   (X前, Y左, Z上) 后再进行 NDT 匹配。
 *
 *   这样 NDT 匹配结果就是准确的 map→odom 变换，不需要额外的矩阵补偿。
 *
 * ============================================================================
 * 与 hdl_localization 的对比
 * ============================================================================
 *
 * hdl_localization (koide3, 名古屋大学/AIST):
 *   - 算法：UKF (无迹卡尔曼滤波) + NDT 扫描匹配
 *   - 全局定位：hdl_global_localization 独立模块，分支定界搜索
 *   - 优点：经过大量验证，论文发表，UKF 预测+校正框架成熟
 *   - 缺点：ROS2 移植版仅支持到 Humble，无 Jazzy 版本；
 *           依赖 ndt_omp + fast_gicp 两个外部库，需要源码编译；
 *           整体代码量大，定制和调试门槛高
 *
 * 本节点 (humanoid_global_localization):
 *   - 算法：多分辨率 NDT 网格搜索 + 指数平滑
 *   - 全局定位：内建网格搜索，不需要额外模块
 *   - 优点：纯 PCL 实现，无额外依赖；Jazzy 原生支持；
 *           代码精简 (~500行)，容易理解和调试；
 *           坐标系处理与现有 lidar_localization 完全一致
 *   - 缺点：网格搜索在极大场景 (>100m×100m) 下比分支定界慢；
 *           指数平滑不如 UKF 理论上完备
 *
 * 预期效果对比：
 *   - 在中小场景（<30m×30m，如室内大厅/走廊）中，两种方案效果相当
 *   - 在大场景中，hdl_localization 的分支定界搜索更快
 *   - 在稳定性方面，UKF 理论上优于指数平滑，但实际差异在小场景不明显
 */

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// PCL 点云处理库
#include <pcl/point_cloud.h>       // 点云数据结构
#include <pcl/point_types.h>       // 点类型定义 (PointXYZI: x,y,z + intensity)
#include <pcl/filters/voxel_grid.h> // 体素网格降采样滤波器
#include <pcl/io/pcd_io.h>          // PCD 文件读写
#include <pcl/registration/ndt.h>   // NDT 正态分布变换配准
#include <pcl/common/transforms.h>  // 点云坐标变换

// ROS2 核心库
#include <rclcpp/rclcpp.hpp>                     // ROS2 节点基础
#include <rclcpp_lifecycle/lifecycle_node.hpp>   // 生命周期节点 (支持 configure/activate)

// TF2 坐标变换库
#include <tf2/LinearMath/Quaternion.h>   // 四元数
#include <tf2/LinearMath/Matrix3x3.h>    // 3x3 旋转矩阵
#include <tf2_ros/transform_broadcaster.h> // TF 广播器
#include <tf2_eigen/tf2_eigen.hpp>         // Eigen ↔ TF2 类型转换

// ROS2 消息类型
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp> // 带协方差的位姿
#include <geometry_msgs/msg/pose_stamped.hpp>                 // 带时间戳的位姿
#include <geometry_msgs/msg/transform_stamped.hpp>             // TF 变换
#include <nav_msgs/msg/path.hpp>                               // 路径 (位姿序列)
#include <sensor_msgs/msg/point_cloud2.hpp>                    // 点云消息

// PCL ↔ ROS 消息转换
#include <pcl_conversions/pcl_conversions.h>

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// ============================================================================
// 常量定义
// ============================================================================

// 将点云从 LiDAR 坐标系 (camera_init, Z向后/非标准)
// 旋转到 ROS 标准坐标系 (X前, Y左, Z上) 的四元数
// Eigen 构造函数顺序: (w, x, y, z)
// 等效于绕轴 (-1,-1,1)/√3 旋转 120 度
static const Eigen::Quaternionf Q_CAM_TO_ROS(0.5, -0.5, -0.5, 0.5);
static const Eigen::Vector3f    T_CAM_TO_ROS(0.0, 0.0, 0.0);  // 无平移

// ============================================================================
// 候选位姿结构体 (用于网格搜索)
// ============================================================================
struct Candidate {
  double x, y;      // 候选位置 (在 ROS 标准坐标系下, 单位: 米)
  double yaw;       // 候选朝向 (绕 Z 轴的旋转角, 单位: 弧度)
  double score;     // NDT 匹配得分 (fitness_score, 越低越好)
};

// ============================================================================
// GlobalLocalizationNode 类
// ============================================================================
class GlobalLocalizationNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * @brief 构造函数
   *
   * 在此声明所有 ROS 参数及其默认值。
   * 实际参数值在 on_configure() 阶段从参数文件 (global_localization.yaml) 读取。
   *
   * @param opts ROS 节点选项
   */
  explicit GlobalLocalizationNode(const rclcpp::NodeOptions & opts)
    : rclcpp_lifecycle::LifecycleNode("global_localization", opts),
      tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(*this))
  {
    // ---- 坐标系参数 ----
    declare_parameter("global_frame_id", "map");  // 全局地图坐标系 (TF 父节点)
    declare_parameter("odom_frame_id",  "odom");  // 里程计坐标系 (TF 子节点)

    // ---- 地图参数 ----
    declare_parameter("map_path", "");       // PCD 点云地图文件路径
    declare_parameter("use_pcd_map", true);  // 是否在启动时加载 PCD 地图

    // ---- 全局搜索 ROI (Region Of Interest) 参数 ----
    // 在 ROS 标准坐标系 (X前, Y左, Z上) 中定义搜索范围
    // 仅搜索 XY 平面，Z 固定为 0，同时采样 4 个朝向 (0°, 90°, 180°, 270°)
    declare_parameter("search_x_min",   -15.0);  // X 方向最小搜索边界 (米)
    declare_parameter("search_x_max",    15.0);  // X 方向最大搜索边界 (米)
    declare_parameter("search_y_min",   -15.0);  // Y 方向最小搜索边界 (米)
    declare_parameter("search_y_max",    15.0);  // Y 方向最大搜索边界 (米)
    declare_parameter("search_step_xy",   1.0);  // XY 网格步长 (米), 越小越精细但越慢
    declare_parameter("search_top_k",      5);   // 粗匹配后保留多少个候选进入精匹配

    // ---- 多分辨率 NDT 参数 ----
    // 粗 NDT: 分辨率大 → 收敛范围大、速度快，但精度低
    declare_parameter("coarse_ndt_resolution",  3.0);  // 粗 NDT 网格分辨率 (米)
    declare_parameter("coarse_ndt_iterations",   10);  // 粗 NDT 最大迭代次数
    // 中 NDT: 分辨率适中 → 在速度和精度之间平衡
    declare_parameter("medium_ndt_resolution",  1.5);  // 中 NDT 网格分辨率 (米)
    declare_parameter("medium_ndt_iterations",   20);  // 中 NDT 最大迭代次数
    // 细 NDT: 分辨率小 → 精度高，用于持续跟踪
    declare_parameter("fine_ndt_resolution",    1.0);  // 细 NDT 网格分辨率 (米)
    declare_parameter("fine_ndt_iterations",     35);  // 细 NDT 最大迭代次数

    // ---- 匹配质量阈值 ----
    // fitness_score 含义：匹配点云中每个点到其最近 NDT 网格中心的平均距离
    // 越低表示匹配越好，理想值为 0
    declare_parameter("score_threshold",                  2.0);  // 跟踪模式：超过此值丢弃该帧
    declare_parameter("relocalize_score_threshold",       5.0);  // 全局/局部搜索：最佳候选需低于此值
    declare_parameter("consecutive_failures_for_reloc",    10);  // 连续失败多少帧后触发局部重定位

    // ---- 位姿平滑参数 ----
    // α=0 表示无平滑 (完全信任 NDT 结果)
    // α=1 表示不更新 (完全忽略 NDT 结果)
    // α=0.3 表示新结果占 30%，旧结果占 70%
    declare_parameter("smoothing_alpha", 0.3);

    // ---- 点云滤波参数 ----
    declare_parameter("voxel_leaf_size",  0.2);   // 体素降采样立方体边长 (米)
    declare_parameter("scan_min_range",   1.0);   // 近距离盲区过滤 (米), 过滤掉机器人自身点
    declare_parameter("scan_max_range", 100.0);   // 最大有效距离 (米), 过滤远处不可靠噪点

    // ---- 局部重定位参数 ----
    declare_parameter("reloc_search_radius", 3.0);  // 在最后已知位姿周围的搜索半径 (米)

    // ---- 调试参数 ----
    declare_parameter("enable_debug", false);  // 是否在终端打印详细调试信息

    // ---- 初始位姿参数 (可选) ----
    declare_parameter("set_initial_pose", false);  // 是否使用预设初始位姿 (跳过全局搜索)
    declare_parameter("initial_pose_x", 0.0);
    declare_parameter("initial_pose_y", 0.0);
    declare_parameter("initial_pose_z", 0.0);
    declare_parameter("initial_pose_qx", 0.0);
    declare_parameter("initial_pose_qy", 0.0);
    declare_parameter("initial_pose_qz", 0.0);
    declare_parameter("initial_pose_qw", 1.0);
  }

  // ========================================================================
  // 生命周期回调函数
  //
  // ROS2 Lifecycle Node 状态机：
  //   unconfigured → configure → inactive → activate → active
  //                                            ↑           ↓
  //                                            └─ deactivate ←─
  //
  // configure: 加载参数、分配资源 (但不启动数据处理)
  // activate:  启动发布者和订阅者，开始实际工作
  // deactivate: 停止数据处理，但保留配置
  // cleanup:   释放所有资源，回到 unconfigured 状态
  // ========================================================================

  /**
   * @brief 配置阶段回调
   *
   * 从参数服务器读取所有配置参数值。
   * 在此阶段：参数已加载，但节点尚未开始处理数据。
   *
   * @return SUCCESS 表示配置成功
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "========== 配置阶段开始 ==========");

    // 读取坐标系参数
    get_parameter("global_frame_id", global_frame_id_);
    get_parameter("odom_frame_id",   odom_frame_id_);
    // 读取地图参数
    get_parameter("map_path",        map_path_);
    get_parameter("use_pcd_map",     use_pcd_map_);

    // 读取全局搜索参数
    get_parameter("search_x_min",    search_x_min_);
    get_parameter("search_x_max",    search_x_max_);
    get_parameter("search_y_min",    search_y_min_);
    get_parameter("search_y_max",    search_y_max_);
    get_parameter("search_step_xy",  search_step_xy_);
    get_parameter("search_top_k",    search_top_k_);

    // 读取 NDT 多分辨率参数
    get_parameter("coarse_ndt_resolution", coarse_ndt_res_);
    get_parameter("coarse_ndt_iterations", coarse_ndt_iter_);
    get_parameter("medium_ndt_resolution", medium_ndt_res_);
    get_parameter("medium_ndt_iterations", medium_ndt_iter_);
    get_parameter("fine_ndt_resolution",   fine_ndt_res_);
    get_parameter("fine_ndt_iterations",   fine_ndt_iter_);

    // 读取匹配阈值参数
    get_parameter("score_threshold",                score_threshold_);
    get_parameter("relocalize_score_threshold",     reloc_score_threshold_);
    get_parameter("consecutive_failures_for_reloc", max_fail_for_reloc_);

    // 读取平滑和滤波参数
    get_parameter("smoothing_alpha", smoothing_alpha_);
    get_parameter("voxel_leaf_size", voxel_leaf_size_);
    get_parameter("scan_min_range",  scan_min_range_);
    get_parameter("scan_max_range",  scan_max_range_);

    // 读取重定位参数
    get_parameter("reloc_search_radius", reloc_search_radius_);

    // 读取调试开关
    get_parameter("enable_debug", enable_debug_);

    // 读取初始位姿参数
    get_parameter("set_initial_pose", set_initial_pose_);
    get_parameter("initial_pose_x",   initial_pose_x_);
    get_parameter("initial_pose_y",   initial_pose_y_);
    get_parameter("initial_pose_z",   initial_pose_z_);
    get_parameter("initial_pose_qx",  initial_pose_qx_);
    get_parameter("initial_pose_qy",  initial_pose_qy_);
    get_parameter("initial_pose_qz",  initial_pose_qz_);
    get_parameter("initial_pose_qw",  initial_pose_qw_);

    RCLCPP_INFO(get_logger(), "全局坐标系: %s", global_frame_id_.c_str());
    RCLCPP_INFO(get_logger(), "地图路径: %s", map_path_.c_str());
    RCLCPP_INFO(get_logger(), "搜索范围: X[%.1f, %.1f] Y[%.1f, %.1f] 步长=%.1fm",
                search_x_min_, search_x_max_, search_y_min_, search_y_max_, search_step_xy_);
    RCLCPP_INFO(get_logger(), "========== 配置阶段完成 ==========");

    return CallbackReturn::SUCCESS;
  }

  /**
   * @brief 激活阶段回调
   *
   * 加载 PCD 地图，创建发布者和订阅者，进入可工作状态。
   * 如果有预设初始位姿，立即设置。
   *
   * @return SUCCESS 表示激活成功，FAILURE 表示地图加载失败
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "========== 激活阶段开始 ==========");

    // ---- 加载 PCD 点云地图 ----
    if (use_pcd_map_ && !map_path_.empty()) {
      map_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
      if (pcl::io::loadPCDFile(map_path_, *map_cloud_) == -1) {
        RCLCPP_ERROR(get_logger(), "无法加载 PCD 地图: %s", map_path_.c_str());
        return CallbackReturn::FAILURE;
      }
      RCLCPP_INFO(get_logger(), "已加载 PCD 地图: %ld 个点", map_cloud_->size());

      // ★ 关键步骤：将地图从非标准 LiDAR 坐标系旋转到 ROS 标准坐标系
      // 这个旋转与 lidar_localization 中的处理完全一致
      pcl::transformPointCloud(*map_cloud_, *map_cloud_, T_CAM_TO_ROS, Q_CAM_TO_ROS);
      RCLCPP_INFO(get_logger(), "地图已转换到 ROS 标准坐标系 (Z轴朝上)");
    }

    // ---- 创建发布者 ----
    // 发布定位结果 (/pcl_pose)，使用 transient_local 确保新订阅者能收到最新一帧
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "pcl_pose", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    // 发布运动轨迹 (/path)
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "path", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    path_msg_.header.frame_id = global_frame_id_;  // 路径在 map 坐标系下

    // ---- 创建订阅者 ----
    // 订阅 Fast-LIO 输出的实时点云
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/fast_lio/cloud_registered", rclcpp::SensorDataQoS(),
      std::bind(&GlobalLocalizationNode::cloudCallback, this, std::placeholders::_1));

    // 订阅 RViz "2D Pose Estimate" 初始位姿话题
    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", rclcpp::SystemDefaultsQoS(),
      std::bind(&GlobalLocalizationNode::initialPoseCallback, this, std::placeholders::_1));

    // ---- 处理预设初始位姿 ----
    // 如果配置文件中 set_initial_pose=true，则跳过全局搜索，直接使用预设位姿
    if (set_initial_pose_) {
      auto pose = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
      pose->header.frame_id = global_frame_id_;
      pose->pose.pose.position.x = initial_pose_x_;
      pose->pose.pose.position.y = initial_pose_y_;
      pose->pose.pose.position.z = initial_pose_z_;
      pose->pose.pose.orientation.x = initial_pose_qx_;
      pose->pose.pose.orientation.y = initial_pose_qy_;
      pose->pose.pose.orientation.z = initial_pose_qz_;
      pose->pose.pose.orientation.w = initial_pose_qw_;
      initialPoseCallback(pose);
      RCLCPP_INFO(get_logger(), "已使用预设初始位姿: (%.2f, %.2f)", initial_pose_x_, initial_pose_y_);
    }

    RCLCPP_INFO(get_logger(), "========== 激活阶段完成 ==========");
    if (!has_initial_pose_) {
      RCLCPP_INFO(get_logger(), "【提示】未设置初始位姿，将在收到第一帧点云后自动运行全局搜索");
      RCLCPP_INFO(get_logger(), "【提示】也可以在 RViz 中使用 '2D Pose Estimate' 工具手动指定初始位姿");
    }

    return CallbackReturn::SUCCESS;
  }

  /**
   * @brief 停用阶段回调
   *
   * 停止所有数据处理，等待搜索线程结束，释放发布者和订阅者。
   *
   * @return SUCCESS
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "停用中...");
    // 停止搜索线程
    if (search_thread_ && search_thread_->joinable()) {
      search_running_ = false;
      search_thread_->join();
    }
    pose_pub_.reset();
    path_pub_.reset();
    cloud_sub_.reset();
    initial_pose_sub_.reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    map_cloud_.reset();  // 释放地图内存
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_error(const rclcpp_lifecycle::State &) override
  {
    return CallbackReturn::SUCCESS;
  }

private:
  // ========================================================================
  // 消息回调函数
  // ========================================================================

  /**
   * @brief 初始位姿回调 (RViz "2D Pose Estimate" 工具触发)
   *
   * 收到初始位姿后，将其设置为当前位姿。
   * 全局搜索将被跳过，下次收到点云时直接以该位姿为初始猜测进入跟踪模式。
   *
   * @param msg 初始位姿消息 (包含位置和朝向)
   */
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    // 安全检查：坐标系必须匹配
    if (msg->header.frame_id != global_frame_id_) {
      RCLCPP_WARN(get_logger(), "初始位姿坐标系不匹配: %s != %s (期望)",
                  msg->header.frame_id.c_str(), global_frame_id_.c_str());
      return;
    }

    std::lock_guard<std::mutex> lock(pose_mutex_);
    current_pose_ = *msg;
    has_initial_pose_ = true;

    RCLCPP_INFO(get_logger(), "已收到初始位姿: 位置(%.2f, %.2f, %.2f) 姿态(%.2f, %.2f, %.2f, %.2f)",
                msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z,
                msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  }

  /**
   * @brief 点云回调 (核心处理函数)
   *
   * 每个点云帧到达时调用此函数。根据当前状态执行不同逻辑：
   *
   * 状态机：
   *   ┌─────────────────┐
   *   │ 无初始位姿?      │──是──→ 启动全局搜索 (异步线程)
   *   └────────┬────────┘
   *            │ 否
   *            ↓
   *   ┌─────────────────┐
   *   │ 跟踪模式         │──成功──→ 更新位姿，发布 TF 和话题
   *   └────────┬────────┘
   *            │ 连续失败超过阈值
   *            ↓
   *   ┌─────────────────┐
   *   │ 局部重定位       │──成功──→ 恢复跟踪
   *   └─────────────────┘
   *
   * @param msg 实时点云消息 (来自 Fast-LIO, /fast_lio/cloud_registered)
   */
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    // 前提条件：地图已加载
    if (!map_cloud_ || map_cloud_->empty()) return;

    // ---- 步骤1：将 ROS 点云消息转换为 PCL 格式 ----
    pcl::PointCloud<pcl::PointXYZI>::Ptr raw(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *raw);

    // ---- 步骤2：旋转到 ROS 标准坐标系 (Z向上, X向前, Y向左) ----
    pcl::transformPointCloud(*raw, *raw, T_CAM_TO_ROS, Q_CAM_TO_ROS);

    // ---- 步骤3：点云滤波 (降采样 + 距离过滤) ----
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered = filterCloud(raw);
    if (filtered->size() < 100) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "滤波后点云点数过少: %ld (可能被遮挡或传感器异常)", filtered->size());
      return;
    }

    // ---- 步骤4：状态判断与处理 ----

    // 情况A：尚未定位 → 启动全局搜索 (异步，不阻塞点云接收)
    if (!has_initial_pose_) {
      last_scan_for_search_ = filtered;
      if (!search_running_) {
        startGlobalSearch(filtered);
      }
      return;
    }

    // 情况B：已定位 → 跟踪模式
    // 使用上一帧位姿作为 NDT 初始猜测 (指数平滑后的位姿)
    Eigen::Affine3d guess;
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      tf2::fromMsg(current_pose_.pose.pose, guess);
    }

    // 执行细 NDT 匹配
    auto result = runNDT(filtered, fine_ndt_res_, fine_ndt_iter_, guess.matrix().cast<float>());

    // 匹配质量检查
    if (!result.converged || result.fitness > score_threshold_) {
      consecutive_failures_++;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "NDT 匹配失败 (第%d次, fitness=%.4f > 阈值%.2f)",
                           consecutive_failures_, result.fitness, score_threshold_);

      // 连续失败超过阈值 → 触发局部重定位
      if (consecutive_failures_ >= max_fail_for_reloc_) {
        RCLCPP_WARN(get_logger(), "跟踪丢失! 连续%d帧匹配失败，启动局部重定位...",
                    consecutive_failures_);
        triggerRelocalization(filtered);
        consecutive_failures_ = 0;
      }
      return;
    }

    // 匹配成功 → 重置失败计数，更新位姿
    consecutive_failures_ = 0;
    updatePose(result.transform, msg->header.stamp);
  }

  // ========================================================================
  // 全局搜索 (Phase 1: 多分辨率 NDT 网格搜索)
  // ========================================================================

  /**
   * @brief 启动全局搜索
   *
   * 在新线程中运行，不阻塞主线程。
   *
   * 算法流程：
   *   1. 在 ROI 范围内以 search_step_xy 为步长生成候选位姿网格
   *   2. 对 4 个朝向 (0°, 90°, 180°, 270°) 分别采样
   *   3. 使用 OpenMP 并行对每个候选运行粗 NDT (分辨率大、迭代少、速度快)
   *   4. 按 fitness_score 排序，取 top-K 个候选
   *   5. 对 top-K 候选运行中 NDT (分辨率中等) 进行精化
   *   6. 取最佳候选作为最终初始位姿
   *   7. 如果最佳候选得分仍高于 relocalize_score_threshold，搜索失败
   *
   * @param scan 用于匹配的当前帧点云 (已滤波)
   */
  void startGlobalSearch(pcl::PointCloud<pcl::PointXYZI>::Ptr scan)
  {
    if (search_running_) return;
    search_running_ = true;

    search_thread_ = std::make_shared<std::thread>([this, scan]() {
      RCLCPP_INFO(get_logger(), "");
      RCLCPP_INFO(get_logger(), "╔══════════════════════════════════════════╗");
      RCLCPP_INFO(get_logger(), "║       全局搜索开始 (Global Search)       ║");
      RCLCPP_INFO(get_logger(), "╚══════════════════════════════════════════╝");

      // ---- 步骤1：生成候选位姿网格 ----
      std::vector<Candidate> candidates;
      for (double x = search_x_min_; x <= search_x_max_; x += search_step_xy_) {
        for (double y = search_y_min_; y <= search_y_max_; y += search_step_xy_) {
          // 4 个朝向覆盖主要方向 (0°, 90°, 180°, -90°)
          for (double yaw : {0.0, M_PI_2, M_PI, -M_PI_2}) {
            candidates.push_back({x, y, yaw, 0.0});
          }
        }
      }

      int num_candidates = candidates.size();
      RCLCPP_INFO(get_logger(), "候选位姿数量: %d (网格: %.1f~%.1f x %.1f~%.1f, 步长=%.1fm, 4朝向)",
                  num_candidates,
                  search_x_min_, search_x_max_, search_y_min_, search_y_max_,
                  search_step_xy_);

      // 估算搜索时间
      double est_time = num_candidates * 0.01;  // ~10ms per coarse NDT
      RCLCPP_INFO(get_logger(), "预估搜索时间: %.0f ~ %.0f 秒 (取决于CPU性能)",
                  est_time * 0.5, est_time * 1.5);

      // ---- 步骤2：粗 NDT 并行搜索 ----
      RCLCPP_INFO(get_logger(), "--- 阶段1: 粗 NDT (分辨率=%.1fm, 迭代=%d) ---",
                  coarse_ndt_res_, coarse_ndt_iter_);
      auto t_start = std::chrono::steady_clock::now();

      int done = 0;
      #pragma omp parallel for schedule(dynamic)
      for (size_t i = 0; i < candidates.size(); i++) {
        if (!search_running_) continue;  // 支持提前终止

        // 构造候选位姿的 4x4 变换矩阵
        Eigen::Affine3f guess = makeAffine(candidates[i].x, candidates[i].y, 0, candidates[i].yaw);
        // 运行粗 NDT
        auto res = runNDT(scan, coarse_ndt_res_, coarse_ndt_iter_, guess.matrix());
        // 记录得分 (未收敛则设为极大值)
        candidates[i].score = res.converged ? res.fitness : 1e9;

        #pragma omp critical
        {
          done++;
          if (done % 20 == 0 || done == num_candidates)
            RCLCPP_INFO(get_logger(), "  粗NDT进度: %d/%d (%.0f%%)", done, num_candidates,
                        100.0 * done / num_candidates);
        }
      }

      auto t_coarse = std::chrono::steady_clock::now();
      double dt_coarse = std::chrono::duration<double>(t_coarse - t_start).count();
      RCLCPP_INFO(get_logger(), "粗 NDT 完成, 耗时 %.1f 秒", dt_coarse);

      // ---- 步骤3：排序，取 top-K ----
      std::sort(candidates.begin(), candidates.end(),
                [](const Candidate &a, const Candidate &b) { return a.score < b.score; });

      int topk = std::min<int>(search_top_k_, candidates.size());
      RCLCPP_INFO(get_logger(), "粗 NDT Top-%d 候选位姿:", topk);
      for (int i = 0; i < topk; i++) {
        RCLCPP_INFO(get_logger(), "  #%d: 位置(%.1f, %.1f) 朝向%.0f° score=%.4f",
                    i + 1, candidates[i].x, candidates[i].y,
                    candidates[i].yaw * 180.0 / M_PI, candidates[i].score);
      }

      // ---- 步骤4：中 NDT 精化 top-K ----
      RCLCPP_INFO(get_logger(), "--- 阶段2: 中 NDT 精化 (分辨率=%.1fm, 迭代=%d) ---",
                  medium_ndt_res_, medium_ndt_iter_);
      double best_score = 1e9;
      Eigen::Matrix4f best_transform = Eigen::Matrix4f::Identity();

      for (int i = 0; i < topk && search_running_; i++) {
        Eigen::Affine3f guess = makeAffine(
          candidates[i].x, candidates[i].y, 0, candidates[i].yaw);
        auto res = runNDT(scan, medium_ndt_res_, medium_ndt_iter_, guess.matrix());

        double s = res.converged ? res.fitness : 1e9;
        RCLCPP_INFO(get_logger(), "  精化 #%d: score=%.4f", i + 1, s);
        if (s < best_score) {
          best_score = s;
          best_transform = res.transform;
        }
      }

      auto t_end = std::chrono::steady_clock::now();
      double dt_total = std::chrono::duration<double>(t_end - t_start).count();
      search_running_ = false;

      // ---- 步骤5：判断搜索是否成功 ----
      if (best_score > reloc_score_threshold_) {
        RCLCPP_WARN(get_logger(), "");
        RCLCPP_WARN(get_logger(), "╔══════════════════════════════════════════╗");
        RCLCPP_WARN(get_logger(), "║     全局搜索失败!                       ║");
        RCLCPP_WARN(get_logger(), "║  最佳得分 %.4f > 阈值 %.2f              ║",
                    best_score, reloc_score_threshold_);
        RCLCPP_WARN(get_logger(), "║  总耗时 %.1f 秒                          ║", dt_total);
        RCLCPP_WARN(get_logger(), "║  可能原因：                              ║");
        RCLCPP_WARN(get_logger(), "║  1. 机器人不在已建图区域内               ║");
        RCLCPP_WARN(get_logger(), "║  2. 环境变化太大 (家具移动等)             ║");
        RCLCPP_WARN(get_logger(), "║  3. 搜索范围不够大                       ║");
        RCLCPP_WARN(get_logger(), "║                                          ║");
        RCLCPP_WARN(get_logger(), "║  解决方法：                              ║");
        RCLCPP_WARN(get_logger(), "║  - 在 RViz 中使用 '2D Pose Estimate'     ║");
        RCLCPP_WARN(get_logger(), "║    手动指定大致初始位姿                  ║");
        RCLCPP_WARN(get_logger(), "║  - 或增大 search_x/y_min/max 参数        ║");
        RCLCPP_WARN(get_logger(), "╚══════════════════════════════════════════╝");
        return;
      }

      // ---- 步骤6：搜索成功，设置初始位姿 ----
      RCLCPP_INFO(get_logger(), "");
      RCLCPP_INFO(get_logger(), "╔══════════════════════════════════════════╗");
      RCLCPP_INFO(get_logger(), "║     全局搜索成功!                       ║");
      RCLCPP_INFO(get_logger(), "║  最佳得分: %.4f                          ║", best_score);
      RCLCPP_INFO(get_logger(), "║  总耗时: %.1f 秒                         ║", dt_total);
      RCLCPP_INFO(get_logger(), "╚══════════════════════════════════════════╝");
      {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        updatePoseFromMatrix(best_transform, now());
        has_initial_pose_ = true;
      }
      RCLCPP_INFO(get_logger(), "已切换到跟踪模式。");
    });
  }

  // ========================================================================
  // 局部重定位 (Phase 3: 跟踪丢失后的恢复)
  // ========================================================================

  /**
   * @brief 触发局部重定位
   *
   * 在最后已知位姿周围 reloc_search_radius 范围内，
   * 以 0.5m 步长、3个朝向偏移 (-0.5, 0, +0.5 rad) 搜索恢复位姿。
   *
   * 与全局搜索的区别：
   *   - 搜索范围小 (默认 3m vs 20m+)
   *   - 使用中 NDT 一步到位 (不需要粗→细两级)
   *   - 速度更快 (通常 < 1 秒)
   *
   * @param scan 当前帧点云 (已滤波)
   */
  void triggerRelocalization(pcl::PointCloud<pcl::PointXYZI>::Ptr scan)
  {
    if (search_running_) return;
    search_running_ = true;

    search_thread_ = std::make_shared<std::thread>([this, scan]() {
      RCLCPP_INFO(get_logger(), "--- 局部重定位开始 ---");

      // 从当前位姿中提取位置和朝向 (用于确定搜索中心)
      double cx, cy, cyaw;
      {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        cx = current_pose_.pose.pose.position.x;
        cy = current_pose_.pose.pose.position.y;
        // 从四元数中提取 yaw 角 (绕 Z 轴的旋转)
        const auto &q = current_pose_.pose.pose.orientation;
        Eigen::Quaterniond eq(q.w, q.x, q.y, q.z);
        Eigen::Matrix3d R = eq.toRotationMatrix();
        cyaw = std::atan2(R(1, 0), R(0, 0));
        RCLCPP_INFO(get_logger(), "最后已知位姿: (%.2f, %.2f, %.1f°)",
                    cx, cy, cyaw * 180.0 / M_PI);
      }

      // 在最后已知位姿周围生成候选
      double step = 0.5;  // 搜索步长
      double r = reloc_search_radius_;
      std::vector<Candidate> candidates;
      for (double dx = -r; dx <= r; dx += step)
        for (double dy = -r; dy <= r; dy += step)
          for (double dyaw : {-0.5, 0.0, 0.5})  // ±30° 朝向偏移
            candidates.push_back({cx + dx, cy + dy, cyaw + dyaw, 0.0});

      // 遍历所有候选，运行中 NDT
      double best_score = 1e9;
      Eigen::Matrix4f best_transform = Eigen::Matrix4f::Identity();

      for (auto &c : candidates) {
        if (!search_running_) break;
        Eigen::Affine3f guess = makeAffine(c.x, c.y, 0, c.yaw);
        auto res = runNDT(scan, medium_ndt_res_, medium_ndt_iter_, guess.matrix());
        c.score = res.converged ? res.fitness : 1e9;
        if (c.score < best_score) {
          best_score = c.score;
          best_transform = res.transform;
        }
      }

      search_running_ = false;

      if (best_score < reloc_score_threshold_) {
        RCLCPP_INFO(get_logger(), "局部重定位成功! score=%.4f", best_score);
        std::lock_guard<std::mutex> lock(pose_mutex_);
        updatePoseFromMatrix(best_transform, now());
      } else {
        RCLCPP_WARN(get_logger(), "局部重定位失败 (best=%.4f > %.2f), 将在下帧重试",
                    best_score, reloc_score_threshold_);
      }
    });
  }

  // ========================================================================
  // NDT 配准
  // ========================================================================

  /**
   * @brief NDT 匹配结果
   */
  struct NDTResult {
    bool converged = false;            // 是否收敛
    double fitness = 1e9;             // fitness_score (越低越好)
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();  // 最终变换矩阵
  };

  /**
   * @brief 执行单次 NDT 配准
   *
   * 使用 PCL 的 NormalDistributionsTransform 实现。
   *
   * NDT 原理简述：
   *   将目标点云 (地图) 划分成固定大小的体素网格，每个网格内点云
   *   拟合一个正态分布 N(μ, Σ)。然后优化源点云 (扫描) 的位姿，
   *   使得源点云中每个点在其所在网格的正态分布中的概率之和最大化。
   *
   * @param scan       源点云 (当前扫描，已滤波)
   * @param resolution NDT 体素网格分辨率 (米)，越大收敛范围越大但精度越低
   * @param max_iter   最大牛顿迭代次数
   * @param init_guess 初始位姿猜测 (4x4 变换矩阵)
   * @return NDTResult 包含收敛状态、得分、最终变换
   */
  NDTResult runNDT(pcl::PointCloud<pcl::PointXYZI>::Ptr scan,
                   double resolution, int max_iter,
                   const Eigen::Matrix4f &init_guess)
  {
    NDTResult result;
    pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;

    // 配置 NDT 参数
    ndt.setResolution(static_cast<float>(resolution));  // 体素网格大小
    ndt.setMaximumIterations(max_iter);                 // 最大迭代次数
    ndt.setTransformationEpsilon(0.01);                  // 收敛阈值: 两次迭代位姿变化小于此值则停止
    ndt.setStepSize(0.1);                                // 牛顿迭代步长

    ndt.setInputTarget(map_cloud_);   // 目标点云 = 地图
    ndt.setInputSource(scan);         // 源点云 = 当前扫描

    // 执行配准
    pcl::PointCloud<pcl::PointXYZI>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZI>());
    ndt.align(*aligned, init_guess);

    result.converged = ndt.hasConverged();
    result.fitness   = ndt.getFitnessScore();
    result.transform = ndt.getFinalTransformation();
    return result;
  }

  // ========================================================================
  // 工具函数
  // ========================================================================

  /**
   * @brief 点云滤波
   *
   * 两步滤波：
   *   1. 体素降采样: 将点云划分成 voxel_leaf_size 大小的立方体，
   *      每个立方体内的点用其质心代替，大幅减少点数。
   *   2. 距离过滤: 过滤掉太近 (机器人自身反射) 和太远 (不可靠噪点) 的点。
   *
   * @param cloud 输入点云 (已在 ROS 标准坐标系下)
   * @return 滤波后的点云
   */
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
  {
    // 步骤1：体素降采样 (减少计算量)
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    vg.setInputCloud(cloud);
    vg.filter(*tmp);

    // 步骤2：距离过滤 (去除不可靠点)
    pcl::PointCloud<pcl::PointXYZI>::Ptr out(new pcl::PointCloud<pcl::PointXYZI>());
    for (const auto &p : tmp->points) {
      double r = std::sqrt(p.x * p.x + p.y * p.y);  // 水平距离
      if (r >= scan_min_range_ && r <= scan_max_range_)
        out->push_back(p);
    }
    return out;
  }

  /**
   * @brief 从 (x, y, z, yaw) 构造 4x4 齐次变换矩阵
   *
   * 变换 = 平移 * 绕Z轴旋转
   *
   * @param x, y, z  平移分量 (米)
   * @param yaw      绕 Z 轴旋转角 (弧度)
   * @return 4x4 仿射变换矩阵 (Eigen::Affine3f)
   */
  Eigen::Affine3f makeAffine(double x, double y, double z, double yaw)
  {
    Eigen::AngleAxisf rot(yaw, Eigen::Vector3f::UnitZ());  // 绕 Z 轴旋转
    Eigen::Translation3f trans(x, y, z);                    // 平移
    return trans * rot;  // 先旋转再平移
  }

  /**
   * @brief 从 NDT 变换矩阵更新当前位姿 (无平滑，用于初始化和重定位)
   *
   * @param T      NDT 输出的 4x4 变换矩阵
   * @param stamp  时间戳
   */
  void updatePoseFromMatrix(const Eigen::Matrix4f &T, rclcpp::Time stamp)
  {
    Eigen::Matrix3d rot = T.block<3, 3>(0, 0).cast<double>();
    Eigen::Quaterniond q(rot);
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = global_frame_id_;
    pose.pose.pose.position.x = T(0, 3);
    pose.pose.pose.position.y = T(1, 3);
    pose.pose.pose.position.z = T(2, 3);
    pose.pose.pose.orientation = tf2::toMsg(q);
    current_pose_ = pose;
  }

  /**
   * @brief 指数平滑更新位姿 (用于持续跟踪)
   *
   * 公式：
   *   smoothed = α × raw + (1-α) × previous
   *
   * 其中：
   *   α = smoothing_alpha_ (0~1)
   *   raw = 当前帧 NDT 匹配结果
   *   previous = 上一帧平滑后的位姿
   *
   * 位置使用线性插值，朝向使用 SLERP (球面线性插值)。
   *
   * @param T      NDT 输出的 4x4 变换矩阵
   * @param stamp  点云时间戳
   */
  void updatePose(const Eigen::Matrix4f &T, rclcpp::Time stamp)
  {
    if (!has_initial_pose_) {
      updatePoseFromMatrix(T, stamp);
      has_initial_pose_ = true;
      return;
    }

    // 从 NDT 结果提取原始位姿
    geometry_msgs::msg::PoseWithCovarianceStamped raw;
    raw.header.stamp = stamp;
    raw.header.frame_id = global_frame_id_;

    Eigen::Matrix3d rot = T.block<3, 3>(0, 0).cast<double>();
    Eigen::Quaterniond q_raw(rot);
    raw.pose.pose.position.x = T(0, 3);
    raw.pose.pose.position.y = T(1, 3);
    raw.pose.pose.position.z = T(2, 3);
    raw.pose.pose.orientation = tf2::toMsg(q_raw);

    // 位置指数平滑
    double a = smoothing_alpha_;
    current_pose_.pose.pose.position.x =
      a * raw.pose.pose.position.x + (1.0 - a) * current_pose_.pose.pose.position.x;
    current_pose_.pose.pose.position.y =
      a * raw.pose.pose.position.y + (1.0 - a) * current_pose_.pose.pose.position.y;
    current_pose_.pose.pose.position.z =
      a * raw.pose.pose.position.z + (1.0 - a) * current_pose_.pose.pose.position.z;

    // 朝向 SLERP 球面线性插值 (比线性插值更准确地保持旋转的几何意义)
    Eigen::Quaterniond q_prev;
    tf2::fromMsg(current_pose_.pose.pose.orientation, q_prev);
    Eigen::Quaterniond q_smooth = q_prev.slerp(a, q_raw);
    current_pose_.pose.pose.orientation = tf2::toMsg(q_smooth);
    current_pose_.header.stamp = stamp;

    // 发布定位结果
    pose_pub_->publish(current_pose_);
    // 发布 TF 变换 (核心：map → odom)
    publishTF(current_pose_, stamp);
    // 更新并发布运动轨迹
    updatePath(current_pose_, stamp);
  }

  /**
   * @brief 发布 TF 变换 (map → odom)
   *
   * 这是整个定位系统的核心输出。
   * Nav2 导航栈通过这个 TF 知道机器人在 map 坐标系下的位置。
   *
   * TF 树: map ──(本TF)──→ odom ──→ camera_init ──→ body ──→ base_footprint
   *
   * @param pose   当前位姿 (即 NDT 匹配得到的 map→scan 变换)
   * @param stamp  时间戳
   */
  void publishTF(const geometry_msgs::msg::PoseWithCovarianceStamped &pose,
                 rclcpp::Time stamp)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = global_frame_id_;     // 父坐标系: map
    tf.child_frame_id  = odom_frame_id_;        // 子坐标系: odom
    tf.transform.translation.x = pose.pose.pose.position.x;
    tf.transform.translation.y = pose.pose.pose.position.y;
    tf.transform.translation.z = pose.pose.pose.position.z;
    tf.transform.rotation = pose.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);
  }

  /**
   * @brief 更新并发布运动轨迹
   *
   * 将当前位姿追加到 /path 话题中，用于在 RViz 中显示轨迹。
   *
   * @param pose   当前位姿
   * @param stamp  时间戳
   */
  void updatePath(const geometry_msgs::msg::PoseWithCovarianceStamped &pose,
                  rclcpp::Time stamp)
  {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = stamp;
    ps.header.frame_id = global_frame_id_;
    ps.pose = pose.pose.pose;
    path_msg_.poses.push_back(ps);
    path_msg_.header.stamp = stamp;
    path_pub_->publish(path_msg_);
  }

  // ========================================================================
  // 成员变量
  // ========================================================================

  // ---- TF 相关 ----
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;  // TF 广播器
  std::string global_frame_id_ = "map";   // 全局地图坐标系 (TF 父节点)
  std::string odom_frame_id_   = "odom";  // 里程计坐标系 (TF 子节点)

  // ---- 地图 ----
  std::string map_path_;                                      // PCD 文件路径
  bool use_pcd_map_ = true;                                   // 是否加载 PCD 地图
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_;           // 地图点云 (已转换到 ROS 标准坐标系)

  // ---- ROS 发布者和订阅者 ----
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;  // /pcl_pose
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_pub_;                          // /path
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;                                // 点云订阅
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;         // 初始位姿订阅

  // ---- 状态变量 ----
  geometry_msgs::msg::PoseWithCovarianceStamped current_pose_;  // 当前位姿 (指数平滑后)
  nav_msgs::msg::Path path_msg_;                                 // 运动轨迹
  bool has_initial_pose_ = false;     // 是否已获得初始位姿 (控制状态机切换)
  int consecutive_failures_ = 0;      // 连续 NDT 匹配失败计数
  std::mutex pose_mutex_;             // 位姿读写互斥锁 (主线程和搜索线程共享)

  // ---- 全局搜索参数 ----
  double search_x_min_   = -15.0, search_x_max_   = 15.0;  // X 方向搜索范围
  double search_y_min_   = -15.0, search_y_max_   = 15.0;  // Y 方向搜索范围
  double search_step_xy_ = 1.0;                              // 网格步长
  int    search_top_k_   = 5;                                // 粗匹配保留候选数

  // ---- NDT 多分辨率参数 ----
  double coarse_ndt_res_  = 3.0;   // 粗 NDT 网格分辨率 (米)
  int    coarse_ndt_iter_ = 10;    // 粗 NDT 最大迭代次数
  double medium_ndt_res_  = 1.5;   // 中 NDT 网格分辨率 (米)
  int    medium_ndt_iter_ = 20;    // 中 NDT 最大迭代次数
  double fine_ndt_res_    = 1.0;   // 细 NDT 网格分辨率 (米)
  int    fine_ndt_iter_   = 35;    // 细 NDT 最大迭代次数

  // ---- 匹配阈值 ----
  double score_threshold_       = 2.0;   // 跟踪模式 fitness_score 上限
  double reloc_score_threshold_ = 5.0;   // 全局/局部搜索 fitness_score 上限
  int    max_fail_for_reloc_    = 10;    // 触发局部重定位的连续失败帧数

  // ---- 滤波与平滑 ----
  double smoothing_alpha_  = 0.3;   // 指数平滑系数 (0=无平滑, 1=不更新)
  double voxel_leaf_size_  = 0.2;   // 体素降采样立方体边长 (米)
  double scan_min_range_   = 1.0;   // 最小有效距离 (米)
  double scan_max_range_   = 100.0; // 最大有效距离 (米)
  double reloc_search_radius_ = 3.0; // 局部重定位搜索半径 (米)

  // ---- 调试 ----
  bool enable_debug_ = false;

  // ---- 预设初始位姿 ----
  bool   set_initial_pose_ = false;
  double initial_pose_x_=0, initial_pose_y_=0, initial_pose_z_=0;
  double initial_pose_qx_=0, initial_pose_qy_=0, initial_pose_qz_=0, initial_pose_qw_=1;

  // ---- 搜索线程 ----
  std::shared_ptr<std::thread> search_thread_;   // 异步搜索线程
  std::atomic<bool> search_running_{false};      // 搜索是否正在运行 (原子变量，线程安全)
  pcl::PointCloud<pcl::PointXYZI>::Ptr last_scan_for_search_;  // 搜索用扫描缓存
};

// ============================================================================
// 主函数
// ============================================================================
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // 使用 MultiThreadedExecutor：主线程处理 ROS 消息，搜索线程运行 NDT 匹配
  rclcpp::executors::MultiThreadedExecutor exec;
  auto node = std::make_shared<GlobalLocalizationNode>(rclcpp::NodeOptions());
  exec.add_node(node->get_node_base_interface());
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
