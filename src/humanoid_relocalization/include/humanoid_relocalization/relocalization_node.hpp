#ifndef RELOCALIZATION_NODE_HPP
#define RELOCALIZATION_NODE_HPP

#include <memory>
#include <string>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "humanoid_relocalization/scan_context.hpp"

namespace relocalization {

/**
 * @brief 重定位结果结构体
 */
struct RelocalizationResult {
    bool success = false;
    geometry_msgs::msg::Pose pose;
    float yaw_deg = 0.0f;
    float fitness_score = 0.0f;
    int matched_keyframe_id = -1;
    float sc_distance = 0.0f;
    std::string message;
};

/**
 * @brief 重定位状态枚举
 */
enum class RelocalizationState {
    IDLE,           // 空闲，已完成重定位
    WAITING,        // 等待点云数据
    PROCESSING,     // 正在处理
    SUCCESS,        // 重定位成功
    FAILED          // 重定位失败
};

/**
 * @brief 3D 重定位节点
 * 
 * 使用 Scan Context 进行全局位置搜索，然后用 ICP 精配准
 * 最终将初始位姿发布到 /initialpose 供 slam_toolbox 使用
 */
class RelocalizationNode : public rclcpp::Node {
public:
    /**
     * @brief 构造函数
     */
    explicit RelocalizationNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    
    /**
     * @brief 析构函数
     */
    ~RelocalizationNode() override = default;

private:
    // ==================== 初始化函数 ====================
    
    /**
     * @brief 声明并获取 ROS 参数
     */
    void declareParameters();
    
    /**
     * @brief 加载 Scan Context 数据库
     * @return 是否加载成功
     */
    bool loadSCDatabase();
    
    /**
     * @brief 加载全局点云地图
     * @return 是否加载成功
     */
    bool loadGlobalMap();
    
    /**
     * @brief 设置 ROS 通信 (订阅、发布、服务)
     */
    void setupROSCommunication();

    // ==================== 回调函数 ====================
    
    /**
     * @brief 点云回调函数
     * @param msg 点云消息
     */
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    
    /**
     * @brief 手动触发重定位服务回调
     */
    void relocalizeSrvCallback(
        const std_srvs::srv::Trigger::Request::SharedPtr request,
        std_srvs::srv::Trigger::Response::SharedPtr response);
    
    /**
     * @brief 状态发布定时器回调
     */
    void statusTimerCallback();

    // ==================== 核心算法函数 ====================
    
    /**
     * @brief 执行重定位
     * @param current_cloud 当前点云 (已降采样)
     * @return 重定位结果
     */
    RelocalizationResult performRelocalization(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& current_cloud);
    
    /**
     * @brief Scan Context 全局搜索
     * @param cloud 输入点云
     * @param best_idx 输出：最佳匹配关键帧索引
     * @param sc_distance 输出：SC 距离
     * @return 是否找到有效匹配
     */
    bool scanContextSearch(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
        int& best_idx,
        float& sc_distance);
    
    /**
     * @brief ICP 精配准
     * @param source 源点云 (当前帧)
     * @param initial_guess 初始位姿猜测
     * @param final_transform 输出：最终变换矩阵
     * @param fitness_score 输出：配准适应度分数
     * @return 是否配准成功
     */
    bool icpAlignment(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& source,
        const Eigen::Matrix4f& initial_guess,
        Eigen::Matrix4f& final_transform,
        float& fitness_score);
    
    /**
     * @brief 点云降采样
     * @param cloud_in 输入点云
     * @param cloud_out 输出点云
     * @param leaf_size 体素大小
     */
    void downsampleCloud(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_in,
        pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_out,
        float leaf_size);

    // ==================== 工具函数 ====================
    
    /**
     * @brief 发布初始位姿到 /initialpose
     * @param pose 位姿
     */
    void publishInitialPose(const geometry_msgs::msg::Pose& pose);
    
    /**
     * @brief 发布重定位状态
     * @param state 状态
     * @param message 附加消息
     */
    void publishStatus(RelocalizationState state, const std::string& message = "");
    
    /**
     * @brief 从变换矩阵提取 geometry_msgs::Pose
     * @param transform 4x4 变换矩阵
     * @return Pose 消息
     */
    geometry_msgs::msg::Pose matrixToPose(const Eigen::Matrix4f& transform);
    
    /**
     * @brief 从变换矩阵提取 yaw 角 (度)
     * @param transform 4x4 变换矩阵
     * @return yaw 角度
     */
    float extractYawDeg(const Eigen::Matrix4f& transform);

    /**
     * @brief 将位姿从 camera_init 坐标系变换到 base_footprint 坐标系
     * @param pose_camera_init 在 camera_init 坐标系下的位姿
     * @param pose_bf 输出：在 base_footprint 坐标系下的位姿
     * @return 是否变换成功
     */
    bool transformPoseToBaseFootprint(
        const geometry_msgs::msg::Pose& pose_camera_init,
        geometry_msgs::msg::Pose& pose_bf);

    /**
     * @brief 获取状态字符串
     * @param state 状态枚举
     * @return 状态字符串
     */
    std::string stateToString(RelocalizationState state);

    // ==================== 成员变量：核心组件 ====================
    
    /// Scan Context 管理器
    std::unique_ptr<ScanContext> scan_context_;
    
    /// 全局点云地图 (用于 ICP)
    pcl::PointCloud<pcl::PointXYZI>::Ptr global_map_;
    
    /// 降采样后的全局地图 (加速 ICP)
    pcl::PointCloud<pcl::PointXYZI>::Ptr global_map_downsampled_;

    // ==================== 成员变量：参数 ====================
    
    /// SC 数据库路径
    std::string sc_database_path_;
    
    /// PCD 地图路径
    std::string pcd_map_path_;
    
    /// ICP 最大迭代次数
    int icp_max_iterations_;
    
    /// ICP 最大对应点距离
    double icp_max_corr_dist_;
    
    /// ICP 适应度阈值
    double icp_fitness_threshold_;
    
    /// SC 距离阈值
    double sc_distance_threshold_;
    
    /// 体素降采样大小
    double voxel_leaf_size_;
    
    /// 是否自动持续重定位
    bool auto_relocalize_;
    
    /// 启动时是否自动重定位
    bool relocalize_on_startup_;
    
    /// SC 候选数量 (top-k)
    int sc_top_k_candidates_;
    
    /// 重定位最大尝试次数
    int max_relocalization_attempts_;
    
    /// 点云话题名
    std::string cloud_topic_;
    
    /// 初始位姿话题名
    std::string initial_pose_topic_;
    
    /// 目标坐标系
    std::string target_frame_;

    // ==================== 成员变量：状态 ====================
    
    /// 是否需要重定位
    std::atomic<bool> need_relocalize_{false};
    
    /// 是否已完成重定位
    std::atomic<bool> relocalized_{false};
    
    /// 当前状态
    RelocalizationState current_state_{RelocalizationState::IDLE};
    
    /// 重定位尝试次数
    int relocalization_attempts_{0};
    
    /// 最后一次重定位结果
    RelocalizationResult last_result_;
    
    /// 接收到的点云帧数
    int cloud_frame_count_{0};

    // ==================== 成员变量：ROS 通信 ====================
    
    /// 点云订阅器
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    
    /// 初始位姿发布器
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    
    /// 状态发布器
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    
    /// 重定位服务
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr relocalize_srv_;
    
    /// 状态发布定时器
    rclcpp::TimerBase::SharedPtr status_timer_;
    
    /// TF Buffer
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    
    /// TF Listener
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

} // namespace relocalization

#endif // RELOCALIZATION_NODE_HPP