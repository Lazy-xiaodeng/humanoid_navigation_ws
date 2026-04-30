#include "humanoid_relocalization/relocalization_node.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace relocalization {

RelocalizationNode::RelocalizationNode(const rclcpp::NodeOptions& options)
    : Node("relocalization_node", options) {
    RCLCPP_INFO(this->get_logger(), "Initializing Relocalization Node...");
    
    // 初始化
    declareParameters();
    
    if (!loadSCDatabase()) {
        RCLCPP_WARN(this->get_logger(), "SC database not loaded, relocalization will fail");
    }
    
    if (!loadGlobalMap()) {
        RCLCPP_WARN(this->get_logger(), "Global map not loaded, relocalization will fail");
    }
    
    setupROSCommunication();
    
    if (relocalize_on_startup_) {
        need_relocalize_ = true;
        current_state_ = RelocalizationState::WAITING;
        RCLCPP_INFO(this->get_logger(), "Will relocalize on first point cloud");
    }
    
    RCLCPP_INFO(this->get_logger(), "Relocalization Node initialized successfully");
}

void RelocalizationNode::declareParameters() {
    // 路径参数
    this->declare_parameter("sc_database_path", "");
    this->declare_parameter("pcd_map_path", "");
    this->declare_parameter("cloud_topic", "/fast_lio/cloud_registered");
    this->declare_parameter("initial_pose_topic", "/initialpose");
    this->declare_parameter("target_frame", "map");
    
    // SC 参数
    this->declare_parameter("sc_num_sectors", 60);
    this->declare_parameter("sc_num_rings", 20);
    this->declare_parameter("sc_max_range", 60.0);
    this->declare_parameter("lidar_height", 1.48);
    this->declare_parameter("sc_distance_threshold", 0.4);
    this->declare_parameter("sc_top_k_candidates", 10);
    
    // ICP 参数
    this->declare_parameter("icp_max_iterations", 50);
    this->declare_parameter("icp_max_correspondence_distance", 2.0);
    this->declare_parameter("icp_fitness_threshold", 0.5);
    
    // 点云处理
    this->declare_parameter("voxel_leaf_size", 0.2);
    
    // 行为控制
    this->declare_parameter("auto_relocalize", false);
    this->declare_parameter("relocalize_on_startup", true);
    this->declare_parameter("max_relocalization_attempts", 3);
    
    // 获取参数
    sc_database_path_ = this->get_parameter("sc_database_path").as_string();
    pcd_map_path_ = this->get_parameter("pcd_map_path").as_string();
    cloud_topic_ = this->get_parameter("cloud_topic").as_string();
    initial_pose_topic_ = this->get_parameter("initial_pose_topic").as_string();
    target_frame_ = this->get_parameter("target_frame").as_string();
    
    icp_max_iterations_ = this->get_parameter("icp_max_iterations").as_int();
    icp_max_corr_dist_ = this->get_parameter("icp_max_correspondence_distance").as_double();
    icp_fitness_threshold_ = this->get_parameter("icp_fitness_threshold").as_double();
    sc_distance_threshold_ = this->get_parameter("sc_distance_threshold").as_double();
    sc_top_k_candidates_ = this->get_parameter("sc_top_k_candidates").as_int();
    voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();
    auto_relocalize_ = this->get_parameter("auto_relocalize").as_bool();
    relocalize_on_startup_ = this->get_parameter("relocalize_on_startup").as_bool();
    max_relocalization_attempts_ = this->get_parameter("max_relocalization_attempts").as_int();
    
    // 初始化 Scan Context 配置
    ScanContext::Config sc_config;
    sc_config.num_sectors = this->get_parameter("sc_num_sectors").as_int();
    sc_config.num_rings = this->get_parameter("sc_num_rings").as_int();
    sc_config.max_range = this->get_parameter("sc_max_range").as_double();
    sc_config.lidar_height = this->get_parameter("lidar_height").as_double();
    
    scan_context_ = std::make_unique<ScanContext>(sc_config);
    
    RCLCPP_INFO(this->get_logger(), "Parameters loaded:");
    RCLCPP_INFO(this->get_logger(), "  SC database: %s", sc_database_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "  PCD map: %s", pcd_map_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Cloud topic: %s", cloud_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Lidar height: %.2f m", sc_config.lidar_height);
}

bool RelocalizationNode::loadSCDatabase() {
    if (sc_database_path_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "SC database path is empty");
        return false;
    }
    
    if (scan_context_->loadDatabase(sc_database_path_)) {
        RCLCPP_INFO(this->get_logger(), "Loaded SC database with %zu keyframes", 
                   scan_context_->size());
        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to load SC database: %s", 
                    sc_database_path_.c_str());
        return false;
    }
}

bool RelocalizationNode::loadGlobalMap() {
    if (pcd_map_path_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "PCD map path is empty");
        return false;
    }
    
    global_map_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    
    if (pcl::io::loadPCDFile(pcd_map_path_, *global_map_) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load PCD map: %s", pcd_map_path_.c_str());
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Loaded PCD map with %zu points", global_map_->size());
    
    // 降采样全局地图
    global_map_downsampled_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    downsampleCloud(global_map_, global_map_downsampled_, voxel_leaf_size_);
    
    RCLCPP_INFO(this->get_logger(), "Downsampled global map to %zu points", 
               global_map_downsampled_->size());
    
    return true;
}

void RelocalizationNode::setupROSCommunication() {
    // 订阅点云
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, 
        rclcpp::SensorDataQoS(),
        std::bind(&RelocalizationNode::cloudCallback, this, std::placeholders::_1));
    
    // 发布初始位姿
    initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        initial_pose_topic_, 10);
    
    // 发布状态
    status_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/relocalization/status", 10);
    
    // 重定位服务
    relocalize_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/relocalization/trigger",
        std::bind(&RelocalizationNode::relocalizeSrvCallback, this, 
                 std::placeholders::_1, std::placeholders::_2));
    
    // 状态定时器 (1Hz)
    status_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&RelocalizationNode::statusTimerCallback, this));
    
    // TF
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    
    RCLCPP_INFO(this->get_logger(), "ROS communication setup complete");
}

void RelocalizationNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    cloud_frame_count_++;

    if (!need_relocalize_) {
        return;
    }

    // ★★★ 关键修复：直接使用 camera_init 坐标系进行重定位 ★★★
    // PCD 地图是在建图时以 camera_init 坐标系保存的（Fast-LIO 的世界坐标系）
    // 因此不需要将点云变换到 base_footprint，直接在 camera_init 下匹配即可
    //
    // TF 树结构：
    //   odom → camera_init （static transform，零变换）
    //   camera_init → body （Fast-LIO 内部发布）
    //   body → base_footprint （static transform）
    //
    // 重定位后，将结果位姿从 camera_init 变换到 base_footprint 发布

    // Step 1: 将 ROS 点云消息转换为 PCL 点云（点云的 frame_id = "camera_init"）
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_camera_init(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *cloud_in_camera_init);

    RCLCPP_DEBUG(this->get_logger(), "收到点云，frame_id=%s, 点数=%zu",
                msg->header.frame_id.c_str(), cloud_in_camera_init->size());

    if (cloud_in_camera_init->empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "Received empty point cloud");
        return;
    }

    // Step 2: 降采样（直接在 camera_init 坐标系下）
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());
    downsampleCloud(cloud_in_camera_init, cloud_filtered, voxel_leaf_size_);

    if (cloud_filtered->size() < 100) {
        RCLCPP_WARN(this->get_logger(), "Too few points after filtering: %zu",
                   cloud_filtered->size());
        return;
    }

    // 执行重定位（在 camera_init 坐标系下）
    current_state_ = RelocalizationState::PROCESSING;
    RCLCPP_INFO(this->get_logger(), "Starting relocalization (attempt %d/%d)...",
               relocalization_attempts_ + 1, max_relocalization_attempts_);

    auto result = performRelocalization(cloud_filtered);
    last_result_ = result;

    if (result.success) {
        // 重定位成功，结果位姿是在 camera_init 坐标系下
        // 需要变换到 base_footprint 坐标系再发布
        RCLCPP_INFO(this->get_logger(),
            "✅ Relocalization SUCCESS (camera_init)! Pose: [%.2f, %.2f, %.2f], yaw: %.1f°, "
            "SC dist: %.3f, ICP fitness: %.4f, matched KF: %d",
            result.pose.position.x, result.pose.position.y, result.pose.position.z,
            result.yaw_deg, result.sc_distance, result.fitness_score,
            result.matched_keyframe_id);

        // 将位姿从 camera_init 变换到 base_footprint
        geometry_msgs::msg::Pose pose_bf;
        if (transformPoseToBaseFootprint(result.pose, pose_bf)) {
            publishInitialPose(pose_bf);
            RCLCPP_INFO(this->get_logger(), "位姿已变换到 base_footprint 坐标系并发布");
        } else {
            // 如果 TF 不可用，直接发布 camera_init 坐标系的位姿
            RCLCPP_WARN(this->get_logger(), "TF 不可用，直接发布 camera_init 坐标系位姿");
            publishInitialPose(result.pose);
        }

        need_relocalize_ = false;
        relocalized_ = true;
        relocalization_attempts_ = 0;
        current_state_ = RelocalizationState::SUCCESS;

    } else {
        relocalization_attempts_++;
        RCLCPP_WARN(this->get_logger(), "❌ Relocalization FAILED: %s", result.message.c_str());

        if (relocalization_attempts_ >= max_relocalization_attempts_) {
            RCLCPP_ERROR(this->get_logger(),
                "Max relocalization attempts reached (%d), giving up",
                max_relocalization_attempts_);
            need_relocalize_ = false;
            current_state_ = RelocalizationState::FAILED;
        } else {
            current_state_ = RelocalizationState::WAITING;
        }
    }
}

RelocalizationResult RelocalizationNode::performRelocalization(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& current_cloud) {
    
    RelocalizationResult result;
    
    // 检查前置条件
    if (scan_context_->size() == 0) {
        result.message = "SC database is empty";
        return result;
    }
    
    if (!global_map_downsampled_ || global_map_downsampled_->empty()) {
        result.message = "Global map is empty";
        return result;
    }
    
    // Step 1: Scan Context 全局搜索
    int best_idx;
    float sc_distance;
    
    if (!scanContextSearch(current_cloud, best_idx, sc_distance)) {
        result.message = "SC search failed: no matching keyframe (dist=" + 
                        std::to_string(sc_distance) + ")";
        result.sc_distance = sc_distance;
        return result;
    }
    
    result.matched_keyframe_id = best_idx;
    result.sc_distance = sc_distance;
    
    RCLCPP_INFO(this->get_logger(), "SC search: matched KF %d with distance %.4f", 
               best_idx, sc_distance);
    
    // Step 2: 获取初始位姿猜测
    const auto& candidate_kf = scan_context_->getKeyFrame(best_idx);
    Eigen::Matrix4f initial_guess = candidate_kf.pose;
    
    // Step 3: ICP 精配准
    Eigen::Matrix4f final_transform;
    float fitness_score;
    
    if (!icpAlignment(current_cloud, initial_guess, final_transform, fitness_score)) {
        result.message = "ICP alignment failed (fitness=" + std::to_string(fitness_score) + ")";
        result.fitness_score = fitness_score;
        return result;
    }
    
    result.fitness_score = fitness_score;
    
    // Step 4: 提取位姿
    result.pose = matrixToPose(final_transform);
    result.yaw_deg = extractYawDeg(final_transform);
    result.success = true;
    
    return result;
}

bool RelocalizationNode::scanContextSearch(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    int& best_idx,
    float& sc_distance) {
    
    auto query_sc = scan_context_->computeDescriptor(cloud);
    auto [idx, dist] = scan_context_->searchKeyFrame(query_sc, sc_top_k_candidates_);
    
    best_idx = idx;
    sc_distance = dist;
    
    return (idx >= 0 && dist <= sc_distance_threshold_);
}

bool RelocalizationNode::icpAlignment(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& source,
    const Eigen::Matrix4f& initial_guess,
    Eigen::Matrix4f& final_transform,
    float& fitness_score) {
    
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> gicp;
    gicp.setInputSource(source);
    gicp.setInputTarget(global_map_downsampled_);
    gicp.setMaximumIterations(icp_max_iterations_);
    gicp.setMaxCorrespondenceDistance(icp_max_corr_dist_);
    gicp.setTransformationEpsilon(1e-6);
    gicp.setEuclideanFitnessEpsilon(1e-6);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZI>());
    gicp.align(*aligned, initial_guess);
    
    fitness_score = gicp.getFitnessScore();
    
    if (!gicp.hasConverged()) {
        RCLCPP_WARN(this->get_logger(), "ICP did not converge");
        return false;
    }
    
    if (fitness_score > icp_fitness_threshold_) {
        RCLCPP_WARN(this->get_logger(), "ICP fitness too high: %.4f > %.4f", 
                   fitness_score, icp_fitness_threshold_);
        return false;
    }
    
    final_transform = gicp.getFinalTransformation();
    return true;
}

void RelocalizationNode::downsampleCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_out,
    float leaf_size) {
    
    pcl::VoxelGrid<pcl::PointXYZI> voxel;
    voxel.setInputCloud(cloud_in);
    voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel.filter(*cloud_out);
}

void RelocalizationNode::publishInitialPose(const geometry_msgs::msg::Pose& pose) {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = target_frame_;
    msg.pose.pose = pose;
    
    // 设置协方差矩阵 (对角线元素)
    // 较小的值表示高置信度
    std::fill(msg.pose.covariance.begin(), msg.pose.covariance.end(), 0.0);
    msg.pose.covariance[0] = 0.1;   // x 方差
    msg.pose.covariance[7] = 0.1;   // y 方差
    msg.pose.covariance[14] = 0.1;  // z 方差
    msg.pose.covariance[21] = 0.05; // roll 方差
    msg.pose.covariance[28] = 0.05; // pitch 方差
    msg.pose.covariance[35] = 0.05; // yaw 方差
    
    initial_pose_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published initial pose to %s", initial_pose_topic_.c_str());
}

void RelocalizationNode::publishStatus(RelocalizationState state, const std::string& message) {
    std_msgs::msg::String msg;
    
    std::stringstream ss;
    ss << "{";
    ss << "\"state\": \"" << stateToString(state) << "\", ";
    ss << "\"relocalized\": " << (relocalized_.load() ? "true" : "false") << ", ";
    ss << "\"attempts\": " << relocalization_attempts_ << ", ";
    ss << "\"cloud_frames\": " << cloud_frame_count_ << ", ";
    ss << "\"sc_database_size\": " << scan_context_->size() << ", ";
    
    if (last_result_.success) {
        ss << "\"last_pose\": {"
           << "\"x\": " << last_result_.pose.position.x << ", "
           << "\"y\": " << last_result_.pose.position.y << ", "
           << "\"z\": " << last_result_.pose.position.z << ", "
           << "\"yaw_deg\": " << last_result_.yaw_deg << "}, ";
        ss << "\"last_sc_distance\": " << last_result_.sc_distance << ", ";
        ss << "\"last_icp_fitness\": " << last_result_.fitness_score << ", ";
        ss << "\"matched_keyframe\": " << last_result_.matched_keyframe_id << ", ";
    }
    
    ss << "\"message\": \"" << message << "\"";
    ss << "}";
    
    msg.data = ss.str();
    status_pub_->publish(msg);
}

geometry_msgs::msg::Pose RelocalizationNode::matrixToPose(const Eigen::Matrix4f& transform) {
    geometry_msgs::msg::Pose pose;
    
    // 提取平移
    pose.position.x = transform(0, 3);
    pose.position.y = transform(1, 3);
    pose.position.z = transform(2, 3);
    
    // 提取旋转矩阵并转换为四元数
    Eigen::Matrix3f rotation = transform.block<3, 3>(0, 0);
    Eigen::Quaternionf q(rotation);
    q.normalize();
    
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    
    return pose;
}

float RelocalizationNode::extractYawDeg(const Eigen::Matrix4f& transform) {
    Eigen::Matrix3f rotation = transform.block<3, 3>(0, 0);
    Eigen::Quaternionf q(rotation);
    
    // 从四元数提取 yaw 角
    float siny_cosp = 2.0f * (q.w() * q.z() + q.x() * q.y());
    float cosy_cosp = 1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z());
    float yaw = std::atan2(siny_cosp, cosy_cosp);
    
    return yaw * 180.0f / M_PI;
}

std::string RelocalizationNode::stateToString(RelocalizationState state) {
    switch (state) {
        case RelocalizationState::IDLE:       return "IDLE";
        case RelocalizationState::WAITING:    return "WAITING";
        case RelocalizationState::PROCESSING: return "PROCESSING";
        case RelocalizationState::SUCCESS:    return "SUCCESS";
        case RelocalizationState::FAILED:     return "FAILED";
        default:                              return "UNKNOWN";
    }
}

bool RelocalizationNode::transformPoseToBaseFootprint(
    const geometry_msgs::msg::Pose& pose_camera_init,
    geometry_msgs::msg::Pose& pose_bf) {

    try {
        // 获取 camera_init → base_footprint 的 TF 变换
        geometry_msgs::msg::TransformStamped tf_cam_to_bf = tf_buffer_->lookupTransform(
            "base_footprint",    // 目标坐标系
            "camera_init",       // 源坐标系
            tf2::TimePointZero
        );

        // 将 Pose 从 camera_init 变换到 base_footprint
        tf2::doTransform(pose_camera_init, pose_bf, tf_cam_to_bf);
        return true;

    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "TF 变换失败 (camera_init → base_footprint): %s", ex.what());
        return false;
    }
}

void RelocalizationNode::relocalizeSrvCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
    
    RCLCPP_INFO(this->get_logger(), "Relocalization manually triggered via service");
    
    need_relocalize_ = true;
    relocalized_ = false;
    relocalization_attempts_ = 0;
    current_state_ = RelocalizationState::WAITING;
    
    response->success = true;
    response->message = "Relocalization triggered, waiting for point cloud...";
}

void RelocalizationNode::statusTimerCallback() {
    std::string message;
    
    switch (current_state_) {
        case RelocalizationState::IDLE:
            message = relocalized_.load() ? "Relocalization complete" : "Idle";
            break;
        case RelocalizationState::WAITING:
            message = "Waiting for point cloud data...";
            break;
        case RelocalizationState::PROCESSING:
            message = "Processing relocalization...";
            break;
        case RelocalizationState::SUCCESS:
            message = "Relocalization successful";
            break;
        case RelocalizationState::FAILED:
            message = "Relocalization failed: " + last_result_.message;
            break;
    }
    
    publishStatus(current_state_, message);
}

} // namespace relocalization

// ==================== Main 函数 ====================
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<relocalization::RelocalizationNode>();
    
    RCLCPP_INFO(node->get_logger(), "Relocalization node started");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}