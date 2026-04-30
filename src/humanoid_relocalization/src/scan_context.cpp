#include "humanoid_relocalization/scan_context.hpp"
#include <fstream>
#include <algorithm>
#include <cmath>
#include <iostream>

namespace relocalization {

ScanContext::ScanContext(const Config& config) : config_(config) {
    // 初始化 ring_key_matrix_ 为空矩阵
    ring_key_matrix_.resize(config_.num_rings, 0);
}

Eigen::MatrixXf ScanContext::computeDescriptor(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) const {
    // ★★★ 修复：使用相对高度而不是绝对高度，消除雷达安装高度变化的影响
    // 原因：建图时雷达高度 1.31m，导航时 1.21m，差了 0.1m
    //      如果用绝对高度，描述子不匹配；用相对高度则不受影响
    
    // 第 1 步：找到点云中的最小 Z 值（地面估计）
    float min_z = std::numeric_limits<float>::max();
    for (const auto& point : cloud->points) {
        double range = std::sqrt(point.x * point.x + point.y * point.y);
        if (range > config_.max_range || range < 0.1) continue;
        if (point.z < min_z) min_z = point.z;
    }
    
    // 初始化描述子矩阵 (rings x sectors)
    Eigen::MatrixXf descriptor = Eigen::MatrixXf::Zero(config_.num_rings, config_.num_sectors);
    Eigen::MatrixXi count = Eigen::MatrixXi::Zero(config_.num_rings, config_.num_sectors);

    const double ring_step = config_.max_range / config_.num_rings;
    const double sector_step = 2.0 * M_PI / config_.num_sectors;

    for (const auto& point : cloud->points) {
        // 计算点到原点的水平距离
        double range = std::sqrt(point.x * point.x + point.y * point.y);

        // 跳过超出范围的点
        if (range > config_.max_range || range < 0.1) {
            continue;
        }

        // 计算角度 (0 ~ 2*PI)
        double angle = std::atan2(point.y, point.x);
        if (angle < 0) {
            angle += 2.0 * M_PI;
        }

        // 计算 ring 和 sector 索引
        int ring_idx = static_cast<int>(range / ring_step);
        int sector_idx = static_cast<int>(angle / sector_step);

        // 边界检查
        ring_idx = std::min(ring_idx, config_.num_rings - 1);
        sector_idx = std::min(sector_idx, config_.num_sectors - 1);

        // ★★★ 修改：使用相对高度（相对于地面），而不是绝对高度
        // 这样即使雷达安装高度变化，描述子也保持一致
        float relative_height = point.z - min_z;  // 相对于地面的高度
        
        // 使用最大相对高度作为描述子值
        if (relative_height > descriptor(ring_idx, sector_idx)) {
            descriptor(ring_idx, sector_idx) = relative_height;
        }
        count(ring_idx, sector_idx)++;
    }

    return descriptor;
}

Eigen::VectorXf ScanContext::computeRingKey(const Eigen::MatrixXf& descriptor) const {
    // Ring Key 是每个环的平均值
    Eigen::VectorXf ring_key(config_.num_rings);
    for (int i = 0; i < config_.num_rings; ++i) {
        ring_key(i) = descriptor.row(i).mean();
    }
    return ring_key;
}

Eigen::MatrixXf ScanContext::circularShift(const Eigen::MatrixXf& matrix, int shift) const {
    int cols = matrix.cols();
    shift = ((shift % cols) + cols) % cols;  // 确保 shift 为正
    
    if (shift == 0) {
        return matrix;
    }
    
    Eigen::MatrixXf shifted(matrix.rows(), cols);
    shifted.leftCols(cols - shift) = matrix.rightCols(cols - shift);
    shifted.rightCols(shift) = matrix.leftCols(shift);
    
    return shifted;
}

float ScanContext::computeColumnDistance(const Eigen::MatrixXf& desc1, const Eigen::MatrixXf& desc2) const {
    // 计算两个描述子的列向量余弦距离
    float sum = 0.0f;
    int valid_cols = 0;
    
    for (int i = 0; i < desc1.cols(); ++i) {
        Eigen::VectorXf col1 = desc1.col(i);
        Eigen::VectorXf col2 = desc2.col(i);
        
        float norm1 = col1.norm();
        float norm2 = col2.norm();
        
        if (norm1 > 1e-6 && norm2 > 1e-6) {
            float cos_sim = col1.dot(col2) / (norm1 * norm2);
            sum += (1.0f - cos_sim);  // 转换为距离
            valid_cols++;
        }
    }
    
    return valid_cols > 0 ? sum / valid_cols : 1.0f;
}

std::pair<float, int> ScanContext::computeDistance(const Eigen::MatrixXf& desc1, const Eigen::MatrixXf& desc2) const {
    float min_distance = std::numeric_limits<float>::max();
    int best_shift = 0;
    
    // 尝试所有可能的列移位
    for (int shift = 0; shift < config_.num_sectors; ++shift) {
        Eigen::MatrixXf shifted_desc2 = circularShift(desc2, shift);
        float distance = computeColumnDistance(desc1, shifted_desc2);
        
        if (distance < min_distance) {
            min_distance = distance;
            best_shift = shift;
        }
    }
    
    return std::make_pair(min_distance, best_shift);
}

void ScanContext::addKeyFrame(const KeyFrame& keyframe) {
    keyframes_.push_back(keyframe);
    
    // 更新 ring_key_matrix_
    if (ring_key_matrix_.cols() == 0) {
        ring_key_matrix_ = keyframe.ring_key;
    } else {
        Eigen::MatrixXf new_matrix(config_.num_rings, ring_key_matrix_.cols() + 1);
        new_matrix.leftCols(ring_key_matrix_.cols()) = ring_key_matrix_;
        new_matrix.col(ring_key_matrix_.cols()) = keyframe.ring_key;
        ring_key_matrix_ = new_matrix;
    }
}

void ScanContext::addKeyFrame(int id, const Eigen::Matrix4f& pose, 
                               const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    KeyFrame kf;
    kf.id = id;
    kf.pose = pose;
    kf.cloud = cloud;
    kf.sc_descriptor = computeDescriptor(cloud);
    kf.ring_key = computeRingKey(kf.sc_descriptor);
    
    addKeyFrame(kf);
}

std::vector<ScanContext::SearchResult> ScanContext::search(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& query_cloud, 
    int top_k, 
    float distance_threshold) const {
    
    std::vector<SearchResult> results;
    
    if (keyframes_.empty()) {
        return results;
    }
    
    // 计算查询点云的描述子
    Eigen::MatrixXf query_desc = computeDescriptor(query_cloud);
    Eigen::VectorXf query_ring_key = computeRingKey(query_desc);
    
    // 使用 ring key 快速筛选候选帧
    std::vector<std::pair<float, int>> candidates;
    for (size_t i = 0; i < keyframes_.size(); ++i) {
        float ring_dist = (query_ring_key - keyframes_[i].ring_key).norm();
        candidates.emplace_back(ring_dist, static_cast<int>(i));
    }
    
    // 按 ring key 距离排序，取前 top_k * 2 个候选
    std::sort(candidates.begin(), candidates.end());
    int num_candidates = std::min(static_cast<int>(candidates.size()), top_k * 2);
    
    // 对候选帧进行精确匹配
    std::vector<std::pair<float, SearchResult>> scored_results;
    for (int i = 0; i < num_candidates; ++i) {
        int kf_idx = candidates[i].second;
        const KeyFrame& kf = keyframes_[kf_idx];
        
        auto [distance, shift] = computeDistance(query_desc, kf.sc_descriptor);
        
        if (distance < distance_threshold) {
            SearchResult result;
            result.keyframe_id = kf.id;
            result.sc_distance = distance;
            result.yaw_offset = shift;
            
            // 计算估计位姿（考虑 yaw 偏移）
            float yaw_angle = static_cast<float>(shift) * 2.0f * M_PI / config_.num_sectors;
            Eigen::Matrix4f yaw_correction = Eigen::Matrix4f::Identity();
            yaw_correction(0, 0) = std::cos(yaw_angle);
            yaw_correction(0, 1) = -std::sin(yaw_angle);
            yaw_correction(1, 0) = std::sin(yaw_angle);
            yaw_correction(1, 1) = std::cos(yaw_angle);
            
            result.estimated_pose = kf.pose * yaw_correction;
            
            scored_results.emplace_back(distance, result);
        }
    }
    
    // 按距离排序
    std::sort(scored_results.begin(), scored_results.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
    
    // 取前 top_k 个结果
    for (int i = 0; i < std::min(top_k, static_cast<int>(scored_results.size())); ++i) {
        results.push_back(scored_results[i].second);
    }
    
    return results;
}

void ScanContext::clear() {
    keyframes_.clear();
    ring_key_matrix_.resize(config_.num_rings, 0);
}

bool ScanContext::saveDatabase(const std::string& filepath) const {
    std::ofstream ofs(filepath, std::ios::binary);
    if (!ofs.is_open()) {
        std::cerr << "Failed to open file for writing: " << filepath << std::endl;
        return false;
    }
    
    // 写入配置
    ofs.write(reinterpret_cast<const char*>(&config_.num_sectors), sizeof(int));
    ofs.write(reinterpret_cast<const char*>(&config_.num_rings), sizeof(int));
    ofs.write(reinterpret_cast<const char*>(&config_.max_range), sizeof(double));
    ofs.write(reinterpret_cast<const char*>(&config_.lidar_height), sizeof(double));
    
    // 写入关键帧数量
    size_t num_keyframes = keyframes_.size();
    ofs.write(reinterpret_cast<const char*>(&num_keyframes), sizeof(size_t));
    
    // 写入每个关键帧
    for (const auto& kf : keyframes_) {
        // ID
        ofs.write(reinterpret_cast<const char*>(&kf.id), sizeof(int));
        
        // Pose (4x4 matrix)
        ofs.write(reinterpret_cast<const char*>(kf.pose.data()), sizeof(float) * 16);
        
        // SC Descriptor
        int rows = kf.sc_descriptor.rows();
        int cols = kf.sc_descriptor.cols();
        ofs.write(reinterpret_cast<const char*>(&rows), sizeof(int));
        ofs.write(reinterpret_cast<const char*>(&cols), sizeof(int));
        ofs.write(reinterpret_cast<const char*>(kf.sc_descriptor.data()), sizeof(float) * rows * cols);
        
        // Ring Key
        int ring_size = kf.ring_key.size();
        ofs.write(reinterpret_cast<const char*>(&ring_size), sizeof(int));
        ofs.write(reinterpret_cast<const char*>(kf.ring_key.data()), sizeof(float) * ring_size);
    }
    
    ofs.close();
    std::cout << "Saved " << num_keyframes << " keyframes to " << filepath << std::endl;
    return true;
}

bool ScanContext::loadDatabase(const std::string& filepath) {
    std::ifstream ifs(filepath, std::ios::binary);
    if (!ifs.is_open()) {
        std::cerr << "Failed to open file for reading: " << filepath << std::endl;
        return false;
    }
    
    clear();
    
    // 读取配置
    ifs.read(reinterpret_cast<char*>(&config_.num_sectors), sizeof(int));
    ifs.read(reinterpret_cast<char*>(&config_.num_rings), sizeof(int));
    ifs.read(reinterpret_cast<char*>(&config_.max_range), sizeof(double));
    ifs.read(reinterpret_cast<char*>(&config_.lidar_height), sizeof(double));
    
    // 读取关键帧数量
    size_t num_keyframes;
    ifs.read(reinterpret_cast<char*>(&num_keyframes), sizeof(size_t));
    
    // 读取每个关键帧
    for (size_t i = 0; i < num_keyframes; ++i) {
        KeyFrame kf;
        
        // ID
        ifs.read(reinterpret_cast<char*>(&kf.id), sizeof(int));
        
        // Pose
        ifs.read(reinterpret_cast<char*>(kf.pose.data()), sizeof(float) * 16);
        
        // SC Descriptor
        int rows, cols;
        ifs.read(reinterpret_cast<char*>(&rows), sizeof(int));
        ifs.read(reinterpret_cast<char*>(&cols), sizeof(int));
        kf.sc_descriptor.resize(rows, cols);
        ifs.read(reinterpret_cast<char*>(kf.sc_descriptor.data()), sizeof(float) * rows * cols);
        
        // Ring Key
        int ring_size;
        ifs.read(reinterpret_cast<char*>(&ring_size), sizeof(int));
        kf.ring_key.resize(ring_size);
        ifs.read(reinterpret_cast<char*>(kf.ring_key.data()), sizeof(float) * ring_size);
        
        // 点云设为空（加载时不需要原始点云）
        kf.cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
        
        addKeyFrame(kf);
    }
    
    ifs.close();
    std::cout << "Loaded " << num_keyframes << " keyframes from " << filepath << std::endl;
    return true;
}
std::pair<int, float> ScanContext::searchKeyFrame(const Eigen::MatrixXf& query_desc, int top_k) const {
    if (keyframes_.empty()) {
        return std::make_pair(-1, std::numeric_limits<float>::max());
    }
    
    Eigen::VectorXf query_ring_key = computeRingKey(query_desc);
    
    // 使用 ring key 快速筛选候选帧
    std::vector<std::pair<float, int>> candidates;
    for (size_t i = 0; i < keyframes_.size(); ++i) {
        float ring_dist = (query_ring_key - keyframes_[i].ring_key).norm();
        candidates.emplace_back(ring_dist, static_cast<int>(i));
    }
    
    // 按 ring key 距离排序
    std::sort(candidates.begin(), candidates.end());
    int num_candidates = std::min(static_cast<int>(candidates.size()), top_k * 2);
    
    // 对候选帧进行精确匹配
    int best_idx = -1;
    float min_distance = std::numeric_limits<float>::max();
    
    for (int i = 0; i < num_candidates; ++i) {
        int kf_idx = candidates[i].second;
        const KeyFrame& kf = keyframes_[kf_idx];
        
        auto [distance, shift] = computeDistance(query_desc, kf.sc_descriptor);
        
        if (distance < min_distance) {
            min_distance = distance;
            best_idx = kf_idx;
        }
    }
    
    return std::make_pair(best_idx, min_distance);
}
} // namespace relocalization