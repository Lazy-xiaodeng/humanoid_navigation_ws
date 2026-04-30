#ifndef HUMANOID_RELOCALIZATION_SCAN_CONTEXT_HPP_
#define HUMANOID_RELOCALIZATION_SCAN_CONTEXT_HPP_

#include <vector>
#include <memory>
#include <string>
#include <utility>
#include <limits>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace relocalization {

class ScanContext {
public:
    // 配置结构体
    struct Config {
        int num_sectors;
        int num_rings;
        double max_range;
        double lidar_height;
        
        Config() 
            : num_sectors(60)
            , num_rings(20)
            , max_range(80.0)
            , lidar_height(1.5) 
        {}
        
        Config(int sectors, int rings, double range, double height)
            : num_sectors(sectors)
            , num_rings(rings)
            , max_range(range)
            , lidar_height(height)
        {}
    };

    // 关键帧结构
    struct KeyFrame {
        int id;
        Eigen::Matrix4f pose;
        Eigen::MatrixXf sc_descriptor;
        Eigen::VectorXf ring_key;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
        
        KeyFrame() 
            : id(-1)
            , pose(Eigen::Matrix4f::Identity())
            , cloud(new pcl::PointCloud<pcl::PointXYZI>) 
        {}
    };

    // 搜索结果结构
    struct SearchResult {
        int keyframe_id;
        float sc_distance;
        int yaw_offset;
        Eigen::Matrix4f estimated_pose;
        
        SearchResult() 
            : keyframe_id(-1)
            , sc_distance(std::numeric_limits<float>::max())
            , yaw_offset(0)
            , estimated_pose(Eigen::Matrix4f::Identity()) 
        {}
    };

public:
    // 构造函数和析构函数
    explicit ScanContext(const Config& config = Config());
    ~ScanContext() = default;

    // 计算 Scan Context 描述子
    Eigen::MatrixXf computeDescriptor(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) const;
    
    // 计算 Ring Key
    Eigen::VectorXf computeRingKey(const Eigen::MatrixXf& descriptor) const;
    
    // 计算两个描述子之间的距离（返回距离和最佳偏移）
    std::pair<float, int> computeDistance(const Eigen::MatrixXf& desc1, const Eigen::MatrixXf& desc2) const;
    
    // 添加关键帧
    void addKeyFrame(const KeyFrame& keyframe);
    void addKeyFrame(int id, const Eigen::Matrix4f& pose, 
                     const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    
    // 搜索最相似的关键帧
    std::vector<SearchResult> search(const pcl::PointCloud<pcl::PointXYZI>::Ptr& query_cloud, 
                                     int top_k = 5, 
                                     float distance_threshold = 0.5f) const;
    // 搜索最匹配的关键帧（返回索引和距离）
    std::pair<int, float> searchKeyFrame(const Eigen::MatrixXf& query_desc, int top_k = 5) const;
    
    // 获取关键帧
    const KeyFrame& getKeyFrame(int index) const { return keyframes_[index]; }
    const std::vector<KeyFrame>& getAllKeyFrames() const { return keyframes_; }
    
    // 数据库操作
    size_t size() const { return keyframes_.size(); }
    bool empty() const { return keyframes_.empty(); }
    void clear();
    
    // 保存/加载数据库
    bool saveDatabase(const std::string& filepath) const;
    bool loadDatabase(const std::string& filepath);
    
    // 获取配置
    const Config& getConfig() const { return config_; }

private:
    // 列循环移位
    Eigen::MatrixXf circularShift(const Eigen::MatrixXf& matrix, int shift) const;
    
    // 计算单次移位距离
    float computeColumnDistance(const Eigen::MatrixXf& desc1, const Eigen::MatrixXf& desc2) const;

private:
    Config config_;
    std::vector<KeyFrame> keyframes_;
    Eigen::MatrixXf ring_key_matrix_;  // 所有关键帧的 ring key 矩阵，用于快速搜索
};

} // namespace relocalization

#endif // HUMANOID_RELOCALIZATION_SCAN_CONTEXT_HPP_