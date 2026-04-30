/**
 * @file elevation_map_opencl.hpp
 * @brief OpenCL 高程图核心类声明
 */

#ifndef ELEVATION_MAPPING_OPENCL__ELEVATION_MAP_OPENCL_HPP_
#define ELEVATION_MAPPING_OPENCL__ELEVATION_MAP_OPENCL_HPP_

#include <CL/cl.h>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <memory>

namespace elevation_mapping_opencl
{

/**
 * @brief 高程图配置参数
 */
struct ElevationMapConfig
{
  double resolution          = 0.05;   // 分辨率（米/像素）
  double map_length          = 10.0;   // 地图边长（米）

  // 传感器噪声模型
  double sensor_noise_factor = 0.0025; // 传感器噪声系数（方差 = factor × distance²）
  double mahalanobis_thresh  = 3.0;    // 离群点马氏距离阈值
  double outlier_variance    = 0.01;   // 离群点方差增量

  // 时间衰减
  double time_variance       = 0.0001; // 每帧方差增量（表示时间不确定性）
  double max_variance        = 1.0;    // 最大方差限制

  // 距离过滤
  double min_valid_distance  = 0.3;    // 最小有效距离（过滤机体遮挡）
  double max_height_range    = 5.0;    // 最大高度范围（米）

  // 可通行性
  double max_slope           = 0.5;    // 最大可通行坡度（弧度，约 28.6°）
  double max_step_height     = 0.15;   // 最大台阶高度（米）
  double slope_weight        = 0.5;    // 坡度权重
  double step_weight         = 0.5;    // 台阶权重
};

/**
 * @brief OpenCL 高程图核心类
 * 
 * 使用 Intel OpenCL 在 Arc GPU 上加速高程图构建
 * 图层布局（elevation_map_buffer_）：
 *   [0] elevation       - 高程（米）
 *   [1] variance        - 方差
 *   [2] is_valid        - 是否有效（0 或 1）
 *   [3] traversability  - 可通行性（存档用，实际来自独立缓冲区）
 *   [4] time            - 时间戳
 *   [5] upper_bound     - 上界高程
 *   [6] is_upper_bound  - 是否上界有效
 */
class ElevationMapOpenCL
{
public:
  // 图层索引常量
  static constexpr int NUM_LAYERS         = 7;
  static constexpr int LAYER_ELEVATION    = 0;
  static constexpr int LAYER_VARIANCE     = 1;
  static constexpr int LAYER_IS_VALID     = 2;
  static constexpr int LAYER_TRAVERSABILITY = 3;
  static constexpr int LAYER_TIME         = 4;
  static constexpr int LAYER_UPPER_BOUND  = 5;
  static constexpr int LAYER_IS_UPPER_BOUND = 6;

  explicit ElevationMapOpenCL(const ElevationMapConfig & config);
  ~ElevationMapOpenCL();

  // ---- 主要接口 ----
  void inputPointCloud(
    const std::vector<Eigen::Vector3f> & points,
    const Eigen::Matrix3f & R,
    const Eigen::Vector3f & t);

  void updateVariance();
  void computeNormals();
  void computeTraversability();

  void moveTo(const Eigen::Vector3f & position, const Eigen::Matrix3f & R);
  void clear();

  void getLayer(const std::string & name, std::vector<float> & data);
  void compensateDrift(float drift_offset);

  // ---- 状态查询 ----
  Eigen::Vector3f getCenter() const { return center_; }
  Eigen::Matrix3f getRotation() const { return base_rotation_; }
  double getResolution() const { return config_.resolution; }
  int getWidth() const { return width_; }
  int getHeight() const { return height_; }

  int gaussian_kernel_size_ = 3;   // 当前高斯核实际尺寸

  // ===== 重置后处理状态（每帧开始时） =====
  void resetPostProcessState() { post_process_state_.reset(); }

  // ---- 新增后处理接口 ----
  void smooth();                    // 高斯平滑
  void inpaint();                   // 空洞填补
  void computeErosion();            // 形态学侵蚀
  void cleanupVisibility(const Eigen::Vector3f & sensor_pos);  // 视野清理
  void applyMinFilter();            // 最小值滤波

  // ===== 侵蚀参数配置 =====
  struct ErosionConfig {
    int erosion_radius = 3;           // 侵蚀半径（像素）
    float safety_threshold = 0.3f;    // 安全阈值（traversability < 此值为危险）
    float erosion_strength = 0.7f;    // 侵蚀强度（0~1，越大侵蚀越强）
  } erosion_config_;

  // ===== 空洞参数配置 =====
  struct InpaintConfig {
  int max_iters = 4;                 //最大迭代次数
  float max_inpaint_distance = 1.0f;
  int kernel_radius = 2;
  } inpaint_config_;

private:
  void initializeOpenCL();
  void compileKernels();
  void allocateBuffers();

  ElevationMapConfig config_;
  Eigen::Vector3f center_;
  Eigen::Matrix3f base_rotation_;
  int width_;
  int height_;

  // ---- 新增 Kernel 对象 ----
  cl_kernel gaussian_smooth_kernel_;
  cl_kernel inpaint_kernel_;
  cl_kernel morphological_erosion_kernel_;
  cl_kernel visibility_cleanup_kernel_;
  cl_kernel min_filter_kernel_;
  cl_kernel drift_compensation_kernel_;

  // ---- 新增临时缓冲区（用于多步处理）----
  cl_mem erosion_buffer_;               // 侵蚀后的可通行性缓冲区

  // ===== 后处理缓冲区（双缓冲模式）=====
  cl_mem elevation_map_buffer_post_a_;    // 后处理缓冲 A
  cl_mem elevation_map_buffer_post_b_;    // 后处理缓冲 B

  // ===== 高斯核权重缓冲 =====
  cl_mem gaussian_kernel_buffer_;  // 存储高斯权重

  // ===== 后处理状态追踪 =====
  struct PostProcessState {
    bool smooth_applied = false;
    bool inpaint_applied = false;
    bool min_filter_applied = false;
    bool visibility_cleanup_applied = false;
    bool erosion_applied = false;
    
    void reset() {
      smooth_applied = false;
      inpaint_applied = false;
      min_filter_applied = false;
      visibility_cleanup_applied = false;
      erosion_applied = false;
    }
  } post_process_state_;

  // ===== 后处理辅助函数 =====
  void swapPostProcessBuffers();  // 交换双缓冲
  void copyPostProcessToMain();   // 将后处理结果复制回主缓冲

  // ===== 高斯核生成函数 =====
  void generateGaussianKernel(float sigma, int kernel_size);

  // OpenCL 核心对象
  cl_context       context_;
  cl_command_queue queue_;
  cl_program       program_;

  // Kernel 对象
  cl_kernel add_points_kernel_;
  cl_kernel update_variance_kernel_;
  cl_kernel compute_normals_kernel_;
  cl_kernel compute_traversability_kernel_;
  cl_kernel shift_map_kernel_;

  // GPU 缓冲区
  cl_mem elevation_map_buffer_;      // 主高程图 [NUM_LAYERS, H, W]
  cl_mem elevation_map_buffer_tmp_;  // 平移用临时缓冲区
  cl_mem normal_map_buffer_;         // 法向量 [3, H, W]
  cl_mem traversability_buffer_;     // 可通行性 [H, W]
};

}  // namespace elevation_mapping_opencl

#endif  // ELEVATION_MAPPING_OPENCL__ELEVATION_MAP_OPENCL_HPP_