#pragma once

/**
 * @file terrain_analyzer_core.hpp
 * @brief OpenCL 加速的地形分析核心类头文件
 *
 * 职责：
 *   - 接收 ROI 数据（二维高程格栅）
 *   - 在 GPU 上做中值滤波、残差平方计算
 *   - 在 CPU 上做平面拟合、台阶高度、安全性判定
 *   - 返回 TerrainAnalysisResult
 *
 * 设计原则：
 *   - 不依赖任何 ROS 类型，只处理原始 float 数组
 *   - 不负责消息订阅/发布，只负责计算
 *   - 可在非 ROS 环境下单独测试
 */

#include <CL/cl.h>
#include <Eigen/Dense>
#include <cstdint>
#include <string>
#include <vector>

namespace humanoid_terrain_analyzer
{

/**
 * @brief 地形分析结果结构体
 */
struct TerrainAnalysisResult
{
  float max_step_height = 0.0f;   ///< 最大台阶高度（米），95 分位数 - 5 分位数
  float avg_slope       = 0.0f;   ///< 平均坡度（弧度），由平面拟合得到
  float roughness       = 0.0f;   ///< 粗糙度，拟合残差的 RMS 值
  float valid_ratio     = 0.0f;   ///< ROI 内有效数据点比例 [0, 1]
  bool  is_safe_to_step = false;  ///< 综合安全判定结果
};

/**
 * @brief 地形分析配置参数
 *
 * 所有参数都可以从 ROS 参数服务器读取，
 * 由 TerrainAnalyzerNode 负责填充后传入 TerrainAnalyzerCore。
 */
struct TerrainAnalyzerConfig
{
  // ===== ROI 范围（相对于机器人 base_link，单位：米）=====
  float roi_x_min   = 0.3f;    ///< 前方最近距离（米）
  float roi_x_max   = 1.0f;    ///< 前方最远距离（米）
  float roi_y_width = 0.6f;    ///< 横向宽度（米）

  // ===== 安全阈值 =====
  float fatal_obstacle_height = 0.35f;  ///< 最大允许台阶高度（米）
  float min_valid_data_ratio  = 0.4f;   ///< 最小有效数据比例，低于此值不分析
  float safe_slope_threshold  = 0.35f;  ///< 最大允许坡度（弧度，约 20°）
  float roughness_threshold   = 0.08f;  ///< 最大允许粗糙度（米）
};

/**
 * @brief 基于 OpenCL 的地形分析核心处理类
 *
 * 使用方式：
 *   1. 构造时传入配置
 *   2. 每帧调用 analyze() 传入 ROI 数据
 *   3. 接收返回的 TerrainAnalysisResult
 */
class TerrainAnalyzerCore
{
public:
  /**
   * @brief 构造函数，初始化 OpenCL 环境并编译 kernels
   * @param config 分析配置
   * @throws std::runtime_error 如果 OpenCL 初始化或编译失败
   */
  explicit TerrainAnalyzerCore(const TerrainAnalyzerConfig & config);

  /**
   * @brief 析构函数，释放所有 OpenCL 资源
   */
  ~TerrainAnalyzerCore();

  /**
   * @brief 分析 ROI 区域
   * @param roi    输入 ROI 数据，行优先存储，NaN 表示无效
   * @param rows   ROI 行数
   * @param cols   ROI 列数
   * @param resolution 栅格分辨率（米/格）
   * @return TerrainAnalysisResult 分析结果
   */
  TerrainAnalysisResult analyze(
    const std::vector<float> & roi,
    int rows,
    int cols,
    float resolution);

private:
  // ===== OpenCL 初始化与清理 =====

  /// 初始化 OpenCL 平台、设备、上下文、命令队列
  void initializeOpenCL();

  /// 从文件加载并编译 kernel 源码
  void compileKernels();

  /// 按需分配/重新分配 GPU 缓冲区
  void allocateBuffers(int rows, int cols);

  /// 释放所有 GPU 缓冲区
  void releaseBuffers();

  // ===== 辅助计算（CPU 侧）=====

  /**
   * @brief 计算有序数组的百分位数
   * @param values 数据，会在内部排序（传值，不影响外部）
   * @param p      百分位，[0, 1]
   */
  float computePercentile(std::vector<float> values, float p) const;

  /**
   * @brief 用最小二乘法拟合平面 z = a*x + b*y + c
   * @param roi        经过滤波后的 ROI 数据
   * @param rows/cols  ROI 尺寸
   * @param valid_mask 有效性掩码（1=有效，0=无效）
   * @param resolution 分辨率
   * @param a, b, c    输出：平面参数
   */
  void computePlaneCoefficients(
    const std::vector<float> & roi,
    int rows,
    int cols,
    const std::vector<uint8_t> & valid_mask,
    float resolution,
    float & a,
    float & b,
    float & c) const;

  // ===== 配置 =====
  TerrainAnalyzerConfig config_;

  // ===== OpenCL 资源 =====
  cl_context       context_;
  cl_command_queue queue_;
  cl_program       program_;

  cl_kernel median_filter_kernel_;    ///< 3x3 中值滤波 kernel
  cl_kernel residual_squared_kernel_; ///< 残差平方 kernel

  cl_mem roi_buffer_;       ///< GPU 上的 ROI 原始数据
  cl_mem filtered_buffer_;  ///< GPU 上的滤波后数据
  cl_mem residual_buffer_;  ///< GPU 上的残差平方数据

  cl_device_id device_;

  // ===== 缓冲区状态 =====
  int  buffer_rows_;       ///< 当前已分配缓冲区的行数
  int  buffer_cols_;       ///< 当前已分配缓冲区的列数
  bool buffers_allocated_; ///< 缓冲区是否已分配
};

}  // namespace humanoid_terrain_analyzer