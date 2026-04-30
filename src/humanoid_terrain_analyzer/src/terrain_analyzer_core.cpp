/**
 * @file terrain_analyzer_core.cpp
 * @brief OpenCL 加速地形分析核心实现
 */

#include "humanoid_terrain_analyzer/terrain_analyzer_core.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>

namespace humanoid_terrain_analyzer
{

// ===========================================================================
// 构造 / 析构
// ===========================================================================

TerrainAnalyzerCore::TerrainAnalyzerCore(const TerrainAnalyzerConfig & config)
: config_(config),
  context_(nullptr),
  queue_(nullptr),
  program_(nullptr),
  median_filter_kernel_(nullptr),
  residual_squared_kernel_(nullptr),
  roi_buffer_(nullptr),
  filtered_buffer_(nullptr),
  residual_buffer_(nullptr),
  device_(nullptr),
  buffer_rows_(0),
  buffer_cols_(0),
  buffers_allocated_(false)
{
  initializeOpenCL();
  compileKernels();
  std::cout << "[TerrainAnalyzerCore] 初始化完成" << std::endl;
}

TerrainAnalyzerCore::~TerrainAnalyzerCore()
{
  // 按创建的逆序释放资源
  releaseBuffers();

  if (median_filter_kernel_) {
    clReleaseKernel(median_filter_kernel_);
    median_filter_kernel_ = nullptr;
  }
  if (residual_squared_kernel_) {
    clReleaseKernel(residual_squared_kernel_);
    residual_squared_kernel_ = nullptr;
  }
  if (program_) {
    clReleaseProgram(program_);
    program_ = nullptr;
  }
  if (queue_) {
    clReleaseCommandQueue(queue_);
    queue_ = nullptr;
  }
  if (context_) {
    clReleaseContext(context_);
    context_ = nullptr;
  }

  std::cout << "[TerrainAnalyzerCore] 资源已释放" << std::endl;
}

// ===========================================================================
// OpenCL 初始化
// ===========================================================================

void TerrainAnalyzerCore::initializeOpenCL()
{
  cl_int err = CL_SUCCESS;
  // 使用成员变量 device_，不再声明局部变量

  // ----- 1. 枚举所有平台 -----
  cl_uint num_platforms = 0;
  err = clGetPlatformIDs(0, nullptr, &num_platforms);
  if (err != CL_SUCCESS || num_platforms == 0) {
    throw std::runtime_error("[TerrainAnalyzerCore] 未找到任何 OpenCL 平台");
  }

  std::vector<cl_platform_id> platforms(num_platforms);
  clGetPlatformIDs(num_platforms, platforms.data(), nullptr);

  // ----- 2. 优先选择 Intel 平台（Orin NX 上通常是集成显卡）-----
  cl_platform_id selected_platform = platforms[0];
  for (auto & p : platforms) {
    char name[256] = {0};
    clGetPlatformInfo(p, CL_PLATFORM_NAME, sizeof(name), name, nullptr);
    std::string name_str(name);
    if (name_str.find("Intel") != std::string::npos) {
      selected_platform = p;
      break;
    }
  }

  {
    char pname[256] = {0};
    clGetPlatformInfo(selected_platform, CL_PLATFORM_NAME, sizeof(pname), pname, nullptr);
    std::cout << "[TerrainAnalyzerCore] 使用 OpenCL 平台: " << pname << std::endl;
  }

  // ----- 3. 获取 GPU 设备，若无则回退到 CPU -----
  device_ = nullptr;
  cl_uint num_devices = 0;

  err = clGetDeviceIDs(selected_platform, CL_DEVICE_TYPE_GPU, 1, &device_, &num_devices);
  if (err != CL_SUCCESS || num_devices == 0) {
    std::cerr << "[TerrainAnalyzerCore] 警告：未找到 GPU，回退到 CPU 执行" << std::endl;
    err = clGetDeviceIDs(selected_platform, CL_DEVICE_TYPE_CPU, 1, &device_, &num_devices);
    if (err != CL_SUCCESS || num_devices == 0) {
      throw std::runtime_error("[TerrainAnalyzerCore] 未找到任何可用 OpenCL 设备");
    }
  }

  {
    char dname[256] = {0};
    clGetDeviceInfo(device_, CL_DEVICE_NAME, sizeof(dname), dname, nullptr);
    std::cout << "[TerrainAnalyzerCore] 使用设备: " << dname << std::endl;
  }

  // ----- 4. 创建上下文 -----
  context_ = clCreateContext(nullptr, 1, &device_, nullptr, nullptr, &err);
  if (err != CL_SUCCESS || !context_) {
    throw std::runtime_error("[TerrainAnalyzerCore] 创建 OpenCL 上下文失败");
  }

  // ----- 5. 创建命令队列 -----
  // CL_QUEUE_PROFILING_ENABLE 允许性能分析，可按需去掉
  queue_ = clCreateCommandQueue(context_, device_, CL_QUEUE_PROFILING_ENABLE, &err);
  if (err != CL_SUCCESS || !queue_) {
    throw std::runtime_error("[TerrainAnalyzerCore] 创建命令队列失败");
  }
}

// ===========================================================================
// Kernel 编译
// ===========================================================================

void TerrainAnalyzerCore::compileKernels()
{
  // 从安装目录加载 kernel 文件
  std::string pkg_dir = ament_index_cpp::get_package_share_directory(
    "humanoid_terrain_analyzer");
  std::string kernel_path = pkg_dir + "/kernels/terrain_analyzer.cl";

  std::ifstream file(kernel_path);
  if (!file.is_open()) {
    throw std::runtime_error(
      "[TerrainAnalyzerCore] 无法打开 kernel 文件: " + kernel_path);
  }

  std::stringstream ss;
  ss << file.rdbuf();
  std::string source = ss.str();

  const char * src  = source.c_str();
  size_t       size = source.size();

  cl_int err = CL_SUCCESS;

  // 创建程序对象
  program_ = clCreateProgramWithSource(context_, 1, &src, &size, &err);
  if (err != CL_SUCCESS || !program_) {
    throw std::runtime_error("[TerrainAnalyzerCore] 创建 OpenCL 程序失败");
  }

  // 编译
  // -cl-fast-relaxed-math 开启浮点快速数学，提升性能
  err = clBuildProgram(program_, 0, nullptr, "-cl-fast-relaxed-math", nullptr, nullptr);
  if (err != CL_SUCCESS) {
    // 获取编译日志并抛出
    size_t log_size = 0;
    clGetProgramBuildInfo(program_, device_, CL_PROGRAM_BUILD_LOG, 0, nullptr, &log_size);
    std::vector<char> log(log_size + 1, '\0');
    clGetProgramBuildInfo(program_, device_, CL_PROGRAM_BUILD_LOG, log_size, log.data(), nullptr);
    throw std::runtime_error(
      std::string("[TerrainAnalyzerCore] OpenCL kernel 编译失败:\n") + log.data());
  }

  // 创建各个 kernel 对象
  median_filter_kernel_ = clCreateKernel(program_, "median_filter_3x3", &err);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("[TerrainAnalyzerCore] 创建 median_filter_3x3 kernel 失败");
  }

  residual_squared_kernel_ = clCreateKernel(program_, "residual_squared", &err);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("[TerrainAnalyzerCore] 创建 residual_squared kernel 失败");
  }

  std::cout << "[TerrainAnalyzerCore] kernel 编译成功" << std::endl;
}

// ===========================================================================
// 缓冲区管理
// ===========================================================================

void TerrainAnalyzerCore::allocateBuffers(int rows, int cols)
{
  // 如果尺寸没变，复用缓冲区
  if (buffers_allocated_ && rows == buffer_rows_ && cols == buffer_cols_) {
    return;
  }

  // 释放旧缓冲区
  releaseBuffers();

  buffer_rows_ = rows;
  buffer_cols_ = cols;

  size_t cell_count = static_cast<size_t>(rows) * static_cast<size_t>(cols);
  size_t bytes      = cell_count * sizeof(float);

  cl_int err = CL_SUCCESS;

  // ROI 原始数据缓冲区
  roi_buffer_ = clCreateBuffer(context_, CL_MEM_READ_WRITE, bytes, nullptr, &err);
  if (err != CL_SUCCESS || !roi_buffer_) {
    throw std::runtime_error("[TerrainAnalyzerCore] 分配 roi_buffer 失败");
  }

  // 中值滤波输出缓冲区
  filtered_buffer_ = clCreateBuffer(context_, CL_MEM_READ_WRITE, bytes, nullptr, &err);
  if (err != CL_SUCCESS || !filtered_buffer_) {
    throw std::runtime_error("[TerrainAnalyzerCore] 分配 filtered_buffer 失败");
  }

  // 残差平方缓冲区
  residual_buffer_ = clCreateBuffer(context_, CL_MEM_READ_WRITE, bytes, nullptr, &err);
  if (err != CL_SUCCESS || !residual_buffer_) {
    throw std::runtime_error("[TerrainAnalyzerCore] 分配 residual_buffer 失败");
  }

  buffers_allocated_ = true;
  std::cout << "[TerrainAnalyzerCore] 缓冲区已分配: "
            << rows << "x" << cols << std::endl;
}

void TerrainAnalyzerCore::releaseBuffers()
{
  if (roi_buffer_) {
    clReleaseMemObject(roi_buffer_);
    roi_buffer_ = nullptr;
  }
  if (filtered_buffer_) {
    clReleaseMemObject(filtered_buffer_);
    filtered_buffer_ = nullptr;
  }
  if (residual_buffer_) {
    clReleaseMemObject(residual_buffer_);
    residual_buffer_ = nullptr;
  }

  buffers_allocated_ = false;
  buffer_rows_ = 0;
  buffer_cols_ = 0;
}

// ===========================================================================
// 辅助函数：百分位数（CPU）
// ===========================================================================

float TerrainAnalyzerCore::computePercentile(std::vector<float> values, float p) const
{
  if (values.empty()) {
    return 0.0f;
  }

  if (p <= 0.0f) {
    return *std::min_element(values.begin(), values.end());
  }
  if (p >= 1.0f) {
    return *std::max_element(values.begin(), values.end());
  }

  // nth_element 比完整排序快，O(n) 平均复杂度
  size_t k = static_cast<size_t>(p * static_cast<float>(values.size() - 1));
  std::nth_element(values.begin(), values.begin() + k, values.end());
  return values[k];
}

// ===========================================================================
// 辅助函数：最小二乘平面拟合（CPU 端，Eigen 求解）
// ===========================================================================

void TerrainAnalyzerCore::computePlaneCoefficients(
  const std::vector<float> & roi,
  int rows,
  int cols,
  const std::vector<uint8_t> & valid_mask,
  float resolution,
  float & a,
  float & b,
  float & c) const
{
  // 平面模型：z = a*x + b*y + c
  // 正规方程（3x3 对称矩阵）：
  //   [sum_x2  sum_xy  sum_x ] [a]   [sum_xz]
  //   [sum_xy  sum_y2  sum_y ] [b] = [sum_yz]
  //   [sum_x   sum_y   N     ] [c]   [sum_z ]

  double sum_x2 = 0.0, sum_xy = 0.0, sum_x  = 0.0;
  double sum_y2 = 0.0, sum_y  = 0.0;
  double sum_xz = 0.0, sum_yz = 0.0, sum_z  = 0.0;
  double N = 0.0;

  for (int r = 0; r < rows; ++r) {
    for (int col_idx = 0; col_idx < cols; ++col_idx) {
      int idx = r * cols + col_idx;
      if (!valid_mask[idx]) {
        continue;
      }

      float z_val = roi[idx];
      if (!std::isfinite(z_val)) {
        continue;
      }

      // 将坐标原点移到ROI中心，使x/y范围对称（数值计算更稳定）
      double x_center = (cols - 1) * 0.5 * resolution;
      double y_center = (rows - 1) * 0.5 * resolution;
      
      double x = static_cast<double>(col_idx) * resolution - x_center;
      double y = static_cast<double>(r) * resolution - y_center;
      double z = static_cast<double>(z_val);

      sum_x2 += x * x;
      sum_xy += x * y;
      sum_x  += x;
      sum_y2 += y * y;
      sum_y  += y;
      sum_xz += x * z;
      sum_yz += y * z;
      sum_z  += z;
      N      += 1.0;
    }
  }

  if (N < 3.0) {
    // 有效点太少，无法拟合
    a = 0.0f;
    b = 0.0f;
    c = 0.0f;
    return;
  }

  // 用 Eigen 构建并求解 3x3 线性系统
  Eigen::Matrix3d A;
  A << sum_x2, sum_xy, sum_x,
       sum_xy, sum_y2, sum_y,
       sum_x,  sum_y,  N;

  Eigen::Vector3d rhs;
  rhs << sum_xz, sum_yz, sum_z;

  // colPivHouseholderQr 比 LLT 更鲁棒，对近奇异矩阵也能给出合理结果
  Eigen::Vector3d x_sol = A.colPivHouseholderQr().solve(rhs);

  a = static_cast<float>(x_sol(0));
  b = static_cast<float>(x_sol(1));
  c = static_cast<float>(x_sol(2));
}

// ===========================================================================
// 核心分析函数
// ===========================================================================

TerrainAnalysisResult TerrainAnalyzerCore::analyze(
  const std::vector<float> & roi,
  int rows,
  int cols,
  float resolution)
{
  TerrainAnalysisResult result;

  // 安全检查
  if (roi.empty() || rows <= 0 || cols <= 0) {
    result.is_safe_to_step = false;
    return result;
  }

  const size_t cell_count = static_cast<size_t>(rows) * static_cast<size_t>(cols);
  const size_t bytes      = cell_count * sizeof(float);

  // =========================================================================
  // Step 1：生成有效性掩码，统计有效比例
  // =========================================================================
  std::vector<uint8_t> valid_mask(cell_count, 0);
  std::vector<float>   valid_values;
  valid_values.reserve(cell_count);

  for (size_t i = 0; i < cell_count; ++i) {
    if (std::isfinite(roi[i])) {
      valid_mask[i] = 1;
      valid_values.push_back(roi[i]);
    }
  }

  result.valid_ratio = static_cast<float>(valid_values.size()) /
                       static_cast<float>(cell_count);

  // 有效数据不足，直接返回不安全
  if (result.valid_ratio < config_.min_valid_data_ratio) {
    result.is_safe_to_step = false;
    return result;
  }

  // =========================================================================
  // Step 2：确保缓冲区已分配，上传 ROI 到 GPU
  // =========================================================================
  allocateBuffers(rows, cols);

  cl_int err = clEnqueueWriteBuffer(
    queue_, roi_buffer_, CL_TRUE,  // CL_TRUE = 阻塞写入，保证数据到达 GPU
    0, bytes, roi.data(),
    0, nullptr, nullptr);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("[TerrainAnalyzerCore] 上传 ROI 到 GPU 失败");
  }

  // =========================================================================
  // Step 3：GPU 中值滤波（去除雷达噪点）
  // =========================================================================
  int rows_i = rows;
  int cols_i = cols;

  clSetKernelArg(median_filter_kernel_, 0, sizeof(cl_mem), &roi_buffer_);
  clSetKernelArg(median_filter_kernel_, 1, sizeof(cl_mem), &filtered_buffer_);
  clSetKernelArg(median_filter_kernel_, 2, sizeof(int),    &rows_i);
  clSetKernelArg(median_filter_kernel_, 3, sizeof(int),    &cols_i);

  size_t global_work_size[2] = {
    static_cast<size_t>(rows),
    static_cast<size_t>(cols)
  };

  err = clEnqueueNDRangeKernel(
    queue_, median_filter_kernel_,
    2, nullptr, global_work_size, nullptr,
    0, nullptr, nullptr);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("[TerrainAnalyzerCore] 执行 median_filter_3x3 失败");
  }

  // 等待 GPU 完成滤波
  clFinish(queue_);

  // 把滤波结果读回 CPU
  std::vector<float> filtered(cell_count, std::numeric_limits<float>::quiet_NaN());
  err = clEnqueueReadBuffer(
    queue_, filtered_buffer_, CL_TRUE,
    0, bytes, filtered.data(),
    0, nullptr, nullptr);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("[TerrainAnalyzerCore] 读取滤波结果失败");
  }

  // =========================================================================
  // Step 4：CPU 计算台阶高度（P95 - P05）
  //   使用滤波后数据，去除孤立噪点的干扰
  // =========================================================================
  std::vector<float> filtered_valid;
  filtered_valid.reserve(cell_count);
  for (float v : filtered) {
    if (std::isfinite(v)) {
      filtered_valid.push_back(v);
    }
  }

  if (filtered_valid.empty()) {
    result.is_safe_to_step = false;
    return result;
  }

  // 一次排序计算多个百分位（比多次nth_element更高效）
  std::sort(filtered_valid.begin(), filtered_valid.end());
  float p05 = filtered_valid[static_cast<size_t>(0.05f * (filtered_valid.size() - 1))];
  float p95 = filtered_valid[static_cast<size_t>(0.95f * (filtered_valid.size() - 1))];
  result.max_step_height = p95 - p05;

  // =========================================================================
  // Step 5：CPU 平面拟合，计算坡度
  //   使用滤波后的有效掩码（重新生成，基于 filtered 数据）
  // =========================================================================
  std::vector<uint8_t> filtered_mask(cell_count, 0);
  for (size_t i = 0; i < cell_count; ++i) {
    if (std::isfinite(filtered[i])) {
      filtered_mask[i] = 1;
    }
  }

  float a = 0.0f, b = 0.0f, c_plane = 0.0f;
  computePlaneCoefficients(filtered, rows, cols, filtered_mask, resolution,
                           a, b, c_plane);

  // 坡度 = atan(|gradient|)，gradient = sqrt(a^2 + b^2)
  result.avg_slope = std::atan(std::sqrt(a * a + b * b));

  // =========================================================================
  // Step 6：GPU 计算残差平方（粗糙度原材料）
  //   把平面参数传给 GPU，让 GPU 并行算每个像素的 residual^2
  // =========================================================================
  clSetKernelArg(residual_squared_kernel_, 0, sizeof(cl_mem), &filtered_buffer_);
  clSetKernelArg(residual_squared_kernel_, 1, sizeof(cl_mem), &residual_buffer_);
  clSetKernelArg(residual_squared_kernel_, 2, sizeof(int),    &rows_i);
  clSetKernelArg(residual_squared_kernel_, 3, sizeof(int),    &cols_i);
  clSetKernelArg(residual_squared_kernel_, 4, sizeof(float),  &a);
  clSetKernelArg(residual_squared_kernel_, 5, sizeof(float),  &b);
  clSetKernelArg(residual_squared_kernel_, 6, sizeof(float),  &c_plane);
  clSetKernelArg(residual_squared_kernel_, 7, sizeof(float),  &resolution);

  err = clEnqueueNDRangeKernel(
    queue_, residual_squared_kernel_,
    2, nullptr, global_work_size, nullptr,
    0, nullptr, nullptr);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("[TerrainAnalyzerCore] 执行 residual_squared 失败");
  }

  clFinish(queue_);

  // 读回残差平方
  std::vector<float> residuals(cell_count, std::numeric_limits<float>::quiet_NaN());
  err = clEnqueueReadBuffer(
    queue_, residual_buffer_, CL_TRUE,
    0, bytes, residuals.data(),
    0, nullptr, nullptr);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("[TerrainAnalyzerCore] 读取残差结果失败");
  }

  // =========================================================================
  // Step 7：CPU 聚合残差，计算 RMS 粗糙度
  // =========================================================================
  double sum_res  = 0.0;
  int    res_count = 0;
  for (float v : residuals) {
    if (std::isfinite(v)) {
      sum_res += static_cast<double>(v);
      ++res_count;
    }
  }

  if (res_count > 0) {
    result.roughness = std::sqrt(static_cast<float>(sum_res / res_count));
  } else {
    result.roughness = 0.0f;
  }

  // =========================================================================
  // Step 8：综合安全性判定
  //   所有条件都满足才认为安全
  // =========================================================================
  result.is_safe_to_step =
    (result.max_step_height <= config_.fatal_obstacle_height) &&
    (result.avg_slope       <= config_.safe_slope_threshold)  &&
    (result.roughness       <= config_.roughness_threshold)   &&
    (result.valid_ratio     >= config_.min_valid_data_ratio);

  return result;
}

}  // namespace humanoid_terrain_analyzer