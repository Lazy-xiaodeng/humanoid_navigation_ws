/**
 * @file elevation_map_opencl.cpp
 * @brief OpenCL 高程图核心类实现
 * @date 2024
 * 
 * 功能：
 * 1. 管理 OpenCL 上下文和 GPU 缓冲区
 * 2. 调用 GPU kernel 进行点云融合
 * 3. 计算法线和可通行性
 * 4. 支持地图中心移动（跟随机器人）
 */

#include "elevation_mapping_opencl/elevation_map_opencl.hpp"
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace elevation_mapping_opencl
{

// =====================================================================
// 构造函数
// =====================================================================
ElevationMapOpenCL::ElevationMapOpenCL(const ElevationMapConfig & config)
: config_(config),
  center_(Eigen::Vector3f::Zero()),
  base_rotation_(Eigen::Matrix3f::Identity()),
  context_(nullptr),
  queue_(nullptr),
  program_(nullptr),
  add_points_kernel_(nullptr),
  update_variance_kernel_(nullptr),
  compute_normals_kernel_(nullptr),
  compute_traversability_kernel_(nullptr),
  shift_map_kernel_(nullptr),
  elevation_map_buffer_(nullptr),
  elevation_map_buffer_tmp_(nullptr),
  normal_map_buffer_(nullptr),
  traversability_buffer_(nullptr),
  drift_compensation_kernel_(nullptr),
  gaussian_smooth_kernel_(nullptr),
  inpaint_kernel_(nullptr),
  morphological_erosion_kernel_(nullptr),
  visibility_cleanup_kernel_(nullptr),
  min_filter_kernel_(nullptr),
  erosion_buffer_(nullptr),
  elevation_map_buffer_post_a_(nullptr),
  elevation_map_buffer_post_b_(nullptr),
  gaussian_kernel_buffer_(nullptr),      
  gaussian_kernel_size_(3) 

{
  // ===== 计算地图尺寸（像素）=====
  // 确保为偶数（便于中心对齐计算）
  width_ = static_cast<int>(std::ceil(config_.map_length / config_.resolution));
  if (width_ % 2 != 0) width_++;
  height_ = width_;

  std::cout << "[ElevationMapOpenCL] 开始初始化..." << std::endl;
  std::cout << "  地图尺寸: " << width_ << " x " << height_ << " 像素" << std::endl;
  std::cout << "  分辨率: " << config_.resolution << " 米/像素" << std::endl;
  std::cout << "  物理尺寸: " << (width_ * config_.resolution) 
            << " x " << (height_ * config_.resolution) << " 米" << std::endl;

  // ===== 初始化 OpenCL =====
  initializeOpenCL();
  compileKernels();
  allocateBuffers();

  std::cout << "[ElevationMapOpenCL] 初始化完成 ✅" << std::endl;
}

// =====================================================================
// 析构函数（释放所有 OpenCL 资源）
// =====================================================================
ElevationMapOpenCL::~ElevationMapOpenCL()
{
  // 按创建的逆序释放资源
  if (gaussian_smooth_kernel_) { clReleaseKernel(gaussian_smooth_kernel_); gaussian_smooth_kernel_ = nullptr; }
  if (inpaint_kernel_) { clReleaseKernel(inpaint_kernel_); inpaint_kernel_= nullptr; }
  if (morphological_erosion_kernel_) { clReleaseKernel(morphological_erosion_kernel_); morphological_erosion_kernel_ = nullptr; }
  if (visibility_cleanup_kernel_) { clReleaseKernel(visibility_cleanup_kernel_); visibility_cleanup_kernel_ = nullptr; }
  if (min_filter_kernel_) { clReleaseKernel(min_filter_kernel_); min_filter_kernel_ = nullptr; }
  if (add_points_kernel_) { clReleaseKernel(add_points_kernel_); add_points_kernel_ = nullptr; }
  if (update_variance_kernel_) { clReleaseKernel(update_variance_kernel_); update_variance_kernel_ = nullptr; }
  if (compute_normals_kernel_) { clReleaseKernel(compute_normals_kernel_); compute_normals_kernel_ = nullptr; }
  if (compute_traversability_kernel_) { clReleaseKernel(compute_traversability_kernel_); compute_traversability_kernel_ = nullptr; }
  if (shift_map_kernel_) { clReleaseKernel(shift_map_kernel_); shift_map_kernel_ = nullptr; }
  if (drift_compensation_kernel_) { clReleaseKernel(drift_compensation_kernel_); drift_compensation_kernel_ = nullptr; }

  if (erosion_buffer_) { clReleaseMemObject(erosion_buffer_); erosion_buffer_ = nullptr; }
  if (elevation_map_buffer_) { clReleaseMemObject(elevation_map_buffer_); elevation_map_buffer_ = nullptr; }
  if (elevation_map_buffer_tmp_) { clReleaseMemObject(elevation_map_buffer_tmp_); elevation_map_buffer_tmp_ = nullptr; }
  if (normal_map_buffer_) { clReleaseMemObject(normal_map_buffer_); normal_map_buffer_ = nullptr; }
  if (traversability_buffer_) { clReleaseMemObject(traversability_buffer_); traversability_buffer_ = nullptr; }
  if (gaussian_kernel_buffer_) { clReleaseMemObject(gaussian_kernel_buffer_); gaussian_kernel_buffer_ = nullptr; }
  if (elevation_map_buffer_post_a_) { clReleaseMemObject(elevation_map_buffer_post_a_); elevation_map_buffer_post_a_ = nullptr; }
  if (elevation_map_buffer_post_b_) { clReleaseMemObject(elevation_map_buffer_post_b_); elevation_map_buffer_post_b_ = nullptr; }

  if (program_) { clReleaseProgram(program_); program_ = nullptr; }
  if (queue_) { clReleaseCommandQueue(queue_); queue_ = nullptr; }
  if (context_) { clReleaseContext(context_); context_ = nullptr; }

  std::cout << "[ElevationMapOpenCL] 所有 GPU 资源已释放 ✅" << std::endl;
}

// =====================================================================
// 初始化 OpenCL（平台 → 设备 → 上下文 → 命令队列）
// =====================================================================
void ElevationMapOpenCL::initializeOpenCL()
{
  cl_int err;

  // ===== 步骤 1：枚举所有平台，优先选择 Intel 平台 =====
  cl_uint num_platforms = 0;
  clGetPlatformIDs(0, nullptr, &num_platforms);
  if (num_platforms == 0) {
    throw std::runtime_error("❌ 未找到任何 OpenCL 平台！请检查驱动安装。");
  }

  std::vector<cl_platform_id> platforms(num_platforms);
  clGetPlatformIDs(num_platforms, platforms.data(), nullptr);

  // 优先选择 Intel 平台
  cl_platform_id selected_platform = platforms[0];
  for (auto & plat : platforms) {
    char plat_name[128];
    clGetPlatformInfo(plat, CL_PLATFORM_NAME, sizeof(plat_name), plat_name, nullptr);
    std::string name_str(plat_name);
    if (name_str.find("Intel") != std::string::npos) {
      selected_platform = plat;
      break;
    }
  }

  char platform_name[128];
  clGetPlatformInfo(selected_platform, CL_PLATFORM_NAME, sizeof(platform_name), platform_name, nullptr);
  std::cout << "[OpenCL] 选择平台: " << platform_name << std::endl;

  // ===== 步骤 2：获取 GPU 设备（失败则回退到 CPU）=====
  cl_device_id device;
  cl_uint num_devices = 0;

  err = clGetDeviceIDs(selected_platform, CL_DEVICE_TYPE_GPU, 1, &device, &num_devices);
  if (err != CL_SUCCESS || num_devices == 0) {
    std::cerr << "⚠️  未找到 GPU，回退到 CPU 模式..." << std::endl;
    err = clGetDeviceIDs(selected_platform, CL_DEVICE_TYPE_CPU, 1, &device, &num_devices);
    if (err != CL_SUCCESS || num_devices == 0) {
      throw std::runtime_error("❌ 未找到可用的 OpenCL 设备！");
    }
  }

  // 打印设备详细信息
  char device_name[256];
  cl_ulong global_mem_size;
  cl_ulong local_mem_size;
  cl_uint compute_units;
  size_t max_workgroup_size;

  clGetDeviceInfo(device, CL_DEVICE_NAME, sizeof(device_name), device_name, nullptr);
  clGetDeviceInfo(device, CL_DEVICE_GLOBAL_MEM_SIZE, sizeof(global_mem_size), &global_mem_size, nullptr);
  clGetDeviceInfo(device, CL_DEVICE_LOCAL_MEM_SIZE, sizeof(local_mem_size), &local_mem_size, nullptr);
  clGetDeviceInfo(device, CL_DEVICE_MAX_COMPUTE_UNITS, sizeof(compute_units), &compute_units, nullptr);
  clGetDeviceInfo(device, CL_DEVICE_MAX_WORK_GROUP_SIZE, sizeof(max_workgroup_size), &max_workgroup_size, nullptr);

  std::cout << "[OpenCL] 设备信息：" << std::endl;
  std::cout << "  名称: " << device_name << std::endl;
  std::cout << "  显存: " << (global_mem_size / 1024 / 1024) << " MB" << std::endl;
  std::cout << "  本地内存: " << (local_mem_size / 1024) << " KB" << std::endl;
  std::cout << "  计算单元: " << compute_units << std::endl;
  std::cout << "  最大工作组: " << max_workgroup_size << std::endl;

  // ===== 步骤 3：创建上下文 =====
  context_ = clCreateContext(nullptr, 1, &device, nullptr, nullptr, &err);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("❌ 创建 OpenCL 上下文失败！错误码: " + std::to_string(err));
  }

  // ===== 步骤 4：创建命令队列（支持性能分析）=====
  queue_ = clCreateCommandQueue(context_, device, CL_QUEUE_PROFILING_ENABLE, &err);
  if (err != CL_SUCCESS) {
    // 不带 profiling 再试
    queue_ = clCreateCommandQueue(context_, device, 0, &err);
    if (err != CL_SUCCESS) {
      throw std::runtime_error("❌ 创建 OpenCL 命令队列失败！错误码: " + std::to_string(err));
    }
  }

  std::cout << "[OpenCL] 上下文和命令队列创建成功 ✅" << std::endl;
}

// =====================================================================
// 编译 OpenCL Kernels
// =====================================================================
void ElevationMapOpenCL::compileKernels()
{
  std::cout << "[OpenCL] 开始编译 kernels..." << std::endl;

  // ===== 步骤 1：定位 kernel 文件 =====
  // 使用 ament_index 获取包的共享目录（ROS2 标准方式）
  std::string kernel_path;
  try {
    std::string pkg_dir = ament_index_cpp::get_package_share_directory("elevation_mapping_opencl");
    kernel_path = pkg_dir + "/kernels/elevation_kernels.cl";
  } catch (const std::exception & e) {
    // 备用路径（开发时使用）
    kernel_path = "elevation_kernels.cl";
    std::cerr << "⚠️  无法获取包路径，尝试当前目录: " << kernel_path << std::endl;
  }

  // ===== 步骤 2：读取 kernel 源码 =====
  std::ifstream kernel_file(kernel_path);
  if (!kernel_file.is_open()) {
    throw std::runtime_error(
      "❌ 无法打开 kernel 文件: " + kernel_path +
      "\n请确认文件存在，或在 CMakeLists.txt 中正确配置 install。");
  }

  std::stringstream ss;
  ss << kernel_file.rdbuf();
  std::string kernel_source = ss.str();
  kernel_file.close();

  const char* source_ptr = kernel_source.c_str();
  size_t source_size = kernel_source.size();

  // ===== 步骤 3：创建程序对象 =====
  cl_int err;
  program_ = clCreateProgramWithSource(context_, 1, &source_ptr, &source_size, &err);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("❌ 创建 OpenCL 程序对象失败！");
  }

  // ===== 步骤 4：编译程序 =====
  // 编译选项：-cl-fast-relaxed-math 允许快速浮点运算
  const char* build_options = "-cl-fast-relaxed-math";
  err = clBuildProgram(program_, 0, nullptr, build_options, nullptr, nullptr);
  if (err != CL_SUCCESS) {
    // 获取设备 ID（用于查询编译日志）
    size_t num_devices_ret;
    clGetProgramInfo(program_, CL_PROGRAM_NUM_DEVICES, sizeof(size_t), &num_devices_ret, nullptr);
    std::vector<cl_device_id> devices(num_devices_ret);
    clGetProgramInfo(program_, CL_PROGRAM_DEVICES,
      num_devices_ret * sizeof(cl_device_id), devices.data(), nullptr);

    // 打印编译日志
    size_t log_size;
    clGetProgramBuildInfo(program_, devices[0], CL_PROGRAM_BUILD_LOG, 0, nullptr, &log_size);
    std::vector<char> log(log_size + 1, '\0');
    clGetProgramBuildInfo(program_, devices[0], CL_PROGRAM_BUILD_LOG, log_size, log.data(), nullptr);

    std::cerr << "❌ OpenCL 编译错误:\n" << log.data() << std::endl;
    throw std::runtime_error("OpenCL kernel 编译失败！");
  }

  // ===== 步骤 5：创建各 kernel 对象 =====
  add_points_kernel_ = clCreateKernel(program_, "add_points_kernel", &err);
  if (err != CL_SUCCESS) throw std::runtime_error("❌ 创建 add_points_kernel 失败！");

  update_variance_kernel_ = clCreateKernel(program_, "update_variance_kernel", &err);
  if (err != CL_SUCCESS) throw std::runtime_error("❌ 创建 update_variance_kernel 失败！");

  compute_normals_kernel_ = clCreateKernel(program_, "compute_normals_kernel", &err);
  if (err != CL_SUCCESS) throw std::runtime_error("❌ 创建 compute_normals_kernel 失败！");

  compute_traversability_kernel_ = clCreateKernel(program_, "compute_traversability_kernel", &err);
  if (err != CL_SUCCESS) throw std::runtime_error("❌ 创建 compute_traversability_kernel 失败！");

  shift_map_kernel_ = clCreateKernel(program_, "shift_map_kernel", &err);
  if (err != CL_SUCCESS) throw std::runtime_error("❌ 创建 shift_map_kernel 失败！");

  gaussian_smooth_kernel_ = clCreateKernel(program_, "gaussian_smooth_kernel", &err);
  if (err != CL_SUCCESS) throw std::runtime_error("❌ 创建 gaussian_smooth_kernel 失败！");

  inpaint_kernel_ = clCreateKernel(program_, "inpaint_kernel", &err);
  if (err != CL_SUCCESS) throw std::runtime_error("❌ 创建 inpaint_kernel 失败！");

  morphological_erosion_kernel_ = clCreateKernel(program_, "morphological_erosion_kernel", &err);
  if (err != CL_SUCCESS) throw std::runtime_error("❌ 创建 morphological_erosion_kernel 失败！");

  visibility_cleanup_kernel_ = clCreateKernel(program_, "visibility_cleanup_kernel", &err);
  if (err != CL_SUCCESS) throw std::runtime_error("❌ 创建 visibility_cleanup_kernel 失败！");

  min_filter_kernel_ = clCreateKernel(program_, "min_filter_kernel", &err);
  if (err != CL_SUCCESS) throw std::runtime_error("❌ 创建 min_filter_kernel 失败！");

  std::cout << "[OpenCL] 所有kernels 编译成功 ✅" << std::endl;
}

  

// =====================================================================
// 分配 GPU 缓冲区
// =====================================================================
void ElevationMapOpenCL::allocateBuffers()
{
  cl_int err;

  // 计算各缓冲区字节数
  size_t elevation_map_bytes = NUM_LAYERS * width_ * height_ * sizeof(float);
  size_t normal_map_bytes    = 3 * width_ * height_ * sizeof(float);
  size_t traversability_bytes = width_ * height_ * sizeof(float);

  std::cout << "[OpenCL] 分配 GPU 缓冲区..." << std::endl;
  std::cout << "  高程图（" << NUM_LAYERS << " 层）: "
            << (elevation_map_bytes / 1024 / 1024) << " MB" << std::endl;
  std::cout << "  法向量图: "
            << (normal_map_bytes / 1024 / 1024) << " MB" << std::endl;
  std::cout << "  可通行性图: "
            << (traversability_bytes / 1024) << " KB" << std::endl;

  // 主高程图缓冲区
  elevation_map_buffer_ = clCreateBuffer(
    context_, CL_MEM_READ_WRITE, elevation_map_bytes, nullptr, &err);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("❌ 分配 elevation_map_buffer 失败！可能显存不足。");
  }

  // 用于地图平移的临时缓冲区
  elevation_map_buffer_tmp_ = clCreateBuffer(
    context_, CL_MEM_READ_WRITE, elevation_map_bytes, nullptr, &err);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("❌ 分配 elevation_map_buffer_tmp 失败！");
  }

  // 法向量缓冲区
  normal_map_buffer_ = clCreateBuffer(
    context_, CL_MEM_READ_WRITE, normal_map_bytes, nullptr, &err);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("❌ 分配 normal_map_buffer 失败！");
  }

  // 可通行性缓冲区
  traversability_buffer_ = clCreateBuffer(
    context_, CL_MEM_READ_WRITE, traversability_bytes, nullptr, &err);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("❌ 分配 traversability_buffer 失败！");
  }

  // 侵蚀后的可通行性缓冲区
  erosion_buffer_ = clCreateBuffer(
    context_, CL_MEM_READ_WRITE, traversability_bytes, nullptr, &err);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("❌ 分配 erosion_buffer 失败！");
  }

  // ===== 初始化高程图为默认值 =====
  // 所有高程和有效标志为 0，方差初始化为较大值
  std::vector<float> initial_map(NUM_LAYERS * width_ * height_, 0.0f);
  for (int i = 0; i < width_ * height_; ++i) {
    // 图层 1（方差）初始化为 10.0（表示不确定）
    initial_map[LAYER_VARIANCE * width_ * height_ + i] = 10.0f;
  }

  err = clEnqueueWriteBuffer(
    queue_, elevation_map_buffer_, CL_TRUE, 0,
    elevation_map_bytes, initial_map.data(), 0, nullptr, nullptr);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("❌ 初始化高程图缓冲区失败！");
  }

  // 后处理缓冲 A
  elevation_map_buffer_post_a_ = clCreateBuffer(
    context_, CL_MEM_READ_WRITE, elevation_map_bytes, nullptr, &err);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("❌ 分配 elevation_map_buffer_post_a 失败！");
  }

  // 后处理缓冲 B
  elevation_map_buffer_post_b_ = clCreateBuffer(
    context_, CL_MEM_READ_WRITE, elevation_map_bytes, nullptr, &err);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("❌ 分配 elevation_map_buffer_post_b 失败！");
  }

  // 生成默认高斯核（sigma=1.0, 3x3）
  generateGaussianKernel(1.0f, 3);
  
  std::cout << "[OpenCL] GPU 缓冲区分配完成 ✅" << std::endl;
}

// =====================================================================
// 点云融合（主要接口）
// =====================================================================
void ElevationMapOpenCL::inputPointCloud(
  const std::vector<Eigen::Vector3f> & points,
  const Eigen::Matrix3f & R,
  const Eigen::Vector3f & t)
{
  if (points.empty()) {
    std::cerr << "[ElevationMapOpenCL] 输入点云为空，跳过" << std::endl;
    return;
  }

  size_t N = points.size();

  // ===== 步骤 1：将 Eigen 数据拍平为连续的 float 数组 =====
  std::vector<float> points_flat(N * 3);
  for (size_t i = 0; i < N; ++i) {
    points_flat[i * 3 + 0] = points[i].x();
    points_flat[i * 3 + 1] = points[i].y();
    points_flat[i * 3 + 2] = points[i].z();
  }

  // 旋转矩阵（行优先）
  std::vector<float> R_flat(9);
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      R_flat[r * 3 + c] = R(r, c);
    }
  }

  // 平移向量
  std::vector<float> t_flat = {t.x(), t.y(), t.z()};

  // ===== 步骤 2：创建临时 GPU 缓冲区 =====
  cl_int err;

  cl_mem points_buf = clCreateBuffer(
    context_,
    CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
    points_flat.size() * sizeof(float),
    points_flat.data(), &err);
  if (err != CL_SUCCESS) throw std::runtime_error("❌ 上传点云数据失败！");

  cl_mem R_buf = clCreateBuffer(
    context_,
    CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
    R_flat.size() * sizeof(float),
    R_flat.data(), &err);
  if (err != CL_SUCCESS) throw std::runtime_error("❌ 上传旋转矩阵失败！");

  cl_mem t_buf = clCreateBuffer(
    context_,
    CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
    t_flat.size() * sizeof(float),
    t_flat.data(), &err);
  if (err != CL_SUCCESS) throw std::runtime_error("❌ 上传平移向量失败！");

  // ===== 步骤 3：设置 kernel 参数 =====
  float sensor_noise_factor_f = static_cast<float>(config_.sensor_noise_factor);
  float mahalanobis_thresh_f  = static_cast<float>(config_.mahalanobis_thresh);
  float outlier_variance_f    = static_cast<float>(config_.outlier_variance);
  float min_valid_distance_f  = static_cast<float>(config_.min_valid_distance);
  float max_height_range_f    = static_cast<float>(config_.max_height_range);
  float resolution_f          = static_cast<float>(config_.resolution);
  int width_i  = width_;
  int height_i = height_;
  float center_x = center_.x();
  float center_y = center_.y();

  clSetKernelArg(add_points_kernel_, 0, sizeof(cl_mem), &elevation_map_buffer_);
  clSetKernelArg(add_points_kernel_, 1, sizeof(cl_mem), &points_buf);
  clSetKernelArg(add_points_kernel_, 2, sizeof(cl_mem), &R_buf);
  clSetKernelArg(add_points_kernel_, 3, sizeof(cl_mem), &t_buf);
  clSetKernelArg(add_points_kernel_, 4, sizeof(float),  &resolution_f);
  clSetKernelArg(add_points_kernel_, 5, sizeof(int),    &width_i);
  clSetKernelArg(add_points_kernel_, 6, sizeof(int),    &height_i);
  clSetKernelArg(add_points_kernel_, 7,  sizeof(float),  &center_x);  // ← 新增
  clSetKernelArg(add_points_kernel_, 8,  sizeof(float),  &center_y);  // ← 新增
  clSetKernelArg(add_points_kernel_, 9, sizeof(float),  &sensor_noise_factor_f);
  clSetKernelArg(add_points_kernel_, 10, sizeof(float),  &mahalanobis_thresh_f);
  clSetKernelArg(add_points_kernel_, 11, sizeof(float),  &outlier_variance_f);
  clSetKernelArg(add_points_kernel_, 12, sizeof(float), &min_valid_distance_f);
  clSetKernelArg(add_points_kernel_, 13, sizeof(float), &max_height_range_f);

  // ===== 步骤 4：执行 kernel =====
  size_t global_work_size = N;
  err = clEnqueueNDRangeKernel(
    queue_, add_points_kernel_,
    1,               // 1D 工作空间
    nullptr,         // 全局偏移
    &global_work_size,
    nullptr,         // 自动选择本地工作组大小
    0, nullptr, nullptr);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("❌ 执行 add_points_kernel 失败！错误码: " + std::to_string(err));
  }

  // 等待 kernel 执行完成
  clFinish(queue_);

  // ===== 步骤 5：释放临时缓冲区 =====
  clReleaseMemObject(points_buf);
  clReleaseMemObject(R_buf);
  clReleaseMemObject(t_buf);
}

// =====================================================================
// 更新方差（随时间衰减）
// =====================================================================
void ElevationMapOpenCL::updateVariance()
{
  float time_variance_f = static_cast<float>(config_.time_variance);
  float max_variance_f  = static_cast<float>(config_.max_variance);
  int width_i  = width_;
  int height_i = height_;

  clSetKernelArg(update_variance_kernel_, 0, sizeof(cl_mem), &elevation_map_buffer_);
  clSetKernelArg(update_variance_kernel_, 1, sizeof(float),  &time_variance_f);
  clSetKernelArg(update_variance_kernel_, 2, sizeof(float),  &max_variance_f);
  clSetKernelArg(update_variance_kernel_, 3, sizeof(int),    &width_i);
  clSetKernelArg(update_variance_kernel_, 4, sizeof(int),    &height_i);

  size_t global_work_size = static_cast<size_t>(width_ * height_);
  cl_int err = clEnqueueNDRangeKernel(
    queue_, update_variance_kernel_,
    1, nullptr, &global_work_size, nullptr,
    0, nullptr, nullptr);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("❌ 执行 update_variance_kernel 失败！");
  }
  clFinish(queue_);
}

// =====================================================================
// 计算法向量
// =====================================================================
void ElevationMapOpenCL::computeNormals()
{
  float resolution_f = static_cast<float>(config_.resolution);
  int width_i  = width_;
  int height_i = height_;

  clSetKernelArg(compute_normals_kernel_, 0, sizeof(cl_mem), &elevation_map_buffer_);
  clSetKernelArg(compute_normals_kernel_, 1, sizeof(cl_mem), &normal_map_buffer_);
  clSetKernelArg(compute_normals_kernel_, 2, sizeof(float),  &resolution_f);
  clSetKernelArg(compute_normals_kernel_, 3, sizeof(int),    &width_i);
  clSetKernelArg(compute_normals_kernel_, 4, sizeof(int),    &height_i);

  // 2D 工作空间（每个像素一个线程）
  size_t global_work_size[2] = {
    static_cast<size_t>(width_),
    static_cast<size_t>(height_)
  };

  cl_int err = clEnqueueNDRangeKernel(
    queue_, compute_normals_kernel_,
    2,       // 2D 工作空间
    nullptr, global_work_size, nullptr,
    0, nullptr, nullptr);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("❌ 执行 compute_normals_kernel 失败！");
  }
  clFinish(queue_);
}

// =====================================================================
// 计算可通行性
// =====================================================================
void ElevationMapOpenCL::computeTraversability()
{
  float max_slope_f       = static_cast<float>(config_.max_slope);
  float max_step_height_f = static_cast<float>(config_.max_step_height);
  float slope_weight_f    = static_cast<float>(config_.slope_weight);
  float step_weight_f     = static_cast<float>(config_.step_weight);
  int width_i  = width_;
  int height_i = height_;

  clSetKernelArg(compute_traversability_kernel_, 0, sizeof(cl_mem), &elevation_map_buffer_);
  clSetKernelArg(compute_traversability_kernel_, 1, sizeof(cl_mem), &normal_map_buffer_);
  clSetKernelArg(compute_traversability_kernel_, 2, sizeof(cl_mem), &traversability_buffer_);
  clSetKernelArg(compute_traversability_kernel_, 3, sizeof(float),  &max_slope_f);
  clSetKernelArg(compute_traversability_kernel_, 4, sizeof(float),  &max_step_height_f);
  clSetKernelArg(compute_traversability_kernel_, 5, sizeof(float),  &slope_weight_f);
  clSetKernelArg(compute_traversability_kernel_, 6, sizeof(float),  &step_weight_f);
  clSetKernelArg(compute_traversability_kernel_, 7, sizeof(int),    &width_i);
  clSetKernelArg(compute_traversability_kernel_, 8, sizeof(int),    &height_i);

  size_t global_work_size[2] = {
    static_cast<size_t>(width_),
    static_cast<size_t>(height_)
  };

  cl_int err = clEnqueueNDRangeKernel(
    queue_, compute_traversability_kernel_,
    2, nullptr, global_work_size, nullptr,
    0, nullptr, nullptr);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("❌ 执行 compute_traversability_kernel 失败！");
  }
  clFinish(queue_);
}

// =====================================================================
// 地图中心移动（跟随机器人）
// =====================================================================
void ElevationMapOpenCL::moveTo(
  const Eigen::Vector3f & new_center,
  const Eigen::Matrix3f & new_rotation)
{
  // ===== 计算需要平移的像素数 =====
  Eigen::Vector3f delta = new_center - center_;
  float res = static_cast<float>(config_.resolution);

  // 水平方向的像素偏移（取整）
  int shift_x = static_cast<int>(std::round(delta.x() / res));
  int shift_y = static_cast<int>(std::round(delta.y() / res));

  // 如果偏移量为零，只更新旋转
  if (shift_x == 0 && shift_y == 0) {
    base_rotation_ = new_rotation;
    center_ = new_center;
    return;
  }

  std::cout << "[ElevationMapOpenCL] 地图中心移动: dx=" << shift_x
            << ", dy=" << shift_y << " 像素" << std::endl;

  // ===== 调用 GPU 平移 kernel =====
  int width_i  = width_;
  int height_i = height_;
  int num_layers = NUM_LAYERS;

  clSetKernelArg(shift_map_kernel_, 0, sizeof(cl_mem), &elevation_map_buffer_);
  clSetKernelArg(shift_map_kernel_, 1, sizeof(cl_mem), &elevation_map_buffer_tmp_);
  clSetKernelArg(shift_map_kernel_, 2, sizeof(int),    &shift_x);
  clSetKernelArg(shift_map_kernel_, 3, sizeof(int),    &shift_y);
  clSetKernelArg(shift_map_kernel_, 4, sizeof(int),    &width_i);
  clSetKernelArg(shift_map_kernel_, 5, sizeof(int),    &height_i);
  clSetKernelArg(shift_map_kernel_, 6, sizeof(int),    &num_layers);

  // 3D 工作空间：col × row × layer
  size_t global_work_size[3] = {
    static_cast<size_t>(width_),
    static_cast<size_t>(height_),
    static_cast<size_t>(num_layers)
  };

  cl_int err = clEnqueueNDRangeKernel(
    queue_, shift_map_kernel_,
    3, nullptr, global_work_size, nullptr,
    0, nullptr, nullptr);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("❌ 执行 shift_map_kernel 失败！");
  }
  clFinish(queue_);

  // ===== 将临时缓冲区复制回主缓冲区 =====
  size_t total_bytes = NUM_LAYERS * width_ * height_ * sizeof(float);
  err = clEnqueueCopyBuffer(
    queue_, elevation_map_buffer_tmp_, elevation_map_buffer_,
    0, 0, total_bytes, 0, nullptr, nullptr);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("❌ 拷贝临时缓冲区失败！");
  }
  clFinish(queue_);

  // ===== 更新状态 =====
  // 修正中心（只移动整像素对应的实际距离）
  center_.x() += shift_x * res;
  center_.y() += shift_y * res;
  base_rotation_ = new_rotation;
}

// =====================================================================
// 高斯平滑
// =====================================================================
void ElevationMapOpenCL::smooth()
{
  // 如果高斯核还未生成，自动生成默认核
  if (!gaussian_kernel_buffer_) {
    generateGaussianKernel(1.0f, 3);  // sigma=1.0, 3x3
  }
  
  if (post_process_state_.smooth_applied) {
    std::cout << "smooth 已在本帧应用过，跳过重复应用" << std::endl;
    return;
  }

  int width_i = width_;
  int height_i = height_;
  int kernel_size = gaussian_kernel_size_;

  clSetKernelArg(gaussian_smooth_kernel_, 0, sizeof(cl_mem), &elevation_map_buffer_);
  clSetKernelArg(gaussian_smooth_kernel_, 1, sizeof(cl_mem), &elevation_map_buffer_post_a_);
  clSetKernelArg(gaussian_smooth_kernel_, 2, sizeof(cl_mem), &gaussian_kernel_buffer_);
  clSetKernelArg(gaussian_smooth_kernel_, 3, sizeof(int), &width_i);
  clSetKernelArg(gaussian_smooth_kernel_, 4, sizeof(int), &height_i);
  clSetKernelArg(gaussian_smooth_kernel_, 5, sizeof(int), &kernel_size);

  size_t global_work_size[2] = {
    static_cast<size_t>(width_),
    static_cast<size_t>(height_)
  };

  cl_int err = clEnqueueNDRangeKernel(
    queue_, gaussian_smooth_kernel_,
    2, nullptr, global_work_size, nullptr,
    0, nullptr, nullptr);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("❌ 执行 gaussian_smooth_kernel 失败！");
  }
  clFinish(queue_);

  copyPostProcessToMain();
  post_process_state_.smooth_applied = true;

  // std::cout << "[ElevationMapOpenCL] smooth 完成" << std::endl;  // 已禁用，避免刷屏
}

// =====================================================================
// 空洞填补
// =====================================================================
void ElevationMapOpenCL::inpaint()
{
  if (post_process_state_.inpaint_applied) {
    std::cout << "inpaint 已在本帧应用过，跳过重复应用" << std::endl;
    return;
  }

  int width_i = width_;
  int height_i = height_;
  int kernel_radius = inpaint_config_.kernel_radius;
  float max_inpaint_distance = inpaint_config_.max_inpaint_distance;
  int max_iters = inpaint_config_.max_iters;
  float resolution_f = static_cast<float>(config_.resolution); 

  if (max_iters == 0) {  
    post_process_state_.inpaint_applied = true;
    return;
  }

  size_t global_work_size[2] = {
    static_cast<size_t>(width_),
    static_cast<size_t>(height_)
  };

  bool result_in_a = true;

  for (int iter = 0; iter < max_iters; ++iter) {
    cl_mem src_buf;
    cl_mem dst_buf;

    if (iter == 0) {
      // 第一次从主缓冲区读
      src_buf = elevation_map_buffer_;
      dst_buf = elevation_map_buffer_post_a_;
      result_in_a = true;
    } else if (result_in_a) {
      // 上次结果在 post_a，这次从 post_a 读，写入 post_b
      src_buf = elevation_map_buffer_post_a_;
      dst_buf = elevation_map_buffer_post_b_;
      result_in_a = false;
    } else {
      // 上次结果在 post_b，这次从 post_b 读，写入 post_a
      src_buf = elevation_map_buffer_post_b_;
      dst_buf = elevation_map_buffer_post_a_;
      result_in_a = true;
    }

    clSetKernelArg(inpaint_kernel_, 0, sizeof(cl_mem), &src_buf);
    clSetKernelArg(inpaint_kernel_, 1, sizeof(cl_mem), &dst_buf);
    clSetKernelArg(inpaint_kernel_, 2, sizeof(int), &width_i);
    clSetKernelArg(inpaint_kernel_, 3, sizeof(int), &height_i);
    clSetKernelArg(inpaint_kernel_, 4, sizeof(int), &kernel_radius);
    clSetKernelArg(inpaint_kernel_, 5, sizeof(float), &max_inpaint_distance);
    clSetKernelArg(inpaint_kernel_, 6, sizeof(float), &resolution_f);

    cl_int err = clEnqueueNDRangeKernel(
      queue_, inpaint_kernel_,
      2, nullptr, global_work_size, nullptr,
      0, nullptr, nullptr);
    if (err != CL_SUCCESS) {
      throw std::runtime_error("❌ 执行 inpaint_kernel 失败！");
    }
    clFinish(queue_);
  }

  // 如果最后结果不在 post_a，确保复制回 post_a 方便统一下游处理
  cl_mem final_buf = result_in_a ? elevation_map_buffer_post_a_ : elevation_map_buffer_post_b_;

  if (final_buf != elevation_map_buffer_post_a_) {
      size_t total_bytes = NUM_LAYERS * width_ * height_ * sizeof(float);
      cl_int err = clEnqueueCopyBuffer(
        queue_, final_buf, elevation_map_buffer_post_a_,
        0, 0, total_bytes, 0, nullptr, nullptr);
      if (err != CL_SUCCESS) {
        throw std::runtime_error("❌ inpaint 结果回拷失败！");
      }
      clFinish(queue_);
  }

  copyPostProcessToMain();
  post_process_state_.inpaint_applied = true;  // 统一放这里

  // std::cout << "[ElevationMapOpenCL] inpaint 完成" << std::endl;  // 已禁用，避免刷屏
}

// =====================================================================
// 形态学侵蚀（真正的 erosion）
// =====================================================================
void ElevationMapOpenCL::computeErosion()
{
  int erosion_radius = erosion_config_.erosion_radius;
  float safety_threshold = erosion_config_.safety_threshold;
  float erosion_strength = erosion_config_.erosion_strength;
  int width_i = width_;
  int height_i = height_;

  clSetKernelArg(morphological_erosion_kernel_, 0, sizeof(cl_mem), &traversability_buffer_);
  clSetKernelArg(morphological_erosion_kernel_, 1, sizeof(cl_mem), &erosion_buffer_);
  clSetKernelArg(morphological_erosion_kernel_, 2, sizeof(int), &width_i);
  clSetKernelArg(morphological_erosion_kernel_, 3, sizeof(int), &height_i);
  clSetKernelArg(morphological_erosion_kernel_, 4, sizeof(int), &erosion_radius);
  clSetKernelArg(morphological_erosion_kernel_, 5, sizeof(float), &safety_threshold);
  clSetKernelArg(morphological_erosion_kernel_, 6, sizeof(float), &erosion_strength);

  size_t global_work_size[2] = {
    static_cast<size_t>(width_),
    static_cast<size_t>(height_)
  };

  cl_int err = clEnqueueNDRangeKernel(
    queue_, morphological_erosion_kernel_,
    2, nullptr, global_work_size, nullptr,
    0, nullptr, nullptr);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("❌ 执行 morphological_erosion_kernel 失败！");
  }
  clFinish(queue_);
}

// =====================================================================
// 视野清理
// =====================================================================
void ElevationMapOpenCL::cleanupVisibility(const Eigen::Vector3f & sensor_pos)
{
  float sensor_x = sensor_pos.x();
  float sensor_y = sensor_pos.y();
  float sensor_z = sensor_pos.z();  // 目前可不使用，但保留接口扩展性
  float resolution_f = static_cast<float>(config_.resolution);
  int width_i = width_;
  int height_i = height_;
  float max_ray_length = 10.0f;
  float cleanup_threshold = 5.0f;
  float invalidation_factor = 1.5f;

  clSetKernelArg(visibility_cleanup_kernel_, 0, sizeof(cl_mem), &elevation_map_buffer_);
  clSetKernelArg(visibility_cleanup_kernel_, 1, sizeof(float), &sensor_x);
  clSetKernelArg(visibility_cleanup_kernel_, 2, sizeof(float), &sensor_y);
  clSetKernelArg(visibility_cleanup_kernel_, 3, sizeof(float), &sensor_z);
  clSetKernelArg(visibility_cleanup_kernel_, 4, sizeof(float), &resolution_f);
  clSetKernelArg(visibility_cleanup_kernel_, 5, sizeof(int), &width_i);
  clSetKernelArg(visibility_cleanup_kernel_, 6, sizeof(int), &height_i);
  clSetKernelArg(visibility_cleanup_kernel_, 7, sizeof(float), &max_ray_length);
  clSetKernelArg(visibility_cleanup_kernel_, 8, sizeof(float), &cleanup_threshold);
  clSetKernelArg(visibility_cleanup_kernel_, 9, sizeof(float), &invalidation_factor);

  size_t global_work_size[2] = {
    static_cast<size_t>(width_),
    static_cast<size_t>(height_)
  };

  cl_int err = clEnqueueNDRangeKernel(
    queue_, visibility_cleanup_kernel_,
    2, nullptr, global_work_size, nullptr,
    0, nullptr, nullptr);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("❌ 执行 visibility_cleanup_kernel 失败！");
  }
  clFinish(queue_);
}

// =====================================================================
// 最小值滤波
// =====================================================================
void ElevationMapOpenCL::applyMinFilter()
{
  if (post_process_state_.min_filter_applied) {
    std::cout << "[ElevationMapOpenCL] min_filter 已在本帧应用过，跳过" << std::endl;
    return;
  }

  int filter_radius = 1;  // 3x3 邻域
  int width_i = width_;
  int height_i = height_;
  float spike_threshold = 0.10f;   // 高于邻居均值 10cm 才视为孤立高点

  clSetKernelArg(min_filter_kernel_, 0, sizeof(cl_mem), &elevation_map_buffer_);
  clSetKernelArg(min_filter_kernel_, 1, sizeof(cl_mem), &elevation_map_buffer_post_a_);
  clSetKernelArg(min_filter_kernel_, 2, sizeof(int), &width_i);
  clSetKernelArg(min_filter_kernel_, 3, sizeof(int), &height_i);
  clSetKernelArg(min_filter_kernel_, 4, sizeof(int), &filter_radius);
  clSetKernelArg(min_filter_kernel_, 5, sizeof(float),  &spike_threshold);

  size_t global_work_size[2] = {
    static_cast<size_t>(width_),
    static_cast<size_t>(height_)
  };

  cl_int err = clEnqueueNDRangeKernel(
    queue_, min_filter_kernel_,
    2, nullptr, global_work_size, nullptr,
    0, nullptr, nullptr);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("❌ 执行 min_filter_kernel 失败！");
  }
  clFinish(queue_);

  copyPostProcessToMain();
  post_process_state_.min_filter_applied = true;

  // std::cout << "[ElevationMapOpenCL] min_filter 完成" << std::endl;  // 已禁用，避免刷屏
}

// =====================================================================
// 生成高斯核权重
// =====================================================================
void ElevationMapOpenCL::generateGaussianKernel(float sigma, int kernel_size)
{
  // kernel_size 应该是奇数，如 3, 5, 7
  if (kernel_size % 2 == 0) kernel_size++;
  
  int half_size = kernel_size / 2;
  std::vector<float> kernel(kernel_size * kernel_size);
  
  float sum = 0.0f;
  float sigma_sq = sigma * sigma;
  
  // 生成高斯权重
  for (int y = -half_size; y <= half_size; ++y) {
    for (int x = -half_size; x <= half_size; ++x) {
      int idx = (y + half_size) * kernel_size + (x + half_size);
      float dist_sq = x * x + y * y;
      kernel[idx] = std::exp(-dist_sq / (2.0f * sigma_sq));
      sum += kernel[idx];
    }
  }
  
  // 归一化
  for (int i = 0; i < kernel_size * kernel_size; ++i) {
    kernel[i] /= sum;
  }
  
  // 上传到 GPU
  cl_int err;
  if (gaussian_kernel_buffer_) {
    clReleaseMemObject(gaussian_kernel_buffer_);
  }
  
  gaussian_kernel_buffer_ = clCreateBuffer(
    context_,
    CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
    kernel_size * kernel_size * sizeof(float),
    kernel.data(), &err);
  
  if (err != CL_SUCCESS) {
    throw std::runtime_error("❌ 上传高斯核权重失败！");
  }
  
  gaussian_kernel_size_ = kernel_size;

  std::cout << "[OpenCL] 高斯核生成完成: sigma=" << sigma 
            << ", kernel_size=" << kernel_size << std::endl;
}

void ElevationMapOpenCL::compensateDrift(float drift_offset)
{
  int width_i = width_;
  int height_i = height_;

  clSetKernelArg(drift_compensation_kernel_, 0, sizeof(cl_mem), &elevation_map_buffer_);
  clSetKernelArg(drift_compensation_kernel_, 1, sizeof(int), &width_i);
  clSetKernelArg(drift_compensation_kernel_, 2, sizeof(int), &height_i);
  clSetKernelArg(drift_compensation_kernel_, 3, sizeof(float), &drift_offset);

  size_t global_work_size[2] = {
    static_cast<size_t>(width_),
    static_cast<size_t>(height_)
  };

  cl_int err = clEnqueueNDRangeKernel(
    queue_, drift_compensation_kernel_,
    2, nullptr, global_work_size, nullptr,
    0, nullptr, nullptr);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("❌ 执行 drift_compensation_kernel 失败！");
  }
  clFinish(queue_);
}

// =====================================================================
// 后处理缓冲交换
// =====================================================================
void ElevationMapOpenCL::swapPostProcessBuffers()
{
  cl_mem temp = elevation_map_buffer_post_a_;
  elevation_map_buffer_post_a_ = elevation_map_buffer_post_b_;
  elevation_map_buffer_post_b_ = temp;
}

// =====================================================================
// 将后处理结果复制回主缓冲区
// =====================================================================
void ElevationMapOpenCL::copyPostProcessToMain()
{
  // 只复制 elevation / variance / is_valid 三层
  size_t cell_count = static_cast<size_t>(width_ * height_);
  size_t three_layers_bytes = 3 * cell_count * sizeof(float);
  
  cl_int err = clEnqueueCopyBuffer(
    queue_, elevation_map_buffer_post_a_, elevation_map_buffer_,
    0, 0, three_layers_bytes, 0, nullptr, nullptr);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("❌ 复制后处理结果到主缓冲失败！");
  }
  clFinish(queue_);
}

// =====================================================================
// 获取指定图层数据（从 GPU 读回 CPU）
// =====================================================================
void ElevationMapOpenCL::getLayer(
  const std::string & name,
  std::vector<float> & data)
{
  // 确定图层索引
  int layer_idx = -1;
  if (name == "elevation")     layer_idx = LAYER_ELEVATION;
  else if (name == "variance") layer_idx = LAYER_VARIANCE;
  else if (name == "is_valid") layer_idx = LAYER_IS_VALID;
  else if (name == "traversability") {
    // 可通行性来自独立缓冲区
    data.resize(width_ * height_);
    size_t bytes = width_ * height_ * sizeof(float);
    cl_int err = clEnqueueReadBuffer(
      queue_, traversability_buffer_, CL_TRUE,
      0, bytes, data.data(), 0, nullptr, nullptr);
    if (err != CL_SUCCESS) {
      throw std::runtime_error("❌ 读取 traversability 图层失败！");
    }
    return;
  }
  else if (name == "erosion") {
    // 新增：读取侵蚀后的可通行性
    data.resize(width_ * height_);
    size_t bytes = width_ * height_ * sizeof(float);
    cl_int err = clEnqueueReadBuffer(
      queue_, erosion_buffer_, CL_TRUE,
      0, bytes, data.data(), 0, nullptr, nullptr);
    if (err != CL_SUCCESS) {
      throw std::runtime_error("❌ 读取 erosion 图层失败！");
    }
    return;
  }

  else if (name == "time")           layer_idx = LAYER_TIME;
  else if (name == "upper_bound")    layer_idx = LAYER_UPPER_BOUND;
  else if (name == "is_upper_bound") layer_idx = LAYER_IS_UPPER_BOUND;
  else {
    throw std::runtime_error("❌ 未知图层名称: " + name);
  }

  // 读取对应图层数据
  size_t cell_count = static_cast<size_t>(width_ * height_);
  size_t offset_bytes = static_cast<size_t>(layer_idx) * cell_count * sizeof(float);
  size_t layer_bytes  = cell_count * sizeof(float);

  data.resize(cell_count);
  cl_int err = clEnqueueReadBuffer(
    queue_, elevation_map_buffer_, CL_TRUE,
    offset_bytes, layer_bytes, data.data(),
    0, nullptr, nullptr);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("❌ 读取图层 [" + name + "] 失败！");
  }
}

// =====================================================================
// 清空地图（重置所有栅格）
// =====================================================================
void ElevationMapOpenCL::clear()
{
  size_t total_cells = static_cast<size_t>(width_ * height_);
  std::vector<float> initial_map(NUM_LAYERS * total_cells, 0.0f);

  // 初始化方差层
  for (size_t i = 0; i < total_cells; ++i) {
    initial_map[LAYER_VARIANCE * total_cells + i] = 10.0f;
  }

  size_t total_bytes = NUM_LAYERS * total_cells * sizeof(float);
  cl_int err = clEnqueueWriteBuffer(
    queue_, elevation_map_buffer_, CL_TRUE, 0,
    total_bytes, initial_map.data(), 0, nullptr, nullptr);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("❌ 清空地图失败！");
  }

  std::cout << "[ElevationMapOpenCL] 地图已清空 ✅" << std::endl;
}

}  // namespace elevation_mapping_opencl