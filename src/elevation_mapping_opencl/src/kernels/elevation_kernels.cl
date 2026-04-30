/**
 * @file elevation_kernels.cl
 * @brief OpenCL kernels for elevation mapping on Intel Arc GPU
 * @author Your Name
 * @date 2024
 * 
 * 功能说明：
 * 1. add_points_kernel: 将点云融合到高程图（卡尔曼滤波）
 * 2. update_variance_kernel: 更新方差（时间衰减）
 * 3. compute_normals_kernel: 计算地形法向量
 * 4. compute_traversability_kernel: 计算可通行性
 */

// =====================================================================
// 1. 点云融合 Kernel（核心算法：卡尔曼滤波）
// =====================================================================
__kernel void add_points_kernel(
    __global float* elevation_map,      // 输入/输出：高程图 [7, height, width]
                                        // 图层0: elevation (高程)
                                        // 图层1: variance (方差)
                                        // 图层2: is_valid (是否有效)
                                        // 图层3: traversability (可通行性)
                                        // 图层4: time (时间戳)
                                        // 图层5: upper_bound (上界)
                                        // 图层6: is_upper_bound (是否上界)
    __global const float* points,       // 输入：点云 [N, 3] (x, y, z)
    __global const float* R,            // 输入：旋转矩阵 [3, 3] (行优先)
    __global const float* t,            // 输入：平移向量 [3]
    const float resolution,             // 地图分辨率（米/像素）
    const int width,                    // 地图宽度（像素）
    const int height,                   // 地图高度（像素）
    const float center_x,               // 地图中心 X（世界坐标，米）
    const float center_y,               // 地图中心 Y（世界坐标，米）
    const float sensor_noise_factor,    // 传感器噪声系数
    const float mahalanobis_thresh,     // 马氏距离阈值（离群点检测）
    const float outlier_variance,       // 离群点方差增量
    const float min_valid_distance,     // 最小有效距离（米）
    const float max_height_range        // 最大高度范围（米）
)
{
    // 获取当前线程处理的点索引
    int idx = get_global_id(0);
    int N = get_global_size(0);
    
    if (idx >= N) return;
    
    // ===== 步骤1：读取点坐标 =====
    float px = points[idx * 3 + 0];
    float py = points[idx * 3 + 1];
    float pz = points[idx * 3 + 2];
    
    // 检查 NaN
    if (isnan(px) || isnan(py) || isnan(pz)) {
        return;
    }
    
    // ===== 步骤2：坐标变换（传感器坐标系 → 地图坐标系）=====
    // p_map = R * p_sensor + t
    float x_map = R[0] * px + R[1] * py + R[2] * pz + t[0];
    float y_map = R[3] * px + R[4] * py + R[5] * pz + t[1];
    float z_map = R[6] * px + R[7] * py + R[8] * pz + t[2];
    
    // ===== 步骤3：距离过滤 =====
    float distance = sqrt(px * px + py * py + pz * pz);
    if (distance < min_valid_distance) {
        return;  // 太近的点（可能是机器人自身）
    }
    
    // ===== 步骤4：高度过滤 =====
    if (fabs(z_map) > max_height_range) {
        return;  // 高度超出范围
    }
    
    // ===== 步骤5：转换为栅格索引 =====
    // 地图中心在 (width/2, height/2)
    int col = (int)(((x_map - center_x) / resolution) + width  * 0.5f);
    int row = (int)(((y_map - center_y) / resolution) + height * 0.5f);
    
    // 边界检查
    if (col < 1 || col >= width - 1 || row < 1 || row >= height - 1) {
        return;  // 超出地图范围
    }
    
    // ===== 步骤6：计算线性索引 =====
    int cell_idx = row * width + col;
    int total_cells = width * height;
    
    // ===== 步骤7：读取当前栅格状态 =====
    float current_elevation = elevation_map[0 * total_cells + cell_idx];
    float current_variance = elevation_map[1 * total_cells + cell_idx];
    float is_valid = elevation_map[2 * total_cells + cell_idx];
    
    // ===== 步骤8：计算测量噪声（距离越远，噪声越大）=====
    float measurement_variance = sensor_noise_factor * distance * distance;
    
    // ===== 步骤9：卡尔曼滤波融合 =====
    if (is_valid > 0.5f) {
        // --- 情况A：已有数据，进行融合 ---
        
        // 计算新息（innovation）= 测量值 - 预测值
        float innovation = z_map - current_elevation;
        
        // 新息方差 = 预测方差 + 测量方差
        float innovation_variance = current_variance + measurement_variance;
        
        // 马氏距离（用于离群点检测）
        float mahalanobis_distance = (innovation * innovation) / innovation_variance;
        
        if (mahalanobis_distance < mahalanobis_thresh) {
            // --- 正常点：执行卡尔曼更新 ---
            
            // 卡尔曼增益 K = P / (P + R)
            float kalman_gain = current_variance / innovation_variance;
            
            // 更新高程：x = x + K * innovation
            float new_elevation = current_elevation + kalman_gain * innovation;
            
            // 更新方差：P = (1 - K) * P
            float new_variance = (1.0f - kalman_gain) * current_variance;
            
            // 写入结果（注意：OpenCL 不保证原子性，但高程图更新通常可以容忍小误差）
            elevation_map[0 * total_cells + cell_idx] = new_elevation;
            elevation_map[1 * total_cells + cell_idx] = new_variance;
            
        } else {
            // --- 离群点：只增加方差，不更新高程 ---
            float new_variance = current_variance + outlier_variance;
            elevation_map[1 * total_cells + cell_idx] = new_variance;
        }
        
    } else {
        // --- 情况B：首次观测，直接初始化 ---
        elevation_map[0 * total_cells + cell_idx] = z_map;
        elevation_map[1 * total_cells + cell_idx] = measurement_variance;
        elevation_map[2 * total_cells + cell_idx] = 1.0f;  // 标记为有效
    }
}

// =====================================================================
// 2. 方差更新 Kernel（时间衰减）
// =====================================================================
__kernel void update_variance_kernel(
    __global float* elevation_map,      // 输入/输出：高程图
    const float time_variance,          // 时间方差增量
    const float max_variance,           // 最大方差限制
    const int width,
    const int height
)
{
    int idx = get_global_id(0);
    int total = width * height;
    
    if (idx >= total) return;
    
    // 只更新有效栅格的方差
    float is_valid = elevation_map[2 * total + idx];
    
    if (is_valid > 0.5f) {
        float current_variance = elevation_map[1 * total + idx];
        float new_variance = current_variance + time_variance;
        
        // 限制最大方差
        new_variance = fmin(new_variance, max_variance);
        
        elevation_map[1 * total + idx] = new_variance;
    }
}

// =====================================================================
// 3. 法线计算 Kernel（用于坡度分析）
// =====================================================================
__kernel void compute_normals_kernel(
    __global const float* elevation_map,
    __global float* normal_map,
    const float resolution,
    const int width,
    const int height)
{
    int col = get_global_id(0);
    int row = get_global_id(1);
    if (col >= width || row >= height) return;

    int idx = row * width + col;
    int total = width * height;

    // 检查当前点是否有效
    if (elevation_map[2 * total + idx] < 0.5f) {
        normal_map[0 * total + idx] = 0.0f;
        normal_map[1 * total + idx] = 0.0f;
        normal_map[2 * total + idx] = 1.0f;  // 默认朝上
        return;
    }

    float z_center = elevation_map[0 * total + idx];

    // ===== 关键修改：检查邻居有效性，无效时用中心值代替 =====
    // 左邻居
    float z_left = z_center;
    if (col > 0) {
        int l_idx = row * width + (col - 1);
        if (elevation_map[2 * total + l_idx] > 0.5f) {
            z_left = elevation_map[0 * total + l_idx];
        }
    }

    // 右邻居
    float z_right = z_center;
    if (col < width - 1) {
        int r_idx = row * width + (col + 1);
        if (elevation_map[2 * total + r_idx] > 0.5f) {
            z_right = elevation_map[0 * total + r_idx];
        }
    }

    // 上邻居
    float z_up = z_center;
    if (row > 0) {
        int u_idx = (row - 1) * width + col;
        if (elevation_map[2 * total + u_idx] > 0.5f) {
            z_up = elevation_map[0 * total + u_idx];
        }
    }

    // 下邻居
    float z_down = z_center;
    if (row < height - 1) {
        int d_idx = (row + 1) * width + col;
        if (elevation_map[2 * total + d_idx] > 0.5f) {
            z_down = elevation_map[0 * total + d_idx];
        }
    }

    // 中心差分计算法向量（归一化后）
    float dx = (z_right - z_left) / (2.0f * resolution);
    float dy = (z_down  - z_up  ) / (2.0f * resolution);

    // 法向量为 (-dx, -dy, 1)，然后归一化
    float nx = -dx;
    float ny = -dy;
    float nz =  1.0f;
    float len = sqrt(nx*nx + ny*ny + nz*nz);
    if (len > 1e-6f) { nx /= len; ny /= len; nz /= len; }

    normal_map[0 * total + idx] = nx;
    normal_map[1 * total + idx] = ny;
    normal_map[2 * total + idx] = nz;
}

// =====================================================================
// 4. 可通行性计算 Kernel（综合坡度和台阶高度）
// =====================================================================
__kernel void compute_traversability_kernel(
    __global const float* elevation_map,        // 输入：高程图
    __global const float* normal_map,           // 输入：法向量图
    __global float* traversability_map,         // 输出：可通行性图 [height, width]
    const float max_slope,                      // 最大坡度（弧度）
    const float max_step_height,                // 最大台阶高度（米）
    const float slope_weight,                   // 坡度权重
    const float step_weight,                    // 台阶权重
    const int width,
    const int height
)
{
    int col = get_global_id(0);
    int row = get_global_id(1);
    
    if (col >= width || row >= height) return;
    
    int idx = row * width + col;
    int total = width * height;
    
    // ===== 检查有效性 =====
    float is_valid = elevation_map[2 * total + idx];
    if (is_valid < 0.5f) {
        traversability_map[idx] = 0.0f;  // 无效栅格不可通行
        return;
    }
    
    // ===== 步骤1：计算坡度评分 =====
    float nx = normal_map[0 * total + idx];
    float ny = normal_map[1 * total + idx];
    float nz = normal_map[2 * total + idx];
    
    // 坡度 = 法向量与竖直方向的夹角
    // cos(θ) = nz / |n| = nz (因为已归一化)
    // θ = acos(nz)
    float slope = acos(fmax(fmin(nz, 1.0f), -1.0f));  // 限制在 [-1, 1]
    
    // 坡度评分：线性映射 [0, max_slope] → [1, 0]
    float slope_score = 1.0f - fmin(slope / max_slope, 1.0f);
    
    // ===== 步骤2：计算台阶高度评分 =====
    float z_center = elevation_map[0 * total + idx];
    float max_height_diff = 0.0f;
    
    // 遍历 8 邻域
    for (int dr = -1; dr <= 1; dr++) {
        for (int dc = -1; dc <= 1; dc++) {
            if (dr == 0 && dc == 0) continue;  // 跳过中心
            
            int neighbor_row = row + dr;
            int neighbor_col = col + dc;
            
            // 边界检查
            if (neighbor_row >= 0 && neighbor_row < height && 
                neighbor_col >= 0 && neighbor_col < width) {
                
                int neighbor_idx = neighbor_row * width + neighbor_col;
                float is_valid_neighbor = elevation_map[2 * total + neighbor_idx];
                
                if (is_valid_neighbor > 0.5f) {
                    float z_neighbor = elevation_map[0 * total + neighbor_idx];
                    float height_diff = fabs(z_center - z_neighbor);
                    max_height_diff = fmax(max_height_diff, height_diff);
                }
            }
        }
    }
    
    // 台阶评分：线性映射 [0, max_step_height] → [1, 0]
    float step_score = 1.0f - fmin(max_height_diff / max_step_height, 1.0f);
    
    // ===== 步骤3：综合评分（加权平均）=====
    float traversability = slope_weight * slope_score + step_weight * step_score;
    
    // 限制在 [0, 1]
    traversability = fmax(0.0f, fmin(1.0f, traversability));
    
    traversability_map[idx] = traversability;
}

// =====================================================================
// 5. 地图平移 Kernel（用于地图中心移动）
// =====================================================================
__kernel void shift_map_kernel(
    __global const float* src_map,      // 输入：原始地图
    __global float* dst_map,            // 输出：平移后的地图
    const int shift_x,                  // X 方向平移（像素）
    const int shift_y,                  // Y 方向平移（像素）
    const int width,
    const int height,
    const int num_layers                // 图层数量
)
{
    int col = get_global_id(0);
    int row = get_global_id(1);
    int layer = get_global_id(2);
    
    if (col >= width || row >= height || layer >= num_layers) return;
    
    int dst_idx = layer * width * height + row * width + col;
    
    // 计算源位置
    int src_col = col - shift_x;
    int src_row = row - shift_y;
    
    // 边界检查
    if (src_col >= 0 && src_col < width && src_row >= 0 && src_row < height) {
        int src_idx = layer * width * height + src_row * width + src_col;
        dst_map[dst_idx] = src_map[src_idx];
    } else {
        // 超出边界，填充默认值
        if (layer == 1) {
            dst_map[dst_idx] = 10.0f;  // 方差层填充初始方差
        } else {
            dst_map[dst_idx] = 0.0f;   // 其他层填充 0
        }
    }
}

// =====================================================================
// 6. 高斯平滑 Kernel（使用动态高斯权重）
// =====================================================================
__kernel void gaussian_smooth_kernel(
    __global const float* src_map,      // 输入：原始高程图
    __global float* dst_map,            // 输出：平滑后的高程图
    __global const float* gaussian_kernel,  // 高斯权重
    const int width,
    const int height,
    const int kernel_size               // 高斯核大小（3, 5, 7 等）
)
{
    int col = get_global_id(0);
    int row = get_global_id(1);
    
    if (col >= width || row >= height) return;
    
    int idx = row * width + col;
    int total = width * height;
    
    // 检查有效性
    float is_valid = src_map[2 * total + idx];
    if (is_valid < 0.5f) {
        // 无效栅格直接复制
        dst_map[0 * total + idx] = src_map[0 * total + idx];
        dst_map[1 * total + idx] = src_map[1 * total + idx];
        dst_map[2 * total + idx] = 0.0f;
        return;
    }
    
    int half_size = kernel_size / 2;
    float sum_elevation = 0.0f;
    float sum_weight = 0.0f;
    
    // 遍历邻域
    for (int dr = -half_size; dr <= half_size; dr++) {
        for (int dc = -half_size; dc <= half_size; dc++) {
            int neighbor_row = row + dr;
            int neighbor_col = col + dc;
            
            // 边界检查
            if (neighbor_row >= 0 && neighbor_row < height && 
                neighbor_col >= 0 && neighbor_col < width) {
                
                int neighbor_idx = neighbor_row * width + neighbor_col;
                float neighbor_valid = src_map[2 * total + neighbor_idx];
                
                if (neighbor_valid > 0.5f) {
                    float neighbor_elevation = src_map[0 * total + neighbor_idx];
                    int kernel_idx = (dr + half_size) * kernel_size + (dc + half_size);
                    float weight = gaussian_kernel[kernel_idx];
                    
                    sum_elevation += neighbor_elevation * weight;
                    sum_weight += weight;
                }
            }
        }
    }
    
    // 归一化
    if (sum_weight > 1e-6f) {
        dst_map[0 * total + idx] = sum_elevation / sum_weight;
        dst_map[1 * total + idx] = src_map[1 * total + idx];  // 方差保持
        dst_map[2 * total + idx] = 1.0f;
    } else {
        dst_map[0 * total + idx] = src_map[0 * total + idx];
        dst_map[1 * total + idx] = src_map[1 * total + idx];
        dst_map[2 * total + idx] = is_valid;
    }
}

// =====================================================================
// 7. 空洞填补 Kernel（迭代式扩散）
// =====================================================================
__kernel void inpaint_kernel(
    __global const float* src_map,
    __global float* dst_map,
    const int width,
    const int height,
    const int kernel_radius,
    const float max_inpaint_distance,
    const float resolution              
)
{
    int col = get_global_id(0);
    int row = get_global_id(1);

    if (col >= width || row >= height) return;

    int idx = row * width + col;
    int total = width * height;

    float is_valid = src_map[2 * total + idx];

    if (is_valid > 0.5f) {
        dst_map[0 * total + idx] = src_map[0 * total + idx];
        dst_map[1 * total + idx] = src_map[1 * total + idx];
        dst_map[2 * total + idx] = 1.0f;
        return;
    }

    float sum_elevation = 0.0f;
    float sum_weight = 0.0f;
    int found = 0;

    for (int dr = -kernel_radius; dr <= kernel_radius; dr++) {
        for (int dc = -kernel_radius; dc <= kernel_radius; dc++) {
            if (dr == 0 && dc == 0) continue;

            int neighbor_row = row + dr;
            int neighbor_col = col + dc;

            if (neighbor_row >= 0 && neighbor_row < height &&
                neighbor_col >= 0 && neighbor_col < width) {

                int neighbor_idx = neighbor_row * width + neighbor_col;
                float neighbor_valid = src_map[2 * total + neighbor_idx];

                if (neighbor_valid > 0.5f) {
                    float dist_pixels = sqrt((float)(dr * dr + dc * dc));
                    float dist_meters = dist_pixels * resolution;
                    if (dist_meters > max_inpaint_distance) continue;  // 用米距离和米阈值比较

                    float neighbor_elevation = src_map[0 * total + neighbor_idx];
                    float weight = 1.0f / (dist_meters + 0.01f);

                    sum_elevation += neighbor_elevation * weight;
                    sum_weight += weight;
                    found = 1;
                }
            }
        }
    }

    if (sum_weight > 1e-6f) {
        dst_map[0 * total + idx] = sum_elevation / sum_weight;
        dst_map[1 * total + idx] = 0.5f;
        dst_map[2 * total + idx] = 1.0f;
    } else {
        dst_map[0 * total + idx] = src_map[0 * total + idx];
        dst_map[1 * total + idx] = src_map[1 * total + idx];
        dst_map[2 * total + idx] = 0.0f;
    }
}

// =====================================================================
// 8. 形态学侵蚀 Kernel（带阈值的安全缓冲区）
// =====================================================================
__kernel void morphological_erosion_kernel(
    __global const float* traversability_map,   // 输入：可通行性图
    __global float* erosion_map,                // 输出：侵蚀后的可通行性
    const int width,
    const int height,
    const int erosion_radius,                   // 侵蚀半径（像素）
    const float safety_threshold,               // 安全阈值 (0~1)
    const float erosion_strength                // 侵蚀强度 (0~1)
)
{
    int col = get_global_id(0);
    int row = get_global_id(1);
    
    if (col >= width || row >= height) return;
    
    int idx = row * width + col;
    
    float current_trav = traversability_map[idx];
    
    // ===== 在邻域内取最小值（侵蚀）=====
    float min_trav = current_trav;
    int valid_neighbors = 0;
    
    for (int dr = -erosion_radius; dr <= erosion_radius; dr++) {
        for (int dc = -erosion_radius; dc <= erosion_radius; dc++) {
            // 圆形邻域
            if (dr * dr + dc * dc > erosion_radius * erosion_radius) continue;
            
            int neighbor_row = row + dr;
            int neighbor_col = col + dc;
            
            if (neighbor_row >= 0 && neighbor_row < height && 
                neighbor_col >= 0 && neighbor_col < width) {
                
                int neighbor_idx = neighbor_row * width + neighbor_col;
                float neighbor_trav = traversability_map[neighbor_idx];
                
                min_trav = fmin(min_trav, neighbor_trav);
                valid_neighbors++;
            }
        }
    }
    
    // ===== 基于阈值的分级侵蚀 =====
    float eroded_trav;
    
    if (current_trav < safety_threshold) {
        // 情况 A：当前栅格已经低于安全阈值
        // 强烈侵蚀：取邻域最小值
        eroded_trav = min_trav;
    } else if (min_trav < safety_threshold) {
        // 情况 B：邻域内有低于阈值的栅格
        // 中等侵蚀：线性插值
        float blend_factor = erosion_strength;
        eroded_trav = current_trav * (1.0f - blend_factor) + min_trav * blend_factor;
    } else {
        // 情况 C：邻域内全部高于阈值
        // 轻微侵蚀：只做小幅降低
        float light_erosion = current_trav * (1.0f - erosion_strength * 0.1f);
        eroded_trav = fmin(light_erosion, current_trav);
    }
    
    erosion_map[idx] = eroded_trav;
}

// =====================================================================
// 9. 视野清理 Kernel（清除动态障碍残影）
// =====================================================================
__kernel void visibility_cleanup_kernel(
    __global float* elevation_map,
    const float sensor_x,
    const float sensor_y,
    const float sensor_z,
    const float resolution,
    const int width,
    const int height,
    const float max_ray_length,
    const float cleanup_threshold,      // 方差阈值：超过此值才考虑清理
    const float invalidation_factor     // 清理强度
)
{
    int col = get_global_id(0);
    int row = get_global_id(1);

    if (col >= width || row >= height) return;

    int idx = row * width + col;
    int total = width * height;

    // 传感器在地图坐标系中的像素位置
    float sensor_col_f = sensor_x / resolution + width  * 0.5f;
    float sensor_row_f = sensor_y / resolution + height * 0.5f;

    float dc = col - sensor_col_f;
    float dr = row - sensor_row_f;
    float distance_pixels = sqrt(dc * dc + dr * dr);
    float distance_meters = distance_pixels * resolution;

    // 只处理传感器有效范围内的栅格
    if (distance_meters > max_ray_length) return;

    float current_variance = elevation_map[1 * total + idx];
    float is_valid = elevation_map[2 * total + idx];

    // 只清理：有效 AND 方差已经较高（说明长时间没被更新，可能是残影）
    if (is_valid > 0.5f && current_variance > cleanup_threshold) {
      float distance_factor = distance_meters / max_ray_length;
      float new_variance = current_variance * (1.0f + invalidation_factor * distance_factor);
      new_variance = fmin(new_variance, 10.0f);
      elevation_map[1 * total + idx] = new_variance;

      // 超过硬阈值时直接标无效
      if (new_variance > 8.0f) {
        elevation_map[2 * total + idx] = 0.0f;
      }
    }
}

// =====================================================================
// 10. 最小值滤波 Kernel（抑制孤立高点）
// =====================================================================
__kernel void min_filter_kernel(
    __global const float* src_map,      // 输入：原始高程图
    __global float* dst_map,            // 输出：滤波后的高程图
    const int width,
    const int height,
    const int filter_radius,             // 滤波半径（像素）
    const float spike_threshold      // ← 新增：孤立高点阈值（米），超过才滤
)
{
    int col = get_global_id(0);
    int row = get_global_id(1);
    
    if (col >= width || row >= height) return;
    
    int idx = row * width + col;
    int total = width * height;
    
    float is_valid = src_map[2 * total + idx];
    if (is_valid < 0.5f) {
        dst_map[0 * total + idx] = src_map[0 * total + idx];
        dst_map[1 * total + idx] = src_map[1 * total + idx];
        dst_map[2 * total + idx] = 0.0f;
        return;
    }
    
    float center_elevation = src_map[0 * total + idx];

    // 收集邻域有效高程
    float sum_neighbor = 0.0f;
    int   cnt_neighbor = 0;
    float min_elevation = center_elevation;

    for (int dr = -filter_radius; dr <= filter_radius; dr++) {
        for (int dc = -filter_radius; dc <= filter_radius; dc++) {
            if (dr == 0 && dc == 0) continue;
            int nr = row + dr;
            int nc = col + dc;
            if (nr < 0 || nr >= height || nc < 0 || nc >= width) continue;

            int nidx = nr * width + nc;
            if (src_map[2 * total + nidx] < 0.5f) continue;

            float ne = src_map[0 * total + nidx];
            sum_neighbor += ne;
            cnt_neighbor++;
            min_elevation = fmin(min_elevation, ne);
        }
    }

    if (cnt_neighbor == 0) {
        // 孤立点，没有邻居，保持原值
        dst_map[0 * total + idx] = center_elevation;
        dst_map[1 * total + idx] = src_map[1 * total + idx];
        dst_map[2 * total + idx] = 1.0f;
        return;
    }

    float mean_neighbor = sum_neighbor / cnt_neighbor;

    // 只有当中心高程显著高于邻居均值时（孤立高点），才替换为邻域最小值
    if (center_elevation - mean_neighbor > spike_threshold) {
        dst_map[0 * total + idx] = min_elevation;
    } else {
        dst_map[0 * total + idx] = center_elevation;   // 保持原值
    }

    dst_map[1 * total + idx] = src_map[1 * total + idx];
    dst_map[2 * total + idx] = 1.0f;
}

// =====================================================================
// 11. 漂移补偿 Kernel（全局高度修正）
// =====================================================================
__kernel void drift_compensation_kernel(
    __global float* elevation_map,      // 输入/输出：高程图
    const int width,
    const int height,
    const float drift_offset            // 漂移偏移量（米）
)
{
    int col = get_global_id(0);
    int row = get_global_id(1);
    
    if (col >= width || row >= height) return;
    
    int idx = row * width + col;
    int total = width * height;
    
    float is_valid = elevation_map[2 * total + idx];
    if (is_valid > 0.5f) {
        // 对有效栅格的高程值进行修正
        elevation_map[0 * total + idx] -= drift_offset;
    }
}