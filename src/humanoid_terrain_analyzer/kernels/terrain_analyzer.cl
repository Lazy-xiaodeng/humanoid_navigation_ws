/**
 * @file terrain_analyzer.cl
 * @brief 地形分析 OpenCL Kernel 文件
 *
 * 包含两个 kernel：
 *   1. median_filter_3x3  - 对 ROI 做 3x3 中值滤波
 *   2. residual_squared   - 计算拟合平面的残差平方
 *
 * 数据格式：
 *   - ROI 以行优先（row-major）的一维 float 数组存储
 *   - 无效值用 NaN 表示
 *   - 坐标约定：第 0 维 = row，第 1 维 = col
 */

/**
 * @brief 3x3 中值滤波
 *
 * 对每个像素，采集 3x3 邻域内的有效（非 NaN）值，
 * 排序后取中位数作为输出。
 * 若邻域内全是 NaN，则输出 NaN。
 *
 * @param src   输入 ROI（行优先 float 数组）
 * @param dst   输出滤波结果
 * @param rows  ROI 行数
 * @param cols  ROI 列数
 */
__kernel void median_filter_3x3(
    __global const float* src,
    __global float*       dst,
    const int             rows,
    const int             cols
)
{
    /* 每个工作项处理一个像素 */
    int r = get_global_id(0);
    int c = get_global_id(1);

    if (r >= rows || c >= cols) {
        return;
    }

    /* 收集 3x3 邻域内的有效值，最多 9 个 */
    float vals[9];
    int   count = 0;

    for (int dr = -1; dr <= 1; ++dr) {
        for (int dc = -1; dc <= 1; ++dc) {
            int nr = r + dr;
            int nc = c + dc;

            /* 边界检查 */
            if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) {
                continue;
            }

            float v = src[nr * cols + nc];

            /* 跳过 NaN */
            if (!isnan(v)) {
                vals[count++] = v;
            }
        }
    }

    /* 邻域内全是无效点，输出 NaN */
    if (count == 0) {
        dst[r * cols + c] = NAN;
        return;
    }

    /* 插入排序（最多 9 个元素，开销可忽略）*/
    for (int i = 1; i < count; ++i) {
        float key = vals[i];
        int   j   = i - 1;
        while (j >= 0 && vals[j] > key) {
            vals[j + 1] = vals[j];
            --j;
        }
        vals[j + 1] = key;
    }

    /* 取中位数 */
    dst[r * cols + c] = vals[count / 2];
}

/**
 * @brief 计算每个像素相对于拟合平面的残差平方
 *
 * 拟合平面模型：z_fit = a * (col * resolution) + b * (row * resolution) + c
 * （注意：x 对应 col * resolution，y 对应 row * resolution，单位：米）
 * 残差平方：(z - z_fit)^2
 *
 * 无效像素（NaN）的残差输出也为 NaN，由 CPU 端跳过。
 *
 * @param roi       滤波后的 ROI（行优先 float 数组）
 * @param residuals 输出残差平方数组
 * @param rows      ROI 行数
 * @param cols      ROI 列数
 * @param a         平面参数 a（x/col 方向梯度）
 * @param b         平面参数 b（y/row 方向梯度）
 * @param c         平面参数 c（截距）
 */
__kernel void residual_squared(
    __global const float* roi,
    __global float*       residuals,
    const int             rows,
    const int             cols,
    const float           a,
    const float           b,
    const float           c,
    const float           resolution
)
{
    int r   = get_global_id(0);
    int col = get_global_id(1);

    if (r >= rows || col >= cols) {
        return;
    }

    int idx = r * cols + col;
    float z = roi[idx];

    if (isnan(z)) {
        residuals[idx] = NAN;
        return;
    }

    /* 拟合平面上对应点的 z 值 */
    float x      = (float)col * resolution;  
    float y      = (float)r   * resolution;  
    float z_fit  = a * x + b * y + c;

    /* 残差平方 */
    float res    = z - z_fit;
    residuals[idx] = res * res;
}