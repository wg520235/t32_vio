/**
 * @file frontend.c
 * @brief 视觉前端实现 - FAST + KLT
 */

#include "frontend.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

/* ============================================================================
 * 静态变量
 * ============================================================================ */

static t32vio_frame_t s_prev_frame;
static t32vio_frame_t s_curr_frame;
static uint32_t s_feature_id_counter = 0;
static uint32_t s_frame_counter = 0;
static bool s_initialized = false;

// 配置参数
static float s_fast_threshold = 20.0f;
static uint32_t s_max_features = T32VIO_MAX_FEATURES;
static uint32_t s_min_features = T32VIO_MIN_FEATURES;

// 关键帧判断阈值
static float s_kf_translation_threshold = 0.05f;  // 5cm
static float s_kf_rotation_threshold = 0.087f;    // 5度
static float s_kf_time_threshold = 0.1f;          // 100ms

/* ============================================================================
 * 静态函数声明
 * ============================================================================ */

static void build_pyramid_level(const uint8_t *src, uint8_t *dst,
                                uint32_t src_w, uint32_t src_h,
                                uint32_t dst_w, uint32_t dst_h);
static float corner_score(const uint8_t *img, uint32_t width, int x, int y);
static void track_features(void);
static void detect_new_features(void);

/* ============================================================================
 * API实现
 * ============================================================================ */

int t32vio_frontend_init(void) {
    memset(&s_prev_frame, 0, sizeof(s_prev_frame));
    memset(&s_curr_frame, 0, sizeof(s_curr_frame));
    s_feature_id_counter = 0;
    s_frame_counter = 0;
    s_initialized = true;
    return 0;
}

void t32vio_frontend_deinit(void) {
    s_initialized = false;
}

int t32vio_frontend_process(const uint8_t *image, t32vio_frame_t *frame) {
    if (!s_initialized || image == NULL || frame == NULL) {
        return -1;
    }

    // 交换帧缓冲
    t32vio_frame_t temp = s_prev_frame;
    s_prev_frame = s_curr_frame;
    s_curr_frame = temp;

    // 构建图像金字塔
    t32vio_build_pyramid(image, &s_curr_frame.pyramid);
    s_curr_frame.timestamp_us = 0;  // 由调用者设置
    s_curr_frame.num_features = 0;
    s_curr_frame.is_keyframe = false;

    // 第一帧：只检测特征点
    if (s_frame_counter == 0) {
        detect_new_features();
        s_curr_frame.is_keyframe = true;
    } else {
        // 跟踪上一帧的特征点
        track_features();
        
        // 补充特征点
        if (s_curr_frame.num_features < s_min_features) {
            detect_new_features();
        }
        
        // 判断是否需要关键帧
        s_curr_frame.is_keyframe = t32vio_frontend_need_keyframe(&s_curr_frame, &s_prev_frame);
    }

    // 复制输出
    *frame = s_curr_frame;
    s_frame_counter++;
    
    return 0;
}

bool t32vio_frontend_need_keyframe(const t32vio_frame_t *curr, const t32vio_frame_t *prev) {
    if (prev->num_features == 0) return true;
    
    // 1. 跟踪点数量不足
    if (curr->num_features < s_min_features) return true;
    
    // 2. 时间间隔
    float dt = (curr->timestamp_us - prev->timestamp_us) * 1e-6f;
    if (dt > s_kf_time_threshold) return true;
    
    // 3. 计算平均视差
    float avg_parallax = 0;
    int count = 0;
    
    for (uint32_t i = 0; i < curr->num_features; i++) {
        if (!curr->features[i].tracked) continue;
        
        // 找到上一帧对应点
        for (uint32_t j = 0; j < prev->num_features; j++) {
            if (prev->features[j].id == curr->features[i].id) {
                float du = curr->features[i].u - prev->features[j].u;
                float dv = curr->features[i].v - prev->features[j].v;
                avg_parallax += sqrtf(du * du + dv * dv);
                count++;
                break;
            }
        }
    }
    
    if (count > 0) {
        avg_parallax /= count;
        // 视差 > 10像素视为足够运动
        if (avg_parallax > 10.0f) return true;
    }
    
    return false;
}

uint32_t t32vio_frontend_tracked_count(void) {
    if (!s_initialized) return 0;
    
    uint32_t count = 0;
    for (uint32_t i = 0; i < s_curr_frame.num_features; i++) {
        if (s_curr_frame.features[i].tracked) count++;
    }
    return count;
}

float t32vio_frontend_average_parallax(void) {
    if (!s_initialized || s_prev_frame.num_features == 0) return 0;
    
    float total_parallax = 0;
    int count = 0;
    
    for (uint32_t i = 0; i < s_curr_frame.num_features; i++) {
        if (!s_curr_frame.features[i].tracked) continue;
        
        for (uint32_t j = 0; j < s_prev_frame.num_features; j++) {
            if (s_prev_frame.features[j].id == s_curr_frame.features[i].id) {
                float du = s_curr_frame.features[i].u - s_prev_frame.features[j].u;
                float dv = s_curr_frame.features[i].v - s_prev_frame.features[j].v;
                total_parallax += sqrtf(du * du + dv * dv);
                count++;
                break;
            }
        }
    }
    
    return (count > 0) ? (total_parallax / count) : 0;
}

/* ============================================================================
 * 图像金字塔
 * ============================================================================ */

void t32vio_build_pyramid(const uint8_t *src, t32vio_pyramid_t *pyramid) {
    // Level 0: 原始图像
    pyramid->data[0] = (uint8_t *)src;  // 注意：这里假设src在调用期间有效
    pyramid->width[0] = T32VIO_PYRAMID_L0_W;
    pyramid->height[0] = T32VIO_PYRAMID_L0_H;
    pyramid->stride[0] = T32VIO_PYRAMID_L0_W;
    
    // Level 1: 1/2 分辨率
    static uint8_t level1[T32VIO_PYRAMID_L1_W * T32VIO_PYRAMID_L1_H];
    build_pyramid_level(src, level1, 
                        T32VIO_PYRAMID_L0_W, T32VIO_PYRAMID_L0_H,
                        T32VIO_PYRAMID_L1_W, T32VIO_PYRAMID_L1_H);
    pyramid->data[1] = level1;
    pyramid->width[1] = T32VIO_PYRAMID_L1_W;
    pyramid->height[1] = T32VIO_PYRAMID_L1_H;
    pyramid->stride[1] = T32VIO_PYRAMID_L1_W;
    
    // Level 2: 1/4 分辨率
    static uint8_t level2[T32VIO_PYRAMID_L2_W * T32VIO_PYRAMID_L2_H];
    build_pyramid_level(level1, level2,
                        T32VIO_PYRAMID_L1_W, T32VIO_PYRAMID_L1_H,
                        T32VIO_PYRAMID_L2_W, T32VIO_PYRAMID_L2_H);
    pyramid->data[2] = level2;
    pyramid->width[2] = T32VIO_PYRAMID_L2_W;
    pyramid->height[2] = T32VIO_PYRAMID_L2_H;
    pyramid->stride[2] = T32VIO_PYRAMID_L2_W;
}

static void build_pyramid_level(const uint8_t *src, uint8_t *dst,
                                uint32_t src_w, uint32_t src_h,
                                uint32_t dst_w, uint32_t dst_h) {
    // 简单的2x2平均下采样
    for (uint32_t y = 0; y < dst_h; y++) {
        for (uint32_t x = 0; x < dst_w; x++) {
            uint32_t src_x = x * 2;
            uint32_t src_y = y * 2;
            
            uint16_t sum = src[src_y * src_w + src_x];
            sum += src[src_y * src_w + src_x + 1];
            sum += src[(src_y + 1) * src_w + src_x];
            sum += src[(src_y + 1) * src_w + src_x + 1];
            
            dst[y * dst_w + x] = (uint8_t)(sum >> 2);
        }
    }
}

/* ============================================================================
 * FAST角点检测
 * ============================================================================ */

// FAST-9 检测: 圆周上连续9个像素都比中心亮/暗 threshold
static bool fast_test(const uint8_t *img, uint32_t width, int x, int y, int threshold) {
    const int16_t circle_offset[16] = {
        3, 0, width, width+1, width+2, width+3, width*2+3, width*3+3,
        width*3+2, width*3+1, width*3, width*2, width+0, 0, -width, -width+3
    };
    
    const uint8_t *center = &img[y * width + x];
    int16_t center_val = *center;
    
    // 快速测试：先检查1,5,9,13四个点
    int16_t val0 = center_val + threshold;
    int16_t val1 = center_val - threshold;
    
    int bright = 0, dark = 0;
    
    if (center[circle_offset[0]] > val0) bright++;
    else if (center[circle_offset[0]] < val1) dark++;
    
    if (center[circle_offset[4]] > val0) bright++;
    else if (center[circle_offset[4]] < val1) dark++;
    
    if (center[circle_offset[8]] > val0) bright++;
    else if (center[circle_offset[8]] < val1) dark++;
    
    if (center[circle_offset[12]] > val0) bright++;
    else if (center[circle_offset[12]] < val1) dark++;
    
    if (bright < 3 && dark < 3) return false;
    
    // 完整测试
    for (int i = 0; i < 16; i++) {
        int16_t pixel = center[circle_offset[i]];
        if (pixel > val0) {
            int count = 1;
            for (int j = 1; j < 9; j++) {
                if (center[circle_offset[(i+j)&15]] > val0) count++;
                else break;
            }
            for (int j = 1; j < 9 && count < 9; j++) {
                if (center[circle_offset[(i-j+16)&15]] > val0) count++;
                else break;
            }
            if (count >= 9) return true;
        } else if (pixel < val1) {
            int count = 1;
            for (int j = 1; j < 9; j++) {
                if (center[circle_offset[(i+j)&15]] < val1) count++;
                else break;
            }
            for (int j = 1; j < 9 && count < 9; j++) {
                if (center[circle_offset[(i-j+16)&15]] < val1) count++;
                else break;
            }
            if (count >= 9) return true;
        }
    }
    
    return false;
}

int t32vio_fast_detect(const uint8_t *image, uint32_t width, uint32_t height,
                       float threshold, t32vio_feature_t *features, int max_features) {
    int num_features = 0;
    int t = (int)threshold;
    
    // 边界处理：FAST需要3像素边界
    for (uint32_t y = 3; y < height - 3 && num_features < max_features; y += 2) {
        for (uint32_t x = 3; x < width - 3 && num_features < max_features; x += 2) {
            if (fast_test(image, width, x, y, t)) {
                features[num_features].u = (float)x;
                features[num_features].v = (float)y;
                features[num_features].score = corner_score(image, width, x, y);
                features[num_features].level = 0;
                features[num_features].tracked = false;
                num_features++;
            }
        }
    }
    
    return num_features;
}

static float corner_score(const uint8_t *img, uint32_t width, int x, int y) {
    // 简单的响应分数：圆周像素与中心的绝对差之和
    const int16_t circle_offset[16] = {
        3, 0, width, width+1, width+2, width+3, width*2+3, width*3+3,
        width*3+2, width*3+1, width*3, width*2, width+0, 0, -width, -width+3
    };
    
    const uint8_t *center = &img[y * width + x];
    int16_t center_val = *center;
    int32_t score = 0;
    
    for (int i = 0; i < 16; i++) {
        score += abs(center[circle_offset[i]] - center_val);
    }
    
    return (float)score;
}

void t32vio_grid_nms(t32vio_feature_t *features, int *num_features,
                     uint32_t width, uint32_t height, uint32_t grid_size) {
    if (*num_features <= 0) return;
    
    int num_grids_x = (width + grid_size - 1) / grid_size;
    int num_grids_y = (height + grid_size - 1) / grid_size;
    
    // 每个网格保留分数最高的特征点
    // 简单实现：按分数排序，然后网格去重
    
    // 冒泡排序（小数据量够用）
    for (int i = 0; i < *num_features - 1; i++) {
        for (int j = 0; j < *num_features - i - 1; j++) {
            if (features[j].score < features[j+1].score) {
                t32vio_feature_t temp = features[j];
                features[j] = features[j+1];
                features[j+1] = temp;
            }
        }
    }
    
    // 网格标记
    uint8_t *grid_occupied = (uint8_t *)calloc(num_grids_x * num_grids_y, sizeof(uint8_t));
    
    int kept = 0;
    for (int i = 0; i < *num_features; i++) {
        int gx = (int)features[i].u / grid_size;
        int gy = (int)features[i].v / grid_size;
        int idx = gy * num_grids_x + gx;
        
        if (!grid_occupied[idx]) {
            grid_occupied[idx] = 1;
            features[kept++] = features[i];
        }
    }
    
    free(grid_occupied);
    *num_features = kept;
}

/* ============================================================================
 * KLT光流跟踪
 * ============================================================================ */

static void track_features(void) {
    if (s_prev_frame.num_features == 0) return;
    
    uint8_t status[T32VIO_MAX_FEATURES];
    
    // 金字塔LK跟踪
    t32vio_klt_track(&s_prev_frame.pyramid, &s_curr_frame.pyramid,
                     s_prev_frame.features, s_prev_frame.num_features,
                     s_curr_frame.features, (int *)&s_curr_frame.num_features,
                     status);
    
    // 逆向验证
    float back_errors[T32VIO_MAX_FEATURES];
    t32vio_klt_verify(&s_prev_frame.pyramid, &s_curr_frame.pyramid,
                      s_curr_frame.features, s_curr_frame.num_features,
                      back_errors);
    
    // 标记跟踪状态
    for (uint32_t i = 0; i < s_curr_frame.num_features; i++) {
        if (status[i] && back_errors[i] < 1.0f) {
            s_curr_frame.features[i].tracked = true;
            s_curr_frame.features[i].id = s_prev_frame.features[i].id;
        } else {
            s_curr_frame.features[i].tracked = false;
            s_curr_frame.features[i].id = -1;
        }
    }
    
    // 压缩数组，移除未跟踪的点
    uint32_t kept = 0;
    for (uint32_t i = 0; i < s_curr_frame.num_features; i++) {
        if (s_curr_frame.features[i].tracked) {
            s_curr_frame.features[kept++] = s_curr_frame.features[i];
        }
    }
    s_curr_frame.num_features = kept;
}

void t32vio_klt_track(const t32vio_pyramid_t *prev_pyr, const t32vio_pyramid_t *curr_pyr,
                      const t32vio_feature_t *prev_features, int num_prev,
                      t32vio_feature_t *curr_features, int *num_curr,
                      uint8_t *status) {
    *num_curr = num_prev;
    
    for (int i = 0; i < num_prev; i++) {
        status[i] = 0;
        
        // 从顶层开始估计
        float u = prev_features[i].u;
        float v = prev_features[i].v;
        float gu = 0, gv = 0;  // 全局位移估计
        
        for (int level = T32VIO_PYRAMID_LEVELS - 1; level >= 0; level--) {
            float scale = 1.0f / (1 << level);
            float ul = u * scale;
            float vl = v * scale;
            
            // 在当前层进行LK迭代
            float dx = gu * scale;
            float dy = gv * scale;
            
            const uint8_t *prev_img = prev_pyr->data[level];
            const uint8_t *curr_img = curr_pyr->data[level];
            uint32_t w = curr_pyr->width[level];
            uint32_t h = curr_pyr->height[level];
            
            int half_patch = T32VIO_PATCH_SIZE / 2;
            
            // 检查边界
            if (ul < half_patch || ul >= w - half_patch ||
                vl < half_patch || vl >= h - half_patch) {
                break;
            }
            
            // LK迭代
            for (int iter = 0; iter < T32VIO_MAX_ITERATIONS; iter++) {
                // 计算图像梯度
                float A[2][2] = {{0, 0}, {0, 0}};
                float b[2] = {0, 0};
                
                for (int py = -half_patch; py < half_patch; py++) {
                    for (int px = -half_patch; px < half_patch; px++) {
                        int prev_x = (int)(ul + px);
                        int prev_y = (int)(vl + py);
                        int curr_x = (int)(ul + dx + px);
                        int curr_y = (int)(vl + dy + py);
                        
                        // 边界检查
                        if (curr_x < 1 || curr_x >= (int)w - 1 ||
                            curr_y < 1 || curr_y >= (int)h - 1) {
                            continue;
                        }
                        
                        // 计算梯度 (中心差分)
                        float Ix = (prev_img[prev_y * w + prev_x + 1] - 
                                   prev_img[prev_y * w + prev_x - 1]) * 0.5f;
                        float Iy = (prev_img[(prev_y + 1) * w + prev_x] - 
                                   prev_img[(prev_y - 1) * w + prev_x]) * 0.5f;
                        
                        // 灰度差
                        float It = (float)prev_img[prev_y * w + prev_x] - 
                                   (float)curr_img[curr_y * w + curr_x];
                        
                        A[0][0] += Ix * Ix;
                        A[0][1] += Ix * Iy;
                        A[1][0] += Ix * Iy;
                        A[1][1] += Iy * Iy;
                        
                        b[0] += Ix * It;
                        b[1] += Iy * It;
                    }
                }
                
                // 解线性方程 A * d = b
                float det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
                if (fabsf(det) < 1e-6f) break;  // 奇异矩阵
                
                float d[2];
                d[0] = (A[1][1] * b[0] - A[0][1] * b[1]) / det;
                d[1] = (A[0][0] * b[1] - A[1][0] * b[0]) / det;
                
                dx += d[0];
                dy += d[1];
                
                // 收敛判断
                if (d[0] * d[0] + d[1] * d[1] < T32VIO_EPSILON * T32VIO_EPSILON) {
                    break;
                }
            }
            
            // 更新全局位移估计
            gu = dx / scale;
            gv = dy / scale;
        }
        
        // 输出结果
        curr_features[i].u = u + gu;
        curr_features[i].v = v + gv;
        curr_features[i].level = 0;
        curr_features[i].score = prev_features[i].score;
        
        // 检查最终位置是否有效
        if (curr_features[i].u >= 3 && curr_features[i].u < T32VIO_IMAGE_WIDTH - 3 &&
            curr_features[i].v >= 3 && curr_features[i].v < T32VIO_IMAGE_HEIGHT - 3) {
            status[i] = 1;
        }
    }
}

void t32vio_klt_verify(const t32vio_pyramid_t *prev_pyr, const t32vio_pyramid_t *curr_pyr,
                       const t32vio_feature_t *curr_features, int num_curr,
                       float *back_errors) {
    // 简化的逆向验证：计算反向光流误差
    for (int i = 0; i < num_curr; i++) {
        back_errors[i] = 0;
        
        // 在当前帧位置提取patch
        float u = curr_features[i].u;
        float v = curr_features[i].v;
        
        // 简化：只计算单层误差
        const uint8_t *prev_img = prev_pyr->data[0];
        const uint8_t *curr_img = curr_pyr->data[0];
        uint32_t w = T32VIO_IMAGE_WIDTH;
        
        int half_patch = 4;  // 小patch快速验证
        float error = 0;
        int count = 0;
        
        for (int py = -half_patch; py < half_patch; py++) {
            for (int px = -half_patch; px < half_patch; px++) {
                int x = (int)(u + px);
                int y = (int)(v + py);
                
                if (x < 0 || x >= (int)w || y < 0 || y >= (int)T32VIO_IMAGE_HEIGHT) {
                    continue;
                }
                
                // 这里简化处理，实际应该反向跟踪
                // 用灰度差作为近似误差
                float diff = (float)curr_img[y * w + x] - (float)prev_img[y * w + x];
                error += diff * diff;
                count++;
            }
        }
        
        if (count > 0) {
            back_errors[i] = sqrtf(error / count);
        }
    }
}

/* ============================================================================
 * 特征点管理
 * ============================================================================ */

static void detect_new_features(void) {
    int need = s_max_features - s_curr_frame.num_features;
    if (need <= 0) return;
    
    // 临时缓冲区
    t32vio_feature_t new_features[T32VIO_MAX_FEATURES];
    
    // 自适应阈值
    float threshold = s_fast_threshold;
    int num_detected = 0;
    
    while (threshold > 5.0f && num_detected < need) {
        num_detected = t32vio_fast_detect(
            s_curr_frame.pyramid.data[0],
            T32VIO_IMAGE_WIDTH, T32VIO_IMAGE_HEIGHT,
            threshold, new_features, need
        );
        threshold *= 0.8f;
    }
    
    if (num_detected == 0) return;
    
    // 网格NMS，避免与已有特征点冲突
    // 标记已有特征点占据的网格
    uint8_t grid_occupied[8 * 6] = {0};  // 320/40=8, 240/40=6
    const uint32_t grid_size = T32VIO_FEATURE_GRID_SIZE;
    
    for (uint32_t i = 0; i < s_curr_frame.num_features; i++) {
        int gx = (int)s_curr_frame.features[i].u / grid_size;
        int gy = (int)s_curr_frame.features[i].v / grid_size;
        if (gx >= 0 && gx < 8 && gy >= 0 && gy < 6) {
            grid_occupied[gy * 8 + gx] = 1;
        }
    }
    
    // 添加新特征点
    for (int i = 0; i < num_detected && s_curr_frame.num_features < s_max_features; i++) {
        int gx = (int)new_features[i].u / grid_size;
        int gy = (int)new_features[i].v / grid_size;
        
        if (gx >= 0 && gx < 8 && gy >= 0 && gy < 6 && !grid_occupied[gy * 8 + gx]) {
            grid_occupied[gy * 8 + gx] = 1;
            
            new_features[i].id = s_feature_id_counter++;
            new_features[i].tracked = true;
            s_curr_frame.features[s_curr_frame.num_features++] = new_features[i];
        }
    }
}
