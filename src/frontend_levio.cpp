/**
 * @file frontend_levio.cpp
 * @brief LEVIO增强的视觉前端实现
 */

#include "frontend_levio.h"
#include <cmath>

namespace t32vio {

/* ============================================================================
 * 视差计算 - LEVIO方法
 * ============================================================================ */

float levioCalculateParallax(const FeatureFrame* curr, 
                              const FeatureFrame* prev,
                              const CameraIntrinsics* K) {
    if (curr == nullptr || prev == nullptr || K == nullptr) {
        return 0.0f;
    }
    
    if (curr->num_features == 0 || prev->num_features == 0) {
        return 0.0f;
    }
    
    float total_parallax = 0.0f;
    int match_count = 0;
    
    // 遍历当前帧的特征点
    for (uint32_t i = 0; i < curr->num_features; i++) {
        if (!curr->features[i].tracked) {
            continue;
        }
        
        // 在上一帧中查找对应的特征点
        for (uint32_t j = 0; j < prev->num_features; j++) {
            if (prev->features[j].id == curr->features[i].id) {
                // 计算像素位移
                float du = curr->features[i].u - prev->features[j].u;
                float dv = curr->features[i].v - prev->features[j].v;
                
                // LEVIO方法: 归一化到焦距 (消除分辨率影响)
                // parallax_normalized = parallax_pixel / focal_length
                du /= K->fx;
                dv /= K->fy;
                
                float parallax = std::sqrt(du * du + dv * dv);
                total_parallax += parallax;
                match_count++;
                break;
            }
        }
    }
    
    if (match_count == 0) {
        return 0.0f;
    }
    
    // 返回平均视差 (归一化单位)
    return total_parallax / match_count;
}

/* ============================================================================
 * 跟踪成功率计算
 * ============================================================================ */

float levioTrackSuccessRate(const FeatureFrame* curr, const FeatureFrame* prev) {
    if (curr == nullptr || prev == nullptr) {
        return 0.0f;
    }
    
    if (prev->num_features == 0) {
        return 0.0f;
    }
    
    uint32_t tracked_count = 0;
    
    for (uint32_t i = 0; i < curr->num_features; i++) {
        if (curr->features[i].tracked) {
            tracked_count++;
        }
    }
    
    return static_cast<float>(tracked_count) / static_cast<float>(prev->num_features);
}

/* ============================================================================
 * LEVIO风格关键帧判断
 * ============================================================================ */

bool levioNeedKeyframe(const FeatureFrame* curr,
                        const FeatureFrame* prev,
                        const CameraIntrinsics* K,
                        uint64_t curr_time_us,
                        uint64_t last_kf_time_us) {
    
    if (curr == nullptr || prev == nullptr || K == nullptr) {
        return true;  // 异常情况，创建关键帧
    }
    
    // 计算时间间隔
    float dt = static_cast<float>(curr_time_us - last_kf_time_us) * 1e-6f;
    
    // 1. 最小时间间隔检查 - 避免过于密集的关键帧
    if (dt < LEVIO_MIN_KF_INTERVAL) {
        return false;
    }
    
    // 2. 最大时间间隔检查 - 强制创建关键帧
    if (dt > LEVIO_MAX_KF_INTERVAL) {
        return true;
    }
    
    // 3. 跟踪点数量检查 (LEVIO: 跟踪点少于阈值)
    float track_ratio = levioTrackSuccessRate(curr, prev);
    if (track_ratio < LEVIO_MIN_TRACK_RATIO) {
        return true;
    }
    
    // 4. 视差判断 (LEVIO核心方法)
    // 计算归一化视差
    float avg_parallax = levioCalculateParallax(curr, prev, K);
    
    // LEVIO阈值: 0.05 (约5.7度视角变化)
    // 对于T32相机 (fx≈240): 12像素
    if (avg_parallax > (LEVIO_PARALLAX_THRESHOLD / K->fx)) {
        return true;
    }
    
    // 5. 特征点数量不足
    if (curr->num_features < MIN_FEATURES) {
        return true;
    }
    
    return false;
}

} // namespace t32vio
