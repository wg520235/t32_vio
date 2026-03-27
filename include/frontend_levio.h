/**
 * @file frontend_levio.h
 * @brief LEVIO增强的视觉前端 - 关键帧视差判断
 */

#ifndef FRONTEND_LEVIO_H
#define FRONTEND_LEVIO_H

#include "t32vio.h"

namespace t32vio {

/* ============================================================================
 * LEVIO风格的关键帧判断参数
 * ============================================================================ */

// 视差阈值 (像素，归一化到焦距1.0)
// LEVIO使用: 0.05 * focal_length_pixels
// 对于T32 (fx≈240): 0.05 * 240 = 12像素
constexpr float LEVIO_PARALLAX_THRESHOLD = 12.0f;

// 最小跟踪点比例 (LEVIO: 如果跟踪点少于30%则触发关键帧)
constexpr float LEVIO_MIN_TRACK_RATIO = 0.3f;

// 最大关键帧间隔时间 (秒)
constexpr float LEVIO_MAX_KF_INTERVAL = 0.5f;

// 最小关键帧间隔时间 (秒) - 避免过于密集
constexpr float LEVIO_MIN_KF_INTERVAL = 0.05f;

/* ============================================================================
 * 增强的关键帧判断函数
 * ============================================================================ */

/**
 * @brief 计算两帧之间的平均视差 (LEVIO方法)
 * 
 * @param curr 当前帧
 * @param prev 上一帧
 * @param K 相机内参矩阵
 * @return float 平均视差 (像素)
 */
float levioCalculateParallax(const FeatureFrame* curr, 
                              const FeatureFrame* prev,
                              const CameraIntrinsics* K);

/**
 * @brief LEVIO风格的关键帧判断
 * 
 * 综合以下因素:
 * 1. 视差阈值 (主要因素)
 * 2. 跟踪点数量
 * 3. 时间间隔
 * 
 * @param curr 当前帧
 * @param prev 上一关键帧
 * @param K 相机内参
 * @param curr_time_us 当前时间戳
 * @param last_kf_time_us 上一关键帧时间戳
 * @return true 需要创建关键帧
 * @return false 不需要
 */
bool levioNeedKeyframe(const FeatureFrame* curr,
                        const FeatureFrame* prev,
                        const CameraIntrinsics* K,
                        uint64_t curr_time_us,
                        uint64_t last_kf_time_us);

/**
 * @brief 计算跟踪成功率
 */
float levioTrackSuccessRate(const FeatureFrame* curr, const FeatureFrame* prev);

} // namespace t32vio

#endif /* FRONTEND_LEVIO_H */
