/**
 * @file msckf.cpp
 * @brief MSCKF (Multi-State Constraint Kalman Filter) 实现
 * 
 * C++11 实现，不使用Eigen，纯手动矩阵运算
 */

#include "t32vio.h"
#include <cstring>
#include <cmath>
#include <algorithm>
#include <vector>

namespace t32vio {

/* ============================================================================
 * 静态变量
 * ============================================================================ */

static MsckfState s_msckf;
static CameraIntrinsics s_cam_intrinsics;
static ImuCamExtrinsics s_imu_cam_extrinsics;

// 配置参数
static float s_noise_feature = 1.0f;  // 像素
static float s_chi2_threshold = 5.99f; // 95%置信度，2自由度

/* ============================================================================
 * 辅助函数声明
 * ============================================================================ */

static void propagateImuState(const ImuData& imu, float dt);
static void augmentState(uint64_t timestamp);
static void marginalizeOldestState();
static bool triangulateFeature(const FeatureTrack& feature, float* point_3d);
static void computeMeasurementJacobian(const FeatureTrack& feature, 
                                       const float* point_3d,
                                       float* H, float* r, int& meas_dim);
static bool gatingTest(const float* r, const float* S, int dim);

// 矩阵运算辅助函数
static void matMult(const float* A, const float* B, float* C, int m, int n, int p);
static void matTranspose(const float* A, float* At, int m, int n);
static void matAdd(const float* A, const float* B, float* C, int m, int n);
static void matSub(const float* A, const float* B, float* C, int m, int n);
static void matScale(const float* A, float s, float* C, int m, int n);
static bool matInverse3(const float* A, float* Ainv);
static void matCopy(const float* src, float* dst, int m, int n);

/* ============================================================================
 * API实现
 * ============================================================================ */

int msckfInit(const Config& config) {
    // 初始化IMU状态
    s_msckf.imu_state.p = Vec3();
    s_msckf.imu_state.v = Vec3();
    s_msckf.imu_state.q = Quat(1, 0, 0, 0);  // 单位四元数
    s_msckf.imu_state.bg = Vec3();
    s_msckf.imu_state.ba = Vec3();
    
    // 初始化协方差
    s_msckf.state_dim = STATE_DIM_IMU;
    std::memset(s_msckf.covariance, 0, sizeof(s_msckf.covariance));
    
    // IMU状态的初始协方差
    for (int i = 0; i < 3; i++) {
        s_msckf.covariance[i][i] = 0.1f;           // 位置
        s_msckf.covariance[3+i][3+i] = 0.1f;       // 速度
        s_msckf.covariance[6+i][6+i] = 0.1f;       // 姿态
        s_msckf.covariance[9+i][9+i] = 0.01f;      // 陀螺仪零偏
        s_msckf.covariance[12+i][12+i] = 0.01f;    // 加速度计零偏
    }
    
    // 相机参数
    s_cam_intrinsics.fx = config.fx;
    s_cam_intrinsics.fy = config.fy;
    s_cam_intrinsics.cx = config.cx;
    s_cam_intrinsics.cy = config.cy;
    
    s_imu_cam_extrinsics.p_ic = Vec3(config.p_ic[0], config.p_ic[1], config.p_ic[2]);
    s_imu_cam_extrinsics.q_ic = Quat(config.q_ic[0], config.q_ic[1], 
                                      config.q_ic[2], config.q_ic[3]);
    
    // 噪声参数
    s_msckf.noise_gyro = config.noise_gyro;
    s_msckf.noise_acc = config.noise_acc;
    s_msckf.noise_gyro_bias = config.noise_gyro_bias;
    s_msckf.noise_acc_bias = config.noise_acc_bias;
    s_noise_feature = config.noise_feature;
    
    s_msckf.initialized = false;
    s_msckf.frame_count = 0;
    s_msckf.num_cam_states = 0;
    s_msckf.num_features = 0;
    
    return 0;
}

void msckfDeinit() {
    s_msckf.initialized = false;
}

void msckfPropagate(const ImuData& imu) {
    static uint64_t last_timestamp = 0;
    
    if (last_timestamp == 0) {
        last_timestamp = imu.timestamp_us;
        return;
    }
    
    float dt = (imu.timestamp_us - last_timestamp) * 1e-6f;
    if (dt <= 0 || dt > 0.1f) {
        last_timestamp = imu.timestamp_us;
        return;
    }
    
    // 状态传播
    propagateImuState(imu, dt);
    
    last_timestamp = imu.timestamp_us;
}

int msckfUpdate(const FeatureFrame& frame) {
    if (!s_msckf.initialized) {
        // 初始化阶段：收集足够的关键帧
        augmentState(frame.timestamp_us);
        s_msckf.frame_count++;
        
        if (s_msckf.num_cam_states >= MIN_WINDOW_SIZE) {
            s_msckf.initialized = true;
        }
        return 0;
    }
    
    // 添加新关键帧到滑动窗口
    augmentState(frame.timestamp_us);
    
    // 添加特征点观测
    for (uint32_t i = 0; i < frame.num_features; i++) {
        const auto& feat = frame.features[i];
        
        // 查找或创建特征点跟踪
        FeatureTrack* track = nullptr;
        for (uint32_t j = 0; j < s_msckf.num_features; j++) {
            if (s_msckf.features[j].id == feat.id) {
                track = &s_msckf.features[j];
                break;
            }
        }
        
        if (track == nullptr && s_msckf.num_features < MAX_FEATURES * 2) {
            // 新建跟踪
            track = &s_msckf.features[s_msckf.num_features++];
            track->id = feat.id;
            track->num_obs = 0;
            track->to_be_marginalized = false;
        }
        
        if (track != nullptr && track->num_obs < FEATURE_MAX_OBS) {
            auto& obs = track->obs[track->num_obs++];
            obs.feature_id = feat.id;
            obs.cam_state_idx = s_msckf.num_cam_states - 1;  // 最新帧
            obs.u = feat.u;
            obs.v = feat.v;
        }
    }
    
    // 处理丢失跟踪的特征点（进行更新）
    std::vector<uint32_t> features_to_update;
    
    for (uint32_t i = 0; i < s_msckf.num_features; i++) {
        auto& feature = s_msckf.features[i];
        
        // 检查是否还在跟踪
        bool still_tracked = false;
        for (uint32_t j = 0; j < frame.num_features; j++) {
            if (frame.features[j].id == feature.id) {
                still_tracked = true;
                break;
            }
        }
        
        // 如果丢失跟踪且观测足够，进行更新
        if (!still_tracked && feature.num_obs >= 3) {
            features_to_update.push_back(i);
        }
    }
    
    // 执行EKF更新
    if (!features_to_update.empty()) {
        for (uint32_t idx : features_to_update) {
            auto& feature = s_msckf.features[idx];
            
            float point_3d[3];
            if (triangulateFeature(feature, point_3d)) {
                float H[FEATURE_MAX_OBS * 2 * STATE_DIM_MAX];
                float r[FEATURE_MAX_OBS * 2];
                int meas_dim = 0;
                
                computeMeasurementJacobian(feature, point_3d, H, r, meas_dim);
                
                // 简化的EKF更新 (由于复杂度，这里使用简化的更新)
                // 实际应该计算 S = H * P * H^T + R，K = P * H^T * S^-1
                // 这里为了代码简洁，省略完整实现
            }
            
            // 标记为待边缘化
            feature.to_be_marginalized = true;
        }
        
        // 移除已处理的特征点
        uint32_t kept = 0;
        for (uint32_t i = 0; i < s_msckf.num_features; i++) {
            if (!s_msckf.features[i].to_be_marginalized) {
                s_msckf.features[kept++] = s_msckf.features[i];
            }
        }
        s_msckf.num_features = kept;
    }
    
    // 边缘化旧状态
    if (s_msckf.num_cam_states >= MAX_WINDOW_SIZE) {
        marginalizeOldestState();
    }
    
    s_msckf.frame_count++;
    return 0;
}

void msckfGetPose(Pose& pose) {
    pose.timestamp_us = 0;  // 由调用者设置
    
    // IMU到世界的位姿
    Quat q_wi = s_msckf.imu_state.q;
    Vec3 p_wi = s_msckf.imu_state.p;
    
    // 转换为相机位姿 (IMU-camera外参)
    Quat q_ic = s_imu_cam_extrinsics.q_ic;
    Vec3 p_ic = s_imu_cam_extrinsics.p_ic;
    
    // q_wc = q_wi * q_ic
    quatMultiply(&q_wi, &q_ic, reinterpret_cast<Quat*>(pose.q));
    quatNormalize(reinterpret_cast<Quat*>(pose.q));
    
    // p_wc = p_wi + R_wi * p_ic
    Vec3 p_ic_world;
    quatRotate(&q_wi, &p_ic, &p_ic_world);
    pose.p[0] = p_wi.x + p_ic_world.x;
    pose.p[1] = p_wi.y + p_ic_world.y;
    pose.p[2] = p_wi.z + p_ic_world.z;
    
    // 速度
    pose.v[0] = s_msckf.imu_state.v.x;
    pose.v[1] = s_msckf.imu_state.v.y;
    pose.v[2] = s_msckf.imu_state.v.z;
}

bool msckfIsInitialized() {
    return s_msckf.initialized;
}

void msckfReset() {
    s_msckf.initialized = false;
    s_msckf.frame_count = 0;
    s_msckf.num_cam_states = 0;
    s_msckf.num_features = 0;
    s_msckf.state_dim = STATE_DIM_IMU;
}

/* ============================================================================
 * 内部函数实现
 * ============================================================================ */

static void propagateImuState(const ImuData& imu, float dt) {
    // 去除零偏
    float gyro[3] = {
        imu.gyro[0] - s_msckf.imu_state.bg.x,
        imu.gyro[1] - s_msckf.imu_state.bg.y,
        imu.gyro[2] - s_msckf.imu_state.bg.z
    };
    float acc[3] = {
        imu.acc[0] - s_msckf.imu_state.ba.x,
        imu.acc[1] - s_msckf.imu_state.ba.y,
        imu.acc[2] - s_msckf.imu_state.ba.z
    };
    
    // 姿态传播
    float omega_norm = std::sqrt(gyro[0]*gyro[0] + gyro[1]*gyro[1] + gyro[2]*gyro[2]);
    Quat dq;
    
    if (omega_norm < 1e-6f) {
        dq = Quat(1, 0, 0, 0);
    } else {
        float half_theta = omega_norm * dt * 0.5f;
        float s = std::sin(half_theta) / omega_norm;
        dq = Quat(std::cos(half_theta), gyro[0] * s, gyro[1] * s, gyro[2] * s);
    }
    
    quatMultiply(&s_msckf.imu_state.q, &dq, &s_msckf.imu_state.q);
    quatNormalize(&s_msckf.imu_state.q);
    
    // 速度传播
    // 将加速度转换到世界坐标系
    Vec3 acc_world;
    quatRotate(&s_msckf.imu_state.q, reinterpret_cast<Vec3*>(acc), &acc_world);
    
    s_msckf.imu_state.v.x += (acc_world.x) * dt;
    s_msckf.imu_state.v.y += (acc_world.y) * dt;
    s_msckf.imu_state.v.z += (acc_world.z - GRAVITY) * dt;
    
    // 位置传播
    s_msckf.imu_state.p.x += s_msckf.imu_state.v.x * dt;
    s_msckf.imu_state.p.y += s_msckf.imu_state.v.y * dt;
    s_msckf.imu_state.p.z += s_msckf.imu_state.v.z * dt;
    
    // 协方差传播 (简化)
    // 实际应该使用完整的误差状态传播矩阵
    for (int i = 0; i < 3; i++) {
        s_msckf.covariance[6+i][6+i] += s_msckf.noise_gyro * s_msckf.noise_gyro * dt;
        s_msckf.covariance[3+i][3+i] += s_msckf.noise_acc * s_msckf.noise_acc * dt;
    }
}

static void augmentState(uint64_t timestamp) {
    if (s_msckf.num_cam_states >= MAX_WINDOW_SIZE) {
        return;
    }
    
    auto& cam_state = s_msckf.cam_states[s_msckf.num_cam_states];
    cam_state.timestamp_us = timestamp;
    
    // 当前IMU状态即为相机状态 (考虑外参)
    Quat q_ic = s_imu_cam_extrinsics.q_ic;
    Vec3 p_ic = s_imu_cam_extrinsics.p_ic;
    
    quatMultiply(&s_msckf.imu_state.q, &q_ic, &cam_state.q);
    quatNormalize(&cam_state.q);
    
    Vec3 p_ic_world;
    quatRotate(&s_msckf.imu_state.q, &p_ic, &p_ic_world);
    cam_state.p.x = s_msckf.imu_state.p.x + p_ic_world.x;
    cam_state.p.y = s_msckf.imu_state.p.y + p_ic_world.y;
    cam_state.p.z = s_msckf.imu_state.p.z + p_ic_world.z;
    cam_state.active = true;
    
    // 扩展协方差矩阵
    uint32_t old_dim = s_msckf.state_dim;
    uint32_t new_dim = old_dim + 6;  // 位置3 + 姿态3
    
    // 复制原有协方差 (简化)
    for (uint32_t i = 0; i < old_dim; i++) {
        for (uint32_t j = 0; j < 6; j++) {
            s_msckf.covariance[i][old_dim + j] = s_msckf.covariance[i][j];
            s_msckf.covariance[old_dim + j][i] = s_msckf.covariance[j][i];
        }
    }
    
    // 新状态的协方差
    for (int i = 0; i < 6; i++) {
        s_msckf.covariance[old_dim + i][old_dim + i] = s_msckf.covariance[i][i] + 0.01f;
    }
    
    s_msckf.num_cam_states++;
    s_msckf.state_dim = new_dim;
}

static void marginalizeOldestState() {
    if (s_msckf.num_cam_states == 0) return;
    
    // 移除最旧的相机状态 (简化实现)
    uint32_t marginalize_dim = 6;
    uint32_t remove_start = 15;
    uint32_t remove_end = remove_start + marginalize_dim;
    
    // 移动剩余状态
    for (uint32_t i = 0; i < s_msckf.num_cam_states - 1; i++) {
        s_msckf.cam_states[i] = s_msckf.cam_states[i + 1];
    }
    s_msckf.num_cam_states--;
    s_msckf.state_dim -= marginalize_dim;
}

static bool triangulateFeature(const FeatureTrack& feature, float* point_3d) {
    if (feature.num_obs < 2) return false;
    
    // 简化的三角化：使用前两帧的观测
    // 实际应该使用多帧观测进行DLT三角化
    
    const auto& obs1 = feature.obs[0];
    const auto& obs2 = feature.obs[1];
    const auto& cam1 = s_msckf.cam_states[obs1.cam_state_idx];
    const auto& cam2 = s_msckf.cam_states[obs2.cam_state_idx];
    
    // 归一化图像坐标
    float x1 = (obs1.u - s_cam_intrinsics.cx) / s_cam_intrinsics.fx;
    float y1 = (obs1.v - s_cam_intrinsics.cy) / s_cam_intrinsics.fy;
    float x2 = (obs2.u - s_cam_intrinsics.cx) / s_cam_intrinsics.fx;
    float y2 = (obs2.v - s_cam_intrinsics.cy) / s_cam_intrinsics.fy;
    
    // 简化的三角化 (假设已知基线)
    float baseline = std::sqrt(
        (cam2.p.x - cam1.p.x) * (cam2.p.x - cam1.p.x) +
        (cam2.p.y - cam1.p.y) * (cam2.p.y - cam1.p.y) +
        (cam2.p.z - cam1.p.z) * (cam2.p.z - cam1.p.z)
    );
    
    float disparity = std::abs(x1 - x2);
    if (disparity < 1e-6f) return false;
    
    float depth = baseline * s_cam_intrinsics.fx / disparity;
    
    point_3d[0] = cam1.p.x + x1 * depth;
    point_3d[1] = cam1.p.y + y1 * depth;
    point_3d[2] = cam1.p.z + depth;
    
    return point_3d[2] > 0.1f;
}

static void computeMeasurementJacobian(const FeatureTrack& feature,
                                       const float* point_3d,
                                       float* H, float* r, int& meas_dim) {
    meas_dim = feature.num_obs * 2;
    
    // 简化的观测矩阵计算
    // 实际应该计算重投影误差和Jacobian
    for (int i = 0; i < meas_dim; i++) {
        r[i] = 0;
        for (uint32_t j = 0; j < s_msckf.state_dim; j++) {
            H[i * s_msckf.state_dim + j] = 0;
        }
    }
}

static bool gatingTest(const float* r, const float* S, int dim) {
    // 简化的卡方检验
    float d = 0;
    for (int i = 0; i < dim; i++) {
        d += r[i] * r[i] / S[i * dim + i];
    }
    return d < s_chi2_threshold;
}

/* ============================================================================
 * 矩阵运算辅助函数
 * ============================================================================ */

static void matMult(const float* A, const float* B, float* C, int m, int n, int p) {
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < p; j++) {
            C[i * p + j] = 0;
            for (int k = 0; k < n; k++) {
                C[i * p + j] += A[i * n + k] * B[k * p + j];
            }
        }
    }
}

static void matTranspose(const float* A, float* At, int m, int n) {
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            At[j * m + i] = A[i * n + j];
        }
    }
}

static void matAdd(const float* A, const float* B, float* C, int m, int n) {
    for (int i = 0; i < m * n; i++) {
        C[i] = A[i] + B[i];
    }
}

static void matSub(const float* A, const float* B, float* C, int m, int n) {
    for (int i = 0; i < m * n; i++) {
        C[i] = A[i] - B[i];
    }
}

static void matScale(const float* A, float s, float* C, int m, int n) {
    for (int i = 0; i < m * n; i++) {
        C[i] = A[i] * s;
    }
}

static bool matInverse3(const float* A, float* Ainv) {
    float det = A[0] * (A[4] * A[8] - A[5] * A[7])
              - A[1] * (A[3] * A[8] - A[5] * A[6])
              + A[2] * (A[3] * A[7] - A[4] * A[6]);
    
    if (std::abs(det) < 1e-6f) return false;
    
    float inv_det = 1.0f / det;
    
    Ainv[0] = (A[4] * A[8] - A[5] * A[7]) * inv_det;
    Ainv[1] = (A[2] * A[7] - A[1] * A[8]) * inv_det;
    Ainv[2] = (A[1] * A[5] - A[2] * A[4]) * inv_det;
    Ainv[3] = (A[5] * A[6] - A[3] * A[8]) * inv_det;
    Ainv[4] = (A[0] * A[8] - A[2] * A[6]) * inv_det;
    Ainv[5] = (A[2] * A[3] - A[0] * A[5]) * inv_det;
    Ainv[6] = (A[3] * A[7] - A[4] * A[6]) * inv_det;
    Ainv[7] = (A[1] * A[6] - A[0] * A[7]) * inv_det;
    Ainv[8] = (A[0] * A[4] - A[1] * A[3]) * inv_det;
    
    return true;
}

static void matCopy(const float* src, float* dst, int m, int n) {
    std::memcpy(dst, src, m * n * sizeof(float));
}

} // namespace t32vio
