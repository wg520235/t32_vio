/**
 * @file t32vio.h
 * @brief T32 VIO C++ API 头文件
 */

#ifndef T32VIO_HPP
#define T32VIO_HPP

#include <cstdint>
#include <cstddef>

namespace t32vio {

/* ============================================================================
 * 常量定义
 * ============================================================================ */

constexpr uint32_t IMAGE_WIDTH = 320;
constexpr uint32_t IMAGE_HEIGHT = 240;
constexpr uint32_t IMAGE_SIZE = IMAGE_WIDTH * IMAGE_HEIGHT;

constexpr uint32_t MAX_FEATURES = 200;
constexpr uint32_t MIN_FEATURES = 50;
constexpr uint32_t FEATURE_GRID_SIZE = 40;

constexpr uint32_t MAX_WINDOW_SIZE = 15;
constexpr uint32_t MIN_WINDOW_SIZE = 5;

constexpr uint32_t PYRAMID_LEVELS = 3;
constexpr uint32_t PYRAMID_L0_W = 320;
constexpr uint32_t PYRAMID_L0_H = 240;
constexpr uint32_t PYRAMID_L1_W = 160;
constexpr uint32_t PYRAMID_L1_H = 120;
constexpr uint32_t PYRAMID_L2_W = 80;
constexpr uint32_t PYRAMID_L2_H = 60;

constexpr uint32_t PATCH_SIZE = 21;
constexpr uint32_t MAX_ITERATIONS = 30;
constexpr float EPSILON = 0.01f;

constexpr uint32_t STATE_DIM_IMU = 15;
constexpr uint32_t STATE_DIM_CAM = 7;
constexpr uint32_t STATE_DIM_MAX = STATE_DIM_IMU + MAX_WINDOW_SIZE * STATE_DIM_CAM;

constexpr uint32_t FEATURE_MAX_OBS = 20;

constexpr uint32_t IMU_BUFFER_SIZE = 512;
constexpr float GRAVITY = 9.81f;

/* ============================================================================
 * 基础数据类型
 * ============================================================================ */

struct Vec3 {
    float x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

struct Quat {
    float w, x, y, z;
    Quat() : w(1), x(0), y(0), z(0) {}
    Quat(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}
};

/* ============================================================================
 * 传感器数据
 * ============================================================================ */

struct ImuData {
    uint64_t timestamp_us;
    float gyro[3];   // rad/s
    float acc[3];    // m/s^2
};

struct ImageFrame {
    uint64_t timestamp_us;
    const uint8_t* data;  // Y通道，320x240
    uint32_t width;
    uint32_t height;
};

/* ============================================================================
 * 特征点
 * ============================================================================ */

struct Feature {
    float u, v;      // 像素坐标
    int32_t id;      // 全局唯一ID
    uint8_t level;   // 金字塔层级
    float score;     // 响应分数
    bool tracked;    // 是否被跟踪
};

struct Pyramid {
    uint8_t* data[PYRAMID_LEVELS];
    uint32_t width[PYRAMID_LEVELS];
    uint32_t height[PYRAMID_LEVELS];
    uint32_t stride[PYRAMID_LEVELS];
};

struct FeatureFrame {
    uint64_t timestamp_us;
    Pyramid pyramid;
    Feature features[MAX_FEATURES];
    uint32_t num_features;
    bool is_keyframe;
};

/* ============================================================================
 * 输出数据
 * ============================================================================ */

struct Pose {
    uint64_t timestamp_us;
    float p[3];      // 位置 (m)
    float q[4];      // 姿态四元数 (w, x, y, z)
    float v[3];      // 速度 (m/s)
};

enum class State {
    UNINITIALIZED = 0,
    INITIALIZING,
    TRACKING,
    LOST
};

struct Stats {
    uint32_t total_frames;
    uint32_t keyframes;
    uint32_t tracked_features;
    float processing_time_ms;
    float position_drift;
};

/* ============================================================================
 * 配置
 * ============================================================================ */

struct Config {
    // 相机内参
    float fx, fy, cx, cy;
    
    // IMU-Camera外参
    float p_ic[3];
    float q_ic[4];  // w, x, y, z
    
    // 特征点参数
    uint32_t max_features;
    uint32_t min_features;
    float fast_threshold;
    
    // 滑动窗口
    uint32_t window_size;
    
    // IMU噪声
    float noise_gyro;
    float noise_acc;
    float noise_gyro_bias;
    float noise_acc_bias;
    float noise_feature;
    
    // 关键帧阈值
    float kf_translation_threshold;
    float kf_rotation_threshold;
    float kf_time_threshold;
};

/* ============================================================================
 * 相机参数
 * ============================================================================ */

struct CameraIntrinsics {
    float fx, fy, cx, cy;
    float k1, k2, p1, p2, k3;
};

struct ImuCamExtrinsics {
    Vec3 p_ic;
    Quat q_ic;
};

/* ============================================================================
 * MSCKF状态
 * ============================================================================ */

struct CamState {
    uint64_t timestamp_us;
    Vec3 p;
    Quat q;
    bool active;
};

struct FeatureObs {
    int32_t feature_id;
    int cam_state_idx;
    float u, v;
};

struct FeatureTrack {
    int32_t id;
    FeatureObs obs[FEATURE_MAX_OBS];
    uint32_t num_obs;
    bool to_be_marginalized;
};

struct ImuState {
    Vec3 p;
    Vec3 v;
    Quat q;
    Vec3 bg;
    Vec3 ba;
};

struct MsckfState {
    ImuState imu_state;
    CamState cam_states[MAX_WINDOW_SIZE];
    uint32_t num_cam_states;
    
    float covariance[STATE_DIM_MAX][STATE_DIM_MAX];
    uint32_t state_dim;
    
    FeatureTrack features[MAX_FEATURES * 2];
    uint32_t num_features;
    
    float noise_gyro;
    float noise_acc;
    float noise_gyro_bias;
    float noise_acc_bias;
    
    bool initialized;
    uint32_t frame_count;
};

/* ============================================================================
 * API函数
 * ============================================================================ */

// 配置
Config getDefaultConfig();

// 初始化和反初始化
int init(const Config* config = nullptr);
void deinit();

// 数据输入
int imuInput(const ImuData* imu);
int imageInput(const ImageFrame* image);

// 输出
int getPose(Pose* pose);
State getState();
void getStats(Stats* stats);

// 控制
void reset();
void setOrigin();

/* ============================================================================
 * 前端API
 * ============================================================================ */

int frontendInit(void);
void frontendDeinit(void);
int frontendProcess(const uint8_t* image, FeatureFrame* frame);
bool frontendNeedKeyframe(const FeatureFrame* curr, const FeatureFrame* prev);
uint32_t frontendTrackedCount(void);
float frontendAverageParallax(void);

void buildPyramid(const uint8_t* src, Pyramid* pyramid);
int fastDetect(const uint8_t* image, uint32_t width, uint32_t height,
               float threshold, Feature* features, int max_features);
void gridNms(Feature* features, int* num_features,
             uint32_t width, uint32_t height, uint32_t grid_size);
void kltTrack(const Pyramid* prev_pyr, const Pyramid* curr_pyr,
              const Feature* prev_features, int num_prev,
              Feature* curr_features, int* num_curr,
              uint8_t* status);
void kltVerify(const Pyramid* prev_pyr, const Pyramid* curr_pyr,
               const Feature* curr_features, int num_curr,
               float* back_errors);

/* ============================================================================
 * MSCKF API
 * ============================================================================ */

int msckfInit(const Config& config);
void msckfDeinit(void);
void msckfPropagate(const ImuData& imu);
int msckfUpdate(const FeatureFrame& frame);
void msckfGetPose(Pose& pose);
bool msckfIsInitialized(void);
void msckfReset(void);

/* ============================================================================
 * 数学工具
 * ============================================================================ */

extern "C" {
void quatNormalize(Quat* q);
void quatMultiply(const Quat* q1, const Quat* q2, Quat* out);
void quatRotate(const Quat* q, const Vec3* v, Vec3* out);

void vec3Add(const Vec3* a, const Vec3* b, Vec3* out);
void vec3Sub(const Vec3* a, const Vec3* b, Vec3* out);
void vec3Scale(const Vec3* v, float s, Vec3* out);
float vec3Dot(const Vec3* a, const Vec3* b);
float vec3Norm(const Vec3* v);
void vec3Normalize(Vec3* v);
}

} // namespace t32vio

#endif // T32VIO_HPP
