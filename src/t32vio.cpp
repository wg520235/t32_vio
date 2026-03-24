/**
 * @file t32vio.cpp
 * @brief T32 VIO 主API实现 (C++11)
 */

#include "t32vio.h"
#include <cstring>
#include <cmath>
#include <thread>
#include <mutex>
#include <atomic>
#include <queue>
#include <condition_variable>
#include <chrono>

namespace t32vio {

/* ============================================================================
 * 配置默认值
 * ============================================================================ */

Config getDefaultConfig() {
    Config config;
    
    // 相机内参 (需要根据实际标定修改)
    config.fx = 240.0f;
    config.fy = 240.0f;
    config.cx = 160.0f;
    config.cy = 120.0f;
    
    // IMU-Camera外参 (需要根据实际安装修改)
    config.p_ic[0] = 0.0f;  config.p_ic[1] = 0.0f;  config.p_ic[2] = 0.0f;
    config.q_ic[0] = 1.0f;  config.q_ic[1] = 0.0f;  config.q_ic[2] = 0.0f;  config.q_ic[3] = 0.0f;
    
    // 特征点参数
    config.max_features = 200;
    config.min_features = 50;
    config.fast_threshold = 20.0f;
    
    // 滑动窗口
    config.window_size = 15;
    
    // IMU噪声 (需要根据实际传感器修改)
    config.noise_gyro = 0.01f;
    config.noise_acc = 0.1f;
    config.noise_gyro_bias = 0.0001f;
    config.noise_acc_bias = 0.001f;
    config.noise_feature = 1.0f;
    
    // 关键帧阈值
    config.kf_translation_threshold = 0.05f;
    config.kf_rotation_threshold = 0.087f;  // 5度
    config.kf_time_threshold = 0.1f;
    
    return config;
}

/* ============================================================================
 * 内部状态
 * ============================================================================ */

static struct {
    std::atomic<bool> initialized{false};
    std::atomic<bool> running{false};
    
    // 线程
    std::thread imu_thread;
    std::thread vision_thread;
    std::thread output_thread;
    
    // 数据队列
    std::queue<ImuData> imu_queue;
    std::queue<ImageFrame> image_queue;
    
    // 同步
    std::mutex imu_mutex;
    std::mutex image_mutex;
    std::mutex state_mutex;
    std::condition_variable imu_cv;
    std::condition_variable image_cv;
    
    // 当前输出
    Pose current_pose;
    std::atomic<State> state{State::UNINITIALIZED};
    
    // 统计
    Stats stats{};
    
    // 配置
    Config config;
} g_vio;

/* ============================================================================
 * 内部函数
 * ============================================================================ */

static void imuThreadFunc();
static void visionThreadFunc();
static void outputThreadFunc();
static void updateStats(const FeatureFrame& frame, float process_time_ms);

/* ============================================================================
 * API实现
 * ============================================================================ */

int init(const Config* config) {
    if (g_vio.initialized) {
        return -1;  // 已初始化
    }
    
    // 使用默认配置或用户配置
    if (config) {
        g_vio.config = *config;
    } else {
        g_vio.config = getDefaultConfig();
    }
    
    // 初始化各模块
    if (frontendInit() != 0) {
        return -1;
    }
    
    if (msckfInit(g_vio.config) != 0) {
        frontendDeinit();
        return -1;
    }
    
    // 启动线程
    g_vio.running = true;
    g_vio.imu_thread = std::thread(imuThreadFunc);
    g_vio.vision_thread = std::thread(visionThreadFunc);
    g_vio.output_thread = std::thread(outputThreadFunc);
    
    g_vio.initialized = true;
    g_vio.state = State::INITIALIZING;
    
    return 0;
}

void deinit() {
    if (!g_vio.initialized) return;
    
    g_vio.running = false;
    
    // 唤醒所有等待的线程
    g_vio.imu_cv.notify_all();
    g_vio.image_cv.notify_all();
    
    // 等待线程结束
    if (g_vio.imu_thread.joinable()) {
        g_vio.imu_thread.join();
    }
    if (g_vio.vision_thread.joinable()) {
        g_vio.vision_thread.join();
    }
    if (g_vio.output_thread.joinable()) {
        g_vio.output_thread.join();
    }
    
    // 清理队列
    {
        std::lock_guard<std::mutex> lock(g_vio.imu_mutex);
        while (!g_vio.imu_queue.empty()) g_vio.imu_queue.pop();
    }
    {
        std::lock_guard<std::mutex> lock(g_vio.image_mutex);
        while (!g_vio.image_queue.empty()) g_vio.image_queue.pop();
    }
    
    // 反初始化模块
    msckfDeinit();
    frontendDeinit();
    
    g_vio.initialized = false;
    g_vio.state = State::UNINITIALIZED;
}

int imuInput(const ImuData* imu) {
    if (!g_vio.initialized || !imu) return -1;
    
    std::lock_guard<std::mutex> lock(g_vio.imu_mutex);
    
    // 限制队列长度，避免内存溢出
    if (g_vio.imu_queue.size() < 512) {
        g_vio.imu_queue.push(*imu);
        g_vio.imu_cv.notify_one();
    }
    
    return 0;
}

int imageInput(const ImageFrame* image) {
    if (!g_vio.initialized || !image) return -1;
    
    std::lock_guard<std::mutex> lock(g_vio.image_mutex);
    
    // 限制队列长度
    if (g_vio.image_queue.size() < 8) {
        g_vio.image_queue.push(*image);
        g_vio.image_cv.notify_one();
    }
    
    return 0;
}

int getPose(Pose* pose) {
    if (!g_vio.initialized || !pose) return -1;
    
    std::lock_guard<std::mutex> lock(g_vio.state_mutex);
    
    if (g_vio.state != State::TRACKING) {
        return -1;
    }
    
    *pose = g_vio.current_pose;
    return 0;
}

State getState() {
    return g_vio.state.load();
}

void getStats(Stats* stats) {
    if (!stats) return;
    
    std::lock_guard<std::mutex> lock(g_vio.state_mutex);
    *stats = g_vio.stats;
}

void reset() {
    if (!g_vio.initialized) return;
    
    std::lock_guard<std::mutex> lock(g_vio.state_mutex);
    
    msckfReset();
    g_vio.state = State::INITIALIZING;
    
    // 清空队列
    {
        std::lock_guard<std::mutex> imu_lock(g_vio.imu_mutex);
        while (!g_vio.imu_queue.empty()) g_vio.imu_queue.pop();
    }
    {
        std::lock_guard<std::mutex> img_lock(g_vio.image_mutex);
        while (!g_vio.image_queue.empty()) g_vio.image_queue.pop();
    }
}

void setOrigin() {
    std::lock_guard<std::mutex> lock(g_vio.state_mutex);
    
    // 重置位姿为原点
    g_vio.current_pose.p[0] = 0;
    g_vio.current_pose.p[1] = 0;
    g_vio.current_pose.p[2] = 0;
    g_vio.current_pose.q[0] = 1;
    g_vio.current_pose.q[1] = 0;
    g_vio.current_pose.q[2] = 0;
    g_vio.current_pose.q[3] = 0;
}

/* ============================================================================
 * 线程函数
 * ============================================================================ */

static void imuThreadFunc() {
    while (g_vio.running) {
        ImuData imu;
        
        {
            std::unique_lock<std::mutex> lock(g_vio.imu_mutex);
            g_vio.imu_cv.wait(lock, [] { return !g_vio.imu_queue.empty() || !g_vio.running; });
            
            if (!g_vio.running) break;
            
            imu = g_vio.imu_queue.front();
            g_vio.imu_queue.pop();
        }
        
        // IMU状态传播
        msckfPropagate(imu);
    }
}

static void visionThreadFunc() {
    FeatureFrame prev_frame;
    bool has_prev_frame = false;
    
    while (g_vio.running) {
        ImageFrame image;
        
        {
            std::unique_lock<std::mutex> lock(g_vio.image_mutex);
            g_vio.image_cv.wait(lock, [] { return !g_vio.image_queue.empty() || !g_vio.running; });
            
            if (!g_vio.running) break;
            
            image = g_vio.image_queue.front();
            g_vio.image_queue.pop();
        }
        
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // 前端处理
        FeatureFrame curr_frame;
        int ret = frontendProcess(image.data, &curr_frame);
        
        if (ret != 0) continue;
        
        curr_frame.timestamp_us = image.timestamp_us;
        
        // 判断关键帧
        if (has_prev_frame) {
            curr_frame.is_keyframe = frontendNeedKeyframe(&curr_frame, &prev_frame);
        } else {
            curr_frame.is_keyframe = true;
        }
        
        // MSCKF更新
        if (curr_frame.is_keyframe) {
            msckfUpdate(curr_frame);
            
            // 更新状态
            {
                std::lock_guard<std::mutex> lock(g_vio.state_mutex);
                
                if (msckfIsInitialized()) {
                    g_vio.state = State::TRACKING;
                    msckfGetPose(g_vio.current_pose);
                    g_vio.current_pose.timestamp_us = image.timestamp_us;
                }
            }
            
            prev_frame = curr_frame;
            has_prev_frame = true;
            
            // 更新统计
            auto end_time = std::chrono::high_resolution_clock::now();
            float process_time = std::chrono::duration<float, std::milli>(end_time - start_time).count();
            updateStats(curr_frame, process_time);
        }
    }
}

static void outputThreadFunc() {
    // 高频输出线程 (200Hz)
    const auto period = std::chrono::microseconds(5000);  // 5ms = 200Hz
    
    while (g_vio.running) {
        auto start = std::chrono::high_resolution_clock::now();
        
        {
            std::lock_guard<std::mutex> lock(g_vio.state_mutex);
            
            if (g_vio.state == State::TRACKING) {
                // 可以在这里添加插值逻辑，输出更高频率的位姿
                // 目前直接输出最新估计
            }
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed = end - start;
        
        if (elapsed < period) {
            std::this_thread::sleep_for(period - elapsed);
        }
    }
}

static void updateStats(const FeatureFrame& frame, float process_time_ms) {
    std::lock_guard<std::mutex> lock(g_vio.state_mutex);
    
    g_vio.stats.total_frames++;
    if (frame.is_keyframe) {
        g_vio.stats.keyframes++;
    }
    g_vio.stats.tracked_features = frontendTrackedCount();
    g_vio.stats.processing_time_ms = process_time_ms;
}

/* ============================================================================
 * 数学工具函数 (C接口兼容)
 * ============================================================================ */

extern "C" {

void quatNormalize(Quat* q) {
    float norm = std::sqrt(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);
    if (norm > 1e-6f) {
        q->w /= norm;
        q->x /= norm;
        q->y /= norm;
        q->z /= norm;
    }
}

void quatMultiply(const Quat* q1, const Quat* q2, Quat* out) {
    out->w = q1->w*q2->w - q1->x*q2->x - q1->y*q2->y - q1->z*q2->z;
    out->x = q1->w*q2->x + q1->x*q2->w + q1->y*q2->z - q1->z*q2->y;
    out->y = q1->w*q2->y - q1->x*q2->z + q1->y*q2->w + q1->z*q2->x;
    out->z = q1->w*q2->z + q1->x*q2->y - q1->y*q2->x + q1->z*q2->w;
}

void quatRotate(const Quat* q, const Vec3* v, Vec3* out) {
    // v' = q * v * q^-1
    Quat qv = {0, v->x, v->y, v->z};
    Quat q_conj = {q->w, -q->x, -q->y, -q->z};
    Quat temp;
    quatMultiply(q, &qv, &temp);
    quatMultiply(&temp, &q_conj, &temp);
    out->x = temp.x;
    out->y = temp.y;
    out->z = temp.z;
}

void vec3Add(const Vec3* a, const Vec3* b, Vec3* out) {
    out->x = a->x + b->x;
    out->y = a->y + b->y;
    out->z = a->z + b->z;
}

void vec3Sub(const Vec3* a, const Vec3* b, Vec3* out) {
    out->x = a->x - b->x;
    out->y = a->y - b->y;
    out->z = a->z - b->z;
}

void vec3Scale(const Vec3* v, float s, Vec3* out) {
    out->x = v->x * s;
    out->y = v->y * s;
    out->z = v->z * s;
}

float vec3Dot(const Vec3* a, const Vec3* b) {
    return a->x*b->x + a->y*b->y + a->z*b->z;
}

float vec3Norm(const Vec3* v) {
    return std::sqrt(v->x*v->x + v->y*v->y + v->z*v->z);
}

void vec3Normalize(Vec3* v) {
    float norm = vec3Norm(v);
    if (norm > 1e-6f) {
        v->x /= norm;
        v->y /= norm;
        v->z /= norm;
    }
}

} // extern "C"

} // namespace t32vio
