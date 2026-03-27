/**
 * @file imu_preint_levio.h
 * @brief LEVIO-style IMU预积分实现 - 集成到T32VIO
 * 
 * 基于LEVIO的梯形积分和解析Jacobian实现
 */

#ifndef IMU_PREINT_LEVIO_H
#define IMU_PREINT_LEVIO_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * 常量定义
 * ============================================================================ */

#define LEVIO_PI 3.14159265358979323846f
#define LEVIO_GRAVITY 9.81f

/* ============================================================================
 * 基础数据类型
 * ============================================================================ */

typedef struct {
    float x, y, z;
} levio_vec3_t;

typedef struct {
    float w, x, y, z;
} levio_quat_t;

typedef struct {
    float m[9];  // 行优先: m[row*3 + col]
} levio_mat3x3_t;

typedef struct {
    uint64_t timestamp_us;
    levio_vec3_t gyro;  // rad/s
    levio_vec3_t acc;   // m/s^2
} levio_imu_t;

typedef struct {
    levio_vec3_t p;     // 位置
    levio_vec3_t v;     // 速度
    levio_quat_t q;     // 姿态四元数
    levio_vec3_t bg;    // 陀螺仪零偏
    levio_vec3_t ba;    // 加速度计零偏
} levio_imu_state_t;

/**
 * @brief LEVIO-style预积分结果
 * 
 * 包含delta状态和对零偏的Jacobian
 */
typedef struct {
    // 预积分时间
    float dt;
    uint64_t start_time_us;
    uint64_t end_time_us;
    
    // Delta状态 (从参考帧到当前帧的变换)
    levio_vec3_t delta_p;       // 位置变化
    levio_vec3_t delta_v;       // 速度变化
    levio_quat_t delta_q;       // 姿态变化
    
    // 对陀螺仪零偏的Jacobian
    levio_mat3x3_t J_p_bg;      // delta_p对bg的Jacobian
    levio_mat3x3_t J_v_bg;      // delta_v对bg的Jacobian
    levio_mat3x3_t J_q_bg;      // delta_q对bg的Jacobian (实际上是旋转矢量的Jacobian)
    
    // 对加速度计零偏的Jacobian
    levio_mat3x3_t J_p_ba;      // delta_p对ba的Jacobian
    levio_mat3x3_t J_v_ba;      // delta_v对ba的Jacobian
    
    // 协方差 (9x9: [dp, dv, dtheta])
    float covariance[9][9];
    
} levio_preint_t;

/* ============================================================================
 * API函数声明
 * ============================================================================ */

/**
 * @brief 初始化预积分状态
 */
void levio_preint_init(levio_preint_t* preint, uint64_t start_time_us);

/**
 * @brief 重置预积分状态
 */
void levio_preint_reset(levio_preint_t* preint, uint64_t start_time_us);

/**
 * @brief 单步积分 - LEVIO梯形积分法
 * 
 * @param preint 预积分状态
 * @param imu_prev 上一帧IMU数据
 * @param imu_curr 当前帧IMU数据
 * @param gyro_bias 陀螺仪零偏估计
 * @param acc_bias 加速度计零偏估计
 * @param noise_gyro 陀螺仪噪声 (rad/s/sqrt(Hz))
 * @param noise_acc 加速度计噪声 (m/s^2/sqrt(Hz))
 */
void levio_preint_integrate_step(levio_preint_t* preint,
                                  const levio_imu_t* imu_prev,
                                  const levio_imu_t* imu_curr,
                                  const levio_vec3_t* gyro_bias,
                                  const levio_vec3_t* acc_bias,
                                  float noise_gyro,
                                  float noise_acc);

/**
 * @brief 使用Jacobian校正零偏变化
 * 
 * 当零偏估计更新时，用此函数校正预积分结果，避免重新积分
 */
void levio_preint_correct_bias(levio_preint_t* preint,
                                const levio_vec3_t* delta_bg,
                                const levio_vec3_t* delta_ba);

/**
 * @brief 从预积分结果传播IMU状态
 */
void levio_preint_propagate_state(const levio_imu_state_t* state_prev,
                                   levio_imu_state_t* state_curr,
                                   const levio_preint_t* preint);

/* ============================================================================
 * 数学工具函数
 * ============================================================================ */

// 向量运算
void levio_vec3_add(const levio_vec3_t* a, const levio_vec3_t* b, levio_vec3_t* out);
void levio_vec3_sub(const levio_vec3_t* a, const levio_vec3_t* b, levio_vec3_t* out);
void levio_vec3_scale(const levio_vec3_t* v, float s, levio_vec3_t* out);
float levio_vec3_norm(const levio_vec3_t* v);
float levio_vec3_dot(const levio_vec3_t* a, const levio_vec3_t* b);
void levio_vec3_cross(const levio_vec3_t* a, const levio_vec3_t* b, levio_vec3_t* out);

// 四元数运算
void levio_quat_normalize(levio_quat_t* q);
void levio_quat_multiply(const levio_quat_t* q1, const levio_quat_t* q2, levio_quat_t* out);
void levio_quat_rotate(const levio_quat_t* q, const levio_vec3_t* v, levio_vec3_t* out);
void levio_quat_conjugate(const levio_quat_t* q, levio_quat_t* out);

// 3x3矩阵运算
void levio_mat3x3_identity(levio_mat3x3_t* A);
void levio_mat3x3_add(const levio_mat3x3_t* A, const levio_mat3x3_t* B, levio_mat3x3_t* C);
void levio_mat3x3_sub(const levio_mat3x3_t* A, const levio_mat3x3_t* B, levio_mat3x3_t* C);
void levio_mat3x3_mul(const levio_mat3x3_t* A, const levio_mat3x3_t* B, levio_mat3x3_t* C);
void levio_mat3x3_scale(const levio_mat3x3_t* A, float s, levio_mat3x3_t* B);
void levio_mat3x3_transpose(const levio_mat3x3_t* A, levio_mat3x3_t* B);
void levio_mat3x3_vec3_mul(const levio_mat3x3_t* A, const levio_vec3_t* v, levio_vec3_t* out);
void levio_mat3x3_skew_symmetric(const levio_vec3_t* v, levio_mat3x3_t* out);

// Rodrigues指数映射: 旋转向量 -> 旋转矩阵
void levio_mat3x3_rodrigues_exp(const levio_vec3_t* phi, levio_mat3x3_t* R);

// 从旋转向量计算delta四元数
void levio_quat_from_rotation_vector(const levio_vec3_t* theta, levio_quat_t* q);

#ifdef __cplusplus
}
#endif

#endif /* IMU_PREINT_LEVIO_H */
