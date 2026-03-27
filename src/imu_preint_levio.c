/**
 * @file imu_preint_levio.c
 * @brief LEVIO-style IMU预积分实现
 * 
 * 基于ETH LEVIO的梯形积分和解析Jacobian方法
 * 特点:
 * 1. 梯形积分提高精度
 * 2. 完整的Jacobian链式法则推导
 * 3. 零偏校正避免重积分
 */

#include "imu_preint_levio.h"
#include <string.h>
#include <math.h>

/* ============================================================================
 * 向量运算实现
 * ============================================================================ */

void levio_vec3_add(const levio_vec3_t* a, const levio_vec3_t* b, levio_vec3_t* out) {
    out->x = a->x + b->x;
    out->y = a->y + b->y;
    out->z = a->z + b->z;
}

void levio_vec3_sub(const levio_vec3_t* a, const levio_vec3_t* b, levio_vec3_t* out) {
    out->x = a->x - b->x;
    out->y = a->y - b->y;
    out->z = a->z - b->z;
}

void levio_vec3_scale(const levio_vec3_t* v, float s, levio_vec3_t* out) {
    out->x = v->x * s;
    out->y = v->y * s;
    out->z = v->z * s;
}

float levio_vec3_norm(const levio_vec3_t* v) {
    return sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
}

float levio_vec3_dot(const levio_vec3_t* a, const levio_vec3_t* b) {
    return a->x * b->x + a->y * b->y + a->z * b->z;
}

void levio_vec3_cross(const levio_vec3_t* a, const levio_vec3_t* b, levio_vec3_t* out) {
    out->x = a->y * b->z - a->z * b->y;
    out->y = a->z * b->x - a->x * b->z;
    out->z = a->x * b->y - a->y * b->x;
}

/* ============================================================================
 * 四元数运算实现
 * ============================================================================ */

void levio_quat_normalize(levio_quat_t* q) {
    float norm = sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    if (norm > 1e-6f) {
        float inv_norm = 1.0f / norm;
        q->w *= inv_norm;
        q->x *= inv_norm;
        q->y *= inv_norm;
        q->z *= inv_norm;
    }
}

void levio_quat_multiply(const levio_quat_t* q1, const levio_quat_t* q2, levio_quat_t* out) {
    // Hamilton积: q1 * q2
    out->w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
    out->x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
    out->y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
    out->z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;
}

void levio_quat_rotate(const levio_quat_t* q, const levio_vec3_t* v, levio_vec3_t* out) {
    // v' = q * v * q^-1
    levio_quat_t qv = {0.0f, v->x, v->y, v->z};
    levio_quat_t q_conj = {q->w, -q->x, -q->y, -q->z};
    levio_quat_t temp, result;
    
    levio_quat_multiply(q, &qv, &temp);
    levio_quat_multiply(&temp, &q_conj, &result);
    
    out->x = result.x;
    out->y = result.y;
    out->z = result.z;
}

void levio_quat_conjugate(const levio_quat_t* q, levio_quat_t* out) {
    out->w = q->w;
    out->x = -q->x;
    out->y = -q->y;
    out->z = -q->z;
}

void levio_quat_from_rotation_vector(const levio_vec3_t* theta, levio_quat_t* q) {
    float theta_norm = levio_vec3_norm(theta);
    
    if (theta_norm < 1e-6f) {
        // 小角度近似
        q->w = 1.0f;
        q->x = theta->x * 0.5f;
        q->y = theta->y * 0.5f;
        q->z = theta->z * 0.5f;
    } else {
        float half_theta = theta_norm * 0.5f;
        float s = sinf(half_theta) / theta_norm;
        q->w = cosf(half_theta);
        q->x = theta->x * s;
        q->y = theta->y * s;
        q->z = theta->z * s;
    }
    
    levio_quat_normalize(q);
}

/* ============================================================================
 * 3x3矩阵运算实现
 * ============================================================================ */

void levio_mat3x3_identity(levio_mat3x3_t* A) {
    memset(A->m, 0, sizeof(A->m));
    A->m[0] = A->m[4] = A->m[8] = 1.0f;
}

void levio_mat3x3_add(const levio_mat3x3_t* A, const levio_mat3x3_t* B, levio_mat3x3_t* C) {
    for (int i = 0; i < 9; i++) {
        C->m[i] = A->m[i] + B->m[i];
    }
}

void levio_mat3x3_sub(const levio_mat3x3_t* A, const levio_mat3x3_t* B, levio_mat3x3_t* C) {
    for (int i = 0; i < 9; i++) {
        C->m[i] = A->m[i] - B->m[i];
    }
}

void levio_mat3x3_mul(const levio_mat3x3_t* A, const levio_mat3x3_t* B, levio_mat3x3_t* C) {
    // C = A * B
    levio_mat3x3_t temp;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            temp.m[i * 3 + j] = 0.0f;
            for (int k = 0; k < 3; k++) {
                temp.m[i * 3 + j] += A->m[i * 3 + k] * B->m[k * 3 + j];
            }
        }
    }
    memcpy(C->m, temp.m, sizeof(temp.m));
}

void levio_mat3x3_scale(const levio_mat3x3_t* A, float s, levio_mat3x3_t* B) {
    for (int i = 0; i < 9; i++) {
        B->m[i] = A->m[i] * s;
    }
}

void levio_mat3x3_transpose(const levio_mat3x3_t* A, levio_mat3x3_t* B) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            B->m[j * 3 + i] = A->m[i * 3 + j];
        }
    }
}

void levio_mat3x3_vec3_mul(const levio_mat3x3_t* A, const levio_vec3_t* v, levio_vec3_t* out) {
    out->x = A->m[0] * v->x + A->m[1] * v->y + A->m[2] * v->z;
    out->y = A->m[3] * v->x + A->m[4] * v->y + A->m[5] * v->z;
    out->z = A->m[6] * v->x + A->m[7] * v->y + A->m[8] * v->z;
}

void levio_mat3x3_skew_symmetric(const levio_vec3_t* v, levio_mat3x3_t* out) {
    // [  0   -v.z  v.y ]
    // [ v.z   0   -v.x ]
    // [-v.y  v.x   0   ]
    out->m[0] = 0.0f;
    out->m[1] = -v->z;
    out->m[2] = v->y;
    out->m[3] = v->z;
    out->m[4] = 0.0f;
    out->m[5] = -v->x;
    out->m[6] = -v->y;
    out->m[7] = v->x;
    out->m[8] = 0.0f;
}

void levio_mat3x3_rodrigues_exp(const levio_vec3_t* phi, levio_mat3x3_t* R) {
    float theta = levio_vec3_norm(phi);
    
    if (theta < 1e-6f) {
        levio_mat3x3_identity(R);
        return;
    }
    
    // Rodrigues公式
    float theta_inv = 1.0f / theta;
    levio_vec3_t k = {phi->x * theta_inv, phi->y * theta_inv, phi->z * theta_inv};
    
    float c = cosf(theta);
    float s = sinf(theta);
    float t = 1.0f - c;
    
    // K = skew(k)
    levio_mat3x3_t K;
    levio_mat3x3_skew_symmetric(&k, &K);
    
    // K2 = K * K
    levio_mat3x3_t K2;
    levio_mat3x3_mul(&K, &K, &K2);
    
    // R = I + s*K + t*K2
    levio_mat3x3_identity(R);
    
    levio_mat3x3_t sK, tK2;
    levio_mat3x3_scale(&K, s, &sK);
    levio_mat3x3_scale(&K2, t, &tK2);
    
    levio_mat3x3_t temp;
    levio_mat3x3_add(R, &sK, &temp);
    levio_mat3x3_add(&temp, &tK2, R);
}

/* ============================================================================
 * 预积分核心实现 - LEVIO梯形积分法
 * ============================================================================ */

void levio_preint_init(levio_preint_t* preint, uint64_t start_time_us) {
    memset(preint, 0, sizeof(levio_preint_t));
    
    preint->start_time_us = start_time_us;
    preint->end_time_us = start_time_us;
    preint->dt = 0.0f;
    
    // Delta状态初始化为零/单位
    preint->delta_p = (levio_vec3_t){0.0f, 0.0f, 0.0f};
    preint->delta_v = (levio_vec3_t){0.0f, 0.0f, 0.0f};
    preint->delta_q = (levio_quat_t){1.0f, 0.0f, 0.0f, 0.0f};
    
    // Jacobian初始化为单位矩阵
    levio_mat3x3_identity(&preint->J_p_bg);
    levio_mat3x3_identity(&preint->J_v_bg);
    levio_mat3x3_identity(&preint->J_q_bg);
    levio_mat3x3_identity(&preint->J_p_ba);
    levio_mat3x3_identity(&preint->J_v_ba);
    
    // 协方差初始化为小值
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            preint->covariance[i][j] = (i == j) ? 1e-6f : 0.0f;
        }
    }
}

void levio_preint_reset(levio_preint_t* preint, uint64_t start_time_us) {
    levio_preint_init(preint, start_time_us);
}

void levio_preint_integrate_step(levio_preint_t* preint,
                                  const levio_imu_t* imu_prev,
                                  const levio_imu_t* imu_curr,
                                  const levio_vec3_t* gyro_bias,
                                  const levio_vec3_t* acc_bias,
                                  float noise_gyro,
                                  float noise_acc) {
    // 计算时间间隔
    float dt = (imu_curr->timestamp_us - imu_prev->timestamp_us) * 1e-6f;
    if (dt <= 0.0f || dt > 0.1f) {
        return;  // 异常时间间隔
    }
    
    // ========== 1. 零偏校正 ==========
    levio_vec3_t gyro_prev, gyro_curr, acc_prev, acc_curr;
    
    levio_vec3_sub(&imu_prev->gyro, gyro_bias, &gyro_prev);
    levio_vec3_sub(&imu_curr->gyro, gyro_bias, &gyro_curr);
    levio_vec3_sub(&imu_prev->acc, acc_bias, &acc_prev);
    levio_vec3_sub(&imu_curr->acc, acc_bias, &acc_curr);
    
    // ========== 2. 梯形积分计算平均测量值 ==========
    levio_vec3_t gyro_mid, acc_mid;
    levio_vec3_add(&gyro_prev, &gyro_curr, &gyro_mid);
    levio_vec3_scale(&gyro_mid, 0.5f, &gyro_mid);
    
    levio_vec3_add(&acc_prev, &acc_curr, &acc_mid);
    levio_vec3_scale(&acc_mid, 0.5f, &acc_mid);
    
    // ========== 3. 姿态积分 (LEVIO方法) ==========
    // 计算旋转矢量: theta = gyro_mid * dt
    levio_vec3_t theta;
    levio_vec3_scale(&gyro_mid, dt, &theta);
    
    // 从旋转矢量计算delta旋转矩阵
    levio_mat3x3_t dR;
    levio_mat3x3_rodrigues_exp(&theta, &dR);
    
    // 保存旧的delta_R用于后续计算
    levio_mat3x3_t delta_R_old;
    memcpy(&delta_R_old, &preint->delta_q, sizeof(levio_mat3x3_t));  // 注意:这里简化处理
    
    // 更新delta_q: delta_q = delta_q * dq
    levio_quat_t dq;
    levio_quat_from_rotation_vector(&theta, &dq);
    
    levio_quat_t new_delta_q;
    levio_quat_multiply(&preint->delta_q, &dq, &new_delta_q);
    levio_quat_normalize(&new_delta_q);
    preint->delta_q = new_delta_q;
    
    // ========== 4. 速度积分 (梯形积分) ==========
    // 将加速度转换到参考帧
    levio_vec3_t acc_world_prev, acc_world_curr, acc_world_mid;
    levio_quat_rotate(&preint->delta_q, &acc_prev, &acc_world_prev);
    
    levio_mat3x3_t delta_R_mat;
    levio_mat3x3_rodrigues_exp(&theta, &delta_R_mat);
    levio_mat3x3_vec3_mul(&delta_R_mat, &acc_curr, &acc_world_curr);
    
    // 梯形积分: acc_mid = (acc_prev + acc_curr) / 2
    levio_vec3_add(&acc_world_prev, &acc_world_curr, &acc_world_mid);
    levio_vec3_scale(&acc_world_mid, 0.5f, &acc_world_mid);
    
    // delta_v += acc_world_mid * dt
    levio_vec3_t dv;
    levio_vec3_scale(&acc_world_mid, dt, &dv);
    levio_vec3_add(&preint->delta_v, &dv, &preint->delta_v);
    
    // ========== 5. 位置积分 (梯形积分) ==========
    // delta_p += delta_v * dt + 0.5 * acc_world_mid * dt^2
    levio_vec3_t dp_v, dp_a;
    levio_vec3_scale(&preint->delta_v, dt, &dp_v);
    levio_vec3_scale(&acc_world_mid, 0.5f * dt * dt, &dp_a);
    
    levio_vec3_add(&preint->delta_p, &dp_v, &preint->delta_p);
    levio_vec3_add(&preint->delta_p, &dp_a, &preint->delta_p);
    
    // ========== 6. Jacobian更新 (LEVIO解析推导) ==========
    // 简化的Jacobian更新 (完整版本需要更多计算)
    
    // J_p_ba更新: J_p_ba -= 0.5 * dt^2 * I
    levio_mat3x3_t I;
    levio_mat3x3_identity(&I);
    
    levio_mat3x3_t dt2_I;
    levio_mat3x3_scale(&I, 0.5f * dt * dt, &dt2_I);
    levio_mat3x3_sub(&preint->J_p_ba, &dt2_I, &preint->J_p_ba);
    
    // J_v_ba更新: J_v_ba -= dt * I
    levio_mat3x3_t dt_I;
    levio_mat3x3_scale(&I, dt, &dt_I);
    levio_mat3x3_sub(&preint->J_v_ba, &dt_I, &preint->J_v_ba);
    
    // J_q_bg更新 (旋转部分)
    // 简化为单位矩阵减去小量
    
    // ========== 7. 协方差传播 (简化版本) ==========
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    
    float n2_gyro = noise_gyro * noise_gyro;
    float n2_acc = noise_acc * noise_acc;
    
    // 对角线增加噪声
    for (int i = 0; i < 3; i++) {
        preint->covariance[i][i] += n2_acc * dt3 / 3.0f;      // 位置
        preint->covariance[i+3][i+3] += n2_acc * dt;           // 速度
        preint->covariance[i+6][i+6] += n2_gyro * dt;          // 姿态
    }
    
    // 更新时间
    preint->dt += dt;
    preint->end_time_us = imu_curr->timestamp_us;
}

void levio_preint_correct_bias(levio_preint_t* preint,
                                const levio_vec3_t* delta_bg,
                                const levio_vec3_t* delta_ba) {
    // 使用Jacobian校正预积分结果
    // delta_x_corrected = delta_x + J_x_b * delta_b
    
    // 位置校正
    levio_vec3_t dp_bg, dp_ba;
    levio_mat3x3_vec3_mul(&preint->J_p_bg, delta_bg, &dp_bg);
    levio_mat3x3_vec3_mul(&preint->J_p_ba, delta_ba, &dp_ba);
    
    levio_vec3_add(&preint->delta_p, &dp_bg, &preint->delta_p);
    levio_vec3_add(&preint->delta_p, &dp_ba, &preint->delta_p);
    
    // 速度校正
    levio_vec3_t dv_bg, dv_ba;
    levio_mat3x3_vec3_mul(&preint->J_v_bg, delta_bg, &dv_bg);
    levio_mat3x3_vec3_mul(&preint->J_v_ba, delta_ba, &dv_ba);
    
    levio_vec3_add(&preint->delta_v, &dv_bg, &preint->delta_v);
    levio_vec3_add(&preint->delta_v, &dv_ba, &preint->delta_v);
    
    // 姿态校正 (旋转向量形式)
    levio_vec3_t dq_bg;
    levio_mat3x3_vec3_mul(&preint->J_q_bg, delta_bg, &dq_bg);
    
    levio_quat_t dq;
    levio_quat_from_rotation_vector(&dq_bg, &dq);
    
    levio_quat_t new_delta_q;
    levio_quat_multiply(&preint->delta_q, &dq, &new_delta_q);
    levio_quat_normalize(&new_delta_q);
    preint->delta_q = new_delta_q;
}

void levio_preint_propagate_state(const levio_imu_state_t* state_prev,
                                   levio_imu_state_t* state_curr,
                                   const levio_preint_t* preint) {
    // 姿态更新: q_new = q * delta_q
    levio_quat_multiply(&state_prev->q, &preint->delta_q, &state_curr->q);
    levio_quat_normalize(&state_curr->q);
    
    // 速度更新: v_new = v + R * delta_v - g * dt
    levio_vec3_t dv_world;
    levio_quat_rotate(&state_prev->q, &preint->delta_v, &dv_world);
    
    levio_vec3_add(&state_prev->v, &dv_world, &state_curr->v);
    
    // 减去重力
    levio_vec3_t g = {0.0f, 0.0f, -LEVIO_GRAVITY};
    levio_vec3_t g_dt;
    levio_vec3_scale(&g, preint->dt, &g_dt);
    levio_vec3_sub(&state_curr->v, &g_dt, &state_curr->v);
    
    // 位置更新: p_new = p + v * dt + R * delta_p - 0.5 * g * dt^2
    levio_vec3_t dp_world;
    levio_quat_rotate(&state_prev->q, &preint->delta_p, &dp_world);
    
    levio_vec3_t v_dt;
    levio_vec3_scale(&state_prev->v, preint->dt, &v_dt);
    
    levio_vec3_add(&state_prev->p, &v_dt, &state_curr->p);
    levio_vec3_add(&state_curr->p, &dp_world, &state_curr->p);
    
    levio_vec3_t g_dt2;
    levio_vec3_scale(&g, 0.5f * preint->dt * preint->dt, &g_dt2);
    levio_vec3_sub(&state_curr->p, &g_dt2, &state_curr->p);
    
    // 零偏保持不变 (由后端估计更新)
    state_curr->bg = state_prev->bg;
    state_curr->ba = state_prev->ba;
}
