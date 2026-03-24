/**
 * @file imu_preint.c
 * @brief IMU预积分实现
 */

#include "imu_preint.h"
#include <string.h>
#include <math.h>

/* ============================================================================
 * 静态变量
 * ============================================================================ */

// IMU环形缓冲区
static t32vio_imu_t s_imu_buffer[T32VIO_IMU_BUFFER_SIZE];
static volatile uint32_t s_imu_head = 0;
static volatile uint32_t s_imu_tail = 0;
static uint32_t s_imu_count = 0;

// 当前IMU状态
static t32vio_imu_state_t s_imu_state;
static bool s_state_valid = false;

// 预积分状态
static t32vio_preint_t s_preint;
static bool s_preint_active = false;

// 噪声参数
static float s_noise_gyro = T32VIO_GYRO_NOISE;
static float s_noise_acc = T32VIO_ACC_NOISE;
static float s_noise_gyro_bias = T32VIO_GYRO_BIAS_NOISE;
static float s_noise_acc_bias = T32VIO_ACC_BIAS_NOISE;

/* ============================================================================
 * 静态函数声明
 * ============================================================================ */

static void integrate_imu(const t32vio_imu_t *imu_prev, const t32vio_imu_t *imu_curr);
static void update_covariance(float dt, const t32vio_vec3_t *acc, const t32vio_vec3_t *gyro);
static void quat_integrate(t32vio_quat_t *q, const t32vio_vec3_t *omega, float dt);

/* ============================================================================
 * API实现
 * ============================================================================ */

int t32vio_imu_init(void) {
    memset(s_imu_buffer, 0, sizeof(s_imu_buffer));
    s_imu_head = 0;
    s_imu_tail = 0;
    s_imu_count = 0;
    
    memset(&s_imu_state, 0, sizeof(s_imu_state));
    s_imu_state.q.w = 1.0f;  // 单位四元数
    s_state_valid = false;
    
    memset(&s_preint, 0, sizeof(s_preint));
    s_preint.delta_q.w = 1.0f;
    s_preint_active = false;
    
    return 0;
}

void t32vio_imu_deinit(void) {
    s_state_valid = false;
    s_preint_active = false;
}

int t32vio_imu_add(const t32vio_imu_t *imu) {
    if (imu == NULL) return -1;
    
    // 环形缓冲区写入
    uint32_t next_head = (s_imu_head + 1) % T32VIO_IMU_BUFFER_SIZE;
    if (next_head == s_imu_tail) {
        // 缓冲区满，覆盖最旧数据
        s_imu_tail = (s_imu_tail + 1) % T32VIO_IMU_BUFFER_SIZE;
    }
    
    s_imu_buffer[s_imu_head] = *imu;
    s_imu_head = next_head;
    s_imu_count++;
    
    return 0;
}

void t32vio_preint_reset(uint64_t start_time_us) {
    memset(&s_preint, 0, sizeof(s_preint));
    s_preint.delta_q.w = 1.0f;  // 单位四元数
    s_preint.start_time_us = start_time_us;
    s_preint.end_time_us = start_time_us;
    s_preint.dt = 0.0f;
    
    // 初始化协方差为单位矩阵的缩放
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            s_preint.covariance[i][j] = (i == j) ? 1e-6f : 0.0f;
        }
    }
    
    // 初始化Jacobian为单位矩阵
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            s_preint.jacobian_p_bg[i][j] = (i == j) ? 1.0f : 0.0f;
            s_preint.jacobian_p_ba[i][j] = (i == j) ? 1.0f : 0.0f;
            s_preint.jacobian_v_bg[i][j] = (i == j) ? 1.0f : 0.0f;
            s_preint.jacobian_v_ba[i][j] = (i == j) ? 1.0f : 0.0f;
            s_preint.jacobian_q_bg[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    
    s_preint_active = true;
}

int t32vio_preint_integrate(uint64_t end_time_us, t32vio_preint_t *result) {
    if (!s_preint_active || result == NULL) return -1;
    
    // 找到起始位置
    uint32_t idx = s_imu_tail;
    bool found_start = false;
    
    while (idx != s_imu_head) {
        if (s_imu_buffer[idx].timestamp_us >= s_preint.start_time_us) {
            found_start = true;
            break;
        }
        idx = (idx + 1) % T32VIO_IMU_BUFFER_SIZE;
    }
    
    if (!found_start) return -1;
    
    // 积分到结束时间
    uint32_t prev_idx = idx;
    idx = (idx + 1) % T32VIO_IMU_BUFFER_SIZE;
    
    while (idx != s_imu_head && s_imu_buffer[idx].timestamp_us <= end_time_us) {
        integrate_imu(&s_imu_buffer[prev_idx], &s_imu_buffer[idx]);
        prev_idx = idx;
        idx = (idx + 1) % T32VIO_IMU_BUFFER_SIZE;
    }
    
    // 输出结果
    *result = s_preint;
    result->end_time_us = end_time_us;
    
    return 0;
}

void t32vio_imu_propagate(t32vio_imu_state_t *state, const t32vio_preint_t *preint,
                          const t32vio_config_t *noise) {
    if (state == NULL || preint == NULL) return;
    
    // 去除零偏的IMU测量
    t32vio_vec3_t gyro_unbias, acc_unbias;
    t32vio_vec3_sub(&preint->delta_q.x, &state->bg, &gyro_unbias);  // 简化处理
    
    // 姿态更新: q_new = q * delta_q
    t32vio_quat_t q_new;
    t32vio_quat_multiply(&state->q, &preint->delta_q, &q_new);
    t32vio_quat_normalize(&q_new);
    state->q = q_new;
    
    // 速度更新: v_new = v + R * delta_v - g * dt
    t32vio_vec3_t dv_world;
    t32vio_quat_rotate(&state->q, &preint->delta_v, &dv_world);
    
    t32vio_vec3_add(&state->v, &dv_world, &state->v);
    
    t32vio_vec3_t g = {0, 0, -T32VIO_GRAVITY};
    t32vio_vec3_t g_dt;
    t32vio_vec3_scale(&g, preint->dt, &g_dt);
    t32vio_vec3_sub(&state->v, &g_dt, &state->v);
    
    // 位置更新: p_new = p + v * dt + 0.5 * (R * delta_p - g * dt^2)
    t32vio_vec3_t dp_world;
    t32vio_quat_rotate(&state->q, &preint->delta_p, &dp_world);
    
    t32vio_vec3_t v_dt;
    t32vio_vec3_scale(&state->v, preint->dt, &v_dt);
    
    t32vio_vec3_add(&state->p, &v_dt, &state->p);
    t32vio_vec3_add(&state->p, &dp_world, &state->p);
    
    // 零偏随机游走 (简化)
    // 实际应该在EKF中更新
}

void t32vio_preint_correct_bias(t32vio_preint_t *preint,
                                const t32vio_vec3_t *dbg,
                                const t32vio_vec3_t *dba) {
    if (preint == NULL || dbg == NULL || dba == NULL) return;
    
    // 使用Jacobian校正预积分结果
    // delta_p_corrected = delta_p + J_p_bg * dbg + J_p_ba * dba
    // delta_v_corrected = delta_v + J_v_bg * dbg + J_v_ba * dba
    // delta_q_corrected = delta_q * exp(J_q_bg * dbg)
    
    t32vio_vec3_t correction_p, correction_v;
    
    // 位置校正
    for (int i = 0; i < 3; i++) {
        correction_p.x = preint->jacobian_p_bg[i][0] * dbg->x + 
                        preint->jacobian_p_bg[i][1] * dbg->y +
                        preint->jacobian_p_bg[i][2] * dbg->z +
                        preint->jacobian_p_ba[i][0] * dba->x +
                        preint->jacobian_p_ba[i][1] * dba->y +
                        preint->jacobian_p_ba[i][2] * dba->z;
        correction_p.y = preint->jacobian_p_bg[i][1] * dbg->x + 
                        preint->jacobian_p_bg[i][1] * dbg->y +
                        preint->jacobian_p_bg[i][1] * dbg->z +
                        preint->jacobian_p_ba[i][1] * dba->x +
                        preint->jacobian_p_ba[i][1] * dba->y +
                        preint->jacobian_p_ba[i][1] * dba->z;
        correction_p.z = preint->jacobian_p_bg[i][2] * dbg->x + 
                        preint->jacobian_p_bg[i][2] * dbg->y +
                        preint->jacobian_p_bg[i][2] * dbg->z +
                        preint->jacobian_p_ba[i][2] * dba->x +
                        preint->jacobian_p_ba[i][2] * dba->y +
                        preint->jacobian_p_ba[i][2] * dba->z;
    }
    
    t32vio_vec3_add(&preint->delta_p, &correction_p, &preint->delta_p);
    
    // 速度校正
    for (int i = 0; i < 3; i++) {
        correction_v.x = preint->jacobian_v_bg[i][0] * dbg->x + 
                        preint->jacobian_v_bg[i][1] * dbg->y +
                        preint->jacobian_v_bg[i][2] * dbg->z +
                        preint->jacobian_v_ba[i][0] * dba->x +
                        preint->jacobian_v_ba[i][1] * dba->y +
                        preint->jacobian_v_ba[i][2] * dba->z;
        correction_v.y = preint->jacobian_v_bg[i][1] * dbg->x + 
                        preint->jacobian_v_bg[i][1] * dbg->y +
                        preint->jacobian_v_bg[i][1] * dbg->z +
                        preint->jacobian_v_ba[i][1] * dba->x +
                        preint->jacobian_v_ba[i][1] * dba->y +
                        preint->jacobian_v_ba[i][1] * dba->z;
        correction_v.z = preint->jacobian_v_bg[i][2] * dbg->x + 
                        preint->jacobian_v_bg[i][2] * dbg->y +
                        preint->jacobian_v_bg[i][2] * dbg->z +
                        preint->jacobian_v_ba[i][2] * dba->x +
                        preint->jacobian_v_ba[i][2] * dba->y +
                        preint->jacobian_v_ba[i][2] * dba->z;
    }
    
    t32vio_vec3_add(&preint->delta_v, &correction_v, &preint->delta_v);
    
    // 姿态校正 (简化)
    t32vio_vec3_t theta;
    for (int i = 0; i < 3; i++) {
        theta.x = preint->jacobian_q_bg[i][0] * dbg->x + 
                 preint->jacobian_q_bg[i][1] * dbg->y +
                 preint->jacobian_q_bg[i][2] * dbg->z;
        theta.y = preint->jacobian_q_bg[i][1] * dbg->x + 
                 preint->jacobian_q_bg[i][1] * dbg->y +
                 preint->jacobian_q_bg[i][1] * dbg->z;
        theta.z = preint->jacobian_q_bg[i][2] * dbg->x + 
                 preint->jacobian_q_bg[i][2] * dbg->y +
                 preint->jacobian_q_bg[i][2] * dbg->z;
    }
    
    // theta 转换为四元数并乘以原四元数
    float theta_norm = t32vio_vec3_norm(&theta);
    if (theta_norm > 1e-6f) {
        t32vio_quat_t dq;
        dq.w = cosf(theta_norm * 0.5f);
        float s = sinf(theta_norm * 0.5f) / theta_norm;
        dq.x = theta.x * s;
        dq.y = theta.y * s;
        dq.z = theta.z * s;
        
        t32vio_quat_multiply(&preint->delta_q, &dq, &preint->delta_q);
        t32vio_quat_normalize(&preint->delta_q);
    }
}

void t32vio_imu_get_state(t32vio_imu_state_t *state) {
    if (state != NULL && s_state_valid) {
        *state = s_imu_state;
    }
}

void t32vio_imu_set_state(const t32vio_imu_state_t *state) {
    if (state != NULL) {
        s_imu_state = *state;
        s_state_valid = true;
    }
}

/* ============================================================================
 * 内部函数实现
 * ============================================================================ */

static void integrate_imu(const t32vio_imu_t *imu_prev, const t32vio_imu_t *imu_curr) {
    // 计算时间间隔
    float dt = (imu_curr->timestamp_us - imu_prev->timestamp_us) * 1e-6f;
    if (dt <= 0 || dt > 0.1f) return;  // 异常时间间隔
    
    // 中值积分
    t32vio_vec3_t gyro_mid, acc_mid;
    gyro_mid.x = (imu_prev->gyro.x + imu_curr->gyro.x) * 0.5f;
    gyro_mid.y = (imu_prev->gyro.y + imu_curr->gyro.y) * 0.5f;
    gyro_mid.z = (imu_prev->gyro.z + imu_curr->gyro.z) * 0.5f;
    
    acc_mid.x = (imu_prev->acc.x + imu_curr->acc.x) * 0.5f;
    acc_mid.y = (imu_prev->acc.y + imu_curr->acc.y) * 0.5f;
    acc_mid.z = (imu_prev->acc.z + imu_curr->acc.z) * 0.5f;
    
    // 姿态积分
    t32vio_quat_t delta_q;
    quat_integrate(&delta_q, &gyro_mid, dt);
    
    // 更新delta_q
    t32vio_quat_multiply(&s_preint.delta_q, &delta_q, &s_preint.delta_q);
    t32vio_quat_normalize(&s_preint.delta_q);
    
    // 速度积分
    t32vio_vec3_t acc_world;
    t32vio_quat_rotate(&s_preint.delta_q, &acc_mid, &acc_world);
    
    t32vio_vec3_t dv;
    dv.x = acc_world.x * dt;
    dv.y = acc_world.y * dt;
    dv.z = acc_world.z * dt;
    
    t32vio_vec3_add(&s_preint.delta_v, &dv, &s_preint.delta_v);
    
    // 位置积分
    t32vio_vec3_t dp;
    dp.x = s_preint.delta_v.x * dt + 0.5f * dv.x * dt;
    dp.y = s_preint.delta_v.y * dt + 0.5f * dv.y * dt;
    dp.z = s_preint.delta_v.z * dt + 0.5f * dv.z * dt;
    
    t32vio_vec3_add(&s_preint.delta_p, &dp, &s_preint.delta_p);
    
    // 更新协方差和Jacobian
    update_covariance(dt, &acc_mid, &gyro_mid);
    
    s_preint.dt += dt;
    s_preint.end_time_us = imu_curr->timestamp_us;
}

static void update_covariance(float dt, const t32vio_vec3_t *acc, const t32vio_vec3_t *gyro) {
    // 简化的协方差传播
    // 实际应该使用完整的误差状态传播矩阵
    
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    
    // 噪声功率谱密度
    float n2_gyro = s_noise_gyro * s_noise_gyro;
    float n2_acc = s_noise_acc * s_noise_acc;
    
    // 对角线元素增加噪声
    for (int i = 0; i < 3; i++) {
        s_preint.covariance[i][i] += n2_acc * dt3 / 3.0f;      // 位置
        s_preint.covariance[i+3][i+3] += n2_acc * dt;           // 速度
        s_preint.covariance[i+6][i+6] += n2_gyro * dt;          // 姿态
    }
    
    // Jacobian更新 (简化)
    // 实际应该使用完整的矩阵更新
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            s_preint.jacobian_p_ba[i][j] += (i == j) ? (-0.5f * dt2) : 0.0f;
            s_preint.jacobian_v_ba[i][j] += (i == j) ? (-dt) : 0.0f;
        }
    }
}

static void quat_integrate(t32vio_quat_t *q, const t32vio_vec3_t *omega, float dt) {
    // 指数映射: q = exp(0.5 * omega * dt)
    float theta_norm = sqrtf(omega->x * omega->x + 
                            omega->y * omega->y + 
                            omega->z * omega->z);
    
    if (theta_norm < 1e-6f) {
        // 小角度近似
        q->w = 1.0f;
        q->x = omega->x * dt * 0.5f;
        q->y = omega->y * dt * 0.5f;
        q->z = omega->z * dt * 0.5f;
    } else {
        float half_theta = theta_norm * dt * 0.5f;
        float s = sinf(half_theta) / theta_norm;
        q->w = cosf(half_theta);
        q->x = omega->x * s;
        q->y = omega->y * s;
        q->z = omega->z * s;
    }
    
    t32vio_quat_normalize(q);
}
