# T32 VIO 系统设计文档

## 1. 系统概述

### 1.1 项目目标
在君正 T32 芯片（1.2GHz MIPS, 64MB RAM, 1TOPS NPU）上实现高精度单目 VIO 系统，用于室内高速无人机。

### 1.2 性能指标
| 指标 | 目标值 |
|------|--------|
| 位置漂移 | < 0.3% |
| 姿态精度 | < 0.5° |
| 输出频率 | 200Hz |
| 端到端延迟 | < 8ms |
| 内存占用 | < 45MB |

### 1.3 核心算法
- **前端**: FAST角点 + 金字塔LK光流
- **后端**: MSCKF (Multi-State Constraint Kalman Filter)
- **融合**: 紧耦合 IMU + 视觉

---

## 2. 软件架构

```
┌─────────────────────────────────────────────────────────────┐
│                        API Layer                             │
│  get_pose(), get_velocity(), is_tracking(), get_status()     │
└─────────────────────────────────────────────────────────────┘
                              │
┌─────────────────────────────────────────────────────────────┐
│                      MSCKF Core                              │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐  │
│  │ State Prop  │  │ Update      │  │ Marginalization     │  │
│  │ (IMU 200Hz) │  │ (Visual)    │  │ (Sliding Window)    │  │
│  └─────────────┘  └─────────────┘  └─────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                              │
        ┌─────────────────────┼─────────────────────┐
        ▼                     ▼                     ▼
┌───────────────┐    ┌───────────────┐    ┌───────────────┐
│ Visual Front  │    │ IMU Preint    │    │ Outlier Rej   │
│ - FAST detect │    │ - 200Hz       │    │ - RANSAC      │
│ - KLT track   │    │ - Delta p/q/v │    │ - Chi-square  │
│ - Keyframe sel│    │ - Covariance  │    │ - Parallax chk│
└───────────────┘    └───────────────┘    └───────────────┘
```

---

## 3. 数据流设计

### 3.1 线程模型
| 线程 | 优先级 | 频率 | 职责 |
|------|--------|------|------|
| IMU Thread | RT | 200-400Hz | 数据采集、预积分、状态传播 |
| Vision Thread | High | 30-60Hz | 特征提取、跟踪、关键帧检测 |
| MSCKF Thread | High | 10-20Hz | 更新、边缘化、协方差维护 |
| Output Thread | RT | 200Hz | 高频位姿输出 (插值) |

### 3.2 数据缓冲
```c
// 环形缓冲区设计，无锁队列
imu_buffer[256]      // IMU原始数据
feature_buffer[32]   // 特征帧数据
pose_buffer[64]      // 输出位姿历史
```

---

## 4. 状态向量设计

### 4.1 IMU状态 (15维)
```
X_imu = [p_wi, v_wi, q_wi, ba, bg]
        [  3    3     4    3   3 ] = 16维 (四元数4维，实际优化3维)
```

### 4.2 滑动窗口状态 (N个相机帧)
```
X_cam = [p_wc1, q_wc1, p_wc2, q_wc2, ..., p_wcN, q_wcN]
        [   3      4       3      4           3      4   ]
        
总维度: 15 + N * 7 (N=15时，共120维)
```

### 4.3 特征点表示
- 不维护全局3D点
- 仅保留滑动窗口内的观测
- 三角化后用于更新，然后边缘化丢弃

---

## 5. 关键算法细节

### 5.1 FAST角点检测
```c
// 自适应阈值，保证特征点数量
threshold = base_threshold;
while (num_corners < min_corners && threshold > min_threshold) {
    fast_detect(image, threshold, &corners);
    threshold *= 0.8;
}
// 网格均匀化，避免聚集
grid_non_max_suppression(corners, grid_size=40);
```

### 5.2 金字塔LK光流
```c
// 3层金字塔，应对快速运动
pyramid[0] = image;  // 320x240
pyramid[1] = downsample(pyramid[0]);  // 160x120
pyramid[2] = downsample(pyramid[1]);  // 80x60

// 从顶层开始跟踪，逐层细化
calcOpticalFlowPyrLK(pyramid, prev_pts, &curr_pts, &status);

// 逆向LK验证，剔除外点
verify_backward_LK(curr_pts, &back_pts);
outlier = (|back_pts - prev_pts| > threshold);
```

### 5.3 关键帧选择策略
```c
bool need_keyframe() {
    // 1. 跟踪点数量不足
    if (tracked_points < 100) return true;
    
    // 2. 平移足够大 (>5%平均深度)
    if (translation > 0.05 * average_depth) return true;
    
    // 3. 旋转足够大 (>5度)
    if (rotation_angle > 5.0 * DEG2RAD) return true;
    
    // 4. 时间间隔过长 (>100ms)
    if (time_since_last_kf > 0.1) return true;
    
    return false;
}
```

### 5.4 MSCKF更新
```c
// 1. 对滑动窗口内的每个特征点
for (feature in sliding_window) {
    // 2. 三角化得到3D点
    triangulate(feature.observations, &point_3d);
    
    // 3. 计算重投影误差
    for (obs in feature.observations) {
        residual = project(point_3d, cam_pose) - obs.pixel;
        
        // 4. 构建观测矩阵 H
        H = d(residual)/d(state);
        
        // 5. 卡方检验外点剔除
        if (residual^T * S^-1 * residual < chi2_threshold) {
            // 6. EKF更新
            K = P * H^T * (H * P * H^T + R)^-1;
            X = X + K * residual;
            P = (I - K * H) * P;
        }
    }
}
```

### 5.5 边缘化策略
```c
// 当滑动窗口满时，边缘化最旧的帧
if (window_size > MAX_WINDOW_SIZE) {
    // 1. 构建边缘化先验
    prior_H = J_marg^T * W * J_marg;
    prior_b = J_marg^T * W * r_marg;
    
    // 2. 将先验加入当前状态
    H_total = H_current + prior_H;
    b_total = b_current + prior_b;
    
    // 3. 移除最旧帧的状态
    remove_oldest_state();
}
```

---

## 6. 内存管理

### 6.1 内存池设计
```c
// 预分配，避免运行时malloc
struct MemoryPool {
    uint8_t image_pyramid[3 * 320 * 240];      // 230KB
    Feature features[MAX_FEATURES];             // 200 * 64B = 12.8KB
    Frame frames[MAX_WINDOW_SIZE];              // 15 * 512B = 7.5KB
    float state_covariance[MAX_STATE_DIM^2];    // 120^2 * 4B = 57.6KB
    // ...
} pool;
```

### 6.2 内存预算
| 模块 | 内存 | 说明 |
|------|------|------|
| 图像金字塔 | 2 MB | 3层 + 双缓冲 |
| 特征点管理 | 4 MB | 200点 × 多帧 |
| 滑动窗口状态 | 12 MB | 15帧 + 协方差 |
| IMU预积分 | 2 MB | 缓冲 + 状态 |
| 代码/堆栈 | 5 MB | 程序运行 |
| 系统预留 | 20 MB | 安全缓冲 |
| **总计** | **~45 MB** | |

---

## 7. 性能优化

### 7.1 MIPS优化
- 使用定点数替代部分浮点运算
- 手写关键循环的汇编优化
- 利用MIPS SIMD (如果有)

### 7.2 算法优化
- FAST检测用机器码优化版本
- LK光流用迭代次数限制 (max 30次)
- EKF更新稀疏化计算

### 7.3 流水线优化
```
时间轴:
IMU:  I-I-I-I-I-I-I-I-I-I-I-I-I-I-I-I-I-I-I-I  (200Hz)
Vision:  V---V---V---V---V---V---V---V---V---   (30Hz)
MSCKF:      U-----U-----U-----U-----U-----U---   (15Hz)
Output: O-O-O-O-O-O-O-O-O-O-O-O-O-O-O-O-O-O-O  (200Hz)
```

---

## 8. 文件结构

```
t32-vio/
├── include/
│   ├── t32vio.h          # 公共API
│   ├── msckf.h           # MSCKF核心
│   ├── frontend.h        # 视觉前端
│   ├── imu_preint.h      # IMU预积分
│   └── utils.h           # 工具函数
├── src/
│   ├── msckf.c           # MSCKF实现
│   ├── frontend.c        # 前端实现
│   ├── imu_preint.c      # 预积分实现
│   ├── feature.c         # 特征管理
│   └── utils.c           # 工具实现
├── third_party/
│   └── fast/             # FAST角点库
├── tests/
│   └── test_basic.c      # 基础测试
└── Makefile
```

---

## 9. 开发计划

| 阶段 | 时间 | 目标 |
|------|------|------|
| Phase 1 | 1周 | 基础框架 + FAST + LK |
| Phase 2 | 1周 | IMU预积分 + 松耦合 |
| Phase 3 | 1周 | MSCKF紧耦合 |
| Phase 4 | 1周 | 优化 + 测试 + 调参 |

---

*文档版本: 1.0*
*日期: 2026-03-24*
