# T32 VIO - 高精度单目视觉惯性里程计

专为君正 T32 芯片设计的高性能 VIO 系统。

## 硬件要求

- **芯片**: 君正 T32 (MIPS架构, 1.2GHz)
- **内存**: 64MB RAM
- **NPU**: 1 TOPS (可选使用)
- **摄像头**: 320x240@180fps 全局曝光 CMOS
- **IMU**: 200-400Hz 输出

## 编译

```bash
mkdir build
cd build
cmake ..
make -j4
```

## 交叉编译 (示例)

```bash
mkdir build
cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/mips-linux-gnu.cmake
make -j4
```

## 使用

```cpp
#include "t32vio.h"

// 初始化
t32vio::Config config = t32vio::getDefaultConfig();
t32vio::init(&config);

// 主循环
while (running) {
    // 输入IMU数据
    t32vio::ImuData imu;
    // ... 填充imu数据
    t32vio::imuInput(&imu);
    
    // 输入图像
    t32vio::ImageFrame image;
    image.data = camera_buffer;
    image.timestamp_us = get_timestamp();
    t32vio::imageInput(&image);
    
    // 获取位姿
    t32vio::Pose pose;
    if (t32vio::getPose(&pose) == 0) {
        // 使用pose.p (位置) 和 pose.q (姿态)
    }
}

// 反初始化
t32vio::deinit();
```

## 性能指标

| 指标 | 目标值 |
|------|--------|
| 位置漂移 | < 0.3% |
| 姿态精度 | < 0.5° |
| 输出频率 | 200Hz |
| 延迟 | < 8ms |
| 内存占用 | < 45MB |

## 算法架构

- **前端**: FAST角点 + 金字塔LK光流
- **后端**: MSCKF (Multi-State Constraint Kalman Filter)
- **融合**: 紧耦合 IMU + 视觉

## 目录结构

```
t32-vio/
├── include/          # 头文件
├── src/              # 源文件
├── tests/            # 测试代码
├── docs/             # 文档
├── CMakeLists.txt    # CMake配置
└── README.md         # 本文件
```

## 许可证

MIT License
