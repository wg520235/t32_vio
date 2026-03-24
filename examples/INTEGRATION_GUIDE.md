# T32 VIO 集成指南

## 快速开始

### 1. 编译演示程序

```bash
cd t32-vio
mkdir build-mips && cd build-mips
cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/mips-linux-gnu.cmake
make t32_vio_demo
```

### 2. 运行演示程序

```bash
./t32_vio_demo [波特率]
# 例如:
./t32_vio_demo 115200
./t32_vio_demo 921600
```

## 集成到你的工程

### 步骤1: 接入摄像头数据 (NV12格式)

```cpp
// 从摄像头获取NV12帧
uint8_t nv12_buffer[320 * 240 * 1.5];  // NV12大小

// 提取Y通道 (VIO只需要灰度图)
uint8_t y_buffer[320 * 240];
memcpy(y_buffer, nv12_buffer, 320 * 240);

// 输入VIO
t32vio::ImageFrame image;
image.timestamp_us = get_timestamp_us();
image.data = y_buffer;
image.width = 320;
image.height = 240;
t32vio::imageInput(&image);
```

### 步骤2: 接入IMU数据

```cpp
t32vio::ImuData imu;
imu.timestamp_us = get_timestamp_us();
imu.gyro[0] = gyro_x;  // rad/s
imu.gyro[1] = gyro_y;
imu.gyro[2] = gyro_z;
imu.acc[0] = acc_x;    // m/s^2
imu.acc[1] = acc_y;
imu.acc[2] = acc_z;

t32vio::imuInput(&imu);
```

### 步骤3: 获取位姿并发送给飞控

```cpp
t32vio::Pose pose;
if (t32vio::getPose(&pose) == 0) {
    // pose.p[0], pose.p[1], pose.p[2] - 位置X,Y,Z (米)
    // pose.q[0], pose.q[1], pose.q[2], pose.q[3] - 姿态四元数W,X,Y,Z
    // pose.v[0], pose.v[1], pose.v[2] - 速度X,Y,Z (m/s)
    
    // 通过UART发送给飞控MCU
    send_to_mcu(pose);
}
```

## UART通信协议

### 数据格式 (二进制，小端)

| 字节 | 内容 | 说明 |
|------|------|------|
| 0-1 | 0xAA 0x55 | 帧头 |
| 2 | 40 | 数据长度 |
| 3-6 | float | 位置X (米) |
| 7-10 | float | 位置Y (米) |
| 11-14 | float | 位置Z (米) |
| 15-18 | float | 四元数W |
| 19-22 | float | 四元数X |
| 23-26 | float | 四元数Y |
| 27-30 | float | 四元数Z |
| 31-34 | float | 速度X (m/s) |
| 35-38 | float | 速度Y (m/s) |
| 39-42 | float | 速度Z (m/s) |
| 43 | uint8 | 状态 (0-3) |
| 44-45 | uint16 | CRC16 |
| 46 | 0xEE | 帧尾 |

### 状态码

- 0: 未初始化
- 1: 初始化中
- 2: 跟踪中 (正常)
- 3: 丢失

## 硬件连接

### T32芯片连接图

```
+-------------+        +-------------+
|   T32芯片   |        |   飞控MCU   |
|             |        |             |
| 摄像头接口  |        |             |
|    (DVP)    |        |             |
|      |      |        |             |
|   NV12数据  |        |             |
|      |      |        |             |
|  VIO算法    |        |             |
|      |      |        |             |
|  UART1(TX)  |------->|  UART(RX)   |
|             |        |             |
+-------------+        +-------------+
```

### 推荐的UART配置

- 波特率: 115200 或 921600
- 数据位: 8
- 停止位: 1
- 校验: 无
- 流控: 无

## 性能指标

| 指标 | 数值 |
|------|------|
| 输入帧率 | 30-60 fps |
| IMU频率 | 200-400 Hz |
| 输出频率 | 200 Hz |
| 延迟 | < 10ms |
| 内存占用 | < 45MB |
| UART数据率 | ~10KB/s @ 200Hz |

## 常见问题

### Q: 摄像头数据格式不是NV12怎么办？

A: VIO只需要Y通道（灰度图）。如果你的摄像头输出其他格式：
- YUV422: 提取Y通道
- RGB: 转换为灰度: Y = 0.299R + 0.587G + 0.114B

### Q: IMU坐标系和摄像头不一致怎么办？

A: 修改 `config.p_ic` 和 `config.q_ic` 配置外参。

### Q: 位姿漂移大怎么办？

A: 1. 检查摄像头和IMU时间同步
   2. 确保摄像头帧率稳定
   3. 校准IMU零偏
   4. 检查外参是否准确

### Q: VIO状态一直是"初始化中"？

A: 1. 确保有足够的运动（平移或旋转）
   2. 检查图像是否有足够的纹理
   3. 确保IMU数据正常

## 调试技巧

### 启用调试输出

```cpp
t32vio::Stats stats;
t32vio::getStats(&stats);
printf("特征点: %u, 处理时间: %.2fms\n", 
       stats.tracked_features, 
       stats.processing_time_ms);
```

### 检查数据流

```cpp
t32vio::State state = t32vio::getState();
if (state == t32vio::State::TRACKING) {
    // VIO正常工作
} else if (state == t32vio::State::LOST) {
    // 丢失跟踪，需要重置
    t32vio::reset();
}
```
