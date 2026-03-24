/**
 * @file t32_vio_demo.cpp
 * @brief T32 VIO 集成演示程序
 * 
 * 演示如何：
 * 1. 接入NV12视频流
 * 2. 接入IMU数据
 * 3. 获取VIO输出位姿
 * 4. 通过UART发送给飞控MCU
 */

#include "t32vio.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <signal.h>
#include <sys/time.h>

// ============ 配置参数 ============
// UART配置 (连接飞控MCU)
#define UART_DEVICE     "/dev/ttyS1"      // 根据实际情况修改
#define UART_BAUDRATE   B115200           // 波特率

// 摄像头配置
#define CAM_WIDTH       320
#define CAM_HEIGHT      240
#define CAM_FPS         180

// IMU配置
#define IMU_RATE_HZ     200

// 数据流配置
#define NV12_BUFFER_SIZE ((int)(CAM_WIDTH * CAM_HEIGHT * 1.5))  // Y + UV

// ============ 全局变量 ============
static volatile bool g_running = true;
static int g_uart_fd = -1;

// 模拟数据缓冲区 (实际工程中替换为真实数据源)
static uint8_t g_nv12_buffer[NV12_BUFFER_SIZE];
static uint64_t g_frame_count = 0;
static uint64_t g_imu_count = 0;

// ============ 信号处理 ============
static void signal_handler(int sig) {
    printf("\n收到信号 %d，正在退出...\n", sig);
    g_running = false;
}

// ============ UART通信 ============

/**
 * @brief 初始化UART串口
 */
int uart_init(const char* device, int baudrate) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        perror("打开UART失败");
        return -1;
    }
    
    // 配置串口
    struct termios options;
    tcgetattr(fd, &options);
    
    // 设置波特率
    switch (baudrate) {
        case 115200:  cfsetispeed(&options, B115200); cfsetospeed(&options, B115200); break;
        case 921600:  cfsetispeed(&options, B921600); cfsetospeed(&options, B921600); break;
        default:      cfsetispeed(&options, B115200); cfsetospeed(&options, B115200); break;
    }
    
    // 8N1
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag |= CREAD | CLOCAL;
    
    // 原始模式
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    
    tcsetattr(fd, TCSANOW, &options);
    tcflush(fd, TCIOFLUSH);
    
    printf("UART已初始化: %s @ %d baud\n", device, baudrate);
    return fd;
}

/**
 * @brief 发送位姿数据到飞控MCU
 * 
 * 数据协议 (二进制，小端格式):
 * 字节0:    帧头0xAA
 * 字节1:    帧头0x55
 * 字节2:    数据长度
 * 字节3-6:  位置X (float, 米)
 * 字节7-10: 位置Y (float, 米)
 * 字节11-14:位置Z (float, 米)
 * 字节15-18:四元数W (float)
 * 字节19-22:四元数X (float)
 * 字节23-26:四元数Y (float)
 * 字节27-30:四元数Z (float)
 * 字节31-34:速度X (float, m/s)
 * 字节35-38:速度Y (float, m/s)
 * 字节39-42:速度Z (float, m/s)
 * 字节43:   状态 (0=未初始化, 1=初始化中, 2=跟踪中, 3=丢失)
 * 字节44-45:CRC16校验
 * 字节46:   帧尾0xEE
 */
void send_pose_to_mcu(int uart_fd, const t32vio::Pose& pose, t32vio::State state) {
    if (uart_fd < 0) return;
    
    uint8_t packet[47];
    uint8_t* p = packet;
    
    // 帧头
    *p++ = 0xAA;
    *p++ = 0x55;
    
    // 数据长度
    *p++ = 40;  // 40字节数据
    
    // 位置 (米)
    memcpy(p, &pose.p[0], 4); p += 4;  // X
    memcpy(p, &pose.p[1], 4); p += 4;  // Y
    memcpy(p, &pose.p[2], 4); p += 4;  // Z
    
    // 姿态四元数
    memcpy(p, &pose.q[0], 4); p += 4;  // W
    memcpy(p, &pose.q[1], 4); p += 4;  // X
    memcpy(p, &pose.q[2], 4); p += 4;  // Y
    memcpy(p, &pose.q[3], 4); p += 4;  // Z
    
    // 速度 (m/s)
    memcpy(p, &pose.v[0], 4); p += 4;  // VX
    memcpy(p, &pose.v[1], 4); p += 4;  // VY
    memcpy(p, &pose.v[2], 4); p += 4;  // VZ
    
    // 状态
    *p++ = static_cast<uint8_t>(state);
    
    // CRC16 (简化，实际应该计算)
    uint16_t crc = 0x1234;
    *p++ = crc & 0xFF;
    *p++ = (crc >> 8) & 0xFF;
    
    // 帧尾
    *p++ = 0xEE;
    
    // 发送
    write(uart_fd, packet, sizeof(packet));
}

// ============ 摄像头数据接入 ============

/**
 * @brief 从NV12中提取Y通道 (灰度图)
 * 
 * NV12格式:
 * - Y通道: width * height 字节
 * - UV通道: width * height / 2 字节 (交错存储)
 */
void extract_y_from_nv12(const uint8_t* nv12, uint8_t* y_buffer, int width, int height) {
    // NV12的Y通道就是前width*height字节
    memcpy(y_buffer, nv12, width * height);
}

/**
 * @brief 模拟从摄像头获取NV12帧
 * 
 * 实际工程中，这里应该：
 * 1. 从V4L2接口读取
 * 2. 从摄像头驱动缓冲区获取
 * 3. 通过DMA直接访问
 */
int get_camera_frame(uint8_t* nv12_buffer) {
    // TODO: 替换为实际的摄像头数据获取
    // 示例: 生成测试图案
    static uint8_t pattern = 0;
    for (int i = 0; i < CAM_WIDTH * CAM_HEIGHT; i++) {
        nv12_buffer[i] = (i + pattern) % 256;  // Y通道
    }
    // UV通道填充128 (中性色)
    for (int i = CAM_WIDTH * CAM_HEIGHT; i < NV12_BUFFER_SIZE; i++) {
        nv12_buffer[i] = 128;
    }
    pattern += 1;
    
    g_frame_count++;
    return 0;
}

// ============ IMU数据接入 ============

/**
 * @brief 模拟从IMU获取数据
 * 
 * 实际工程中，这里应该：
 * 1. 从SPI/I2C接口读取
 * 2. 从中断服务程序获取
 * 3. 从传感器驱动缓冲区获取
 */
int get_imu_data(t32vio::ImuData& imu) {
    // TODO: 替换为实际的IMU数据获取
    struct timeval tv;
    gettimeofday(&tv, NULL);
    imu.timestamp_us = tv.tv_sec * 1000000ULL + tv.tv_usec;
    
    // 模拟静止状态的数据
    imu.gyro[0] = 0.0f;   // 角速度 X (rad/s)
    imu.gyro[1] = 0.0f;   // 角速度 Y (rad/s)
    imu.gyro[2] = 0.0f;   // 角速度 Z (rad/s)
    
    imu.acc[0] = 0.0f;    // 加速度 X (m/s^2)
    imu.acc[1] = 0.0f;    // 加速度 Y (m/s^2)
    imu.acc[2] = 9.81f;   // 加速度 Z (重力)
    
    g_imu_count++;
    return 0;
}

// ============ 主程序 ============

int main(int argc, char* argv[]) {
    printf("========================================\n");
    printf("  T32 VIO 集成演示程序\n");
    printf("========================================\n\n");
    
    // 设置信号处理
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // 1. 初始化UART (连接飞控MCU)
    printf("[1/4] 初始化UART...\n");
    int baudrate = 115200;
    if (argc > 1) {
        baudrate = atoi(argv[1]);
    }
    g_uart_fd = uart_init(UART_DEVICE, baudrate);
    if (g_uart_fd < 0) {
        printf("警告: UART初始化失败，将继续运行但不发送数据\n");
    }
    
    // 2. 初始化VIO
    printf("[2/4] 初始化VIO...\n");
    t32vio::Config config = t32vio::getDefaultConfig();
    
    // 根据实际摄像头参数修改
    config.fx = 240.0f;   // 焦距X (像素)
    config.fy = 240.0f;   // 焦距Y (像素)
    config.cx = 160.0f;   // 主点X
    config.cy = 120.0f;   // 主点Y
    
    // IMU-摄像头外参 (根据实际安装修改)
    config.p_ic[0] = 0.0f;  // IMU到摄像头的X偏移 (米)
    config.p_ic[1] = 0.0f;  // Y偏移
    config.p_ic[2] = 0.0f;  // Z偏移
    config.q_ic[0] = 1.0f;  // 四元数W (无旋转)
    config.q_ic[1] = 0.0f;  // X
    config.q_ic[2] = 0.0f;  // Y
    config.q_ic[3] = 0.0f;  // Z
    
    if (t32vio::init(&config) != 0) {
        printf("错误: VIO初始化失败\n");
        return 1;
    }
    printf("VIO初始化成功\n\n");
    
    // 3. 分配缓冲区
    printf("[3/4] 分配缓冲区...\n");
    uint8_t* y_buffer = new uint8_t[CAM_WIDTH * CAM_HEIGHT];
    printf("缓冲区分配完成\n\n");
    
    // 4. 主循环
    printf("[4/4] 开始主循环 (按Ctrl+C退出)...\n\n");
    
    uint64_t last_print_time = 0;
    uint64_t last_frame_time = 0;
    uint64_t frame_interval_us = 1000000 / 30;  // 30fps = 33333us
    
    while (g_running) {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        uint64_t current_time = tv.tv_sec * 1000000ULL + tv.tv_usec;
        
        // ---- 输入IMU数据 (200Hz) ----
        t32vio::ImuData imu;
        if (get_imu_data(imu) == 0) {
            t32vio::imuInput(&imu);
        }
        
        // ---- 输入图像数据 (30Hz) ----
        if (current_time - last_frame_time >= frame_interval_us) {
            // 获取NV12帧
            if (get_camera_frame(g_nv12_buffer) == 0) {
                // 提取Y通道
                extract_y_from_nv12(g_nv12_buffer, y_buffer, CAM_WIDTH, CAM_HEIGHT);
                
                // 输入VIO
                t32vio::ImageFrame image;
                image.timestamp_us = current_time;
                image.data = y_buffer;
                image.width = CAM_WIDTH;
                image.height = CAM_HEIGHT;
                t32vio::imageInput(&image);
                
                last_frame_time = current_time;
            }
        }
        
        // ---- 获取VIO输出并发送给飞控 ----
        t32vio::Pose pose;
        if (t32vio::getPose(&pose) == 0) {
            t32vio::State state = t32vio::getState();
            
            // 发送到飞控MCU
            send_pose_to_mcu(g_uart_fd, pose, state);
        }
        
        // ---- 状态打印 (1秒一次) ----
        if (current_time - last_print_time >= 1000000) {
            t32vio::Stats stats;
            t32vio::getStats(&stats);
            t32vio::State state = t32vio::getState();
            
            const char* state_str[] = {"未初始化", "初始化中", "跟踪中", "丢失"};
            
            printf("状态: %s | 帧: %llu | IMU: %llu | 特征点: %u | 处理时间: %.2fms\n",
                   state_str[static_cast<int>(state)],
                   (unsigned long long)g_frame_count,
                   (unsigned long long)g_imu_count,
                   stats.tracked_features,
                   stats.processing_time_ms);
            
            last_print_time = current_time;
        }
        
        // 小延时，避免CPU占用过高
        usleep(1000);  // 1ms
    }
    
    // 清理
    printf("\n清理资源...\n");
    t32vio::deinit();
    delete[] y_buffer;
    
    if (g_uart_fd >= 0) {
        close(g_uart_fd);
    }
    
    printf("程序已退出\n");
    return 0;
}
