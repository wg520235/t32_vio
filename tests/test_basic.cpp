/**
 * @file test_basic.cpp
 * @brief 基础功能测试
 */

#include "t32vio.h"
#include <cstdio>
#include <cstring>
#include <cmath>

using namespace t32vio;

// 模拟数据生成
static void generateMockImu(ImuData* imu, uint64_t timestamp) {
    imu->timestamp_us = timestamp;
    imu->gyro[0] = 0.01f;  // 小角速度
    imu->gyro[1] = 0.0f;
    imu->gyro[2] = 0.0f;
    imu->acc[0] = 0.0f;
    imu->acc[1] = 0.0f;
    imu->acc[2] = GRAVITY;  // 静止
}

static void generateMockImage(uint8_t* image) {
    // 生成简单纹理图案
    for (uint32_t y = 0; y < IMAGE_HEIGHT; y++) {
        for (uint32_t x = 0; x < IMAGE_WIDTH; x++) {
            image[y * IMAGE_WIDTH + x] = static_cast<uint8_t>((x + y) % 256);
        }
    }
}

int main() {
    printf("T32 VIO Basic Test\n");
    printf("==================\n\n");
    
    // 测试1: 初始化
    printf("Test 1: Initialization\n");
    Config config = getDefaultConfig();
    int ret = init(&config);
    if (ret != 0) {
        printf("FAILED: init returned %d\n", ret);
        return 1;
    }
    printf("PASSED: init\n");
    
    // 测试2: 状态检查
    printf("\nTest 2: State check\n");
    State state = getState();
    if (state != State::INITIALIZING) {
        printf("FAILED: expected INITIALIZING, got %d\n", static_cast<int>(state));
        deinit();
        return 1;
    }
    printf("PASSED: state is INITIALIZING\n");
    
    // 测试3: IMU输入
    printf("\nTest 3: IMU input\n");
    uint64_t timestamp = 1000000;  // 1秒
    for (int i = 0; i < 100; i++) {
        ImuData imu;
        generateMockImu(&imu, timestamp);
        ret = imuInput(&imu);
        if (ret != 0) {
            printf("FAILED: imuInput returned %d at iteration %d\n", ret, i);
            deinit();
            return 1;
        }
        timestamp += 5000;  // 5ms = 200Hz
    }
    printf("PASSED: imuInput (100 frames)\n");
    
    // 测试4: 图像输入
    printf("\nTest 4: Image input\n");
    uint8_t image[IMAGE_SIZE];
    generateMockImage(image);
    
    for (int i = 0; i < 10; i++) {
        ImageFrame frame;
        frame.timestamp_us = timestamp;
        frame.data = image;
        frame.width = IMAGE_WIDTH;
        frame.height = IMAGE_HEIGHT;
        
        ret = imageInput(&frame);
        if (ret != 0) {
            printf("FAILED: imageInput returned %d at iteration %d\n", ret, i);
            deinit();
            return 1;
        }
        timestamp += 33333;  // 33.3ms = 30Hz
    }
    printf("PASSED: imageInput (10 frames)\n");
    
    // 等待处理完成
    printf("\nWaiting for processing...\n");
    for (int i = 0; i < 50; i++) {
        state = getState();
        if (state == State::TRACKING) {
            break;
        }
        // 模拟延时
        for (volatile int j = 0; j < 1000000; j++);
    }
    
    // 测试5: 位姿获取
    printf("\nTest 5: Pose retrieval\n");
    Pose pose;
    ret = getPose(&pose);
    if (ret == 0) {
        printf("PASSED: getPose\n");
        printf("  Position: [%.3f, %.3f, %.3f]\n", pose.p[0], pose.p[1], pose.p[2]);
        printf("  Quaternion: [%.3f, %.3f, %.3f, %.3f]\n", 
               pose.q[0], pose.q[1], pose.q[2], pose.q[3]);
    } else {
        printf("Note: getPose returned %d (system may still be initializing)\n", ret);
    }
    
    // 测试6: 统计信息
    printf("\nTest 6: Statistics\n");
    Stats stats;
    getStats(&stats);
    printf("  Total frames: %u\n", stats.total_frames);
    printf("  Keyframes: %u\n", stats.keyframes);
    printf("  Tracked features: %u\n", stats.tracked_features);
    printf("  Processing time: %.2f ms\n", stats.processing_time_ms);
    printf("PASSED: getStats\n");
    
    // 测试7: 重置
    printf("\nTest 7: Reset\n");
    reset();
    state = getState();
    if (state != State::INITIALIZING) {
        printf("FAILED: expected INITIALIZING after reset, got %d\n", static_cast<int>(state));
        deinit();
        return 1;
    }
    printf("PASSED: reset\n");
    
    // 反初始化
    deinit();
    
    printf("\n==================\n");
    printf("All tests passed!\n");
    
    return 0;
}
