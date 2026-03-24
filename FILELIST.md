# T32 VIO 项目文件清单

## 文档
- docs/design.md          # 系统设计文档
- constraints.md          # 硬件约束分析
- README.md               # 项目说明

## 头文件 (include/)
- t32vio.h                # 主API头文件 (C++)
- frontend.h              # 视觉前端头文件
- imu_preint.h            # IMU预积分头文件 (保留)
- msckf.h                 # MSCKF头文件 (保留)

## 源文件 (src/)
- t32vio.cpp              # 主API实现
- frontend.cpp            # 视觉前端实现 (FAST + KLT)
- msckf.cpp               # MSCKF实现 (C++11 + Eigen)

## 构建配置
- CMakeLists.txt          # 主CMake配置
- cmake/mips-linux-gnu.cmake.example  # 交叉编译工具链示例

## 测试
- tests/CMakeLists.txt    # 测试CMake配置
- tests/test_basic.cpp    # 基础功能测试

## 编译步骤

### 本地编译 (测试)
```bash
cd /path/to/t32-vio
mkdir build && cd build
cmake ..
make -j4
./tests/test_basic
```

### 交叉编译 (T32芯片)
```bash
# 1. 根据实际工具链修改 cmake/mips-linux-gnu.cmake
cp cmake/mips-linux-gnu.cmake.example cmake/mips-linux-gnu.cmake
# 编辑文件，设置正确的编译器路径

# 2. 编译
mkdir build && cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/mips-linux-gnu.cmake
make -j4

# 3. 输出: libt32vio.a (静态库)
```

## 使用示例

```cpp
#include "t32vio.h"

int main() {
    // 初始化
    t32vio::Config config = t32vio::getDefaultConfig();
    t32vio::init(&config);
    
    // 主循环
    while (true) {
        // 输入IMU (200Hz)
        t32vio::ImuData imu = {...};
        t32vio::imuInput(&imu);
        
        // 输入图像 (30-60Hz)
        t32vio::ImageFrame image = {...};
        t32vio::imageInput(&image);
        
        // 获取位姿 (200Hz输出)
        t32vio::Pose pose;
        if (t32vio::getPose(&pose) == 0) {
            // pose.p[3]: 位置 x,y,z
            // pose.q[4]: 姿态四元数 w,x,y,z
        }
    }
    
    t32vio::deinit();
    return 0;
}
```

## 关键设计

1. **内存控制**: 预分配内存池，无运行时malloc
2. **多线程**: IMU/视觉/输出三线程并行
3. **算法**: MSCKF紧耦合，滑动窗口优化
4. **精度**: 目标 <0.3% 漂移，200Hz输出

## 待优化项

- [ ] MIPS SIMD优化 (如有)
- [ ] 定点化加速
- [ ] NPU加速评估
- [ ] 更完善的测试用例
- [ ] 实际硬件调参
