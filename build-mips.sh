#!/bin/bash
# T32 VIO 交叉编译脚本 (君正T32专用)
# 工具链路径: /opt/t32_toolchain/mips-gcc540-glibc222-r3.3.7.mxu2.mem03

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 工具链配置
TOOLCHAIN_ROOT="/opt/t32_toolchain/mips-gcc540-glibc222-r3.3.7.mxu2.mem03"
COMPILER_C="${TOOLCHAIN_ROOT}/bin/mips-linux-gnu-gcc"
COMPILER_CXX="${TOOLCHAIN_ROOT}/bin/mips-linux-gnu-g++"

echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}  T32 VIO 交叉编译脚本${NC}"
echo -e "${YELLOW}  君正T32芯片专用${NC}"
echo -e "${YELLOW}========================================${NC}"
echo ""

# 检查工具链
echo -e "${BLUE}[检查] 验证T32工具链...${NC}"

if [ ! -f "$COMPILER_C" ]; then
    echo -e "${RED}错误: 未找到C编译器: $COMPILER_C${NC}"
    exit 1
fi

if [ ! -f "$COMPILER_CXX" ]; then
    echo -e "${RED}错误: 未找到C++编译器: $COMPILER_CXX${NC}"
    exit 1
fi

echo -e "${GREEN}✓ 工具链路径: $TOOLCHAIN_ROOT${NC}"
echo ""

echo -e "${GREEN}编译器版本:${NC}"
$COMPILER_C --version | head -1
$COMPILER_CXX --version | head -1
echo ""

# 获取项目根目录
PROJECT_ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$PROJECT_ROOT"

# 创建构建目录
BUILD_DIR="build-mips"
if [ -d "$BUILD_DIR" ]; then
    echo -e "${YELLOW}[清理] 删除旧构建目录...${NC}"
    rm -rf "$BUILD_DIR"
fi

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

echo -e "${YELLOW}[1/4] 配置CMake...${NC}"
echo "  工具链: ../cmake/mips-linux-gnu.cmake"
echo "  C编译器: $COMPILER_C"
echo "  C++编译器: $COMPILER_CXX"
echo ""

# 运行CMake
cmake .. \
    -DCMAKE_TOOLCHAIN_FILE=../cmake/mips-linux-gnu.cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTS=OFF \
    2>&1 | tee cmake.log

if [ $? -ne 0 ]; then
    echo ""
    echo -e "${RED}========================================${NC}"
    echo -e "${RED}  CMake配置失败${NC}"
    echo -e "${RED}========================================${NC}"
    echo ""
    echo "错误日志:"
    tail -50 cmake.log
    exit 1
fi

echo -e "${GREEN}✓ CMake配置成功${NC}"
echo ""

echo -e "${YELLOW}[2/4] 编译...${NC}"
make -j$(nproc) 2>&1 | tee make.log

if [ $? -ne 0 ]; then
    echo ""
    echo -e "${RED}========================================${NC}"
    echo -e "${RED}  编译失败${NC}"
    echo -e "${RED}========================================${NC}"
    echo ""
    echo "错误日志:"
    tail -100 make.log
    exit 1
fi

echo -e "${GREEN}✓ 编译成功${NC}"
echo ""

echo -e "${YELLOW}[3/4] 检查输出...${NC}"
if [ -f "libt32vio.a" ]; then
    echo -e "${GREEN}✓ 静态库已生成:${NC}"
    ls -lh libt32vio.a
    echo ""
    
    # 检查文件类型
    echo "文件信息:"
    file libt32vio.a
    echo ""
    
    # 检查MIPS架构
    echo "目标架构:"
    ${TOOLCHAIN_ROOT}/bin/mips-linux-gnu-objdump -f libt32vio.a 2>/dev/null | head -10 || echo "(使用工具链objdump检查)"
else
    echo -e "${RED}错误: 未找到生成的库文件${NC}"
    exit 1
fi

echo ""
echo -e "${YELLOW}[4/4] 生成发布包...${NC}"

# 创建发布目录
RELEASE_DIR="t32vio-mips-t32-$(date +%Y%m%d)"
mkdir -p "$RELEASE_DIR"

# 复制文件
cp libt32vio.a "$RELEASE_DIR/"
cp -r ../include "$RELEASE_DIR/"
cp ../README.md "$RELEASE_DIR/" 2>/dev/null || true
cp ../FILELIST.md "$RELEASE_DIR/" 2>/dev/null || true

# 创建使用说明
cat > "$RELEASE_DIR/USAGE.txt" << 'EOF'
T32 VIO 静态库使用说明 (君正T32芯片)
======================================

1. 链接库文件
   在你的Makefile中添加:
   
   LIBS += -L/path/to/lib -lt32vio -lpthread

2. 包含头文件
   #include "t32vio.h"

3. 初始化VIO
   t32vio::Config config = t32vio::getDefaultConfig();
   t32vio::init(&config);

4. 输入数据
   // IMU数据 (200Hz)
   t32vio::ImuData imu = {...};
   t32vio::imuInput(&imu);
   
   // 图像数据 (30-60Hz)
   t32vio::ImageFrame image = {...};
   t32vio::imageInput(&image);

5. 获取位姿
   t32vio::Pose pose;
   if (t32vio::getPose(&pose) == 0) {
       // pose.p[3] - 位置 x,y,z (米)
       // pose.q[4] - 姿态四元数 w,x,y,z
       // pose.v[3] - 速度 x,y,z (m/s)
   }

6. 反初始化
   t32vio::deinit();

硬件要求:
  - 芯片: 君正T32
  - 内存: 64MB+
  - 摄像头: 320x240@180fps 全局曝光
  - IMU: 200-400Hz

更多信息请参考 README.md
EOF

# 打包
tar czf "${RELEASE_DIR}.tar.gz" "$RELEASE_DIR"
echo -e "${GREEN}✓ 发布包已生成: ${RELEASE_DIR}.tar.gz${NC}"

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  交叉编译完成!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "输出文件:"
echo "  静态库: $(pwd)/libt32vio.a"
echo "  发布包: $(pwd)/${RELEASE_DIR}.tar.gz"
echo ""
echo "文件大小:"
ls -lh libt32vio.a
ls -lh ${RELEASE_DIR}.tar.gz
echo ""
echo "下一步:"
echo "  1. 将 libt32vio.a 复制到T32芯片项目"
echo "  2. 将 include/ 目录复制到项目头文件路径"
echo "  3. 链接时添加: -lt32vio -lpthread"
echo ""
