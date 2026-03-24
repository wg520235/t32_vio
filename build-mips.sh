#!/bin/bash
# T32 VIO 交叉编译脚本
# 用于在x86主机上编译MIPS目标代码

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}  T32 VIO 交叉编译脚本${NC}"
echo -e "${YELLOW}========================================${NC}"
echo ""

# 检查交叉编译器
if ! command -v mips-linux-gnu-gcc &> /dev/null; then
    echo -e "${RED}错误: 未找到 mips-linux-gnu-gcc${NC}"
    echo ""
    echo "请安装MIPS交叉编译器:"
    echo "  Ubuntu/Debian:"
    echo "    sudo apt-get update"
    echo "    sudo apt-get install gcc-mips-linux-gnu g++-mips-linux-gnu"
    echo ""
    echo "  或从源码编译器:"
    echo "    https://www.kernel.org/pub/tools/crosstool/files/bin/x86_64/"
    echo ""
    exit 1
fi

echo -e "${GREEN}✓ 找到交叉编译器:${NC}"
mips-linux-gnu-gcc --version | head -1
mips-linux-gnu-g++ --version | head -1
echo ""

# 创建构建目录
BUILD_DIR="build-mips"
if [ -d "$BUILD_DIR" ]; then
    echo -e "${YELLOW}清理旧构建目录...${NC}"
    rm -rf "$BUILD_DIR"
fi

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# 运行CMake
echo -e "${YELLOW}[1/3] 配置CMake...${NC}"
cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/mips-linux-gnu.cmake \
         -DCMAKE_BUILD_TYPE=Release \
         -DBUILD_TESTS=OFF 2>&1

if [ $? -ne 0 ]; then
    echo -e "${RED}CMake配置失败${NC}"
    exit 1
fi
echo -e "${GREEN}✓ CMake配置成功${NC}"
echo ""

# 编译
echo -e "${YELLOW}[2/3] 编译...${NC}"
make -j$(nproc) 2>&1

if [ $? -ne 0 ]; then
    echo -e "${RED}编译失败${NC}"
    exit 1
fi
echo -e "${GREEN}✓ 编译成功${NC}"
echo ""

# 检查输出
echo -e "${YELLOW}[3/3] 检查输出...${NC}"
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
    mips-linux-gnu-objdump -f libt32vio.a 2>/dev/null | head -5 || echo "(需要安装binutils-mips-linux-gnu)"
else
    echo -e "${RED}错误: 未找到生成的库文件${NC}"
    exit 1
fi

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  交叉编译完成!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "输出文件: $(pwd)/libt32vio.a"
echo ""
echo "下一步:"
echo "  1. 将 libt32vio.a 复制到T32芯片项目"
echo "  2. 链接时添加: -lt32vio -lpthread"
echo ""
