#!/bin/bash
# 测试CMake配置 (不实际编译)

echo "========================================"
echo "  测试CMake配置"
echo "========================================"
echo ""

# 测试1: 检查工具链文件语法
echo "[测试1] 检查工具链文件..."
if [ -f "cmake/mips-linux-gnu.cmake" ]; then
    echo "✓ 工具链文件存在"
    # 检查关键配置
    grep -q "CMAKE_SYSTEM_NAME" cmake/mips-linux-gnu.cmake && echo "✓ CMAKE_SYSTEM_NAME 已设置"
    grep -q "CMAKE_SYSTEM_PROCESSOR" cmake/mips-linux-gnu.cmake && echo "✓ CMAKE_SYSTEM_PROCESSOR 已设置"
    grep -q "CMAKE_C_COMPILER" cmake/mips-linux-gnu.cmake && echo "✓ CMAKE_C_COMPILER 已设置"
    grep -q "CMAKE_CXX_COMPILER" cmake/mips-linux-gnu.cmake && echo "✓ CMAKE_CXX_COMPILER 已设置"
else
    echo "✗ 工具链文件不存在"
    exit 1
fi
echo ""

# 测试2: 检查CMakeLists.txt
echo "[测试2] 检查CMakeLists.txt..."
if [ -f "CMakeLists.txt" ]; then
    echo "✓ CMakeLists.txt 存在"
    grep -q "CMAKE_CROSSCOMPILING" CMakeLists.txt && echo "✓ 交叉编译检测已添加"
    grep -q "CMAKE_CXX_STANDARD 11" CMakeLists.txt && echo "✓ C++11标准已设置"
else
    echo "✗ CMakeLists.txt 不存在"
    exit 1
fi
echo ""

# 测试3: 尝试运行CMake (干运行)
echo "[测试3] 测试CMake配置..."
mkdir -p build-test
cd build-test

# 使用假编译器测试CMake解析
cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/mips-linux-gnu.cmake \
         -DCMAKE_C_COMPILER=/bin/false \
         -DCMAKE_CXX_COMPILER=/bin/false \
         -DBUILD_TESTS=OFF 2>&1 | grep -E "(Error|error|Error:|error:)" | head -5

if [ $? -eq 0 ]; then
    echo "✗ CMake解析错误"
else
    echo "✓ CMake配置语法正确"
fi

cd ..
rm -rf build-test

echo ""
echo "========================================"
echo "  配置测试完成"
echo "========================================"
echo ""
echo "注意: 实际编译需要安装MIPS交叉编译器"
echo ""
