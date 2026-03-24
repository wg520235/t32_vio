# 交叉编译工具链 (MIPS Linux)
# 君正T32芯片专用工具链

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR mips)

# 工具链根目录 - 自动检测常见路径
if(EXISTS "/opt/t32_toolchain/mips-gcc540-glibc222-r3.3.7.mxu2.mem03")
    set(T32_TOOLCHAIN_ROOT "/opt/t32_toolchain/mips-gcc540-glibc222-r3.3.7.mxu2.mem03")
elseif(EXISTS "/opt/mips-gcc540-glibc222-r3.3.7.mxu2.mem03")
    set(T32_TOOLCHAIN_ROOT "/opt/mips-gcc540-glibc222-r3.3.7.mxu2.mem03")
elseif(EXISTS "$ENV{T32_TOOLCHAIN}")
    set(T32_TOOLCHAIN_ROOT "$ENV{T32_TOOLCHAIN}")
else()
    message(FATAL_ERROR "T32 toolchain not found. Please set T32_TOOLCHAIN environment variable or install to /opt/t32_toolchain/")
endif()

message(STATUS "T32 Toolchain: ${T32_TOOLCHAIN_ROOT}")

# 指定交叉编译器
set(CMAKE_C_COMPILER "${T32_TOOLCHAIN_ROOT}/bin/mips-linux-gnu-gcc" CACHE FILEPATH "C compiler")
set(CMAKE_CXX_COMPILER "${T32_TOOLCHAIN_ROOT}/bin/mips-linux-gnu-g++" CACHE FILEPATH "C++ compiler")

# 指定sysroot
set(CMAKE_SYSROOT "${T32_TOOLCHAIN_ROOT}/mips-linux-gnu/libc")

# 查找程序、库、头文件时不会搜索主机目录
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# 编译选项 - 针对君正T32优化
# T32使用MIPS32R2架构，带硬件浮点
set(CMAKE_C_FLAGS "-mips32r2 -mhard-float -O2 -ffunction-sections -fdata-sections" CACHE STRING "C flags")
set(CMAKE_CXX_FLAGS "-mips32r2 -mhard-float -O2 -std=c++11 -ffunction-sections -fdata-sections" CACHE STRING "C++ flags")

# 链接选项
set(CMAKE_EXE_LINKER_FLAGS "-static -Wl,--gc-sections" CACHE STRING "Linker flags")

# 禁用一些在嵌入式平台可能有问题的特性
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
