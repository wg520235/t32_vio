# 交叉编译工具链 (MIPS Linux)
# 君正T32芯片使用

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR mips)

# 指定交叉编译器
set(CMAKE_C_COMPILER mips-linux-gnu-gcc CACHE FILEPATH "C compiler")
set(CMAKE_CXX_COMPILER mips-linux-gnu-g++ CACHE FILEPATH "C++ compiler")

# 指定sysroot (如果需要)
# set(CMAKE_SYSROOT /path/to/sysroot)

# 查找程序、库、头文件时不会搜索主机目录
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# 编译选项 - 针对君正T32优化
# T32使用MIPS32R2架构，带硬件浮点
set(CMAKE_C_FLAGS "-mips32r2 -mhard-float -O2" CACHE STRING "C flags")
set(CMAKE_CXX_FLAGS "-mips32r2 -mhard-float -O2 -std=c++11" CACHE STRING "C++ flags")

# 链接选项
set(CMAKE_EXE_LINKER_FLAGS "-static" CACHE STRING "Linker flags")

# 禁用一些在嵌入式平台可能有问题的特性
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
