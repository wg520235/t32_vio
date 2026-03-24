cmake_minimum_required(VERSION 3.8.2)
project(t32vio VERSION 1.0.0 LANGUAGES CXX C)

# C++11标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 99)

# 选项
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)
option(BUILD_TESTS "Build tests" ON)
option(ENABLE_OPTIMIZATION "Enable optimization flags" ON)

# 查找Eigen3 (如果系统中有)
find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
    # 使用内置的Eigen
    message(STATUS "Using bundled Eigen3")
    set(EIGEN3_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen)
endif()

# 包含目录
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
)

# 编译选项
if(ENABLE_OPTIMIZATION)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        add_compile_options(-O3 -march=native -ffast-math)
        add_compile_options(-funroll-loops -fomit-frame-pointer)
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
        add_compile_options(/O2)
    endif()
endif()

# 警告选项
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wno-unused-parameter)
endif()

# 源文件
set(T32VIO_SOURCES
    src/t32vio.cpp
    src/frontend.cpp
    src/msckf.cpp
)

# 创建静态库
add_library(t32vio STATIC ${T32VIO_SOURCES})

target_include_directories(t32vio PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# 链接线程库
find_package(Threads REQUIRED)
target_link_libraries(t32vio PUBLIC Threads::Threads)

# 安装规则
install(TARGETS t32vio
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# 测试
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# 打印配置信息
message(STATUS "")
message(STATUS "T32 VIO Configuration:")
message(STATUS "  Version: ${PROJECT_VERSION}")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  Optimization: ${ENABLE_OPTIMIZATION}")
message(STATUS "  Eigen3: ${EIGEN3_INCLUDE_DIR}")
message(STATUS "")
