cmake_minimum_required(VERSION 3.8.2)

# 测试可执行文件
add_executable(test_basic test_basic.cpp)
target_link_libraries(test_basic t32vio)

# 添加测试
add_test(NAME basic_test COMMAND test_basic)
