# 示例程序CMake配置

# 演示程序
add_executable(t32_vio_demo t32_vio_demo.cpp)
target_link_libraries(t32_vio_demo t32vio pthread)

# 安装示例
install(TARGETS t32_vio_demo
    RUNTIME DESTINATION bin/examples
)

install(FILES 
    t32_vio_demo.cpp
    INTEGRATION_GUIDE.md
    DESTINATION share/t32vio/examples
)
