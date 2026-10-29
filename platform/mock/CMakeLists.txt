# platform_mock：宿主测试用的平台替身（评审报告 §5.2 图例）。
# 用 C++ 实现 hal_os（时基与临界区），时间与临界区状态都可由测试直接控制。
# 它是测试替身，不构成 src 对 platform 的依赖；只在宿主构建中加入，不得进入固件。
# P3a 起再实现 hal 的 C++ 契约，替换 test/host/support 中迁移期的 C ABI 桩。
add_library(platform_mock STATIC
    mock_os.cpp
    mock_os.hpp)
target_include_directories(platform_mock PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(platform_mock PUBLIC hal_os)
enable_strict_warnings(platform_mock)
