# platform_mock：宿主测试用的平台替身（评审报告 §5.2 图例）。
# 用 C++ 实现 hal_os（时基、临界区、Signal），线程安全：时间由测试推进，临界区是一把进程内的锁，
# Signal 用条件变量实现并检查误用（Decision-20261031-01～03）。
# 它是测试替身，不构成 src 对 platform 的依赖；只在宿主构建中加入，不得进入固件。
# F2 起再实现 hal 的 C++ 契约，替换 test/host/support 中迁移期的 C ABI 桩。
find_package(Threads REQUIRED)
add_library(platform_mock STATIC
    mock_os.cpp
    mock_os.hpp)
target_include_directories(platform_mock PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(platform_mock PUBLIC hal_os Threads::Threads)
enable_strict_warnings(platform_mock)
