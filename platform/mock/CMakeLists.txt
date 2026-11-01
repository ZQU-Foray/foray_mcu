# platform_mock：宿主测试用的平台替身（评审报告 §5.2 图例）。
# 用 C++ 实现 hal_os（时基、临界区、Signal），线程安全：时间由测试推进，临界区是一把进程内的锁，
# Signal 用条件变量实现并检查误用（Decision-20261031-01～03）。
# 它是测试替身，不构成 src 对 platform 的依赖；只在宿主构建中加入，不得进入固件。
# 也实现 hal 契约 v1 的外设（Decision-20261101-01）：测试按编号注入输入、检查输出，接口在 mock_hal.hpp。
# 契约 v1 的全部外设：can、gpio、pwm、spi、uart、usb_cdc。
find_package(Threads REQUIRED)
add_library(platform_mock STATIC
    mock_can.cpp
    mock_gpio.cpp
    mock_hal.hpp
    mock_os.cpp
    mock_os.hpp
    mock_pwm.cpp
    mock_spi.cpp
    mock_uart.cpp
    mock_usb_cdc.cpp)
target_include_directories(platform_mock PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(platform_mock PUBLIC hal hal_os Threads::Threads)
enable_strict_warnings(platform_mock)
