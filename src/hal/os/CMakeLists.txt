# hal_os：最小 OS 契约，只有 RAII 临界区与单调时基（评审报告 §5.7、Q10）。
# 线程、队列、事件不进框架。实现：固件在 port（P3a），宿主测试在 platform/mock。
# P1 是草案，P3a 冻结契约 v1 时可以调整。
add_library(hal_os INTERFACE
    clock.hpp
    critical_section.hpp)
target_include_directories(hal_os INTERFACE "${PROJECT_SOURCE_DIR}/src")
target_compile_features(hal_os INTERFACE cxx_std_17)
