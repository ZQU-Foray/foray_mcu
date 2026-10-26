# utils：零平台工具与并发原语（计数器、最新值邮箱、SPSC 队列；M0+ 上退回临界区实现）。
# P1 只建立 target；内容在 P3a 由 PortKit 的队列/通知与现有序号快照写法通用化而来（评审报告 §5.7、§5.8）。
add_library(utils INTERFACE)
target_include_directories(utils INTERFACE "${PROJECT_SOURCE_DIR}/src")
target_compile_features(utils INTERFACE cxx_std_17)
