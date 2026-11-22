# 宿主下位机整链测试（I26）：真实 PTY + platform/host_sim + app/pc_link。
# 只在 -DHOST_SIM=ON 的宿主构建里加入；与 platform/mock 互斥。
find_package(Threads REQUIRED)

add_executable(test_host_sim_link host_sim_link.cpp)
target_link_libraries(test_host_sim_link PRIVATE app_pc_link platform_host_sim Threads::Threads)
enable_strict_warnings(test_host_sim_link)
add_test(NAME host_sim_link COMMAND test_host_sim_link)
