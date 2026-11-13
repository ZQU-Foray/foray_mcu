# platform_host_sim：宿主下位机后端（I26）。
#
# 用宿主机顶替 DM-MC02：hal::os 是真实的时钟与线程同步，hal::UsbCdc 是 PTY 字符设备。
# 与 platform/mock 的关系：两者都实现 hal 契约，但**不能链进同一个可执行目标**（符号重复）；
# mock 给单元测试用（可推进时间、可注入输入），host_sim 给整链联调用（真实时间、真实字节流）。
# 只在宿主构建、且打开 -DHOST_SIM=ON 时加入；不进固件。

find_package(Threads REQUIRED)

add_library(platform_host_sim STATIC
    host_os.cpp
    host_usb_cdc.cpp)
target_include_directories(platform_host_sim PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(platform_host_sim PUBLIC hal hal_os Threads::Threads)
enable_strict_warnings(platform_host_sim)

# 宿主下位机的可执行目标在根 CMakeLists.txt：它要链 app_pc_link（产品），
# 框架目录不能引用产品（规则 15），组合点与固件目标一样放在根。
