# config：一台机器人的数据（profiles）与组合根（assembly）。只在固件构建中加入。
# CMake 文件按层组织：config/ 的 target 都定义在这里（Decision-20261026-07）。

# profile_<名称>：纯数据的生成头，叶子 target，没有任何依赖（规则 4）。
set(profile_generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated/${PROFILE_ID}")
configure_file(profiles/profile.hpp.in "${profile_generated_dir}/profile.hpp" @ONLY)
add_library(profile_${PROFILE_ID} INTERFACE)
target_include_directories(profile_${PROFILE_ID} INTERFACE "${profile_generated_dir}")

# assembly_<profile>：组合根——构造对象、建任务表、注入参数、实现启动入口（评审报告 §5.2）。
# 只有最终固件可以链接它（规则 5）。
#
# 迁移期的启动入口是 assembly/entry_legacy.cpp（Application_Init/InitRtos/Start），P3b 收敛为
# boot_init/boot_init_rtos/boot_run（src/boot/boot.h）。
set(assembly assembly_${PROFILE_ID})

add_library(${assembly} STATIC
    assembly/entry_legacy.cpp
    assembly/Application/Entry.h)
target_include_directories(${assembly} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/assembly")
enable_strict_warnings(${assembly})
target_link_libraries(${assembly} PRIVATE
    legacy_build_options
    app_common
    app_tasks
    app_remote
    hal
    boot
    profile_${PROFILE_ID}
    os_freertos_${BOARD_ID})
link_object_library(${assembly} board_${BOARD_ID})

# 生成区接缝（例外 X-09）：CubeMX 生成的 main.c、freertos.c 以 "Application/Entry.h" 引用启动入口，
# 生成区不能手改，头文件只能放在 assembly/Application/Entry.h。由组合根把 assembly/ 注入生成区 target 的
# include 路径（框架与平台的 CMake 不出现产品路径，规则 15）；P3b 生成区改为只调 src/boot 的 boot ABI 后删除。
target_include_directories(mx_generated_${BOARD_ID} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/assembly")
