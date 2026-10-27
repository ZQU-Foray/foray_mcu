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
# 迁移期为 legacy/Application/Entry.cpp（Application_Init/InitRtos/Start，CubeMX 生成的 main.c 与
# freertos.c 调用它们，例外 X-09）；P2.5 移到 config/assembly/，P3b 收敛为 boot_init/boot_init_rtos/boot_run。
set(assembly assembly_${PROFILE_ID})

legacy_library(${assembly}
    "${LEGACY_DIR}/Application/Entry.cpp"
    "${LEGACY_DIR}/Application/Entry.h")
target_link_libraries(${assembly} PRIVATE
    app_common
    app_tasks
    app_remote
    hal
    boot
    profile_${PROFILE_ID}
    os_freertos_${BOARD_ID})
link_object_library(${assembly} board_${BOARD_ID})
