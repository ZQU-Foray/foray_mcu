# config：一台机器人的数据（profiles）与组合根（assembly）。只在固件构建中加入。
# CMake 文件按层组织：config/ 的 target 都定义在这里（Decision-20261026-07）。

# profile_<名称>：纯数据的生成头，叶子 target，没有任何依赖（规则 4）。
set(profile_generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated/${PROFILE_ID}")
configure_file(profiles/profile.hpp.in "${profile_generated_dir}/profile.hpp" @ONLY)
add_library(profile_${PROFILE_ID} INTERFACE)
# params.hpp（只含数据，规则 4）在 profile 目录里；它 include 的能力配置类型头由组合根提供 include 路径。
target_include_directories(profile_${PROFILE_ID} INTERFACE
    "${profile_generated_dir}"
    "${CMAKE_CURRENT_SOURCE_DIR}/profiles/${PROFILE_ID}")

# assembly_<profile>：组合根——构造对象、建任务表、注入参数、实现启动入口（评审报告 §5.2）。
# 只有最终固件可以链接它（规则 5）。
#
# 每个 profile 一个组合根 assembly/<profile>.cpp，链接 profile.cmake 的 CAPABILITIES（Decision-20261031-12）。
# 迁移期例外：baseline 的组合根是 assembly/entry_legacy.cpp（迁移源的装配，Decision-20261031-11），
# F2 第 4 步随旧 app 一起归档。
set(assembly assembly_${PROFILE_ID})

if(PROFILE_ID STREQUAL "baseline")
    add_library(${assembly} STATIC
        assembly/entry_legacy.cpp)
    target_link_libraries(${assembly} PRIVATE
        legacy_build_options
        app_common
        app_tasks
        app_remote
        hal)
else()
    add_library(${assembly} STATIC
        "assembly/${PROFILE_ID}.cpp")
    target_link_libraries(${assembly} PRIVATE app_common hal hal_os)
    foreach(capability IN LISTS CAPABILITIES)
        target_link_libraries(${assembly} PRIVATE app_${capability})
    endforeach()
endif()
enable_strict_warnings(${assembly})
target_link_libraries(${assembly} PRIVATE
    boot
    profile_${PROFILE_ID}
    os_freertos_${BOARD_ID})
link_object_library(${assembly} board_${BOARD_ID})
