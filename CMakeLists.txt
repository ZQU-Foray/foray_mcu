# foray_mcu 构建入口：顶层只做工具链检查、选项与组合（评审报告 §5.2、§5.4）。
#
# 两种构建，用 CMakePresets.json 选择：
#   固件  cmake --preset arm-debug | arm-release    交叉编译，唯一的开关是 -DPROFILE=<名称>，
#                                                   板卡、能力、参数都来自 config/profiles/<名称>/profile.cmake
#   宿主  cmake --preset host-tests                 框架 + platform/mock + 宿主测试（ctest）
#
# 迁移期（P1–P2.5）源码仍在 legacy/ 原地，由各层的 target 引用（cmake/legacy.cmake）。
# legacy/Platform/STM32H7/DM_MC02 下迁移源自带的构建保留到 P2，用于基线 SHA 对照。
cmake_minimum_required(VERSION 3.22)
project(foray_mcu LANGUAGES C CXX)

# 语言标准与迁移源一致：C11（含 GNU 扩展，即 gnu11），C++17（不含扩展）。
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
include(targets)
include(legacy)

if(CMAKE_CROSSCOMPILING)
    enable_language(ASM)

    # ---- profile 与板卡 ----
    set(PROFILE "" CACHE STRING "要构建的 profile（config/profiles/ 下的目录名）")
    if(NOT PROFILE)
        message(FATAL_ERROR "固件构建需要 -DPROFILE=<名称>（config/profiles/ 下的目录名），或使用 arm-debug / arm-release preset")
    endif()
    set(profile_file "${PROJECT_SOURCE_DIR}/config/profiles/${PROFILE}/profile.cmake")
    if(NOT EXISTS "${profile_file}")
        message(FATAL_ERROR "找不到 profile：${profile_file}")
    endif()
    include("${profile_file}")
    if(NOT PROFILE_ID STREQUAL PROFILE)
        message(FATAL_ERROR "${profile_file} 中的 PROFILE_ID（${PROFILE_ID}）必须与目录名 ${PROFILE} 一致")
    endif()

    set(board_root "${PROJECT_SOURCE_DIR}/platform/${BOARD}")
    if(NOT EXISTS "${board_root}/board.cmake")
        message(FATAL_ERROR "profile ${PROFILE} 指定的板卡 ${BOARD} 不存在（缺少 ${board_root}/board.cmake）")
    endif()
    include("${board_root}/board.cmake")
    if(NOT BOARD_CPU STREQUAL TOOLCHAIN_CPU)
        message(FATAL_ERROR "板卡 ${BOARD} 需要 ${BOARD_CPU} 工具链，当前工具链是 ${TOOLCHAIN_CPU}")
    endif()
    message(STATUS "profile=${PROFILE} board=${BOARD}（${BOARD_ID}） cpu=${BOARD_CPU}")

    # ---- 各层 ----
    add_subdirectory(src)
    add_subdirectory("platform/${BOARD}" "platform/${BOARD_ID}")
    add_subdirectory(app)
    add_subdirectory(config)

    # ---- 固件：只链接组合根与 port，其余经它们传递（评审报告 §5.2） ----
    set(firmware firmware_${PROFILE_ID})
    add_executable(${firmware})
    target_link_libraries(${firmware} PRIVATE assembly_${PROFILE_ID} port_${BOARD_ID})
    add_firmware_outputs(${firmware})
    add_flash_targets(${firmware} "${BOARD_OPENOCD_CONFIG}")
else()
    if(NOT CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        message(FATAL_ERROR "宿主构建需要 GCC 或 Clang（当前为 ${CMAKE_CXX_COMPILER_ID}）")
    endif()

    # 框架 + 平台替身 + 宿主测试；不含 app/、config/ 与任何板卡。
    add_subdirectory(src)
    add_subdirectory(platform/mock)
    enable_testing()
    add_subdirectory(test/host)
endif()
