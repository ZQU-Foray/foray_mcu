# 统一的 target 辅助函数（评审报告 §5.2、§5.3 规则 8）：告警、OBJECT 库链接、固件产物、烧录。
# 不使用 file(GLOB) 与全局 include_directories()；每个 target 显式声明自己的 include 与依赖。

# enable_strict_warnings(<target>)
#   手写代码统一 -Wall -Wextra -Werror（只作用于 C/C++，汇编不加）。厂商与生成代码不调用。
#   固件构建的 -Wall 已由工具链加给全部代码（见 arm-gcc-common.cmake），这里只补宿主构建。
function(enable_strict_warnings target)
    if(NOT CMAKE_CROSSCOMPILING)
        target_compile_options(${target} PRIVATE $<$<COMPILE_LANGUAGE:C,CXX>:-Wall>)
    endif()
    target_compile_options(${target} PRIVATE $<$<COMPILE_LANGUAGE:C,CXX>:-Wextra;-Werror>)
endfunction()

# link_object_library(<consumer> <OBJECT 库>...)
#   CMake 只把"直接链接"的 OBJECT 库的目标文件放进最终链接，经其他库间接依赖的不会进去。
#   这里同时做两件事：PRIVATE 链接（取得 include、宏等使用要求，依赖图里留下这条边），
#   并把目标文件放进 consumer 的链接接口，让它们随 consumer 一起进入最终固件。
#   port 与 board 必须用 OBJECT 库：覆盖 HAL 弱符号的回调放在静态库里会被链接器静默丢弃（规则说明见 §5.2 图例）。
function(link_object_library consumer)
    foreach(object_library IN LISTS ARGN)
        target_link_libraries(${consumer}
            PRIVATE ${object_library}
            INTERFACE $<TARGET_OBJECTS:${object_library}>)
    endforeach()
endfunction()

# add_firmware_outputs(<可执行文件 target>)
#   生成 map（与 ELF 同目录同名），链接后输出 .hex、.bin 并打印段大小。
function(add_firmware_outputs target)
    target_link_options(${target} PRIVATE
        "${TOOLCHAIN_MAP_FILE_OPTION}$<TARGET_FILE_DIR:${target}>/$<TARGET_FILE_BASE_NAME:${target}>.map")
    set_property(TARGET ${target} APPEND PROPERTY ADDITIONAL_CLEAN_FILES
        "$<TARGET_FILE_DIR:${target}>/$<TARGET_FILE_BASE_NAME:${target}>.map")
    add_custom_command(TARGET ${target} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex
                $<TARGET_FILE:${target}> $<TARGET_FILE_DIR:${target}>/$<TARGET_FILE_BASE_NAME:${target}>.hex
        COMMAND ${CMAKE_OBJCOPY} -O binary
                $<TARGET_FILE:${target}> $<TARGET_FILE_DIR:${target}>/$<TARGET_FILE_BASE_NAME:${target}>.bin
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${target}>
        COMMENT "生成 ${target} 的 HEX/BIN 镜像"
        VERBATIM)
endfunction()

# add_flash_targets(<可执行文件 target> <OpenOCD 配置文件>)
#   flash：烧录并校验后复位；openocd-server：启动调试服务器。找不到 openocd 时不建这两个 target。
#   OpenOCD 的位置取自 PATH，或缓存变量/环境变量 OPENOCD_ROOT（其下的 bin/）。
function(add_flash_targets target openocd_config)
    if(NOT OPENOCD_ROOT AND DEFINED ENV{OPENOCD_ROOT})
        set(OPENOCD_ROOT "$ENV{OPENOCD_ROOT}")
    endif()
    find_program(OPENOCD_EXECUTABLE NAMES openocd openocd.exe HINTS "${OPENOCD_ROOT}/bin")
    if(NOT OPENOCD_EXECUTABLE)
        message(STATUS "未找到 OpenOCD，不生成 flash / openocd-server")
        return()
    endif()
    add_custom_target(flash
        COMMAND "${OPENOCD_EXECUTABLE}" -f "${openocd_config}"
                -c "program $<TARGET_FILE:${target}> verify reset exit"
        DEPENDS ${target}
        USES_TERMINAL
        COMMENT "用 OpenOCD 烧录 ${target}"
        VERBATIM)
    add_custom_target(openocd-server
        COMMAND "${OPENOCD_EXECUTABLE}" -f "${openocd_config}"
        USES_TERMINAL
        COMMENT "启动 OpenOCD 调试服务器"
        VERBATIM)
endfunction()
