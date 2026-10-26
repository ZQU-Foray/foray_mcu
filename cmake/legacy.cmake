# 迁移期过渡件（P1 建立）：源码仍在 legacy/ 原地编译，按迁移源根路径 include（如 "Libraries/..."）。
# 只在搬迁完成前存在，到期删除（docs/ai/architecture.md 例外 X-11、X-14）：
#   - legacy_include：P2 结束时删除（源码搬出、include 改写之后）；
#   - legacy_build_options：P3b 把两个构建期开关收进 profile 后删除；
#   - legacy_library()：P2 搬迁时改为各目录自己的 add_library()。

set(LEGACY_DIR "${PROJECT_SOURCE_DIR}/legacy")

# 暴露 legacy/ 作为 include 根：零源码改动下无法去掉迁移源的 include 写法（评审报告 §7.2 P1）。
add_library(legacy_include INTERFACE)
target_include_directories(legacy_include INTERFACE "${LEGACY_DIR}")

# 迁移源的两个构建期开关（评审报告 §2.4、§5.4）。固件构建中所有业务代码都带上，与迁移源一致；
# 宿主测试不带（与 P0 的测试工程一致，测试需要时在源码里自行定义）。
add_library(legacy_build_options INTERFACE)
if(CMAKE_CROSSCOMPILING)
    set(IMU_GYRO_BANDWIDTH "532" CACHE STRING "BMI088 陀螺带宽：532 或 116 Hz（只改传感器寄存器）")
    set_property(CACHE IMU_GYRO_BANDWIDTH PROPERTY STRINGS 532 116)
    if(IMU_GYRO_BANDWIDTH STREQUAL "532")
        set(bandwidth_register 0x00U)
    elseif(IMU_GYRO_BANDWIDTH STREQUAL "116")
        set(bandwidth_register 0x02U)
    else()
        message(FATAL_ERROR "IMU_GYRO_BANDWIDTH 只能是 532 或 116")
    endif()
    target_compile_definitions(legacy_build_options INTERFACE
        BMI088_GYRO_BANDWIDTH_REGISTER=${bandwidth_register}
        $<$<CONFIG:Debug>:IMU_ENABLE_DIAGNOSTICS>)
endif()

# legacy_library(<名称> <源文件与头文件>...)
#   迁移期的手写业务库（STATIC）：带严格告警，公开 legacy/ include 根，私有带构建期开关。
#   头文件也列出来，表明它属于哪个 target（P2 按这份归属搬迁）。
function(legacy_library name)
    add_library(${name} STATIC ${ARGN})
    target_link_libraries(${name} PUBLIC legacy_include PRIVATE legacy_build_options)
    enable_strict_warnings(${name})
endfunction()

# legacy_interface(<名称> <头文件>...)：只有头文件的业务库。
function(legacy_interface name)
    add_library(${name} INTERFACE ${ARGN})
    target_link_libraries(${name} INTERFACE legacy_include)
endfunction()
