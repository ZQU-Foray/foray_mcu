# 迁移期过渡件（P1 建立）：仍在 legacy/ 原地编译的源码按迁移源根路径 include（如 "Application/Imu/..."）。
# P2 之后只剩 P2.5 的文件（Application/{Entry,Imu,Task/TaskManager}、PortKit、Port/UsbCdc）。
# 到期删除（docs/ai/architecture.md 例外 X-11、X-14）：
#   - legacy_include、legacy_library()：P2.5 搬完剩余文件后删除（原计划 P2，因 P2.5 的文件仍按旧路径被引用而推迟）；
#   - legacy_build_options：P3b 把两个构建期开关收进 profile 后删除。

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
