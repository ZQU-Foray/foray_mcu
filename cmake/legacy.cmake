# 迁移期过渡件：迁移源的两个构建期开关（例外 X-14）。P3b 把它们收进 profile 后删除本文件。
# P2.5 搬完 legacy/ 的源码后，legacy_include 与 legacy_library() 已删除（例外 X-11 关闭）。

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
