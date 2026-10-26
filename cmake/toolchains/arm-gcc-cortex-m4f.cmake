# Cortex-M4F（STM32F407 C 板、TM4C；只有单精度 FPU，double 走软件库，见评审报告 §5.8、§5.10）。
# P3a 的 F407 bring-up 首次使用；P1 只提供工具链，没有使用它的 profile。
set(TOOLCHAIN_CPU cortex-m4f)
set(TOOLCHAIN_CPU_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb")
include("${CMAKE_CURRENT_LIST_DIR}/arm-gcc-common.cmake")
