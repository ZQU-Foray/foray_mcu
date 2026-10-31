# Cortex-M0+（MSPM0 等；没有 FPU，没有 LDREX/STREX，原子读改写只能编译成库调用，架构 §8.2）。
# 现在只用于框架独立构建的 framework-m0plus preset：检查框架能为 M0+ 编译、不出现原子读改写的库调用（规则 13）。
set(TOOLCHAIN_CPU cortex-m0plus)
set(TOOLCHAIN_CPU_FLAGS "-mcpu=cortex-m0plus -mfloat-abi=soft -mthumb")
include("${CMAKE_CURRENT_LIST_DIR}/arm-gcc-common.cmake")
