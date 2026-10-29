# Cortex-M7（STM32H7，硬件单/双精度 FPU fpv5-d16）。板卡在 board.cmake 中声明 BOARD_CPU，配置时与 TOOLCHAIN_CPU 核对。
set(TOOLCHAIN_CPU cortex-m7)
set(TOOLCHAIN_CPU_FLAGS "-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb")
include("${CMAKE_CURRENT_LIST_DIR}/arm-gcc-common.cmake")
