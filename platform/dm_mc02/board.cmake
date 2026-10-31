# 板卡描述（纯数据），在根 CMakeLists.txt 中先于各层加载：其他层的 target 名要用 BOARD_ID。
# DM-MC02：STM32H723VGT6，Cortex-M7 480 MHz。

# 板标识：用作 target 名后缀（port_dm_mc02、os_freertos_dm_mc02 等）。
set(BOARD_ID dm_mc02)

# 必须与工具链一致（cmake/toolchains/arm-gcc-cortex-m7.cmake 的 TOOLCHAIN_CPU）。
set(BOARD_CPU cortex-m7)

# 烧录与调试：CMSIS-DAP 调试器（如 Vllink Basic2），SWD。
set(BOARD_OPENOCD_CONFIG "${PROJECT_SOURCE_DIR}/tool/flash/openocd/dm_mc02_cmsis_dap.cfg")
