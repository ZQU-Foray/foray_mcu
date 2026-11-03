# DM-MC02 板的平台层（评审报告 §5.2）：
#   mx_generated_<板>  CubeMX 生成物（Core、USB_DEVICE、HAL 驱动、启动文件）  OBJECT
#   rtos_<板>          FreeRTOS 与 CMSIS-RTOS2 适配                            OBJECT
#   usb_<板>           ST USB Device 库（CDC）                                  OBJECT
#   port_<板>          hal 与 hal/os 契约的 C++ 实现、HAL 回调、USB 生成区接缝   OBJECT
#   board_<板>         board.hpp（句柄与能力常量）与链接脚本                       OBJECT
#   os_freertos_<板>   RTOS 头文件，只给 app_tasks、app_common 与 assembly       INTERFACE
#
# port 与 board 必须是 OBJECT 库（覆盖 HAL 弱符号的回调在静态库里会被静默丢弃）。
# 生成区（cubemx/）冻结，只能重新生成；cubemx/cmake/stm32cubemx/CMakeLists.txt 是 CubeMX 生成的，根工程不用，
# 重新生成后可对照它更新下面的源文件清单。
#
# 编译选项与迁移源一致（P1 的符号对照依赖这一点）：
#   - 宏：CubeMX 的宏给全部平台代码；ARM_MATH_CM7 给生成的应用代码（迁移源如此，实际未使用）；
#   - 手写的 port 带严格告警（-Wextra -Werror），厂商与生成代码不带。
set(cubemx_dir "${CMAKE_CURRENT_SOURCE_DIR}/cubemx")
set(port_dir "${CMAKE_CURRENT_SOURCE_DIR}/port")

# CubeMX 生成的宏与 include 路径（迁移源 cmake/stm32cubemx/CMakeLists.txt 的 MX_Defines_Syms 与 MX_Include_Dirs）。
set(cubemx_definitions
    USE_PWR_LDO_SUPPLY
    USE_HAL_DRIVER
    STM32H723xx
    $<$<CONFIG:Debug>:DEBUG>)
# FreeRTOS 的配置里 CubeMX 没有、又必须设置的项，以编译宏的形式统一给出：
#   configTASK_NOTIFICATION_ARRAY_ENTRIES=2：hal::os::Signal 用 1 号任务通知槽。FreeRTOS.h 在没有定义时取 1，
#   FreeRTOSConfig.h（生成区）不用改。它决定任务控制块与 StaticTask_t 的大小，凡是 include FreeRTOS 头的编译单元
#   都必须一致，所以同时放进 cubemx_definitions（生成代码、内核、USB、port）与 os_freertos 的接口。
set(rtos_definitions configTASK_NOTIFICATION_ARRAY_ENTRIES=2)
list(APPEND cubemx_definitions ${rtos_definitions})
set(cubemx_include_dirs
    "${cubemx_dir}/Core/Inc"
    "${cubemx_dir}/USB_DEVICE/App"
    "${cubemx_dir}/USB_DEVICE/Target"
    "${cubemx_dir}/Drivers/STM32H7xx_HAL_Driver/Inc"
    "${cubemx_dir}/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy"
    "${cubemx_dir}/Middlewares/Third_Party/FreeRTOS/Source/include"
    "${cubemx_dir}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2"
    "${cubemx_dir}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F"
    "${cubemx_dir}/Drivers/CMSIS/RTOS2/Include"
    "${cubemx_dir}/Middlewares/ST/STM32_USB_Device_Library/Core/Inc"
    "${cubemx_dir}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc"
    "${cubemx_dir}/Drivers/CMSIS/Device/ST/STM32H7xx/Include"
    "${cubemx_dir}/Drivers/CMSIS/Include"
    "${cubemx_dir}/Middlewares/ST/ARM/DSP/Inc")

# 再生成防护（沿用迁移源）：CubeMX 重新生成后这两处配置不得丢失。
file(READ "${cubemx_dir}/USB_DEVICE/Target/usbd_conf.c" usbd_conf_source)
if(NOT usbd_conf_source MATCHES "hpcd_USB_OTG_HS\\.Init\\.dma_enable[ \t]*=[ \t]*ENABLE")
    message(FATAL_ERROR "CubeMX 重新生成后必须保持 USB OTG HS 内部 DMA 开启")
endif()
file(READ "${port_dir}/usb_cdc.cpp" usb_cdc_port_source)
if(NOT usb_cdc_port_source MATCHES "gpio_init\\.Alternate[ \t]*=[ \t]*GPIO_AF10_OTG1_FS")
    message(FATAL_ERROR "UsbCdc port 缺少 USB OTG HS PA11/PA12 的复用功能配置")
endif()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${cubemx_dir}/USB_DEVICE/Target/usbd_conf.c"
    "${port_dir}/usb_cdc.cpp")

# ---- mx_generated：CubeMX 生成的应用代码 + HAL 驱动 + 启动文件 ----
set(cubemx_application_sources
    "${cubemx_dir}/USB_DEVICE/Target/usbd_conf.c"
    "${cubemx_dir}/USB_DEVICE/App/usb_device.c"
    "${cubemx_dir}/USB_DEVICE/App/usbd_desc.c"
    "${cubemx_dir}/USB_DEVICE/App/usbd_cdc_if.c"
    "${cubemx_dir}/Core/Src/main.c"
    "${cubemx_dir}/Core/Src/gpio.c"
    "${cubemx_dir}/Core/Src/freertos.c"
    "${cubemx_dir}/Core/Src/adc.c"
    "${cubemx_dir}/Core/Src/bdma.c"
    "${cubemx_dir}/Core/Src/dma.c"
    "${cubemx_dir}/Core/Src/fdcan.c"
    "${cubemx_dir}/Core/Src/mdma.c"
    "${cubemx_dir}/Core/Src/octospi.c"
    "${cubemx_dir}/Core/Src/spi.c"
    "${cubemx_dir}/Core/Src/tim.c"
    "${cubemx_dir}/Core/Src/usart.c"
    "${cubemx_dir}/Core/Src/stm32h7xx_it.c"
    "${cubemx_dir}/Core/Src/stm32h7xx_hal_msp.c"
    "${cubemx_dir}/Core/Src/stm32h7xx_hal_timebase_tim.c"
    "${cubemx_dir}/Core/Src/sysmem.c"
    "${cubemx_dir}/Core/Src/syscalls.c")
set(hal_driver_dir "${cubemx_dir}/Drivers/STM32H7xx_HAL_Driver/Src")
set(cubemx_driver_sources
    "${cubemx_dir}/Core/Src/system_stm32h7xx.c"
    "${hal_driver_dir}/stm32h7xx_hal_tim.c"
    "${hal_driver_dir}/stm32h7xx_hal_tim_ex.c"
    "${hal_driver_dir}/stm32h7xx_hal_pcd.c"
    "${hal_driver_dir}/stm32h7xx_hal_pcd_ex.c"
    "${hal_driver_dir}/stm32h7xx_ll_usb.c"
    "${hal_driver_dir}/stm32h7xx_hal_rcc.c"
    "${hal_driver_dir}/stm32h7xx_hal_rcc_ex.c"
    "${hal_driver_dir}/stm32h7xx_hal_flash.c"
    "${hal_driver_dir}/stm32h7xx_hal_flash_ex.c"
    "${hal_driver_dir}/stm32h7xx_hal_gpio.c"
    "${hal_driver_dir}/stm32h7xx_hal_hsem.c"
    "${hal_driver_dir}/stm32h7xx_hal_dma.c"
    "${hal_driver_dir}/stm32h7xx_hal_dma_ex.c"
    "${hal_driver_dir}/stm32h7xx_hal_mdma.c"
    "${hal_driver_dir}/stm32h7xx_hal_pwr.c"
    "${hal_driver_dir}/stm32h7xx_hal_pwr_ex.c"
    "${hal_driver_dir}/stm32h7xx_hal_cortex.c"
    "${hal_driver_dir}/stm32h7xx_hal.c"
    "${hal_driver_dir}/stm32h7xx_hal_i2c.c"
    "${hal_driver_dir}/stm32h7xx_hal_i2c_ex.c"
    "${hal_driver_dir}/stm32h7xx_hal_exti.c"
    "${hal_driver_dir}/stm32h7xx_hal_adc.c"
    "${hal_driver_dir}/stm32h7xx_hal_adc_ex.c"
    "${hal_driver_dir}/stm32h7xx_hal_fdcan.c"
    "${hal_driver_dir}/stm32h7xx_hal_ospi.c"
    "${hal_driver_dir}/stm32h7xx_hal_spi.c"
    "${hal_driver_dir}/stm32h7xx_hal_spi_ex.c"
    "${hal_driver_dir}/stm32h7xx_hal_uart.c"
    "${hal_driver_dir}/stm32h7xx_hal_uart_ex.c")

add_library(mx_generated_${BOARD_ID} OBJECT
    ${cubemx_application_sources}
    ${cubemx_driver_sources}
    "${cubemx_dir}/startup_stm32h723xx.s")
target_compile_definitions(mx_generated_${BOARD_ID}
    PUBLIC ${cubemx_definitions}
    PRIVATE ARM_MATH_CM7)
target_include_directories(mx_generated_${BOARD_ID}
    PUBLIC ${cubemx_include_dirs}
    # USB 生成桩（Q8 方案①）：USB_DEVICE 调用 port 私有的 UsbCdcPort_*（UsbCdc.h），
    # 这个头被生成代码按名字 include，所以保持原文件名。
    PRIVATE "${port_dir}")
target_link_libraries(mx_generated_${BOARD_ID}
    # main.c、freertos.c 的 USER CODE 区只调 boot_*（src/boot/boot.h）
    PRIVATE boot
    # 迁移源链接的库：libm 与预编译的 CMSIS-DSP（顺序与迁移源一致）。
    INTERFACE m "${cubemx_dir}/Middlewares/ST/ARM/DSP/Lib/libarm_cortexM7lfsp_math.a")

# ---- rtos：FreeRTOS 内核、CMSIS-RTOS2 适配、heap_4、Cortex-M4F/M7 移植层 ----
set(freertos_dir "${cubemx_dir}/Middlewares/Third_Party/FreeRTOS/Source")
add_library(rtos_${BOARD_ID} OBJECT
    "${freertos_dir}/portable/Common/mpu_wrappers_v2.c"
    "${freertos_dir}/croutine.c"
    "${freertos_dir}/event_groups.c"
    "${freertos_dir}/list.c"
    "${freertos_dir}/queue.c"
    "${freertos_dir}/stream_buffer.c"
    "${freertos_dir}/tasks.c"
    "${freertos_dir}/timers.c"
    "${freertos_dir}/CMSIS_RTOS_V2/cmsis_os2.c"
    "${freertos_dir}/portable/MemMang/heap_4.c"
    "${freertos_dir}/portable/GCC/ARM_CM4F/port.c")
target_compile_definitions(rtos_${BOARD_ID} PRIVATE ${cubemx_definitions})
target_include_directories(rtos_${BOARD_ID} PRIVATE ${cubemx_include_dirs})

# ---- usb：ST USB Device 库 ----
set(usb_library_dir "${cubemx_dir}/Middlewares/ST/STM32_USB_Device_Library")
add_library(usb_${BOARD_ID} OBJECT
    "${usb_library_dir}/Core/Src/usbd_core.c"
    "${usb_library_dir}/Core/Src/usbd_ctlreq.c"
    "${usb_library_dir}/Core/Src/usbd_ioreq.c"
    "${usb_library_dir}/Class/CDC/Src/usbd_cdc.c")
target_compile_definitions(usb_${BOARD_ID} PRIVATE ${cubemx_definitions})
target_include_directories(usb_${BOARD_ID} PRIVATE ${cubemx_include_dirs})

# ---- os_freertos：只暴露 RTOS 头文件（迁移期含 CMSIS-RTOS2，终态改 FreeRTOS 原生 API，规则 10） ----
# FreeRTOSConfig.h 由 CubeMX 生成在 Core/Inc，只能整目录暴露；Core/Inc 里的其他生成头因此对任务层可见（迁移期限制）。
add_library(os_freertos_${BOARD_ID} INTERFACE)
target_compile_definitions(os_freertos_${BOARD_ID} INTERFACE ${rtos_definitions})
target_include_directories(os_freertos_${BOARD_ID} INTERFACE
    "${cubemx_dir}/Core/Inc"
    "${freertos_dir}/include"
    "${freertos_dir}/CMSIS_RTOS_V2"
    "${freertos_dir}/portable/GCC/ARM_CM4F"
    "${cubemx_dir}/Drivers/CMSIS/RTOS2/Include")

# ---- port：hal 与 hal/os 契约 v1 的 C++ 实现 ----
# 每个外设一个 .cpp，覆盖 HAL 弱符号的回调写在对应的 .cpp 里（规则 16）；port.hpp、os/os.hpp 是 port 私有的头。
# UsbCdc.h 是 USB 生成桩调用的 C 接口（Q8 方案①），被生成代码按名字 include，保持原文件名。
add_library(port_${BOARD_ID} OBJECT
    port/can.cpp
    port/gpio.cpp
    port/hal_init.cpp
    port/os/clock.cpp
    port/os/critical_section.cpp
    port/os/os.hpp
    port/os/signal.cpp
    port/port.hpp
    port/pwm.cpp
    port/spi.cpp
    port/uart.cpp
    port/usb_cdc.cpp
    port/UsbCdc.h)
enable_strict_warnings(port_${BOARD_ID})
# board：board.hpp（句柄编号、外设个数），port 用它核对自己的外设表；utils_concurrency：接收队列。
target_link_libraries(port_${BOARD_ID} PRIVATE hal hal_os utils_concurrency board_${BOARD_ID})
link_object_library(port_${BOARD_ID}
    mx_generated_${BOARD_ID}
    rtos_${BOARD_ID}
    usb_${BOARD_ID})
# USB 发送的链接接缝（例外 X-02）：OTG HS 的 DMA 访问不到 DTCM，发送经 port 包装，按需拷进弹跳缓冲区。
# USB 设备句柄等也放进 DMA 可达内存（随 X-03）后再删除。
target_link_options(port_${BOARD_ID} INTERFACE "-Wl,--wrap=HAL_PCD_EP_Transmit")

# ---- board：板级内容——board.hpp（物理资源句柄、外设个数与能力常量）与链接脚本
#      （CubeMX 生成，冻结）；链接脚本 overlay 在第一个自定义段出现时加入（Q9） ----
# board.hpp 只给组合根与本板的 port：include 路径只开到 board/，看不到 port/（规则 1）。
set(linker_script "${cubemx_dir}/STM32H723xG_flash.ld")
add_library(board_${BOARD_ID} OBJECT "${linker_script}" board/board.hpp)
set_target_properties(board_${BOARD_ID} PROPERTIES LINKER_LANGUAGE C)
target_include_directories(board_${BOARD_ID} INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/board")
target_link_libraries(board_${BOARD_ID} INTERFACE hal)
target_link_options(board_${BOARD_ID} INTERFACE "-T${linker_script}")
set_property(TARGET board_${BOARD_ID} APPEND PROPERTY INTERFACE_LINK_DEPENDS "${linker_script}")
