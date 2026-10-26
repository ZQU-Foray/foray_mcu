# DM-MC02 板的平台层（评审报告 §5.2）：
#   mx_generated_<板>  CubeMX 生成物（Core、USB_DEVICE、HAL 驱动、启动文件）  OBJECT
#   rtos_<板>          FreeRTOS 与 CMSIS-RTOS2 适配                            OBJECT
#   usb_<板>           ST USB Device 库（CDC）                                  OBJECT
#   portkit            迁移期：队列/通知/临界区（P2.5 拆到 utils、hal/os 与 port） OBJECT
#   port_<板>          契约的手写实现（迁移期为 C，P3a 改写为 C++）              OBJECT
#   board_<板>         链接脚本等板级内容（P1 只有链接脚本）                      OBJECT
#   os_freertos_<板>   RTOS 头文件，只给 app_tasks、app_common 与 assembly       INTERFACE
#
# port 与 board 必须是 OBJECT 库（覆盖 HAL 弱符号的回调在静态库里会被静默丢弃）。
# 生成区（cubemx）冻结，只能重新生成；迁移期源码在 legacy/Platform/STM32H7/DM_MC02，P2 拆到 cubemx/、port/、board/。
#
# 编译选项与迁移源一致（P1 的符号对照依赖这一点，Decision-20261026-07）：
#   - 宏：CubeMX 的宏给全部平台代码；ARM_MATH_CM7 给生成的应用代码（迁移源如此，实际未使用）；
#   - 手写的 port 带严格告警（-Wextra -Werror），厂商与生成代码不带。
set(board_dir "${LEGACY_DIR}/Platform/STM32H7/DM_MC02")

# CubeMX 生成的宏与 include 路径（迁移源 cmake/stm32cubemx/CMakeLists.txt 的 MX_Defines_Syms 与 MX_Include_Dirs）。
set(cubemx_definitions
    USE_PWR_LDO_SUPPLY
    USE_HAL_DRIVER
    STM32H723xx
    $<$<CONFIG:Debug>:DEBUG>)
set(cubemx_include_dirs
    "${board_dir}/Core/Inc"
    "${board_dir}/USB_DEVICE/App"
    "${board_dir}/USB_DEVICE/Target"
    "${board_dir}/Drivers/STM32H7xx_HAL_Driver/Inc"
    "${board_dir}/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy"
    "${board_dir}/Middlewares/Third_Party/FreeRTOS/Source/include"
    "${board_dir}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2"
    "${board_dir}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F"
    "${board_dir}/Drivers/CMSIS/RTOS2/Include"
    "${board_dir}/Middlewares/ST/STM32_USB_Device_Library/Core/Inc"
    "${board_dir}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc"
    "${board_dir}/Drivers/CMSIS/Device/ST/STM32H7xx/Include"
    "${board_dir}/Drivers/CMSIS/Include"
    "${board_dir}/Middlewares/ST/ARM/DSP/Inc")

# 再生成防护（沿用迁移源）：CubeMX 重新生成后这两处配置不得丢失。
file(READ "${board_dir}/USB_DEVICE/Target/usbd_conf.c" usbd_conf_source)
if(NOT usbd_conf_source MATCHES "hpcd_USB_OTG_HS\\.Init\\.dma_enable[ \t]*=[ \t]*ENABLE")
    message(FATAL_ERROR "CubeMX 重新生成后必须保持 USB OTG HS 内部 DMA 开启")
endif()
file(READ "${board_dir}/Port/UsbCdc.c" usb_cdc_port_source)
if(NOT usb_cdc_port_source MATCHES "gpio_init\\.Alternate[ \t]*=[ \t]*GPIO_AF10_OTG1_FS")
    message(FATAL_ERROR "UsbCdc port 缺少 USB OTG HS PA11/PA12 的复用功能配置")
endif()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${board_dir}/USB_DEVICE/Target/usbd_conf.c"
    "${board_dir}/Port/UsbCdc.c")

# ---- mx_generated：CubeMX 生成的应用代码 + HAL 驱动 + 启动文件 ----
set(cubemx_application_sources
    "${board_dir}/USB_DEVICE/Target/usbd_conf.c"
    "${board_dir}/USB_DEVICE/App/usb_device.c"
    "${board_dir}/USB_DEVICE/App/usbd_desc.c"
    "${board_dir}/USB_DEVICE/App/usbd_cdc_if.c"
    "${board_dir}/Core/Src/main.c"
    "${board_dir}/Core/Src/gpio.c"
    "${board_dir}/Core/Src/freertos.c"
    "${board_dir}/Core/Src/adc.c"
    "${board_dir}/Core/Src/bdma.c"
    "${board_dir}/Core/Src/dma.c"
    "${board_dir}/Core/Src/fdcan.c"
    "${board_dir}/Core/Src/mdma.c"
    "${board_dir}/Core/Src/octospi.c"
    "${board_dir}/Core/Src/spi.c"
    "${board_dir}/Core/Src/tim.c"
    "${board_dir}/Core/Src/usart.c"
    "${board_dir}/Core/Src/stm32h7xx_it.c"
    "${board_dir}/Core/Src/stm32h7xx_hal_msp.c"
    "${board_dir}/Core/Src/stm32h7xx_hal_timebase_tim.c"
    "${board_dir}/Core/Src/sysmem.c"
    "${board_dir}/Core/Src/syscalls.c")
set(hal_driver_dir "${board_dir}/Drivers/STM32H7xx_HAL_Driver/Src")
set(cubemx_driver_sources
    "${board_dir}/Core/Src/system_stm32h7xx.c"
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
    "${board_dir}/startup_stm32h723xx.s")
target_compile_definitions(mx_generated_${BOARD_ID}
    PUBLIC ${cubemx_definitions}
    PRIVATE ARM_MATH_CM7)
target_include_directories(mx_generated_${BOARD_ID}
    PUBLIC ${cubemx_include_dirs}
    # 生成区接缝（例外 X-01、X-09）：main.c 调用 port 的 Port_Init（Port/Init.h），
    # USB_DEVICE 调用 port 私有的 UsbCdcPort_*（Port/UsbCdc.h），main.c/freertos.c 调用 Application_*。
    PRIVATE "${board_dir}/Port")
target_link_libraries(mx_generated_${BOARD_ID}
    PRIVATE hal_detail legacy_include
    # 迁移源链接的库：libm 与预编译的 CMSIS-DSP（顺序与迁移源一致）。
    INTERFACE m "${board_dir}/Middlewares/ST/ARM/DSP/Lib/libarm_cortexM7lfsp_math.a")

# ---- rtos：FreeRTOS 内核、CMSIS-RTOS2 适配、heap_4、Cortex-M4F/M7 移植层 ----
set(freertos_dir "${board_dir}/Middlewares/Third_Party/FreeRTOS/Source")
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
set(usb_library_dir "${board_dir}/Middlewares/ST/STM32_USB_Device_Library")
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
target_include_directories(os_freertos_${BOARD_ID} INTERFACE
    "${board_dir}/Core/Inc"
    "${freertos_dir}/include"
    "${freertos_dir}/CMSIS_RTOS_V2"
    "${freertos_dir}/portable/GCC/ARM_CM4F"
    "${board_dir}/Drivers/CMSIS/RTOS2/Include")

# ---- portkit：迁移期的 SPSC 队列、中断通知与短临界区（C，依赖 CMSIS 内联函数） ----
# P2.5 拆分：队列与通知 → src/utils，临界区契约 → src/hal/os，CMSIS 实现 → port。
set(portkit_dir "${LEGACY_DIR}/Platform/PortKit")
add_library(portkit OBJECT
    "${portkit_dir}/PortKit.c"
    "${portkit_dir}/PortKit.h")
target_compile_definitions(portkit PRIVATE ${cubemx_definitions})
target_include_directories(portkit
    PUBLIC "${portkit_dir}"
    PRIVATE ${cubemx_include_dirs})

# ---- port：契约（迁移期为 hal_detail 的 C ABI）的手写实现 ----
set(port_dir "${board_dir}/Port")
add_library(port_${BOARD_ID} OBJECT
    "${port_dir}/Can.c"
    "${port_dir}/Gpio.c"
    "${port_dir}/Init.c"
    "${port_dir}/Init.h"
    "${port_dir}/Pwm.c"
    "${port_dir}/Spi.c"
    "${port_dir}/Time.c"
    "${port_dir}/Uart.c"
    "${port_dir}/UsbCdc.c"
    "${port_dir}/UsbCdc.h")
enable_strict_warnings(port_${BOARD_ID})
target_link_libraries(port_${BOARD_ID} PRIVATE hal_detail)
link_object_library(port_${BOARD_ID}
    mx_generated_${BOARD_ID}
    rtos_${BOARD_ID}
    usb_${BOARD_ID}
    portkit)
# 生成区链接接缝（例外 X-02）：USB 发送经 port 包装，P4 改为回调注册后删除。
target_link_options(port_${BOARD_ID} INTERFACE "-Wl,--wrap=HAL_PCD_EP_Transmit")

# ---- board：板级内容。P1 只有链接脚本（CubeMX 生成，冻结）；
#      引脚表、能力常量、设备 ID 在 P3a 加入，链接脚本 overlay 在 P4 加入（Q9） ----
set(linker_script "${board_dir}/STM32H723xG_flash.ld")
add_library(board_${BOARD_ID} OBJECT "${linker_script}")
set_target_properties(board_${BOARD_ID} PROPERTIES LINKER_LANGUAGE C)
target_link_options(board_${BOARD_ID} INTERFACE "-T${linker_script}")
set_property(TARGET board_${BOARD_ID} APPEND PROPERTY INTERFACE_LINK_DEPENDS "${linker_script}")
