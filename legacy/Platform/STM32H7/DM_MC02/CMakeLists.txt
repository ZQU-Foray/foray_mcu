cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME mc02_cpp)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM, C and C++ languages
enable_language(C CXX ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE $<$<CONFIG:Debug>:IMU_ENABLE_DIAGNOSTICS>)

# Sensor-register-only A/B diagnostic; never changes CubeMX SPI configuration.
set(IMU_GYRO_BANDWIDTH "532" CACHE STRING "BMI088 gyro bandwidth: 532 or 116 Hz")
set_property(CACHE IMU_GYRO_BANDWIDTH PROPERTY STRINGS 532 116)
if(IMU_GYRO_BANDWIDTH STREQUAL "532")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE BMI088_GYRO_BANDWIDTH_REGISTER=0x00U)
elseif(IMU_GYRO_BANDWIDTH STREQUAL "116")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE BMI088_GYRO_BANDWIDTH_REGISTER=0x02U)
else()
    message(FATAL_ERROR "IMU_GYRO_BANDWIDTH must be 532 or 116")
endif()

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

file(READ "${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/Target/usbd_conf.c"
    USB_DEVICE_LOW_LEVEL_SOURCE)
if(NOT USB_DEVICE_LOW_LEVEL_SOURCE MATCHES
   "hpcd_USB_OTG_HS\\.Init\\.dma_enable[ \t]*=[ \t]*ENABLE")
    message(FATAL_ERROR
        "USB OTG HS internal DMA must remain enabled after CubeMX generation")
endif()
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/Port/UsbCdc.c"
    USB_CDC_PORT_SOURCE)
if(NOT USB_CDC_PORT_SOURCE MATCHES
   "gpio_init\\.Alternate[ \t]*=[ \t]*GPIO_AF10_OTG1_FS")
    message(FATAL_ERROR
        "USB OTG HS PA11/PA12 alternate-function setup is missing from UsbCdc Port")
endif()

# Add the platform-independent interface.
add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Interface
    ${CMAKE_CURRENT_BINARY_DIR}/platform_interface
)

# Add the platform-independent Porting Kit (queue / notifier / critical section).
add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/../../PortKit
    ${CMAKE_CURRENT_BINARY_DIR}/portkit
)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # CMSIS-DSP 静态库
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Lib
)

file(GLOB_RECURSE PROJECT_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/Port/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Port/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../Application/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../Application/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../Libraries/Algorithm/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../Libraries/Algorithm/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../Libraries/Component/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../Libraries/Component/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../Libraries/Device/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../Libraries/Device/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../Libraries/Protocol/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../Libraries/Protocol/*.cpp
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${PROJECT_SOURCES}
)

# 严格编译（-Wextra -Werror）自动应用于全部业务源文件（随 PROJECT_SOURCES
# 收集，新增/删除/改名文件零维护）。若有目录暂未达标，可先用 foreach 对
# PROJECT_SOURCES 按 MATCHES 过滤暂免，清零告警后恢复全量。
set_source_files_properties(${PROJECT_SOURCES} PROPERTIES COMPILE_OPTIONS "-Wextra;-Werror")

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/Port
    ${CMAKE_CURRENT_SOURCE_DIR}/../../..
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ARM_MATH_CM7
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    platform_interface
    portkit

    # CMSIS-DSP
    arm_cortexM7lfsp_math
)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    "-Wl,--wrap=HAL_PCD_EP_Transmit"
)

# Generate standard firmware images next to the ELF file.
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMENT "Generating HEX/BIN firmware images"
)

# OpenOCD targets appear directly in CLion's CMake target list.
if(NOT OPENOCD_ROOT AND DEFINED ENV{OPENOCD_ROOT})
    set(OPENOCD_ROOT "$ENV{OPENOCD_ROOT}")
endif()
find_program(OPENOCD_EXECUTABLE
    NAMES openocd openocd.exe
    HINTS "${OPENOCD_ROOT}/bin"
)
set(OPENOCD_CONFIG "${CMAKE_CURRENT_SOURCE_DIR}/openocd/stm32h723_stlink.cfg")

if(OPENOCD_EXECUTABLE AND EXISTS "${OPENOCD_CONFIG}")
    add_custom_target(flash
        COMMAND "${OPENOCD_EXECUTABLE}"
                -f "${OPENOCD_CONFIG}"
                -c "program $<TARGET_FILE:${CMAKE_PROJECT_NAME}> verify reset exit"
        DEPENDS ${CMAKE_PROJECT_NAME}
        USES_TERMINAL
        COMMENT "Flashing STM32H723 with OpenOCD"
    )

    add_custom_target(openocd-server
        COMMAND "${OPENOCD_EXECUTABLE}" -f "${OPENOCD_CONFIG}"
        USES_TERMINAL
        COMMENT "Starting the STM32H723 OpenOCD server"
    )
else()
    message(STATUS "OpenOCD board configuration not found; flash targets are unavailable")
endif()
