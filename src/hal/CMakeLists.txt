# hal：硬件资源的 C++ 契约（can、gpio、pwm、spi、time、uart、usb_cdc），由 platform 的 port 实现。
# 迁移期为 legacy/Platform/Interface/*.hpp，P2 搬到本目录，P3a 定稿契约 v1。
# 迁移期还要暴露 legacy/Platform/Interface 本身：迁移源有按裸头名 include 的写法（如 "Time.hpp"），P2 去掉。
set(interface_dir "${LEGACY_DIR}/Platform/Interface")

add_library(hal INTERFACE
    "${interface_dir}/Can.hpp"
    "${interface_dir}/Gpio.hpp"
    "${interface_dir}/Pwm.hpp"
    "${interface_dir}/Spi.hpp"
    "${interface_dir}/Time.hpp"
    "${interface_dir}/Uart.hpp"
    "${interface_dir}/UsbCdc.hpp")
target_include_directories(hal INTERFACE "${interface_dir}")
target_link_libraries(hal INTERFACE hal_detail legacy_include)
target_compile_features(hal INTERFACE cxx_std_17)

# hal_detail：迁移期的 C ABI 层（Detail/*.h），port 的 C 实现与 C++ 契约之间的过渡；
# P2 暂放 src/hal/detail/，P3a 并入 C++ 契约后删除（例外 X-07）。
add_library(hal_detail INTERFACE
    "${interface_dir}/Detail/Can.h"
    "${interface_dir}/Detail/Gpio.h"
    "${interface_dir}/Detail/Pwm.h"
    "${interface_dir}/Detail/Spi.h"
    "${interface_dir}/Detail/Time.h"
    "${interface_dir}/Detail/Uart.h"
    "${interface_dir}/Detail/UsbCdc.h")
target_include_directories(hal_detail INTERFACE "${interface_dir}")

add_subdirectory(os)
