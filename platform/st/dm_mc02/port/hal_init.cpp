// hal::init() 在 DM-MC02 上的实现：先启动 hal/os 的时基，
// 再按 port.hpp 的 init_*() 逐个启动外设。CubeMX 的 MX_*_Init 已在 boot_init() 之前配置好外设与 DMA，
// 这里只做 port 自己的检查与启动。USB CDC 由生成的 MX_USB_DEVICE_Init 经 UsbCdcPort_Init 初始化（USB 生成桩，Q8 方案①）。
#include "hal/init.hpp"

#include "os/os.hpp"
#include "port.hpp"

namespace hal
{

bool init() noexcept
{
    return port::os::start_clock() && port::init_pwm() && port::init_can() && port::init_spi() && port::init_uart();
}

} // namespace hal
