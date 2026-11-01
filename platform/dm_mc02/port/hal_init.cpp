// hal::init() 在 DM-MC02 上的实现（Decision-20261031-11）：先启动 hal/os 的时基，再初始化各外设。
// 已改写为 C++ 的外设由 port.hpp 的 init_*() 初始化；其余仍由迁移期的 C port 初始化（Port_Init），
// F2 第 5 步逐个改写后去掉（Decision-20261101-01）。
#include "hal/init.hpp"

#include "Init.h" // Port_Init（C）
#include "os/os.hpp"
#include "port.hpp"

namespace hal
{

bool init() noexcept
{
    return port::os::start_clock() && port::init_pwm() && port::init_can() && port::init_spi() && port::init_uart() &&
           Port_Init();
}

} // namespace hal
