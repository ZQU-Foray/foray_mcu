// hal::init() 在 DM-MC02 上的实现（Decision-20261031-11）：先启动 hal/os 的时基，再初始化各外设。
// 外设目前仍由迁移期的 C port 初始化（Port_Init），F2 第 5 步逐个改写为 C++ 后去掉。
#include "hal/init.hpp"

#include "Init.h" // Port_Init（C）
#include "os/os.hpp"

namespace hal
{

bool init() noexcept
{
    return port::os::start_clock() && Port_Init();
}

} // namespace hal
