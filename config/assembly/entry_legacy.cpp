#include "Entry.h"
#include "remote/remote_receiver.hpp"
#include "Application/Task/TaskManager.hpp"
#include "hal/can.hpp"
#include "hal/gpio.hpp"
#include "hal/spi.hpp"
#include "hal/time.hpp"
#include "hal/uart.hpp"

bool Application_Init(void)
{
    // 核前初始化：只做平台能力校验与各任务的非 RTOS 部分初始化，
    // 不得创建线程或依赖内核状态的对象
    if (!platform::Time::is_ready())
    {
        return false;
    }

    platform::Gpio::Level key_level = platform::Gpio::Level::LOW;
    if (!platform::Gpio::read(platform::Gpio::Pin::USER_KEY, key_level))
    {
        return false;
    }

    platform::Gpio::Level power_5v_level = platform::Gpio::Level::HIGH;
    if (!platform::Gpio::write(platform::Gpio::Pin::POWER_5V, power_5v_level))
    {
        return false;
    }

    if (!platform::Can::is_ready(platform::Can::Channel::CHANNEL_1) ||
        !platform::Can::is_ready(platform::Can::Channel::CHANNEL_2) ||
        !platform::Can::is_ready(platform::Can::Channel::CHANNEL_3))
    {
        return false;
    }

    if (!platform::Spi::is_ready(platform::Spi::Device::IMU_ACCELEROMETER) ||
        !platform::Spi::is_ready(platform::Spi::Device::IMU_GYROSCOPE) ||
        !platform::Spi::is_ready(platform::Spi::Device::ADDRESSABLE_LED))
    {
        return false;
    }

    if (!platform::Uart::is_ready(platform::Uart::Endpoint::REMOTE_RECEIVER) ||
        !platform::Uart::is_ready(platform::Uart::Endpoint::DEBUG_CONSOLE))
    {
        return false;
    }

    if (!application::task::TaskManager::init())
    {
        return false;
    }
    return application::RemoteReceiver::init();
}

bool Application_InitRtos(void)
{
    // 应用线程的唯一创建入口：任务清单、优先级、栈与句柄集中在 TaskManager 任务表。
    return application::task::TaskManager::init_rtos();
}

bool Application_Start(void)
{
    // 避免漏接一行就整机静默不启动；创建时机会被记为 POST_KERNEL 以暴露接线缺陷。
    if (!Application_InitRtos())
    {
        return false;
    }

    // 任务表未就绪（初始化失败或启动顺序错误）时不执行任何控制或通信周期工作。
    if (!application::task::TaskManager::start())
    {
        return false;
    }

    // 栈余量巡检：本入口由 defaultTask 每拍调用，采样在 TaskManager 内按 1 Hz 限频。
    // 栈溢出钩子位于 CubeMX 生成文件且为空实现（P0 禁止修改），因此把各任务的高水位
    // 余量写到调试器可见存档里，让"接近溢出"在冻结之前就能被读到。
    application::task::TaskManager::sample_stack_health();

    application::RemoteReceiver::process();
    return true;
}
