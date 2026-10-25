#include "Entry.h"
#include "Application/Task/4_communication/RemoteReceiver.hpp"
#include "Application/Task/TaskManager.hpp"
#include "Can.hpp"
#include "Gpio.hpp"
#include "Spi.hpp"
#include "Time.hpp"
#include "Uart.hpp"

bool Application_Init(void)
{
  // 核前初始化：只做平台能力校验与各任务的非 RTOS 部分初始化，
  // 不得创建线程或依赖内核状态的对象
  if (!platform::Time::IsReady())
  {
    return false;
  }

  platform::Gpio::Level keyLevel = platform::Gpio::Level::Low;
  if (!platform::Gpio::Read(platform::Gpio::Pin::UserKey, keyLevel))
  {
    return false;
  }

  platform::Gpio::Level Power5VLevel = platform::Gpio::Level::High;
  if (!platform::Gpio::Write(platform::Gpio::Pin::Power5V, Power5VLevel))
  {
    return false;
  }

  if (!platform::Can::IsReady(platform::Can::Channel::Channel1) ||
      !platform::Can::IsReady(platform::Can::Channel::Channel2) ||
      !platform::Can::IsReady(platform::Can::Channel::Channel3))
  {
    return false;
  }

  if (!platform::Spi::IsReady(platform::Spi::Device::ImuAccelerometer) ||
      !platform::Spi::IsReady(platform::Spi::Device::ImuGyroscope) ||
      !platform::Spi::IsReady(platform::Spi::Device::AddressableLed))
  {
    return false;
  }

  if (!platform::Uart::IsReady(platform::Uart::Endpoint::RemoteReceiver) ||
      !platform::Uart::IsReady(platform::Uart::Endpoint::DebugConsole))
  {
    return false;
  }

  if (!application::task::TaskManager::Init())
  {
    return false;
  }
  return application::RemoteReceiver::Init();
}

bool Application_InitRtos(void)
{
  // 应用线程的唯一创建入口：任务清单、优先级、栈与句柄集中在 TaskManager 任务表。
  return application::task::TaskManager::InitRtos();
}

bool Application_Start(void)
{
  // 避免漏接一行就整机静默不启动；创建时机会被记为 PostKernel 以暴露接线缺陷。
  if (!Application_InitRtos())
  {
    return false;
  }

  // 任务表未就绪（初始化失败或启动顺序错误）时不执行任何控制或通信周期工作。
  if (!application::task::TaskManager::Start())
  {
    return false;
  }

  // 栈余量巡检：本入口由 defaultTask 每拍调用，采样在 TaskManager 内按 1 Hz 限频。
  // 栈溢出钩子位于 CubeMX 生成文件且为空实现（P0 禁止修改），因此把各任务的高水位
  // 余量写到调试器可见存档里，让"接近溢出"在冻结之前就能被读到。
  application::task::TaskManager::SampleStackHealth();

  application::RemoteReceiver::Process();
  return true;
}
