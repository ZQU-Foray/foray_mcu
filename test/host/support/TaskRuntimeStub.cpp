#include "cmsis_os2.h"
#include "Platform/Interface/Detail/Time.h"
#include <cstdlib>
// 纯控制测试仅调用任务配置获取函数；意外启动任务立即使测试失败。
std::int32_t osDelay(std::uint32_t) { std::abort(); }
[[noreturn]] void osThreadExit() { std::abort(); }
extern "C" {
bool TimePort_Init() { return true; }
bool TimePort_IsReady() { return true; }
std::uint32_t TimePort_NowTicks() { return 0; }
std::uint32_t TimePort_FrequencyHz() { return 1000; }
void TimePort_DelayUs(std::uint32_t) {}
}
