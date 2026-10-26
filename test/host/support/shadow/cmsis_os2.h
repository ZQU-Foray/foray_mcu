#pragma once
#include <cstdint>
std::int32_t osDelay(std::uint32_t ticks);
[[noreturn]] void osThreadExit();
