#include "Detail/Time.h"
#include "main.h"
#include <limits.h>
#include <stdint.h>

#define DWT_UNLOCK_KEY 0xC5ACCE55UL
#define MICROSECONDS_PER_SECOND 1000000ULL

// 将单次等待限制在半个计数周期内，避免长延时跨越一次完整回绕
#define MAX_DELAY_CHUNK_TICKS (UINT32_MAX / 2U)

static uint32_t time_tick_frequency_hz = 0U;
static bool time_port_ready = false;

static bool TimePort_EnsureCounterRunning(void)
{
  if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) != 0U && (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U)
  {
    return true;
  }

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->LAR = DWT_UNLOCK_KEY;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  __DSB();
  __ISB();
  return (CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) != 0U && (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U;
}

/**
 * @brief 初始化 Cortex-M7 DWT 周期计数器
 * @return 初始化成功时返回 true，否则返回 false
 * @note 本函数应在系统时钟配置完成后调用。若运行期间修改内核频率，则必须重新
 * @note 初始化时间 Port，使计数频率与 SystemCoreClock 保持一致
 */
bool TimePort_Init(void)
{
  time_port_ready = false;
  time_tick_frequency_hz = 0U;

  if (SystemCoreClock == 0U || (DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) != 0U)
  {
    return false;
  }

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->LAR = DWT_UNLOCK_KEY;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  __DSB();
  __ISB();

  if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U)
  {
    return false;
  }

  time_tick_frequency_hz = SystemCoreClock;
  time_port_ready = true;
  return true;
}

bool TimePort_IsReady(void)
{
  return time_port_ready && time_tick_frequency_hz != 0U && TimePort_EnsureCounterRunning();
}

uint32_t TimePort_NowTicks(void)
{
  if (!TimePort_IsReady())
  {
    return 0U;
  }

  return DWT->CYCCNT;
}

uint32_t TimePort_FrequencyHz(void)
{
  if (!TimePort_IsReady())
  {
    return 0U;
  }

  return time_tick_frequency_hz;
}

void TimePort_DelayUs(uint32_t delay_us)
{
  uint64_t remaining_ticks;

  if (!TimePort_IsReady() || delay_us == 0U)
  {
    return;
  }

  // 向上取整，保证非零延时时间不会因整数截断而短于请求值
  remaining_ticks = ((uint64_t)delay_us * (uint64_t)time_tick_frequency_hz + MICROSECONDS_PER_SECOND - 1ULL) /
                    MICROSECONDS_PER_SECOND;

  while (remaining_ticks > 0ULL)
  {
    const uint32_t chunk_ticks =
        remaining_ticks > MAX_DELAY_CHUNK_TICKS ? MAX_DELAY_CHUNK_TICKS : (uint32_t)remaining_ticks;
    const uint32_t start_tick = DWT->CYCCNT;

    while ((uint32_t)(DWT->CYCCNT - start_tick) < chunk_ticks)
    {
    }

    remaining_ticks -= chunk_ticks;
  }
}
