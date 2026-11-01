// hal::Pwm 在 DM-MC02 上的实现（Decision-20261101-01）。通道表的下标就是 board.hpp 里句柄的编号。
// 两个定时器都在 APB1 上，CubeMX 把预分频配成 1 MHz 计数；init_pwm() 按实际时钟核对，不对就拒绝启动。
#include "hal/pwm.hpp"

#include "board.hpp"
#include "port.hpp"
#include "tim.h" // htim3、htim12；带进 stm32h7xx_hal.h

#include <array>
#include <cstdint>

namespace
{

struct Channel
{
    TIM_HandleTypeDef *timer;
    std::uint32_t timer_channel;
};

// 顺序与 board.hpp 一致
const std::array CHANNELS{
    Channel{&htim12, TIM_CHANNEL_2}, // PWM_BUZZER
    Channel{&htim3, TIM_CHANNEL_4},  // PWM_IMU_HEATER
};
static_assert(std::tuple_size_v<decltype(CHANNELS)> == board::PWM_COUNT, "表长必须与 board.hpp 一致");

constexpr std::uint32_t COUNTER_FREQUENCY_HZ{1000000U};
constexpr std::uint32_t MAX_PERIOD_COUNTS{65536U}; // 16 位定时器
constexpr std::uint32_t MIN_PERIOD_COUNTS{2U};
static_assert(COUNTER_FREQUENCY_HZ / MAX_PERIOD_COUNTS + 1U == board::PWM_MIN_FREQUENCY_HZ);
static_assert(COUNTER_FREQUENCY_HZ / MIN_PERIOD_COUNTS == board::PWM_MAX_FREQUENCY_HZ);

bool g_ready{false};

// APB1 定时器的输入时钟（RM0468 的 TIMPRE 规则）：
// TIMPRE=0 时，APB1 不分频或二分频取 HCLK，否则取 PCLK1 × 2；TIMPRE=1 时，四分频以内取 HCLK，否则取 PCLK1 × 4。
std::uint32_t apb1_timer_clock_hz() noexcept
{
    RCC_ClkInitTypeDef clock{};
    RCC_PeriphCLKInitTypeDef peripheral_clock{};
    std::uint32_t flash_latency = 0U;
    HAL_RCC_GetClockConfig(&clock, &flash_latency);
    HAL_RCCEx_GetPeriphCLKConfig(&peripheral_clock);

    const std::uint32_t divider = clock.APB1CLKDivider;
    if (peripheral_clock.TIMPresSelection == RCC_TIMPRES_DESACTIVATED)
    {
        return divider == RCC_APB1_DIV1 || divider == RCC_APB1_DIV2 ? HAL_RCC_GetHCLKFreq()
                                                                    : HAL_RCC_GetPCLK1Freq() * 2U;
    }
    return divider == RCC_APB1_DIV1 || divider == RCC_APB1_DIV2 || divider == RCC_APB1_DIV4
               ? HAL_RCC_GetHCLKFreq()
               : HAL_RCC_GetPCLK1Freq() * 4U;
}

} // namespace

bool port::init_pwm() noexcept
{
    g_ready = false;
    const std::uint32_t timer_clock_hz = apb1_timer_clock_hz();
    for (const Channel &channel : CHANNELS)
    {
        if (channel.timer->State != HAL_TIM_STATE_READY ||
            timer_clock_hz / (channel.timer->Init.Prescaler + 1U) != COUNTER_FREQUENCY_HZ)
        {
            return false;
        }
    }

    for (std::size_t started = 0U; started < CHANNELS.size(); ++started)
    {
        const Channel &channel = CHANNELS[started];
        __HAL_TIM_SET_COMPARE(channel.timer, channel.timer_channel, 0U);
        if (HAL_TIM_PWM_Start(channel.timer, channel.timer_channel) != HAL_OK)
        {
            for (std::size_t index = 0U; index < started; ++index)
            {
                (void)HAL_TIM_PWM_Stop(CHANNELS[index].timer, CHANNELS[index].timer_channel);
            }
            return false;
        }
    }
    g_ready = true;
    return true;
}

namespace hal
{

bool Pwm::is_ready() const noexcept
{
    return g_ready && index_ < CHANNELS.size();
}

Pwm::Result Pwm::set(std::uint32_t frequency_hz, std::uint16_t duty_permille) const noexcept
{
    if (index_ >= CHANNELS.size() || frequency_hz == 0U || duty_permille > MAX_DUTY_PERMILLE)
    {
        return Result::INVALID_ARGUMENT;
    }
    if (!g_ready)
    {
        return Result::NOT_READY;
    }

    // 四舍五入到整数个计数
    const std::uint32_t period_counts = (COUNTER_FREQUENCY_HZ + frequency_hz / 2U) / frequency_hz;
    if (period_counts < MIN_PERIOD_COUNTS || period_counts > MAX_PERIOD_COUNTS)
    {
        return Result::INVALID_ARGUMENT;
    }
    const std::uint32_t compare_counts =
        (period_counts * duty_permille + MAX_DUTY_PERMILLE / 2U) / static_cast<std::uint32_t>(MAX_DUTY_PERMILLE);

    // 先把比较值清零再改周期，避免新旧周期交替时出现一个过长的脉冲
    const Channel &channel = CHANNELS[index_];
    __HAL_TIM_SET_COMPARE(channel.timer, channel.timer_channel, 0U);
    __HAL_TIM_SET_AUTORELOAD(channel.timer, period_counts - 1U);
    __HAL_TIM_SET_COUNTER(channel.timer, 0U);
    __HAL_TIM_SET_COMPARE(channel.timer, channel.timer_channel, compare_counts);
    return Result::COMPLETED;
}

Pwm::Result Pwm::off() const noexcept
{
    if (index_ >= CHANNELS.size())
    {
        return Result::INVALID_ARGUMENT;
    }
    if (!g_ready)
    {
        return Result::NOT_READY;
    }
    __HAL_TIM_SET_COMPARE(CHANNELS[index_].timer, CHANNELS[index_].timer_channel, 0U);
    return Result::COMPLETED;
}

} // namespace hal
