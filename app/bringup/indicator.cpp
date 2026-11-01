#include "bringup/indicator.hpp"

namespace bringup
{

namespace
{

// 上电提示音：两段各 80 ms，然后停止
constexpr hal::os::Microseconds CHIRP_STEP_US{80000U};
constexpr std::uint32_t CHIRP_FIRST_HZ{2000U};
constexpr std::uint32_t CHIRP_SECOND_HZ{2600U};
constexpr std::uint8_t CHIRP_VOLUME{96U};
constexpr hal::os::Microseconds BLINK_PERIOD_US{1000000U};

} // namespace

bool Indicator::init(const Config &config) noexcept
{
    config_ = config;
    started_ = false;
    seen_heartbeat_ = false;
    seen_imu_ = false;
    chirp_phase_ = 0U;
    status_ = Status::STOPPED;
    if (config.led != nullptr)
    {
        config.led->set_brightness(config.parameters.brightness);
    }
    return true;
}

void Indicator::step(hal::os::Microseconds now) noexcept
{
    if (!started_)
    {
        started_ = true;
        start_us_ = now;
    }
    topics::bringup::Heartbeat heartbeat{};
    if (heartbeat_.update(heartbeat))
    {
        seen_heartbeat_ = true;
        last_heartbeat_us_ = now;
    }
    topics::bringup::Imu imu{};
    if (imu_.update(imu))
    {
        seen_imu_ = true;
        last_imu_us_ = now;
    }

    const hal::os::Microseconds stale = config_.parameters.stale_after_us;
    if (seen_imu_ && !hal::os::has_elapsed(last_imu_us_, stale, now))
    {
        status_ = Status::IMU_RUNNING;
    }
    else if (seen_heartbeat_ && !hal::os::has_elapsed(last_heartbeat_us_, stale, now))
    {
        status_ = Status::HEARTBEAT;
    }
    else
    {
        status_ = Status::STOPPED;
    }

    if (config_.buzzer != nullptr && config_.parameters.chirp)
    {
        update_chirp(hal::os::elapsed_us(start_us_, now));
    }
    if (config_.led != nullptr)
    {
        const bool lit = (hal::os::elapsed_us(start_us_, now) % BLINK_PERIOD_US) < BLINK_PERIOD_US / 2U;
        device::RgbColor color = device::ws2812_color::BLACK;
        if (lit)
        {
            color = status_ == Status::IMU_RUNNING ? device::ws2812_color::GREEN
                    : status_ == Status::HEARTBEAT ? device::ws2812_color::YELLOW
                                                   : device::ws2812_color::RED;
        }
        config_.led->set_color(color);
        config_.led->process();
    }
}

void Indicator::update_chirp(hal::os::Microseconds elapsed) noexcept
{
    // 每段只在进入时设置一次；蜂鸣器失败（PWM 未就绪）不影响别的指示
    const std::uint8_t phase = elapsed < CHIRP_STEP_US ? 1U : (elapsed < 2U * CHIRP_STEP_US ? 2U : 3U);
    if (phase == chirp_phase_)
    {
        return;
    }
    chirp_phase_ = phase;
    if (phase == 1U)
    {
        (void)config_.buzzer->play(CHIRP_FIRST_HZ, CHIRP_VOLUME);
    }
    else if (phase == 2U)
    {
        (void)config_.buzzer->play(CHIRP_SECOND_HZ, CHIRP_VOLUME);
    }
    else
    {
        (void)config_.buzzer->stop();
    }
}

} // namespace bringup
