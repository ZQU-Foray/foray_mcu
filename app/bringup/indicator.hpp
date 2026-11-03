#ifndef BRINGUP_INDICATOR_HPP
#define BRINGUP_INDICATOR_HPP

#include "driver/buzzer/buzzer.hpp"
#include "driver/ws2812/ws2812.hpp"
#include "hal/os/clock.hpp"
#include "middleware/topic/topic.hpp"
#include "topics/bringup.hpp"

#include <cstdint>

namespace bringup
{

/**
 * @brief 指示：上电时蜂鸣器响两声，板载 RGB 灯按状态闪烁，验证 PWM 与 SPI 发送。
 *        灯：IMU 数据在更新为绿色；只有心跳为黄色；心跳也停了为红色。1 Hz 闪烁（亮 0.5 s）。
 */
class Indicator final
{
  public:
    struct Parameters
    {
        hal::os::Microseconds stale_after_us; // 超过这么久没有新数据就算停了
        std::uint8_t brightness;              // 灯的亮度，0–255
        bool chirp;                           // 上电时响两声
    };

    struct Config
    {
        device::Buzzer *buzzer; // 可以为空
        device::Ws2812 *led;    // 可以为空
        Parameters parameters;
    };

    enum class Status : std::uint8_t
    {
        IMU_RUNNING, // 绿
        HEARTBEAT,   // 黄
        STOPPED      // 红
    };

    [[nodiscard]] bool init(const Config &config) noexcept;

    void step(hal::os::Microseconds now) noexcept;

    [[nodiscard]] Status status() const noexcept
    {
        return status_;
    }

  private:
    void update_chirp(hal::os::Microseconds elapsed) noexcept;

    Config config_{nullptr, nullptr, Parameters{0U, 0U, false}};
    topic::Subscriber<topics::bringup::Heartbeat> heartbeat_{topics::bringup::heartbeat};
    topic::Subscriber<topics::bringup::Imu> imu_{topics::bringup::imu};
    hal::os::Microseconds start_us_{0U};
    hal::os::Microseconds last_heartbeat_us_{0U};
    hal::os::Microseconds last_imu_us_{0U};
    bool started_{false};
    bool seen_heartbeat_{false};
    bool seen_imu_{false};
    std::uint8_t chirp_phase_{0U};
    Status status_{Status::STOPPED};
};

} // namespace bringup

#endif // BRINGUP_INDICATOR_HPP
