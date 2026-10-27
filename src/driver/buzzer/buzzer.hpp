#ifndef DRIVER_BUZZER_BUZZER_HPP
#define DRIVER_BUZZER_BUZZER_HPP

#include "hal/pwm.hpp"
#include <cstdint>

namespace device
{

class Buzzer final
{
  public:
    static constexpr std::uint32_t MIN_FREQUENCY_HZ{196U};
    static constexpr std::uint32_t MAX_FREQUENCY_HZ{4000U};
    static constexpr std::uint8_t MAX_VOLUME{255U};

    enum class State : std::uint8_t
    {
        STOPPED,
        PLAYING
    };

    using Result = platform::Pwm::Result;

    Buzzer() noexcept = default;

    /**
   * @brief 查询蜂鸣器底层 PWM 是否已经初始化。
   * @return 可以设置蜂鸣器输出时返回 true。
   */
    [[nodiscard]] bool is_ready() const noexcept;

    /**
   * @brief 以指定频率和音量持续播放蜂鸣器。
   * @param frequency_hz 频率，单位 Hz，有效范围为 196 到 4000。
   * @param volume 驱动音量，0 表示静音，255 对应 50% PWM 占空比。
   * @return 返回完成、未就绪、参数错误或底层错误。
   */
    [[nodiscard]] Result play(std::uint32_t frequency_hz, std::uint8_t volume) noexcept;

    /**
   * @brief 将蜂鸣器 PWM 占空比设置为 0。
   * @return 返回完成、未就绪或底层错误。
   */
    [[nodiscard]] Result stop() noexcept;

    /**
   * @brief 获取当前蜂鸣器的实际播放状态。
   * @return 最近一次成功操作后对应的停止或播放状态。
   */
    [[nodiscard]] State get_state() const noexcept
    {
        return state_;
    }

    /**
   * @brief 获取最近一次成功设置的播放频率。
   * @return 正在播放时返回频率 Hz，停止时返回 0。
   */
    [[nodiscard]] std::uint32_t get_frequency_hz() const noexcept
    {
        return frequency_hz_;
    }

    /**
   * @brief 获取最近一次成功设置的音量。
   * @return 正在播放时返回 1 到 255，停止时返回 0。
   */
    [[nodiscard]] std::uint8_t get_volume() const noexcept
    {
        return volume_;
    }

    /**
   * @brief 获取最近一次播放或停止操作的结果。
   * @return 最近一次操作结果。
   */
    [[nodiscard]] Result get_last_result() const noexcept
    {
        return last_result_;
    }

  private:
    static constexpr std::uint16_t MAX_DUTY_PERMILLE{500U};

    [[nodiscard]] static std::uint16_t volume_to_duty_permille(std::uint8_t volume) noexcept;

    State state_{State::STOPPED};
    std::uint32_t frequency_hz_{0U};
    std::uint8_t volume_{0U};
    Result last_result_{Result::NOT_READY};
};

} // namespace device

#endif
