#ifndef DRIVER_BMI088_BMI088_HEATER_HPP
#define DRIVER_BMI088_BMI088_HEATER_HPP

#include "lib/control/pid.hpp"
#include "driver/bmi088/bmi088_temperature.hpp"
#include "hal/pwm.hpp"
#include "hal/time.hpp"
#include <cstdint>

namespace device
{

/**
 * @brief BMI088 恒温控制器，以新鲜温度样本驱动板载加热 PWM。
 * @note 控制参数由原工程 10000 计数量纲等比例换算为 PWM 千分比；
 *       温度无效、过温或样本超时时输出立即归零。
 */
class Bmi088Heater final
{
  public:
    // 恒温服务点。上电零偏标定的温度门控必须与本值一致（见 ImuTask 的静态断言），
    // 否则标定永远进不了采样状态、z 轴零偏完全不会被扣除。
    static constexpr float DEFAULT_TARGET_CELSIUS{50.0F};
    static constexpr float MINIMUM_TARGET_CELSIUS{20.0F};
    static constexpr float MAXIMUM_TARGET_CELSIUS{60.0F};
    static constexpr float PREHEAT_THRESHOLD_CELSIUS{45.0F};
    static constexpr float MAXIMUM_SAFE_CELSIUS{65.0F};
    static constexpr std::uint32_t CONTROL_PERIOD_MS{128U};
    static constexpr std::uint32_t SAMPLE_TIMEOUT_MS{500U};
    static constexpr std::uint32_t PWM_FREQUENCY_HZ{100U};

    enum class State : std::uint8_t
    {
        DISABLED,
        WAITING_FOR_TEMPERATURE,
        PREHEATING,
        REGULATING,
        FAULT
    };

    Bmi088Heater() noexcept = default;

    /**
   * @brief 初始化 PID 并确保加热输出为零。
   * @return PID 与 PWM 通道均就绪且成功关闭输出时返回 true。
   */
    [[nodiscard]] bool init() noexcept;

    /**
   * @brief 启用或关闭恒温控制；关闭时清空 PID 并立即关闭 PWM。
   */
    void set_enabled(bool enabled) noexcept;

    /**
   * @brief 设置目标温度。
   * @return 温度有限且处于 20 至 60 摄氏度范围内时返回 true。
   */
    [[nodiscard]] bool set_target_celsius(float target_celsius) noexcept;

    /**
   * @brief 消费 BMI088 温度子设备的最新样本并推进恒温控制。
   * @note 可在主任务每轮调用，内部最多每 128 ms 更新一次控制量。
   */
    void process(const Bmi088Temperature &temperature) noexcept;

    [[nodiscard]] bool is_enabled() const noexcept
    {
        return enabled_;
    }
    [[nodiscard]] State get_state() const noexcept
    {
        return state_;
    }
    [[nodiscard]] float get_target_celsius() const noexcept
    {
        return target_celsius_;
    }
    [[nodiscard]] float get_temperature_celsius() const noexcept
    {
        return temperature_celsius_;
    }
    [[nodiscard]] std::uint16_t get_duty_permille() const noexcept
    {
        return duty_permille_;
    }
    [[nodiscard]] platform::Pwm::Result get_last_pwm_result() const noexcept
    {
        return last_pwm_result_;
    }

  private:
    // 预热占空比：满功率（1000‰）实测会把芯体冲到约 92 ℃。BMI088 温度寄存器
    // 每 1.28 s 才更新一次（数据手册 5.3.7 节），叠加 128 ms 控制周期后反馈盲区
    // 约 1.45 s；满功率下芯体温升约 39 ℃/s，等控制环首次“看见”45 ℃ 预热门槛时
    // 真实温度已过冲约 +47 ℃，随后触发 65 ℃ 安全保护并进入分钟级回摆。
    // 200‰（20%）由实测“10% 稳态 42.4 ℃”外推平衡温度约高于环境 37 ℃，仍能
    // 越过门槛，而盲区过冲被限制在几度以内；更冷的环境由 PI 项补足。
    static constexpr std::uint16_t PREHEAT_DUTY_PERMILLE{200U};

    void disable_output(State next_state) noexcept;
    [[nodiscard]] bool set_duty(std::uint16_t duty_permille) noexcept;

    control::PID pid_{};
    State state_{State::DISABLED};
    bool initialized_{false};
    bool enabled_{false};
    bool sample_seen_{false};
    float target_celsius_{DEFAULT_TARGET_CELSIUS};
    float temperature_celsius_{0.0F};
    std::uint16_t duty_permille_{0U};
    std::uint32_t last_temperature_read_count_{0U};
    platform::Time::Tick last_sample_tick_{0U};
    platform::Time::Tick last_control_tick_{0U};
    platform::Pwm::Result last_pwm_result_{platform::Pwm::Result::NOT_READY};
};

} // namespace device

#endif
