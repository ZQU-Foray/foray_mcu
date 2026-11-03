#ifndef DRIVER_BMI088_BMI088_HEATER_HPP
#define DRIVER_BMI088_BMI088_HEATER_HPP

#include "driver/bmi088/bmi088_temperature.hpp"
#include "hal/os/clock.hpp"
#include "hal/pwm.hpp"
#include "lib/control/heater_control.hpp"

#include <cstdint>

namespace device
{

/**
 * @brief BMI088 恒温：用温度样本驱动板载加热 PWM。
 * @note 控制算法在 control::HeaterControl：按板子的热模型预测预热、提前收功率，再恒温；本类负责读数、
 *       时间、PWM 与安全保护。温度无效、过温或样本超时时输出立即归零。
 * @note 加热 PWM 的句柄与本板的热模型由组合根在构造时注入（如 board::PWM_IMU_HEATER、params::HEATER）。
 */
class Bmi088Heater final
{
  public:
    // 恒温服务点。上电零偏标定的温度门控必须与本值一致，否则标定永远进不了采样状态。
    static constexpr float DEFAULT_TARGET_CELSIUS{50.0F};
    static constexpr float MINIMUM_TARGET_CELSIUS{20.0F};
    static constexpr float MAXIMUM_TARGET_CELSIUS{60.0F};
    static constexpr float MAXIMUM_SAFE_CELSIUS{65.0F};
    static constexpr hal::os::Microseconds CONTROL_PERIOD_US{128000U};
    static constexpr hal::os::Microseconds SAMPLE_TIMEOUT_US{500000U};
    static constexpr std::uint32_t PWM_FREQUENCY_HZ{100U};
    // BMI088 温度寄存器每 1.28 s 更新一次（数据手册 5.3.7 节），分辨率 0.125 ℃
    static constexpr float TEMPERATURE_UPDATE_PERIOD_S{1.28F};
    // 驱动每 40 ms 读一次温度（Bmi088::TEMPERATURE_PERIOD_US），看到读数变化时平均滞后半个读取周期
    static constexpr float TEMPERATURE_READ_LAG_S{0.02F};

    /** 来自 profile 的参数。 */
    struct Config
    {
        control::HeaterControl::Model model;     // 本板的热模型（实测辨识）
        control::HeaterControl::Tuning tuning{}; // 控制参数，一般用默认值
    };

    enum class State : std::uint8_t
    {
        DISABLED,
        WAITING_FOR_TEMPERATURE,
        PREHEATING,
        REGULATING,
        FAULT
    };

    constexpr Bmi088Heater(hal::Pwm pwm, const Config &config) noexcept : pwm_{pwm}, config_{config} {}

    /**
   * @brief 初始化控制器并确保加热输出为零。
   * @return 参数有效、PWM 通道就绪且成功关闭输出时返回 true。
   */
    [[nodiscard]] bool init() noexcept;

    /**
   * @brief 启用或关闭恒温控制；启用时控制器从下一个读数重新开始，关闭时立即关闭 PWM。
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
    [[nodiscard]] hal::Pwm::Result get_last_pwm_result() const noexcept
    {
        return last_pwm_result_;
    }
    /** 控制器的内部状态（阶段、加热器强度修正系数、模型估计），供观测。 */
    [[nodiscard]] const control::HeaterControl &get_control() const noexcept
    {
        return control_;
    }

  private:
    void disable_output(State next_state) noexcept;
    [[nodiscard]] bool set_duty(std::uint16_t duty_permille) noexcept;

    hal::Pwm pwm_;
    Config config_;
    control::HeaterControl control_{};
    State state_{State::DISABLED};
    bool initialized_{false};
    bool enabled_{false};
    bool sample_seen_{false};
    float target_celsius_{DEFAULT_TARGET_CELSIUS};
    float temperature_celsius_{0.0F};
    std::uint16_t duty_permille_{0U};
    std::uint32_t last_temperature_read_count_{0U};
    std::int16_t last_raw_{0};
    hal::os::Microseconds last_change_us_{0U}; // 读数最近一次变化的那次读取完成的时刻
    hal::os::Microseconds last_sample_us_{0U};
    hal::os::Microseconds last_control_us_{0U};
    hal::Pwm::Result last_pwm_result_{hal::Pwm::Result::NOT_READY};
};

} // namespace device

#endif // DRIVER_BMI088_BMI088_HEATER_HPP
