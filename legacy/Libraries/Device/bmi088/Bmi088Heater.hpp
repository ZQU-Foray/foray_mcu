#ifndef LIBRARIES_DEVICE_BMI088_BMI088_HEATER_HPP
#define LIBRARIES_DEVICE_BMI088_BMI088_HEATER_HPP

#include "Libraries/Algorithm/alg_controller/Pid.h"
#include "Libraries/Device/bmi088/Bmi088Temperature.hpp"
#include "Platform/Interface/Pwm.hpp"
#include "Platform/Interface/Time.hpp"
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
  static constexpr float DefaultTargetCelsius{50.0F};
  static constexpr float MinimumTargetCelsius{20.0F};
  static constexpr float MaximumTargetCelsius{60.0F};
  static constexpr float PreheatThresholdCelsius{45.0F};
  static constexpr float MaximumSafeCelsius{65.0F};
  static constexpr std::uint32_t ControlPeriodMs{128U};
  static constexpr std::uint32_t SampleTimeoutMs{500U};
  static constexpr std::uint32_t PwmFrequencyHz{100U};

  enum class State : std::uint8_t
  {
    Disabled,
    WaitingForTemperature,
    Preheating,
    Regulating,
    Fault
  };

  Bmi088Heater() noexcept = default;

  /**
   * @brief 初始化 PID 并确保加热输出为零。
   * @return PID 与 PWM 通道均就绪且成功关闭输出时返回 true。
   */
  [[nodiscard]] bool Init() noexcept;

  /**
   * @brief 启用或关闭恒温控制；关闭时清空 PID 并立即关闭 PWM。
   */
  void SetEnabled(bool enabled) noexcept;

  /**
   * @brief 设置目标温度。
   * @return 温度有限且处于 20 至 60 摄氏度范围内时返回 true。
   */
  [[nodiscard]] bool SetTargetCelsius(float targetCelsius) noexcept;

  /**
   * @brief 消费 BMI088 温度子设备的最新样本并推进恒温控制。
   * @note 可在主任务每轮调用，内部最多每 128 ms 更新一次控制量。
   */
  void Process(const Bmi088Temperature &temperature) noexcept;

  [[nodiscard]] bool IsEnabled() const noexcept
  {
    return enabled_;
  }
  [[nodiscard]] State GetState() const noexcept
  {
    return state_;
  }
  [[nodiscard]] float GetTargetCelsius() const noexcept
  {
    return targetCelsius_;
  }
  [[nodiscard]] float GetTemperatureCelsius() const noexcept
  {
    return temperatureCelsius_;
  }
  [[nodiscard]] std::uint16_t GetDutyPermille() const noexcept
  {
    return dutyPermille_;
  }
  [[nodiscard]] platform::Pwm::Result GetLastPwmResult() const noexcept
  {
    return lastPwmResult_;
  }

private:
  // 预热占空比：满功率（1000‰）实测会把芯体冲到约 92 ℃。BMI088 温度寄存器
  // 每 1.28 s 才更新一次（数据手册 5.3.7 节），叠加 128 ms 控制周期后反馈盲区
  // 约 1.45 s；满功率下芯体温升约 39 ℃/s，等控制环首次“看见”45 ℃ 预热门槛时
  // 真实温度已过冲约 +47 ℃，随后触发 65 ℃ 安全保护并进入分钟级回摆。
  // 200‰（20%）由实测“10% 稳态 42.4 ℃”外推平衡温度约高于环境 37 ℃，仍能
  // 越过门槛，而盲区过冲被限制在几度以内；更冷的环境由 PI 项补足。
  static constexpr std::uint16_t PreheatDutyPermille{200U};

  void DisableOutput(State nextState) noexcept;
  [[nodiscard]] bool SetDuty(std::uint16_t dutyPermille) noexcept;

  alg_controller::PID pid_{};
  State state_{State::Disabled};
  bool initialized_{false};
  bool enabled_{false};
  bool sampleSeen_{false};
  float targetCelsius_{DefaultTargetCelsius};
  float temperatureCelsius_{0.0F};
  std::uint16_t dutyPermille_{0U};
  std::uint32_t lastTemperatureReadCount_{0U};
  platform::Time::Tick lastSampleTick_{0U};
  platform::Time::Tick lastControlTick_{0U};
  platform::Pwm::Result lastPwmResult_{platform::Pwm::Result::NotReady};
};

} // namespace device

#endif
