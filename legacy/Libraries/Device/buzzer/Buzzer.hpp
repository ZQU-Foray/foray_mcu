#ifndef LIBRARIES_DEVICE_BUZZER_BUZZER_HPP
#define LIBRARIES_DEVICE_BUZZER_BUZZER_HPP

#include "Platform/Interface/Pwm.hpp"
#include <cstdint>

namespace device
{

class Buzzer final
{
public:
  static constexpr std::uint32_t MinFrequencyHz{196U};
  static constexpr std::uint32_t MaxFrequencyHz{4000U};
  static constexpr std::uint8_t MaxVolume{255U};

  enum class State : std::uint8_t
  {
    Stopped,
    Playing
  };

  using Result = platform::Pwm::Result;

  Buzzer() noexcept = default;

  /**
   * @brief 查询蜂鸣器底层 PWM 是否已经初始化。
   * @return 可以设置蜂鸣器输出时返回 true。
   */
  [[nodiscard]] bool IsReady() const noexcept;

  /**
   * @brief 以指定频率和音量持续播放蜂鸣器。
   * @param frequencyHz 频率，单位 Hz，有效范围为 196 到 4000。
   * @param volume 驱动音量，0 表示静音，255 对应 50% PWM 占空比。
   * @return 返回完成、未就绪、参数错误或底层错误。
   */
  [[nodiscard]] Result Play(std::uint32_t frequencyHz, std::uint8_t volume) noexcept;

  /**
   * @brief 将蜂鸣器 PWM 占空比设置为 0。
   * @return 返回完成、未就绪或底层错误。
   */
  [[nodiscard]] Result Stop() noexcept;

  /**
   * @brief 获取当前蜂鸣器的实际播放状态。
   * @return 最近一次成功操作后对应的停止或播放状态。
   */
  [[nodiscard]] State GetState() const noexcept
  {
    return state_;
  }

  /**
   * @brief 获取最近一次成功设置的播放频率。
   * @return 正在播放时返回频率 Hz，停止时返回 0。
   */
  [[nodiscard]] std::uint32_t GetFrequencyHz() const noexcept
  {
    return frequencyHz_;
  }

  /**
   * @brief 获取最近一次成功设置的音量。
   * @return 正在播放时返回 1 到 255，停止时返回 0。
   */
  [[nodiscard]] std::uint8_t GetVolume() const noexcept
  {
    return volume_;
  }

  /**
   * @brief 获取最近一次播放或停止操作的结果。
   * @return 最近一次操作结果。
   */
  [[nodiscard]] Result GetLastResult() const noexcept
  {
    return lastResult_;
  }

private:
  static constexpr std::uint16_t MaxDutyPermille{500U};

  [[nodiscard]] static std::uint16_t VolumeToDutyPermille(std::uint8_t volume) noexcept;

  State state_{State::Stopped};
  std::uint32_t frequencyHz_{0U};
  std::uint8_t volume_{0U};
  Result lastResult_{Result::NotReady};
};

} // namespace device

#endif
