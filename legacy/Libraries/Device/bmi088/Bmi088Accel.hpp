#ifndef LIBRARIES_DEVICE_BMI088_BMI088_ACCEL_HPP
#define LIBRARIES_DEVICE_BMI088_BMI088_ACCEL_HPP

#include "Libraries/Device/bmi088/Bmi088Transfer.hpp"
#include "Platform/Interface/Time.hpp"
#include <array>
#include <cstddef>
#include <cstdint>

namespace device
{

/**
 * @brief BMI088 加速度计子设备驱动。
 * @note 全部 SPI 事务均通过 DMA 非阻塞接口完成，初始化配置链与数据读取
 *       由单一任务周期调用 Harvest 与 TryStart 推进，任何路径都不阻塞。
 * @note 初始化配置为最大输出速率：ACC_CONF = AccelBandwidthNormal |
 *       AccelOdr1600Hz（0xAC，1600 Hz ODR、normal 带宽组合），量程
 *       AccelRange3g（±3 g），并将高有效 DRDY 映射到 INT1；配置链
 *       完成后逐项读回校验，失败重试后进入 Error 状态。
 */
class Bmi088Accel final
{
public:
  static constexpr std::uint8_t ExpectedChipId{0x1EU};

  // ACC_CONF(0x40)：加速度带宽 acc_bwp[7:4] | 输出速率 acc_odr[3:0]
  static constexpr std::uint8_t AccelBandwidthOsr4{0x00U};
  static constexpr std::uint8_t AccelBandwidthOsr2{0x10U};
  static constexpr std::uint8_t AccelBandwidthNormal{0xA0U};
  static constexpr std::uint8_t AccelOdr12_5Hz{0x05U};
  static constexpr std::uint8_t AccelOdr25Hz{0x06U};
  static constexpr std::uint8_t AccelOdr50Hz{0x07U};
  static constexpr std::uint8_t AccelOdr100Hz{0x08U};
  static constexpr std::uint8_t AccelOdr200Hz{0x09U};
  static constexpr std::uint8_t AccelOdr400Hz{0x0AU};
  static constexpr std::uint8_t AccelOdr800Hz{0x0BU};
  static constexpr std::uint8_t AccelOdr1600Hz{0x0CU};

  // ACC_RANGE(0x41)：量程 acc_range[1:0]
  static constexpr std::uint8_t AccelRange3g{0x00U};
  static constexpr std::uint8_t AccelRange6g{0x01U};
  static constexpr std::uint8_t AccelRange12g{0x02U};
  static constexpr std::uint8_t AccelRange24g{0x03U};

  static constexpr std::uint8_t ConfRegisterValue{static_cast<std::uint8_t>(AccelBandwidthNormal | AccelOdr1600Hz)};
  static constexpr std::uint8_t RangeRegisterValue{AccelRange3g};
  static constexpr float Mps2PerLsb{
      (RangeRegisterValue == AccelRange6g
           ? 6.0F
           : (RangeRegisterValue == AccelRange12g ? 12.0F : (RangeRegisterValue == AccelRange24g ? 24.0F : 3.0F))) *
      9.80665F / 32768.0F};

  enum class State : std::uint8_t
  {
    Shutdown,
    Initializing,
    Ready,
    Error
  };

  struct Sample final
  {
    std::int16_t xAxis{0};
    std::int16_t yAxis{0};
    std::int16_t zAxis{0};
  };

  Bmi088Accel() noexcept = default;

  /**
   * @brief 重置状态机并进入初始化序列，本函数不访问硬件。
   * @note 首个 ID 读取前等待 PowerUpSettleMs；ID 与配置读回校验失败时
   *       按有限次数重试，超过上限才进入 Error。
   */
  void Init() noexcept;

  /**
   * @brief 查询并收割在途异步事务的终态，完成配置步校验或样本解析。
   */
  void Harvest() noexcept;

  /**
   * @brief 在无在途事务且步进等待窗已过时启动下一步 SPI 事务。
   * @note 总线被共享方占用时收到 Busy 并直接返回，下一周期重试。
   */
  // 仅当本次调用启动了新传输（而非既有传输）时为 true。
  bool TryStart() noexcept;

  void SetCompletionNotification(platform::Spi::CompletionNotification callback, void *context) noexcept
  {
    completion_.SetNotification(callback, context);
  }
  [[nodiscard]] std::uint32_t GetCompletionTick() const noexcept
  {
    return completion_.CompletedTick();
  }

  /**
   * @brief 查询初始化与数据链路是否已完成并可提供样本。
   * @return 配置链验证通过且处于周期读取状态时返回 true。
   */
  [[nodiscard]] bool IsReady() const noexcept
  {
    return state_ == State::Ready;
  }

  /**
   * @brief 查询本子设备是否有尚未收割终态的异步事务。
   * @return 存在在途事务时返回 true，供协调器进行总线避让。
   */
  [[nodiscard]] bool HasPendingTransfer() const noexcept
  {
    return transferActive_;
  }

  [[nodiscard]] State GetState() const noexcept
  {
    return state_;
  }

  /**
   * @brief 获取最近一次成功读取的原始三轴加速度计数。
   * @param sample 用于接收原始样本的输出对象。
   * @return 至少完成一次数据读取时返回 true。
   */
  [[nodiscard]] bool TryGetSample(Sample &sample) const noexcept;

  [[nodiscard]] std::uint32_t GetReadCount() const noexcept
  {
    return readCount_;
  }

  [[nodiscard]] std::uint32_t GetErrorCount() const noexcept
  {
    return errorCount_;
  }

  /**
   * @brief 将原始计数换算为物理加速度，单位 m/s^2。
   * @param raw 加速度计原始计数值。
   * @return 按 RangeRegisterValue 所选量程换算的加速度值。
   */
  [[nodiscard]] static float ToMps2(std::int16_t raw) noexcept
  {
    return static_cast<float>(raw) * Mps2PerLsb;
  }

private:
  enum class Step : std::uint8_t
  {
    CheckChipId,
    WritePowerConf,
    WritePowerCtrl,
    WriteRange,
    WriteConf,
    WriteInterruptIo,
    WriteInterruptMap,
    VerifyCoreConfiguration,
    VerifyInterruptIo,
    VerifyInterruptMap,
    ReadData
  };

  static constexpr std::size_t MaximumTransferLength{8U};
  static constexpr std::uint32_t PowerUpSettleMs{100U};
  static constexpr std::uint32_t ChipIdRetryWaitMs{100U};
  static constexpr std::uint32_t ChipIdMaxAttempts{3U};
  static constexpr std::uint32_t ConfigurationVerifyRetryWaitMs{100U};
  static constexpr std::uint32_t ConfigurationVerifyMaxAttempts{3U};

  void BeginStep() noexcept;
  void CompleteStep() noexcept;
  void Advance(Step nextStep, std::uint32_t waitMs) noexcept;
  void Fail() noexcept;
  [[nodiscard]] bool WaitElapsed() const noexcept;

  State state_{State::Shutdown};
  Step step_{Step::CheckChipId};
  platform::Time::Tick stepTick_{0U};
  std::uint32_t stepWaitMs_{0U};
  Bmi088Transfer completion_{};
  bool transferActive_{false};
  bool sampleValid_{false};
  Sample sample_{};
  std::uint32_t readCount_{0U};
  std::uint32_t errorCount_{0U};
  std::uint32_t chipIdAttempts_{0U};
  std::uint32_t configurationVerifyAttempts_{0U};
  std::array<std::uint8_t, MaximumTransferLength> transmitBuffer_{};
  std::array<std::uint8_t, MaximumTransferLength> receiveBuffer_{};
};

} // namespace device

#endif
