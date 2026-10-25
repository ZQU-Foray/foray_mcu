#ifndef LIBRARIES_DEVICE_BMI088_BMI088_GYRO_HPP
#define LIBRARIES_DEVICE_BMI088_BMI088_GYRO_HPP

#include "Libraries/Device/bmi088/Bmi088Transfer.hpp"
#include "Platform/Interface/Time.hpp"
#include <array>
#include <cstddef>
#include <cstdint>

namespace device
{

/**
 * @brief BMI088 陀螺仪子设备驱动。
 * @note 全部 SPI 事务均通过 DMA 非阻塞接口完成，初始化配置链与数据读取
 *       由单一任务周期调用 Harvest 与 TryStart 推进，任何路径都不阻塞。
 * @note 默认使用 2000 Hz ODR / 532 Hz 带宽；构建选项可选择 1000 Hz / 116 Hz
 * 对照测试。 （0x00，2000 Hz ODR、532 Hz 滤波带宽），量程 GyroRange2000Dps
 *       （±2000 °/s），并启用高有效推挽 INT3 DRDY；配置链完成后
 *       逐项读回校验（带宽保留位按掩码忽略），失败重试后进入 Error。
 */
class Bmi088Gyro final
{
public:
  static constexpr std::uint8_t ExpectedChipId{0x0FU};

  // GYRO_RANGE(0x0F)：量程 gyro_range[1:0]
  static constexpr std::uint8_t GyroRange2000Dps{0x00U};
  static constexpr std::uint8_t GyroRange1000Dps{0x01U};
  static constexpr std::uint8_t GyroRange500Dps{0x02U};
  static constexpr std::uint8_t GyroRange250Dps{0x03U};

  // GYRO_BANDWIDTH(0x10)：gyro_bw[3:2] | gyro_odr[1:0]，bit7 保留。
  static constexpr std::uint8_t GyroBandwidth2000Hz532Hz{0x00U};
  static constexpr std::uint8_t GyroBandwidth2000Hz230Hz{0x01U};
  static constexpr std::uint8_t GyroBandwidth1000Hz116Hz{0x02U};
  static constexpr std::uint8_t GyroBandwidth1000Hz47Hz{0x03U};
  static constexpr std::uint8_t GyroBandwidth400Hz47Hz{0x04U};
  static constexpr std::uint8_t GyroBandwidth400Hz23Hz{0x05U};
  static constexpr std::uint8_t GyroBandwidth100Hz12Hz{0x06U};
  static constexpr std::uint8_t GyroBandwidth100Hz6Hz{0x07U};

  static constexpr std::uint8_t RangeRegisterValue{GyroRange2000Dps};
#ifndef BMI088_GYRO_BANDWIDTH_REGISTER
#define BMI088_GYRO_BANDWIDTH_REGISTER 0x00U
#endif
  static constexpr std::uint8_t BandwidthRegisterValue{BMI088_GYRO_BANDWIDTH_REGISTER};
  static_assert(BandwidthRegisterValue == GyroBandwidth2000Hz532Hz ||
                BandwidthRegisterValue == GyroBandwidth1000Hz116Hz);
  static constexpr float DpsPerLsb{
      (RangeRegisterValue == GyroRange1000Dps
           ? 1000.0F
           : (RangeRegisterValue == GyroRange500Dps ? 500.0F
                                                    : (RangeRegisterValue == GyroRange250Dps ? 250.0F : 2000.0F))) /
      32768.0F};

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

  Bmi088Gyro() noexcept = default;

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
   * @brief 获取最近一次成功读取的原始三轴角速度计数。
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
   * @brief 将原始计数换算为物理角速度，单位 °/s。
   * @param raw 陀螺仪原始计数值。
   * @return 按 RangeRegisterValue 所选量程换算的角速度值。
   */
  [[nodiscard]] static float ToDps(std::int16_t raw) noexcept
  {
    return static_cast<float>(raw) * DpsPerLsb;
  }

private:
  enum class Step : std::uint8_t
  {
    CheckChipId,
    WriteRange,
    WriteBandwidth,
    WriteInterruptControl,
    WriteInterruptIo,
    WriteInterruptMap,
    VerifyCoreConfiguration,
    VerifyInterruptControlAndIo,
    VerifyInterruptMap,
    ReadData
  };

  static constexpr std::size_t MaximumTransferLength{7U};
  static constexpr std::uint32_t PowerUpSettleMs{100U};
  static constexpr std::uint32_t ChipIdRetryWaitMs{100U};
  static constexpr std::uint32_t ChipIdMaxAttempts{3U};
  static constexpr std::uint32_t ConfigurationVerifyRetryWaitMs{100U};
  static constexpr std::uint32_t ConfigurationVerifyMaxAttempts{3U};
  static constexpr std::uint8_t RangeRegisterReadMask{0x03U};
  static constexpr std::uint8_t BandwidthRegisterReadMask{0x0FU};

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
