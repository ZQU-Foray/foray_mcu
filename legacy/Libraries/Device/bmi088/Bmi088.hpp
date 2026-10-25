#ifndef LIBRARIES_DEVICE_BMI088_BMI088_HPP
#define LIBRARIES_DEVICE_BMI088_BMI088_HPP

#include "Libraries/Device/bmi088/Bmi088Accel.hpp"
#include "Libraries/Device/bmi088/Bmi088Gyro.hpp"
#include "Libraries/Device/bmi088/Bmi088SampleQueue.hpp"
#include "Libraries/Device/bmi088/Bmi088Temperature.hpp"
#include "Platform/Interface/Time.hpp"
#include <cstdint>

namespace device
{

/**
 * @brief BMI088 模块协调器，负责三个子设备在共享 SPI2 总线上的调度。
 * @note 初始化阶段由任务周期轮转推进；就绪后只消费 DRDY 事件，并在
 *       两路事件同时待处理时交替发放启动权。加速度计与温度共用同一
 *       逻辑 SPI 设备，双方启动前避让对方的在途事务。
 * @note 温度按 40 ms 时间周期读取；数据事件积压时只读取传感器输出
 *       寄存器中的最新样本，并记录无法逐个消费的合并事件数。
 */
class Bmi088 final
{
public:
  static constexpr std::uint32_t TemperaturePeriodMs{40U};

  enum class State : std::uint8_t
  {
    Initializing,
    Ready,
    Error
  };

  Bmi088() noexcept = default;

  /**
   * @brief 重置三个子设备状态机，本函数不访问硬件。
   */
  void Init() noexcept;

  /**
   * @brief 收割全部在途事务，并按初始化状态或 DRDY 事件发放启动许可。
   * @note 仅允许所属任务调用，不得在中断上下文调用。
   */
  void Process() noexcept;

  /**
   * @brief 提交累计 DRDY 序号与最近一次中断时间的一致快照。
   * @param accelerometer 加速度计累计事件快照。
   * @param gyroscope 陀螺仪累计事件快照。
   * @note 多个尚未消费的事件会合并为一次最新数据读取，并计入合并统计。
   */
  void NotifyDataReady(Bmi088DataReady accelerometer, Bmi088DataReady gyroscope) noexcept;

  // 在 Process 启动前设置；转发给全部共享总线事务。
  void SetCompletionNotification(platform::Spi::CompletionNotification callback, void *context) noexcept;
  bool PopGyroscopeSample(Bmi088SampleRecord &record) noexcept
  {
    return gyroscopeQueue_.Pop(record);
  }
  bool PopAccelerometerSample(Bmi088SampleRecord &record) noexcept
  {
    return accelerometerQueue_.Pop(record);
  }
  // 两传感器按时间顺序交付。已知更早的 DRDY 仍挂起/在途时，
  // 较新的记录会被保持，直到较早的传输被收割。
  // 不要在同一应用中把本消费者 API 与单传感器 Pop API 混用。
  bool PopNextSample(Bmi088SampleRecord &record, bool &gyroscope) noexcept;
  const Bmi088SampleQueue<> &GetGyroscopeQueue() const noexcept
  {
    return gyroscopeQueue_;
  }
  const Bmi088SampleQueue<> &GetAccelerometerQueue() const noexcept
  {
    return accelerometerQueue_;
  }

  /**
   * @brief 查询加速度计与陀螺仪是否均已就绪。
   * @return 两个核心子设备均完成配置并进入读取状态时返回 true。
   */
  [[nodiscard]] bool IsReady() const noexcept
  {
    return accelerometer_.IsReady() && gyroscope_.IsReady();
  }

  /**
   * @brief 获取模块聚合状态，温度子设备不计入聚合判定。
   * @return 任一核心子设备错误时返回 Error，全部就绪时返回 Ready。
   */
  [[nodiscard]] State GetState() const noexcept;

  [[nodiscard]] const Bmi088Accel &GetAccelerometer() const noexcept
  {
    return accelerometer_;
  }

  [[nodiscard]] const Bmi088Gyro &GetGyroscope() const noexcept
  {
    return gyroscope_;
  }

  [[nodiscard]] const Bmi088Temperature &GetTemperature() const noexcept
  {
    return temperature_;
  }

  [[nodiscard]] std::uint32_t GetAccelerometerDataReadyCount() const noexcept
  {
    return accelerometerDataReadyCount_;
  }

  [[nodiscard]] std::uint32_t GetGyroscopeDataReadyCount() const noexcept
  {
    return gyroscopeDataReadyCount_;
  }

  [[nodiscard]] std::uint32_t GetAccelerometerCoalescedEventCount() const noexcept
  {
    return accelerometerCoalescedEventCount_;
  }

  [[nodiscard]] std::uint32_t GetGyroscopeCoalescedEventCount() const noexcept
  {
    return gyroscopeCoalescedEventCount_;
  }

private:
  template <typename Sensor>
  void Publish(Sensor &sensor,
               std::uint32_t previousReads,
               Bmi088SampleRecord &inflight,
               Bmi088DataReady latest,
               Bmi088SampleQueue<> &queue) noexcept;
  bool StartAccelerometer() noexcept;
  bool StartGyroscope() noexcept;
  Bmi088DataReady accelerometerEvent_{}, gyroscopeEvent_{};
  Bmi088SampleRecord accelerometerInflight_{}, gyroscopeInflight_{};
  Bmi088SampleQueue<> accelerometerQueue_{}, gyroscopeQueue_{};
  Bmi088Accel accelerometer_{};
  Bmi088Gyro gyroscope_{};
  Bmi088Temperature temperature_{};
  bool accelerometerDataReadyPending_{false};
  bool gyroscopeDataReadyPending_{false};
  std::uint32_t turn_{0U};
  std::uint32_t accelerometerDataReadyCount_{0U};
  std::uint32_t gyroscopeDataReadyCount_{0U};
  std::uint32_t accelerometerCoalescedEventCount_{0U};
  std::uint32_t gyroscopeCoalescedEventCount_{0U};
  platform::Time::Tick lastTemperatureStartTick_{0U};
};

} // namespace device

#endif
