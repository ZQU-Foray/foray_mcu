#include "Libraries/Device/bmi088/Bmi088.hpp"

namespace device
{

namespace
{
// 两个事件的时间间隔必须小于 32 位 tick 回绕周期的一半。
bool Before(std::uint32_t a, std::uint32_t b) noexcept
{
  return ((a - b) & 0x80000000U) != 0U;
}
} // namespace

bool Bmi088::PopNextSample(Bmi088SampleRecord &record, bool &gyroscope) noexcept
{
  Bmi088SampleRecord accel{}, gyro{};
  const bool hasAccel = accelerometerQueue_.Peek(accel);
  const bool hasGyro = gyroscopeQueue_.Peek(gyro);
  if (!hasAccel && !hasGyro)
    return false;
  // 时间戳相同时先交付加速度，使修正恰好落在陀螺区间端点，
  // 而不是滞后一个区间。
  gyroscope = hasGyro && (!hasAccel || Before(gyro.drdyTick, accel.drdyTick));
  const auto candidate = gyroscope ? gyro.drdyTick : accel.drdyTick;
  const auto notLater = [candidate](std::uint32_t tick) { return tick == candidate || Before(tick, candidate); };
  if (accelerometer_.IsReady() && ((accelerometerDataReadyPending_ && notLater(accelerometerEvent_.tick)) ||
                                   (accelerometer_.HasPendingTransfer() && notLater(accelerometerInflight_.drdyTick))))
    return false;
  if (gyroscope_.IsReady() && ((gyroscopeDataReadyPending_ && Before(gyroscopeEvent_.tick, candidate)) ||
                               (gyroscope_.HasPendingTransfer() && Before(gyroscopeInflight_.drdyTick, candidate))))
    return false;
  return gyroscope ? gyroscopeQueue_.Pop(record) : accelerometerQueue_.Pop(record);
}

void Bmi088::Init() noexcept
{
  accelerometer_.Init();
  gyroscope_.Init();
  temperature_.Init();
  accelerometerQueue_.Reset();
  gyroscopeQueue_.Reset();
  accelerometerEvent_ = gyroscopeEvent_ = {};
  accelerometerInflight_ = gyroscopeInflight_ = {};
  accelerometerDataReadyPending_ = gyroscopeDataReadyPending_ = false;
  turn_ = 0U;
  accelerometerDataReadyCount_ = gyroscopeDataReadyCount_ = 0U;
  accelerometerCoalescedEventCount_ = gyroscopeCoalescedEventCount_ = 0U;
  lastTemperatureStartTick_ = platform::Time::NowTicks();
}

void Bmi088::SetCompletionNotification(platform::Spi::CompletionNotification callback, void *context) noexcept
{
  accelerometer_.SetCompletionNotification(callback, context);
  gyroscope_.SetCompletionNotification(callback, context);
  temperature_.SetCompletionNotification(callback, context);
}

template <typename Sensor>
void Bmi088::Publish(Sensor &sensor,
                     std::uint32_t previousReads,
                     Bmi088SampleRecord &inflight,
                     Bmi088DataReady latest,
                     Bmi088SampleQueue<> &queue) noexcept
{
  if (sensor.GetReadCount() == previousReads)
    return;
  typename Sensor::Sample sample{};
  if (!sensor.TryGetSample(sample))
    return;
  inflight.sequence = sensor.GetReadCount();
  inflight.completedTick = sensor.GetCompletionTick();
  inflight.harvestedTick = platform::Time::NowTicks();
  inflight.xyz[0] = sample.xAxis;
  inflight.xyz[1] = sample.yAxis;
  inflight.xyz[2] = sample.zAxis;
  // 传输完成前下一次任务迭代又观测到更晚的边沿时，归属变得
  // 有歧义。这是保守告警，并非丢失的证明。
  if (latest.sequence != inflight.drdySequence &&
      static_cast<std::uint32_t>(latest.tick - inflight.drdyTick) <=
          static_cast<std::uint32_t>(inflight.completedTick - inflight.drdyTick))
  {
    inflight.flags |= Bmi088SampleRecord::NewerEventBeforeCompletion;
  }
  (void)queue.Push(inflight);
}

bool Bmi088::StartAccelerometer() noexcept
{
  if (!accelerometerDataReadyPending_ || temperature_.HasPendingTransfer())
    return false;
  const auto start = platform::Time::NowTicks();
  if (!accelerometer_.TryStart())
    return false;
  accelerometerInflight_ = {};
  accelerometerInflight_.drdySequence = accelerometerEvent_.sequence;
  accelerometerInflight_.drdyTick = accelerometerEvent_.tick;
  accelerometerInflight_.startTick = start;
  if (static_cast<std::uint32_t>(start - accelerometerEvent_.tick) >= platform::Time::TickFrequencyHz() / 1600U)
  {
    accelerometerInflight_.flags |= Bmi088SampleRecord::LateStart;
  }
  accelerometerDataReadyPending_ = false;
  return true;
}

bool Bmi088::StartGyroscope() noexcept
{
  if (!gyroscopeDataReadyPending_)
    return false;
  const auto start = platform::Time::NowTicks();
  if (!gyroscope_.TryStart())
    return false;
  gyroscopeInflight_ = {};
  gyroscopeInflight_.drdySequence = gyroscopeEvent_.sequence;
  gyroscopeInflight_.drdyTick = gyroscopeEvent_.tick;
  gyroscopeInflight_.startTick = start;
  constexpr auto odr = Bmi088Gyro::BandwidthRegisterValue == Bmi088Gyro::GyroBandwidth2000Hz532Hz ? 2000U : 1000U;
  if (static_cast<std::uint32_t>(start - gyroscopeEvent_.tick) >= platform::Time::TickFrequencyHz() / odr)
  {
    gyroscopeInflight_.flags |= Bmi088SampleRecord::LateStart;
  }
  gyroscopeDataReadyPending_ = false;
  return true;
}

void Bmi088::Process() noexcept
{
  const auto accelReads = accelerometer_.GetReadCount();
  const auto gyroReads = gyroscope_.GetReadCount();
  accelerometer_.Harvest();
  gyroscope_.Harvest();
  temperature_.Harvest();
  Publish(accelerometer_, accelReads, accelerometerInflight_, accelerometerEvent_, accelerometerQueue_);
  Publish(gyroscope_, gyroReads, gyroscopeInflight_, gyroscopeEvent_, gyroscopeQueue_);

  if (!temperature_.IsEnabled() && accelerometer_.IsReady())
    temperature_.Enable();

  if (!IsReady())
  {
    // 一侧传感器正在配置时，就绪的另一侧不得执行无触发读取。
    if ((turn_ % 2U) == 0U)
    {
      if (!accelerometer_.IsReady())
        (void)accelerometer_.TryStart();
    }
    else
    {
      if (!gyroscope_.IsReady())
        (void)gyroscope_.TryStart();
    }
    ++turn_;
    return;
  }

  if (platform::Time::HasElapsedMs(lastTemperatureStartTick_, TemperaturePeriodMs) && temperature_.IsEnabled() &&
      !accelerometer_.HasPendingTransfer() && !temperature_.HasPendingTransfer() && temperature_.TryStart())
  {
    lastTemperatureStartTick_ = platform::Time::NowTicks();
  }

  // 按边沿年龄调度：先服务等待更久的挂起边沿，再试另一侧。固定的两侧交替
  // 会让 2000 Hz 的陀螺长期占先，使周期更长（625 us）的加速度计边沿偶发被
  // 合并；改为最老优先后，两路都能在自己的周期内被服务。
  // 首选一侧无法启动时改试另一侧；Busy 或先前传输仍在进行时绝不消费挂起的边沿。
  const std::uint32_t now = platform::Time::NowTicks();
  bool preferAccelerometer = accelerometerDataReadyPending_;
  if (accelerometerDataReadyPending_ && gyroscopeDataReadyPending_)
  {
    preferAccelerometer = static_cast<std::uint32_t>(now - accelerometerEvent_.tick) >=
                          static_cast<std::uint32_t>(now - gyroscopeEvent_.tick);
  }
  if (preferAccelerometer)
  {
    if (!StartAccelerometer())
      (void)StartGyroscope();
  }
  else
  {
    if (!StartGyroscope())
      (void)StartAccelerometer();
  }
  ++turn_;
}

void Bmi088::NotifyDataReady(Bmi088DataReady accelerometer, Bmi088DataReady gyroscope) noexcept
{
  const auto accelEvents = accelerometer.sequence - accelerometerEvent_.sequence;
  const auto gyroEvents = gyroscope.sequence - gyroscopeEvent_.sequence;
  accelerometerDataReadyCount_ += accelEvents;
  gyroscopeDataReadyCount_ += gyroEvents;
  if (accelEvents > 0U)
  {
    accelerometerCoalescedEventCount_ += accelEvents - (accelerometerDataReadyPending_ ? 0U : 1U);
    accelerometerDataReadyPending_ = true;
    accelerometerEvent_ = accelerometer;
  }
  if (gyroEvents > 0U)
  {
    gyroscopeCoalescedEventCount_ += gyroEvents - (gyroscopeDataReadyPending_ ? 0U : 1U);
    gyroscopeDataReadyPending_ = true;
    gyroscopeEvent_ = gyroscope;
  }
}

Bmi088::State Bmi088::GetState() const noexcept
{
  if (accelerometer_.GetState() == Bmi088Accel::State::Error || gyroscope_.GetState() == Bmi088Gyro::State::Error)
    return State::Error;
  return IsReady() ? State::Ready : State::Initializing;
}
} // namespace device
