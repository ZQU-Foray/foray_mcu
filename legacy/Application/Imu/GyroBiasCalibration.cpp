#include "Application/Imu/GyroBiasCalibration.hpp"
#include <cmath>

namespace application
{

namespace
{
constexpr float ReferenceGravityMps2{9.80665F};
constexpr std::uint64_t MillisecondsPerSecond{1000U};
} // namespace

void GyroBiasCalibration::Init(const Config &config) noexcept
{
  config_ = config;
  Reset();
}

void GyroBiasCalibration::Reset() noexcept
{
  state_ = State::WaitingForTemperature;
  bias_ = alg_math::Vector3{};
  spread_ = alg_math::Vector3{};
  RestartWindow();
  restarts_ = 0U; // 必须在 RestartWindow 之后清零，否则复位本身会被计成一次重启
  samplingTicks_ = 0U;
  holdTicks_ = 0U;
  holdMinimum_ = 0.0F;
  holdMaximum_ = 0.0F;
  holdActive_ = false;
  timingAnchored_ = false;
  lastTick_ = 0U;
  temperatureCelsius_ = 0.0F;
  biasValid_ = false;
}

void GyroBiasCalibration::RestartWindow() noexcept
{
  if (count_ != 0U)
  {
    ++restarts_; // 只统计"已经开始积累后被中断"的情况
  }
  sumX_ = 0.0;
  sumY_ = 0.0;
  sumZ_ = 0.0;
  minimum_ = alg_math::Vector3{};
  maximum_ = alg_math::Vector3{};
  count_ = 0U;
  windowTicks_ = 0U;
}

std::uint64_t GyroBiasCalibration::TicksFromMs(std::uint32_t milliseconds) const noexcept
{
  // 不做 32 位饱和：480 MHz 下 20 s 是 9.6e9 tick，远大于 2^32，
  // 饱和成 0xFFFFFFFF 会让"超过时限"的比较永远不成立。
  return static_cast<std::uint64_t>(milliseconds) * static_cast<std::uint64_t>(config_.tickFrequencyHz) /
         MillisecondsPerSecond;
}

void GyroBiasCalibration::Process(const alg_math::Vector3 &gyroRadPerSec,
                                  const alg_math::Vector3 &accelMetersPerSec2,
                                  float temperatureCelsius,
                                  std::uint32_t tick) noexcept
{
  if (state_ == State::Completed || state_ == State::TimedOut)
  {
    return; // 终态不再改变：标定只做一次
  }
  if (config_.tickFrequencyHz == 0U)
  {
    return; // 没有可信时基：既不积累也不判定
  }

  // 建立时基：首个样本只记录起点，不产生时间增量。
  const std::uint32_t deltaTicks = timingAnchored_ ? (tick - lastTick_) : 0U;
  timingAnchored_ = true;
  lastTick_ = tick;
  temperatureCelsius_ = temperatureCelsius;

  // 温度门控：离开容差就清空窗口与稳定保持，并重新开始计时。
  const bool temperatureReady =
      std::isfinite(temperatureCelsius) &&
      std::fabs(temperatureCelsius - config_.targetCelsius) <= config_.temperatureToleranceCelsius;
  if (!temperatureReady)
  {
    if (state_ != State::WaitingForTemperature || count_ != 0U)
    {
      RestartWindow();
    }
    state_ = State::WaitingForTemperature;
    holdActive_ = false;
    holdTicks_ = 0U;
    return;
  }

  // 温度稳定门控：进带后还要保持足够久且极差足够小才允许开窗，
  // 带内极差超限就从当前温度重新开始保持计时。
  if (!holdActive_)
  {
    holdActive_ = true;
    holdTicks_ = 0U;
    holdMinimum_ = temperatureCelsius;
    holdMaximum_ = temperatureCelsius;
  }
  else
  {
    if (temperatureCelsius < holdMinimum_)
    {
      holdMinimum_ = temperatureCelsius;
    }
    if (temperatureCelsius > holdMaximum_)
    {
      holdMaximum_ = temperatureCelsius;
    }
  }
  holdTicks_ += deltaTicks;
  if (holdMaximum_ - holdMinimum_ > config_.temperatureStabilityCelsius)
  {
    holdMinimum_ = temperatureCelsius;
    holdMaximum_ = temperatureCelsius;
    holdTicks_ = 0U;
  }

  if (state_ == State::WaitingForTemperature)
  {
    if (holdTicks_ < TicksFromMs(config_.temperatureSettleMs))
    {
      return; // 温度还没稳定够久：不开窗、不积累样本
    }
    state_ = State::Sampling;
    RestartWindow();
  }

  // 采样预算跨窗口累计：窗口因运动重启不重置预算；温度出带后暂停增长
  // 但保留已用额度，避免"开机后被搬动几秒"就花掉整轮标定机会。
  samplingTicks_ += deltaTicks;
  if (samplingTicks_ >= TicksFromMs(config_.timeoutMs))
  {
    state_ = State::TimedOut;
    biasValid_ = false;
    RestartWindow();
    return;
  }

  // 静止门控：加速度范数必须接近重力。
  const float accelNorm = alg_math::Norm(accelMetersPerSec2);
  if (!alg_math::IsFinite(accelMetersPerSec2) ||
      std::fabs(accelNorm - ReferenceGravityMps2) > config_.maximumAccelNormErrorMps2)
  {
    RestartWindow();
    return;
  }

  Accumulate(gyroRadPerSec, deltaTicks);
}

void GyroBiasCalibration::Accumulate(const alg_math::Vector3 &gyroRadPerSec, std::uint32_t deltaTicks) noexcept
{
  if (!alg_math::IsFinite(gyroRadPerSec))
  {
    RestartWindow();
    return;
  }

  if (count_ == 0U)
  {
    minimum_ = gyroRadPerSec;
    maximum_ = gyroRadPerSec;
  }
  else
  {
    if (gyroRadPerSec.x < minimum_.x)
    {
      minimum_.x = gyroRadPerSec.x;
    }
    if (gyroRadPerSec.y < minimum_.y)
    {
      minimum_.y = gyroRadPerSec.y;
    }
    if (gyroRadPerSec.z < minimum_.z)
    {
      minimum_.z = gyroRadPerSec.z;
    }
    if (gyroRadPerSec.x > maximum_.x)
    {
      maximum_.x = gyroRadPerSec.x;
    }
    if (gyroRadPerSec.y > maximum_.y)
    {
      maximum_.y = gyroRadPerSec.y;
    }
    if (gyroRadPerSec.z > maximum_.z)
    {
      maximum_.z = gyroRadPerSec.z;
    }
  }

  sumX_ += static_cast<double>(gyroRadPerSec.x);
  sumY_ += static_cast<double>(gyroRadPerSec.y);
  sumZ_ += static_cast<double>(gyroRadPerSec.z);
  ++count_;
  windowTicks_ += deltaTicks;

  // 每个样本都检查极差：被碰或振动必须在下一个样本就重启窗口，
  // 否则短促扰动会被记进窗口，只能等到周期检查或收尾时才被剔除。
  const alg_math::Vector3 spread{maximum_.x - minimum_.x, maximum_.y - minimum_.y, maximum_.z - minimum_.z};
  if (spread.x > config_.maximumSpreadRadPerSec || spread.y > config_.maximumSpreadRadPerSec ||
      spread.z > config_.maximumSpreadRadPerSec)
  {
    RestartWindow();
    return;
  }

  // 周期性检查部分均值：抓缓慢转动，代价是与样本数无关的常数开销。
  if ((count_ % PartialCheckInterval) == 0U)
  {
    const double inverse = 1.0 / static_cast<double>(count_);
    const alg_math::Vector3 mean{
        static_cast<float>(sumX_ * inverse), static_cast<float>(sumY_ * inverse), static_cast<float>(sumZ_ * inverse)};
    if (std::fabs(mean.x) > config_.maximumMeanRateRadPerSec || std::fabs(mean.y) > config_.maximumMeanRateRadPerSec ||
        std::fabs(mean.z) > config_.maximumMeanRateRadPerSec)
    {
      RestartWindow();
      return;
    }
  }

  if (windowTicks_ >= TicksFromMs(config_.windowMs))
  {
    Finalize();
  }
}

void GyroBiasCalibration::Finalize() noexcept
{
  if (count_ == 0U)
  {
    RestartWindow();
    return;
  }

  const double inverse = 1.0 / static_cast<double>(count_);
  const alg_math::Vector3 mean{
      static_cast<float>(sumX_ * inverse), static_cast<float>(sumY_ * inverse), static_cast<float>(sumZ_ * inverse)};
  const alg_math::Vector3 spread{maximum_.x - minimum_.x, maximum_.y - minimum_.y, maximum_.z - minimum_.z};

  // 均值超限说明窗口内存在慢速转动，不能当作零偏，重新开始。
  if (!alg_math::IsFinite(mean) || std::fabs(mean.x) > config_.maximumMeanRateRadPerSec ||
      std::fabs(mean.y) > config_.maximumMeanRateRadPerSec || std::fabs(mean.z) > config_.maximumMeanRateRadPerSec ||
      spread.x > config_.maximumSpreadRadPerSec || spread.y > config_.maximumSpreadRadPerSec ||
      spread.z > config_.maximumSpreadRadPerSec)
  {
    RestartWindow();
    return;
  }

  bias_ = mean;
  spread_ = spread;
  biasValid_ = true;
  state_ = State::Completed;
}

GyroBiasCalibration::Snapshot GyroBiasCalibration::GetSnapshot() const noexcept
{
  Snapshot snapshot{};
  snapshot.state = state_;
  snapshot.biasRadPerSec = bias_;
  snapshot.spreadRadPerSec = spread_;
  snapshot.samples = count_;
  snapshot.restarts = restarts_;
  // 时基不可用时不做除零，时长一律报 0。
  const std::uint32_t frequencyHz = config_.tickFrequencyHz;
  if (frequencyHz != 0U)
  {
    snapshot.elapsedMs = static_cast<std::uint32_t>(samplingTicks_ * MillisecondsPerSecond / frequencyHz);
    snapshot.temperatureHoldMs = static_cast<std::uint32_t>(holdTicks_ * MillisecondsPerSecond / frequencyHz);
  }
  snapshot.temperatureCelsius = temperatureCelsius_;
  snapshot.biasValid = biasValid_;
  return snapshot;
}

} // namespace application
