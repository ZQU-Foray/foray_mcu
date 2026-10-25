#include "Application/Imu/ZeroRateBiasObserver.hpp"
#include <cmath>

namespace application
{

namespace
{
constexpr float ReferenceGravityMps2{9.80665F};
constexpr double MillisecondsPerSecond{1000.0};
} // namespace

void ZeroRateBiasObserver::Init(const Config &config) noexcept
{
  config_ = config;
  Reset();
}

void ZeroRateBiasObserver::Reset() noexcept
{
  biasRadPerSec_ = 0.0F;
  variance_ = config_.initialVarianceRadPerSec2;
  processNoisePerTick_ = config_.tickFrequencyHz == 0U
                             ? 0.0F
                             : config_.processNoiseRadPerSec2PerSec / static_cast<float>(config_.tickFrequencyHz);
  for (int axis = 0; axis < 3; ++axis)
  {
    sum_[axis] = 0.0;
    sumSquares_[axis] = 0.0;
    blockSum_[axis] = 0.0F;
    blockSumSquares_[axis] = 0.0F;
    windowMean_[axis] = 0.0F;
    windowStd_[axis] = 0.0F;
  }
  accelNormSum_ = 0.0;
  accelNormSumSquares_ = 0.0;
  blockAccelSum_ = 0.0F;
  blockAccelSumSquares_ = 0.0F;
  accelNormMean_ = 0.0F;
  accelNormStd_ = 0.0F;
  count_ = 0U;
  lastTick_ = 0U;
  qualifiedWindows_ = 0U;
  observationCount_ = 0U;
  rejectedWindows_ = 0U;
  windowTicks_ = 0U;
  staticTicks_ = 0U;
  windowValid_ = true;
  staticQualified_ = false;
  timingAnchored_ = false;
}

std::uint64_t ZeroRateBiasObserver::TicksFromMs(std::uint32_t ms) const noexcept
{
  // 不做 32 位饱和：480 MHz 下超过 8.9 s 的时长无法用 32 位差值表示。
  return static_cast<std::uint64_t>(ms) * static_cast<std::uint64_t>(config_.tickFrequencyHz) / 1000U;
}

void ZeroRateBiasObserver::FoldBlock() noexcept
{
  for (int axis = 0; axis < 3; ++axis)
  {
    sum_[axis] += static_cast<double>(blockSum_[axis]);
    sumSquares_[axis] += static_cast<double>(blockSumSquares_[axis]);
    blockSum_[axis] = 0.0F;
    blockSumSquares_[axis] = 0.0F;
  }
  accelNormSum_ += static_cast<double>(blockAccelSum_);
  accelNormSumSquares_ += static_cast<double>(blockAccelSumSquares_);
  blockAccelSum_ = 0.0F;
  blockAccelSumSquares_ = 0.0F;
}

void ZeroRateBiasObserver::Process(const alg_math::Vector3 &gyroRadPerSec,
                                   const alg_math::Vector3 &accelMetersPerSec2,
                                   std::uint32_t tick) noexcept
{
  if (config_.windowSamples == 0U || config_.tickFrequencyHz == 0U)
  {
    return; // 参数或时基不可用：既不统计也不判定
  }

  // 建立时基：首个样本只记录起点，不产生时间增量。
  const std::uint32_t deltaTicks = timingAnchored_ ? (tick - lastTick_) : 0U;
  timingAnchored_ = true;
  lastTick_ = tick;
  windowTicks_ += deltaTicks;
  // 过程噪声（预测步）：零偏随机游走，与是否静止无关；
  // 运动期间没有观测，协方差就只长不消，回到静止后增益自然变大、重新收敛。
  variance_ += processNoisePerTick_ * static_cast<float>(deltaTicks);
  if (variance_ > config_.maximumVarianceRadPerSec2)
  {
    variance_ = config_.maximumVarianceRadPerSec2;
  }

  if (alg_math::IsFinite(gyroRadPerSec) && alg_math::IsFinite(accelMetersPerSec2))
  {
    // 热路径只用单精度累加当前子块；双精度总量在子块满时才更新。
    const float axes[3]{gyroRadPerSec.x, gyroRadPerSec.y, gyroRadPerSec.z};
    for (int axis = 0; axis < 3; ++axis)
    {
      blockSum_[axis] += axes[axis];
      blockSumSquares_[axis] += axes[axis] * axes[axis];
      // 撞击/异常样本让整窗作废：均值门挡不住它，方差门又可能被单点带偏。
      if (std::fabs(axes[axis]) > config_.maximumInstantaneousRateRadPerSec)
      {
        windowValid_ = false;
      }
    }
    const float accelNorm = alg_math::Norm(accelMetersPerSec2);
    blockAccelSum_ += accelNorm;
    blockAccelSumSquares_ += accelNorm * accelNorm;
  }
  else
  {
    windowValid_ = false;
  }

  ++count_;
  const std::uint32_t blockSamples = config_.subBlockSamples == 0U ? 1U : config_.subBlockSamples;
  if ((count_ % blockSamples) == 0U)
  {
    FoldBlock();
  }
  if (count_ >= config_.windowSamples)
  {
    FinishWindow();
  }
}

void ZeroRateBiasObserver::FinishWindow() noexcept
{
  // 收尾：把可能不满一个子块的尾料并入总量，保证统计量覆盖全部 count_ 个样本。
  FoldBlock();
  const double inverse = count_ == 0U ? 0.0 : 1.0 / static_cast<double>(count_);

  for (int axis = 0; axis < 3; ++axis)
  {
    const double mean = sum_[axis] * inverse;
    const double variance = sumSquares_[axis] * inverse - mean * mean;
    windowMean_[axis] = static_cast<float>(mean);
    windowStd_[axis] = static_cast<float>(std::sqrt(variance > 0.0 ? variance : 0.0));
  }
  accelNormMean_ = static_cast<float>(accelNormSum_ * inverse);
  const double accelVariance =
      accelNormSumSquares_ * inverse - static_cast<double>(accelNormMean_) * static_cast<double>(accelNormMean_);
  accelNormStd_ = static_cast<float>(std::sqrt(accelVariance > 0.0 ? accelVariance : 0.0));

  bool qualified = windowValid_ && count_ > 0U;
  // 严格均值门只作用于被观测的 z 轴：x/y 的零偏可以远大于该门槛（板上实测
  // y 轴 0.52 °/s），把它们一起判会把所有窗口都拒掉；而慢速倾斜并不影响
  // z 轴零偏的可观测性，只需一个宽松上限排除大幅翻滚。
  if (std::fabs(windowMean_[2]) > config_.maximumMeanRateRadPerSec)
  {
    qualified = false;
  }
  if (std::fabs(windowMean_[0]) > config_.maximumCrossAxisMeanRateRadPerSec ||
      std::fabs(windowMean_[1]) > config_.maximumCrossAxisMeanRateRadPerSec)
  {
    qualified = false;
  }
  for (int axis = 0; axis < 3; ++axis)
  {
    if (windowStd_[axis] > config_.maximumStandardDeviationRadPerSec)
    {
      qualified = false;
    }
  }
  if (std::fabs(accelNormMean_ - ReferenceGravityMps2) > config_.maximumAccelNormErrorMps2)
  {
    qualified = false;
  }
  if (accelNormStd_ > config_.maximumAccelStandardDeviationMps2)
  {
    qualified = false;
  }
  staticQualified_ = qualified;

  if (qualified)
  {
    ++qualifiedWindows_;
    staticTicks_ += windowTicks_;
    // 连续静止够久才允许修正；此前只统计，避免被瞬态骗到。
    if (staticTicks_ >= TicksFromMs(config_.confirmMs) && config_.applyCorrection)
    {
      ApplyZeroRateObservation();
    }
  }
  else
  {
    ++rejectedWindows_;
    staticTicks_ = 0U;
  }

  for (int axis = 0; axis < 3; ++axis)
  {
    sum_[axis] = 0.0;
    sumSquares_[axis] = 0.0;
  }
  accelNormSum_ = 0.0;
  accelNormSumSquares_ = 0.0;
  count_ = 0U;
  windowTicks_ = 0U;
  windowValid_ = true;
}
void ZeroRateBiasObserver::ApplyZeroRateObservation() noexcept
{
  // 量测：静止时窗口内的 z 均值应等于零偏本身。
  // 量测方差取"窗口均值"的方差，而不是单样本方差。
  float measureVariance = (windowStd_[2] * windowStd_[2]) / static_cast<float>(config_.windowSamples);
  if (measureVariance < config_.minimumMeasureVarianceRadPerSec2)
  {
    measureVariance = config_.minimumMeasureVarianceRadPerSec2;
  }

  const float gain = variance_ / (variance_ + measureVariance);
  float correction = gain * (windowMean_[2] - biasRadPerSec_);
  // 限幅：即使某个窗口被误判为静止，也只能把 trim 拉动有限一步。
  if (correction > config_.maximumCorrectionStepRadPerSec)
  {
    correction = config_.maximumCorrectionStepRadPerSec;
  }
  else if (correction < -config_.maximumCorrectionStepRadPerSec)
  {
    correction = -config_.maximumCorrectionStepRadPerSec;
  }
  biasRadPerSec_ += correction;
  variance_ = (1.0F - gain) * variance_;

  ++observationCount_;
}

ZeroRateBiasObserver::Snapshot ZeroRateBiasObserver::GetSnapshot() const noexcept
{
  Snapshot snapshot{};
  snapshot.staticQualified = staticQualified_;
  snapshot.correctionApplied = IsCorrectionApplied();
  snapshot.biasRadPerSec = biasRadPerSec_;
  snapshot.varianceRadPerSec2 = variance_;
  for (int axis = 0; axis < 3; ++axis)
  {
    snapshot.windowMeanRadPerSec[axis] = windowMean_[axis];
    snapshot.windowStdRadPerSec[axis] = windowStd_[axis];
  }
  snapshot.accelNormMeanMps2 = accelNormMean_;
  snapshot.accelNormStdMps2 = accelNormStd_;
  if (config_.tickFrequencyHz != 0U)
  {
    snapshot.staticMs = static_cast<std::uint32_t>(staticTicks_ * 1000U / config_.tickFrequencyHz);
  }
  snapshot.qualifiedWindows = qualifiedWindows_;
  snapshot.observationCount = observationCount_;
  snapshot.rejectedWindows = rejectedWindows_;
  return snapshot;
}

} // namespace application
