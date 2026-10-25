#include "Libraries/Algorithm/alg_estimate/Vqf.hpp"
#include "Libraries/Algorithm/alg_math/BasicMath.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace alg_estimate
{

namespace
{

constexpr double Epsilon{static_cast<double>(std::numeric_limits<float>::epsilon())};

/**
 * @brief 把 3×3 旋转矩阵写入扁平矩阵。
 */
void SetRotationMatrix(const alg_math::Quaternion &attitude, alg_math::Matrix3 &rotation) noexcept
{
  const float w = attitude.w;
  const float x = attitude.x;
  const float y = attitude.y;
  const float z = attitude.z;
  rotation.value[0] = static_cast<double>(1.0F - 2.0F * y * y - 2.0F * z * z);
  rotation.value[1] = static_cast<double>(2.0F * (y * x - w * z));
  rotation.value[2] = static_cast<double>(2.0F * (w * y + z * x));
  rotation.value[3] = static_cast<double>(2.0F * (w * z + y * x));
  rotation.value[4] = static_cast<double>(1.0F - 2.0F * x * x - 2.0F * z * z);
  rotation.value[5] = static_cast<double>(2.0F * (y * z - x * w));
  rotation.value[6] = static_cast<double>(2.0F * (z * x - w * y));
  rotation.value[7] = static_cast<double>(2.0F * (w * x + z * y));
  rotation.value[8] = static_cast<double>(1.0F - 2.0F * x * x - 2.0F * y * y);
}

bool SameConfig(const Vqf::Config &first, const Vqf::Config &second) noexcept
{
  return first.tauAccSeconds == second.tauAccSeconds && first.restBiasEstEnabled == second.restBiasEstEnabled &&
         first.motionBiasEstEnabled == second.motionBiasEstEnabled &&
         first.biasSigmaInitDegPerSec == second.biasSigmaInitDegPerSec &&
         first.biasForgettingTimeSeconds == second.biasForgettingTimeSeconds &&
         first.biasClipRadPerSec == second.biasClipRadPerSec &&
         first.biasSigmaMotionDegPerSec == second.biasSigmaMotionDegPerSec &&
         first.biasVerticalForgettingFactor == second.biasVerticalForgettingFactor &&
         first.biasSigmaRestDegPerSec == second.biasSigmaRestDegPerSec &&
         first.restMinSeconds == second.restMinSeconds && first.restFilterTauSeconds == second.restFilterTauSeconds &&
         first.restThresholdGyrRadPerSec == second.restThresholdGyrRadPerSec &&
         first.restThresholdAccMetersPerSec2 == second.restThresholdAccMetersPerSec2;
}

} // namespace

bool Vqf::Init(const Config &config, float gyroPeriodSeconds, float accelPeriodSeconds) noexcept
{
  const bool periodsValid = std::isfinite(gyroPeriodSeconds) && std::isfinite(accelPeriodSeconds) &&
                            gyroPeriodSeconds > 0.0F && accelPeriodSeconds > 0.0F;
  const bool timeConstantsValid = std::isfinite(config.tauAccSeconds) && config.tauAccSeconds > 0.0F &&
                                  std::isfinite(config.restFilterTauSeconds) && config.restFilterTauSeconds > 0.0F;
  const bool scalesValid =
      std::isfinite(config.restMinSeconds) && config.restMinSeconds >= 0.0F &&
      std::isfinite(config.biasForgettingTimeSeconds) && config.biasForgettingTimeSeconds > 0.0F &&
      std::isfinite(config.biasSigmaInitDegPerSec) && config.biasSigmaInitDegPerSec > 0.0F &&
      std::isfinite(config.biasSigmaMotionDegPerSec) && config.biasSigmaMotionDegPerSec > 0.0F &&
      std::isfinite(config.biasSigmaRestDegPerSec) && config.biasSigmaRestDegPerSec > 0.0F &&
      std::isfinite(config.biasClipRadPerSec) && config.biasClipRadPerSec > 0.0F &&
      std::isfinite(config.biasVerticalForgettingFactor) && config.biasVerticalForgettingFactor > 0.0F &&
      std::isfinite(config.restThresholdGyrRadPerSec) && config.restThresholdGyrRadPerSec >= 0.0F &&
      std::isfinite(config.restThresholdAccMetersPerSec2) && config.restThresholdAccMetersPerSec2 >= 0.0F;
  if (!periodsValid || !timeConstantsValid || !scalesValid)
  {
    initialized_ = false;
    return false;
  }

  gyroPeriod_ = static_cast<double>(gyroPeriodSeconds);
  accelPeriod_ = static_cast<double>(accelPeriodSeconds);
  restMin_ = static_cast<double>(config.restMinSeconds);
  biasClip_ = static_cast<double>(config.biasClipRadPerSec);
  restThresholdGyr_ = static_cast<double>(config.restThresholdGyrRadPerSec);
  restThresholdAcc_ = static_cast<double>(config.restThresholdAccMetersPerSec2);
  restBiasEstEnabled_ = config.restBiasEstEnabled;
  motionBiasEstEnabled_ = config.motionBiasEstEnabled;

  // 三组低通：加速度计（τ_acc）、静止判定（τ_rest，两路各自周期）、运动零偏估计（τ_acc）。
  if (!accelLowPass_.Init({config.tauAccSeconds, accelPeriodSeconds}) ||
      !restGyroLowPass_.Init({config.restFilterTauSeconds, gyroPeriodSeconds}) ||
      !restAccelLowPass_.Init({config.restFilterTauSeconds, accelPeriodSeconds}) ||
      !motionBiasRotationLowPass_.Init({config.tauAccSeconds, accelPeriodSeconds}) ||
      !motionBiasProjectionLowPass_.Init({config.tauAccSeconds, accelPeriodSeconds}))
  {
    initialized_ = false;
    return false;
  }

  config_ = config;
  // 零偏协方差与过程/量测噪声
  // 该单位约定不影响零偏估计值（增益对 P、V、W 的整体缩放不变），只影响不确定度输出。
  biasP0_ = alg_math::Square(static_cast<double>(config.biasSigmaInitDegPerSec) * 100.0);
  biasV_ = alg_math::Square(0.1 * 100.0) * accelPeriod_ / static_cast<double>(config.biasForgettingTimeSeconds);
  const double pMotion = alg_math::Square(static_cast<double>(config.biasSigmaMotionDegPerSec) * 100.0);
  biasMotionW_ = alg_math::Square(pMotion) / biasV_ + pMotion;
  biasVerticalW_ = biasMotionW_ / (static_cast<double>(config.biasVerticalForgettingFactor) > 1.0e-10
                                       ? static_cast<double>(config.biasVerticalForgettingFactor)
                                       : 1.0e-10);
  const double pRest = alg_math::Square(static_cast<double>(config.biasSigmaRestDegPerSec) * 100.0);
  biasRestW_ = alg_math::Square(pRest) / biasV_ + pRest;

  initialized_ = true;
  Reset();
  return true;
}

void Vqf::Reset() noexcept
{
  gyroQuat_ = alg_math::Quaternion{};
  accelQuat_ = alg_math::Quaternion{};
  lastAccelLp_ = alg_math::Vector3{};
  lastAccelCorrectionRadians_ = 0.0F;
  accelLowPass_.Reset();

  gyroBias_ = alg_math::Vector3{};
  biasCovariance_ = alg_math::ScaledIdentity(biasP0_);
  motionBiasRotationLowPass_.Reset();
  motionBiasProjectionLowPass_.Reset();

  restDetected_ = false;
  restT_ = 0.0;
  restSquaredDeviations_[0] = 0.0;
  restSquaredDeviations_[1] = 0.0;
  restLastGyroLp_ = alg_math::Vector3{};
  restLastAccelLp_ = alg_math::Vector3{};
  restGyroLowPass_.Reset();
  restAccelLowPass_.Reset();

  updateCount_ = 0U;
  valid_ = false;
}

void Vqf::UpdateGyroscope(const alg_math::Vector3 &gyroRadPerSec) noexcept
{
  if (!initialized_ || !alg_math::IsFinite(gyroRadPerSec))
  {
    return;
  }

  const float gyro[3]{gyroRadPerSec.x, gyroRadPerSec.y, gyroRadPerSec.z};

  // 静止判定的陀螺侧：低通后方差超阈值，或低通值本身超过零偏限幅，都判为"运动中"。
  if (restBiasEstEnabled_)
  {
    float gyroLp[3]{};
    restGyroLowPass_.Filter(gyro, gyroLp);
    restLastGyroLp_ = alg_math::Vector3{gyroLp[0], gyroLp[1], gyroLp[2]};

    restSquaredDeviations_[0] = alg_math::Square(static_cast<double>(gyro[0]) - static_cast<double>(gyroLp[0])) +
                                alg_math::Square(static_cast<double>(gyro[1]) - static_cast<double>(gyroLp[1])) +
                                alg_math::Square(static_cast<double>(gyro[2]) - static_cast<double>(gyroLp[2]));

    if (restSquaredDeviations_[0] >= alg_math::Square(restThresholdGyr_) ||
        std::fabs(static_cast<double>(gyroLp[0])) > biasClip_ ||
        std::fabs(static_cast<double>(gyroLp[1])) > biasClip_ || std::fabs(static_cast<double>(gyroLp[2])) > biasClip_)
    {
      restT_ = 0.0;
      restDetected_ = false;
    }
  }

  // 去除估计零偏后做角速度积分。
  const alg_math::Vector3 gyroNoBias{
      gyroRadPerSec.x - gyroBias_.x, gyroRadPerSec.y - gyroBias_.y, gyroRadPerSec.z - gyroBias_.z};
  const double gyroNorm = static_cast<double>(alg_math::Norm(gyroNoBias));
  if (gyroNorm > Epsilon)
  {
    const double angle = gyroNorm * gyroPeriod_;
    const double cosine = std::cos(angle / 2.0);
    const double sine = std::sin(angle / 2.0) / gyroNorm;
    const alg_math::Quaternion step{static_cast<float>(cosine),
                                    static_cast<float>(sine * static_cast<double>(gyroNoBias.x)),
                                    static_cast<float>(sine * static_cast<double>(gyroNoBias.y)),
                                    static_cast<float>(sine * static_cast<double>(gyroNoBias.z))};
    gyroQuat_ = alg_math::Multiply(gyroQuat_, step);
    (void)alg_math::Normalize(gyroQuat_);
  }

  ++updateCount_;
  valid_ = true;
}

void Vqf::UpdateAccelerometer(const alg_math::Vector3 &accelMetersPerSec2) noexcept
{
  if (!initialized_ || !alg_math::IsFinite(accelMetersPerSec2))
  {
    return;
  }
  if (accelMetersPerSec2.x == 0.0F && accelMetersPerSec2.y == 0.0F && accelMetersPerSec2.z == 0.0F)
  {
    return;
  }

  const float accel[3]{accelMetersPerSec2.x, accelMetersPerSec2.y, accelMetersPerSec2.z};

  // 静止判定的加速度侧：这一侧才会把 restDetected 置真（陀螺侧只负责置假）。
  if (restBiasEstEnabled_)
  {
    float accelLp[3]{};
    restAccelLowPass_.Filter(accel, accelLp);
    restLastAccelLp_ = alg_math::Vector3{accelLp[0], accelLp[1], accelLp[2]};

    restSquaredDeviations_[1] = alg_math::Square(static_cast<double>(accel[0]) - static_cast<double>(accelLp[0])) +
                                alg_math::Square(static_cast<double>(accel[1]) - static_cast<double>(accelLp[1])) +
                                alg_math::Square(static_cast<double>(accel[2]) - static_cast<double>(accelLp[2]));

    if (restSquaredDeviations_[1] >= alg_math::Square(restThresholdAcc_))
    {
      restT_ = 0.0;
      restDetected_ = false;
    }
    else
    {
      restT_ += accelPeriod_;
      if (restT_ >= restMin_)
      {
        restDetected_ = true;
      }
    }
  }

  // 先转到惯性系（角速度积分系）再做低通，避免运动方向变化污染重力方向的提取。
  const alg_math::Vector3 accelInertial = alg_math::Rotate(gyroQuat_, accelMetersPerSec2);
  float accelInertialArray[3]{accelInertial.x, accelInertial.y, accelInertial.z};
  accelLowPass_.Filter(accelInertialArray);
  lastAccelLp_ = alg_math::Vector3{accelInertialArray[0], accelInertialArray[1], accelInertialArray[2]};

  // 转到 6 维地球系并归一化，得到重力方向。
  alg_math::Vector3 accelEarth = alg_math::Rotate(accelQuat_, lastAccelLp_);
  if (!alg_math::Normalize(accelEarth))
  {
    return; // 退化输入：不做修正，避免写入非有限姿态
  }

  // 倾角修正：构造把地球系 +z 对齐到测得的重力方向的修正四元数。
  alg_math::Quaternion correction{};
  const float halfW = std::sqrt((accelEarth.z + 1.0F) / 2.0F);
  if (halfW > 1.0e-6F)
  {
    correction.w = halfW;
    correction.x = 0.5F * accelEarth.y / halfW;
    correction.y = -0.5F * accelEarth.x / halfW;
    correction.z = 0.0F;
  }
  else
  {
    // 测得的重力方向接近 -z（修正角接近 180°）时用 180° 绕 x 轴的等价表示，避免数值问题。
    correction = alg_math::Quaternion{0.0F, 1.0F, 0.0F, 0.0F};
  }
  accelQuat_ = alg_math::Multiply(correction, accelQuat_);
  (void)alg_math::Normalize(accelQuat_);

  // 诊断量：本次倾角修正的转角。每个加速度计样本一次 acos（约 0.1% CPU 量级），
  // 上游把它除以采样周期当作"修正角速度"，这里只暴露转角，换算交给调用方。
  const double accelEarthZ =
      static_cast<double>(accelEarth.z) > 1.0
          ? 1.0
          : (static_cast<double>(accelEarth.z) < -1.0 ? -1.0 : static_cast<double>(accelEarth.z));
  lastAccelCorrectionRadians_ = std::acos(accelEarthZ);

  if (!restBiasEstEnabled_ && !motionBiasEstEnabled_)
  {
    return;
  }

  // 零偏估计（3×3 协方差的上推式卡尔曼更新，静止更新与运动更新二选一）
  alg_math::Matrix3 rotation{};
  SetRotationMatrix(GetQuaternion(), rotation);

  // R·b 的 x/y 分量（z 分量不参与水平修正）。
  double biasProjection[2]{
      rotation.value[0] * gyroBias_.x + rotation.value[1] * gyroBias_.y + rotation.value[2] * gyroBias_.z,
      rotation.value[3] * gyroBias_.x + rotation.value[4] * gyroBias_.y + rotation.value[5] * gyroBias_.z};

  float rotationLp[9];
  float biasProjectionLp[2];
  for (std::size_t index = 0U; index < 9U; ++index)
  {
    rotationLp[index] = static_cast<float>(rotation.value[index]);
  }
  biasProjectionLp[0] = static_cast<float>(biasProjection[0]);
  biasProjectionLp[1] = static_cast<float>(biasProjection[1]);
  motionBiasRotationLowPass_.Filter(rotationLp);
  motionBiasProjectionLowPass_.Filter(biasProjectionLp);

  alg_math::Matrix3 rotationLowPassed{};
  for (std::size_t index = 0U; index < 9U; ++index)
  {
    rotationLowPassed.value[index] = static_cast<double>(rotationLp[index]);
  }
  double biasProjectionLowPassed[2]{static_cast<double>(biasProjectionLp[0]), static_cast<double>(biasProjectionLp[1])};

  double innovation[3]{};
  double variance[3]{};
  if (restDetected_ && restBiasEstEnabled_)
  {
    innovation[0] = static_cast<double>(restLastGyroLp_.x) - gyroBias_.x;
    innovation[1] = static_cast<double>(restLastGyroLp_.y) - gyroBias_.y;
    innovation[2] = static_cast<double>(restLastGyroLp_.z) - gyroBias_.z;
    rotationLowPassed = alg_math::ScaledIdentity(1.0);
    variance[0] = biasRestW_;
    variance[1] = biasRestW_;
    variance[2] = biasRestW_;
  }
  else if (motionBiasEstEnabled_)
  {
    innovation[0] = -static_cast<double>(accelEarth.y) / accelPeriod_ + biasProjectionLowPassed[0] -
                    rotationLowPassed.value[0] * gyroBias_.x - rotationLowPassed.value[1] * gyroBias_.y -
                    rotationLowPassed.value[2] * gyroBias_.z;
    innovation[1] = static_cast<double>(accelEarth.x) / accelPeriod_ + biasProjectionLowPassed[1] -
                    rotationLowPassed.value[3] * gyroBias_.x - rotationLowPassed.value[4] * gyroBias_.y -
                    rotationLowPassed.value[5] * gyroBias_.z;
    innovation[2] = -rotationLowPassed.value[6] * gyroBias_.x - rotationLowPassed.value[7] * gyroBias_.y -
                    rotationLowPassed.value[8] * gyroBias_.z;
    variance[0] = biasMotionW_;
    variance[1] = biasMotionW_;
    variance[2] = biasVerticalW_;
  }
  else
  {
    variance[0] = -1.0; // 两个更新都关闭：只做协方差增长，不做量测更新
  }

  // 预测步：协方差只增不减，涨到初值上限为止。
  if (biasCovariance_.value[0] < biasP0_)
  {
    biasCovariance_.value[0] += biasV_;
  }
  if (biasCovariance_.value[4] < biasP0_)
  {
    biasCovariance_.value[4] += biasV_;
  }
  if (biasCovariance_.value[8] < biasP0_)
  {
    biasCovariance_.value[8] += biasV_;
  }

  if (variance[0] >= 0.0)
  {
    // 新息限幅：等价 2 deg/s，同时限制首次倾角修正带来的冲击。
    for (std::size_t index = 0U; index < 3U; ++index)
    {
      innovation[index] = alg_math::Limit(innovation[index], -biasClip_, biasClip_);
    }

    // K = P R^T inv(W + R P R^T)
    const alg_math::Matrix3 gainTransposed = alg_math::MultiplyTransposedSecond(biasCovariance_, rotationLowPassed);
    alg_math::Matrix3 innovationCovariance = alg_math::Multiply(rotationLowPassed, gainTransposed);
    innovationCovariance.value[0] += variance[0];
    innovationCovariance.value[4] += variance[1];
    innovationCovariance.value[8] += variance[2];
    if (!alg_math::Inverted(innovationCovariance, innovationCovariance))
    {
      return; // 奇异：本拍不做零偏更新
    }
    const alg_math::Matrix3 gainBase =
        alg_math::MultiplyTransposedFirst(rotationLowPassed, innovationCovariance); // R^T inv(...)
    const alg_math::Matrix3 gain = alg_math::Multiply(biasCovariance_, gainBase);   // P R^T inv(...)

    gyroBias_.x += static_cast<float>(gain.value[0] * innovation[0] + gain.value[1] * innovation[1] +
                                      gain.value[2] * innovation[2]);
    gyroBias_.y += static_cast<float>(gain.value[3] * innovation[0] + gain.value[4] * innovation[1] +
                                      gain.value[5] * innovation[2]);
    gyroBias_.z += static_cast<float>(gain.value[6] * innovation[0] + gain.value[7] * innovation[1] +
                                      gain.value[8] * innovation[2]);

    // P = P - K R P
    const alg_math::Matrix3 update = alg_math::Multiply(alg_math::Multiply(gain, rotationLowPassed), biasCovariance_);
    for (std::size_t index = 0U; index < 9U; ++index)
    {
      biasCovariance_.value[index] -= update.value[index];
    }

    double biasArray[3]{
        static_cast<double>(gyroBias_.x), static_cast<double>(gyroBias_.y), static_cast<double>(gyroBias_.z)};
    for (std::size_t index = 0U; index < 3U; ++index)
    {
      biasArray[index] = alg_math::Limit(biasArray[index], -biasClip_, biasClip_);
    }
    gyroBias_ = alg_math::Vector3{
        static_cast<float>(biasArray[0]), static_cast<float>(biasArray[1]), static_cast<float>(biasArray[2])};
  }
}

void Vqf::Update(const alg_math::Vector3 &gyroRadPerSec, const alg_math::Vector3 &accelMetersPerSec2) noexcept
{
  UpdateGyroscope(gyroRadPerSec);
  UpdateAccelerometer(accelMetersPerSec2);
}

alg_math::Quaternion Vqf::GetQuaternion() const noexcept
{
  return alg_math::Multiply(accelQuat_, gyroQuat_);
}

Vqf::RestObservables Vqf::GetRestObservables() const noexcept
{
  // 因此 1.0 正好是判据边界，便于直接与阈值对照。
  const auto ratio = [](double squaredDeviation, double threshold) noexcept
  {
    if (!(threshold > 0.0))
    {
      return std::numeric_limits<float>::infinity(); // 阈值为 0：永判为运动，如实上报
    }
    return static_cast<float>(std::sqrt(squaredDeviation) / threshold);
  };

  RestObservables observables{};
  observables.gyroscopeDeviationRatio = ratio(restSquaredDeviations_[0], restThresholdGyr_);
  observables.accelerometerDeviationRatio = ratio(restSquaredDeviations_[1], restThresholdAcc_);
  observables.accumulatedRestSeconds = static_cast<float>(restT_);
  observables.restDetected = restDetected_;
  return observables;
}

void Vqf::SetBiasEstimate(const alg_math::Vector3 &biasRadPerSec, float uncertaintyRadPerSec) noexcept
{
  if (!initialized_ || !alg_math::IsFinite(biasRadPerSec))
  {
    return;
  }

  gyroBias_ = biasRadPerSec;

  // σ ≤ 0 或非有限 → 只改零偏，保留现有协方差
  if (std::isfinite(uncertaintyRadPerSec) && uncertaintyRadPerSec > 0.0F)
  {
    const double scaled = static_cast<double>(uncertaintyRadPerSec) * 180.0 * 100.0 / static_cast<double>(alg_math::pi);
    biasCovariance_ = alg_math::ScaledIdentity(alg_math::Square(scaled));
  }
}

Vqf::InternalState Vqf::GetInternalState() const noexcept
{
  InternalState state{};
  state.config = config_;
  state.gyroPeriodSeconds = gyroPeriod_;
  state.accelPeriodSeconds = accelPeriod_;
  state.gyroQuaternion = gyroQuat_;
  state.accelQuaternion = accelQuat_;
  state.lastAccelLp = lastAccelLp_;
  state.lastAccelCorrectionRadians = lastAccelCorrectionRadians_;
  state.accelLowPass = accelLowPass_.GetInternalState();
  state.gyroBiasRadPerSec = gyroBias_;
  state.biasCovariance = biasCovariance_;
  state.motionBiasRotationLowPass = motionBiasRotationLowPass_.GetInternalState();
  state.motionBiasProjectionLowPass = motionBiasProjectionLowPass_.GetInternalState();
  state.restDetected = restDetected_;
  state.restSeconds = restT_;
  state.restSquaredDeviations[0] = restSquaredDeviations_[0];
  state.restSquaredDeviations[1] = restSquaredDeviations_[1];
  state.restLastGyroLp = restLastGyroLp_;
  state.restLastAccelLp = restLastAccelLp_;
  state.restGyroLowPass = restGyroLowPass_.GetInternalState();
  state.restAccelLowPass = restAccelLowPass_.GetInternalState();
  state.updateCount = updateCount_;
  state.valid = valid_;
  return state;
}

bool Vqf::SetInternalState(const InternalState &state) noexcept
{
  if (!initialized_)
  {
    return false;
  }

  // 两阶段恢复：先整体校验（不写任何状态），全部通过后再写入，
  // 避免出现"部分子状态已更新、其余仍是旧值"的混合态。
  if (!SameConfig(state.config, config_) || state.gyroPeriodSeconds != gyroPeriod_ ||
      state.accelPeriodSeconds != accelPeriod_ || !accelLowPass_.IsInternalStateCompatible(state.accelLowPass) ||
      !motionBiasRotationLowPass_.IsInternalStateCompatible(state.motionBiasRotationLowPass) ||
      !motionBiasProjectionLowPass_.IsInternalStateCompatible(state.motionBiasProjectionLowPass) ||
      !restGyroLowPass_.IsInternalStateCompatible(state.restGyroLowPass) ||
      !restAccelLowPass_.IsInternalStateCompatible(state.restAccelLowPass))
  {
    return false;
  }

  (void)accelLowPass_.SetInternalState(state.accelLowPass);
  (void)motionBiasRotationLowPass_.SetInternalState(state.motionBiasRotationLowPass);
  (void)motionBiasProjectionLowPass_.SetInternalState(state.motionBiasProjectionLowPass);
  (void)restGyroLowPass_.SetInternalState(state.restGyroLowPass);
  (void)restAccelLowPass_.SetInternalState(state.restAccelLowPass);

  gyroQuat_ = state.gyroQuaternion;
  accelQuat_ = state.accelQuaternion;
  lastAccelLp_ = state.lastAccelLp;
  lastAccelCorrectionRadians_ = state.lastAccelCorrectionRadians;
  gyroBias_ = state.gyroBiasRadPerSec;
  biasCovariance_ = state.biasCovariance;
  restDetected_ = state.restDetected;
  restT_ = state.restSeconds;
  restSquaredDeviations_[0] = state.restSquaredDeviations[0];
  restSquaredDeviations_[1] = state.restSquaredDeviations[1];
  restLastGyroLp_ = state.restLastGyroLp;
  restLastAccelLp_ = state.restLastAccelLp;
  updateCount_ = state.updateCount;
  valid_ = state.valid;
  return true;
}

Vqf::State Vqf::GetState() const noexcept
{
  State state{};
  state.quaternion = GetQuaternion();
  state.gyroBiasRadPerSec = gyroBias_;
  state.eulerRadians = alg_math::ToEulerZyxRadians(state.quaternion);
  state.eulerDegrees = alg_math::Vector3{alg_math::RadToDeg(state.eulerRadians.x),
                                         alg_math::RadToDeg(state.eulerRadians.y),
                                         alg_math::RadToDeg(state.eulerRadians.z)};
  state.lastAccelCorrectionRadians = lastAccelCorrectionRadians_;
  state.restDetected = restDetected_;
  state.valid = valid_;
  state.updateCount = updateCount_;

  // 零偏不确定度：用最大绝对行和作为最大特征值的上界（Gershgorin）
  // 从 0.01 deg/s 换算到 rad/s。
  const double row1 =
      std::fabs(biasCovariance_.value[0]) + std::fabs(biasCovariance_.value[1]) + std::fabs(biasCovariance_.value[2]);
  const double row2 =
      std::fabs(biasCovariance_.value[3]) + std::fabs(biasCovariance_.value[4]) + std::fabs(biasCovariance_.value[5]);
  const double row3 =
      std::fabs(biasCovariance_.value[6]) + std::fabs(biasCovariance_.value[7]) + std::fabs(biasCovariance_.value[8]);
  const double bound = std::min(std::max(std::max(row1, row2), row3), biasP0_);
  state.biasUncertaintyRadPerSec =
      static_cast<float>(std::sqrt(bound) * static_cast<double>(alg_math::pi) / 100.0 / 180.0);
  return state;
}

} // namespace alg_estimate
