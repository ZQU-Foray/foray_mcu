#include "Libraries/Algorithm/alg_estimate/QuaternionEkf.hpp"
#include "Libraries/Algorithm/alg_math/BasicMath.h"
#include <cmath>
#include <cstddef>

namespace alg_estimate
{

namespace
{
// 以下门限与比例取自算法前身的固定取值，移植时保持数值不变。
constexpr float ChiSquareThreshold{1.0e-8F};            // 卡方检验门限
constexpr float ConvergenceThresholdRatio{0.5F};        // 低于该比例的统计量判定为已收敛
constexpr float AdaptiveThresholdRatio{0.1F};           // 自适应增益的启动比例
constexpr float AdaptiveSpanRatio{0.9F};                // 自适应增益的线性区间比例
constexpr std::uint32_t DivergenceErrorLimit{50U};      // 连续未通过次数达到该值判定为发散
constexpr float StableGyroNorm{0.3F};                   // 静止判据：角速度范数上限，rad/s
constexpr float ReferenceGravity{9.8F};                 // 静止判据：重力范数中心值，m/s^2
constexpr float StableGravityTolerance{0.5F};           // 静止判据：重力范数容差，m/s^2
constexpr float BiasCorrectionRateLimit{1.0e-2F};       // 收敛后零偏增量限幅，每秒
constexpr float BiasVarianceLimit{1.0e4F};              // 零偏方差限幅，防止发散
constexpr float OrientationCosineReference{1.5707963F}; // 方向余弦归一化参考，约 π/2

/** 世界系重力方向：静止时机体加速度计测得的比力方向与该方向一致。 */
constexpr alg_math::Vector3 WorldGravityDirection{0.0F, 0.0F, 1.0F};
} // namespace

bool QuaternionEkf::Init(const Config &config) noexcept
{
    const bool valid = std::isfinite(config.processNoiseQuaternion) && config.processNoiseQuaternion > 0.0F &&
                       std::isfinite(config.processNoiseBias) && config.processNoiseBias > 0.0F &&
                       std::isfinite(config.measureNoise) && config.measureNoise > 0.0F &&
                       std::isfinite(config.fadingFactor) && config.fadingFactor > 0.0F &&
                       std::isfinite(config.initialQuaternionVariance) && config.initialQuaternionVariance > 0.0F &&
                       std::isfinite(config.initialBiasVariance) && config.initialBiasVariance > 0.0F &&
                       std::isfinite(config.initialCrossVariance) && config.initialCrossVariance >= 0.0F;
    if (!valid)
    {
        initialized_ = false;
        return false;
    }

    config_ = config;
    initialized_ = true;
    Reset();
    return true;
}

void QuaternionEkf::Reset() noexcept
{
    filter_.Reset();

    // 后验状态由自定义校正步骤完成，其余标准步骤保持启用：增益步骤提供 S⁻¹ 与 K，
    // 协方差步骤消费被自适应缩放后的 K。
    filter_.SetSkipStep(4U, true);
    filter_.SetUserStep(1U, LinearizationStep, this);
    filter_.SetUserStep(2U, ObservationStep, this);
    filter_.SetUserStep(3U, CorrectionStep, this);

    // 初值：单位四元数、零零偏、四元数与零偏各占对角块，非对角项为固定小量。
    Filter::StateVector initialState{};
    initialState.At(StateQuaternionIndex, 0U) = 1.0F;
    filter_.state = initialState;
    for (std::size_t row = 0U; row < Filter::StateSize; ++row)
    {
        for (std::size_t column = 0U; column < Filter::StateSize; ++column)
        {
            filter_.covariance.At(row, column) = config_.initialCrossVariance;
        }
    }
    for (std::size_t index = 0U; index < 4U; ++index)
    {
        filter_.covariance.At(index, index) = config_.initialQuaternionVariance;
    }
    filter_.covariance.At(StateBiasXIndex, StateBiasXIndex) = config_.initialBiasVariance;
    filter_.covariance.At(StateBiasYIndex, StateBiasYIndex) = config_.initialBiasVariance;

    // 量测噪声在整个运行期是常值，只在复位时写入，热路径不重复写。
    Filter::MeasureMatrix measurementNoise{};
    for (std::size_t index = 0U; index < Filter::MeasureSize; ++index)
    {
        measurementNoise.At(index, index) = config_.measureNoise;
    }
    filter_.measurementNoise = measurementNoise;

    converged_ = false;
    stable_ = false;
    chiSquare_ = 0.0F;
    adaptiveGainScale_ = 1.0F;
    errorCount_ = 0U;
    divergenceCount_ = 0U;
    updateCount_ = 0U;
    yawRoundCount_ = 0;
    yawAngleLast_ = 0.0F;
    orientationCosine_[0] = 0.0F;
    orientationCosine_[1] = 0.0F;
    orientationCosine_[2] = 0.0F;
    stepSeconds_ = 0.0F;

    PublishState();
}

void QuaternionEkf::Update(const alg_math::Vector3 &gyroRadPerSec,
                           const alg_math::Vector3 &accelMetersPerSec2,
                           float dtSeconds) noexcept
{
    if (!initialized_ || !std::isfinite(dtSeconds) || !(dtSeconds > 0.0F) || !alg_math::IsFinite(gyroRadPerSec) ||
        !alg_math::IsFinite(accelMetersPerSec2))
    {
        return;
    }

    // 加速度方向作为重力观测向量；范数用于静止判据。
    alg_math::Vector3 gravityDirection = accelMetersPerSec2;
    const float accelNorm = alg_math::Norm(gravityDirection);
    if (!alg_math::Normalize(gravityDirection))
    {
        return;
    }

    // 静止判据使用上一轮后验零偏修正后的角速度范数。
    const float correctedNorm =
        alg_math::Norm(alg_math::Vector3{gyroRadPerSec.x - filter_.state.At(StateBiasXIndex, 0U),
                                         gyroRadPerSec.y - filter_.state.At(StateBiasYIndex, 0U),
                                         gyroRadPerSec.z});
    stable_ = (correctedNorm < StableGyroNorm) && (accelNorm > ReferenceGravity - StableGravityTolerance) &&
              (accelNorm < ReferenceGravity + StableGravityTolerance);

    filter_.measurementInput.At(0U, 0U) = gravityDirection.x;
    filter_.measurementInput.At(1U, 0U) = gravityDirection.y;
    filter_.measurementInput.At(2U, 0U) = gravityDirection.z;

    filter_.SetSkipStep(3U, false); // 增益步骤提供新息协方差逆与增益，供卡方检验与校正使用
    PreparePrediction(gyroRadPerSec, dtSeconds);
    correcting_ = true;
    filter_.Update();
    correcting_ = false;
    ++updateCount_;
    PublishState();
}

void QuaternionEkf::SeedGyroBias(const alg_math::Vector3 &biasRadPerSec) noexcept
{
    if (!initialized_ || !std::isfinite(biasRadPerSec.x) || !std::isfinite(biasRadPerSec.y))
    {
        return;
    }
    filter_.state.At(StateBiasXIndex, 0U) = biasRadPerSec.x;
    filter_.state.At(StateBiasYIndex, 0U) = biasRadPerSec.y;
    filter_.filteredValue.At(StateBiasXIndex, 0U) = biasRadPerSec.x;
    filter_.filteredValue.At(StateBiasYIndex, 0U) = biasRadPerSec.y;
    PublishState();
}

void QuaternionEkf::Predict(const alg_math::Vector3 &gyroRadPerSec, float dtSeconds) noexcept
{
    if (!initialized_ || !std::isfinite(dtSeconds) || !(dtSeconds > 0.0F) || !alg_math::IsFinite(gyroRadPerSec))
    {
        return;
    }

    PreparePrediction(gyroRadPerSec, dtSeconds);
    correcting_ = false;
    filter_.SetSkipStep(3U, true); // 仅预测：不需要增益
    filter_.SetSkipStep(5U, true); // 仅预测：不使用上一轮的增益修正协方差
    filter_.Update();
    ++updateCount_;
    PublishState();
}

void QuaternionEkf::GetCovarianceDiagonal(float diagonal[6]) const noexcept
{
    for (std::size_t index = 0U; index < Filter::StateSize; ++index)
    {
        diagonal[index] = filter_.covariance.At(index, index);
    }
}

void QuaternionEkf::PreparePrediction(const alg_math::Vector3 &gyroRadPerSec, float dtSeconds) noexcept
{
    stepSeconds_ = dtSeconds;

    const float biasX = filter_.state.At(StateBiasXIndex, 0U);
    const float biasY = filter_.state.At(StateBiasYIndex, 0U);
    const float halfX = 0.5F * (gyroRadPerSec.x - biasX) * dtSeconds;
    const float halfY = 0.5F * (gyroRadPerSec.y - biasY) * dtSeconds;
    const float halfZ = 0.5F * gyroRadPerSec.z * dtSeconds; // z 轴零偏恒为零，不参与估计

    // 状态转移：四元数分块为 I + 0.5·dt·Ω(ω−b)，零偏分块为单位阵。
    // 零偏对四元数的耦合分块在 LinearizationStep 中写入，因此不影响本轮先验状态。
    Filter::StateMatrix transition = Filter::StateMatrix::Identity();
    transition.At(0U, 1U) = -halfX;
    transition.At(0U, 2U) = -halfY;
    transition.At(0U, 3U) = -halfZ;
    transition.At(1U, 0U) = halfX;
    transition.At(1U, 2U) = halfZ;
    transition.At(1U, 3U) = -halfY;
    transition.At(2U, 0U) = halfY;
    transition.At(2U, 1U) = -halfZ;
    transition.At(2U, 3U) = halfX;
    transition.At(3U, 0U) = halfZ;
    transition.At(3U, 1U) = halfY;
    transition.At(3U, 2U) = -halfX;
    filter_.transition = transition;

    // 过程噪声：四元数 4 维与零偏 2 维分别按方差率乘以步长。
    Filter::StateMatrix processNoise{};
    for (std::size_t index = 0U; index < 4U; ++index)
    {
        processNoise.At(index, index) = config_.processNoiseQuaternion * dtSeconds;
    }
    processNoise.At(StateBiasXIndex, StateBiasXIndex) = config_.processNoiseBias * dtSeconds;
    processNoise.At(StateBiasYIndex, StateBiasYIndex) = config_.processNoiseBias * dtSeconds;
    filter_.processNoise = processNoise;
}

void QuaternionEkf::LinearizationStep(Filter &filter, void *context) noexcept
{
    auto &self = *static_cast<QuaternionEkf *>(context);

    // 先验四元数就地单位化，后续观测矩阵与残差都基于单位四元数。
    alg_math::Quaternion quaternion{filter.predictedState.At(0U, 0U),
                                    filter.predictedState.At(1U, 0U),
                                    filter.predictedState.At(2U, 0U),
                                    filter.predictedState.At(3U, 0U)};
    if (!alg_math::Normalize(quaternion))
    {
        // 非有限输入：退化为单位四元数，避免把无效值继续传播到观测与残差。
        quaternion = alg_math::Quaternion{};
    }
    filter.predictedState.At(0U, 0U) = quaternion.w;
    filter.predictedState.At(1U, 0U) = quaternion.x;
    filter.predictedState.At(2U, 0U) = quaternion.y;
    filter.predictedState.At(3U, 0U) = quaternion.z;

    // 零偏对四元数的耦合分块（状态列 4、5）只在协方差传播中生效。
    const float halfStep = 0.5F * self.stepSeconds_;
    filter.transition.At(0U, StateBiasXIndex) = quaternion.x * halfStep;
    filter.transition.At(0U, StateBiasYIndex) = quaternion.y * halfStep;
    filter.transition.At(1U, StateBiasXIndex) = -quaternion.w * halfStep;
    filter.transition.At(1U, StateBiasYIndex) = quaternion.z * halfStep;
    filter.transition.At(2U, StateBiasXIndex) = -quaternion.z * halfStep;
    filter.transition.At(2U, StateBiasYIndex) = -quaternion.w * halfStep;
    filter.transition.At(3U, StateBiasXIndex) = quaternion.y * halfStep;
    filter.transition.At(3U, StateBiasYIndex) = -quaternion.x * halfStep;

    // 零偏方差渐消与限幅：抑制零偏过度收敛，同时防止发散。
    const float fadingFactor = self.config_.fadingFactor > 1.0F ? 1.0F : self.config_.fadingFactor;
    float &biasXVariance = filter.covariance.At(StateBiasXIndex, StateBiasXIndex);
    float &biasYVariance = filter.covariance.At(StateBiasYIndex, StateBiasYIndex);
    biasXVariance /= fadingFactor;
    biasYVariance /= fadingFactor;
    if (biasXVariance > BiasVarianceLimit)
    {
        biasXVariance = BiasVarianceLimit;
    }
    if (biasYVariance > BiasVarianceLimit)
    {
        biasYVariance = BiasVarianceLimit;
    }
}

void QuaternionEkf::ObservationStep(Filter &filter, void *context) noexcept
{
    (void)context;

    // 观测方程 h(q) 的雅可比：只有前四列非零，零偏不进入观测。
    const float q0 = filter.predictedState.At(0U, 0U);
    const float q1 = filter.predictedState.At(1U, 0U);
    const float q2 = filter.predictedState.At(2U, 0U);
    const float q3 = filter.predictedState.At(3U, 0U);

    Filter::MeasurementMatrix observation{};
    observation.At(0U, 0U) = -2.0F * q2;
    observation.At(0U, 1U) = 2.0F * q3;
    observation.At(0U, 2U) = -2.0F * q0;
    observation.At(0U, 3U) = 2.0F * q1;
    observation.At(1U, 0U) = 2.0F * q1;
    observation.At(1U, 1U) = 2.0F * q0;
    observation.At(1U, 2U) = 2.0F * q3;
    observation.At(1U, 3U) = 2.0F * q2;
    observation.At(2U, 0U) = 2.0F * q0;
    observation.At(2U, 1U) = -2.0F * q1;
    observation.At(2U, 2U) = -2.0F * q2;
    observation.At(2U, 3U) = 2.0F * q3;
    filter.measurement = observation;
}

void QuaternionEkf::CorrectionStep(Filter &filter, void *context) noexcept
{
    auto &self = *static_cast<QuaternionEkf *>(context);
    if (!self.correcting_)
    {
        return; // 仅预测模式：不使用上一轮增益做量测校正
    }

    // 预测的机体重力方向，用于残差与方向余弦。
    const alg_math::Quaternion quaternion{filter.predictedState.At(0U, 0U),
                                          filter.predictedState.At(1U, 0U),
                                          filter.predictedState.At(2U, 0U),
                                          filter.predictedState.At(3U, 0U)};
    const alg_math::Vector3 predictedGravity = alg_math::RotateInverse(quaternion, WorldGravityDirection);
    const float gravityComponents[3]{predictedGravity.x, predictedGravity.y, predictedGravity.z};
    for (std::size_t index = 0U; index < 3U; ++index)
    {
        self.orientationCosine_[index] = std::acos(std::fabs(gravityComponents[index]));
    }

    // 残差使用解析观测 h(q) = Rᵀ·ĝ；不能使用线性化的 H·x̂'，
    // 因为对二次型观测 H·x̂' = 2·h，与算法前身不一致。
    Filter::MeasureVector prediction{};
    prediction.At(0U, 0U) = predictedGravity.x;
    prediction.At(1U, 0U) = predictedGravity.y;
    prediction.At(2U, 0U) = predictedGravity.z;
    const Filter::MeasureVector residual = filter.measurementVector.Subtracted(prediction);
    float chiSquare = 0.0F;
    for (std::size_t row = 0U; row < Filter::MeasureSize; ++row)
    {
        for (std::size_t column = 0U; column < Filter::MeasureSize; ++column)
        {
            chiSquare += residual.At(row, 0U) * filter.innovationInverse.At(row, column) * residual.At(column, 0U);
        }
    }
    self.chiSquare_ = chiSquare;

    if (chiSquare < ConvergenceThresholdRatio * ChiSquareThreshold)
    {
        self.converged_ = true;
    }

    if (chiSquare > ChiSquareThreshold && self.converged_)
    {
        if (self.stable_)
        {
            ++self.errorCount_; // 静止工况下仍无法通过卡方检验，按发散计数
        }
        else
        {
            self.errorCount_ = 0U;
        }

        if (self.errorCount_ > DivergenceErrorLimit)
        {
            // 判定为发散：清收敛标志并恢复完整的协方差更新。
            self.converged_ = false;
            ++self.divergenceCount_;
        }
        else
        {
            // 残差未通过卡方检验：只保留预测结果，不做量测修正。
            filter.state = filter.predictedState;
            filter.covariance = filter.predictedCovariance;
            filter.SetSkipStep(5U, true);
            return;
        }
    }
    else
    {
        // 自适应增益：残差越小增益越接近 1，否则更相信预测。
        if (chiSquare > AdaptiveThresholdRatio * ChiSquareThreshold && self.converged_)
        {
            self.adaptiveGainScale_ = (ChiSquareThreshold - chiSquare) / (AdaptiveSpanRatio * ChiSquareThreshold);
        }
        else
        {
            self.adaptiveGainScale_ = 1.0F;
        }
        self.errorCount_ = 0U;
    }
    filter.SetSkipStep(5U, false);

    // 自适应缩放全部增益；零偏两行再按方向余弦缩放：重力在该轴投影不足时抑制零偏修正。
    for (std::size_t row = 0U; row < Filter::StateSize; ++row)
    {
        for (std::size_t column = 0U; column < Filter::MeasureSize; ++column)
        {
            filter.gain.At(row, column) *= self.adaptiveGainScale_;
        }
    }
    for (std::size_t index = 0U; index < BiasStateCount; ++index)
    {
        for (std::size_t column = 0U; column < Filter::MeasureSize; ++column)
        {
            filter.gain.At(StateBiasXIndex + index, column) *=
                self.orientationCosine_[index] / OrientationCosineReference;
        }
    }

    Filter::StateVector delta = Filter::StateVector::Multiply<Filter::MeasureSize>(filter.gain, residual);
    if (self.converged_)
    {
        // 收敛后限制零偏单步增量，避免突变。
        const float limit = BiasCorrectionRateLimit * self.stepSeconds_;
        for (std::size_t index = 0U; index < BiasStateCount; ++index)
        {
            float &value = delta.At(StateBiasXIndex + index, 0U);
            if (value > limit)
            {
                value = limit;
            }
            else if (value < -limit)
            {
                value = -limit;
            }
        }
    }
    delta.At(3U, 0U) = 0.0F; // 重力观测不修正偏航分量
    filter.state = filter.predictedState.Added(delta);
}

void QuaternionEkf::PublishState() noexcept
{
    State state{};

    alg_math::Quaternion quaternion{filter_.filteredValue.At(0U, 0U),
                                    filter_.filteredValue.At(1U, 0U),
                                    filter_.filteredValue.At(2U, 0U),
                                    filter_.filteredValue.At(3U, 0U)};
    const bool quaternionFinite = alg_math::IsFinite(quaternion);
    if (quaternionFinite && !alg_math::Normalize(quaternion))
    {
        state.quaternion = alg_math::Quaternion{};
    }
    else
    {
        state.quaternion = quaternionFinite ? quaternion : alg_math::Quaternion{};
    }
    state.gyroBiasRadPerSec = alg_math::Vector3{
        filter_.filteredValue.At(StateBiasXIndex, 0U), filter_.filteredValue.At(StateBiasYIndex, 0U), 0.0F};

    state.eulerRadians = alg_math::ToEulerZyxRadians(state.quaternion);
    state.eulerDegrees = alg_math::Vector3{alg_math::RadToDeg(state.eulerRadians.x),
                                           alg_math::RadToDeg(state.eulerRadians.y),
                                           alg_math::RadToDeg(state.eulerRadians.z)};

    // yaw 跨 ±180° 时累计圈数，得到连续角度。
    if (state.eulerDegrees.x - yawAngleLast_ > 180.0F)
    {
        --yawRoundCount_;
    }
    else if (state.eulerDegrees.x - yawAngleLast_ < -180.0F)
    {
        ++yawRoundCount_;
    }
    yawAngleLast_ = state.eulerDegrees.x;
    state.yawTotalDegrees = 360.0F * static_cast<float>(yawRoundCount_) + state.eulerDegrees.x;

    state.chiSquare = chiSquare_;
    state.adaptiveGainScale = adaptiveGainScale_;
    state.updateCount = updateCount_;
    state.divergenceCount = divergenceCount_;
    state.converged = converged_;
    state.stable = stable_;
    state.valid = initialized_ && quaternionFinite && updateCount_ > 0U;
    state_ = state;
}

} // namespace alg_estimate
