#include "Libraries/Algorithm/alg_estimate/QuaternionEkf.hpp"
#include "Libraries/Algorithm/alg_math/BasicMath.h"
#include <cmath>
#include <cstddef>

namespace alg_estimate
{

namespace
{
// 以下门限与比例取自算法前身的固定取值，移植时保持数值不变。
constexpr float CHI_SQUARE_THRESHOLD{1.0e-8F};            // 卡方检验门限
constexpr float CONVERGENCE_THRESHOLD_RATIO{0.5F};        // 低于该比例的统计量判定为已收敛
constexpr float ADAPTIVE_THRESHOLD_RATIO{0.1F};           // 自适应增益的启动比例
constexpr float ADAPTIVE_SPAN_RATIO{0.9F};                // 自适应增益的线性区间比例
constexpr std::uint32_t DIVERGENCE_ERROR_LIMIT{50U};      // 连续未通过次数达到该值判定为发散
constexpr float STABLE_GYRO_NORM{0.3F};                   // 静止判据：角速度范数上限，rad/s
constexpr float REFERENCE_GRAVITY{9.8F};                 // 静止判据：重力范数中心值，m/s^2
constexpr float STABLE_GRAVITY_TOLERANCE{0.5F};           // 静止判据：重力范数容差，m/s^2
constexpr float BIAS_CORRECTION_RATE_LIMIT{1.0e-2F};       // 收敛后零偏增量限幅，每秒
constexpr float BIAS_VARIANCE_LIMIT{1.0e4F};              // 零偏方差限幅，防止发散
constexpr float ORIENTATION_COSINE_REFERENCE{1.5707963F}; // 方向余弦归一化参考，约 π/2

/** 世界系重力方向：静止时机体加速度计测得的比力方向与该方向一致。 */
constexpr alg_math::Vector3 WORLD_GRAVITY_DIRECTION{0.0F, 0.0F, 1.0F};
} // namespace

bool QuaternionEkf::init(const Config &config) noexcept
{
    const bool valid = std::isfinite(config.process_noise_quaternion) && config.process_noise_quaternion > 0.0F &&
                       std::isfinite(config.process_noise_bias) && config.process_noise_bias > 0.0F &&
                       std::isfinite(config.measure_noise) && config.measure_noise > 0.0F &&
                       std::isfinite(config.fading_factor) && config.fading_factor > 0.0F &&
                       std::isfinite(config.initial_quaternion_variance) && config.initial_quaternion_variance > 0.0F &&
                       std::isfinite(config.initial_bias_variance) && config.initial_bias_variance > 0.0F &&
                       std::isfinite(config.initial_cross_variance) && config.initial_cross_variance >= 0.0F;
    if (!valid)
    {
        initialized_ = false;
        return false;
    }

    config_ = config;
    initialized_ = true;
    reset();
    return true;
}

void QuaternionEkf::reset() noexcept
{
    filter_.reset();

    // 后验状态由自定义校正步骤完成，其余标准步骤保持启用：增益步骤提供 S⁻¹ 与 K，
    // 协方差步骤消费被自适应缩放后的 K。
    filter_.set_skip_step(4U, true);
    filter_.set_user_step(1U, linearization_step, this);
    filter_.set_user_step(2U, observation_step, this);
    filter_.set_user_step(3U, correction_step, this);

    // 初值：单位四元数、零零偏、四元数与零偏各占对角块，非对角项为固定小量。
    Filter::StateVector initial_state{};
    initial_state.at(STATE_QUATERNION_INDEX, 0U) = 1.0F;
    filter_.state = initial_state;
    for (std::size_t row = 0U; row < Filter::STATE_SIZE; ++row)
    {
        for (std::size_t column = 0U; column < Filter::STATE_SIZE; ++column)
        {
            filter_.covariance.at(row, column) = config_.initial_cross_variance;
        }
    }
    for (std::size_t index = 0U; index < 4U; ++index)
    {
        filter_.covariance.at(index, index) = config_.initial_quaternion_variance;
    }
    filter_.covariance.at(STATE_BIAS_X_INDEX, STATE_BIAS_X_INDEX) = config_.initial_bias_variance;
    filter_.covariance.at(STATE_BIAS_Y_INDEX, STATE_BIAS_Y_INDEX) = config_.initial_bias_variance;

    // 量测噪声在整个运行期是常值，只在复位时写入，热路径不重复写。
    Filter::MeasureMatrix measurement_noise{};
    for (std::size_t index = 0U; index < Filter::MEASURE_SIZE; ++index)
    {
        measurement_noise.at(index, index) = config_.measure_noise;
    }
    filter_.measurement_noise = measurement_noise;

    converged_ = false;
    stable_ = false;
    chi_square_ = 0.0F;
    adaptive_gain_scale_ = 1.0F;
    error_count_ = 0U;
    divergence_count_ = 0U;
    update_count_ = 0U;
    yaw_round_count_ = 0;
    yaw_angle_last_ = 0.0F;
    orientation_cosine_[0] = 0.0F;
    orientation_cosine_[1] = 0.0F;
    orientation_cosine_[2] = 0.0F;
    step_seconds_ = 0.0F;

    publish_state();
}

void QuaternionEkf::update(const alg_math::Vector3 &gyro_rad_per_sec,
                           const alg_math::Vector3 &accel_meters_per_sec2,
                           float dt_seconds) noexcept
{
    if (!initialized_ || !std::isfinite(dt_seconds) || !(dt_seconds > 0.0F) || !alg_math::is_finite(gyro_rad_per_sec) ||
        !alg_math::is_finite(accel_meters_per_sec2))
    {
        return;
    }

    // 加速度方向作为重力观测向量；范数用于静止判据。
    alg_math::Vector3 gravity_direction = accel_meters_per_sec2;
    const float accel_norm = alg_math::norm(gravity_direction);
    if (!alg_math::normalize(gravity_direction))
    {
        return;
    }

    // 静止判据使用上一轮后验零偏修正后的角速度范数。
    const float corrected_norm =
        alg_math::norm(alg_math::Vector3{gyro_rad_per_sec.x - filter_.state.at(STATE_BIAS_X_INDEX, 0U),
                                         gyro_rad_per_sec.y - filter_.state.at(STATE_BIAS_Y_INDEX, 0U),
                                         gyro_rad_per_sec.z});
    stable_ = (corrected_norm < STABLE_GYRO_NORM) && (accel_norm > REFERENCE_GRAVITY - STABLE_GRAVITY_TOLERANCE) &&
              (accel_norm < REFERENCE_GRAVITY + STABLE_GRAVITY_TOLERANCE);

    filter_.measurement_input.at(0U, 0U) = gravity_direction.x;
    filter_.measurement_input.at(1U, 0U) = gravity_direction.y;
    filter_.measurement_input.at(2U, 0U) = gravity_direction.z;

    filter_.set_skip_step(3U, false); // 增益步骤提供新息协方差逆与增益，供卡方检验与校正使用
    prepare_prediction(gyro_rad_per_sec, dt_seconds);
    correcting_ = true;
    filter_.update();
    correcting_ = false;
    ++update_count_;
    publish_state();
}

void QuaternionEkf::seed_gyro_bias(const alg_math::Vector3 &bias_rad_per_sec) noexcept
{
    if (!initialized_ || !std::isfinite(bias_rad_per_sec.x) || !std::isfinite(bias_rad_per_sec.y))
    {
        return;
    }
    filter_.state.at(STATE_BIAS_X_INDEX, 0U) = bias_rad_per_sec.x;
    filter_.state.at(STATE_BIAS_Y_INDEX, 0U) = bias_rad_per_sec.y;
    filter_.filtered_value.at(STATE_BIAS_X_INDEX, 0U) = bias_rad_per_sec.x;
    filter_.filtered_value.at(STATE_BIAS_Y_INDEX, 0U) = bias_rad_per_sec.y;
    publish_state();
}

void QuaternionEkf::predict(const alg_math::Vector3 &gyro_rad_per_sec, float dt_seconds) noexcept
{
    if (!initialized_ || !std::isfinite(dt_seconds) || !(dt_seconds > 0.0F) || !alg_math::is_finite(gyro_rad_per_sec))
    {
        return;
    }

    prepare_prediction(gyro_rad_per_sec, dt_seconds);
    correcting_ = false;
    filter_.set_skip_step(3U, true); // 仅预测：不需要增益
    filter_.set_skip_step(5U, true); // 仅预测：不使用上一轮的增益修正协方差
    filter_.update();
    ++update_count_;
    publish_state();
}

void QuaternionEkf::get_covariance_diagonal(float diagonal[6]) const noexcept
{
    for (std::size_t index = 0U; index < Filter::STATE_SIZE; ++index)
    {
        diagonal[index] = filter_.covariance.at(index, index);
    }
}

void QuaternionEkf::prepare_prediction(const alg_math::Vector3 &gyro_rad_per_sec, float dt_seconds) noexcept
{
    step_seconds_ = dt_seconds;

    const float bias_x = filter_.state.at(STATE_BIAS_X_INDEX, 0U);
    const float bias_y = filter_.state.at(STATE_BIAS_Y_INDEX, 0U);
    const float half_x = 0.5F * (gyro_rad_per_sec.x - bias_x) * dt_seconds;
    const float half_y = 0.5F * (gyro_rad_per_sec.y - bias_y) * dt_seconds;
    const float half_z = 0.5F * gyro_rad_per_sec.z * dt_seconds; // z 轴零偏恒为零，不参与估计

    // 状态转移：四元数分块为 I + 0.5·dt·Ω(ω−b)，零偏分块为单位阵。
    // 零偏对四元数的耦合分块在 linearization_step 中写入，因此不影响本轮先验状态。
    Filter::StateMatrix transition = Filter::StateMatrix::identity();
    transition.at(0U, 1U) = -half_x;
    transition.at(0U, 2U) = -half_y;
    transition.at(0U, 3U) = -half_z;
    transition.at(1U, 0U) = half_x;
    transition.at(1U, 2U) = half_z;
    transition.at(1U, 3U) = -half_y;
    transition.at(2U, 0U) = half_y;
    transition.at(2U, 1U) = -half_z;
    transition.at(2U, 3U) = half_x;
    transition.at(3U, 0U) = half_z;
    transition.at(3U, 1U) = half_y;
    transition.at(3U, 2U) = -half_x;
    filter_.transition = transition;

    // 过程噪声：四元数 4 维与零偏 2 维分别按方差率乘以步长。
    Filter::StateMatrix process_noise{};
    for (std::size_t index = 0U; index < 4U; ++index)
    {
        process_noise.at(index, index) = config_.process_noise_quaternion * dt_seconds;
    }
    process_noise.at(STATE_BIAS_X_INDEX, STATE_BIAS_X_INDEX) = config_.process_noise_bias * dt_seconds;
    process_noise.at(STATE_BIAS_Y_INDEX, STATE_BIAS_Y_INDEX) = config_.process_noise_bias * dt_seconds;
    filter_.process_noise = process_noise;
}

void QuaternionEkf::linearization_step(Filter &filter, void *context) noexcept
{
    auto &self = *static_cast<QuaternionEkf *>(context);

    // 先验四元数就地单位化，后续观测矩阵与残差都基于单位四元数。
    alg_math::Quaternion quaternion{filter.predicted_state.at(0U, 0U),
                                    filter.predicted_state.at(1U, 0U),
                                    filter.predicted_state.at(2U, 0U),
                                    filter.predicted_state.at(3U, 0U)};
    if (!alg_math::normalize(quaternion))
    {
        // 非有限输入：退化为单位四元数，避免把无效值继续传播到观测与残差。
        quaternion = alg_math::Quaternion{};
    }
    filter.predicted_state.at(0U, 0U) = quaternion.w;
    filter.predicted_state.at(1U, 0U) = quaternion.x;
    filter.predicted_state.at(2U, 0U) = quaternion.y;
    filter.predicted_state.at(3U, 0U) = quaternion.z;

    // 零偏对四元数的耦合分块（状态列 4、5）只在协方差传播中生效。
    const float half_step = 0.5F * self.step_seconds_;
    filter.transition.at(0U, STATE_BIAS_X_INDEX) = quaternion.x * half_step;
    filter.transition.at(0U, STATE_BIAS_Y_INDEX) = quaternion.y * half_step;
    filter.transition.at(1U, STATE_BIAS_X_INDEX) = -quaternion.w * half_step;
    filter.transition.at(1U, STATE_BIAS_Y_INDEX) = quaternion.z * half_step;
    filter.transition.at(2U, STATE_BIAS_X_INDEX) = -quaternion.z * half_step;
    filter.transition.at(2U, STATE_BIAS_Y_INDEX) = -quaternion.w * half_step;
    filter.transition.at(3U, STATE_BIAS_X_INDEX) = quaternion.y * half_step;
    filter.transition.at(3U, STATE_BIAS_Y_INDEX) = -quaternion.x * half_step;

    // 零偏方差渐消与限幅：抑制零偏过度收敛，同时防止发散。
    const float fading_factor = self.config_.fading_factor > 1.0F ? 1.0F : self.config_.fading_factor;
    float &bias_x_variance = filter.covariance.at(STATE_BIAS_X_INDEX, STATE_BIAS_X_INDEX);
    float &bias_y_variance = filter.covariance.at(STATE_BIAS_Y_INDEX, STATE_BIAS_Y_INDEX);
    bias_x_variance /= fading_factor;
    bias_y_variance /= fading_factor;
    if (bias_x_variance > BIAS_VARIANCE_LIMIT)
    {
        bias_x_variance = BIAS_VARIANCE_LIMIT;
    }
    if (bias_y_variance > BIAS_VARIANCE_LIMIT)
    {
        bias_y_variance = BIAS_VARIANCE_LIMIT;
    }
}

void QuaternionEkf::observation_step(Filter &filter, void *context) noexcept
{
    (void)context;

    // 观测方程 h(q) 的雅可比：只有前四列非零，零偏不进入观测。
    const float q0 = filter.predicted_state.at(0U, 0U);
    const float q1 = filter.predicted_state.at(1U, 0U);
    const float q2 = filter.predicted_state.at(2U, 0U);
    const float q3 = filter.predicted_state.at(3U, 0U);

    Filter::MeasurementMatrix observation{};
    observation.at(0U, 0U) = -2.0F * q2;
    observation.at(0U, 1U) = 2.0F * q3;
    observation.at(0U, 2U) = -2.0F * q0;
    observation.at(0U, 3U) = 2.0F * q1;
    observation.at(1U, 0U) = 2.0F * q1;
    observation.at(1U, 1U) = 2.0F * q0;
    observation.at(1U, 2U) = 2.0F * q3;
    observation.at(1U, 3U) = 2.0F * q2;
    observation.at(2U, 0U) = 2.0F * q0;
    observation.at(2U, 1U) = -2.0F * q1;
    observation.at(2U, 2U) = -2.0F * q2;
    observation.at(2U, 3U) = 2.0F * q3;
    filter.measurement = observation;
}

void QuaternionEkf::correction_step(Filter &filter, void *context) noexcept
{
    auto &self = *static_cast<QuaternionEkf *>(context);
    if (!self.correcting_)
    {
        return; // 仅预测模式：不使用上一轮增益做量测校正
    }

    // 预测的机体重力方向，用于残差与方向余弦。
    const alg_math::Quaternion quaternion{filter.predicted_state.at(0U, 0U),
                                          filter.predicted_state.at(1U, 0U),
                                          filter.predicted_state.at(2U, 0U),
                                          filter.predicted_state.at(3U, 0U)};
    const alg_math::Vector3 predicted_gravity = alg_math::rotate_inverse(quaternion, WORLD_GRAVITY_DIRECTION);
    const float gravity_components[3]{predicted_gravity.x, predicted_gravity.y, predicted_gravity.z};
    for (std::size_t index = 0U; index < 3U; ++index)
    {
        self.orientation_cosine_[index] = std::acos(std::fabs(gravity_components[index]));
    }

    // 残差使用解析观测 h(q) = Rᵀ·ĝ；不能使用线性化的 H·x̂'，
    // 因为对二次型观测 H·x̂' = 2·h，与算法前身不一致。
    Filter::MeasureVector prediction{};
    prediction.at(0U, 0U) = predicted_gravity.x;
    prediction.at(1U, 0U) = predicted_gravity.y;
    prediction.at(2U, 0U) = predicted_gravity.z;
    const Filter::MeasureVector residual = filter.measurement_vector.subtracted(prediction);
    float chi_square = 0.0F;
    for (std::size_t row = 0U; row < Filter::MEASURE_SIZE; ++row)
    {
        for (std::size_t column = 0U; column < Filter::MEASURE_SIZE; ++column)
        {
            chi_square += residual.at(row, 0U) * filter.innovation_inverse.at(row, column) * residual.at(column, 0U);
        }
    }
    self.chi_square_ = chi_square;

    if (chi_square < CONVERGENCE_THRESHOLD_RATIO * CHI_SQUARE_THRESHOLD)
    {
        self.converged_ = true;
    }

    if (chi_square > CHI_SQUARE_THRESHOLD && self.converged_)
    {
        if (self.stable_)
        {
            ++self.error_count_; // 静止工况下仍无法通过卡方检验，按发散计数
        }
        else
        {
            self.error_count_ = 0U;
        }

        if (self.error_count_ > DIVERGENCE_ERROR_LIMIT)
        {
            // 判定为发散：清收敛标志并恢复完整的协方差更新。
            self.converged_ = false;
            ++self.divergence_count_;
        }
        else
        {
            // 残差未通过卡方检验：只保留预测结果，不做量测修正。
            filter.state = filter.predicted_state;
            filter.covariance = filter.predicted_covariance;
            filter.set_skip_step(5U, true);
            return;
        }
    }
    else
    {
        // 自适应增益：残差越小增益越接近 1，否则更相信预测。
        if (chi_square > ADAPTIVE_THRESHOLD_RATIO * CHI_SQUARE_THRESHOLD && self.converged_)
        {
            self.adaptive_gain_scale_ = (CHI_SQUARE_THRESHOLD - chi_square) / (ADAPTIVE_SPAN_RATIO * CHI_SQUARE_THRESHOLD);
        }
        else
        {
            self.adaptive_gain_scale_ = 1.0F;
        }
        self.error_count_ = 0U;
    }
    filter.set_skip_step(5U, false);

    // 自适应缩放全部增益；零偏两行再按方向余弦缩放：重力在该轴投影不足时抑制零偏修正。
    for (std::size_t row = 0U; row < Filter::STATE_SIZE; ++row)
    {
        for (std::size_t column = 0U; column < Filter::MEASURE_SIZE; ++column)
        {
            filter.gain.at(row, column) *= self.adaptive_gain_scale_;
        }
    }
    for (std::size_t index = 0U; index < BIAS_STATE_COUNT; ++index)
    {
        for (std::size_t column = 0U; column < Filter::MEASURE_SIZE; ++column)
        {
            filter.gain.at(STATE_BIAS_X_INDEX + index, column) *=
                self.orientation_cosine_[index] / ORIENTATION_COSINE_REFERENCE;
        }
    }

    Filter::StateVector delta = Filter::StateVector::multiply<Filter::MEASURE_SIZE>(filter.gain, residual);
    if (self.converged_)
    {
        // 收敛后限制零偏单步增量，避免突变。
        const float limit = BIAS_CORRECTION_RATE_LIMIT * self.step_seconds_;
        for (std::size_t index = 0U; index < BIAS_STATE_COUNT; ++index)
        {
            float &value = delta.at(STATE_BIAS_X_INDEX + index, 0U);
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
    delta.at(3U, 0U) = 0.0F; // 重力观测不修正偏航分量
    filter.state = filter.predicted_state.added(delta);
}

void QuaternionEkf::publish_state() noexcept
{
    State state{};

    alg_math::Quaternion quaternion{filter_.filtered_value.at(0U, 0U),
                                    filter_.filtered_value.at(1U, 0U),
                                    filter_.filtered_value.at(2U, 0U),
                                    filter_.filtered_value.at(3U, 0U)};
    const bool quaternion_finite = alg_math::is_finite(quaternion);
    if (quaternion_finite && !alg_math::normalize(quaternion))
    {
        state.quaternion = alg_math::Quaternion{};
    }
    else
    {
        state.quaternion = quaternion_finite ? quaternion : alg_math::Quaternion{};
    }
    state.gyro_bias_rad_per_sec = alg_math::Vector3{
        filter_.filtered_value.at(STATE_BIAS_X_INDEX, 0U), filter_.filtered_value.at(STATE_BIAS_Y_INDEX, 0U), 0.0F};

    state.euler_radians = alg_math::to_euler_zyx_radians(state.quaternion);
    state.euler_degrees = alg_math::Vector3{alg_math::rad_to_deg(state.euler_radians.x),
                                           alg_math::rad_to_deg(state.euler_radians.y),
                                           alg_math::rad_to_deg(state.euler_radians.z)};

    // yaw 跨 ±180° 时累计圈数，得到连续角度。
    if (state.euler_degrees.x - yaw_angle_last_ > 180.0F)
    {
        --yaw_round_count_;
    }
    else if (state.euler_degrees.x - yaw_angle_last_ < -180.0F)
    {
        ++yaw_round_count_;
    }
    yaw_angle_last_ = state.euler_degrees.x;
    state.yaw_total_degrees = 360.0F * static_cast<float>(yaw_round_count_) + state.euler_degrees.x;

    state.chi_square = chi_square_;
    state.adaptive_gain_scale = adaptive_gain_scale_;
    state.update_count = update_count_;
    state.divergence_count = divergence_count_;
    state.converged = converged_;
    state.stable = stable_;
    state.valid = initialized_ && quaternion_finite && update_count_ > 0U;
    state_ = state;
}

} // namespace alg_estimate
