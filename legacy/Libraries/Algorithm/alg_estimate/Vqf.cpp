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

constexpr double EPSILON{static_cast<double>(std::numeric_limits<float>::epsilon())};

/**
 * @brief 把 3×3 旋转矩阵写入扁平矩阵。
 */
void set_rotation_matrix(const alg_math::Quaternion &attitude, alg_math::Matrix3 &rotation) noexcept
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

bool same_config(const Vqf::Config &first, const Vqf::Config &second) noexcept
{
    return first.tau_acc_seconds == second.tau_acc_seconds &&
           first.rest_bias_est_enabled == second.rest_bias_est_enabled &&
           first.motion_bias_est_enabled == second.motion_bias_est_enabled &&
           first.bias_sigma_init_deg_per_sec == second.bias_sigma_init_deg_per_sec &&
           first.bias_forgetting_time_seconds == second.bias_forgetting_time_seconds &&
           first.bias_clip_rad_per_sec == second.bias_clip_rad_per_sec &&
           first.bias_sigma_motion_deg_per_sec == second.bias_sigma_motion_deg_per_sec &&
           first.bias_vertical_forgetting_factor == second.bias_vertical_forgetting_factor &&
           first.bias_sigma_rest_deg_per_sec == second.bias_sigma_rest_deg_per_sec &&
           first.rest_min_seconds == second.rest_min_seconds &&
           first.rest_filter_tau_seconds == second.rest_filter_tau_seconds &&
           first.rest_threshold_gyr_rad_per_sec == second.rest_threshold_gyr_rad_per_sec &&
           first.rest_threshold_acc_meters_per_sec2 == second.rest_threshold_acc_meters_per_sec2;
}

} // namespace

bool Vqf::init(const Config &config, float gyro_period_seconds, float accel_period_seconds) noexcept
{
    const bool periods_valid = std::isfinite(gyro_period_seconds) && std::isfinite(accel_period_seconds) &&
                               gyro_period_seconds > 0.0F && accel_period_seconds > 0.0F;
    const bool time_constants_valid = std::isfinite(config.tau_acc_seconds) && config.tau_acc_seconds > 0.0F &&
                                      std::isfinite(config.rest_filter_tau_seconds) &&
                                      config.rest_filter_tau_seconds > 0.0F;
    const bool scales_valid =
        std::isfinite(config.rest_min_seconds) && config.rest_min_seconds >= 0.0F &&
        std::isfinite(config.bias_forgetting_time_seconds) && config.bias_forgetting_time_seconds > 0.0F &&
        std::isfinite(config.bias_sigma_init_deg_per_sec) && config.bias_sigma_init_deg_per_sec > 0.0F &&
        std::isfinite(config.bias_sigma_motion_deg_per_sec) && config.bias_sigma_motion_deg_per_sec > 0.0F &&
        std::isfinite(config.bias_sigma_rest_deg_per_sec) && config.bias_sigma_rest_deg_per_sec > 0.0F &&
        std::isfinite(config.bias_clip_rad_per_sec) && config.bias_clip_rad_per_sec > 0.0F &&
        std::isfinite(config.bias_vertical_forgetting_factor) && config.bias_vertical_forgetting_factor > 0.0F &&
        std::isfinite(config.rest_threshold_gyr_rad_per_sec) && config.rest_threshold_gyr_rad_per_sec >= 0.0F &&
        std::isfinite(config.rest_threshold_acc_meters_per_sec2) && config.rest_threshold_acc_meters_per_sec2 >= 0.0F;
    if (!periods_valid || !time_constants_valid || !scales_valid)
    {
        initialized_ = false;
        return false;
    }

    gyro_period_ = static_cast<double>(gyro_period_seconds);
    accel_period_ = static_cast<double>(accel_period_seconds);
    rest_min_ = static_cast<double>(config.rest_min_seconds);
    bias_clip_ = static_cast<double>(config.bias_clip_rad_per_sec);
    rest_threshold_gyr_ = static_cast<double>(config.rest_threshold_gyr_rad_per_sec);
    rest_threshold_acc_ = static_cast<double>(config.rest_threshold_acc_meters_per_sec2);
    rest_bias_est_enabled_ = config.rest_bias_est_enabled;
    motion_bias_est_enabled_ = config.motion_bias_est_enabled;

    // 三组低通：加速度计（τ_acc）、静止判定（τ_rest，两路各自周期）、运动零偏估计（τ_acc）。
    if (!accel_low_pass_.init({config.tau_acc_seconds, accel_period_seconds}) ||
        !rest_gyro_low_pass_.init({config.rest_filter_tau_seconds, gyro_period_seconds}) ||
        !rest_accel_low_pass_.init({config.rest_filter_tau_seconds, accel_period_seconds}) ||
        !motion_bias_rotation_low_pass_.init({config.tau_acc_seconds, accel_period_seconds}) ||
        !motion_bias_projection_low_pass_.init({config.tau_acc_seconds, accel_period_seconds}))
    {
        initialized_ = false;
        return false;
    }

    config_ = config;
    // 零偏协方差与过程/量测噪声
    // 该单位约定不影响零偏估计值（增益对 P、V、W 的整体缩放不变），只影响不确定度输出。
    bias_p0_ = alg_math::square(static_cast<double>(config.bias_sigma_init_deg_per_sec) * 100.0);
    bias_v_ = alg_math::square(0.1 * 100.0) * accel_period_ / static_cast<double>(config.bias_forgetting_time_seconds);
    const double p_motion = alg_math::square(static_cast<double>(config.bias_sigma_motion_deg_per_sec) * 100.0);
    bias_motion_w_ = alg_math::square(p_motion) / bias_v_ + p_motion;
    bias_vertical_w_ = bias_motion_w_ / (static_cast<double>(config.bias_vertical_forgetting_factor) > 1.0e-10
                                             ? static_cast<double>(config.bias_vertical_forgetting_factor)
                                             : 1.0e-10);
    const double p_rest = alg_math::square(static_cast<double>(config.bias_sigma_rest_deg_per_sec) * 100.0);
    bias_rest_w_ = alg_math::square(p_rest) / bias_v_ + p_rest;

    initialized_ = true;
    reset();
    return true;
}

void Vqf::reset() noexcept
{
    gyro_quat_ = alg_math::Quaternion{};
    accel_quat_ = alg_math::Quaternion{};
    last_accel_lp_ = alg_math::Vector3{};
    last_accel_correction_radians_ = 0.0F;
    accel_low_pass_.reset();

    gyro_bias_ = alg_math::Vector3{};
    bias_covariance_ = alg_math::scaled_identity(bias_p0_);
    motion_bias_rotation_low_pass_.reset();
    motion_bias_projection_low_pass_.reset();

    rest_detected_ = false;
    rest_t_ = 0.0;
    rest_squared_deviations_[0] = 0.0;
    rest_squared_deviations_[1] = 0.0;
    rest_last_gyro_lp_ = alg_math::Vector3{};
    rest_last_accel_lp_ = alg_math::Vector3{};
    rest_gyro_low_pass_.reset();
    rest_accel_low_pass_.reset();

    update_count_ = 0U;
    valid_ = false;
}

void Vqf::update_gyroscope(const alg_math::Vector3 &gyro_rad_per_sec) noexcept
{
    if (!initialized_ || !alg_math::is_finite(gyro_rad_per_sec))
    {
        return;
    }

    const float gyro[3]{gyro_rad_per_sec.x, gyro_rad_per_sec.y, gyro_rad_per_sec.z};

    // 静止判定的陀螺侧：低通后方差超阈值，或低通值本身超过零偏限幅，都判为"运动中"。
    if (rest_bias_est_enabled_)
    {
        float gyro_lp[3]{};
        rest_gyro_low_pass_.filter(gyro, gyro_lp);
        rest_last_gyro_lp_ = alg_math::Vector3{gyro_lp[0], gyro_lp[1], gyro_lp[2]};

        rest_squared_deviations_[0] = alg_math::square(static_cast<double>(gyro[0]) - static_cast<double>(gyro_lp[0])) +
                                      alg_math::square(static_cast<double>(gyro[1]) - static_cast<double>(gyro_lp[1])) +
                                      alg_math::square(static_cast<double>(gyro[2]) - static_cast<double>(gyro_lp[2]));

        if (rest_squared_deviations_[0] >= alg_math::square(rest_threshold_gyr_) ||
            std::fabs(static_cast<double>(gyro_lp[0])) > bias_clip_ ||
            std::fabs(static_cast<double>(gyro_lp[1])) > bias_clip_ ||
            std::fabs(static_cast<double>(gyro_lp[2])) > bias_clip_)
        {
            rest_t_ = 0.0;
            rest_detected_ = false;
        }
    }

    // 去除估计零偏后做角速度积分。
    const alg_math::Vector3 gyro_no_bias{
        gyro_rad_per_sec.x - gyro_bias_.x, gyro_rad_per_sec.y - gyro_bias_.y, gyro_rad_per_sec.z - gyro_bias_.z};
    const double gyro_norm = static_cast<double>(alg_math::norm(gyro_no_bias));
    if (gyro_norm > EPSILON)
    {
        const double angle = gyro_norm * gyro_period_;
        const double cosine = std::cos(angle / 2.0);
        const double sine = std::sin(angle / 2.0) / gyro_norm;
        const alg_math::Quaternion step{static_cast<float>(cosine),
                                        static_cast<float>(sine * static_cast<double>(gyro_no_bias.x)),
                                        static_cast<float>(sine * static_cast<double>(gyro_no_bias.y)),
                                        static_cast<float>(sine * static_cast<double>(gyro_no_bias.z))};
        gyro_quat_ = alg_math::multiply(gyro_quat_, step);
        (void)alg_math::normalize(gyro_quat_);
    }

    ++update_count_;
    valid_ = true;
}

void Vqf::update_accelerometer(const alg_math::Vector3 &accel_meters_per_sec2) noexcept
{
    if (!initialized_ || !alg_math::is_finite(accel_meters_per_sec2))
    {
        return;
    }
    if (accel_meters_per_sec2.x == 0.0F && accel_meters_per_sec2.y == 0.0F && accel_meters_per_sec2.z == 0.0F)
    {
        return;
    }

    const float accel[3]{accel_meters_per_sec2.x, accel_meters_per_sec2.y, accel_meters_per_sec2.z};

    // 静止判定的加速度侧：这一侧才会把 rest_detected 置真（陀螺侧只负责置假）。
    if (rest_bias_est_enabled_)
    {
        float accel_lp[3]{};
        rest_accel_low_pass_.filter(accel, accel_lp);
        rest_last_accel_lp_ = alg_math::Vector3{accel_lp[0], accel_lp[1], accel_lp[2]};

        rest_squared_deviations_[1] =
            alg_math::square(static_cast<double>(accel[0]) - static_cast<double>(accel_lp[0])) +
            alg_math::square(static_cast<double>(accel[1]) - static_cast<double>(accel_lp[1])) +
            alg_math::square(static_cast<double>(accel[2]) - static_cast<double>(accel_lp[2]));

        if (rest_squared_deviations_[1] >= alg_math::square(rest_threshold_acc_))
        {
            rest_t_ = 0.0;
            rest_detected_ = false;
        }
        else
        {
            rest_t_ += accel_period_;
            if (rest_t_ >= rest_min_)
            {
                rest_detected_ = true;
            }
        }
    }

    // 先转到惯性系（角速度积分系）再做低通，避免运动方向变化污染重力方向的提取。
    const alg_math::Vector3 accel_inertial = alg_math::rotate(gyro_quat_, accel_meters_per_sec2);
    float accel_inertial_array[3]{accel_inertial.x, accel_inertial.y, accel_inertial.z};
    accel_low_pass_.filter(accel_inertial_array);
    last_accel_lp_ = alg_math::Vector3{accel_inertial_array[0], accel_inertial_array[1], accel_inertial_array[2]};

    // 转到 6 维地球系并归一化，得到重力方向。
    alg_math::Vector3 accel_earth = alg_math::rotate(accel_quat_, last_accel_lp_);
    if (!alg_math::normalize(accel_earth))
    {
        return; // 退化输入：不做修正，避免写入非有限姿态
    }

    // 倾角修正：构造把地球系 +z 对齐到测得的重力方向的修正四元数。
    alg_math::Quaternion correction{};
    const float half_w = std::sqrt((accel_earth.z + 1.0F) / 2.0F);
    if (half_w > 1.0e-6F)
    {
        correction.w = half_w;
        correction.x = 0.5F * accel_earth.y / half_w;
        correction.y = -0.5F * accel_earth.x / half_w;
        correction.z = 0.0F;
    }
    else
    {
        // 测得的重力方向接近 -z（修正角接近 180°）时用 180° 绕 x 轴的等价表示，避免数值问题。
        correction = alg_math::Quaternion{0.0F, 1.0F, 0.0F, 0.0F};
    }
    accel_quat_ = alg_math::multiply(correction, accel_quat_);
    (void)alg_math::normalize(accel_quat_);

    // 诊断量：本次倾角修正的转角。每个加速度计样本一次 acos（约 0.1% CPU 量级），
    // 上游把它除以采样周期当作"修正角速度"，这里只暴露转角，换算交给调用方。
    const double accel_earth_z =
        static_cast<double>(accel_earth.z) > 1.0
            ? 1.0
            : (static_cast<double>(accel_earth.z) < -1.0 ? -1.0 : static_cast<double>(accel_earth.z));
    last_accel_correction_radians_ = std::acos(accel_earth_z);

    if (!rest_bias_est_enabled_ && !motion_bias_est_enabled_)
    {
        return;
    }

    // 零偏估计（3×3 协方差的上推式卡尔曼更新，静止更新与运动更新二选一）
    alg_math::Matrix3 rotation{};
    set_rotation_matrix(get_quaternion(), rotation);

    // R·b 的 x/y 分量（z 分量不参与水平修正）。
    double bias_projection[2]{
        rotation.value[0] * gyro_bias_.x + rotation.value[1] * gyro_bias_.y + rotation.value[2] * gyro_bias_.z,
        rotation.value[3] * gyro_bias_.x + rotation.value[4] * gyro_bias_.y + rotation.value[5] * gyro_bias_.z};

    float rotation_lp[9];
    float bias_projection_lp[2];
    for (std::size_t index = 0U; index < 9U; ++index)
    {
        rotation_lp[index] = static_cast<float>(rotation.value[index]);
    }
    bias_projection_lp[0] = static_cast<float>(bias_projection[0]);
    bias_projection_lp[1] = static_cast<float>(bias_projection[1]);
    motion_bias_rotation_low_pass_.filter(rotation_lp);
    motion_bias_projection_low_pass_.filter(bias_projection_lp);

    alg_math::Matrix3 rotation_low_passed{};
    for (std::size_t index = 0U; index < 9U; ++index)
    {
        rotation_low_passed.value[index] = static_cast<double>(rotation_lp[index]);
    }
    double bias_projection_low_passed[2]{static_cast<double>(bias_projection_lp[0]),
                                         static_cast<double>(bias_projection_lp[1])};

    double innovation[3]{};
    double variance[3]{};
    if (rest_detected_ && rest_bias_est_enabled_)
    {
        innovation[0] = static_cast<double>(rest_last_gyro_lp_.x) - gyro_bias_.x;
        innovation[1] = static_cast<double>(rest_last_gyro_lp_.y) - gyro_bias_.y;
        innovation[2] = static_cast<double>(rest_last_gyro_lp_.z) - gyro_bias_.z;
        rotation_low_passed = alg_math::scaled_identity(1.0);
        variance[0] = bias_rest_w_;
        variance[1] = bias_rest_w_;
        variance[2] = bias_rest_w_;
    }
    else if (motion_bias_est_enabled_)
    {
        innovation[0] = -static_cast<double>(accel_earth.y) / accel_period_ + bias_projection_low_passed[0] -
                        rotation_low_passed.value[0] * gyro_bias_.x - rotation_low_passed.value[1] * gyro_bias_.y -
                        rotation_low_passed.value[2] * gyro_bias_.z;
        innovation[1] = static_cast<double>(accel_earth.x) / accel_period_ + bias_projection_low_passed[1] -
                        rotation_low_passed.value[3] * gyro_bias_.x - rotation_low_passed.value[4] * gyro_bias_.y -
                        rotation_low_passed.value[5] * gyro_bias_.z;
        innovation[2] = -rotation_low_passed.value[6] * gyro_bias_.x - rotation_low_passed.value[7] * gyro_bias_.y -
                        rotation_low_passed.value[8] * gyro_bias_.z;
        variance[0] = bias_motion_w_;
        variance[1] = bias_motion_w_;
        variance[2] = bias_vertical_w_;
    }
    else
    {
        variance[0] = -1.0; // 两个更新都关闭：只做协方差增长，不做量测更新
    }

    // 预测步：协方差只增不减，涨到初值上限为止。
    if (bias_covariance_.value[0] < bias_p0_)
    {
        bias_covariance_.value[0] += bias_v_;
    }
    if (bias_covariance_.value[4] < bias_p0_)
    {
        bias_covariance_.value[4] += bias_v_;
    }
    if (bias_covariance_.value[8] < bias_p0_)
    {
        bias_covariance_.value[8] += bias_v_;
    }

    if (variance[0] >= 0.0)
    {
        // 新息限幅：等价 2 deg/s，同时限制首次倾角修正带来的冲击。
        for (std::size_t index = 0U; index < 3U; ++index)
        {
            innovation[index] = alg_math::limit(innovation[index], -bias_clip_, bias_clip_);
        }

        // K = P R^T inv(W + R P R^T)
        const alg_math::Matrix3 gain_transposed =
            alg_math::multiply_transposed_second(bias_covariance_, rotation_low_passed);
        alg_math::Matrix3 innovation_covariance = alg_math::multiply(rotation_low_passed, gain_transposed);
        innovation_covariance.value[0] += variance[0];
        innovation_covariance.value[4] += variance[1];
        innovation_covariance.value[8] += variance[2];
        if (!alg_math::inverted(innovation_covariance, innovation_covariance))
        {
            return; // 奇异：本拍不做零偏更新
        }
        const alg_math::Matrix3 gain_base =
            alg_math::multiply_transposed_first(rotation_low_passed, innovation_covariance); // R^T inv(...)
        const alg_math::Matrix3 gain = alg_math::multiply(bias_covariance_, gain_base);      // P R^T inv(...)

        gyro_bias_.x += static_cast<float>(gain.value[0] * innovation[0] + gain.value[1] * innovation[1] +
                                           gain.value[2] * innovation[2]);
        gyro_bias_.y += static_cast<float>(gain.value[3] * innovation[0] + gain.value[4] * innovation[1] +
                                           gain.value[5] * innovation[2]);
        gyro_bias_.z += static_cast<float>(gain.value[6] * innovation[0] + gain.value[7] * innovation[1] +
                                           gain.value[8] * innovation[2]);

        // P = P - K R P
        const alg_math::Matrix3 update =
            alg_math::multiply(alg_math::multiply(gain, rotation_low_passed), bias_covariance_);
        for (std::size_t index = 0U; index < 9U; ++index)
        {
            bias_covariance_.value[index] -= update.value[index];
        }

        double bias_array[3]{
            static_cast<double>(gyro_bias_.x), static_cast<double>(gyro_bias_.y), static_cast<double>(gyro_bias_.z)};
        for (std::size_t index = 0U; index < 3U; ++index)
        {
            bias_array[index] = alg_math::limit(bias_array[index], -bias_clip_, bias_clip_);
        }
        gyro_bias_ = alg_math::Vector3{
            static_cast<float>(bias_array[0]), static_cast<float>(bias_array[1]), static_cast<float>(bias_array[2])};
    }
}

void Vqf::update(const alg_math::Vector3 &gyro_rad_per_sec, const alg_math::Vector3 &accel_meters_per_sec2) noexcept
{
    update_gyroscope(gyro_rad_per_sec);
    update_accelerometer(accel_meters_per_sec2);
}

alg_math::Quaternion Vqf::get_quaternion() const noexcept
{
    return alg_math::multiply(accel_quat_, gyro_quat_);
}

Vqf::RestObservables Vqf::get_rest_observables() const noexcept
{
    // 因此 1.0 正好是判据边界，便于直接与阈值对照。
    const auto ratio = [](double squared_deviation, double threshold) noexcept
    {
        if (!(threshold > 0.0))
        {
            return std::numeric_limits<float>::infinity(); // 阈值为 0：永判为运动，如实上报
        }
        return static_cast<float>(std::sqrt(squared_deviation) / threshold);
    };

    RestObservables observables{};
    observables.gyroscope_deviation_ratio = ratio(rest_squared_deviations_[0], rest_threshold_gyr_);
    observables.accelerometer_deviation_ratio = ratio(rest_squared_deviations_[1], rest_threshold_acc_);
    observables.accumulated_rest_seconds = static_cast<float>(rest_t_);
    observables.rest_detected = rest_detected_;
    return observables;
}

void Vqf::set_bias_estimate(const alg_math::Vector3 &bias_rad_per_sec, float uncertainty_rad_per_sec) noexcept
{
    if (!initialized_ || !alg_math::is_finite(bias_rad_per_sec))
    {
        return;
    }

    gyro_bias_ = bias_rad_per_sec;

    // σ ≤ 0 或非有限 → 只改零偏，保留现有协方差
    if (std::isfinite(uncertainty_rad_per_sec) && uncertainty_rad_per_sec > 0.0F)
    {
        const double scaled =
            static_cast<double>(uncertainty_rad_per_sec) * 180.0 * 100.0 / static_cast<double>(alg_math::PI);
        bias_covariance_ = alg_math::scaled_identity(alg_math::square(scaled));
    }
}

Vqf::InternalState Vqf::get_internal_state() const noexcept
{
    InternalState state{};
    state.config = config_;
    state.gyro_period_seconds = gyro_period_;
    state.accel_period_seconds = accel_period_;
    state.gyro_quaternion = gyro_quat_;
    state.accel_quaternion = accel_quat_;
    state.last_accel_lp = last_accel_lp_;
    state.last_accel_correction_radians = last_accel_correction_radians_;
    state.accel_low_pass = accel_low_pass_.get_internal_state();
    state.gyro_bias_rad_per_sec = gyro_bias_;
    state.bias_covariance = bias_covariance_;
    state.motion_bias_rotation_low_pass = motion_bias_rotation_low_pass_.get_internal_state();
    state.motion_bias_projection_low_pass = motion_bias_projection_low_pass_.get_internal_state();
    state.rest_detected = rest_detected_;
    state.rest_seconds = rest_t_;
    state.rest_squared_deviations[0] = rest_squared_deviations_[0];
    state.rest_squared_deviations[1] = rest_squared_deviations_[1];
    state.rest_last_gyro_lp = rest_last_gyro_lp_;
    state.rest_last_accel_lp = rest_last_accel_lp_;
    state.rest_gyro_low_pass = rest_gyro_low_pass_.get_internal_state();
    state.rest_accel_low_pass = rest_accel_low_pass_.get_internal_state();
    state.update_count = update_count_;
    state.valid = valid_;
    return state;
}

bool Vqf::set_internal_state(const InternalState &state) noexcept
{
    if (!initialized_)
    {
        return false;
    }

    // 两阶段恢复：先整体校验（不写任何状态），全部通过后再写入，
    // 避免出现"部分子状态已更新、其余仍是旧值"的混合态。
    if (!same_config(state.config, config_) || state.gyro_period_seconds != gyro_period_ ||
        state.accel_period_seconds != accel_period_ ||
        !accel_low_pass_.is_internal_state_compatible(state.accel_low_pass) ||
        !motion_bias_rotation_low_pass_.is_internal_state_compatible(state.motion_bias_rotation_low_pass) ||
        !motion_bias_projection_low_pass_.is_internal_state_compatible(state.motion_bias_projection_low_pass) ||
        !rest_gyro_low_pass_.is_internal_state_compatible(state.rest_gyro_low_pass) ||
        !rest_accel_low_pass_.is_internal_state_compatible(state.rest_accel_low_pass))
    {
        return false;
    }

    (void)accel_low_pass_.set_internal_state(state.accel_low_pass);
    (void)motion_bias_rotation_low_pass_.set_internal_state(state.motion_bias_rotation_low_pass);
    (void)motion_bias_projection_low_pass_.set_internal_state(state.motion_bias_projection_low_pass);
    (void)rest_gyro_low_pass_.set_internal_state(state.rest_gyro_low_pass);
    (void)rest_accel_low_pass_.set_internal_state(state.rest_accel_low_pass);

    gyro_quat_ = state.gyro_quaternion;
    accel_quat_ = state.accel_quaternion;
    last_accel_lp_ = state.last_accel_lp;
    last_accel_correction_radians_ = state.last_accel_correction_radians;
    gyro_bias_ = state.gyro_bias_rad_per_sec;
    bias_covariance_ = state.bias_covariance;
    rest_detected_ = state.rest_detected;
    rest_t_ = state.rest_seconds;
    rest_squared_deviations_[0] = state.rest_squared_deviations[0];
    rest_squared_deviations_[1] = state.rest_squared_deviations[1];
    rest_last_gyro_lp_ = state.rest_last_gyro_lp;
    rest_last_accel_lp_ = state.rest_last_accel_lp;
    update_count_ = state.update_count;
    valid_ = state.valid;
    return true;
}

Vqf::State Vqf::get_state() const noexcept
{
    State state{};
    state.quaternion = get_quaternion();
    state.gyro_bias_rad_per_sec = gyro_bias_;
    state.euler_radians = alg_math::to_euler_zyx_radians(state.quaternion);
    state.euler_degrees = alg_math::Vector3{alg_math::rad_to_deg(state.euler_radians.x),
                                            alg_math::rad_to_deg(state.euler_radians.y),
                                            alg_math::rad_to_deg(state.euler_radians.z)};
    state.last_accel_correction_radians = last_accel_correction_radians_;
    state.rest_detected = rest_detected_;
    state.valid = valid_;
    state.update_count = update_count_;

    // 零偏不确定度：用最大绝对行和作为最大特征值的上界（Gershgorin）
    // 从 0.01 deg/s 换算到 rad/s。
    const double row1 = std::fabs(bias_covariance_.value[0]) + std::fabs(bias_covariance_.value[1]) +
                        std::fabs(bias_covariance_.value[2]);
    const double row2 = std::fabs(bias_covariance_.value[3]) + std::fabs(bias_covariance_.value[4]) +
                        std::fabs(bias_covariance_.value[5]);
    const double row3 = std::fabs(bias_covariance_.value[6]) + std::fabs(bias_covariance_.value[7]) +
                        std::fabs(bias_covariance_.value[8]);
    const double bound = std::min(std::max(std::max(row1, row2), row3), bias_p0_);
    state.bias_uncertainty_rad_per_sec =
        static_cast<float>(std::sqrt(bound) * static_cast<double>(alg_math::PI) / 100.0 / 180.0);
    return state;
}

} // namespace alg_estimate
