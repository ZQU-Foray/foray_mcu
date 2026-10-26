#include "Application/Imu/AttitudeEstimator.hpp"
#include "Libraries/Algorithm/alg_estimate/ImuBodyMapping.hpp"
#include "Libraries/Algorithm/alg_estimate/Vqf.hpp"
#include "Platform/Interface/Time.hpp"
#include <cmath>
#include <cstddef>

namespace application
{

namespace
{

alg_estimate::Vqf g_estimator;
GyroBiasCalibration g_calibration;
bool g_ready{false};
bool g_calibration_injected{false};
std::int32_t g_yaw_turns{0};
float g_last_yaw_degrees{0.0F};
bool g_yaw_anchored{false};

// 时序状态：陀螺节拍与最近的加速度样本。
bool g_gyro_anchored{false};
std::uint32_t g_last_gyro_tick{0U};
std::uint32_t g_last_gyro_sequence{0U};
std::uint32_t g_last_accel_sequence{0U};
std::uint32_t g_last_accel_tick{0U};
bool g_accel_available{false};
alg_math::Vector3 g_accel_body{};

// 调试器可见存档。写读都走字节拷贝，避免逐字段复制漏项。
volatile AttitudeEstimator::Output g_output_archive{};
volatile AttitudeEstimator::Statistics g_statistics_archive{};
static_assert(sizeof(AttitudeEstimator::Output) == sizeof(g_output_archive), "存档布局必须一致");

// 整字访问必须允许别名到浮点字段，否则优化构建可能读到过期状态。
typedef std::uint32_t ArchiveWord __attribute__((__may_alias__));

void publish_archive(const AttitudeEstimator::Output &source) noexcept
{
    // 整字拷贝：本函数在 2 kHz 陀螺节拍上执行，逐字节写 volatile 会让固定开销翻几倍。
    // Output 全由 32 位成员组成（sizeof 已被 static_assert 钉死为 120），对齐有保证。
    static_assert((sizeof(AttitudeEstimator::Output) % sizeof(std::uint32_t)) == 0U, "存档布局必须能按 32 位整字拷贝");
    const ArchiveWord *source_words = reinterpret_cast<const ArchiveWord *>(&source);
    volatile ArchiveWord *target_words = reinterpret_cast<volatile ArchiveWord *>(&g_output_archive);
    for (std::size_t index = 0U; index < sizeof(source) / sizeof(ArchiveWord); ++index)
    {
        target_words[index] = source_words[index];
    }
}

AttitudeEstimator::Output read_archive() noexcept
{
    AttitudeEstimator::Output result{};
    ArchiveWord *target_words = reinterpret_cast<ArchiveWord *>(&result);
    const volatile ArchiveWord *source_words = reinterpret_cast<const volatile ArchiveWord *>(&g_output_archive);
    for (std::size_t index = 0U; index < sizeof(result) / sizeof(ArchiveWord); ++index)
    {
        target_words[index] = source_words[index];
    }
    return result;
}

void reset_archives() noexcept
{
    AttitudeEstimator::Output output{};
    publish_archive(output);
    AttitudeEstimator::Statistics statistics{};
    const std::uint8_t *bytes = reinterpret_cast<const std::uint8_t *>(&statistics);
    volatile std::uint8_t *target = reinterpret_cast<volatile std::uint8_t *>(&g_statistics_archive);
    for (std::size_t index = 0U; index < sizeof(statistics); ++index)
    {
        target[index] = bytes[index];
    }
}

/** 时间戳标志不确定的样本（晚启动或完成前又来了新的 DRDY）视为时序不可信。 */
bool has_uncertain_timing(const device::Bmi088SampleRecord &record)
{
    return (record.flags &
            (device::Bmi088SampleRecord::LATE_START | device::Bmi088SampleRecord::NEWER_EVENT_BEFORE_COMPLETION)) != 0U;
}

void publish_output(const device::Bmi088SampleRecord &record)
{
    const alg_estimate::Vqf::State state = g_estimator.get_state();
    const alg_estimate::Vqf::RestObservables rest = g_estimator.get_rest_observables();
    const GyroBiasCalibration::Snapshot calibration_snapshot = g_calibration.get_snapshot();
    const bool valid = state.valid && alg_math::is_finite(state.quaternion) &&
                       alg_math::is_finite(state.gyro_bias_rad_per_sec) && alg_math::is_finite(state.euler_degrees) &&
                       std::isfinite(state.bias_uncertainty_rad_per_sec);

    if (valid)
    {
        if (g_yaw_anchored)
        {
            const float delta = state.euler_degrees.x - g_last_yaw_degrees;
            if (delta > 180.0F)
            {
                --g_yaw_turns;
            }
            else if (delta < -180.0F)
            {
                ++g_yaw_turns;
            }
        }
        g_yaw_anchored = true;
        g_last_yaw_degrees = state.euler_degrees.x;
    }

    AttitudeEstimator::Output output{};
    output.version = AttitudeEstimator::OUTPUT_VERSION;
    output.update_tick = record.drdy_tick;
    output.quaternion[0] = state.quaternion.w;
    output.quaternion[1] = state.quaternion.x;
    output.quaternion[2] = state.quaternion.y;
    output.quaternion[3] = state.quaternion.z;
    output.euler_degrees[0] = state.euler_degrees.x;
    output.euler_degrees[1] = state.euler_degrees.y;
    output.euler_degrees[2] = state.euler_degrees.z;
    output.gyro_bias_rad_per_sec[0] = state.gyro_bias_rad_per_sec.x;
    output.gyro_bias_rad_per_sec[1] = state.gyro_bias_rad_per_sec.y;
    output.gyro_bias_rad_per_sec[2] = state.gyro_bias_rad_per_sec.z;
    output.yaw_total_degrees = 360.0F * static_cast<float>(g_yaw_turns) + g_last_yaw_degrees;
    output.bias_uncertainty_rad_per_sec = state.bias_uncertainty_rad_per_sec;
    output.update_count = state.update_count;
    output.gyroscope_deviation_ratio = rest.gyroscope_deviation_ratio;
    output.valid = valid ? 1U : 0U;
    output.rest_detected = state.rest_detected ? 1U : 0U;
    output.last_accel_correction_radians = state.last_accel_correction_radians;
    output.accelerometer_deviation_ratio = rest.accelerometer_deviation_ratio;
    output.calibration_bias_rad_per_sec[0] = calibration_snapshot.bias_rad_per_sec.x;
    output.calibration_bias_rad_per_sec[1] = calibration_snapshot.bias_rad_per_sec.y;
    output.calibration_bias_rad_per_sec[2] = calibration_snapshot.bias_rad_per_sec.z;
    output.calibration_spread_rad_per_sec[0] = calibration_snapshot.spread_rad_per_sec.x;
    output.calibration_spread_rad_per_sec[1] = calibration_snapshot.spread_rad_per_sec.y;
    output.calibration_spread_rad_per_sec[2] = calibration_snapshot.spread_rad_per_sec.z;
    output.calibration_state = static_cast<std::uint32_t>(calibration_snapshot.state);
    output.calibration_samples = calibration_snapshot.samples;
    output.calibration_temperature_celsius = calibration_snapshot.temperature_celsius;
    output.calibration_elapsed_ms = calibration_snapshot.elapsed_ms;
    publish_archive(output);

    g_statistics_archive.calibration_restarts = calibration_snapshot.restarts;
}

void process_gyroscope(const device::Bmi088SampleRecord &record, float temperature_celsius) noexcept
{
    if (record.sequence <= g_last_gyro_sequence)
    {
        ++g_statistics_archive.rejected_sequence;
        return;
    }
    g_last_gyro_sequence = record.sequence;

    if (!g_gyro_anchored)
    {
        // 首个陀螺样本只建立时间基准，不积分。
        g_last_gyro_tick = record.drdy_tick;
        g_gyro_anchored = true;
        return;
    }

    const std::uint32_t elapsed_ticks = record.drdy_tick - g_last_gyro_tick;
    g_last_gyro_tick = record.drdy_tick;
    const float step_seconds = platform::Time::ticks_to_seconds(elapsed_ticks);
    if (!(step_seconds > 0.0F) || step_seconds > AttitudeEstimator::GYRO_GAP_LIMIT_SECONDS)
    {
        // 断流：不虚构缺失运动，仅重新建立基准。
        ++g_statistics_archive.gyro_gaps;
        return;
    }

    float gyro_body[3]{};
    alg_estimate::gyro_record_to_body(record, gyro_body);
    const alg_math::Vector3 gyro{gyro_body[0], gyro_body[1], gyro_body[2]};

    const alg_math::Vector3 gate_accel = g_accel_available ? g_accel_body : alg_math::Vector3{};
    g_calibration.process(gyro, gate_accel, temperature_celsius, record.drdy_tick);
    if (!g_calibration_injected && g_calibration.is_bias_valid())
    {
        g_estimator.set_bias_estimate(g_calibration.get_bias_rad_per_sec());
        g_calibration_injected = true;
    }

    if (!g_accel_available || platform::Time::ticks_to_seconds(record.drdy_tick - g_last_accel_tick) >
                                  AttitudeEstimator::ACCELEROMETER_STALE_LIMIT_SECONDS)
    {
        ++g_statistics_archive.accel_stale;
    }
    g_estimator.update_gyroscope(gyro);
    ++g_statistics_archive.gyro_consumed;
    publish_output(record);
}

void process_accelerometer(const device::Bmi088SampleRecord &record) noexcept
{
    if (record.sequence <= g_last_accel_sequence)
    {
        ++g_statistics_archive.rejected_sequence;
        return;
    }
    g_last_accel_sequence = record.sequence;
    g_last_accel_tick = record.drdy_tick;
    float accel[3]{};
    alg_estimate::accel_record_to_body(record, accel);
    g_accel_body = alg_math::Vector3{accel[0], accel[1], accel[2]};
    g_accel_available = true;
    g_estimator.update_accelerometer(g_accel_body);
    ++g_statistics_archive.accel_consumed;
}

} // namespace

bool AttitudeEstimator::init() noexcept
{
    GyroBiasCalibration::Config calibration_config{};
    calibration_config.tick_frequency_hz = platform::Time::tick_frequency_hz();
    g_calibration.init(calibration_config);

    alg_estimate::Vqf::Config config{};
    // 532 Hz 带宽下本板静止三轴瞬时偏离高于上游 2°/s 默认门槛；先按板测候选值验收。
    config.rest_threshold_gyr_rad_per_sec = 0.35F;
    // 标定允许的三轴均值不能在 VQF 首次零偏更新时被限幅截断。
    config.bias_clip_rad_per_sec = calibration_config.maximum_mean_rate_rad_per_sec;
    constexpr float GYRO_PERIOD_SECONDS =
        device::Bmi088Gyro::BANDWIDTH_REGISTER_VALUE == device::Bmi088Gyro::GYRO_BANDWIDTH_2000_HZ_532_HZ
            ? 1.0F / 2000.0F
            : 1.0F / 1000.0F;
    g_ready = g_estimator.init(config, GYRO_PERIOD_SECONDS, 1.0F / 1600.0F);

    g_calibration_injected = false;
    g_yaw_turns = 0;
    g_last_yaw_degrees = 0.0F;
    g_yaw_anchored = false;
    g_gyro_anchored = false;
    g_last_gyro_tick = 0U;
    g_last_gyro_sequence = 0U;
    g_last_accel_sequence = 0U;
    g_last_accel_tick = 0U;
    g_accel_available = false;
    g_accel_body = alg_math::Vector3{};
    reset_archives();
    return g_ready;
}

bool AttitudeEstimator::is_ready() noexcept
{
    return g_ready;
}

void AttitudeEstimator::process(const device::Bmi088SampleRecord &record,
                                bool gyroscope,
                                float temperature_celsius) noexcept
{
    if (gyroscope)
    {
        ++g_statistics_archive.gyro_received;
    }
    else
    {
        ++g_statistics_archive.accel_received;
    }

    if (!g_ready)
    {
        return;
    }
    if (has_uncertain_timing(record))
    {
        ++g_statistics_archive.rejected_flags;
        return;
    }

    if (gyroscope)
    {
        process_gyroscope(record, temperature_celsius);
    }
    else
    {
        process_accelerometer(record);
    }
}

AttitudeEstimator::Output AttitudeEstimator::get_output() noexcept
{
    return read_archive();
}

AttitudeEstimator::Statistics AttitudeEstimator::get_statistics() noexcept
{
    Statistics result{};
    std::uint8_t *bytes = reinterpret_cast<std::uint8_t *>(&result);
    const volatile std::uint8_t *source = reinterpret_cast<const volatile std::uint8_t *>(&g_statistics_archive);
    for (std::size_t index = 0U; index < sizeof(result); ++index)
    {
        bytes[index] = source[index];
    }
    return result;
}

} // namespace application
