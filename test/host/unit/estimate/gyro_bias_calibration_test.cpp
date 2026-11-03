// estimate::GyroBiasCalibration：温度门控与稳定门控、静止窗口、
// 被碰、缓慢转动、加速度模长、超时预算跨窗口累计、温度出带暂停预算、时间戳回绕、终态不变、参数检查。
#include "lib/estimate/gyro_bias_calibration.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>

namespace
{

using estimate::GyroBiasCalibration;
using State = GyroBiasCalibration::State;

constexpr std::uint32_t PERIOD_US{500U}; // 陀螺 2 kHz
constexpr float GRAVITY{9.80665F};
const math::Vector3 STILL_ACCEL{0.0F, 0.0F, 9.7F}; // 旧板实测 |a| ≈ 9.706

// 线性同余的伪随机噪声，结果可复现；幅度 ±amplitude
struct Noise
{
    std::uint32_t seed{12345U};

    float next(float amplitude)
    {
        seed = seed * 1664525U + 1013904223U;
        const float unit = static_cast<float>(seed >> 8U) / 16777216.0F; // [0, 1)
        return (2.0F * unit - 1.0F) * amplitude;
    }
};

GyroBiasCalibration::Config fast_config()
{
    GyroBiasCalibration::Config config{};
    config.temperature_settle_us = 2000000U; // 2 s
    config.window_us = 1000000U;             // 1 s
    config.timeout_us = 4000000U;            // 4 s
    return config;
}

struct Feeder
{
    GyroBiasCalibration &calibration;
    std::uint32_t now_us;
    Noise noise{};

    // 喂 duration_us 的样本：零偏 + 噪声（±0.4 rad/s，与板上单样本噪声同量级）
    void feed(std::uint32_t duration_us,
              const math::Vector3 &rate,
              float temperature,
              const math::Vector3 &accel = STILL_ACCEL,
              float noise_amplitude = 0.4F)
    {
        for (std::uint32_t elapsed = 0U; elapsed < duration_us; elapsed += PERIOD_US)
        {
            const math::Vector3 sample{rate.x + noise.next(noise_amplitude),
                                       rate.y + noise.next(noise_amplitude),
                                       rate.z + noise.next(noise_amplitude)};
            calibration.process(sample, accel, temperature, now_us);
            now_us += PERIOD_US;
        }
    }
};

const math::Vector3 BIAS{0.01F, -0.009F, 0.003F}; // 量级同旧板实测（y 轴 -0.52 °/s）

void completes_after_settling_and_a_still_window()
{
    GyroBiasCalibration calibration;
    assert(calibration.init(fast_config()));
    Feeder feeder{calibration, 1000U};

    // 没有温度读数、温度不在带内：一直等
    feeder.feed(3000000U, BIAS, std::numeric_limits<float>::quiet_NaN());
    assert(calibration.state() == State::WAITING_FOR_TEMPERATURE);
    feeder.feed(3000000U, BIAS, 48.0F);
    assert(calibration.state() == State::WAITING_FOR_TEMPERATURE);

    // 进带但稳定不够 2 s：还在等
    feeder.feed(1900000U, BIAS, 50.0F);
    assert(calibration.state() == State::WAITING_FOR_TEMPERATURE);
    assert(calibration.snapshot().temperature_hold_us >= 1800000U);
    feeder.feed(200000U, BIAS, 50.125F); // 带内极差 0.125 ℃，不超过 0.25：继续计时
    assert(calibration.state() == State::SAMPLING);

    // 1 s 静止窗口
    feeder.feed(1100000U, BIAS, 50.0F);
    assert(calibration.state() == State::COMPLETED && calibration.is_bias_valid());
    const math::Vector3 bias = calibration.bias_rad_per_sec();
    // 2000 个样本、单样本 ±0.4 均匀噪声：均值误差约 0.4/√3/√2000 ≈ 0.005，取 4 倍
    assert(std::fabs(bias.x - BIAS.x) < 0.02F && std::fabs(bias.y - BIAS.y) < 0.02F &&
           std::fabs(bias.z - BIAS.z) < 0.02F);
    assert(calibration.snapshot().restarts == 0U);

    // 终态：之后的样本不改结果
    feeder.feed(1000000U, math::Vector3{1.0F, 1.0F, 1.0F}, 60.0F);
    assert(calibration.state() == State::COMPLETED);
    assert(calibration.bias_rad_per_sec().x == bias.x);
}

void temperature_must_hold_still()
{
    GyroBiasCalibration calibration;
    assert(calibration.init(fast_config()));
    Feeder feeder{calibration, 0U};
    // 升温尾巴：带内但每 0.5 s 涨 0.125 ℃，极差很快超过 0.25，稳定计时一再重来
    float temperature = 49.6F;
    for (int step = 0; step < 6; ++step)
    {
        feeder.feed(500000U, BIAS, temperature);
        temperature += 0.125F;
    }
    assert(calibration.state() == State::WAITING_FOR_TEMPERATURE);
    assert(calibration.snapshot().temperature_hold_us < 2000000U);
    // 出带：稳定计时清零，回带后重新计
    const std::uint32_t held = calibration.snapshot().temperature_hold_us;
    assert(held >= 1000000U);
    feeder.feed(10000U, BIAS, 51.0F);
    assert(calibration.snapshot().temperature_hold_us == 0U);
    feeder.feed(1000000U, BIAS, 50.0F);
    assert(calibration.snapshot().temperature_hold_us < held);
    assert(calibration.state() == State::WAITING_FOR_TEMPERATURE);
}

void motion_restarts_the_window()
{
    GyroBiasCalibration calibration;
    assert(calibration.init(fast_config()));
    Feeder feeder{calibration, 0U};
    feeder.feed(2100000U, BIAS, 50.0F);
    assert(calibration.state() == State::SAMPLING);

    // 被碰：一个样本的极差超过 1 rad/s
    feeder.feed(300000U, BIAS, 50.0F);
    calibration.process(math::Vector3{1.5F, 0.0F, 0.0F}, STILL_ACCEL, 50.0F, feeder.now_us);
    feeder.now_us += PERIOD_US;
    assert(calibration.snapshot().restarts == 1U && calibration.state() == State::SAMPLING);

    // 缓慢转动：均值 0.1 rad/s 超过 0.05，第 512 个样本的部分均值检查抓到
    feeder.feed(400000U, math::Vector3{0.1F, 0.0F, 0.0F}, 50.0F, STILL_ACCEL, 0.05F);
    assert(calibration.snapshot().restarts == 2U);

    // 加速度模长偏离重力（被拿起来晃）
    feeder.feed(100000U, BIAS, 50.0F, math::Vector3{0.0F, 0.0F, GRAVITY + 1.0F});
    assert(calibration.snapshot().samples == 0U);

    // 静下来之后照样完成
    feeder.feed(1100000U, BIAS, 50.0F);
    assert(calibration.state() == State::COMPLETED);
}

void times_out_when_never_still()
{
    GyroBiasCalibration calibration;
    assert(calibration.init(fast_config()));
    Feeder feeder{calibration, 0xFFFFFFFFU - 3000000U}; // 期间跨过时间戳回绕
    feeder.feed(2100000U, BIAS, 50.0F);
    assert(calibration.state() == State::SAMPLING);
    // 一直在转：0.2 rad/s
    feeder.feed(2000000U, math::Vector3{0.0F, 0.0F, 0.2F}, 50.0F);
    assert(calibration.state() == State::SAMPLING);
    // 温度出带：预算暂停，回到等待温度
    feeder.feed(3000000U, math::Vector3{0.0F, 0.0F, 0.2F}, 47.0F);
    assert(calibration.state() == State::WAITING_FOR_TEMPERATURE);
    const std::uint32_t used = calibration.snapshot().sampling_us;
    assert(used >= 1900000U && used <= 2100000U);
    // 回带、重新稳定后继续花剩下的预算：累计 4 s 超时
    feeder.feed(2100000U, math::Vector3{0.0F, 0.0F, 0.2F}, 50.0F);
    feeder.feed(2100000U, math::Vector3{0.0F, 0.0F, 0.2F}, 50.0F);
    assert(calibration.state() == State::TIMED_OUT && !calibration.is_bias_valid());
    feeder.feed(2000000U, BIAS, 50.0F);
    assert(calibration.state() == State::TIMED_OUT); // 终态
}

void rejects_bad_config_and_resets()
{
    GyroBiasCalibration calibration;
    GyroBiasCalibration::Config config = fast_config();
    config.window_us = 0U;
    assert(!calibration.init(config));
    config = fast_config();
    config.maximum_spread_rad_per_sec = std::numeric_limits<float>::quiet_NaN();
    assert(!calibration.init(config));
    config = fast_config();
    config.temperature_tolerance_celsius = -1.0F;
    assert(!calibration.init(config));

    assert(calibration.init(fast_config()));
    Feeder feeder{calibration, 0U};
    feeder.feed(3200000U, BIAS, 50.0F);
    assert(calibration.state() == State::COMPLETED);
    calibration.reset();
    assert(calibration.state() == State::WAITING_FOR_TEMPERATURE && !calibration.is_bias_valid());
    assert(calibration.snapshot().restarts == 0U);
}

} // namespace

int main()
{
    completes_after_settling_and_a_still_window();
    temperature_must_hold_still();
    motion_restarts_the_window();
    times_out_when_never_still();
    rejects_bad_config_and_resets();
    return 0;
}
