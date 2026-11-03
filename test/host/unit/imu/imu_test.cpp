// imu::Imu（Decision-20261103-01 第 9 点）在 platform/mock 上：BMI088 由 SPI 替身的应答函数扮演，
// 数据就绪中断由 GPIO 替身以 2 kHz（陀螺）/ 400 Hz（加速度计）触发，温度 50 ℃。
// 每个 topic 只能有一个发布者，场景按参数分进程运行：
//   ready    静止：SETTLING → 零偏标定完成 → READY；发布的角速度扣掉零偏，yaw 重新归零后保持在 0 附近；
//            安装方向把传感器 z 轴换到云台 x 轴
//   degraded 一直在转：标定超时 → DEGRADED，yaw 零点同样重取
//   reject   参数检查：没有加热器、安装方向无效、发布间隔为 0
#include "imu/imu.hpp"
#include "mock_hal.hpp"
#include "mock_os.hpp"
#include "topics/imu.hpp"
#include "utils/math/quaternion.hpp"

#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace
{

constexpr std::uint8_t ACCEL{0U};
constexpr std::uint8_t GYRO{1U};
constexpr std::uint8_t BUS{0U};
constexpr std::uint8_t ACCEL_PIN{3U};
constexpr std::uint8_t GYRO_PIN{4U};
constexpr std::uint8_t HEATER{1U};
constexpr float DEG{3.14159265F / 180.0F};
constexpr float GYRO_LSB_RAD{2000.0F / 32768.0F * DEG}; // 量程 ±2000 °/s

std::uint8_t g_registers[2][128]{};

void respond(void *, std::uint8_t device, const std::uint8_t *transmit, std::uint8_t *receive, std::size_t length)
{
    const auto address = transmit[0] & 0x7f;
    if ((transmit[0] & 0x80) != 0)
    {
        const std::size_t skip = device == ACCEL ? 2U : 1U;
        for (std::size_t i = skip; i < length; ++i)
        {
            receive[i] = g_registers[device][address + i - skip];
        }
    }
    else
    {
        g_registers[device][address] = transmit[1];
    }
}

void put16(std::uint8_t device, std::uint8_t address, std::int16_t value)
{
    g_registers[device][address] = static_cast<std::uint8_t>(static_cast<std::uint16_t>(value) & 0xFFU);
    g_registers[device][address + 1] = static_cast<std::uint8_t>(static_cast<std::uint16_t>(value) >> 8U);
}

// 陀螺原始值（传感器坐标系，LSB）
void set_gyro(std::int16_t x, std::int16_t y, std::int16_t z)
{
    put16(GYRO, 0x02, x);
    put16(GYRO, 0x04, y);
    put16(GYRO, 0x06, z);
}

void step_and_finish_transfers(imu::Imu &capability)
{
    capability.step(hal::os::now_us());
    while (mock::spi::busy(BUS))
    {
        mock::advance_us(20U);
        assert(mock::spi::complete(BUS));
        capability.step(hal::os::now_us());
    }
}

void setup_sensor()
{
    mock::reset();
    mock::spi::reset();
    mock::gpio::reset();
    mock::pwm::reset();
    mock::spi::set_bus(GYRO, BUS);
    mock::spi::set_responder(ACCEL, respond, nullptr);
    mock::spi::set_responder(GYRO, respond, nullptr);
    g_registers[ACCEL][0x00] = device::Bmi088Accel::EXPECTED_CHIP_ID;
    g_registers[GYRO][0x00] = device::Bmi088Gyro::EXPECTED_CHIP_ID;
    g_registers[ACCEL][0x22] = 27; // 温度：23 + 216 × 0.125 = 50 ℃
    put16(ACCEL, 0x12, 0);         // 加速度：z 约 1 g（量程 ±3 g：1 g ≈ 10923）
    put16(ACCEL, 0x14, 0);
    put16(ACCEL, 0x16, 10923);
    set_gyro(0, 0, 0);
}

imu::Imu::Parameters fast_parameters(const estimate::AxisMapping &mounting)
{
    imu::Imu::Parameters parameters{};
    parameters.driver = device::Bmi088::Config{device::Bmi088Gyro::Bandwidth::ODR_2000_HZ_532_HZ};
    parameters.mounting = mounting;
    parameters.publish_period_us = 1000U;
    parameters.heater_target_celsius = 50.0F;
    parameters.calibration.temperature_settle_us = 300000U; // 测试缩短：0.3 s 稳定、0.5 s 窗口、1.5 s 超时
    parameters.calibration.window_us = 500000U;
    parameters.calibration.timeout_us = 1500000U;
    parameters.calibrated_bias_uncertainty_rad_per_sec = 0.001F;
    return parameters;
}

device::Bmi088 g_sensor{hal::Spi{ACCEL}, hal::Spi{GYRO}};
device::Bmi088Heater g_heater{hal::Pwm{HEATER},
                              device::Bmi088Heater::Config{control::HeaterControl::Model{0.967F, 15.5F, 0.95F}}};

// 驱动初始化：按时间推进（任务的 1 ms 超时）
void bring_up(imu::Imu &capability)
{
    for (int round = 0; round < 1000 && !capability.report().sensor_ready; ++round)
    {
        mock::advance_us(1000U);
        step_and_finish_transfers(capability);
    }
    assert(capability.report().sensor_ready && !capability.report().sensor_error);
}

// 以 2 kHz 陀螺、400 Hz 加速度计跑 duration_us；gyro_z 每个陀螺样本可以变
template <typename GyroZ>
void run(imu::Imu &capability, std::uint32_t duration_us, GyroZ gyro_z)
{
    for (std::uint32_t elapsed = 0U, edge = 0U; elapsed < duration_us; elapsed += 500U, ++edge)
    {
        gyro_z(edge);
        mock::advance_us(470U);
        assert(mock::gpio::trigger_interrupt(GYRO_PIN));
        if (edge % 5U == 0U)
        {
            assert(mock::gpio::trigger_interrupt(ACCEL_PIN));
        }
        mock::advance_us(30U);
        step_and_finish_transfers(capability);
    }
}

// 同上，但中断时刻在名义时刻之后 0–50 µs 抖动，处理延迟在 10–60 µs 之间变（平均间隔仍是 500 µs）。
// 第 k 个中断在 500k + JITTER[k]；上一轮处理完时在 500(k-1) + JITTER[k-1] + DELAY[k-1]，时间只往前走
void run_jittered(imu::Imu &capability, std::uint32_t duration_us)
{
    static constexpr std::uint32_t JITTER[]{0U, 25U, 5U, 45U, 15U, 35U, 50U, 10U};
    static constexpr std::uint32_t DELAY[]{30U, 10U, 60U, 20U, 50U, 40U, 15U, 55U};
    std::uint32_t previous_end = 0U; // 上一轮处理完时相对名义时刻的偏移
    for (std::uint32_t elapsed = 0U, edge = 0U; elapsed < duration_us; elapsed += 500U, ++edge)
    {
        const std::uint32_t jitter = JITTER[edge % 8U];
        const std::uint32_t delay = DELAY[edge % 8U];
        mock::advance_us(500U + jitter - previous_end);
        assert(mock::gpio::trigger_interrupt(GYRO_PIN));
        if (edge % 5U == 0U)
        {
            assert(mock::gpio::trigger_interrupt(ACCEL_PIN));
        }
        mock::advance_us(delay);
        const hal::os::Microseconds before = hal::os::now_us();
        step_and_finish_transfers(capability);
        previous_end = jitter + delay + (hal::os::now_us() - before); // SPI 事务也花时间
    }
}

float yaw_of(const topics::imu::Attitude &attitude)
{
    const math::Quaternion q{
        attitude.quaternion[0], attitude.quaternion[1], attitude.quaternion[2], attitude.quaternion[3]};
    return math::to_euler_zyx_radians(q).x;
}

int scenario_ready()
{
    setup_sensor();
    // 恒等安装：验证 READY、扣零偏与 yaw 归零（安装方向在 degraded 场景里验）
    imu::Imu capability;
    assert(capability.init(imu::Imu::Config{&g_sensor,
                                            &g_heater,
                                            hal::Gpio{ACCEL_PIN},
                                            hal::Gpio{GYRO_PIN},
                                            fast_parameters(estimate::IDENTITY_AXIS_MAPPING)}));
    topic::Subscriber<topics::imu::Attitude> attitude_topic{topics::imu::attitude};
    bring_up(capability);

    // 零偏：z 轴 +17 LSB（约 0.018 rad/s，1.04 °/s），x 轴 -8 LSB
    const float bias_z = 17.0F * GYRO_LSB_RAD;
    const float bias_x = -8.0F * GYRO_LSB_RAD;
    set_gyro(-8, 0, 17);
    run(capability, 200000U, [](std::uint32_t) {});
    topics::imu::Attitude attitude{};
    assert(attitude_topic.update(attitude));
    assert(attitude.state == topics::imu::State::SETTLING && attitude.valid);
    assert(capability.report().state == topics::imu::State::SETTLING);
    // SETTLING 时 VQF 还没拿到标定的零偏，yaw 随零偏漂移
    run(capability, 400000U, [](std::uint32_t) {});
    assert(attitude_topic.update(attitude));
    assert(attitude.state == topics::imu::State::SETTLING);
    const float drift_before = yaw_of(attitude);
    assert(drift_before > 0.003F); // 0.6 s × 0.018 rad/s 的量级（VQF 的静止零偏估计会吃掉一部分）

    // 0.3 s 稳定 + 0.5 s 窗口之后 READY
    run(capability, 400000U, [](std::uint32_t) {});
    assert(capability.report().state == topics::imu::State::READY);
    assert(capability.report().calibration == estimate::GyroBiasCalibration::State::COMPLETED);
    const math::Vector3 calibrated = capability.report().calibrated_bias_rad_per_sec;
    assert(std::fabs(calibrated.z - bias_z) < 1e-4F && std::fabs(calibrated.x - bias_x) < 1e-4F);
    assert(capability.report().yaw_zero_rad > 0.003F && capability.report().settled_us != 0U);

    // READY 之后：角速度扣掉零偏接近 0；yaw 归零，并且不再漂
    run(capability, 50000U, [](std::uint32_t) {});
    assert(attitude_topic.update(attitude));
    assert(attitude.state == topics::imu::State::READY && attitude.valid);
    assert(std::fabs(attitude.gyro_rad_per_sec[2]) < 2e-4F && std::fabs(attitude.gyro_rad_per_sec[0]) < 2e-4F);
    assert(std::fabs(yaw_of(attitude)) < 2e-3F);
    run(capability, 2000000U, [](std::uint32_t) {});
    assert(attitude_topic.update(attitude));
    assert(std::fabs(yaw_of(attitude)) < 2e-3F); // 2 s 不漂

    // 真的转起来：z 轴 +100 LSB（叠加零偏）转 0.5 s，yaw 向左增加约 0.5 × 100 × LSB
    set_gyro(-8, 0, 17 + 100);
    run(capability, 500000U, [](std::uint32_t) {});
    assert(attitude_topic.update(attitude));
    const float expected = 0.5F * 100.0F * GYRO_LSB_RAD;
    assert(std::fabs(yaw_of(attitude) - expected) < 0.1F * expected);
    assert(std::fabs(attitude.gyro_rad_per_sec[2] - 100.0F * GYRO_LSB_RAD) < 1e-3F);
    // 采样时刻是陀螺数据就绪中断的时刻（每轮中断后 30 µs 处理）
    assert(attitude.timestamp_us + 30U <= hal::os::now_us());
    assert(capability.report().published > 1000U);

    // 发布频率：2 kHz 陀螺、1 ms 间隔，中断与处理时刻有抖动时仍是 1 kHz（2026-11-03 上板只有约 860 Hz）
    const std::uint32_t before = capability.report().published;
    run_jittered(capability, 1000000U);
    const std::uint32_t published = capability.report().published - before;
    assert(published >= 998U && published <= 1002U);
    std::puts("imu ready: 全部通过");
    return 0;
}

int scenario_degraded()
{
    setup_sensor();
    imu::Imu capability;
    // 安装方向：云台 x ← 传感器 y，云台 y ← 传感器 -x，云台 z ← 传感器 z（板子转了 90°）
    const estimate::AxisMapping rotated{{1U, 0U, 2U}, {1, -1, 1}};
    assert(capability.init(
        imu::Imu::Config{&g_sensor, &g_heater, hal::Gpio{ACCEL_PIN}, hal::Gpio{GYRO_PIN}, fast_parameters(rotated)}));
    topic::Subscriber<topics::imu::Attitude> attitude_topic{topics::imu::attitude};
    bring_up(capability);

    // 传感器 x 轴一直来回晃（±1.2 rad/s 交替，极差超过 1 rad/s）：标定窗口一直重启，1.5 s 后超时
    run(capability,
        2500000U,
        [](std::uint32_t edge)
        {
            const std::int16_t swing = static_cast<std::int16_t>(1.2F / GYRO_LSB_RAD);
            set_gyro(edge % 2U == 0U ? swing : static_cast<std::int16_t>(-swing), 0, 0);
        });
    assert(capability.report().state == topics::imu::State::DEGRADED);
    assert(capability.report().calibration == estimate::GyroBiasCalibration::State::TIMED_OUT);
    assert(capability.report().calibration_restarts > 10U);
    topics::imu::Attitude attitude{};
    assert(attitude_topic.update(attitude));
    assert(attitude.state == topics::imu::State::DEGRADED && attitude.valid);
    // 传感器 x 落到云台 -y：发布的角速度在 y 轴上
    assert(std::fabs(attitude.gyro_rad_per_sec[1]) > 1.0F && std::fabs(attitude.gyro_rad_per_sec[0]) < 0.05F);
    std::puts("imu degraded: 全部通过");
    return 0;
}

int scenario_reject()
{
    setup_sensor();
    imu::Imu capability;
    imu::Imu::Parameters parameters = fast_parameters(estimate::IDENTITY_AXIS_MAPPING);
    assert(!capability.init(
        imu::Imu::Config{&g_sensor, nullptr, hal::Gpio{ACCEL_PIN}, hal::Gpio{GYRO_PIN}, parameters})); // 没有加热器
    parameters.mounting = estimate::AxisMapping{{0U, 0U, 2U}, {1, 1, 1}};
    assert(!capability.init(imu::Imu::Config{
        &g_sensor, &g_heater, hal::Gpio{ACCEL_PIN}, hal::Gpio{GYRO_PIN}, parameters})); // 安装方向不是旋转
    parameters = fast_parameters(estimate::IDENTITY_AXIS_MAPPING);
    parameters.publish_period_us = 0U;
    assert(!capability.init(
        imu::Imu::Config{&g_sensor, &g_heater, hal::Gpio{ACCEL_PIN}, hal::Gpio{GYRO_PIN}, parameters}));
    parameters = fast_parameters(estimate::IDENTITY_AXIS_MAPPING);
    parameters.calibration.window_us = 0U;
    assert(!capability.init(
        imu::Imu::Config{&g_sensor, &g_heater, hal::Gpio{ACCEL_PIN}, hal::Gpio{GYRO_PIN}, parameters}));
    std::puts("imu reject: 全部通过");
    return 0;
}

} // namespace

int main(int argc, char **argv)
{
    if (argc == 2 && std::strcmp(argv[1], "ready") == 0)
    {
        return scenario_ready();
    }
    if (argc == 2 && std::strcmp(argv[1], "degraded") == 0)
    {
        return scenario_degraded();
    }
    if (argc == 2 && std::strcmp(argv[1], "reject") == 0)
    {
        return scenario_reject();
    }
    std::fprintf(stderr, "用法：%s ready|degraded|reject\n", argv[0]);
    return 2;
}
