// bringup::ImuCheck 在 platform/mock 上：BMI088 由 SPI 替身的应答函数扮演，
// 数据就绪中断由 GPIO 替身触发。验证：中断里发布 bringup.imu_event、驱动初始化、样本换算与 VQF、
// bringup.imu 的发布与限频、加热器启动、延迟统计。
#include "bringup/imu_check.hpp"
#include "mock_hal.hpp"
#include "mock_os.hpp"
#include "topics/bringup.hpp"

#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace
{

constexpr std::uint8_t ACCEL{0U};
constexpr std::uint8_t GYRO{1U};
constexpr std::uint8_t BUS{0U};
constexpr std::uint8_t ACCEL_PIN{3U};
constexpr std::uint8_t GYRO_PIN{4U};
constexpr std::uint8_t HEATER{1U};

std::uint8_t g_registers[2][128]{};

void respond(void *, std::uint8_t device, const std::uint8_t *transmit, std::uint8_t *receive, std::size_t length)
{
    const auto address = transmit[0] & 0x7f;
    if ((transmit[0] & 0x80) != 0)
    {
        const std::size_t skip = device == ACCEL ? 2U : 1U;
        for (std::size_t i = skip; i < length; ++i)
            receive[i] = g_registers[device][address + i - skip];
    }
    else
        g_registers[device][address] = transmit[1];
}

void put16(std::uint8_t device, std::uint8_t address, std::int16_t value)
{
    g_registers[device][address] = static_cast<std::uint8_t>(static_cast<std::uint16_t>(value) & 0xFFU);
    g_registers[device][address + 1] = static_cast<std::uint8_t>(static_cast<std::uint16_t>(value) >> 8U);
}

// 跑一轮：推进时间、step、把进行中的 SPI 事务做完（完成中断里会再发布 imu_event）
void cycle(bringup::ImuCheck &check, hal::os::Microseconds advance)
{
    mock::advance_us(advance);
    check.step(hal::os::now_us());
    while (mock::spi::busy(BUS))
    {
        mock::advance_us(20U);
        assert(mock::spi::complete(BUS));
        check.step(hal::os::now_us());
    }
}

} // namespace

int main()
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
    g_registers[ACCEL][0x22] = 27; // 温度寄存器：23 + 216 × 0.125 = 50 ℃
    put16(GYRO, 0x02, 0);          // 陀螺 x、y、z
    put16(GYRO, 0x04, 0);
    put16(GYRO, 0x06, 1000); // z：1000 × 2000/32768 °/s
    put16(ACCEL, 0x16, 0);   // 加速度 z 轴约 1 g（量程 ±3 g：1 g ≈ 10923）
    put16(ACCEL, 0x12, 0);
    put16(ACCEL, 0x14, 0);
    put16(ACCEL, 0x16, 10923);

    device::Bmi088 imu{hal::Spi{ACCEL}, hal::Spi{GYRO}};
    device::Bmi088Heater heater{hal::Pwm{HEATER},
                                device::Bmi088Heater::Config{control::HeaterControl::Model{0.967F, 15.5F, 0.95F}}};
    bringup::ImuCheck check;
    const bringup::ImuCheck::Parameters parameters{
        device::Bmi088::Config{}, estimate::IDENTITY_AXIS_MAPPING, 1000U, 200U, 50.0F};
    assert(check.init(bringup::ImuCheck::Config{&imu, &heater, hal::Gpio{ACCEL_PIN}, hal::Gpio{GYRO_PIN}, parameters}));
    bringup::ImuCheck second;
    assert(
        !second.init(bringup::ImuCheck::Config{&imu, nullptr, hal::Gpio{ACCEL_PIN}, hal::Gpio{GYRO_PIN}, parameters}));

    // 驱动初始化：按时间推进（任务的 1 ms 超时），完成中断会发布 imu_event
    for (int round = 0; round < 1000 && !check.report().ready; ++round)
    {
        cycle(check, 1000U);
    }
    assert(check.report().ready && !check.report().error);
    const std::uint32_t events_after_init = topics::bringup::imu_event.publish_count();
    assert(events_after_init > 0U); // SPI 完成中断已经在发布

    // 陀螺数据就绪中断（GPIO 外部中断）→ 中断里发布 → 任务读到 → 驱动读样本 → 发布 bringup.imu
    topic::Subscriber<topics::bringup::Imu> imu_topic{topics::bringup::imu};
    for (int edge = 0; edge < 20; ++edge)
    {
        mock::advance_us(250U);
        assert(mock::gpio::trigger_interrupt(GYRO_PIN));
        if (edge % 4 == 0)
        {
            assert(mock::gpio::trigger_interrupt(ACCEL_PIN));
        }
        mock::advance_us(30U); // 中断发布到任务运行的延迟
        cycle(check, 0U);
    }
    const auto &report = check.report();
    assert(report.gyro_samples == 20U);
    assert(report.accel_samples == 5U);
    assert(report.events > 20U);
    assert(report.max_latency_us >= 30U && report.late == 0U);
    assert(report.published >= 4U && report.published <= 8U); // 1 kHz 限频：约 6 ms 里 6 次左右

    topics::bringup::Imu sample{};
    assert(imu_topic.update(sample));
    const float expected_z = 1000.0F * 2000.0F / 32768.0F * 3.14159265F / 180.0F;
    assert(std::fabs(sample.gyro_rad_per_sec[2] - expected_z) < 1e-4F);
    assert(std::fabs(sample.accel_m_per_s2[2] - 9.80665F) < 0.01F);
    assert(sample.gyro_sample_count > 0U && sample.gyro_sample_count <= 20U); // 限频：最后几个样本可能还没发布
    assert(std::fabs(sample.temperature_celsius - 50.0F) < 1e-3F);
    // 姿态：四元数是单位四元数，绕 z 转动使 w 偏离 1
    const float norm = sample.quaternion[0] * sample.quaternion[0] + sample.quaternion[1] * sample.quaternion[1] +
                       sample.quaternion[2] * sample.quaternion[2] + sample.quaternion[3] * sample.quaternion[3];
    assert(std::fabs(norm - 1.0F) < 1e-3F);
    assert(sample.quaternion[0] < 1.0F);

    // 加热器已启动：PWM 有输出设置（读数 50 ℃ 已到目标，直接恒温）
    assert(mock::pwm::output(HEATER).set_count > 0U);
    assert(mock::pwm::output(HEATER).frequency_hz == device::Bmi088Heater::PWM_FREQUENCY_HZ ||
           mock::pwm::output(HEATER).frequency_hz == 0U);

    // 延迟超限计数
    mock::advance_us(250U);
    assert(mock::gpio::trigger_interrupt(GYRO_PIN));
    mock::advance_us(500U);
    cycle(check, 0U);
    assert(check.report().late == 1U && check.report().max_latency_us >= 500U);

    // 任务醒来之后、读 topic 之前又来一次中断发布：延迟按读到的那一份算，不会是负数（2026-11-01 上板发现：
    // 原来用本轮开始的时刻减最近一次发布的时刻，得到 −3～−6 µs，按无符号数计成 42 亿 µs 并记为超限）
    const hal::os::Microseconds woke_us = hal::os::now_us();
    mock::advance_us(5U);
    assert(mock::gpio::trigger_interrupt(GYRO_PIN));
    const std::uint32_t late_before = check.report().late;
    const hal::os::Microseconds max_before = check.report().max_latency_us;
    check.step(woke_us);
    assert(check.report().late == late_before);
    assert(check.report().last_latency_us == 0U && check.report().max_latency_us == max_before);
    while (mock::spi::busy(BUS))
    {
        mock::advance_us(20U);
        assert(mock::spi::complete(BUS));
        check.step(hal::os::now_us());
    }
    return 0;
}
