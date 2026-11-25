// imu::Imu 在 platform/mock 上：BMI088 由 SPI 替身的应答函数扮演，
// 数据就绪中断由 GPIO 替身以 2 kHz（陀螺）/ 400 Hz（加速度计）触发，温度 50 ℃。
// 每个 topic 只能有一个发布者，场景按参数分进程运行：
//   ready    静止：0.5 s 快速零偏 → READY（不等恒温），精确标定随后在后台完成、只换零偏；
//            发布的角速度扣掉零偏，yaw 归零后保持在 0 附近；安装方向把传感器 z 轴换到云台 x 轴
//   degraded 一直在转：快速窗口超时 → DEGRADED，yaw 零点同样在这一刻取
//   heater   温度到不了目标（加热器坏 / 没接 24 V）：仍用快速零偏进 READY，精确标定一直等温度
//   reject   参数检查：没有加热器、安装方向无效、发布间隔为 0、快速窗口为 0
#include "imu/imu.hpp"
#include "lib/estimate/bias_store.hpp"
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
constexpr std::uint8_t NVS_REGION{0U}; // mock 的零偏记录区
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
    mock::nvs::reset();
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

// 记录层的存储后端：接到 mock 的 hal::Nvs 上（与能力里同一个适配方式）
std::size_t nvs_size(void *context) noexcept
{
    return static_cast<const hal::Nvs *>(context)->size();
}

std::size_t nvs_granularity(void *context) noexcept
{
    return static_cast<const hal::Nvs *>(context)->write_granularity();
}

bool nvs_read(void *context, std::size_t offset, std::uint8_t *data, std::size_t length) noexcept
{
    return static_cast<const hal::Nvs *>(context)->read(offset, data, length) == hal::Nvs::Result::OK;
}

bool nvs_write(void *context, std::size_t offset, const std::uint8_t *data, std::size_t length) noexcept
{
    return static_cast<const hal::Nvs *>(context)->write(offset, data, length) == hal::Nvs::Result::OK;
}

bool nvs_erase(void *context) noexcept
{
    return static_cast<const hal::Nvs *>(context)->erase() == hal::Nvs::Result::OK;
}

// 模拟"上一次上电留下了一条记录"
bool prefill_bias(float temperature_celsius, const math::Vector3 &bias_rad_per_sec)
{
    hal::Nvs nvs{NVS_REGION};
    estimate::BiasStore store;
    const estimate::BiasStore::Storage storage{&nvs, nvs_size, nvs_granularity, nvs_read, nvs_write, nvs_erase};
    return store.init(storage, estimate::BiasStore::Config{}) &&
           store.record(temperature_celsius, bias_rad_per_sec, 5000000U);
}

imu::Imu::Parameters fast_parameters(const estimate::AxisMapping &mounting)
{
    imu::Imu::Parameters parameters{};
    parameters.driver = device::Bmi088::Config{device::Bmi088Gyro::Bandwidth::ODR_2000_HZ_532_HZ};
    parameters.mounting = mounting;
    parameters.publish_period_us = 1000U;
    parameters.heater_target_celsius = 50.0F;
    // 精确标定：1 s 稳定、1 s 窗口、4 s 超时（大约 2 s 后才完成，肯定晚于 0.5 s 的快速窗口）
    parameters.calibration.temperature_settle_us = 1000000U;
    parameters.calibration.window_us = 1000000U;
    parameters.calibration.timeout_us = 4000000U;
    // 快速零偏用默认值：不等温度，0.5 s 窗口、1 s 超时
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
                                            hal::Nvs{NVS_REGION},
                                            fast_parameters(estimate::IDENTITY_AXIS_MAPPING)}));
    topic::Subscriber<topics::imu::Attitude> attitude_topic{topics::imu::attitude};
    bring_up(capability);

    // 零偏：z 轴 +17 LSB（约 0.018 rad/s，1.04 °/s），x 轴 -8 LSB
    const float bias_z = 17.0F * GYRO_LSB_RAD;
    const float bias_x = -8.0F * GYRO_LSB_RAD;
    const hal::os::Microseconds first_sample = hal::os::now_us();
    set_gyro(-8, 0, 17);

    // 前 0.2 s 还在快速窗口里：SETTLING，VQF 还没拿到零偏，yaw 随零偏漂移
    run(capability, 200000U, [](std::uint32_t) {});
    topics::imu::Attitude attitude{};
    assert(attitude_topic.update(attitude));
    assert(attitude.state == topics::imu::State::SETTLING && attitude.valid);
    assert(capability.report().state == topics::imu::State::SETTLING);
    assert(std::fabs(yaw_of(attitude)) > 0.001F);

    // 0.5 s 静止窗口（不等温度）之后 READY；精确标定还在后台，没有完成
    run(capability, 400000U, [](std::uint32_t) {});
    assert(capability.report().state == topics::imu::State::READY);
    assert(capability.report().bias_source == imu::Imu::BiasSource::QUICK);
    assert(capability.report().quick_calibration == estimate::GyroBiasCalibration::State::COMPLETED);
    assert(capability.report().calibration != estimate::GyroBiasCalibration::State::COMPLETED);
    const math::Vector3 calibrated = capability.report().calibrated_bias_rad_per_sec;
    assert(std::fabs(calibrated.z - bias_z) < 1e-4F && std::fabs(calibrated.x - bias_x) < 1e-4F);
    const float yaw_zero = capability.report().yaw_zero_rad;
    assert(std::fabs(yaw_zero) > 0.001F && capability.report().settled_us != 0U);
    // 从第一个样本到 READY 不超过 0.7 s（快速窗口 0.5 s）
    assert(capability.report().settled_us - first_sample <= 700000U);

    // READY 之后：角速度扣掉零偏接近 0；yaw 归零，并且不再漂
    run(capability, 50000U, [](std::uint32_t) {});
    assert(attitude_topic.update(attitude));
    assert(attitude.state == topics::imu::State::READY && attitude.valid);
    assert(std::fabs(attitude.gyro_rad_per_sec[2]) < 2e-4F && std::fabs(attitude.gyro_rad_per_sec[0]) < 2e-4F);
    assert(std::fabs(yaw_of(attitude)) < 2e-3F);
    run(capability, 2500000U, [](std::uint32_t) {});
    assert(attitude_topic.update(attitude));
    assert(std::fabs(yaw_of(attitude)) < 2e-3F); // 2 s 不漂

    // 后台精确标定完成：只把零偏交给 VQF；状态、yaw 零点都不动（云台和 PC 的目标不会跳）
    assert(capability.report().precise_applied);
    assert(capability.report().calibration == estimate::GyroBiasCalibration::State::COMPLETED);
    assert(capability.report().yaw_zero_rad == yaw_zero);
    assert(capability.report().state == topics::imu::State::READY);
    assert(std::fabs(capability.report().calibrated_bias_rad_per_sec.z - bias_z) < 1e-4F);
    assert(std::fabs(yaw_of(attitude)) < 2e-3F); // 精确标定换零偏不改变已发布的姿态
    // 快窗口与精确标定各写了一条记录：Flash 里已经不是擦除态
    assert(capability.report().store_available && capability.report().store_written >= 2U);
    assert(!mock::nvs::is_erased(NVS_REGION, 0U, 1U));

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
    assert(capability.init(imu::Imu::Config{&g_sensor,
                                            &g_heater,
                                            hal::Gpio{ACCEL_PIN},
                                            hal::Gpio{GYRO_PIN},
                                            hal::Nvs{NVS_REGION},
                                            fast_parameters(rotated)}));
    topic::Subscriber<topics::imu::Attitude> attitude_topic{topics::imu::attitude};
    bring_up(capability);

    // 传感器 x 轴一直来回晃（±1.2 rad/s 交替，极差超过 1 rad/s）：快速窗口一直重启，1 s 后超时
    const hal::os::Microseconds first_sample = hal::os::now_us();
    run(capability,
        2500000U,
        [](std::uint32_t edge)
        {
            const std::int16_t swing = static_cast<std::int16_t>(1.2F / GYRO_LSB_RAD);
            set_gyro(edge % 2U == 0U ? swing : static_cast<std::int16_t>(-swing), 0, 0);
        });
    assert(capability.report().state == topics::imu::State::DEGRADED);
    assert(capability.report().bias_source == imu::Imu::BiasSource::NONE);
    assert(capability.report().quick_calibration == estimate::GyroBiasCalibration::State::TIMED_OUT);
    assert(capability.report().calibrated_bias_rad_per_sec.x == 0.0F);
    // 1 s 超时、最多再等一拍，就进 DEGRADED
    assert(capability.report().settled_us - first_sample <= 1200000U);
    topics::imu::Attitude attitude{};
    assert(attitude_topic.update(attitude));
    assert(attitude.state == topics::imu::State::DEGRADED && attitude.valid);
    // 传感器 x 落到云台 -y：发布的角速度在 y 轴上
    assert(std::fabs(attitude.gyro_rad_per_sec[1]) > 1.0F && std::fabs(attitude.gyro_rad_per_sec[0]) < 0.05F);
    std::puts("imu degraded: 全部通过");
    return 0;
}

int scenario_heater()
{
    setup_sensor();
    // 温度 42 ℃：加热器到不了 50 ℃ 的目标，精确标定的温度门控永远不满足（加热器坏 / 没接 24 V）
    g_registers[ACCEL][0x22] = 152; // 23 + 152 × 0.125 = 42 ℃
    imu::Imu capability;
    assert(capability.init(imu::Imu::Config{&g_sensor,
                                            &g_heater,
                                            hal::Gpio{ACCEL_PIN},
                                            hal::Gpio{GYRO_PIN},
                                            hal::Nvs{NVS_REGION},
                                            fast_parameters(estimate::IDENTITY_AXIS_MAPPING)}));
    topic::Subscriber<topics::imu::Attitude> attitude_topic{topics::imu::attitude};
    bring_up(capability);

    const float bias_z = 17.0F * GYRO_LSB_RAD;
    set_gyro(0, 0, 17);
    // 不接恒温也能 READY：这正是原来会永远停在 SETTLING 的情形
    run(capability, 800000U, [](std::uint32_t) {});
    assert(capability.report().state == topics::imu::State::READY);
    assert(capability.report().bias_source == imu::Imu::BiasSource::QUICK);
    assert(std::fabs(capability.report().calibrated_bias_rad_per_sec.z - bias_z) < 1e-4F);
    assert(capability.report().calibration == estimate::GyroBiasCalibration::State::WAITING_FOR_TEMPERATURE);
    // 再跑 10 s：精确标定还停在等温度，但姿态一直可用、零偏照样扣
    run(capability, 10000000U, [](std::uint32_t) {});
    assert(capability.report().state == topics::imu::State::READY);
    assert(capability.report().calibration == estimate::GyroBiasCalibration::State::WAITING_FOR_TEMPERATURE);
    assert(!capability.report().precise_applied);
    topics::imu::Attitude attitude{};
    assert(attitude_topic.update(attitude) && attitude.state == topics::imu::State::READY && attitude.valid);
    assert(std::fabs(attitude.gyro_rad_per_sec[2]) < 2e-4F);
    std::puts("imu heater: 全部通过");
    return 0;
}

int scenario_stored()
{
    setup_sensor();
    g_registers[ACCEL][0x22] = 19; // 这次上电在 42 ℃（11 位原始值 152 = 19 << 3）：
                                   // 存储的记录是 40/50 ℃ 两条，要按 b(T0) 重算
    // 上一次上电留下的两条记录：40 ℃ 与 50 ℃，斜率 0.001 rad/s/℃，b(42 ℃) 正好等于本板的零偏
    const float bias_z = 17.0F * GYRO_LSB_RAD;
    const float bias_x = -8.0F * GYRO_LSB_RAD;
    const math::Vector3 actual{bias_x, 0.0F, bias_z};
    const math::Vector3 low{bias_x - 0.002F, 0.0F, bias_z - 0.002F};
    const math::Vector3 high{bias_x + 0.008F, 0.0F, bias_z + 0.008F};
    assert(prefill_bias(40.0F, low));
    {
        hal::Nvs nvs{NVS_REGION};
        estimate::BiasStore store;
        const estimate::BiasStore::Storage storage{&nvs, nvs_size, nvs_granularity, nvs_read, nvs_write, nvs_erase};
        assert(store.init(storage, estimate::BiasStore::Config{}));
        assert(store.record(50.0F, high, 5000000U));
    }

    imu::Imu capability;
    assert(capability.init(imu::Imu::Config{&g_sensor,
                                            &g_heater,
                                            hal::Gpio{ACCEL_PIN},
                                            hal::Gpio{GYRO_PIN},
                                            hal::Nvs{NVS_REGION},
                                            fast_parameters(estimate::IDENTITY_AXIS_MAPPING)}));
    topic::Subscriber<topics::imu::Attitude> attitude_topic{topics::imu::attitude};
    bring_up(capability);
    assert(capability.report().store_has_bias && capability.report().store_records == 2U);

    const hal::os::Microseconds first_sample = hal::os::now_us();
    set_gyro(-8, 0, 17);
    run(capability, 20000U, [](std::uint32_t) {}); // 只跑 20 ms：不等 0.5 s 的快窗口
    assert(capability.report().state == topics::imu::State::READY);
    assert(capability.report().bias_source == imu::Imu::BiasSource::STORED);
    assert(capability.report().settled_us - first_sample <= 5000U);
    topics::imu::Attitude attitude{};
    assert(attitude_topic.update(attitude));
    assert(attitude.state == topics::imu::State::READY && attitude.valid);
    assert(std::fabs(yaw_of(attitude)) < 2e-3F);
    // 一开始用的是 50 ℃ 那条（最新记录）的 b(50)；温度读数到了以后按 b(42) 重算，角速度才扣得准
    // 记录是 40/50 ℃ 两点，模型是直线：按这一次的温度 b(42) 扣，角速度应当接近 0（常数模型会差 0.008）
    assert(std::fabs(capability.report().bias_temperature_celsius - 42.0F) < 1e-4F);
    assert(std::fabs(attitude.gyro_rad_per_sec[2]) < 2e-4F && std::fabs(attitude.gyro_rad_per_sec[0]) < 2e-4F);
    assert(std::fabs(capability.report().calibrated_bias_rad_per_sec.z - actual.z) < 5e-4F);
    run(capability, 200000U, [](std::uint32_t) {});
    assert(capability.report().state == topics::imu::State::READY);
    assert(attitude_topic.update(attitude) && std::fabs(yaw_of(attitude)) < 2e-3F);

    // 继续跑：快窗口在 0.5 s 完成（42 ℃），再记一条；状态与 yaw 零点都不变
    const float yaw_zero = capability.report().yaw_zero_rad;
    run(capability, 700000U, [](std::uint32_t) {});
    assert(capability.report().state == topics::imu::State::READY);
    assert(capability.report().bias_source == imu::Imu::BiasSource::STORED);
    assert(capability.report().yaw_zero_rad == yaw_zero);
    assert(capability.report().store_written >= 1U);
    std::puts("imu stored: 全部通过");
    return 0;
}

int scenario_stored_moving()
{
    setup_sensor();
    // 有记录、上电时云台还在动：仍然立刻 READY（原来会退到 DEGRADED 等后台标定）
    const float bias_z = 17.0F * GYRO_LSB_RAD;
    assert(prefill_bias(45.0F, math::Vector3{0.0F, 0.0F, bias_z}));

    imu::Imu capability;
    assert(capability.init(imu::Imu::Config{&g_sensor,
                                            &g_heater,
                                            hal::Gpio{ACCEL_PIN},
                                            hal::Gpio{GYRO_PIN},
                                            hal::Nvs{NVS_REGION},
                                            fast_parameters(estimate::IDENTITY_AXIS_MAPPING)}));
    topic::Subscriber<topics::imu::Attitude> attitude_topic{topics::imu::attitude};
    bring_up(capability);

    run(capability,
        1500000U,
        [](std::uint32_t edge)
        {
            // z 轴 ±1.2 rad/s 来回晃（零偏叠在上面）：快窗口的极差一直超限，1 s 后超时
            const std::int16_t swing = static_cast<std::int16_t>(1.2F / GYRO_LSB_RAD);
            set_gyro(0, 0, static_cast<std::int16_t>(17 + (edge % 2U == 0U ? swing : -swing)));
        });
    assert(capability.report().state == topics::imu::State::READY);
    assert(capability.report().bias_source == imu::Imu::BiasSource::STORED);
    assert(capability.report().quick_calibration == estimate::GyroBiasCalibration::State::TIMED_OUT);
    topics::imu::Attitude attitude{};
    assert(attitude_topic.update(attitude));
    assert(attitude.state == topics::imu::State::READY && attitude.valid);
    std::puts("imu stored_moving: 全部通过");
    return 0;
}

int scenario_reject()
{
    setup_sensor();
    imu::Imu capability;
    imu::Imu::Parameters parameters = fast_parameters(estimate::IDENTITY_AXIS_MAPPING);
    assert(!capability.init(imu::Imu::Config{&g_sensor,
                                             nullptr,
                                             hal::Gpio{ACCEL_PIN},
                                             hal::Gpio{GYRO_PIN},
                                             hal::Nvs{NVS_REGION},
                                             parameters})); // 没有加热器
    parameters.mounting = estimate::AxisMapping{{0U, 0U, 2U}, {1, 1, 1}};
    assert(!capability.init(imu::Imu::Config{&g_sensor,
                                             &g_heater,
                                             hal::Gpio{ACCEL_PIN},
                                             hal::Gpio{GYRO_PIN},
                                             hal::Nvs{NVS_REGION},
                                             parameters})); // 安装方向不是旋转
    parameters = fast_parameters(estimate::IDENTITY_AXIS_MAPPING);
    parameters.publish_period_us = 0U;
    assert(!capability.init(imu::Imu::Config{
        &g_sensor, &g_heater, hal::Gpio{ACCEL_PIN}, hal::Gpio{GYRO_PIN}, hal::Nvs{NVS_REGION}, parameters}));
    parameters = fast_parameters(estimate::IDENTITY_AXIS_MAPPING);
    parameters.calibration.window_us = 0U;
    assert(!capability.init(imu::Imu::Config{
        &g_sensor, &g_heater, hal::Gpio{ACCEL_PIN}, hal::Gpio{GYRO_PIN}, hal::Nvs{NVS_REGION}, parameters}));
    parameters = fast_parameters(estimate::IDENTITY_AXIS_MAPPING);
    parameters.quick_calibration.window_us = 0U;
    assert(!capability.init(imu::Imu::Config{
        &g_sensor, &g_heater, hal::Gpio{ACCEL_PIN}, hal::Gpio{GYRO_PIN}, hal::Nvs{NVS_REGION}, parameters}));
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
    if (argc == 2 && std::strcmp(argv[1], "stored") == 0)
    {
        return scenario_stored();
    }
    if (argc == 2 && std::strcmp(argv[1], "stored_moving") == 0)
    {
        return scenario_stored_moving();
    }
    if (argc == 2 && std::strcmp(argv[1], "heater") == 0)
    {
        return scenario_heater();
    }
    if (argc == 2 && std::strcmp(argv[1], "reject") == 0)
    {
        return scenario_reject();
    }
    std::fprintf(stderr, "用法：%s ready|degraded|stored|stored_moving|heater|reject\n", argv[0]);
    return 2;
}
