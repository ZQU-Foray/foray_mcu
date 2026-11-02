// device::DjiMotor（Decision-20261102-03）在 platform/mock 上：登记到 CanBus、经关节 Joint 控制。
// 保活帧、反馈换算、力矩换算与饱和、混接与 ID 冲突、绝对位置、机构、离线、负载计算。
#include "driver/can_bus/can_bus.hpp"
#include "driver/motor/dji/dji_models.hpp"
#include "driver/motor/dji/dji_motor.hpp"
#include "driver/motor/joint.hpp"
#include "mock_hal.hpp"
#include "mock_os.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace
{

using device::DjiMotor;
using device::Joint;
using device::JointMechanics;
using Bytes = std::array<std::uint8_t, 8>;

constexpr std::uint8_t BUS{0U};
constexpr float PI{3.14159265F};
constexpr JointMechanics LONG_TIMEOUT{1, 1.0F, 0.0F, 1000000U}; // 只下一次指令再看几轮

void reset()
{
    mock::reset();
    mock::can::reset();
    mock::set_now_us(1000U);
}

DjiMotor::Motor
motor_of(const DjiMotor::Model &model, std::uint8_t device_id, const JointMechanics &mechanics = LONG_TIMEOUT)
{
    return DjiMotor::Motor{nullptr, model, device_id, mechanics};
}

struct Rig final
{
    explicit Rig(const DjiMotor::Config &config) noexcept : driver{bus, with_joints(config, joints)} {}

    [[nodiscard]] bool start() noexcept
    {
        return bus.init() && driver.init();
    }
    void cycle() noexcept
    {
        bus.receive();
        bus.transmit(hal::os::now_us());
        mock::advance_us(1000U);
    }
    [[nodiscard]] DjiMotor::Snapshot snapshot(std::size_t index = 0U) const noexcept
    {
        DjiMotor::Snapshot result{};
        assert(driver.get_snapshot(index, result));
        return result;
    }

    static DjiMotor::Config with_joints(DjiMotor::Config config, std::array<Joint, DjiMotor::MAXIMUM_MOTORS> &joints)
    {
        for (std::size_t index = 0U; index < config.motors.size(); ++index)
        {
            if (config.motors[index].joint == nullptr)
            {
                config.motors[index].joint = &joints[index];
            }
        }
        return config;
    }

    device::CanBus bus{hal::Can{BUS}};
    std::array<Joint, DjiMotor::MAXIMUM_MOTORS> joints{};
    DjiMotor driver;
};

bool near(float actual, float expected, float tolerance)
{
    return std::fabs(actual - expected) <= tolerance;
}

std::int16_t slot_of(const hal::Can::Frame &frame, std::size_t slot)
{
    return static_cast<std::int16_t>((frame.data[slot * 2U] << 8U) | frame.data[slot * 2U + 1U]);
}

/** 本轮发出的帧里，帧 ID 为 identifier 的那一帧。 */
hal::Can::Frame sent_frame(std::uint32_t identifier)
{
    hal::Can::Frame frame{};
    hal::Can::Frame found{};
    bool seen = false;
    while (mock::can::pop_sent(BUS, frame))
    {
        if (frame.identifier == identifier)
        {
            found = frame;
            seen = true;
        }
    }
    assert(seen);
    return found;
}

Bytes feedback(std::uint16_t angle, std::int16_t rpm, std::int16_t current, std::uint8_t temperature)
{
    return Bytes{static_cast<std::uint8_t>(angle >> 8U),
                 static_cast<std::uint8_t>(angle & 0xFFU),
                 static_cast<std::uint8_t>(static_cast<std::uint16_t>(rpm) >> 8U),
                 static_cast<std::uint8_t>(static_cast<std::uint16_t>(rpm) & 0xFFU),
                 static_cast<std::uint8_t>(static_cast<std::uint16_t>(current) >> 8U),
                 static_cast<std::uint8_t>(static_cast<std::uint16_t>(current) & 0xFFU),
                 temperature,
                 0U};
}

void test_keep_alive_and_feedback()
{
    reset();
    DjiMotor::Config config{};
    config.count = 1U;
    config.motors[0] = motor_of(device::dji::M3508, 1U);
    Rig rig{config};
    Joint &joint = rig.joints[0];
    rig.bus.transmit(0U); // 没有 init() 不发送
    assert(mock::can::sent_count(BUS) == 0U);
    assert(rig.start());
    // 接上即可使能（DJI 没有使能命令）；只支持力矩；最大力矩 0.3 × 20 = 6 N·m；位置从第一帧起算；有温度
    assert(joint.available() && joint.supports(device::JointMode::TORQUE) &&
           !joint.supports(device::JointMode::IMPEDANCE));
    assert(near(joint.maximum_torque_newton_meter(), 6.0F, 1e-5F));
    assert(!joint.state().position_absolute && joint.state().has_temperature);

    // 保活：没有请求出力也每周期发 0x200（C620 要先收到控制帧才回反馈）
    rig.cycle();
    hal::Can::Frame frame = sent_frame(0x200U);
    assert(frame.length == 8U && frame.data == Bytes{});

    // 反馈 0x201：转子 4096、100 rpm、电流 50 计数、30 ℃
    mock::set_now_us(100000U);
    assert(mock::can::inject(BUS, 0x201U, feedback(0x1000U, 100, 50, 30U)));
    rig.cycle();
    DjiMotor::Snapshot snapshot = rig.snapshot();
    assert(snapshot.ever_received && snapshot.online && snapshot.feedback_count == 1U);
    assert(snapshot.last_update_us == 100000U && snapshot.rotor_angle_raw == 0x1000U);
    assert(snapshot.total_angle_counts == 0 && snapshot.rotor_speed_rpm == 100 && snapshot.torque_current_raw == 50);
    assert(snapshot.motor_temperature_celsius == 30U && snapshot.feedback_identifier == 0x201U);
    const device::JointState &state = joint.state();
    assert(state.feedback_valid && state.online && state.position_rad == 0.0F && state.stamp_us == 100000U);
    assert(near(state.velocity_rad_per_second, 100.0F * 2.0F * PI / 60.0F / 19.2F, 1e-4F));
    assert(near(state.torque_newton_meter, 50.0F / 16384.0F * 20.0F * 0.3F, 1e-5F));
    assert(state.temperature_celsius == 30.0F && !state.enabled && !state.fault);

    // 多圈：+4095 后跨零前进 +16，净 +4111 计数，输出轴 = 4111 / 8192 圈 / 19.2
    assert(mock::can::inject(BUS, 0x201U, feedback(0x1FFFU, 0, 0, 30U)));
    rig.cycle();
    assert(mock::can::inject(BUS, 0x201U, feedback(0x000FU, 0, 0, 30U)));
    rig.cycle();
    assert(rig.snapshot().total_angle_counts == 4111);
    assert(near(state.position_rad, 4111.0F / 8192.0F * 2.0F * PI / 19.2F, 1e-5F));

    // 50 ms 没有反馈：离线，最后的值保留
    mock::advance_us(DjiMotor::LINK_TIMEOUT_US);
    rig.cycle();
    assert(!rig.snapshot().online && !state.online && state.feedback_valid);
}

void test_torque_commands()
{
    reset();
    DjiMotor::Config config{};
    config.count = 2U;
    config.motors[0] = motor_of(device::dji::M3508, 1U);
    config.motors[1] = motor_of(device::dji::M3508, 6U, JointMechanics{-1, 1.0F, 0.0F, 1000000U}); // 第二组、反向
    Rig rig{config};
    Joint &first = rig.joints[0];
    Joint &second = rig.joints[1];
    assert(rig.start());

    assert(!first.set_torque(1.5F)); // 没有 enable()
    assert(first.enable() && second.enable());
    assert(first.set_torque(1.5F));  // 1.5 N·m / 0.3 = 5 A → 5 / 20 × 16384 = 4096
    assert(second.set_torque(1.5F)); // 反向：−4096，在 0x1FF 的第 2 格（6 号）
    rig.cycle();
    assert(first.state().enabled && second.state().enabled);
    // 两帧都发：0x200 与 0x1FF
    hal::Can::Frame low{};
    hal::Can::Frame high{};
    hal::Can::Frame frame{};
    std::size_t frames = 0U;
    while (mock::can::pop_sent(BUS, frame))
    {
        ++frames;
        (frame.identifier == 0x200U ? low : high) = frame;
    }
    assert(frames == 2U);
    assert(low.identifier == 0x200U && slot_of(low, 0U) == 4096 && slot_of(low, 1U) == 0);
    assert(high.identifier == 0x1FFU && slot_of(high, 1U) == -4096 && slot_of(high, 0U) == 0);
    assert(rig.snapshot(1U).control_identifier == 0x1FFU && rig.snapshot(1U).command_raw == -4096);

    // 超出能力：饱和到 ±16384；非有限值：清掉指令
    assert(first.set_torque(999.0F));
    rig.cycle();
    assert(slot_of(sent_frame(0x200U), 0U) == 16384);
    assert(!first.set_torque(std::numeric_limits<float>::quiet_NaN()));
    rig.cycle();
    assert(slot_of(sent_frame(0x200U), 0U) == 0);

    // disable()：这一格归零，保活照发
    assert(second.set_torque(1.0F));
    second.disable();
    rig.cycle();
    assert(slot_of(sent_frame(0x1FFU), 1U) == 0 && !second.state().enabled);
}

void test_mixed_bus()
{
    // C620 1、2 号 + C610 3 号共用 0x200；GM6020 电流模式 2 号走 0x1FE 第 2 格，反馈 0x206
    reset();
    DjiMotor::Config config{};
    config.count = 4U;
    config.motors[0] = motor_of(device::dji::M3508, 1U);
    config.motors[1] = motor_of(device::dji::M3508, 2U);
    config.motors[2] = motor_of(device::dji::M2006, 3U);
    config.motors[3] = motor_of(device::dji::GM6020_CURRENT, 2U);
    Rig rig{config};
    assert(rig.start());
    assert(rig.snapshot(2U).feedback_identifier == 0x203U && rig.snapshot(3U).feedback_identifier == 0x206U);
    assert(rig.snapshot(3U).control_identifier == 0x1FEU);
    for (Joint &joint : rig.joints)
    {
        if (joint.available())
        {
            assert(joint.enable());
        }
    }
    assert(rig.joints[2].set_torque(0.18F));  // M2006：0.18 / 0.18 = 1 A → 1 / 10 × 10000 = 1000
    assert(rig.joints[3].set_torque(0.741F)); // GM6020：1 A → 1 / 3 × 16384 = 5461
    rig.cycle();
    hal::Can::Frame frame{};
    std::size_t frames = 0U;
    while (mock::can::pop_sent(BUS, frame))
    {
        ++frames;
        if (frame.identifier == 0x200U)
        {
            assert(slot_of(frame, 2U) == 1000 && slot_of(frame, 0U) == 0);
        }
        else
        {
            assert(frame.identifier == 0x1FEU && slot_of(frame, 1U) == 5461);
        }
    }
    assert(frames == 2U);

    // GM6020 位置上电即绝对：第一帧的转子角就是位置（4096 计数 = π）
    assert(mock::can::inject(BUS, 0x206U, feedback(4096U, 0, 0, 40U)));
    rig.cycle();
    assert(rig.joints[3].state().position_absolute && near(rig.joints[3].state().position_rad, PI, 1e-5F));
    assert(!rig.joints[2].state().has_temperature); // C610 没有温度反馈

    // 负载：4 台反馈各 1 kHz + 2 个控制帧 × 1 kHz = 6000 帧/秒 ≈ 81%
    static_assert(device::can_load_percent(DjiMotor::frames_per_second(
                      DjiMotor::Config{{DjiMotor::Motor{nullptr, device::dji::M3508, 1U, JointMechanics{}},
                                        DjiMotor::Motor{nullptr, device::dji::M3508, 2U, JointMechanics{}},
                                        DjiMotor::Motor{nullptr, device::dji::M3508, 3U, JointMechanics{}},
                                        DjiMotor::Motor{nullptr, device::dji::M3508, 4U, JointMechanics{}}},
                                       4U},
                      1000U)) == 68U); // 底盘 4 台 M3508：5 帧/毫秒
    assert(DjiMotor::frames_per_second(config, 1000U) == 6000U);
}

void test_conflicts_and_validation()
{
    reset();
    const auto rejects = [](const DjiMotor::Config &config)
    {
        Rig rig{config};
        assert(rig.bus.init() && !rig.driver.init());
        assert(!rig.joints[0].supports(device::JointMode::TORQUE)); // 失败时关节没被接上
        mock::can::reset();
    };
    DjiMotor::Config config{};
    config.count = 2U;
    config.motors[0] = motor_of(device::dji::M3508, 5U);
    config.motors[1] = motor_of(device::dji::GM6020_CURRENT, 1U); // 反馈都是 0x205
    rejects(config);
    config.motors[1] = motor_of(device::dji::M2006, 5U); // C610 与 C620 同号：同一帧同一格，反馈也相同
    rejects(config);
    config.count = 1U;
    config.motors[0] = motor_of(device::dji::M3508, 0U);
    rejects(config);
    config.motors[0] = motor_of(device::dji::M3508, 9U);
    rejects(config);
    config.motors[0] = motor_of(device::dji::GM6020_CURRENT, 8U); // GM6020 只有 1–7
    rejects(config);
    config.motors[0] = motor_of(DjiMotor::Model{DjiMotor::Profile::gm6020_voltage(), 1.0F, true}, 1U); // 电压模式
    rejects(config);
    config.motors[0] = motor_of(device::dji::M3508, 1U, JointMechanics{0, 1.0F, 0.0F, 3000U});
    rejects(config);
    config.count = 0U;
    rejects(config);
    config.count = DjiMotor::MAXIMUM_MOTORS + 1U;
    rejects(config);

    // 同一个关节不能接两台电机
    Joint shared;
    config.count = 2U;
    config.motors[0] = motor_of(device::dji::M3508, 1U);
    config.motors[0].joint = &shared;
    config.motors[1] = motor_of(device::dji::M3508, 2U);
    config.motors[1].joint = &shared;
    rejects(config);

    // 同一条 CanBus 上两组 DJI：反馈 ID 不能重叠（CanBus 登记时拒绝）
    reset();
    DjiMotor::Config chassis{};
    chassis.count = 1U;
    chassis.motors[0] = motor_of(device::dji::M3508, 1U);
    Rig rig{chassis};
    assert(rig.start() && rig.driver.init()); // 重复 init() 不再登记
    Joint launcher_joint;
    DjiMotor::Config launcher{};
    launcher.count = 1U;
    launcher.motors[0] = motor_of(device::dji::M2006, 1U);
    launcher.motors[0].joint = &launcher_joint;
    DjiMotor overlapping{rig.bus, launcher};
    assert(!overlapping.init() && !launcher_joint.supports(device::JointMode::TORQUE));
    // 反馈 ID 不同也不行：两组都用控制帧 0x200，会各发一份互相覆盖。一条总线上的 DJI 电机要放在同一个 DjiMotor 里
    launcher.motors[0].device_id = 2U;
    DjiMotor neighbor{rig.bus, launcher};
    assert(!neighbor.init());
    // 总线上有别的主机也在发 0x200：计数
    assert(mock::can::inject(BUS, 0x200U, Bytes{}));
    rig.cycle();
    DjiMotor::Statistics statistics{};
    assert(rig.driver.get_statistics(statistics) && statistics.foreign_control_frame_count == 1U);
    DjiMotor::Snapshot snapshot{};
    assert(!rig.driver.get_snapshot(1U, snapshot));
}

} // namespace

int main()
{
    test_keep_alive_and_feedback();
    test_torque_commands();
    test_mixed_bus();
    test_conflicts_and_validation();
    return 0;
}
