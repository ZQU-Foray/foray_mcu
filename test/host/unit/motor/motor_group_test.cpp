// 编译期选型的电机组（device::MotorGroup + DJI 型号映射）在 platform/mock 上。
#include "driver/motor/dji/dji_motor_model_map.hpp"
#include "driver/motor/motor_group.hpp"
#include "mock_hal.hpp"
#include "mock_os.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <type_traits>

using device::MotorGroup;
using device::MotorModel;
using Small = MotorGroup<MotorModel::M2006, 4>;
using Large = MotorGroup<MotorModel::M3508, 1>;
using Gimbal = MotorGroup<MotorModel::GM6020_CURRENT, 1>;
static_assert(Small::count() == 4);
static_assert(!std::is_copy_constructible_v<Small> && !std::is_move_constructible_v<Small>);

namespace
{

constexpr std::uint8_t CHASSIS_BUS{0U};
constexpr std::uint8_t OTHER_BUS{1U};
constexpr std::uint8_t GIMBAL_BUS{2U};

hal::Can::Frame last_sent(std::uint8_t bus)
{
    hal::Can::Frame frame{};
    assert(mock::can::last_sent(bus, frame));
    return frame;
}

} // namespace

int main()
{
    mock::reset();
    mock::can::reset();
    // 无效映射不能占住总线；成功 Init 的对象一直存活到程序结束。
    Small invalid{hal::Can{CHASSIS_BUS}, {{{1, 1}, {1, 1}, {3, 1}, {4, 1}}}};
    assert(!invalid.init());
    Small chassis{hal::Can{CHASSIS_BUS}, {{{1, 1}, {2, 1}, {3, 1}, {4, 1}}}};
    mock::can::set_ready(CHASSIS_BUS, false);
    assert(!chassis.init());
    mock::can::set_ready(CHASSIS_BUS, true);
    assert(chassis.init() && chassis.init());
    Large other{hal::Can{OTHER_BUS}, {{{1, -1}}}};
    Gimbal gimbal{hal::Can{GIMBAL_BUS}, {{{1, 1}}}};
    assert(other.init() && gimbal.init());
    // 同一总线的接收只能被占用一次（不同型号也一样），失败的实例不能接收或发送。
    Large conflict{hal::Can{CHASSIS_BUS}, {{{1, 1}}}};
    assert(!conflict.init());
    const auto sent = mock::can::sent_count(CHASSIS_BUS);
    conflict.process();
    assert(mock::can::sent_count(CHASSIS_BUS) == sent);
    assert(!conflict.set_torque(0, 0.1F) && !conflict.enable());

    assert(!chassis.set_torque(0, 0.18F)); // 没有 enable() 不出力（Decision-20261102-01）
    assert(chassis.enable() && other.enable() && gimbal.enable());

    assert(chassis.set_torque(0, 0.18F));
    assert(other.set_torque(0, 0.3F));
    assert(gimbal.set_torque(0, 0.741F));
    assert(mock::can::sent_count(CHASSIS_BUS) == sent);
    chassis.process();
    assert(last_sent(CHASSIS_BUS).identifier == 0x200);
    assert(last_sent(CHASSIS_BUS).data[0] == 3 && last_sent(CHASSIS_BUS).data[1] == 0xE8); // 1A ->1000
    assert(mock::can::sent_count(CHASSIS_BUS) == sent + 1);                                // four slots still one frame
    other.process();
    const auto other_data = last_sent(OTHER_BUS).data;
    const auto raw = static_cast<std::int16_t>((other_data[0] << 8) | other_data[1]);
    assert(raw == -819); // C620 -1A; direction applies to torque
    gimbal.process();
    assert(last_sent(GIMBAL_BUS).identifier == 0x1FE);
    assert(last_sent(GIMBAL_BUS).data[0] == 0x15 && last_sent(GIMBAL_BUS).data[1] == 0x55); // 1A ->5461

    device::MotorState state{};
    (void)mock::can::inject(CHASSIS_BUS, 0x201, {0, 0, 0x0E, 0x10, 0, 0, 0, 0}); // 3600 rpm
    chassis.process();
    assert(chassis.read_state(0, state) && state.speed_rpm == 100);
    (void)mock::can::inject(OTHER_BUS, 0x201, {0, 0, 0x07, 0x80, 0, 0, 0, 0}); // 1920 rpm
    other.process();
    assert(other.read_state(0, state) && std::fabs(state.speed_rpm + 100) < 1e-5F);
    (void)mock::can::inject(GIMBAL_BUS, 0x205, {0, 0, 0, 100, 0, 0, 0, 0});
    gimbal.process();
    assert(gimbal.read_state(0, state) && state.speed_rpm == 100);
    assert(chassis.read_state(0, state) && state.speed_rpm == 100);         // independent snapshots
    (void)mock::can::inject(OTHER_BUS, 0x201, {0x08, 0, 0, 0, 0, 0, 0, 0}); // +90 rotor degrees
    other.process();
    assert(other.read_state(0, state) && std::fabs(state.angle_degrees + 90.0F / 19.2F) < 1e-5F);
    chassis.clear_commands();
    chassis.process();
    assert(last_sent(CHASSIS_BUS).data == (std::array<std::uint8_t, 8>{}));
    other.process(); // clearing one group does not affect another
    assert(last_sent(OTHER_BUS).data[0] == 0xFC && last_sent(OTHER_BUS).data[1] == 0xCD);
    other.disable(); // 失能一组也不影响别的组
    assert(!other.set_torque(0, 0.3F) && chassis.set_torque(0, 0.18F));
    other.process();
    assert(last_sent(OTHER_BUS).data == (std::array<std::uint8_t, 8>{}));
}
