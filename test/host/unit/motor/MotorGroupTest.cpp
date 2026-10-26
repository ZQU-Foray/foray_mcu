#include "Libraries/Device/motor/dji/DjiMotorModelMap.hpp"
#include "Libraries/Device/motor/MotorGroup.hpp"
#include "support/DjiPlatformStub.hpp"
#include <cmath>
#include <type_traits>

using device::MotorGroup;
using device::MotorModel;
using Channel = platform::Can::Channel;
using Small = MotorGroup<MotorModel::M2006, 4>;
using Large = MotorGroup<MotorModel::M3508, 1>;
using Gimbal = MotorGroup<MotorModel::GM6020_CURRENT, 1>;
static_assert(Small::count() == 4);
static_assert(!std::is_copy_constructible_v<Small> && !std::is_move_constructible_v<Small>);

int main()
{
    // 无效映射不能占住通道；成功 Init 的对象一直存活到程序结束。
    Small invalid{Channel::CHANNEL_1, {{{1, 1}, {1, 1}, {3, 1}, {4, 1}}}};
    assert(!invalid.init());
    Small chassis{Channel::CHANNEL_1, {{{1, 1}, {2, 1}, {3, 1}, {4, 1}}}};
    g_can_stub.ready = false;
    assert(!chassis.init());
    g_can_stub.ready = true;
    assert(chassis.init() && chassis.init());
    Large other{Channel::CHANNEL_2, {{{1, -1}}}};
    Gimbal gimbal{Channel::CHANNEL_3, {{{1, 1}}}};
    assert(other.init() && gimbal.init());
    // 不同型号也共享通道占用检查，失败的实例不能接收或发送。
    Large conflict{Channel::CHANNEL_1, {{{1, 1}}}};
    assert(!conflict.init());
    const auto sent = g_can_stub.tx_count;
    conflict.process();
    assert(g_can_stub.tx_count == sent);
    assert(!conflict.set_torque(0, 0.1F));

    assert(chassis.set_torque(0, 0.18F));
    assert(other.set_torque(0, 0.3F));
    assert(gimbal.set_torque(0, 0.741F));
    assert(g_can_stub.tx_count == sent);
    chassis.process();
    assert(g_can_stub.last_tx_channel == CAN_PORT_CHANNEL_1);
    assert(g_can_stub.last_tx_identifier == 0x200);
    assert(g_can_stub.last_tx_data[0] == 3 && g_can_stub.last_tx_data[1] == 0xE8); // 1A ->1000
    assert(g_can_stub.tx_count == sent + 1);                                 // four slots still one frame
    other.process();
    assert(g_can_stub.last_tx_channel == CAN_PORT_CHANNEL_2);
    const auto raw = static_cast<std::int16_t>((g_can_stub.last_tx_data[0] << 8) | g_can_stub.last_tx_data[1]);
    assert(raw == -819); // C620 -1A; direction applies to torque
    gimbal.process();
    assert(g_can_stub.last_tx_channel == CAN_PORT_CHANNEL_3);
    assert(g_can_stub.last_tx_identifier == 0x1FE);
    assert(g_can_stub.last_tx_data[0] == 0x15 && g_can_stub.last_tx_data[1] == 0x55); // 1A ->5461

    device::MotorState state{};
    reset_rx();
    feed_rx(0x201, {0, 0, 0x0E, 0x10, 0, 0, 0, 0}); // 3600 rpm
    chassis.process();
    assert(chassis.read_state(0, state) && state.speed_rpm == 100);
    reset_rx();
    feed_rx(0x201, {0, 0, 0x07, 0x80, 0, 0, 0, 0}); //1920 rpm
    other.process();
    assert(other.read_state(0, state) && std::fabs(state.speed_rpm + 100) < 1e-5F);
    reset_rx();
    feed_rx(0x205, {0, 0, 0, 100, 0, 0, 0, 0});
    gimbal.process();
    assert(gimbal.read_state(0, state) && state.speed_rpm == 100);
    assert(chassis.read_state(0, state) && state.speed_rpm == 100); // independent snapshots
    reset_rx();
    feed_rx(0x201, {0x08, 0, 0, 0, 0, 0, 0, 0}); //+90 rotor degrees
    other.process();
    assert(other.read_state(0, state) && std::fabs(state.angle_degrees + 90.0F / 19.2F) < 1e-5F);
    chassis.clear_commands();
    chassis.process();
    assert(g_can_stub.last_tx_data == (std::array<std::uint8_t, 8>{}));
    other.process(); // clearing one group does not affect another
    assert(g_can_stub.last_tx_data[0] == 0xFC && g_can_stub.last_tx_data[1] == 0xCD);
}
