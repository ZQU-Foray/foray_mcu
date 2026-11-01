// DJI 电机适配器（逻辑编号 → 设备编号、方向、减速比、力矩口径）在 platform/mock 上。
#include "driver/motor/dji/dji_motor_adapter.hpp"
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

hal::Can::Frame last_sent()
{
    hal::Can::Frame frame{};
    assert(mock::can::last_sent(0U, frame));
    return frame;
}

std::size_t total_sent()
{
    std::size_t total = 0U;
    for (std::uint8_t bus = 0U; bus < mock::can::BUS_COUNT; ++bus)
    {
        total += mock::can::sent_count(bus);
    }
    return total;
}

} // namespace

using Adapter = device::DjiMotorAdapter;
Adapter::Config default_config()
{
    return {hal::Can{0U},
            device::DjiMotor::Profile::c610(),
            {{{2U, 36.0F, -1}, {1U, 36.0F, 1}, {3U, 36.0F, 1}, {4U, 36.0F, 1}}},
            4U};
}
int main()
{
    mock::reset();
    mock::can::reset();
    auto config = default_config();
    Adapter adapter{config};
    device::MotorState state{1.0F, 2.0F, true, true, 1U};
    assert(!adapter.read_state(0, state) && !state.feedback_valid && state.angle_degrees == 0);
    assert(!adapter.set_torque(0, 0.1F));
    adapter.process();
    assert(mock::can::sent_count(0U) == 0);
    assert(adapter.init());
    assert(adapter.count() == 4);
    assert(adapter.read_state(0, state) && !state.feedback_valid && !state.online);
    adapter.process();
    assert(mock::can::sent_count(0U) == 1 && last_sent().identifier == 0x200);
    assert(last_sent().data == (std::array<std::uint8_t, 8>{}));
    // Logical 0 -> device 2, negative direction; output-shaft Kt is not geared twice.
    assert(adapter.set_torque(0, 0.18F));
    assert(adapter.set_torque(1, 0.36F));
    assert(adapter.set_torque(2, 0.54F));
    assert(adapter.set_torque(3, 0.72F));
    assert(mock::can::sent_count(0U) == 1); // setters do not send
    assert(adapter.init());                 // idempotent, retains staged commands
    adapter.process();
    assert(mock::can::sent_count(0U) == 2);
    assert(last_sent().data == (std::array<std::uint8_t, 8>{0x07, 0xD0, 0xFC, 0x18, 0x0B, 0xB8, 0x0F, 0xA0}));
    mock::set_now_us(100000U);
    (void)mock::can::inject(0U, 0x202, {0x10, 0x00, 0x0E, 0x10, 0, 0, 0, 0}); // 3600 rotor rpm
    (void)mock::can::inject(0U, 0x201, {0x00, 0x00, 0x0E, 0x10, 0, 0, 0, 0});
    adapter.process();
    hal::Can::Frame unread{};
    assert(hal::Can{0U}.receive(unread) == hal::Can::ReceiveResult::EMPTY); // 全部反馈都被取走
    assert(adapter.read_state(0, state));
    assert(state.feedback_valid && state.online && state.last_update_us == 100000U);
    assert(state.angle_degrees == 0 && state.speed_rpm == -100);
    assert(adapter.read_state(1, state) && state.speed_rpm == 100);
    (void)mock::can::inject(0U, 0x202, {0x18, 0x00, 0x0E, 0x10, 0, 0, 0, 0}); // +90 rotor deg
    adapter.process();
    assert(adapter.read_state(0, state) && std::fabs(state.angle_degrees + 2.5F) < 1e-6F);
    // a2cfd16 把离线超时从 10 ms 调为 50 ms；引用常量，仍验证"恰好到超时即离线"。
    mock::set_now_us(100000U + device::DjiMotor::LINK_TIMEOUT_US);
    adapter.process();
    assert(adapter.read_state(0, state) && !state.online && state.feedback_valid);
    (void)mock::can::inject(0U, 0x202, {0x18, 0x00, 0, 0, 0, 0, 0, 0});
    adapter.process();
    assert(adapter.read_state(0, state) && state.online);
    assert(!adapter.read_state(4, state));
    assert(!state.online && !state.feedback_valid && state.angle_degrees == 0 && state.last_update_us == 0U);
    assert(!adapter.set_torque(4, 1));
    for (float value : {std::numeric_limits<float>::quiet_NaN(),
                        std::numeric_limits<float>::infinity(),
                        -std::numeric_limits<float>::infinity()})
    {
        assert(adapter.set_torque(0, 0.18F));
        assert(!adapter.set_torque(0, value));
        adapter.process();
        assert(last_sent().data[2] == 0 && last_sent().data[3] == 0);
        assert(last_sent().data[0] == 0x07); // other slots retained
    }
    assert(adapter.set_torque(0, std::numeric_limits<float>::max()));
    adapter.process(); // saturated to -10000
    assert(last_sent().data[2] == 0xD8 && last_sent().data[3] == 0xF0);
    const auto tx_before = mock::can::sent_count(0U);
    adapter.clear_commands();
    assert(mock::can::sent_count(0U) == tx_before);
    adapter.process();
    assert(last_sent().data == (std::array<std::uint8_t, 8>{}));
    const auto rejects = [](const Adapter::Config &bad)
    {
        Adapter invalid{bad};
        const std::size_t count = total_sent();
        assert(!invalid.init());
        invalid.process();
        assert(total_sent() == count); // 被拒绝的适配器在任何总线上都不发送
    };
    config = default_config();
    config.bus = hal::Can{mock::can::BUS_COUNT}; // 无效编号：总线不就绪
    rejects(config);
    config = default_config();
    config.count = 0;
    rejects(config);
    config = default_config();
    config.count = 9;
    rejects(config);
    config = default_config();
    config.mappings[1].device_id = 2;
    rejects(config);
    config = default_config();
    config.mappings[0].device_id = 0;
    rejects(config);
    config = default_config();
    config.mappings[0].device_id = 9;
    rejects(config);
    config = default_config();
    config.mappings[0].direction = 0;
    rejects(config);
    config = default_config();
    config.mappings[0].direction = 2;
    rejects(config);
    for (float bad :
         {0.0F, -1.0F, 0.5F, std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()})
    {
        config = default_config();
        config.mappings[0].gear_ratio = bad;
        rejects(config);
    }
    config = default_config();
    config.profile = device::DjiMotor::Profile::gm6020_voltage();
    rejects(config);
    config = default_config();
    config.profile.torque_constant_newton_meter_per_ampere = 0;
    rejects(config);
    config = default_config();
    config.profile.current_full_scale_ampere = std::numeric_limits<float>::infinity();
    rejects(config);
    mock::can::set_ready(1U, false);
    config = default_config();
    config.bus = hal::Can{1U};
    rejects(config); // 总线未启动：失败，且不占用总线
    mock::can::set_ready(1U, true);
    config = default_config();
    config.bus = hal::Can{1U};
    config.profile = device::DjiMotor::Profile::gm6020_current();
    Adapter current_gm{config};
    assert(current_gm.init());
    config.bus = hal::Can{0U};
    rejects(config); // 总线 0 的接收已被 adapter 占用
}
