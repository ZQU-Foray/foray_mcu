#include "support/DjiPlatformStub.hpp"
#include "Libraries/Device/motor/dji/DjiMotorAdapter.hpp"
#include <cmath>
#include <limits>

using Adapter = device::DjiMotorAdapter;
Adapter::Config default_config()
{
    return {platform::Can::Channel::CHANNEL_1,
            device::DjiMotor::Profile::c610(),
            {{{2U, 36.0F, -1}, {1U, 36.0F, 1}, {3U, 36.0F, 1}, {4U, 36.0F, 1}}},
            4U};
}
int main()
{
    auto config = default_config();
    Adapter adapter{config};
    device::MotorState state{1.0F, 2.0F, true, true, 1U};
    assert(!adapter.read_state(0, state) && !state.feedback_valid && state.angle_degrees == 0);
    assert(!adapter.set_torque(0, 0.1F));
    adapter.process();
    assert(g_can_stub.tx_count == 0);
    assert(adapter.init());
    assert(adapter.count() == 4);
    assert(adapter.read_state(0, state) && !state.feedback_valid && !state.online);
    adapter.process();
    assert(g_can_stub.tx_count == 1 && g_can_stub.last_tx_identifier == 0x200);
    assert(g_can_stub.last_tx_data == (std::array<std::uint8_t, 8>{}));
    // Logical 0 -> device 2, negative direction; output-shaft Kt is not geared twice.
    assert(adapter.set_torque(0, 0.18F));
    assert(adapter.set_torque(1, 0.36F));
    assert(adapter.set_torque(2, 0.54F));
    assert(adapter.set_torque(3, 0.72F));
    assert(g_can_stub.tx_count == 1); // setters do not send
    assert(adapter.init());           // idempotent, retains staged commands
    adapter.process();
    assert(g_can_stub.tx_count == 2);
    assert(g_can_stub.last_tx_data == (std::array<std::uint8_t, 8>{0x07, 0xD0, 0xFC, 0x18, 0x0B, 0xB8, 0x0F, 0xA0}));
    g_time_stub.ticks = 100;
    feed_rx(0x202, {0x10, 0x00, 0x0E, 0x10, 0, 0, 0, 0}); // 3600 rotor rpm
    feed_rx(0x201, {0x00, 0x00, 0x0E, 0x10, 0, 0, 0, 0});
    adapter.process();
    assert(g_can_stub.rx_index == g_can_stub.rx_count);
    assert(adapter.read_state(0, state));
    assert(state.feedback_valid && state.online && state.last_update_tick == 100);
    assert(state.angle_degrees == 0 && state.speed_rpm == -100);
    assert(adapter.read_state(1, state) && state.speed_rpm == 100);
    reset_rx();
    feed_rx(0x202, {0x18, 0x00, 0x0E, 0x10, 0, 0, 0, 0}); // +90 rotor deg
    adapter.process();
    assert(adapter.read_state(0, state) && std::fabs(state.angle_degrees + 2.5F) < 1e-6F);
    // a2cfd16 把离线超时从 10 ms 调为 50 ms；引用常量，仍验证"恰好到超时即离线"。
    g_time_stub.ticks = 100U + device::DjiMotor::LINK_TIMEOUT_MS;
    adapter.process();
    assert(adapter.read_state(0, state) && !state.online && state.feedback_valid);
    reset_rx();
    feed_rx(0x202, {0x18, 0x00, 0, 0, 0, 0, 0, 0});
    adapter.process();
    assert(adapter.read_state(0, state) && state.online);
    assert(!adapter.read_state(4, state));
    assert(!state.online && !state.feedback_valid && state.angle_degrees == 0 && state.last_update_tick == 0);
    assert(!adapter.set_torque(4, 1));
    for (float value : {std::numeric_limits<float>::quiet_NaN(),
                        std::numeric_limits<float>::infinity(),
                        -std::numeric_limits<float>::infinity()})
    {
        assert(adapter.set_torque(0, 0.18F));
        assert(!adapter.set_torque(0, value));
        adapter.process();
        assert(g_can_stub.last_tx_data[2] == 0 && g_can_stub.last_tx_data[3] == 0);
        assert(g_can_stub.last_tx_data[0] == 0x07); // other slots retained
    }
    assert(adapter.set_torque(0, std::numeric_limits<float>::max()));
    adapter.process(); // saturated to -10000
    assert(g_can_stub.last_tx_data[2] == 0xD8 && g_can_stub.last_tx_data[3] == 0xF0);
    const auto tx_before = g_can_stub.tx_count;
    adapter.clear_commands();
    assert(g_can_stub.tx_count == tx_before);
    adapter.process();
    assert(g_can_stub.last_tx_data == (std::array<std::uint8_t, 8>{}));
    const auto rejects = [](const Adapter::Config &bad)
    {
        Adapter invalid{bad};
        auto count = g_can_stub.tx_count;
        assert(!invalid.init());
        invalid.process();
        assert(g_can_stub.tx_count == count);
    };
    config = default_config();
    config.channel = static_cast<platform::Can::Channel>(255);
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
    g_can_stub.ready = false;
    rejects(default_config());
    g_can_stub.ready = true;
    config = default_config();
    config.profile = device::DjiMotor::Profile::gm6020_current();
    Adapter current_gm{config};
    assert(current_gm.init());
}
