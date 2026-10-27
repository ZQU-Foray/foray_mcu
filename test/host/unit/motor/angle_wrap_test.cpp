#include "support/DjiPlatformStub.hpp"
#include <initializer_list>

void check(std::initializer_list<std::uint16_t> angles, std::int32_t expected)
{
    device::DjiMotor bus{platform::Can::Channel::CHANNEL_1, device::DjiMotor::Profile::c610()};
    assert(bus.init());
    assert(bus.enable_motor(1));
    for (const auto angle : angles)
    {
        reset_rx();
        feed_rx(0x201, {static_cast<std::uint8_t>(angle >> 8), static_cast<std::uint8_t>(angle), 0, 0, 0, 0, 0, 0});
        bus.process();
    }
    device::DjiMotor::Snapshot state{};
    assert(bus.get_snapshot(1, state));
    assert(state.total_angle_counts == expected);
}
int main()
{
    check({4096}, 0);
    check({4096, 8191, 15}, 4111);
    check({15, 8191, 4096}, -4111);
    check({8191, 0}, 1);
    check({0, 8191}, -1);
    check({0, 2048, 4096, 6144, 0, 2048, 4096, 6144, 0}, 16384);
    check({0, 6144, 4096, 2048, 0, 6144, 4096, 2048, 0}, -16384);
    check({0, 4096}, 4096);
    check({4096, 0}, -4096);
    check({0, 4095}, 4095);
    check({0, 4097}, -4095);
}
