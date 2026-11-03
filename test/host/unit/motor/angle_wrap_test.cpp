// 多圈累计：相邻两帧转子角差按不超过半圈解释（DjiMotor::accept_feedback，经 CanBus 收帧）。
#include "driver/can_bus/can_bus.hpp"
#include "driver/motor/dji/dji_models.hpp"
#include "driver/motor/dji/dji_motor.hpp"
#include "driver/motor/joint.hpp"
#include "mock_hal.hpp"
#include "mock_os.hpp"

#include <array>
#include <cassert>
#include <cstdint>
#include <initializer_list>

namespace
{

void check(std::initializer_list<std::uint16_t> angles, std::int32_t expected)
{
    mock::can::reset(); // 每次重新占用总线 0 的接收
    device::CanBus bus{hal::Can{0U}};
    device::Joint joint;
    device::DjiMotor::Config config{};
    config.count = 1U;
    config.motors[0] = device::DjiMotor::Motor{&joint, device::dji::M2006, 1U, device::JointMechanics{}};
    device::DjiMotor motor{bus, config};
    assert(bus.init() && motor.init());
    for (const auto angle : angles)
    {
        const std::array<std::uint8_t, 8> data{
            static_cast<std::uint8_t>(angle >> 8), static_cast<std::uint8_t>(angle), 0, 0, 0, 0, 0, 0};
        assert(mock::can::inject(0U, 0x201, data));
        bus.receive();
    }
    device::DjiMotor::Snapshot state{};
    assert(motor.get_snapshot(0U, state));
    assert(state.total_angle_counts == expected);
}

} // namespace

int main()
{
    mock::reset();
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
    return 0;
}
