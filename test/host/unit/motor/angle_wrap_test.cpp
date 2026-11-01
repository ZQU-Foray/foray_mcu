// 多圈累计：相邻两帧机械角差按不超过半圈解释（DjiMotor::receive_frames）。
#include "driver/motor/dji/dji_motor.hpp"
#include "mock_hal.hpp"
#include "mock_os.hpp"

#include <array>
#include <cassert>
#include <cstdint>
#include <initializer_list>

void check(std::initializer_list<std::uint16_t> angles, std::int32_t expected)
{
    mock::can::reset(); // 每个会话重新占用总线 0 的接收
    device::DjiMotor bus{hal::Can{0U}, device::DjiMotor::Profile::c610()};
    assert(bus.init());
    assert(bus.enable_motor(1));
    for (const auto angle : angles)
    {
        const std::array<std::uint8_t, 8> data{
            static_cast<std::uint8_t>(angle >> 8), static_cast<std::uint8_t>(angle), 0, 0, 0, 0, 0, 0};
        assert(mock::can::inject(0U, 0x201, data));
        bus.process();
    }
    device::DjiMotor::Snapshot state{};
    assert(bus.get_snapshot(1, state));
    assert(state.total_angle_counts == expected);
}

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
}
