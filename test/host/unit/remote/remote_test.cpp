// remote::Remote 在 platform/mock 上：UART 替身注入 SBUS 帧，测试核对发布的 two 个 topic。
// 每个 topic 只能有一个发布者，场景按参数分进程运行（与 imu 相同）。
// 覆盖：remote.input 的摇杆归一化/死区/反向、拨杆门限、失控保护与超时保持上一份正常值、参数检查。
// 安全门那条路（拨杆上升沿、arm_sequence）已经搬到仲裁，见 unit/command/arbiter_test.cpp。
#include "mock_hal.hpp"
#include "remote/remote.hpp"
#include "topics/operator_input.hpp"
#include "topics/remote.hpp"
#include "topics/safety.hpp"

#include <array>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace
{

constexpr std::uint8_t PORT{0U};
constexpr hal::os::Microseconds FRAME_TIMEOUT{100000U};
constexpr std::uint8_t RUN_CHANNEL{6U};
constexpr std::uint16_t RUN_MINIMUM{1500U};
constexpr std::uint16_t RUN_MAXIMUM{1900U};

using Channels = std::array<std::uint16_t, protocol::SbusParser::CHANNEL_COUNT>;

Channels channels_with(std::uint16_t value)
{
    Channels channels{};
    channels.fill(value);
    return channels;
}

std::array<std::uint8_t, protocol::SbusParser::FRAME_SIZE> sbus_frame(const Channels &channels, std::uint8_t flags)
{
    std::array<std::uint8_t, protocol::SbusParser::FRAME_SIZE> frame{};
    frame[0] = protocol::SbusParser::START_BYTE;
    std::uint32_t bits = 0U;
    std::uint32_t bit_count = 0U;
    std::size_t index = 1U;
    for (const std::uint16_t value : channels)
    {
        bits |= static_cast<std::uint32_t>(value & 0x07FFU) << bit_count;
        bit_count += 11U;
        while (bit_count >= 8U)
        {
            frame[index++] = static_cast<std::uint8_t>(bits & 0xFFU);
            bits >>= 8U;
            bit_count -= 8U;
        }
    }
    frame[23] = flags;
    frame[24] = 0x00U;
    return frame;
}

void inject(const std::array<std::uint8_t, protocol::SbusParser::FRAME_SIZE> &frame)
{
    assert(mock::uart::inject(PORT, frame.data(), frame.size()) == frame.size());
}

remote::Remote::Parameters parameters()
{
    remote::Remote::Parameters parameters{};
    parameters.sticks = {
        remote::Remote::Stick{0U, 0U, 500U, 1000U, false, 0.05F},
        remote::Remote::Stick{1U, 0U, 500U, 1000U, true, 0.05F}, // 反装
        remote::Remote::Stick{2U, 0U, 500U, 1000U, false, 0.0F},
        remote::Remote::Stick{3U, 0U, 500U, 1000U, false, 0.0F},
    };
    parameters.switches = {
        remote::Remote::Switch{4U, 1000U, 1500U, topics::SwitchPosition::DOWN, topics::SwitchPosition::UP},
        remote::Remote::Switch{5U, 1000U, 1500U, topics::SwitchPosition::DOWN, topics::SwitchPosition::UP},
    };
    parameters.frame_timeout_us = FRAME_TIMEOUT;
    return parameters;
}

struct Harness
{
    remote::Remote remote{hal::Uart{PORT}};
    topic::Subscriber<topics::OperatorInput> input{topics::remote::input};

    bool init()
    {
        mock::uart::reset();
        return remote.init(parameters());
    }

    topics::OperatorInput publish_input(hal::os::Microseconds now)
    {
        remote.step(now);
        topics::OperatorInput message{};
        assert(input.update(message));
        return message;
    }
};

// 参数检查
int scenario_reject()
{
    remote::Remote remote{hal::Uart{PORT}};
    mock::uart::reset();
    remote::Remote::Parameters bad = parameters();
    bad.sticks[0].channel = 16U;
    assert(!remote.init(bad));
    bad = parameters();
    bad.sticks[0].center_raw = 1001U;
    assert(!remote.init(bad));
    bad = parameters();
    bad.sticks[0].deadband = 1.0F;
    assert(!remote.init(bad));
    bad = parameters();
    bad.switches[0].channel = 16U;
    assert(!remote.init(bad));
    bad = parameters();
    bad.switches[0].low_position = topics::SwitchPosition::NONE;
    assert(!remote.init(bad));
    bad = parameters();
    bad.frame_timeout_us = 0U;
    assert(!remote.init(bad));
    std::puts("remote reject: 全部通过");
    return 0;
}

// remote.input：摇杆归一化、死区、反向、拨杆门限；失控保护与超时保持上一份正常值
int scenario_input()
{
    Harness harness;
    assert(harness.init());

    Channels channels = channels_with(500U); // 摇杆都在中位
    channels[4] = 800U;                      // 拨杆 0：低于 1000 → DOWN
    channels[5] = 1400U;                     // 拨杆 1：1000–1500 之间 → MIDDLE
    channels[RUN_CHANNEL] = 1700U;
    inject(sbus_frame(channels, 0x00U));
    topics::OperatorInput input = harness.publish_input(1000U);
    assert(input.has_remote && !input.has_keyboard_mouse && input.connected);
    for (std::size_t index = 0U; index < topics::STICK_COUNT; ++index)
    {
        assert(input.sticks[index] == 0.0F);
    }
    assert(input.switches[0] == topics::SwitchPosition::DOWN);
    assert(input.switches[1] == topics::SwitchPosition::MIDDLE);
    assert(input.switches[2] == topics::SwitchPosition::NONE);
    assert(input.timestamp_us == 1000U);

    // 满量程、中间值；1 号摇杆反装；没有死区的摇杆按原始比例
    channels[0] = 1000U;
    channels[1] = 1000U;
    channels[2] = 525U;  // 归一化 0.05
    channels[3] = 550U;  // 归一化 0.1
    channels[4] = 1700U; // 拨杆 0：高于 1500 → UP
    channels[5] = 1700U; // 拨杆 1：UP
    inject(sbus_frame(channels, 0x00U));
    input = harness.publish_input(2000U);
    assert(input.sticks[topics::STICK_RIGHT_X] == 1.0F);
    assert(input.sticks[topics::STICK_RIGHT_Y] == -1.0F); // 反装
    assert(input.sticks[topics::STICK_LEFT_X] > 0.049F && input.sticks[topics::STICK_LEFT_X] < 0.051F);
    assert(input.sticks[topics::STICK_LEFT_Y] > 0.099F && input.sticks[topics::STICK_LEFT_Y] < 0.101F);
    assert(input.switches[0] == topics::SwitchPosition::UP && input.switches[1] == topics::SwitchPosition::UP);

    // 死区：归一化 0.05 落在死区边界上算 0；0.1 在死区之外，重新拉伸到 0.0526（满量程仍是 ±1）
    channels[0] = 525U;
    inject(sbus_frame(channels, 0x00U));
    assert(harness.publish_input(2500U).sticks[topics::STICK_RIGHT_X] == 0.0F);
    channels[0] = 550U;
    inject(sbus_frame(channels, 0x00U));
    input = harness.publish_input(2600U);
    assert(input.sticks[topics::STICK_RIGHT_X] > 0.052F && input.sticks[topics::STICK_RIGHT_X] < 0.054F);

    // 负极值：0 号摇杆到最小
    channels[0] = 0U;
    inject(sbus_frame(channels, 0x00U));
    input = harness.publish_input(3000U);
    assert(input.sticks[topics::STICK_RIGHT_X] == -1.0F);

    // 失控保护：connected 为假，摇杆与拨杆保持上一份正常值
    channels[0] = 1000U;
    channels[4] = 800U;
    inject(sbus_frame(channels, 0x08U));
    input = harness.publish_input(4000U);
    assert(!input.connected);
    assert(input.sticks[topics::STICK_RIGHT_X] == -1.0F); // 上一份正常值（0 号摇杆到最小）
    assert(input.switches[0] == topics::SwitchPosition::UP);

    // 超时：同样 connected 为假
    input = harness.publish_input(4000U + FRAME_TIMEOUT);
    assert(!input.connected);
    assert(input.sticks[topics::STICK_RIGHT_X] == -1.0F);
    assert(input.timestamp_us == 3000U); // 最近一帧正常帧的时刻
    return 0;
}

} // namespace

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::fprintf(stderr, "用法：%s input|reject\n", argv[0]);
        return 2;
    }
    const char *scenario = argv[1];
    if (std::strcmp(scenario, "reject") == 0)
    {
        return scenario_reject();
    }
    if (std::strcmp(scenario, "input") == 0)
    {
        return scenario_input();
    }
    std::fprintf(stderr, "未知场景：%s\n", scenario);
    return 2;
}
