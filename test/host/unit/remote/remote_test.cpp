// remote::Remote 在 platform/mock 上：UART 替身注入 SBUS 帧，测试核对发布的 safety.stop。
// 每个 topic 只能有一个发布者，场景按参数分进程运行（与 imu 相同）。
// 覆盖：运行档、失控保护（恢复后不算一次拨上）、超时、拨回再拨上（arm_sequence 加 1）、上电就在运行档（不加）。
#include "mock_hal.hpp"
#include "remote/remote.hpp"
#include "topics/safety.hpp"

#include <array>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace
{

constexpr std::uint8_t PORT{0U};
constexpr std::uint16_t RUN_MINIMUM{1500U};
constexpr std::uint16_t RUN_MAXIMUM{1900U};
constexpr std::uint8_t CHANNEL{5U};
constexpr hal::os::Microseconds FRAME_TIMEOUT{100000U};

using Channels = std::array<std::uint16_t, protocol::SbusParser::CHANNEL_COUNT>;

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

Channels channels_with(std::uint16_t value)
{
    Channels channels{};
    channels[CHANNEL] = value;
    return channels;
}

void inject(const std::array<std::uint8_t, protocol::SbusParser::FRAME_SIZE> &frame)
{
    assert(mock::uart::inject(PORT, frame.data(), frame.size()) == frame.size());
}

struct Harness
{
    remote::Remote remote{hal::Uart{PORT}};
    topic::Subscriber<topics::safety::Stop> stop{topics::safety::stop};

    bool init()
    {
        mock::uart::reset();
        return remote.init(remote::Remote::Parameters{CHANNEL, RUN_MINIMUM, RUN_MAXIMUM, FRAME_TIMEOUT});
    }

    topics::safety::Stop publish(hal::os::Microseconds now)
    {
        remote.step(now);
        topics::safety::Stop message{};
        assert(stop.update(message));
        return message;
    }
};

// 运行档：stop 为假；拨回再拨上 → arm_sequence 加 1
int scenario_run()
{
    Harness harness;
    assert(harness.init());
    assert(harness.publish(0U).stop); // 还没有帧
    inject(sbus_frame(channels_with(1700U), 0x00U));
    const topics::safety::Stop first = harness.publish(1000U);
    assert(!first.stop && first.arm_sequence == 0U); // 上电就在运行档：没有"拨"的动作
    assert(harness.remote.report().running && harness.remote.report().fresh);
    // 拨回停止档
    inject(sbus_frame(channels_with(1000U), 0x00U));
    assert(harness.publish(2000U).stop);
    // 再拨到运行档
    inject(sbus_frame(channels_with(1700U), 0x00U));
    const topics::safety::Stop again = harness.publish(3000U);
    assert(!again.stop && again.arm_sequence == 1U);
    // 再拨一次：再加 1
    inject(sbus_frame(channels_with(1000U), 0x00U));
    assert(harness.publish(4000U).stop);
    inject(sbus_frame(channels_with(1700U), 0x00U));
    assert(harness.publish(5000U).arm_sequence == 2U);
    // 运行档之外的值（拨杆没到位）不算运行
    inject(sbus_frame(channels_with(1400U), 0x00U));
    assert(harness.publish(6000U).stop);
    return 0;
}

// 失控保护位：即使拨杆在运行档，stop 为真；位消失后 stop 变假，但这不算一次"拨上"（arm_sequence 不变），
// 因为失控保护期间的拨杆位置不可信，要像上电时一样拨回去再拨上
int scenario_failsafe()
{
    Harness harness;
    assert(harness.init());
    inject(sbus_frame(channels_with(1700U), 0x00U));
    const topics::safety::Stop first = harness.publish(1000U);
    assert(!first.stop && first.arm_sequence == 0U);

    inject(sbus_frame(channels_with(1700U), 0x08U));
    const topics::safety::Stop failsafe = harness.publish(2000U);
    assert(failsafe.stop && failsafe.arm_sequence == 0U);
    assert(harness.remote.report().failsafe == 1U);

    // 信号恢复、拨杆还在运行档：运行恢复，但没有"拨"的动作
    inject(sbus_frame(channels_with(1700U), 0x00U));
    const topics::safety::Stop recovered = harness.publish(3000U);
    assert(!recovered.stop && recovered.arm_sequence == 0U);

    // 拨回去再拨上才加 1
    inject(sbus_frame(channels_with(1000U), 0x00U));
    assert(harness.publish(4000U).stop);
    inject(sbus_frame(channels_with(1700U), 0x00U));
    assert(harness.publish(5000U).arm_sequence == 1U);
    return 0;
}

// 停止注入超过超时：stop 为真；恢复注入后不需要重新"拨"
int scenario_timeout()
{
    Harness harness;
    assert(harness.init());
    inject(sbus_frame(channels_with(1700U), 0x00U));
    assert(!harness.publish(1000U).stop);
    assert(!harness.publish(1000U + FRAME_TIMEOUT - 1U).stop); // 还在有效期
    assert(harness.publish(1000U + FRAME_TIMEOUT).stop);       // 到点
    assert(!harness.remote.report().fresh);
    inject(sbus_frame(channels_with(1700U), 0x00U));
    assert(!harness.publish(1000U + FRAME_TIMEOUT + 1000U).stop);
    return 0;
}

// 参数检查
int scenario_reject()
{
    remote::Remote remote{hal::Uart{PORT}};
    mock::uart::reset();
    assert(!remote.init(remote::Remote::Parameters{16U, RUN_MINIMUM, RUN_MAXIMUM, FRAME_TIMEOUT}));
    assert(!remote.init(remote::Remote::Parameters{CHANNEL, RUN_MAXIMUM, RUN_MINIMUM, FRAME_TIMEOUT}));
    assert(!remote.init(remote::Remote::Parameters{CHANNEL, RUN_MINIMUM, RUN_MAXIMUM, 0U}));
    std::puts("remote reject: 全部通过");
    return 0;
}

} // namespace

int main(int argc, char **argv)
{
    if (argc == 2 && std::strcmp(argv[1], "run") == 0)
    {
        return scenario_run();
    }
    if (argc == 2 && std::strcmp(argv[1], "failsafe") == 0)
    {
        return scenario_failsafe();
    }
    if (argc == 2 && std::strcmp(argv[1], "timeout") == 0)
    {
        return scenario_timeout();
    }
    if (argc == 2 && std::strcmp(argv[1], "reject") == 0)
    {
        return scenario_reject();
    }
    std::fprintf(stderr, "用法：%s run|failsafe|timeout|reject\n", argv[0]);
    return 2;
}
