// hal::Can 契约 v1 在 platform/mock 上的语义：驱动的宿主测试以它为准。
#include "hal/can.hpp"
#include "mock_hal.hpp"

#include <array>
#include <cassert>
#include <cstdint>

namespace
{

struct Wakeup
{
    hal::Can bus;
    std::uint32_t calls{0U};
    bool frame_was_queued{false};
};

// 接收通知：在帧入队之后调用，回调里就能 receive() 到
void on_receive(void *context) noexcept
{
    auto &wakeup = *static_cast<Wakeup *>(context);
    ++wakeup.calls;
    hal::Can::Frame frame{};
    wakeup.frame_was_queued = wakeup.bus.receive(frame) == hal::Can::ReceiveResult::RECEIVED;
}

hal::Can::Frame make_frame(std::uint32_t identifier, hal::Can::IdentifierType type, std::uint8_t length)
{
    hal::Can::Frame frame{};
    frame.identifier = identifier;
    frame.identifier_type = type;
    frame.length = length;
    frame.data[0] = 0x5AU;
    return frame;
}

} // namespace

int main()
{
    mock::can::reset();
    const hal::Can bus{2U};
    static_assert(sizeof(hal::Can) == 1U);
    assert(bus.is_ready());

    // 接收方只能占用一次；无效编号占用失败
    assert(bus.claim_receiver());
    assert(!bus.claim_receiver());
    assert(!hal::Can{mock::can::BUS_COUNT}.claim_receiver());

    // 发送：参数检查先于状态检查
    using Type = hal::Can::IdentifierType;
    assert(bus.send(make_frame(0x7FFU, Type::STANDARD, 8U)) == hal::Can::SendResult::QUEUED);
    assert(bus.send(make_frame(0x800U, Type::STANDARD, 8U)) == hal::Can::SendResult::INVALID_ARGUMENT);
    assert(bus.send(make_frame(0x1FFFFFFFU, Type::EXTENDED, 0U)) == hal::Can::SendResult::QUEUED);
    assert(bus.send(make_frame(0x20000000U, Type::EXTENDED, 0U)) == hal::Can::SendResult::INVALID_ARGUMENT);
    assert(bus.send(make_frame(0x100U, Type::STANDARD, 9U)) == hal::Can::SendResult::INVALID_ARGUMENT);
    assert(mock::can::sent_count(2U) == 2U);
    hal::Can::Frame frame{};
    assert(mock::can::pop_sent(2U, frame) && frame.identifier == 0x7FFU && frame.data[0] == 0x5AU);
    assert(mock::can::pop_sent(2U, frame) && frame.identifier_type == Type::EXTENDED);
    assert(!mock::can::pop_sent(2U, frame));

    // 接收：空、收到、通知在入队之后
    assert(bus.receive(frame) == hal::Can::ReceiveResult::EMPTY);
    Wakeup wakeup{bus};
    assert(bus.set_receive_notification(on_receive, &wakeup));
    assert(mock::can::inject(2U, 0x201U, {1, 2, 3, 4, 5, 6, 7, 8}));
    assert(wakeup.calls == 1U && wakeup.frame_was_queued);
    assert(bus.set_receive_notification(nullptr, nullptr));
    assert(mock::can::inject(2U, 0x202U, {}));
    assert(wakeup.calls == 1U);
    assert(bus.receive(frame) == hal::Can::ReceiveResult::RECEIVED && frame.identifier == 0x202U);

    // 软件队列满：丢弃并计数
    for (std::size_t index = 0U; index < mock::can::QUEUE_CAPACITY; ++index)
    {
        assert(mock::can::inject(2U, 0x300U, {}));
    }
    assert(!mock::can::inject(2U, 0x301U, {}));
    assert(bus.statistics().rx_dropped_count == 1U);
    while (bus.receive(frame) == hal::Can::ReceiveResult::RECEIVED)
    {
    }

    // Bus-Off 与未启动
    mock::can::set_bus_off(2U, true);
    assert(!bus.is_ready());
    assert(bus.send(make_frame(0x100U, Type::STANDARD, 1U)) == hal::Can::SendResult::BUS_OFF);
    assert(bus.receive(frame) == hal::Can::ReceiveResult::BUS_OFF);
    mock::can::set_bus_off(2U, false);
    mock::can::set_ready(2U, false);
    assert(bus.send(make_frame(0x100U, Type::STANDARD, 1U)) == hal::Can::SendResult::NOT_READY);
    assert(bus.receive(frame) == hal::Can::ReceiveResult::NOT_READY);
    mock::can::set_ready(2U, true);

    // 强制发送结果（如硬件队列满）：帧不进发送记录
    mock::can::set_send_result(2U, hal::Can::SendResult::QUEUE_FULL);
    assert(bus.send(make_frame(0x100U, Type::STANDARD, 1U)) == hal::Can::SendResult::QUEUE_FULL);
    assert(mock::can::sent_count(2U) == 0U);

    // 无效编号
    const hal::Can invalid{mock::can::BUS_COUNT};
    assert(!invalid.is_ready());
    assert(invalid.send(make_frame(0x100U, Type::STANDARD, 1U)) == hal::Can::SendResult::INVALID_ARGUMENT);
    assert(invalid.receive(frame) == hal::Can::ReceiveResult::INVALID_ARGUMENT);
    assert(!invalid.set_receive_notification(on_receive, &wakeup));
    assert(invalid.statistics().bus_off_count == 0U);
    return 0;
}
