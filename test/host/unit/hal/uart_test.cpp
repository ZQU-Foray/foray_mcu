// hal::Uart 契约 v1 在 platform/mock 上的语义：驱动与能力的宿主测试以它为准。
#include "hal/uart.hpp"
#include "mock_hal.hpp"

#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>

namespace
{

void count(void *context) noexcept
{
    ++*static_cast<std::uint32_t *>(context);
}

} // namespace

int main()
{
    using WriteResult = hal::Uart::WriteResult;
    mock::uart::reset();
    const hal::Uart port{1U};
    static_assert(sizeof(hal::Uart) == 1U);
    assert(port.is_ready());

    // 接收：注入后通知，read() 按顺序取，分批取
    std::uint32_t received_calls = 0U;
    assert(port.set_receive_notification(count, &received_calls));
    const std::array<std::uint8_t, 5> bytes{1U, 2U, 3U, 4U, 5U};
    assert(mock::uart::inject(1U, bytes.data(), bytes.size()) == 5U);
    assert(received_calls == 1U);
    std::array<std::uint8_t, 3> buffer{};
    assert(port.read(buffer.data(), buffer.size()) == 3U && buffer[0] == 1U && buffer[2] == 3U);
    assert(port.read(buffer.data(), buffer.size()) == 2U && buffer[0] == 4U && buffer[1] == 5U);
    assert(port.read(buffer.data(), buffer.size()) == 0U);
    assert(port.read(nullptr, 3U) == 0U);

    // 接收队列满：丢弃并计数，丢光了不通知
    std::array<std::uint8_t, mock::uart::QUEUE_CAPACITY + 4U> flood{};
    assert(mock::uart::inject(1U, flood.data(), flood.size()) == mock::uart::QUEUE_CAPACITY);
    assert(port.statistics().dropped_byte_count == 4U);
    assert(mock::uart::inject(1U, bytes.data(), 1U) == 0U);
    assert(received_calls == 2U);
    std::array<std::uint8_t, mock::uart::QUEUE_CAPACITY> drain{};
    assert(port.read(drain.data(), drain.size()) == mock::uart::QUEUE_CAPACITY);

    // 发送：一次只能有一个；结束时通知并计数
    std::uint32_t write_calls = 0U;
    assert(port.set_write_notification(count, &write_calls));
    assert(port.write(bytes.data(), 3U) == WriteResult::STARTED);
    assert(port.write(bytes.data(), 3U) == WriteResult::BUSY);
    assert(mock::uart::write_pending(1U));
    assert(mock::uart::complete_write(1U));
    assert(write_calls == 1U && port.statistics().transmitted_byte_count == 3U);
    assert(port.write(bytes.data(), 2U) == WriteResult::STARTED);
    assert(mock::uart::complete_write(1U, true));
    assert(write_calls == 2U && port.statistics().transmit_failure_count == 1U);
    std::array<std::uint8_t, 8> written{};
    assert(mock::uart::take_written(1U, written.data(), written.size()) == 5U);
    assert(written[0] == 1U && written[3] == 1U && written[4] == 2U);

    // 参数与能力
    std::array<std::uint8_t, hal::Uart::MAX_WRITE_LENGTH + 1U> too_long{};
    assert(port.write(too_long.data(), too_long.size()) == WriteResult::INVALID_ARGUMENT);
    assert(port.write(bytes.data(), 0U) == WriteResult::INVALID_ARGUMENT);
    mock::uart::set_capabilities(2U, true, false); // 只接了 RX
    assert(hal::Uart{2U}.write(bytes.data(), 1U) == WriteResult::UNSUPPORTED);
    mock::uart::set_capabilities(3U, false, true);
    assert(!hal::Uart{3U}.is_ready());
    assert(hal::Uart{3U}.write(bytes.data(), 1U) == WriteResult::NOT_READY);
    const hal::Uart invalid{mock::uart::PORT_COUNT};
    assert(!invalid.is_ready());
    assert(invalid.write(bytes.data(), 1U) == WriteResult::INVALID_ARGUMENT);
    assert(invalid.read(buffer.data(), buffer.size()) == 0U);
    assert(!invalid.set_receive_notification(count, &received_calls));
    return 0;
}
