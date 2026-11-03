// hal::UsbCdc 契约 v1 在 platform/mock 上的语义，以及与 hal::Uart 同形：
// 同一个字节流模板对两者都能编译、行为一致。
#include "hal/uart.hpp"
#include "hal/usb_cdc.hpp"
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

// 回显：把读到的字节原样写回。对 hal::Uart 与 hal::UsbCdc 都能用（两者读写接口同形）。
template <typename Stream>
bool echo_once(const Stream &stream)
{
    std::array<std::uint8_t, 16> buffer{};
    const std::size_t length = stream.read(buffer.data(), buffer.size());
    return length > 0U && stream.write(buffer.data(), length) == Stream::WriteResult::STARTED;
}

} // namespace

int main()
{
    using WriteResult = hal::UsbCdc::WriteResult;
    mock::usb_cdc::reset();
    const hal::UsbCdc usb{0U};
    assert(usb.is_ready() && !usb.is_connected());

    // 没连上：不能写，收到的数据算错误
    const std::array<std::uint8_t, 3> bytes{7U, 8U, 9U};
    assert(usb.write(bytes.data(), bytes.size()) == WriteResult::NOT_READY);
    assert(mock::usb_cdc::inject(bytes.data(), bytes.size()) == 0U);
    assert(usb.statistics().receive_error_count == 1U);

    // 连上：收、发、回显
    std::uint32_t received_calls = 0U;
    std::uint32_t write_calls = 0U;
    assert(usb.set_receive_notification(count, &received_calls));
    assert(usb.set_write_notification(count, &write_calls));
    mock::usb_cdc::set_connected(true);
    assert(usb.is_connected() && usb.statistics().connection_count == 1U);
    assert(mock::usb_cdc::inject(bytes.data(), bytes.size()) == 3U && received_calls == 1U);
    assert(echo_once(usb));
    assert(usb.write(bytes.data(), 1U) == WriteResult::BUSY);
    assert(mock::usb_cdc::complete_write());
    assert(write_calls == 1U && usb.statistics().transmitted_byte_count == 3U);
    std::array<std::uint8_t, 8> written{};
    assert(mock::usb_cdc::take_written(written.data(), written.size()) == 3U && written[2] == 9U);

    // 断开：进行中的发送算失败并通知，接收队列清空
    assert(mock::usb_cdc::inject(bytes.data(), bytes.size()) == 3U);
    assert(usb.write(bytes.data(), 2U) == WriteResult::STARTED);
    mock::usb_cdc::set_connected(false);
    assert(!usb.is_connected());
    assert(write_calls == 2U && usb.statistics().transmit_failure_count == 1U);
    assert(usb.statistics().disconnection_count == 1U);
    std::array<std::uint8_t, 8> buffer{};
    assert(usb.read(buffer.data(), buffer.size()) == 0U);

    // 参数与编号
    mock::usb_cdc::set_connected(true);
    std::array<std::uint8_t, hal::UsbCdc::MAX_WRITE_LENGTH + 1U> too_long{};
    assert(usb.write(too_long.data(), too_long.size()) == WriteResult::INVALID_ARGUMENT);
    const hal::UsbCdc invalid{1U};
    assert(!invalid.is_ready() && !invalid.is_connected());
    assert(invalid.write(bytes.data(), 1U) == WriteResult::INVALID_ARGUMENT);

    // 同一个模板用在 UART 上
    mock::uart::reset();
    const hal::Uart uart{0U};
    assert(mock::uart::inject(0U, bytes.data(), bytes.size()) == 3U);
    assert(echo_once(uart));
    assert(mock::uart::take_written(0U, written.data(), written.size()) == 3U && written[0] == 7U);
    return 0;
}
