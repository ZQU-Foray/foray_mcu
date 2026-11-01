// WS2812 驱动（device::Ws2812）在 platform/mock 上：波形编码、亮度、异步刷新与不支持异步时的阻塞回退。
#include "driver/ws2812/ws2812.hpp"
#include "mock_hal.hpp"

#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>

namespace
{

constexpr std::uint8_t LED{2U};

// 从发出的波形里解出一个颜色字节：每位一个 SPI 字节，0x78 为 1、0x60 为 0
std::uint8_t decode(const std::array<std::uint8_t, device::Ws2812::FRAME_SIZE> &frame, std::size_t offset)
{
    std::uint8_t value = 0U;
    for (std::size_t bit = 0U; bit < 8U; ++bit)
    {
        assert(frame[offset + bit] == 0x78U || frame[offset + bit] == 0x60U);
        value = static_cast<std::uint8_t>((value << 1U) | (frame[offset + bit] == 0x78U ? 1U : 0U));
    }
    return value;
}

std::array<std::uint8_t, device::Ws2812::FRAME_SIZE> last_frame()
{
    std::array<std::uint8_t, device::Ws2812::FRAME_SIZE> frame{};
    assert(mock::spi::last_transmit(LED, frame.data(), frame.size()) == frame.size());
    return frame;
}

} // namespace

int main()
{
    mock::spi::reset();
    mock::spi::set_capabilities(LED, true, true, false); // 与 DM-MC02 的 SPI6 一样只能发送
    device::Ws2812 led{hal::Spi{LED}};

    // 上电后第一次 process() 刷新一次（黑色）
    led.process();
    assert(led.get_state() == device::Ws2812::State::TRANSMITTING);
    assert(mock::spi::transaction_count(LED) == 1U);
    led.process(); // 还在发送
    assert(led.get_state() == device::Ws2812::State::TRANSMITTING);
    assert(mock::spi::complete(LED));
    led.process();
    assert(led.get_state() == device::Ws2812::State::IDLE);
    led.process(); // 没有变化：不再发送
    assert(mock::spi::transaction_count(LED) == 1U);

    // 颜色按 G、R、B 顺序编码，复位段为 0
    led.set_color({0x12U, 0x34U, 0x56U});
    led.process();
    auto frame = last_frame();
    assert(decode(frame, 0U) == 0x34U && decode(frame, 8U) == 0x12U && decode(frame, 16U) == 0x56U);
    for (std::size_t index = device::Ws2812::ENCODED_COLOR_SIZE; index < frame.size(); ++index)
    {
        assert(frame[index] == 0U);
    }
    assert(mock::spi::complete(LED));
    led.process();

    // 亮度缩放（四舍五入）
    led.set_brightness(128U);
    led.process();
    frame = last_frame();
    assert(decode(frame, 0U) == 0x1AU && decode(frame, 8U) == 0x09U && decode(frame, 16U) == 0x2BU);
    assert(mock::spi::complete(LED, true)); // DMA 出错
    led.process();
    assert(led.get_state() == device::Ws2812::State::ERROR);
    led.process(); // 出错后重发
    assert(mock::spi::transaction_count(LED) == 4U);
    assert(mock::spi::complete(LED));
    led.process();
    assert(led.get_state() == device::Ws2812::State::IDLE);

    // 板不支持 SPI 异步：退回阻塞发送，一次 process() 就完成
    mock::spi::reset();
    mock::spi::set_capabilities(LED, true, false, false);
    device::Ws2812 blocking_led{hal::Spi{LED}};
    blocking_led.set_color(device::ws2812_color::RED);
    blocking_led.process();
    assert(blocking_led.get_state() == device::Ws2812::State::IDLE);
    assert(mock::spi::transaction_count(LED) == 1U);
    assert(decode(last_frame(), 8U) == 0xFFU);
    return 0;
}
