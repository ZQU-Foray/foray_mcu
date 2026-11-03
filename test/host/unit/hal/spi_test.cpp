// hal::Spi 契约 v1 在 platform/mock 上的语义：驱动的宿主测试以它为准。
#include "hal/spi.hpp"
#include "mock_hal.hpp"

#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>

namespace
{

// 设备：把收到的每个字节加 1 回送
void echo_plus_one(void *, std::uint8_t, const std::uint8_t *transmit, std::uint8_t *receive, std::size_t length)
{
    for (std::size_t index = 0U; index < length && receive != nullptr; ++index)
    {
        receive[index] = static_cast<std::uint8_t>(transmit[index] + 1U);
    }
}

struct Completion
{
    hal::Spi device;
    std::uint32_t calls{0U};
    hal::Spi::Result result_in_callback{hal::Spi::Result::ERROR};
};

// 回调里读到的已经是最终结果，且总线已释放
void on_complete(void *context) noexcept
{
    auto &completion = *static_cast<Completion *>(context);
    ++completion.calls;
    completion.result_in_callback = completion.device.async_result();
}

} // namespace

int main()
{
    using Result = hal::Spi::Result;
    mock::spi::reset();
    const hal::Spi first{0U};
    const hal::Spi second{1U};
    mock::spi::set_bus(1U, 0U); // 两个设备共用总线 0
    mock::spi::set_responder(0U, echo_plus_one, nullptr);
    static_assert(sizeof(hal::Spi) == 1U);

    // 阻塞全双工与发送
    std::array<std::uint8_t, 3> transmit{1U, 2U, 3U};
    std::array<std::uint8_t, 3> receive{};
    assert(first.is_ready());
    assert(first.async_result() == Result::NOT_READY); // 还没有异步事务
    assert(first.transfer(transmit.data(), receive.data(), receive.size(), 1000U) == Result::COMPLETED);
    assert((receive == std::array<std::uint8_t, 3>{2U, 3U, 4U}));
    assert(first.transmit(transmit.data(), transmit.size(), 1000U) == Result::COMPLETED);
    assert(mock::spi::transaction_count(0U) == 2U);

    // 参数检查
    assert(first.transmit(nullptr, 1U, 1000U) == Result::INVALID_ARGUMENT);
    assert(first.transmit(transmit.data(), 0U, 1000U) == Result::INVALID_ARGUMENT);
    assert(first.start_transfer(transmit.data(), nullptr, 3U, nullptr, nullptr) == Result::INVALID_ARGUMENT);
    std::array<std::uint8_t, hal::Spi::MAX_ASYNC_LENGTH + 1U> too_long{};
    assert(first.start_transmit(too_long.data(), too_long.size(), nullptr, nullptr) == Result::INVALID_ARGUMENT);

    // 异步：发送数据返回前拷走；总线忙时同总线的设备得到 BUSY；结束后结果为 COMPLETED，回调在释放总线之后
    Completion completion{first};
    receive.fill(0U);
    assert(first.start_transfer(transmit.data(), receive.data(), receive.size(), on_complete, &completion) ==
           Result::STARTED);
    transmit.fill(9U); // 调用方立即复用发送缓冲区
    assert(first.async_result() == Result::BUSY);
    assert(!first.is_ready() && !second.is_ready());
    assert(second.transmit(transmit.data(), 1U, 1000U) == Result::BUSY);
    assert(second.start_transmit(transmit.data(), 1U, nullptr, nullptr) == Result::BUSY);
    std::uint8_t pending_device = 0xFFU;
    assert(mock::spi::busy(0U, &pending_device) && pending_device == 0U);
    assert(completion.calls == 0U);
    assert(mock::spi::complete(0U));
    assert(completion.calls == 1U && completion.result_in_callback == Result::COMPLETED);
    assert((receive == std::array<std::uint8_t, 3>{2U, 3U, 4U})); // 按开始时的发送数据应答
    assert(second.is_ready());
    assert(!mock::spi::complete(0U)); // 没有进行中的事务

    // 出错：结果为 ERROR，不写接收缓冲区
    receive.fill(0U);
    assert(first.start_transfer(transmit.data(), receive.data(), receive.size(), on_complete, &completion) ==
           Result::STARTED);
    assert(mock::spi::complete(0U, true));
    assert(completion.calls == 2U && completion.result_in_callback == Result::ERROR);
    assert((receive == std::array<std::uint8_t, 3>{}));

    // 立即完成：回调在 start_* 返回之前就被调用
    mock::spi::set_complete_immediately(true);
    assert(first.start_transmit(transmit.data(), 1U, on_complete, &completion) == Result::STARTED);
    assert(completion.calls == 3U && first.async_result() == Result::COMPLETED);
    mock::spi::set_complete_immediately(false);

    // 能力：只能发送的设备、不支持异步的板、未初始化
    mock::spi::set_capabilities(2U, true, true, false);
    const hal::Spi transmit_only{2U};
    assert(transmit_only.transfer(transmit.data(), receive.data(), 1U, 1000U) == Result::UNSUPPORTED);
    assert(transmit_only.start_transfer(transmit.data(), receive.data(), 1U, nullptr, nullptr) == Result::UNSUPPORTED);
    assert(transmit_only.start_transmit(transmit.data(), 1U, nullptr, nullptr) == Result::STARTED);
    mock::spi::set_capabilities(3U, true, false, true);
    const hal::Spi blocking_only{3U};
    assert(blocking_only.start_transmit(transmit.data(), 1U, nullptr, nullptr) == Result::UNSUPPORTED);
    assert(blocking_only.async_result() == Result::UNSUPPORTED);
    assert(blocking_only.transmit(transmit.data(), 1U, 1000U) == Result::COMPLETED);
    mock::spi::set_capabilities(4U, false, true, true);
    assert(hal::Spi{4U}.transmit(transmit.data(), 1U, 1000U) == Result::NOT_READY);

    const hal::Spi invalid{mock::spi::DEVICE_COUNT};
    assert(!invalid.is_ready());
    assert(invalid.transmit(transmit.data(), 1U, 1000U) == Result::INVALID_ARGUMENT);
    assert(invalid.async_result() == Result::INVALID_ARGUMENT);
    return 0;
}
