// device::CanBus（Decision-20261102-03 D9）在 platform/mock 上：按帧 ID 分发、区间重叠、兜底、没人认领、
// 每周期回调的顺序、总线占用、负载估算。
#include "driver/can_bus/can_bus.hpp"
#include "mock_hal.hpp"
#include "mock_os.hpp"

#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>

namespace
{

using device::CanBus;
using Filter = CanBus::Filter;

constexpr std::uint8_t BUS{0U};

struct Recorder final
{
    std::uint32_t frames{0U};
    std::uint32_t last_identifier{0U};
    hal::os::Microseconds last_received_us{0U};
    std::uint32_t transmits{0U};
    hal::os::Microseconds last_now{0U};
    std::uint32_t order{0U}; // 最后一次每周期回调时的全局序号
};

std::uint32_t g_sequence{0U};

void on_frame(void *context, const hal::Can::Frame &frame) noexcept
{
    auto *recorder = static_cast<Recorder *>(context);
    ++recorder->frames;
    recorder->last_identifier = frame.identifier;
    recorder->last_received_us = frame.received_us;
}

void on_transmit(void *context, hal::os::Microseconds now) noexcept
{
    auto *recorder = static_cast<Recorder *>(context);
    ++recorder->transmits;
    recorder->last_now = now;
    recorder->order = ++g_sequence;
}

CanBus::Client client_of(Recorder &recorder, bool transmits = true)
{
    return CanBus::Client{&recorder, on_frame, transmits ? on_transmit : nullptr};
}

void inject(std::uint32_t identifier, hal::Can::IdentifierType type = hal::Can::IdentifierType::STANDARD)
{
    hal::Can::Frame frame{};
    frame.identifier = identifier;
    frame.identifier_type = type;
    frame.length = 8U;
    assert(mock::can::inject(BUS, frame));
}

void reset()
{
    mock::reset();
    mock::can::reset();
    g_sequence = 0U;
}

void test_dispatch()
{
    reset();
    CanBus bus{hal::Can{BUS}};
    Recorder dji;
    Recorder dm;
    Recorder capacitor;
    const std::array<Filter, 1> dji_filters{{{0x201U, 0x208U}}};
    const std::array<Filter, 2> dm_filters{{{0x011U, 0x011U}, {0x012U, 0x012U}}};
    const std::array<Filter, 1> capacitor_filters{{{0x1ABCDEU, 0x1ABCDEU, hal::Can::IdentifierType::EXTENDED}}};
    assert(bus.attach(client_of(dji), dji_filters.data(), dji_filters.size()));
    assert(bus.attach(client_of(dm), dm_filters.data(), dm_filters.size()));
    assert(bus.attach(client_of(capacitor, false), capacitor_filters.data(), capacitor_filters.size()));

    // init() 之前不收也不调回调
    inject(0x201U);
    bus.receive();
    bus.transmit(5U);
    assert(dji.frames == 0U && dji.transmits == 0U && !bus.is_ready());
    assert(bus.init() && bus.init() && bus.is_ready());

    mock::set_now_us(1234U);
    inject(0x205U);
    inject(0x011U);
    inject(0x012U);
    inject(0x1ABCDEU, hal::Can::IdentifierType::EXTENDED);
    inject(0x1ABCDEU); // 同号的标准帧：不属于扩展帧的登记项
    inject(0x300U);
    bus.receive();
    assert(dji.frames == 2U && dji.last_identifier == 0x205U); // 包括 init() 之前排队的 0x201
    assert(dji.last_received_us == 1234U);                     // 帧带着接收时刻（Decision-20261102-04）
    assert(dm.frames == 2U && dm.last_identifier == 0x012U);
    assert(capacitor.frames == 1U);
    CanBus::Statistics statistics = bus.statistics();
    assert(statistics.received_count == 7U && statistics.unclaimed_count == 2U);

    // 每周期回调按登记顺序；只收不发的驱动没有回调
    bus.transmit(2000U);
    assert(dji.transmits == 1U && dm.transmits == 1U && capacitor.transmits == 0U);
    assert(dji.order < dm.order && dm.last_now == 2000U);

    // 驱动用 bus() 直接发送
    hal::Can::Frame frame{};
    frame.identifier = 0x200U;
    frame.length = 8U;
    assert(bus.bus().send(frame) == hal::Can::SendResult::QUEUED && mock::can::sent_count(BUS) == 1U);
}

void test_attach_rules()
{
    reset();
    CanBus bus{hal::Can{BUS}};
    Recorder first;
    Recorder second;
    const std::array<Filter, 1> range{{{0x201U, 0x208U}}};
    assert(bus.attach(client_of(first), range.data(), range.size()));
    // 与已登记的重叠：拒绝，什么都不登记
    const std::array<Filter, 2> overlapping{{{0x011U, 0x011U}, {0x208U, 0x20BU}}};
    assert(!bus.attach(client_of(second), overlapping.data(), overlapping.size()));
    // 本次的两个区间互相重叠
    const std::array<Filter, 2> self_overlapping{{{0x011U, 0x012U}, {0x012U, 0x013U}}};
    assert(!bus.attach(client_of(second), self_overlapping.data(), self_overlapping.size()));
    // 区间颠倒、超出标准帧范围、没有收帧回调、没有区间
    const std::array<Filter, 1> reversed{{{0x20U, 0x10U}}};
    assert(!bus.attach(client_of(second), reversed.data(), reversed.size()));
    const std::array<Filter, 1> too_large{{{0x700U, 0x800U}}};
    assert(!bus.attach(client_of(second), too_large.data(), too_large.size()));
    const std::array<Filter, 1> fine{{{0x011U, 0x011U}}};
    assert(!bus.attach(CanBus::Client{&second, nullptr, on_transmit}, fine.data(), fine.size()));
    assert(!bus.attach(client_of(second), fine.data(), 0U));
    assert(!bus.attach(client_of(second), nullptr, 1U));
    // 前面失败的都没留下：0x011 还能登记
    assert(bus.attach(client_of(second), fine.data(), fine.size()));
    assert(bus.init());
    inject(0x011U);
    inject(0x203U);
    bus.receive();
    assert(first.frames == 1U && second.frames == 1U);

    // 登记表满
    CanBus full{hal::Can{1U}};
    Recorder recorders[CanBus::MAXIMUM_CLIENTS + 1U];
    for (std::size_t index = 0U; index < CanBus::MAXIMUM_CLIENTS; ++index)
    {
        const std::array<Filter, 1> single{
            {{static_cast<std::uint32_t>(0x100U + index), static_cast<std::uint32_t>(0x100U + index)}}};
        assert(full.attach(client_of(recorders[index]), single.data(), single.size()));
    }
    const std::array<Filter, 1> extra{{{0x500U, 0x500U}}};
    assert(!full.attach(client_of(recorders[CanBus::MAXIMUM_CLIENTS]), extra.data(), extra.size()));
}

void test_fallback_and_claim()
{
    reset();
    CanBus bus{hal::Can{BUS}};
    Recorder motor;
    Recorder logger;
    const std::array<Filter, 1> range{{{0x201U, 0x204U}}};
    assert(bus.attach(client_of(motor), range.data(), range.size()));
    assert(bus.attach_fallback(client_of(logger, false)));
    assert(!bus.attach_fallback(client_of(logger, false))); // 只能有一个兜底
    assert(bus.init());
    inject(0x202U);
    inject(0x123U);
    inject(0x7FFU);
    bus.receive();
    assert(motor.frames == 1U && logger.frames == 2U && bus.statistics().unclaimed_count == 0U);

    // 每条总线只有一个接收方：别人已占住时 init() 失败；总线没启动时失败
    CanBus second{hal::Can{BUS}};
    assert(!second.init());
    mock::can::set_ready(1U, false);
    CanBus not_ready{hal::Can{1U}};
    assert(!not_ready.init());
    mock::can::set_ready(1U, true);
    assert(not_ready.init());
}

// 负载估算：每帧按 135 位计
static_assert(device::can_load_percent(0U) == 0U);
static_assert(device::can_load_percent(5000U) == 68U);           // DJI 4 台：每毫秒 5 帧
static_assert(device::can_load_percent(2000U) == 27U);           // 达妙 1 台：每毫秒 2 帧
static_assert(device::can_load_percent(8000U) == 108U);          // 达妙 4 台：超载
static_assert(device::can_load_percent(8000U, 2000000U) == 54U); // 换成 2 Mbps

} // namespace

int main()
{
    test_dispatch();
    test_attach_rules();
    test_fallback_and_claim();
    return 0;
}
