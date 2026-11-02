#include "driver/can_bus/can_bus.hpp"

namespace device
{

namespace
{

constexpr std::uint32_t STANDARD_IDENTIFIER_MAX{0x7FFU};
constexpr std::uint32_t EXTENDED_IDENTIFIER_MAX{0x1FFFFFFFU};

bool filter_valid(const CanBus::Filter &filter)
{
    const std::uint32_t maximum =
        filter.type == hal::Can::IdentifierType::STANDARD ? STANDARD_IDENTIFIER_MAX : EXTENDED_IDENTIFIER_MAX;
    return filter.first <= filter.last && filter.last <= maximum;
}

bool overlaps(const CanBus::Filter &a, const CanBus::Filter &b)
{
    return a.type == b.type && a.first <= b.last && b.first <= a.last;
}

bool matches(const CanBus::Filter &filter, const hal::Can::Frame &frame)
{
    return filter.type == frame.identifier_type && frame.identifier >= filter.first && frame.identifier <= filter.last;
}

} // namespace

bool CanBus::attach(const Client &client, const Filter *filters, std::size_t count) noexcept
{
    if (client.on_frame == nullptr || filters == nullptr || count == 0U || route_count_ + count > routes_.size() ||
        client_count_ >= clients_.size())
    {
        return false;
    }
    for (std::size_t index = 0U; index < count; ++index)
    {
        if (!filter_valid(filters[index]))
        {
            return false;
        }
        // 与已登记的、与本次的其他区间都不能重叠
        for (std::size_t route = 0U; route < route_count_; ++route)
        {
            if (overlaps(filters[index], routes_[route].filter))
            {
                return false;
            }
        }
        for (std::size_t other = 0U; other < index; ++other)
        {
            if (overlaps(filters[index], filters[other]))
            {
                return false;
            }
        }
    }
    std::uint8_t client_index = 0U;
    if (!add_client(client, client_index))
    {
        return false;
    }
    for (std::size_t index = 0U; index < count; ++index)
    {
        routes_[route_count_++] = Route{filters[index], client_index};
    }
    return true;
}

bool CanBus::attach_fallback(const Client &client) noexcept
{
    if (client.on_frame == nullptr || has_fallback_)
    {
        return false;
    }
    // 兜底的每周期回调也按登记顺序调用，所以同样占一个登记位
    std::uint8_t client_index = 0U;
    if (client.on_transmit != nullptr && !add_client(client, client_index))
    {
        return false;
    }
    fallback_ = client;
    has_fallback_ = true;
    return true;
}

bool CanBus::add_client(const Client &client, std::uint8_t &index) noexcept
{
    if (client_count_ >= clients_.size())
    {
        return false;
    }
    index = static_cast<std::uint8_t>(client_count_);
    clients_[client_count_++] = client;
    return true;
}

bool CanBus::init() noexcept
{
    if (claimed_)
    {
        return true;
    }
    if (!bus_.is_ready() || !bus_.claim_receiver())
    {
        return false;
    }
    claimed_ = true;
    return true;
}

void CanBus::receive() noexcept
{
    if (!claimed_)
    {
        return;
    }
    hal::Can::Frame frame{};
    while (bus_.receive(frame) == hal::Can::ReceiveResult::RECEIVED)
    {
        ++received_count_;
        bool routed = false;
        for (std::size_t route = 0U; route < route_count_ && !routed; ++route)
        {
            if (matches(routes_[route].filter, frame))
            {
                const Client &client = clients_[routes_[route].client];
                client.on_frame(client.context, frame);
                routed = true;
            }
        }
        if (!routed)
        {
            if (has_fallback_)
            {
                fallback_.on_frame(fallback_.context, frame);
            }
            else
            {
                ++unclaimed_count_;
            }
        }
    }
}

void CanBus::transmit(hal::os::Microseconds now) noexcept
{
    if (!claimed_)
    {
        return;
    }
    for (std::size_t index = 0U; index < client_count_; ++index)
    {
        if (clients_[index].on_transmit != nullptr)
        {
            clients_[index].on_transmit(clients_[index].context, now);
        }
    }
}

CanBus::Statistics CanBus::statistics() const noexcept
{
    Statistics result{};
    result.received_count = received_count_;
    result.unclaimed_count = unclaimed_count_;
    result.port = bus_.statistics();
    return result;
}

} // namespace device
