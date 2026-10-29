#pragma once
#include "driver/motor/dji/dji_motor.hpp"

#include <array>
#include <cassert>
#include <cstdint>

// ---- 平台 C ABI 桩：被测代码经 platform::Can / platform::Time 调用 ----
namespace
{

struct CanStub
{
    bool ready{true};
    CanPort_Channel last_tx_channel{CAN_PORT_CHANNEL_1};
    std::uint8_t rx_count{0U};
    std::uint8_t rx_index{0U};
    struct RxFrame
    {
        std::uint32_t identifier{0U};
        std::array<std::uint8_t, 8U> data{};
    };
    std::array<RxFrame, 8U> rx{};
    std::uint32_t tx_count{0U};
    std::uint32_t last_tx_identifier{0U};
    std::array<std::uint8_t, 8U> last_tx_data{};
    CanPort_SendResult tx_result{CAN_PORT_SEND_QUEUED};
    CanPort_Statistics statistics{};
} g_can_stub;

struct TimeStub
{
    std::uint32_t ticks{0U};
} g_time_stub;

void feed_rx(std::uint32_t identifier, const std::array<std::uint8_t, 8U> &data)
{
    g_can_stub.rx[g_can_stub.rx_count] = CanStub::RxFrame{identifier, data};
    ++g_can_stub.rx_count;
}

void reset_rx()
{
    g_can_stub.rx_count = 0U;
    g_can_stub.rx_index = 0U;
}

} // namespace

extern "C"
{
    bool CanPort_Init(void)
    {
        return true;
    }

    bool CanPort_IsReady(CanPort_Channel)
    {
        return g_can_stub.ready;
    }

    bool CanPort_ConfigureStandardReceiveFilter(CanPort_Channel, bool, std::uint32_t, std::uint32_t)
    {
        return false;
    }

    bool CanPort_SetReceiveNotification(CanPort_Channel, CanPort_ReceiveNotification, void *)
    {
        return false;
    }

    CanPort_SendResult CanPort_TrySend(CanPort_Channel channel,
                                       std::uint32_t identifier,
                                       CanPort_IdentifierType,
                                       std::uint8_t length,
                                       const std::uint8_t *data)
    {
        g_can_stub.last_tx_channel = channel;
        ++g_can_stub.tx_count;
        g_can_stub.last_tx_identifier = identifier;
        for (std::uint8_t index = 0U; index < 8U; ++index)
        {
            g_can_stub.last_tx_data[index] = index < length ? data[index] : 0U;
        }
        return g_can_stub.tx_result;
    }

    CanPort_ReceiveResult CanPort_TryReceive(CanPort_Channel,
                                             std::uint32_t *identifier,
                                             CanPort_IdentifierType *identifier_type,
                                             std::uint8_t *length,
                                             std::uint8_t *data,
                                             std::uint8_t data_capacity)
    {
        if (g_can_stub.rx_index >= g_can_stub.rx_count)
        {
            return CAN_PORT_RECEIVE_EMPTY;
        }
        const CanStub::RxFrame &frame = g_can_stub.rx[g_can_stub.rx_index];
        ++g_can_stub.rx_index;
        *identifier = frame.identifier;
        *identifier_type = CAN_PORT_IDENTIFIER_STANDARD;
        *length = 8U;
        for (std::uint8_t index = 0U; index < 8U && index < data_capacity; ++index)
        {
            data[index] = frame.data[index];
        }
        return CAN_PORT_RECEIVE_RECEIVED;
    }

    bool CanPort_GetStatistics(CanPort_Channel, CanPort_Statistics *statistics)
    {
        *statistics = g_can_stub.statistics;
        return true;
    }

    bool TimePort_Init(void)
    {
        return true;
    }
    bool TimePort_IsReady(void)
    {
        return true;
    }
    std::uint32_t TimePort_NowTicks(void)
    {
        return g_time_stub.ticks;
    }
    std::uint32_t TimePort_FrequencyHz(void)
    {
        return 1000U;
    }
    void TimePort_DelayUs(std::uint32_t) {}

} // extern "C"
