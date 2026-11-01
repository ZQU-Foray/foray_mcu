// hal::Can 在 DM-MC02 上的实现（Decision-20261101-01）：FDCAN1–3 按 Classic CAN 工作，总线表的下标就是
// board.hpp 里句柄的编号。接收：FIFO0 新消息中断把帧排进每条总线的软件队列（SPSC：中断写、接收方读）。
// 覆盖 HAL 弱符号 HAL_FDCAN_RxFifo0Callback、HAL_FDCAN_ErrorStatusCallback：fdcan.h 带进 HAL 头，
// 定义因此是 C 链接（规则 16）。
#include "hal/can.hpp"

#include "board.hpp"
#include "fdcan.h" // hfdcan1–3；带进 stm32h7xx_hal.h
#include "hal/os/critical_section.hpp"
#include "port.hpp"
#include "utils/concurrency/spsc_queue.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace
{

constexpr std::size_t RECEIVE_QUEUE_CAPACITY{128U}; // 帧；1 Mbps 下约 15 ms 的满载流量
constexpr std::uint32_t ISR_DRAIN_LIMIT{16U};       // 一次中断最多搬运的帧数，FIFO0 深度由 CubeMX 配置
constexpr std::uint32_t STANDARD_IDENTIFIER_MAX{0x7FFU};
constexpr std::uint32_t EXTENDED_IDENTIFIER_MAX{0x1FFFFFFFU};
constexpr std::uint32_t NOTIFICATIONS{FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_MESSAGE_LOST |
                                      FDCAN_IT_BUS_OFF};

struct Bus
{
    concurrency::SpscQueue<hal::Can::Frame, RECEIVE_QUEUE_CAPACITY> receive_queue{};
    hal::Can::Statistics statistics{}; // 只在中断里写；任务在临界区内读
    port::Notifier receive_notification{};
    bool receiver_claimed{false}; // 只在临界区内读写
};

// 顺序与 board.hpp 一致
const std::array<FDCAN_HandleTypeDef *, board::CAN_COUNT> HANDLES{{&hfdcan1, &hfdcan2, &hfdcan3}};
std::array<Bus, board::CAN_COUNT> g_buses{};
bool g_ready{false};

int index_of(const FDCAN_HandleTypeDef *handle) noexcept
{
    for (std::size_t index = 0U; index < HANDLES.size(); ++index)
    {
        if (HANDLES[index] == handle)
        {
            return static_cast<int>(index);
        }
    }
    return -1;
}

bool is_bus_off(FDCAN_HandleTypeDef *handle) noexcept
{
    FDCAN_ProtocolStatusTypeDef status{};
    return HAL_FDCAN_GetProtocolStatus(handle, &status) != HAL_OK || status.BusOff != 0U;
}

// 一个范围过滤器收下全部标准帧或扩展帧，送进 FIFO0
bool accept_all(FDCAN_HandleTypeDef *handle, std::uint32_t identifier_type, std::uint32_t last_identifier) noexcept
{
    FDCAN_FilterTypeDef filter{};
    filter.IdType = identifier_type;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_RANGE;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0U;
    filter.FilterID2 = last_identifier;
    return HAL_FDCAN_ConfigFilter(handle, &filter) == HAL_OK;
}

bool start(FDCAN_HandleTypeDef *handle) noexcept
{
    if (HAL_FDCAN_GetState(handle) != HAL_FDCAN_STATE_READY ||
        !accept_all(handle, FDCAN_STANDARD_ID, STANDARD_IDENTIFIER_MAX) ||
        !accept_all(handle, FDCAN_EXTENDED_ID, EXTENDED_IDENTIFIER_MAX) ||
        HAL_FDCAN_ConfigGlobalFilter(handle, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) !=
            HAL_OK ||
        HAL_FDCAN_Start(handle) != HAL_OK)
    {
        return false;
    }
    if (HAL_FDCAN_ActivateNotification(handle, NOTIFICATIONS, 0U) != HAL_OK)
    {
        (void)HAL_FDCAN_Stop(handle);
        return false;
    }
    return true;
}

bool is_supported(const FDCAN_RxHeaderTypeDef &header) noexcept
{
    return header.RxFrameType == FDCAN_DATA_FRAME && header.FDFormat == FDCAN_CLASSIC_CAN &&
           header.DataLength <= hal::Can::MAX_DATA_LENGTH &&
           (header.IdType == FDCAN_STANDARD_ID || header.IdType == FDCAN_EXTENDED_ID);
}

// 中断里：把 FIFO0 里的帧搬进软件队列。队列满或格式不支持的帧丢弃并计数。
void drain(Bus &bus, FDCAN_HandleTypeDef *handle) noexcept
{
    for (std::uint32_t drained = 0U;
         drained < ISR_DRAIN_LIMIT && HAL_FDCAN_GetRxFifoFillLevel(handle, FDCAN_RX_FIFO0) != 0U;
         ++drained)
    {
        FDCAN_RxHeaderTypeDef header{};
        hal::Can::Frame frame{};
        if (HAL_FDCAN_GetRxMessage(handle, FDCAN_RX_FIFO0, &header, frame.data.data()) != HAL_OK)
        {
            ++bus.statistics.rx_dropped_count;
            return;
        }
        if (!is_supported(header))
        {
            ++bus.statistics.rx_dropped_count;
            continue;
        }
        frame.identifier = header.Identifier;
        frame.identifier_type = header.IdType == FDCAN_STANDARD_ID ? hal::Can::IdentifierType::STANDARD
                                                                   : hal::Can::IdentifierType::EXTENDED;
        frame.length = static_cast<std::uint8_t>(header.DataLength); // H7 HAL 的 DataLength 在 Classic CAN 下就是字节数
        if (!bus.receive_queue.push(frame))
        {
            ++bus.statistics.rx_dropped_count;
        }
    }
}

} // namespace

bool port::init_can() noexcept
{
    g_ready = false;
    for (std::size_t started = 0U; started < HANDLES.size(); ++started)
    {
        if (!start(HANDLES[started]))
        {
            for (std::size_t index = 0U; index < started; ++index)
            {
                (void)HAL_FDCAN_DeactivateNotification(HANDLES[index], NOTIFICATIONS);
                (void)HAL_FDCAN_Stop(HANDLES[index]);
            }
            return false;
        }
    }
    g_ready = true;
    return true;
}

namespace hal
{

bool Can::is_ready() const noexcept
{
    return g_ready && index_ < HANDLES.size() && !is_bus_off(HANDLES[index_]);
}

bool Can::claim_receiver() const noexcept
{
    if (index_ >= HANDLES.size())
    {
        return false;
    }
    const os::CriticalSection guard;
    Bus &bus = g_buses[index_];
    if (bus.receiver_claimed)
    {
        return false;
    }
    bus.receiver_claimed = true;
    return true;
}

Can::SendResult Can::send(const Frame &frame) const noexcept
{
    const bool standard = frame.identifier_type == IdentifierType::STANDARD;
    if (index_ >= HANDLES.size() || frame.length > MAX_DATA_LENGTH ||
        frame.identifier > (standard ? STANDARD_IDENTIFIER_MAX : EXTENDED_IDENTIFIER_MAX))
    {
        return SendResult::INVALID_ARGUMENT;
    }
    FDCAN_HandleTypeDef *const handle = HANDLES[index_];
    if (!g_ready)
    {
        return SendResult::NOT_READY;
    }
    if (is_bus_off(handle))
    {
        return SendResult::BUS_OFF;
    }

    FDCAN_TxHeaderTypeDef header{};
    header.Identifier = frame.identifier;
    header.IdType = standard ? FDCAN_STANDARD_ID : FDCAN_EXTENDED_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = frame.length; // Classic CAN：字节数
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0U;

    // 查空位与写入硬件队列之间不能被别的发送方插入：多个任务可能向同一条总线发送
    const os::CriticalSection guard;
    if (HAL_FDCAN_GetTxFifoFreeLevel(handle) == 0U)
    {
        return SendResult::QUEUE_FULL;
    }
    return HAL_FDCAN_AddMessageToTxFifoQ(handle, &header, frame.data.data()) == HAL_OK ? SendResult::QUEUED
                                                                                       : SendResult::ERROR;
}

Can::ReceiveResult Can::receive(Frame &frame) const noexcept
{
    if (index_ >= HANDLES.size())
    {
        return ReceiveResult::INVALID_ARGUMENT;
    }
    if (g_buses[index_].receive_queue.pop(frame))
    {
        return ReceiveResult::RECEIVED;
    }
    if (!g_ready)
    {
        return ReceiveResult::NOT_READY;
    }
    return is_bus_off(HANDLES[index_]) ? ReceiveResult::BUS_OFF : ReceiveResult::EMPTY;
}

bool Can::set_receive_notification(Callback callback, void *context) const noexcept
{
    if (index_ >= HANDLES.size())
    {
        return false;
    }
    g_buses[index_].receive_notification.set(callback, context);
    return true;
}

Can::Statistics Can::statistics() const noexcept
{
    if (index_ >= HANDLES.size())
    {
        return Statistics{};
    }
    const os::CriticalSection guard; // 计数在中断里写，三个一起读才一致
    return g_buses[index_].statistics;
}

} // namespace hal

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *handle, std::uint32_t fifo_interrupts)
{
    const int index = index_of(handle);
    if (index < 0)
    {
        return;
    }
    Bus &bus = g_buses[static_cast<std::size_t>(index)];
    if ((fifo_interrupts & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0U)
    {
        ++bus.statistics.rx_hardware_loss_event_count;
    }
    drain(bus, handle);
    bus.receive_notification.fire();
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *handle, std::uint32_t error_status_interrupts)
{
    const int index = index_of(handle);
    if (index < 0 || (error_status_interrupts & FDCAN_IT_BUS_OFF) == 0U)
    {
        return;
    }
    ++g_buses[static_cast<std::size_t>(index)].statistics.bus_off_count;
    // Bus-Off 时硬件置 CCCR.INIT 停止收发。清掉 INIT 即发起恢复：控制器等满 128 × 11 个隐性位后重新加入总线
    // （RM0468 FDCAN"Bus-Off recovery"）。HAL 的句柄状态一直是 BUSY，不用改。
    CLEAR_BIT(handle->Instance->CCCR, FDCAN_CCCR_INIT);
}
