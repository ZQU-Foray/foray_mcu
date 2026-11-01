// hal::Uart 在 DM-MC02 上的实现（Decision-20261101-01），由迁移期的 uart.c 改写，收发逻辑不变。
// 端口表的下标就是 board.hpp 里句柄的编号。接收：DMA 接收到空闲（ReceiveToIdle），每次空闲或满缓冲时
// 把收到的字节放进软件队列（SPSC：中断写、任务读），然后重新开始接收。
// 覆盖 HAL 弱符号 HAL_UART_TxCpltCallback、HAL_UARTEx_RxEventCallback、HAL_UART_ErrorCallback：usart.h 带进
// HAL 头，定义因此是 C 链接（规则 16）。
#include "hal/uart.hpp"

#include "board.hpp"
#include "hal/os/critical_section.hpp"
#include "port.hpp"
#include "usart.h" // huart5、huart10；带进 stm32h7xx_hal.h
#include "utils/concurrency/spsc_queue.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace
{

constexpr std::uint32_t CACHE_LINE_SIZE{32U};
constexpr std::uint16_t RECEIVE_DMA_CAPACITY{64U};
constexpr std::size_t RECEIVE_QUEUE_CAPACITY{256U};
constexpr std::size_t TRANSMIT_DMA_CAPACITY{hal::Uart::MAX_WRITE_LENGTH};
static_assert(RECEIVE_DMA_CAPACITY % CACHE_LINE_SIZE == 0U && TRANSMIT_DMA_CAPACITY % CACHE_LINE_SIZE == 0U,
              "DMA 缓冲区按缓存行对齐");

__attribute__((section(".dma_buffer"),
               aligned(CACHE_LINE_SIZE))) std::uint8_t g_uart5_receive_buffer[RECEIVE_DMA_CAPACITY];
__attribute__((section(".dma_buffer"),
               aligned(CACHE_LINE_SIZE))) std::uint8_t g_uart10_receive_buffer[RECEIVE_DMA_CAPACITY];
__attribute__((section(".dma_buffer"),
               aligned(CACHE_LINE_SIZE))) std::uint8_t g_uart10_transmit_buffer[TRANSMIT_DMA_CAPACITY];

struct Mapping
{
    UART_HandleTypeDef *handle;
    std::uint8_t *receive_buffer;
    std::uint8_t *transmit_buffer; // 为空：只能接收
};

struct Port
{
    concurrency::SpscQueue<std::uint8_t, RECEIVE_QUEUE_CAPACITY> receive_queue{};
    hal::Uart::Statistics statistics{}; // 中断里写；任务在临界区内读
    port::Notifier receive_notification{};
    port::Notifier write_notification{};
    std::uint32_t transmit_length{0U}; // 以下只在临界区内或框架中断里读写
    bool transmit_active{false};
    bool receiving{false};
};

// 顺序与 board.hpp 一致
const std::array<Mapping, board::UART_COUNT> MAPPINGS{{
    {&huart5, g_uart5_receive_buffer, nullptr},                    // UART_SBUS：只接了 RX
    {&huart10, g_uart10_receive_buffer, g_uart10_transmit_buffer}, // UART_10
}};

std::array<Port, board::UART_COUNT> g_ports{};

int index_of(const UART_HandleTypeDef *handle) noexcept
{
    for (std::size_t index = 0U; index < MAPPINGS.size(); ++index)
    {
        if (MAPPINGS[index].handle == handle)
        {
            return static_cast<int>(index);
        }
    }
    return -1;
}

bool data_cache_enabled() noexcept
{
    return (SCB->CCR & SCB_CCR_DC_Msk) != 0U;
}

bool is_receiving(const Mapping &mapping) noexcept
{
    const UART_HandleTypeDef *const handle = mapping.handle;
    return handle->hdmarx != nullptr && handle->RxState == HAL_UART_STATE_BUSY_RX &&
           handle->ReceptionType == HAL_UART_RECEPTION_TOIDLE &&
           HAL_DMA_GetState(handle->hdmarx) == HAL_DMA_STATE_BUSY && (handle->Instance->CR3 & USART_CR3_DMAR) != 0U;
}

bool start_receive(const Mapping &mapping) noexcept
{
    UART_HandleTypeDef *const handle = mapping.handle;
    if (handle->hdmarx == nullptr || handle->RxState != HAL_UART_STATE_READY ||
        HAL_DMA_GetState(handle->hdmarx) != HAL_DMA_STATE_READY)
    {
        return false;
    }
    if (data_cache_enabled())
    {
        SCB_CleanInvalidateDCache_by_Addr(mapping.receive_buffer, RECEIVE_DMA_CAPACITY);
    }
    if (HAL_UARTEx_ReceiveToIdle_DMA(handle, mapping.receive_buffer, RECEIVE_DMA_CAPACITY) != HAL_OK)
    {
        return false;
    }
    __HAL_DMA_DISABLE_IT(handle->hdmarx, DMA_IT_HT); // 只在空闲或满缓冲时通知，不要半满中断
    return is_receiving(mapping);
}

// 接收出错后重新开始；失败则端口停止接收
void restart_receive(Port &port, const Mapping &mapping) noexcept
{
    if (!is_receiving(mapping) && !start_receive(mapping))
    {
        ++port.statistics.restart_failure_count;
        port.receiving = false;
    }
}

} // namespace

bool port::init_uart() noexcept
{
    for (std::size_t index = 0U; index < MAPPINGS.size(); ++index)
    {
        Port &port = g_ports[index];
        port.receiving = true; // 先置位：接收一开始中断就可能进来
        if (!start_receive(MAPPINGS[index]))
        {
            port.receiving = false;
            return false;
        }
    }
    return true;
}

namespace hal
{

bool Uart::is_ready() const noexcept
{
    return index_ < MAPPINGS.size() && g_ports[index_].receiving && is_receiving(MAPPINGS[index_]);
}

std::size_t Uart::read(std::uint8_t *data, std::size_t capacity) const noexcept
{
    if (index_ >= MAPPINGS.size() || data == nullptr)
    {
        return 0U;
    }
    return g_ports[index_].receive_queue.pop(data, capacity);
}

Uart::WriteResult Uart::write(const std::uint8_t *data, std::size_t length) const noexcept
{
    if (index_ >= MAPPINGS.size() || data == nullptr || length == 0U || length > MAX_WRITE_LENGTH)
    {
        return WriteResult::INVALID_ARGUMENT;
    }
    const Mapping &mapping = MAPPINGS[index_];
    if (mapping.transmit_buffer == nullptr || mapping.handle->hdmatx == nullptr)
    {
        return WriteResult::UNSUPPORTED;
    }
    Port &port = g_ports[index_];
    if (!port.receiving)
    {
        return WriteResult::NOT_READY;
    }

    const os::CriticalSection guard;
    if (port.transmit_active || mapping.handle->gState != HAL_UART_STATE_READY ||
        HAL_DMA_GetState(mapping.handle->hdmatx) != HAL_DMA_STATE_READY)
    {
        return WriteResult::BUSY;
    }
    std::memcpy(mapping.transmit_buffer, data, length);
    if (data_cache_enabled())
    {
        const auto aligned = static_cast<std::int32_t>((length + CACHE_LINE_SIZE - 1U) & ~(CACHE_LINE_SIZE - 1U));
        SCB_CleanDCache_by_Addr(mapping.transmit_buffer, aligned);
    }
    port.transmit_length = static_cast<std::uint32_t>(length);
    port.transmit_active = true;
    if (HAL_UART_Transmit_DMA(mapping.handle, mapping.transmit_buffer, static_cast<std::uint16_t>(length)) != HAL_OK)
    {
        port.transmit_active = false;
        port.transmit_length = 0U;
        ++port.statistics.transmit_failure_count;
        return WriteResult::ERROR;
    }
    return WriteResult::STARTED;
}

bool Uart::set_receive_notification(Callback callback, void *context) const noexcept
{
    if (index_ >= MAPPINGS.size())
    {
        return false;
    }
    g_ports[index_].receive_notification.set(callback, context);
    return true;
}

bool Uart::set_write_notification(Callback callback, void *context) const noexcept
{
    if (index_ >= MAPPINGS.size())
    {
        return false;
    }
    g_ports[index_].write_notification.set(callback, context);
    return true;
}

Uart::Statistics Uart::statistics() const noexcept
{
    if (index_ >= MAPPINGS.size())
    {
        return Statistics{};
    }
    const os::CriticalSection guard;
    return g_ports[index_].statistics;
}

} // namespace hal

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *handle)
{
    const int index = index_of(handle);
    if (index < 0)
    {
        return;
    }
    Port &port = g_ports[static_cast<std::size_t>(index)];
    if (!port.transmit_active)
    {
        return;
    }
    port.statistics.transmitted_byte_count += port.transmit_length;
    port.transmit_length = 0U;
    port.transmit_active = false;
    port.write_notification.fire();
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *handle, std::uint16_t size)
{
    const int index = index_of(handle);
    if (index < 0)
    {
        return;
    }
    Port &port = g_ports[static_cast<std::size_t>(index)];
    const Mapping &mapping = MAPPINGS[static_cast<std::size_t>(index)];
    if (!port.receiving || HAL_UARTEx_GetRxEventType(handle) == HAL_UART_RXEVENT_HT)
    {
        return;
    }

    bool received = false;
    if (size > 0U && size <= RECEIVE_DMA_CAPACITY)
    {
        if (data_cache_enabled())
        {
            SCB_InvalidateDCache_by_Addr(mapping.receive_buffer, RECEIVE_DMA_CAPACITY);
        }
        const std::size_t accepted = port.receive_queue.push(mapping.receive_buffer, size);
        port.statistics.received_byte_count += size;
        port.statistics.dropped_byte_count += static_cast<std::uint32_t>(size - accepted);
        received = true;
    }
    else
    {
        ++port.statistics.error_event_count;
    }

    restart_receive(port, mapping);
    if (received)
    {
        port.receive_notification.fire();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *handle)
{
    const int index = index_of(handle);
    if (index < 0)
    {
        return;
    }
    Port &port = g_ports[static_cast<std::size_t>(index)];
    if (!port.receiving)
    {
        return;
    }
    ++port.statistics.error_event_count;
    bool write_failed = false;
    if (port.transmit_active)
    {
        port.transmit_length = 0U;
        port.transmit_active = false;
        ++port.statistics.transmit_failure_count;
        write_failed = true;
    }
    restart_receive(port, MAPPINGS[static_cast<std::size_t>(index)]);
    if (write_failed)
    {
        port.write_notification.fire();
    }
}
