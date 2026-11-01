// hal::UsbCdc 在 DM-MC02 上的实现（Decision-20261101-01），由迁移期的 usb_cdc.c 改写，收发逻辑不变。
// USB OTG HS 用内部 DMA：发送数据先拷进 .usb_dma_buffer（AXI SRAM，不在 D-cache 里会出问题的 DTCM）。
// 生成区接缝（例外 X-01、X-02）：
//   - usb_device.c、usbd_cdc_if.c 的 USER CODE 区调用 UsbCdc.h 里的 UsbCdcPort_*，这里以 extern "C" 实现；
//   - 链接选项 --wrap=HAL_PCD_EP_Transmit 把端点发送转到 __wrap_HAL_PCD_EP_Transmit：
//     数据不在 DMA 可达的内存或没有 4 字节对齐时，先拷进每个端点的弹跳缓冲区。
#include "hal/usb_cdc.hpp"

#include "UsbCdc.h"
#include "board.hpp"
#include "hal/os/critical_section.hpp"
#include "port.hpp"
#include "usb_device.h" // 带进 stm32h7xx_hal.h 与 USB 设备库的头
#include "usbd_cdc.h"
#include "utils/concurrency/spsc_queue.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>

extern "C"
{
    // USB 设备句柄，定义在生成的 usb_device.c，生成区没有声明它的头
    extern USBD_HandleTypeDef hUsbDeviceHS; // NOLINT(readability-identifier-naming)：CubeMX 生成的名字
    // 链接脚本定义的 .usb_dma_buffer 段边界
    extern std::uint8_t __usb_dma_buffer_start__; // NOLINT(readability-identifier-naming)：链接脚本的符号名
    extern std::uint8_t __usb_dma_buffer_end__;   // NOLINT(readability-identifier-naming)：同上
    // --wrap 生成的原函数名
    HAL_StatusTypeDef __real_HAL_PCD_EP_Transmit( // NOLINT(readability-identifier-naming)：链接器 --wrap 约定的名字
        PCD_HandleTypeDef *hpcd,
        std::uint8_t ep_addr,
        std::uint8_t *data,
        std::uint32_t length);
}

namespace
{

constexpr std::size_t RECEIVE_QUEUE_CAPACITY{4096U};
constexpr std::uint32_t ENDPOINT_COUNT{9U};
constexpr std::uintptr_t DMA_ALIGNMENT{4U};
constexpr std::size_t TRANSMIT_CAPACITY{hal::UsbCdc::MAX_WRITE_LENGTH};

__attribute__((section(".usb_dma_buffer"), aligned(32))) std::uint8_t g_transmit_buffer[TRANSMIT_CAPACITY];
__attribute__((section(".usb_dma_buffer"),
               aligned(32))) std::uint8_t g_transmit_bounce[ENDPOINT_COUNT][TRANSMIT_CAPACITY];

struct State
{
    concurrency::SpscQueue<std::uint8_t, RECEIVE_QUEUE_CAPACITY> receive_queue{};
    hal::UsbCdc::Statistics statistics{}; // USB 中断里写；任务在临界区内读
    port::Notifier receive_notification{};
    port::Notifier write_notification{};
    std::uint32_t transmit_length{0U}; // 以下只在临界区内或 USB 中断里读写
    bool transmit_active{false};
    bool connected{false};
    bool initialized{false};
};

State g_state{};

static_assert(board::USB_CDC_COUNT == 1U, "DM-MC02 只有一个 CDC 接口（编号 0）");

bool dma_accessible(const std::uint8_t *data, std::uint32_t length) noexcept
{
    const auto start = reinterpret_cast<std::uintptr_t>(data);
    const std::uintptr_t end = start + length;
    if (data == nullptr || end < start)
    {
        return false;
    }
    // OTG HS 的内部 DMA 能访问 AXI SRAM（RAM_D1）、SRAM1–3（RAM_D2）、SRAM4（RAM_D3），不能访问 DTCM
    return (start >= 0x24000000UL && end <= 0x24050000UL) || (start >= 0x30000000UL && end <= 0x30008000UL) ||
           (start >= 0x38000000UL && end <= 0x38004000UL);
}

bool stack_configured() noexcept
{
    return hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED && hUsbDeviceHS.pClassData != nullptr;
}

void configure_pins() noexcept
{
    // CubeMX 生成的 MSP 漏掉了 PA11/PA12 的复用配置；板级 CMake 检查这一行在不在（再生成防护）
    GPIO_InitTypeDef gpio_init{};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio_init.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init.Alternate = GPIO_AF10_OTG1_FS;
    HAL_GPIO_Init(GPIOA, &gpio_init);
}

bool connected() noexcept
{
    return g_state.initialized && g_state.connected && stack_configured();
}

} // namespace

// ---- 生成区接缝（例外 X-01、X-02）----

extern "C" HAL_StatusTypeDef __wrap_HAL_PCD_EP_Transmit( // NOLINT(readability-identifier-naming)：--wrap 约定的名字
    PCD_HandleTypeDef *hpcd,
    std::uint8_t ep_addr,
    std::uint8_t *data,
    std::uint32_t length)
{
    const std::uint32_t endpoint = ep_addr & EP_ADDR_MSK;
    std::uint8_t *dma_data = data;
    if (length == 0U)
    {
        __DSB();
        return __real_HAL_PCD_EP_Transmit(hpcd, ep_addr, nullptr, 0U);
    }
    if (endpoint >= ENDPOINT_COUNT || length > TRANSMIT_CAPACITY || data == nullptr)
    {
        return HAL_ERROR;
    }
    if (!dma_accessible(data, length) || (reinterpret_cast<std::uintptr_t>(data) & (DMA_ALIGNMENT - 1U)) != 0U ||
        (length & (DMA_ALIGNMENT - 1U)) != 0U)
    {
        dma_data = g_transmit_bounce[endpoint];
        std::memcpy(dma_data, data, length);
        // 补齐到 4 字节，补的字节清零
        std::memset(&dma_data[length], 0, (DMA_ALIGNMENT - (length & (DMA_ALIGNMENT - 1U))) & (DMA_ALIGNMENT - 1U));
    }
    __DSB();
    return __real_HAL_PCD_EP_Transmit(hpcd, ep_addr, dma_data, length);
}

extern "C" bool UsbCdcPort_Init(void)
{
    configure_pins();
    const hal::os::CriticalSection guard;
    std::memset(&__usb_dma_buffer_start__,
                0,
                static_cast<std::size_t>(reinterpret_cast<std::uintptr_t>(&__usb_dma_buffer_end__) -
                                         reinterpret_cast<std::uintptr_t>(&__usb_dma_buffer_start__)));
    g_state.receive_queue.clear();
    g_state.statistics = hal::UsbCdc::Statistics{};
    g_state.transmit_length = 0U;
    g_state.transmit_active = false;
    g_state.connected = false;
    g_state.initialized = true;
    return true;
}

extern "C" bool UsbCdcPort_HandleConfigured(std::uint8_t *receive_buffer)
{
    if (!g_state.initialized || receive_buffer == nullptr)
    {
        return false;
    }
    if (USBD_CDC_SetRxBuffer(&hUsbDeviceHS, receive_buffer) != USBD_OK)
    {
        ++g_state.statistics.receive_error_count;
        return false;
    }
    // 新连接：丢掉上次连接留下的数据。clear() 本是消费者的操作，read() 在临界区里取数据，不会与这里交错
    g_state.receive_queue.clear();
    g_state.transmit_length = 0U;
    g_state.transmit_active = false;
    g_state.connected = true;
    ++g_state.statistics.connection_count;
    return true;
}

extern "C" void UsbCdcPort_HandleDisconnected(void)
{
    if (!g_state.initialized || !g_state.connected)
    {
        return;
    }
    g_state.connected = false;
    g_state.receive_queue.clear();
    ++g_state.statistics.disconnection_count;
    if (g_state.transmit_active)
    {
        g_state.transmit_length = 0U;
        g_state.transmit_active = false;
        ++g_state.statistics.transmit_failure_count;
        g_state.write_notification.fire();
    }
}

extern "C" bool UsbCdcPort_HandleReceive(std::uint8_t *data, std::uint32_t length)
{
    if (!g_state.initialized || !g_state.connected || data == nullptr)
    {
        if (g_state.initialized)
        {
            ++g_state.statistics.receive_error_count;
        }
        return false;
    }
    std::size_t accepted = 0U;
    if (length > 0U)
    {
        accepted = g_state.receive_queue.push(data, length);
        g_state.statistics.received_byte_count += length;
        g_state.statistics.dropped_byte_count += static_cast<std::uint32_t>(length - accepted);
    }
    // 交还接收缓冲区，开始收下一包
    if (USBD_CDC_SetRxBuffer(&hUsbDeviceHS, data) != USBD_OK || USBD_CDC_ReceivePacket(&hUsbDeviceHS) != USBD_OK)
    {
        ++g_state.statistics.receive_error_count;
        return false;
    }
    if (accepted > 0U)
    {
        g_state.receive_notification.fire();
    }
    return true;
}

extern "C" void UsbCdcPort_HandleTransmitComplete(void)
{
    if (!g_state.initialized || !g_state.transmit_active)
    {
        return;
    }
    g_state.statistics.transmitted_byte_count += g_state.transmit_length;
    g_state.transmit_length = 0U;
    g_state.transmit_active = false;
    g_state.write_notification.fire();
}

// ---- 契约 ----

namespace hal
{

bool UsbCdc::is_ready() const noexcept
{
    return index_ == 0U && g_state.initialized;
}

bool UsbCdc::is_connected() const noexcept
{
    return index_ == 0U && connected();
}

std::size_t UsbCdc::read(std::uint8_t *data, std::size_t capacity) const noexcept
{
    if (index_ != 0U || data == nullptr || !g_state.initialized)
    {
        return 0U;
    }
    // 连上与断开时 USB 中断会清空接收队列（clear() 本是消费者的操作）：取数据时屏蔽框架中断，
    // 两者不会交错。一次最多拷 capacity 字节，时间很短。
    const os::CriticalSection guard;
    return g_state.receive_queue.pop(data, capacity);
}

UsbCdc::WriteResult UsbCdc::write(const std::uint8_t *data, std::size_t length) const noexcept
{
    if (index_ != 0U || data == nullptr || length == 0U || length > MAX_WRITE_LENGTH)
    {
        return WriteResult::INVALID_ARGUMENT;
    }
    if (!connected())
    {
        return WriteResult::NOT_READY;
    }

    const os::CriticalSection guard; // USB 中断（框架线上）在这期间进不来
    if (!g_state.connected || !stack_configured())
    {
        return WriteResult::NOT_READY;
    }
    const auto *const class_state = static_cast<const USBD_CDC_HandleTypeDef *>(hUsbDeviceHS.pClassData);
    if (g_state.transmit_active || class_state == nullptr || class_state->TxState != 0U)
    {
        return WriteResult::BUSY;
    }
    std::memcpy(g_transmit_buffer, data, length);
    g_state.transmit_length = static_cast<std::uint32_t>(length);
    g_state.transmit_active = true;

    auto status = static_cast<USBD_StatusTypeDef>(
        USBD_CDC_SetTxBuffer(&hUsbDeviceHS, g_transmit_buffer, static_cast<std::uint32_t>(length)));
    if (status == USBD_OK)
    {
        status = static_cast<USBD_StatusTypeDef>(USBD_CDC_TransmitPacket(&hUsbDeviceHS));
    }
    if (status != USBD_OK)
    {
        g_state.transmit_length = 0U;
        g_state.transmit_active = false;
        if (status == USBD_FAIL)
        {
            ++g_state.statistics.transmit_failure_count;
        }
        return status == USBD_BUSY ? WriteResult::BUSY : WriteResult::ERROR;
    }
    return WriteResult::STARTED;
}

bool UsbCdc::set_receive_notification(Callback callback, void *context) const noexcept
{
    if (index_ != 0U || !g_state.initialized)
    {
        return false;
    }
    g_state.receive_notification.set(callback, context);
    return true;
}

bool UsbCdc::set_write_notification(Callback callback, void *context) const noexcept
{
    if (index_ != 0U || !g_state.initialized)
    {
        return false;
    }
    g_state.write_notification.set(callback, context);
    return true;
}

UsbCdc::Statistics UsbCdc::statistics() const noexcept
{
    if (index_ != 0U)
    {
        return Statistics{};
    }
    const os::CriticalSection guard;
    return g_state.statistics;
}

} // namespace hal
