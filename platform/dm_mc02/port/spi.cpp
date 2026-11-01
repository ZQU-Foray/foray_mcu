// hal::Spi 在 DM-MC02 上的实现（Decision-20261101-01），由迁移期的 spi.c 改写，DMA 与出错恢复的逻辑不变。
// 设备表的下标就是 board.hpp 里句柄的编号；同一条总线上的设备共用一份总线状态（busy 标志与异步事务）。
// 覆盖 HAL 弱符号 HAL_SPI_TxCpltCallback、HAL_SPI_TxRxCpltCallback、HAL_SPI_ErrorCallback：spi.h 带进 HAL 头，
// 定义因此是 C 链接（规则 16）。
//
// 异步事务（DMA）：发送数据先拷进 DMA 缓冲区（D-cache 按缓存行清理），调用方的缓冲区返回前就能复用；
// 全双工要等 SPI 传输结束（TxRxCplt）与 TX DMA 结束两件事都发生才算完成，再把接收缓冲区作废缓存后拷给调用方。
#include "hal/spi.hpp"

#include "board.hpp"
#include "hal/os/critical_section.hpp"
#include "main.h" // 片选引脚名
#include "port.hpp"
#include "spi.h" // hspi2、hspi6；带进 stm32h7xx_hal.h

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace
{

constexpr std::uint32_t CACHE_LINE_SIZE{32U};
constexpr std::size_t ASYNC_CAPACITY{hal::Spi::MAX_ASYNC_LENGTH};
constexpr std::size_t BLOCKING_MAX_LENGTH{0xFFFFU}; // HAL 的长度参数是 16 位
static_assert(ASYNC_CAPACITY % CACHE_LINE_SIZE == 0U, "DMA 缓冲区按缓存行对齐");

// DMA1/DMA2 能访问 AXI SRAM（.dma_buffer 在 RAM_D1），BDMA 只能访问 SRAM4（.bdma_buffer 在 RAM_D3）
__attribute__((section(".dma_buffer"), aligned(CACHE_LINE_SIZE))) std::uint8_t g_spi2_transmit_buffer[ASYNC_CAPACITY];
__attribute__((section(".dma_buffer"), aligned(CACHE_LINE_SIZE))) std::uint8_t g_spi2_receive_buffer[ASYNC_CAPACITY];
__attribute__((section(".bdma_buffer"), aligned(CACHE_LINE_SIZE))) std::uint8_t g_spi6_transmit_buffer[ASYNC_CAPACITY];

struct Bus
{
    SPI_HandleTypeDef *handle;
    std::uint8_t *transmit_buffer;
    std::uint8_t *receive_buffer; // 为空：这条总线没有接收 DMA，只能发送
    // 以下只在临界区内或框架中断里读写
    bool busy;
    bool active; // 有异步事务
    bool spi_completed;
    bool transmit_dma_completed;
    bool transfer;
    std::uint8_t device;
    std::uint8_t *receive_data;
    std::uint32_t length;
    hal::Callback callback;
    void *context;
};

struct Device
{
    std::size_t bus;
    GPIO_TypeDef *chip_select_port; // 为空：没有片选
    std::uint16_t chip_select_pin;
    bool supports_transfer;
};

std::array<Bus, 2U> g_buses{{
    {&hspi2,
     g_spi2_transmit_buffer,
     g_spi2_receive_buffer,
     false,
     false,
     false,
     false,
     false,
     0U,
     nullptr,
     0U,
     nullptr,
     nullptr},
    {&hspi6, g_spi6_transmit_buffer, nullptr, false, false, false, false, false, 0U, nullptr, 0U, nullptr, nullptr},
}};

// 顺序与 board.hpp 一致
const std::array<Device, board::SPI_COUNT> DEVICES{{
    {0U, BMI088_ACCEL_CS_GPIO_Port, BMI088_ACCEL_CS_Pin, true}, // SPI_IMU_ACCEL
    {0U, BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, true},   // SPI_IMU_GYRO
    {1U, nullptr, 0U, false},                                   // SPI_LED：只接了 MOSI
}};

std::array<std::atomic<hal::Spi::Result>, board::SPI_COUNT> g_results{};
bool g_initialized{false};

using Result = hal::Spi::Result;

Bus *bus_of(const SPI_HandleTypeDef *handle) noexcept
{
    for (Bus &bus : g_buses)
    {
        if (bus.handle == handle)
        {
            return &bus;
        }
    }
    return nullptr;
}

void select(const Device &device) noexcept
{
    if (device.chip_select_port != nullptr)
    {
        HAL_GPIO_WritePin(device.chip_select_port, device.chip_select_pin, GPIO_PIN_RESET);
    }
}

void deselect(const Device &device) noexcept
{
    if (device.chip_select_port != nullptr)
    {
        HAL_GPIO_WritePin(device.chip_select_port, device.chip_select_pin, GPIO_PIN_SET);
    }
}

Result map_status(HAL_StatusTypeDef status) noexcept
{
    switch (status)
    {
    case HAL_OK:
        return Result::COMPLETED;
    case HAL_BUSY:
        return Result::BUSY;
    case HAL_TIMEOUT:
        return Result::TIMEOUT;
    default:
        return Result::ERROR;
    }
}

bool dma_ready(const DMA_HandleTypeDef *handle) noexcept
{
    return handle != nullptr && HAL_DMA_GetState(const_cast<DMA_HandleTypeDef *>(handle)) == HAL_DMA_STATE_READY &&
           handle->Init.Mode == DMA_NORMAL && handle->Init.MemDataAlignment == DMA_MDATAALIGN_BYTE &&
           handle->Init.PeriphDataAlignment == DMA_PDATAALIGN_BYTE;
}

bool dma_ready(const Bus &bus) noexcept
{
    return dma_ready(bus.handle->hdmatx) && (bus.receive_buffer == nullptr || dma_ready(bus.handle->hdmarx));
}

// ---- D-cache 维护：按缓存行处理整行，缓冲区本身按缓存行对齐 ----

bool data_cache_enabled() noexcept
{
    return (SCB->CCR & SCB_CCR_DC_Msk) != 0U;
}

std::int32_t cache_length(std::uint32_t length) noexcept
{
    return static_cast<std::int32_t>((length + CACHE_LINE_SIZE - 1U) & ~(CACHE_LINE_SIZE - 1U));
}

void clean_for_dma(std::uint8_t *buffer, std::uint32_t length) noexcept
{
    if (data_cache_enabled())
    {
        SCB_CleanDCache_by_Addr(buffer, cache_length(length));
    }
}

void prepare_receive(std::uint8_t *buffer, std::uint32_t length) noexcept
{
    if (data_cache_enabled())
    {
        SCB_CleanInvalidateDCache_by_Addr(buffer, cache_length(length));
    }
}

void complete_receive(std::uint8_t *buffer, std::uint32_t length) noexcept
{
    if (data_cache_enabled())
    {
        SCB_InvalidateDCache_by_Addr(buffer, cache_length(length));
    }
}

// ---- 总线占用 ----

void release(Bus &bus) noexcept
{
    const hal::os::CriticalSection guard;
    bus.busy = false;
}

// 占住设备所在的总线；成功返回 COMPLETED
Result acquire(std::uint8_t index) noexcept
{
    if (index >= DEVICES.size())
    {
        return Result::INVALID_ARGUMENT;
    }
    if (!g_initialized)
    {
        return Result::NOT_READY;
    }
    Bus &bus = g_buses[DEVICES[index].bus];
    {
        const hal::os::CriticalSection guard;
        if (bus.busy)
        {
            return Result::BUSY;
        }
        bus.busy = true;
    }
    const HAL_SPI_StateTypeDef state = HAL_SPI_GetState(bus.handle);
    if (state != HAL_SPI_STATE_READY || !dma_ready(bus))
    {
        release(bus);
        const bool in_progress = !dma_ready(bus) || state == HAL_SPI_STATE_BUSY || state == HAL_SPI_STATE_BUSY_TX ||
                                 state == HAL_SPI_STATE_BUSY_RX || state == HAL_SPI_STATE_BUSY_TX_RX ||
                                 state == HAL_SPI_STATE_ABORT;
        return in_progress ? Result::BUSY : Result::NOT_READY;
    }
    return Result::COMPLETED;
}

// ---- 异步事务的结束（框架中断里） ----

void reset_async(Bus &bus) noexcept
{
    bus.active = false;
    bus.spi_completed = false;
    bus.transmit_dma_completed = false;
    bus.transfer = false;
    bus.receive_data = nullptr;
    bus.length = 0U;
    bus.callback = nullptr;
    bus.context = nullptr;
}

void finalize(Bus &bus, Result result) noexcept
{
    std::uint8_t device = 0U;
    hal::Callback callback = nullptr;
    void *context = nullptr;
    std::uint8_t *receive_data = nullptr;
    std::uint32_t length = 0U;
    bool transfer = false;
    {
        const hal::os::CriticalSection guard;
        if (!bus.active)
        {
            return;
        }
        device = bus.device;
        callback = bus.callback;
        context = bus.context;
        receive_data = bus.receive_data;
        length = bus.length;
        transfer = bus.transfer;
        reset_async(bus);
    }

    deselect(DEVICES[device]);
    if (result == Result::COMPLETED && transfer && receive_data != nullptr)
    {
        complete_receive(bus.receive_buffer, length);
        std::memcpy(receive_data, bus.receive_buffer, length);
    }
    g_results[device].store(result, std::memory_order_release);
    release(bus);
    if (callback != nullptr)
    {
        callback(context);
    }
}

void try_finalize_transfer(Bus &bus) noexcept
{
    if (bus.active && bus.transfer && bus.spi_completed && bus.transmit_dma_completed)
    {
        finalize(bus, Result::COMPLETED);
    }
}

// 全双工时 HAL 不关心 TX DMA 何时结束，这里接管它的完成回调
void on_transmit_dma_complete(DMA_HandleTypeDef *dma) noexcept
{
    if (dma == nullptr)
    {
        return;
    }
    Bus *const bus = bus_of(static_cast<SPI_HandleTypeDef *>(dma->Parent));
    if (bus == nullptr || !bus->active || !bus->transfer)
    {
        return;
    }
    bus->transmit_dma_completed = true;
    try_finalize_transfer(*bus);
}

bool dma_settled(DMA_HandleTypeDef *handle) noexcept
{
    if (handle == nullptr)
    {
        return true;
    }
    const HAL_DMA_StateTypeDef state = HAL_DMA_GetState(handle);
    return state != HAL_DMA_STATE_BUSY && state != HAL_DMA_STATE_ABORT;
}

void complete_error_if_settled(Bus *bus) noexcept
{
    if (bus != nullptr && bus->active && HAL_SPI_GetState(bus->handle) == HAL_SPI_STATE_READY &&
        dma_settled(bus->handle->hdmatx) && dma_settled(bus->handle->hdmarx))
    {
        finalize(*bus, Result::ERROR);
    }
}

void on_dma_aborted_after_error(DMA_HandleTypeDef *dma) noexcept
{
    if (dma == nullptr)
    {
        return;
    }
    dma->XferAbortCallback = nullptr;
    complete_error_if_settled(bus_of(static_cast<SPI_HandleTypeDef *>(dma->Parent)));
}

void abort_busy_dma(DMA_HandleTypeDef *dma) noexcept
{
    if (dma == nullptr || HAL_DMA_GetState(dma) != HAL_DMA_STATE_BUSY)
    {
        return;
    }
    dma->XferAbortCallback = on_dma_aborted_after_error;
    if (HAL_DMA_Abort_IT(dma) != HAL_OK)
    {
        dma->XferAbortCallback = nullptr;
        if (HAL_DMA_GetState(dma) == HAL_DMA_STATE_BUSY)
        {
            (void)HAL_DMA_Abort(dma);
        }
    }
}

void recover_after_error(Bus *bus) noexcept
{
    if (bus == nullptr || !bus->active)
    {
        return;
    }
    abort_busy_dma(bus->handle->hdmatx);
    if (bus->active)
    {
        abort_busy_dma(bus->handle->hdmarx);
    }
    complete_error_if_settled(bus);
}

// ---- 事务的开始（任务里） ----

Result start_async(std::uint8_t index,
                   const std::uint8_t *transmit_data,
                   std::uint8_t *receive_data,
                   std::uint32_t length,
                   bool transfer,
                   hal::Callback callback,
                   void *context) noexcept
{
    Result result = acquire(index);
    if (result != Result::COMPLETED)
    {
        return result;
    }
    const Device &device = DEVICES[index];
    Bus &bus = g_buses[device.bus];
    if (transfer && !device.supports_transfer)
    {
        release(bus);
        return Result::UNSUPPORTED;
    }

    std::memcpy(bus.transmit_buffer, transmit_data, length);
    clean_for_dma(bus.transmit_buffer, length);
    if (transfer)
    {
        prepare_receive(bus.receive_buffer, length);
    }

    HAL_StatusTypeDef status = HAL_OK;
    {
        // DMA 完成中断在临界区结束之前进不来，所以先把事务状态写好再启动 DMA
        const hal::os::CriticalSection guard;
        bus.spi_completed = false;
        bus.transmit_dma_completed = false;
        bus.transfer = transfer;
        bus.device = index;
        bus.receive_data = receive_data;
        bus.length = length;
        bus.callback = callback;
        bus.context = context;
        g_results[index].store(Result::BUSY, std::memory_order_relaxed);
        bus.active = true;

        select(device);
        const auto dma_length = static_cast<std::uint16_t>(length);
        if (transfer)
        {
            status = HAL_SPI_TransmitReceive_DMA(bus.handle, bus.transmit_buffer, bus.receive_buffer, dma_length);
            if (status == HAL_OK)
            {
                bus.handle->hdmatx->XferCpltCallback = on_transmit_dma_complete;
            }
        }
        else
        {
            status = HAL_SPI_Transmit_DMA(bus.handle, bus.transmit_buffer, dma_length);
        }
        if (status != HAL_OK)
        {
            result = map_status(status);
            reset_async(bus);
            g_results[index].store(result, std::memory_order_relaxed);
        }
    }

    if (status == HAL_OK)
    {
        return Result::STARTED;
    }
    deselect(device);
    release(bus);
    return result;
}

Result blocking(std::uint8_t index,
                const std::uint8_t *transmit_data,
                std::uint8_t *receive_data,
                std::size_t length,
                hal::os::Microseconds timeout,
                bool transfer) noexcept
{
    if (transmit_data == nullptr || (transfer && receive_data == nullptr) || length == 0U ||
        length > BLOCKING_MAX_LENGTH)
    {
        return Result::INVALID_ARGUMENT;
    }
    const Result acquired = acquire(index);
    if (acquired != Result::COMPLETED)
    {
        return acquired;
    }
    const Device &device = DEVICES[index];
    Bus &bus = g_buses[device.bus];
    if (transfer && !device.supports_transfer)
    {
        release(bus);
        return Result::UNSUPPORTED;
    }

    // HAL 的超时以 ms 计：向上取整
    const std::uint32_t timeout_ms = timeout / hal::os::MICROSECONDS_PER_MILLISECOND +
                                     (timeout % hal::os::MICROSECONDS_PER_MILLISECOND != 0U ? 1U : 0U);
    auto *const data = const_cast<std::uint8_t *>(transmit_data); // HAL 的参数没有 const，实际只读
    const auto hal_length = static_cast<std::uint16_t>(length);
    select(device);
    const HAL_StatusTypeDef status =
        transfer ? HAL_SPI_TransmitReceive(bus.handle, data, receive_data, hal_length, timeout_ms)
                 : HAL_SPI_Transmit(bus.handle, data, hal_length, timeout_ms);
    deselect(device);
    release(bus);
    return map_status(status);
}

Result check_async_arguments(const std::uint8_t *transmit_data,
                             const std::uint8_t *receive_data,
                             std::size_t length,
                             bool transfer) noexcept
{
    if (transmit_data == nullptr || (transfer && receive_data == nullptr) || length == 0U || length > ASYNC_CAPACITY)
    {
        return Result::INVALID_ARGUMENT;
    }
    return Result::COMPLETED;
}

} // namespace

bool port::init_spi() noexcept
{
    g_initialized = false;
    for (Bus &bus : g_buses)
    {
        bus.busy = false;
        reset_async(bus);
        if (!dma_ready(bus))
        {
            return false;
        }
    }
    for (std::size_t index = 0U; index < DEVICES.size(); ++index)
    {
        g_results[index].store(Result::NOT_READY, std::memory_order_relaxed);
        if (HAL_SPI_GetState(g_buses[DEVICES[index].bus].handle) != HAL_SPI_STATE_READY)
        {
            return false;
        }
        deselect(DEVICES[index]);
    }
    g_initialized = true;
    return true;
}

namespace hal
{

bool Spi::is_ready() const noexcept
{
    if (!g_initialized || index_ >= DEVICES.size())
    {
        return false;
    }
    const Bus &bus = g_buses[DEVICES[index_].bus];
    return !bus.busy && HAL_SPI_GetState(bus.handle) == HAL_SPI_STATE_READY && dma_ready(bus);
}

Spi::Result Spi::transmit(const std::uint8_t *data, std::size_t length, os::Microseconds timeout) const noexcept
{
    return blocking(index_, data, nullptr, length, timeout, false);
}

Spi::Result Spi::transfer(const std::uint8_t *transmit_data,
                          std::uint8_t *receive_data,
                          std::size_t length,
                          os::Microseconds timeout) const noexcept
{
    return blocking(index_, transmit_data, receive_data, length, timeout, true);
}

Spi::Result
Spi::start_transmit(const std::uint8_t *data, std::size_t length, Callback callback, void *context) const noexcept
{
    const Result checked = check_async_arguments(data, nullptr, length, false);
    return checked != Result::COMPLETED
               ? checked
               : start_async(index_, data, nullptr, static_cast<std::uint32_t>(length), false, callback, context);
}

Spi::Result Spi::start_transfer(const std::uint8_t *transmit_data,
                                std::uint8_t *receive_data,
                                std::size_t length,
                                Callback callback,
                                void *context) const noexcept
{
    const Result checked = check_async_arguments(transmit_data, receive_data, length, true);
    return checked != Result::COMPLETED
               ? checked
               : start_async(
                     index_, transmit_data, receive_data, static_cast<std::uint32_t>(length), true, callback, context);
}

Spi::Result Spi::async_result() const noexcept
{
    if (index_ >= DEVICES.size())
    {
        return Result::INVALID_ARGUMENT;
    }
    if (!g_initialized)
    {
        return Result::NOT_READY;
    }
    return g_results[index_].load(std::memory_order_acquire);
}

} // namespace hal

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *handle)
{
    Bus *const bus = bus_of(handle);
    if (bus == nullptr || !bus->active)
    {
        return;
    }
    // 只发送的事务在这里结束；全双工不该走到这里
    finalize(*bus, bus->transfer ? Result::ERROR : Result::COMPLETED);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *handle)
{
    Bus *const bus = bus_of(handle);
    if (bus == nullptr || !bus->active)
    {
        return;
    }
    if (!bus->transfer)
    {
        finalize(*bus, Result::ERROR);
        return;
    }
    bus->spi_completed = true;
    try_finalize_transfer(*bus);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *handle)
{
    recover_after_error(bus_of(handle));
}
