#ifndef HAL_SPI_HPP
#define HAL_SPI_HPP

#include "hal/detail/spi.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace platform
{

class Spi final
{
  public:
    static constexpr std::size_t MAX_ASYNC_LENGTH = SPI_PORT_MAX_ASYNC_LENGTH;

    enum class Device : std::uint8_t
    {
        IMU_ACCELEROMETER = SPI_PORT_DEVICE_IMU_ACCELEROMETER,
        IMU_GYROSCOPE = SPI_PORT_DEVICE_IMU_GYROSCOPE,
        ADDRESSABLE_LED = SPI_PORT_DEVICE_ADDRESSABLE_LED
    };

    enum class Result : std::uint8_t
    {
        COMPLETED = SPI_PORT_RESULT_COMPLETED,
        BUSY = SPI_PORT_RESULT_BUSY,
        TIMEOUT = SPI_PORT_RESULT_TIMEOUT,
        NOT_READY = SPI_PORT_RESULT_NOT_READY,
        UNSUPPORTED = SPI_PORT_RESULT_UNSUPPORTED,
        INVALID_ARGUMENT = SPI_PORT_RESULT_INVALID_ARGUMENT,
        ERROR = SPI_PORT_RESULT_ERROR,
        STARTED = SPI_PORT_RESULT_STARTED
    };

    using CompletionNotification = SpiPort_CompletionNotification;

    /**
   * @brief 禁止创建 Spi 实例，所有能力均通过静态方法访问。
   */
    Spi() = delete;

    /**
   * @brief 查询指定 SPI 逻辑设备是否可以启动新事务。
   * @param device 要查询的逻辑设备。
   * @return 设备有效、底层外设就绪且共享总线空闲时返回 true。
   */
    [[nodiscard]] static bool is_ready(Device device) noexcept
    {
        return SpiPort_IsReady(to_port_device(device));
    }

    /**
   * @brief 以阻塞方式向指定 SPI 逻辑设备发送一段连续数据。
   * @param device 目标逻辑设备。
   * @param data 指向待发送数据的缓冲区。
   * @param length 待发送字节数。
   * @param timeout_ms 允许阻塞等待的最长毫秒数。
   * @return 返回完成、忙、超时、未就绪、不支持或参数错误等结果。
   * @note 仅在线程或初始化上下文调用，事务期间由 Port 管理片选和共享总线。
   */
    [[nodiscard]] static Result
    transmit(Device device, const std::uint8_t *data, std::size_t length, std::uint32_t timeout_ms) noexcept
    {
        if (length > std::numeric_limits<std::uint32_t>::max())
        {
            return Result::INVALID_ARGUMENT;
        }

        return to_result(
            SpiPort_Transmit(to_port_device(device), data, static_cast<std::uint32_t>(length), timeout_ms));
    }

    /**
   * @brief 以阻塞方式向指定 SPI 逻辑设备发送整个定长数组。
   * @tparam Size 数组字节数。
   * @param device 目标逻辑设备。
   * @param data 待发送的定长数组。
   * @param timeout_ms 允许阻塞等待的最长毫秒数。
   * @return 返回完成、忙、超时、未就绪、不支持或参数错误等结果。
   */
    template <std::size_t Size>
    [[nodiscard]] static Result
    transmit(Device device, const std::array<std::uint8_t, Size> &data, std::uint32_t timeout_ms) noexcept
    {
        return transmit(device, data.data(), data.size(), timeout_ms);
    }

    /**
   * @brief 以阻塞方式与指定 SPI 逻辑设备进行全双工传输。
   * @param device 目标逻辑设备。
   * @param transmit_data 指向待发送数据的缓冲区。
   * @param receive_data 用于接收数据的缓冲区。
   * @param length 同时发送和接收的字节数。
   * @param timeout_ms 允许阻塞等待的最长毫秒数。
   * @return 返回完成、忙、超时、未就绪、不支持或参数错误等结果。
   * @note 仅支持具有全双工能力的逻辑设备，且仅在线程或初始化上下文调用。
   */
    [[nodiscard]] static Result transfer(Device device,
                                         const std::uint8_t *transmit_data,
                                         std::uint8_t *receive_data,
                                         std::size_t length,
                                         std::uint32_t timeout_ms) noexcept
    {
        if (length > std::numeric_limits<std::uint32_t>::max())
        {
            return Result::INVALID_ARGUMENT;
        }

        return to_result(SpiPort_Transfer(
            to_port_device(device), transmit_data, receive_data, static_cast<std::uint32_t>(length), timeout_ms));
    }

    /**
   * @brief 以阻塞方式使用两个等长数组完成 SPI 全双工传输。
   * @tparam Size 发送和接收数组的字节数。
   * @param device 目标逻辑设备。
   * @param transmit_data 待发送的定长数组。
   * @param receive_data 用于接收数据的定长数组。
   * @param timeout_ms 允许阻塞等待的最长毫秒数。
   * @return 返回完成、忙、超时、未就绪、不支持或参数错误等结果。
   */
    template <std::size_t Size>
    [[nodiscard]] static Result transfer(Device device,
                                         const std::array<std::uint8_t, Size> &transmit_data,
                                         std::array<std::uint8_t, Size> &receive_data,
                                         std::uint32_t timeout_ms) noexcept
    {
        return transfer(device, transmit_data.data(), receive_data.data(), transmit_data.size(), timeout_ms);
    }

    /**
   * @brief 启动一次基于 DMA 的非阻塞 SPI 发送事务。
   * @note MAY 能力：Port 可能返回 Result::Unsupported，业务须回退阻塞
   * Transmit。
   * @param device 目标逻辑设备。
   * @param data 指向待发送数据的缓冲区，函数返回后调用者可立即复用该缓冲区。
   * @param length 待发送字节数，不得超过 MAX_ASYNC_LENGTH。
   * @param notification 事务终止时调用的通知函数，允许传入 nullptr。
   * @param context 调用通知函数时原样传回的用户上下文。
   * @return 返回已启动、忙、未就绪、不支持或参数错误等结果。
   * @note 通知在中断上下文执行，最终结果应通过 get_async_result 查询。
   */
    [[nodiscard]] static Result start_transmit_async(Device device,
                                                     const std::uint8_t *data,
                                                     std::size_t length,
                                                     CompletionNotification notification,
                                                     void *context) noexcept
    {
        if (length > std::numeric_limits<std::uint32_t>::max())
        {
            return Result::INVALID_ARGUMENT;
        }

        return to_result(SpiPort_StartTransmitAsync(
            to_port_device(device), data, static_cast<std::uint32_t>(length), notification, context));
    }

    /**
   * @brief 使用整个定长数组启动一次非阻塞 SPI 发送事务。
   * @tparam Size 数组字节数。
   * @param device 目标逻辑设备。
   * @param data 待发送的定长数组。
   * @param notification 事务终止时调用的通知函数，允许传入 nullptr。
   * @param context 调用通知函数时原样传回的用户上下文。
   * @return 返回已启动、忙、未就绪、不支持或参数错误等结果。
   */
    template <std::size_t Size>
    [[nodiscard]] static Result start_transmit_async(Device device,
                                                     const std::array<std::uint8_t, Size> &data,
                                                     CompletionNotification notification,
                                                     void *context) noexcept
    {
        return start_transmit_async(device, data.data(), data.size(), notification, context);
    }

    /**
   * @brief 启动一次基于 DMA 的非阻塞 SPI 全双工事务。
   * @note MAY 能力：Port 可能返回 Result::Unsupported，业务须回退阻塞
   * Transfer。
   * @param device 目标逻辑设备。
   * @param transmit_data
   * 指向待发送数据的缓冲区，函数返回后调用者可立即复用该缓冲区。
   * @param receive_data 用于接收数据的缓冲区，必须保持有效直至事务终止。
   * @param length 同时发送和接收的字节数，不得超过 MAX_ASYNC_LENGTH。
   * @param notification 事务终止时调用的通知函数，允许传入 nullptr。
   * @param context 调用通知函数时原样传回的用户上下文。
   * @return 返回已启动、忙、未就绪、不支持或参数错误等结果。
   * @note 通知在中断上下文执行，最终结果应通过 get_async_result 查询。
   */
    [[nodiscard]] static Result start_transfer_async(Device device,
                                                     const std::uint8_t *transmit_data,
                                                     std::uint8_t *receive_data,
                                                     std::size_t length,
                                                     CompletionNotification notification,
                                                     void *context) noexcept
    {
        if (length > std::numeric_limits<std::uint32_t>::max())
        {
            return Result::INVALID_ARGUMENT;
        }

        return to_result(SpiPort_StartTransferAsync(to_port_device(device),
                                                    transmit_data,
                                                    receive_data,
                                                    static_cast<std::uint32_t>(length),
                                                    notification,
                                                    context));
    }

    /**
   * @brief 使用两个等长数组启动一次非阻塞 SPI 全双工事务。
   * @tparam Size 发送和接收数组的字节数。
   * @param device 目标逻辑设备。
   * @param transmit_data 待发送的定长数组。
   * @param receive_data 用于接收数据的定长数组，必须保持有效直至事务终止。
   * @param notification 事务终止时调用的通知函数，允许传入 nullptr。
   * @param context 调用通知函数时原样传回的用户上下文。
   * @return 返回已启动、忙、未就绪、不支持或参数错误等结果。
   */
    template <std::size_t Size>
    [[nodiscard]] static Result start_transfer_async(Device device,
                                                     const std::array<std::uint8_t, Size> &transmit_data,
                                                     std::array<std::uint8_t, Size> &receive_data,
                                                     CompletionNotification notification,
                                                     void *context) noexcept
    {
        return start_transfer_async(
            device, transmit_data.data(), receive_data.data(), transmit_data.size(), notification, context);
    }

    /**
   * @brief 查询指定 SPI 逻辑设备最近一次异步事务的当前或最终结果。
   * @param device 要查询的逻辑设备。
   * @return
   * 事务进行中时返回忙，终止后返回完成或错误，无有效设备时返回参数错误。
   * @note MAY 能力：异步未实现时返回 Result::Unsupported。
   */
    [[nodiscard]] static Result get_async_result(Device device) noexcept
    {
        return to_result(SpiPort_GetAsyncResult(to_port_device(device)));
    }

  private:
    /**
   * @brief 将 C++ 逻辑设备转换为 Detail C ABI 使用的设备值。
   * @param device C++ 逻辑设备。
   * @return 对应的 C ABI 设备值。
   */
    [[nodiscard]] static constexpr SpiPort_Device to_port_device(Device device) noexcept
    {
        return static_cast<SpiPort_Device>(device);
    }

    /**
   * @brief 将 Detail C ABI 的 SPI 结果转换为 C++ 结果枚举。
   * @param result C ABI SPI 结果。
   * @return 对应的 C++ 结果，未知值统一转换为 Result::Error。
   */
    [[nodiscard]] static constexpr Result to_result(SpiPort_Result result) noexcept
    {
        return result <= SPI_PORT_RESULT_STARTED ? static_cast<Result>(result) : Result::ERROR;
    }
};

} // namespace platform

#endif
