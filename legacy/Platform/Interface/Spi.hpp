#ifndef PLATFORM_INTERFACE_SPI_HPP
#define PLATFORM_INTERFACE_SPI_HPP

#include "Detail/Spi.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace platform
{

class Spi final
{
public:
  static constexpr std::size_t MaxAsyncLength = SPI_PORT_MAX_ASYNC_LENGTH;

  enum class Device : std::uint8_t
  {
    ImuAccelerometer = SPI_PORT_DEVICE_IMU_ACCELEROMETER,
    ImuGyroscope = SPI_PORT_DEVICE_IMU_GYROSCOPE,
    AddressableLed = SPI_PORT_DEVICE_ADDRESSABLE_LED
  };

  enum class Result : std::uint8_t
  {
    Completed = SPI_PORT_RESULT_COMPLETED,
    Busy = SPI_PORT_RESULT_BUSY,
    Timeout = SPI_PORT_RESULT_TIMEOUT,
    NotReady = SPI_PORT_RESULT_NOT_READY,
    Unsupported = SPI_PORT_RESULT_UNSUPPORTED,
    InvalidArgument = SPI_PORT_RESULT_INVALID_ARGUMENT,
    Error = SPI_PORT_RESULT_ERROR,
    Started = SPI_PORT_RESULT_STARTED
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
  [[nodiscard]] static bool IsReady(Device device) noexcept
  {
    return SpiPort_IsReady(ToPortDevice(device));
  }

  /**
   * @brief 以阻塞方式向指定 SPI 逻辑设备发送一段连续数据。
   * @param device 目标逻辑设备。
   * @param data 指向待发送数据的缓冲区。
   * @param length 待发送字节数。
   * @param timeoutMs 允许阻塞等待的最长毫秒数。
   * @return 返回完成、忙、超时、未就绪、不支持或参数错误等结果。
   * @note 仅在线程或初始化上下文调用，事务期间由 Port 管理片选和共享总线。
   */
  [[nodiscard]] static Result
  Transmit(Device device, const std::uint8_t *data, std::size_t length, std::uint32_t timeoutMs) noexcept
  {
    if (length > std::numeric_limits<std::uint32_t>::max())
    {
      return Result::InvalidArgument;
    }

    return ToResult(SpiPort_Transmit(ToPortDevice(device), data, static_cast<std::uint32_t>(length), timeoutMs));
  }

  /**
   * @brief 以阻塞方式向指定 SPI 逻辑设备发送整个定长数组。
   * @tparam Size 数组字节数。
   * @param device 目标逻辑设备。
   * @param data 待发送的定长数组。
   * @param timeoutMs 允许阻塞等待的最长毫秒数。
   * @return 返回完成、忙、超时、未就绪、不支持或参数错误等结果。
   */
  template <std::size_t Size>
  [[nodiscard]] static Result
  Transmit(Device device, const std::array<std::uint8_t, Size> &data, std::uint32_t timeoutMs) noexcept
  {
    return Transmit(device, data.data(), data.size(), timeoutMs);
  }

  /**
   * @brief 以阻塞方式与指定 SPI 逻辑设备进行全双工传输。
   * @param device 目标逻辑设备。
   * @param transmitData 指向待发送数据的缓冲区。
   * @param receiveData 用于接收数据的缓冲区。
   * @param length 同时发送和接收的字节数。
   * @param timeoutMs 允许阻塞等待的最长毫秒数。
   * @return 返回完成、忙、超时、未就绪、不支持或参数错误等结果。
   * @note 仅支持具有全双工能力的逻辑设备，且仅在线程或初始化上下文调用。
   */
  [[nodiscard]] static Result Transfer(Device device,
                                       const std::uint8_t *transmitData,
                                       std::uint8_t *receiveData,
                                       std::size_t length,
                                       std::uint32_t timeoutMs) noexcept
  {
    if (length > std::numeric_limits<std::uint32_t>::max())
    {
      return Result::InvalidArgument;
    }

    return ToResult(SpiPort_Transfer(
        ToPortDevice(device), transmitData, receiveData, static_cast<std::uint32_t>(length), timeoutMs));
  }

  /**
   * @brief 以阻塞方式使用两个等长数组完成 SPI 全双工传输。
   * @tparam Size 发送和接收数组的字节数。
   * @param device 目标逻辑设备。
   * @param transmitData 待发送的定长数组。
   * @param receiveData 用于接收数据的定长数组。
   * @param timeoutMs 允许阻塞等待的最长毫秒数。
   * @return 返回完成、忙、超时、未就绪、不支持或参数错误等结果。
   */
  template <std::size_t Size>
  [[nodiscard]] static Result Transfer(Device device,
                                       const std::array<std::uint8_t, Size> &transmitData,
                                       std::array<std::uint8_t, Size> &receiveData,
                                       std::uint32_t timeoutMs) noexcept
  {
    return Transfer(device, transmitData.data(), receiveData.data(), transmitData.size(), timeoutMs);
  }

  /**
   * @brief 启动一次基于 DMA 的非阻塞 SPI 发送事务。
   * @note MAY 能力：Port 可能返回 Result::Unsupported，业务须回退阻塞
   * Transmit。
   * @param device 目标逻辑设备。
   * @param data 指向待发送数据的缓冲区，函数返回后调用者可立即复用该缓冲区。
   * @param length 待发送字节数，不得超过 MaxAsyncLength。
   * @param notification 事务终止时调用的通知函数，允许传入 nullptr。
   * @param context 调用通知函数时原样传回的用户上下文。
   * @return 返回已启动、忙、未就绪、不支持或参数错误等结果。
   * @note 通知在中断上下文执行，最终结果应通过 GetAsyncResult 查询。
   */
  [[nodiscard]] static Result StartTransmitAsync(Device device,
                                                 const std::uint8_t *data,
                                                 std::size_t length,
                                                 CompletionNotification notification,
                                                 void *context) noexcept
  {
    if (length > std::numeric_limits<std::uint32_t>::max())
    {
      return Result::InvalidArgument;
    }

    return ToResult(SpiPort_StartTransmitAsync(
        ToPortDevice(device), data, static_cast<std::uint32_t>(length), notification, context));
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
  [[nodiscard]] static Result StartTransmitAsync(Device device,
                                                 const std::array<std::uint8_t, Size> &data,
                                                 CompletionNotification notification,
                                                 void *context) noexcept
  {
    return StartTransmitAsync(device, data.data(), data.size(), notification, context);
  }

  /**
   * @brief 启动一次基于 DMA 的非阻塞 SPI 全双工事务。
   * @note MAY 能力：Port 可能返回 Result::Unsupported，业务须回退阻塞
   * Transfer。
   * @param device 目标逻辑设备。
   * @param transmitData
   * 指向待发送数据的缓冲区，函数返回后调用者可立即复用该缓冲区。
   * @param receiveData 用于接收数据的缓冲区，必须保持有效直至事务终止。
   * @param length 同时发送和接收的字节数，不得超过 MaxAsyncLength。
   * @param notification 事务终止时调用的通知函数，允许传入 nullptr。
   * @param context 调用通知函数时原样传回的用户上下文。
   * @return 返回已启动、忙、未就绪、不支持或参数错误等结果。
   * @note 通知在中断上下文执行，最终结果应通过 GetAsyncResult 查询。
   */
  [[nodiscard]] static Result StartTransferAsync(Device device,
                                                 const std::uint8_t *transmitData,
                                                 std::uint8_t *receiveData,
                                                 std::size_t length,
                                                 CompletionNotification notification,
                                                 void *context) noexcept
  {
    if (length > std::numeric_limits<std::uint32_t>::max())
    {
      return Result::InvalidArgument;
    }

    return ToResult(SpiPort_StartTransferAsync(
        ToPortDevice(device), transmitData, receiveData, static_cast<std::uint32_t>(length), notification, context));
  }

  /**
   * @brief 使用两个等长数组启动一次非阻塞 SPI 全双工事务。
   * @tparam Size 发送和接收数组的字节数。
   * @param device 目标逻辑设备。
   * @param transmitData 待发送的定长数组。
   * @param receiveData 用于接收数据的定长数组，必须保持有效直至事务终止。
   * @param notification 事务终止时调用的通知函数，允许传入 nullptr。
   * @param context 调用通知函数时原样传回的用户上下文。
   * @return 返回已启动、忙、未就绪、不支持或参数错误等结果。
   */
  template <std::size_t Size>
  [[nodiscard]] static Result StartTransferAsync(Device device,
                                                 const std::array<std::uint8_t, Size> &transmitData,
                                                 std::array<std::uint8_t, Size> &receiveData,
                                                 CompletionNotification notification,
                                                 void *context) noexcept
  {
    return StartTransferAsync(
        device, transmitData.data(), receiveData.data(), transmitData.size(), notification, context);
  }

  /**
   * @brief 查询指定 SPI 逻辑设备最近一次异步事务的当前或最终结果。
   * @param device 要查询的逻辑设备。
   * @return
   * 事务进行中时返回忙，终止后返回完成或错误，无有效设备时返回参数错误。
   * @note MAY 能力：异步未实现时返回 Result::Unsupported。
   */
  [[nodiscard]] static Result GetAsyncResult(Device device) noexcept
  {
    return ToResult(SpiPort_GetAsyncResult(ToPortDevice(device)));
  }

private:
  /**
   * @brief 将 C++ 逻辑设备转换为 Detail C ABI 使用的设备值。
   * @param device C++ 逻辑设备。
   * @return 对应的 C ABI 设备值。
   */
  [[nodiscard]] static constexpr SpiPort_Device ToPortDevice(Device device) noexcept
  {
    return static_cast<SpiPort_Device>(device);
  }

  /**
   * @brief 将 Detail C ABI 的 SPI 结果转换为 C++ 结果枚举。
   * @param result C ABI SPI 结果。
   * @return 对应的 C++ 结果，未知值统一转换为 Result::Error。
   */
  [[nodiscard]] static constexpr Result ToResult(SpiPort_Result result) noexcept
  {
    return result <= SPI_PORT_RESULT_STARTED ? static_cast<Result>(result) : Result::Error;
  }
};

} // namespace platform

#endif