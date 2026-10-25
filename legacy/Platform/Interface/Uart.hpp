#ifndef PLATFORM_INTERFACE_UART_HPP
#define PLATFORM_INTERFACE_UART_HPP

#include "Detail/Uart.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace platform
{

class Uart final
{
public:
  static constexpr std::size_t MaximumTransmitSize{UART_PORT_MAX_TRANSMIT_SIZE};

  enum class Endpoint : std::uint8_t
  {
    RemoteReceiver = UART_PORT_ENDPOINT_REMOTE_RECEIVER,
    DebugConsole = UART_PORT_ENDPOINT_DEBUG_CONSOLE
  };

  enum class ReceiveResult : std::uint8_t
  {
    Received = UART_PORT_RECEIVE_RECEIVED,
    Empty = UART_PORT_RECEIVE_EMPTY,
    NotReady = UART_PORT_RECEIVE_NOT_READY,
    InvalidArgument = UART_PORT_RECEIVE_INVALID_ARGUMENT,
    Error = UART_PORT_RECEIVE_ERROR
  };

  enum class TransmitResult : std::uint8_t
  {
    Started = UART_PORT_TRANSMIT_STARTED,
    Busy = UART_PORT_TRANSMIT_BUSY,
    NotReady = UART_PORT_TRANSMIT_NOT_READY,
    NotSupported = UART_PORT_TRANSMIT_NOT_SUPPORTED,
    InvalidArgument = UART_PORT_TRANSMIT_INVALID_ARGUMENT,
    Error = UART_PORT_TRANSMIT_ERROR
  };

  using ReceiveNotification = UartPort_ReceiveNotification;
  using TransmitNotification = UartPort_TransmitNotification;

  struct Statistics final
  {
    std::uint32_t receivedByteCount{0U};
    std::uint32_t droppedByteCount{0U};
    std::uint32_t errorEventCount{0U};
    std::uint32_t restartFailureCount{0U};
    std::uint32_t transmittedByteCount{0U};
    std::uint32_t transmitFailureCount{0U};
  };

  /**
   * @brief 禁止创建 Uart 实例，所有能力均通过静态方法访问。
   */
  Uart() = delete;

  /**
   * @brief 查询指定 UART 逻辑端点是否已初始化且 DMA 接收保持活动。
   * @param endpoint 要查询的逻辑端点。
   * @return 端点有效并可接收数据时返回 true。
   */
  [[nodiscard]] static bool IsReady(Endpoint endpoint) noexcept
  {
    return UartPort_IsReady(ToPortEndpoint(endpoint));
  }

  /**
   * @brief 为指定 UART 端点注册接收通知函数及其上下文。
   * @param endpoint 要设置通知的逻辑端点。
   * @param notification 新数据进入软件队列后调用的通知函数，传入 nullptr
   * 可取消通知。
   * @param context 调用通知函数时原样传回的用户上下文。
   * @return 端点有效且通知设置成功时返回 true。
   * @note 通知在中断上下文执行，只应用于轻量唤醒，数据仍需通过 TryRead 读取。
   * @note MAY 能力：Port 可能返回 false（未注册），业务应回退轮询 TryRead。
   */
  [[nodiscard]] static bool
  SetReceiveNotification(Endpoint endpoint, ReceiveNotification notification, void *context) noexcept
  {
    return UartPort_SetReceiveNotification(ToPortEndpoint(endpoint), notification, context);
  }

  /**
   * @brief 为指定 UART 端点注册发送终止通知函数及其上下文。
   * @param endpoint 要设置通知的逻辑端点。
   * @param notification DMA 发送完成或异步失败后调用的通知函数，传入 nullptr
   * 可取消通知。
   * @param context 调用通知函数时原样传回的用户上下文。
   * @return 端点有效且通知设置成功时返回 true。
   * @note 通知在中断上下文执行，只应用于轻量唤醒。
   * @note MAY 能力：Port 可能返回 false（未注册），业务应回退为轮询发送状态。
   */
  [[nodiscard]] static bool
  SetTransmitNotification(Endpoint endpoint, TransmitNotification notification, void *context) noexcept
  {
    return UartPort_SetTransmitNotification(ToPortEndpoint(endpoint), notification, context);
  }

  /**
   * @brief 从指定 UART 端点的软件接收队列中非阻塞读取数据。
   * @param endpoint 要读取的逻辑端点。
   * @param data 用于接收数据的缓冲区。
   * @param dataCapacity data 缓冲区可容纳的字节数。
   * @param length 用于接收实际读取字节数的输出引用。
   * @return 返回读取成功、队列为空、端点未就绪或参数错误等结果。
   * @note 每个端点只允许一个任务作为软件队列消费者。
   */
  [[nodiscard]] static ReceiveResult
  TryRead(Endpoint endpoint, std::uint8_t *data, std::size_t dataCapacity, std::size_t &length) noexcept
  {
    length = 0U;
    if (dataCapacity > std::numeric_limits<std::uint32_t>::max())
    {
      return ReceiveResult::InvalidArgument;
    }

    std::uint32_t receivedLength = 0U;
    const ReceiveResult result = ToReceiveResult(
        UartPort_TryRead(ToPortEndpoint(endpoint), data, static_cast<std::uint32_t>(dataCapacity), &receivedLength));
    length = receivedLength;
    return result;
  }

  /**
   * @brief 从指定 UART 端点读取数据到定长数组中。
   * @tparam Size 接收数组的字节数。
   * @param endpoint 要读取的逻辑端点。
   * @param data 用于接收数据的定长数组。
   * @param length 用于接收实际读取字节数的输出引用。
   * @return 返回读取成功、队列为空、端点未就绪或参数错误等结果。
   */
  template <std::size_t Size>
  [[nodiscard]] static ReceiveResult
  TryRead(Endpoint endpoint, std::array<std::uint8_t, Size> &data, std::size_t &length) noexcept
  {
    return TryRead(endpoint, data.data(), data.size(), length);
  }

  /**
   * @brief 尝试通过指定 UART 端点启动一次 DMA 发送。
   * @param endpoint 目标逻辑端点。
   * @param data 指向待发送数据的缓冲区，函数返回后调用者可立即复用该缓冲区。
   * @param length 待发送字节数，不得超过 MaximumTransmitSize。
   * @return 返回已启动、忙、未就绪、不支持或参数错误等结果。
   * @note
   * 仅供任务上下文调用，每个端点只允许一帧在途；忙时应等待发送通知后重试。
   */
  [[nodiscard]] static TransmitResult TryWrite(Endpoint endpoint, const std::uint8_t *data, std::size_t length) noexcept
  {
    if (length > std::numeric_limits<std::uint32_t>::max())
    {
      return TransmitResult::InvalidArgument;
    }

    return ToTransmitResult(UartPort_TryWrite(ToPortEndpoint(endpoint), data, static_cast<std::uint32_t>(length)));
  }

  /**
   * @brief 尝试通过指定 UART 端点发送整个定长数组。
   * @tparam Size 待发送数组的字节数。
   * @param endpoint 目标逻辑端点。
   * @param data 待发送的定长数组。
   * @return 返回已启动、忙、未就绪、不支持或参数错误等结果。
   */
  template <std::size_t Size>
  [[nodiscard]] static TransmitResult TryWrite(Endpoint endpoint, const std::array<std::uint8_t, Size> &data) noexcept
  {
    return TryWrite(endpoint, data.data(), data.size());
  }

  /**
   * @brief 读取指定 UART 端点的收发、丢弃和错误统计。
   * @param endpoint 要查询的逻辑端点。
   * @param statistics 用于接收统计快照的输出对象。
   * @return 参数有效并成功取得统计时返回 true。
   * @note MAY 能力：Port 可能返回 false（未实现），业务应按全 0 统计处理。
   */
  [[nodiscard]] static bool GetStatistics(Endpoint endpoint, Statistics &statistics) noexcept
  {
    UartPort_Statistics portStatistics{};
    if (!UartPort_GetStatistics(ToPortEndpoint(endpoint), &portStatistics))
    {
      return false;
    }

    statistics.receivedByteCount = portStatistics.received_byte_count;
    statistics.droppedByteCount = portStatistics.dropped_byte_count;
    statistics.errorEventCount = portStatistics.error_event_count;
    statistics.restartFailureCount = portStatistics.restart_failure_count;
    statistics.transmittedByteCount = portStatistics.transmitted_byte_count;
    statistics.transmitFailureCount = portStatistics.transmit_failure_count;
    return true;
  }

private:
  /**
   * @brief 将 C++ 逻辑端点转换为 Detail C ABI 使用的端点值。
   * @param endpoint C++ 逻辑端点。
   * @return 对应的 C ABI 端点值。
   */
  [[nodiscard]] static constexpr UartPort_Endpoint ToPortEndpoint(Endpoint endpoint) noexcept
  {
    return static_cast<UartPort_Endpoint>(endpoint);
  }

  /**
   * @brief 将 Detail C ABI 的接收结果转换为 C++ 结果枚举。
   * @param result C ABI 接收结果。
   * @return 对应的 C++ 接收结果，未知值统一转换为 ReceiveResult::Error。
   */
  [[nodiscard]] static constexpr ReceiveResult ToReceiveResult(UartPort_ReceiveResult result) noexcept
  {
    return result <= UART_PORT_RECEIVE_ERROR ? static_cast<ReceiveResult>(result) : ReceiveResult::Error;
  }

  /**
   * @brief 将 Detail C ABI 的发送结果转换为 C++ 结果枚举。
   * @param result C ABI 发送结果。
   * @return 对应的 C++ 发送结果，未知值统一转换为 TransmitResult::Error。
   */
  [[nodiscard]] static constexpr TransmitResult ToTransmitResult(UartPort_TransmitResult result) noexcept
  {
    return result <= UART_PORT_TRANSMIT_ERROR ? static_cast<TransmitResult>(result) : TransmitResult::Error;
  }
};

} // namespace platform

#endif