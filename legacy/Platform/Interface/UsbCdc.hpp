#ifndef PLATFORM_INTERFACE_USB_CDC_HPP
#define PLATFORM_INTERFACE_USB_CDC_HPP

#include "Detail/UsbCdc.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace platform
{

class UsbCdc final
{
public:
  static constexpr std::size_t MaximumTransmitSize{USB_CDC_PORT_MAX_TRANSMIT_SIZE};

  enum class ReceiveResult : std::uint8_t
  {
    Received = USB_CDC_PORT_RECEIVE_RECEIVED,
    Empty = USB_CDC_PORT_RECEIVE_EMPTY,
    NotReady = USB_CDC_PORT_RECEIVE_NOT_READY,
    InvalidArgument = USB_CDC_PORT_RECEIVE_INVALID_ARGUMENT,
    Error = USB_CDC_PORT_RECEIVE_ERROR
  };

  enum class TransmitResult : std::uint8_t
  {
    Started = USB_CDC_PORT_TRANSMIT_STARTED,
    Busy = USB_CDC_PORT_TRANSMIT_BUSY,
    NotReady = USB_CDC_PORT_TRANSMIT_NOT_READY,
    InvalidArgument = USB_CDC_PORT_TRANSMIT_INVALID_ARGUMENT,
    Error = USB_CDC_PORT_TRANSMIT_ERROR
  };

  using ReceiveNotification = UsbCdcPort_ReceiveNotification;
  using TransmitNotification = UsbCdcPort_TransmitNotification;

  struct Statistics final
  {
    std::uint32_t receivedByteCount{0U};
    std::uint32_t droppedByteCount{0U};
    std::uint32_t receiveErrorCount{0U};
    std::uint32_t transmittedByteCount{0U};
    std::uint32_t transmitFailureCount{0U};
    std::uint32_t connectionCount{0U};
    std::uint32_t disconnectionCount{0U};
  };

  /**
   * @brief 禁止创建 UsbCdc 实例，所有能力均通过静态方法访问。
   */
  UsbCdc() = delete;

  /**
   * @brief 查询 USB CDC Port 的软件状态是否已经初始化。
   * @return 已完成 Port 初始化时返回 true。
   */
  [[nodiscard]] static bool IsReady() noexcept
  {
    return UsbCdcPort_IsReady();
  }

  /**
   * @brief 查询 USB CDC 是否已由主机完成配置并可进行数据传输。
   * @return Port 已初始化且 USB Device 处于配置状态时返回 true。
   */
  [[nodiscard]] static bool IsConnected() noexcept
  {
    return UsbCdcPort_IsConnected();
  }

  /**
   * @brief 注册 USB CDC 接收通知函数及其上下文。
   * @param notification 新数据进入软件队列后调用的通知函数，传入 nullptr
   * 可取消通知。
   * @param context 调用通知函数时原样传回的用户上下文。
   * @return Port 已初始化且通知设置成功时返回 true。
   * @note 通知在 USB 中断上下文执行，只应用于轻量唤醒，数据仍需通过 TryRead
   * 读取。
   * @note MAY 能力：Port 可能返回 false（未注册），业务应回退轮询 TryRead。
   */
  [[nodiscard]] static bool SetReceiveNotification(ReceiveNotification notification, void *context) noexcept
  {
    return UsbCdcPort_SetReceiveNotification(notification, context);
  }

  /**
   * @brief 注册 USB CDC 发送完成通知函数及其上下文。
   * @param notification 当前发送完成后调用的通知函数，传入 nullptr 可取消通知。
   * @param context 调用通知函数时原样传回的用户上下文。
   * @return Port 已初始化且通知设置成功时返回 true。
   * @note 通知在 USB 中断上下文执行，只应用于轻量唤醒。
   * @note MAY 能力：Port 可能返回 false（未注册），业务应回退为轮询发送状态。
   */
  [[nodiscard]] static bool SetTransmitNotification(TransmitNotification notification, void *context) noexcept
  {
    return UsbCdcPort_SetTransmitNotification(notification, context);
  }

  /**
   * @brief 从 USB CDC 软件接收队列中非阻塞读取数据。
   * @param data 用于接收数据的缓冲区。
   * @param dataCapacity data 缓冲区可容纳的字节数。
   * @param length 用于接收实际读取字节数的输出引用。
   * @return 返回读取成功、队列为空、Port 未就绪或参数错误等结果。
   * @note 只允许一个任务作为软件队列消费者。
   */
  [[nodiscard]] static ReceiveResult TryRead(std::uint8_t *data, std::size_t dataCapacity, std::size_t &length) noexcept
  {
    length = 0U;
    if (dataCapacity > std::numeric_limits<std::uint32_t>::max())
    {
      return ReceiveResult::InvalidArgument;
    }

    std::uint32_t receivedLength = 0U;
    const ReceiveResult result =
        ToReceiveResult(UsbCdcPort_TryRead(data, static_cast<std::uint32_t>(dataCapacity), &receivedLength));
    length = receivedLength;
    return result;
  }

  /**
   * @brief 从 USB CDC 软件接收队列读取数据到定长数组中。
   * @tparam Size 接收数组的字节数。
   * @param data 用于接收数据的定长数组。
   * @param length 用于接收实际读取字节数的输出引用。
   * @return 返回读取成功、队列为空、Port 未就绪或参数错误等结果。
   */
  template <std::size_t Size>
  [[nodiscard]] static ReceiveResult TryRead(std::array<std::uint8_t, Size> &data, std::size_t &length) noexcept
  {
    return TryRead(data.data(), data.size(), length);
  }

  /**
   * @brief 尝试启动一次 USB CDC 非阻塞发送。
   * @param data 指向待发送数据的缓冲区，函数返回后调用者可立即复用该缓冲区。
   * @param length 待发送字节数，不得超过 MaximumTransmitSize。
   * @return 返回已启动、忙、未连接或参数错误等结果。
   * @note 仅供任务上下文调用，每次只允许一帧在途。
   */
  [[nodiscard]] static TransmitResult TryWrite(const std::uint8_t *data, std::size_t length) noexcept
  {
    if (length > std::numeric_limits<std::uint32_t>::max())
    {
      return TransmitResult::InvalidArgument;
    }

    return ToTransmitResult(UsbCdcPort_TryWrite(data, static_cast<std::uint32_t>(length)));
  }

  /**
   * @brief 尝试通过 USB CDC 发送整个定长数组。
   * @tparam Size 待发送数组的字节数。
   * @param data 待发送的定长数组。
   * @return 返回已启动、忙、未连接或参数错误等结果。
   */
  template <std::size_t Size>
  [[nodiscard]] static TransmitResult TryWrite(const std::array<std::uint8_t, Size> &data) noexcept
  {
    return TryWrite(data.data(), data.size());
  }

  /**
   * @brief 读取 USB CDC 的收发、丢弃、错误和连接统计。
   * @param statistics 用于接收统计快照的输出对象。
   * @return Port 已初始化并成功取得统计时返回 true。
   * @note MAY 能力：Port 可能返回 false（未实现），业务应按全 0 统计处理。
   */
  [[nodiscard]] static bool GetStatistics(Statistics &statistics) noexcept
  {
    UsbCdcPort_Statistics portStatistics{};
    if (!UsbCdcPort_GetStatistics(&portStatistics))
    {
      return false;
    }

    statistics.receivedByteCount = portStatistics.received_byte_count;
    statistics.droppedByteCount = portStatistics.dropped_byte_count;
    statistics.receiveErrorCount = portStatistics.receive_error_count;
    statistics.transmittedByteCount = portStatistics.transmitted_byte_count;
    statistics.transmitFailureCount = portStatistics.transmit_failure_count;
    statistics.connectionCount = portStatistics.connection_count;
    statistics.disconnectionCount = portStatistics.disconnection_count;
    return true;
  }

private:
  /**
   * @brief 将 Detail C ABI 的接收结果转换为 C++ 结果枚举。
   * @param result C ABI 接收结果。
   * @return 对应的 C++ 接收结果，未知值统一转换为 ReceiveResult::Error。
   */
  [[nodiscard]] static constexpr ReceiveResult ToReceiveResult(UsbCdcPort_ReceiveResult result) noexcept
  {
    return result <= USB_CDC_PORT_RECEIVE_ERROR ? static_cast<ReceiveResult>(result) : ReceiveResult::Error;
  }

  /**
   * @brief 将 Detail C ABI 的发送结果转换为 C++ 结果枚举。
   * @param result C ABI 发送结果。
   * @return 对应的 C++ 发送结果，未知值统一转换为 TransmitResult::Error。
   */
  [[nodiscard]] static constexpr TransmitResult ToTransmitResult(UsbCdcPort_TransmitResult result) noexcept
  {
    return result <= USB_CDC_PORT_TRANSMIT_ERROR ? static_cast<TransmitResult>(result) : TransmitResult::Error;
  }
};

} // namespace platform

#endif