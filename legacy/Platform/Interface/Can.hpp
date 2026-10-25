#ifndef PLATFORM_INTERFACE_CAN_HPP
#define PLATFORM_INTERFACE_CAN_HPP

#include "Detail/Can.h"
#include <array>
#include <cstddef>
#include <cstdint>

namespace platform
{

class Can final
{
public:
  static constexpr std::size_t MaxDataLength = CAN_PORT_MAX_DATA_LENGTH;

  enum class Channel : std::uint8_t
  {
    Channel1 = CAN_PORT_CHANNEL_1,
    Channel2 = CAN_PORT_CHANNEL_2,
    Channel3 = CAN_PORT_CHANNEL_3
  };

  enum class IdentifierType : std::uint8_t
  {
    Standard = CAN_PORT_IDENTIFIER_STANDARD,
    Extended = CAN_PORT_IDENTIFIER_EXTENDED
  };

  enum class SendResult : std::uint8_t
  {
    Queued = CAN_PORT_SEND_QUEUED,
    QueueFull = CAN_PORT_SEND_QUEUE_FULL,
    NotReady = CAN_PORT_SEND_NOT_READY,
    BusOff = CAN_PORT_SEND_BUS_OFF,
    InvalidArgument = CAN_PORT_SEND_INVALID_ARGUMENT,
    Error = CAN_PORT_SEND_ERROR
  };

  enum class ReceiveResult : std::uint8_t
  {
    Received = CAN_PORT_RECEIVE_RECEIVED,
    Empty = CAN_PORT_RECEIVE_EMPTY,
    NotReady = CAN_PORT_RECEIVE_NOT_READY,
    BusOff = CAN_PORT_RECEIVE_BUS_OFF,
    InvalidArgument = CAN_PORT_RECEIVE_INVALID_ARGUMENT,
    Error = CAN_PORT_RECEIVE_ERROR
  };

  using ReceiveNotification = CanPort_ReceiveNotification;

  struct Frame final
  {
    std::uint32_t identifier{0U};
    IdentifierType identifierType{IdentifierType::Standard};
    std::uint8_t length{0U};
    std::array<std::uint8_t, MaxDataLength> data{};
  };

  struct Statistics final
  {
    std::uint32_t rxDroppedCount{0U};
    std::uint32_t rxHardwareLossEventCount{0U};
    std::uint32_t busOffCount{0U};
  };

  /**
   * @brief 禁止创建 Can 实例，所有能力均通过静态方法访问。
   */
  Can() = delete;

  /**
   * @brief 查询指定 CAN 通道是否可以收发数据。
   * @param channel 要查询的逻辑通道。
   * @return 通道有效、底层控制器已启动且未处于 Bus-Off 时返回 true。
   */
  [[nodiscard]] static bool IsReady(Channel channel) noexcept
  {
    return CanPort_IsReady(ToPortChannel(channel));
  }

  /**
   * @brief 配置指定通道允许接收的标准帧标识符范围。
   * @param channel 要配置的逻辑通道。
   * @param enabled 为 true 时启用范围过滤，为 false 时禁用标准帧接收。
   * @param firstIdentifier 允许接收的最小标准标识符。
   * @param lastIdentifier 允许接收的最大标准标识符。
   * @return 参数有效且底层过滤器配置成功时返回 true。
   * @note 应由单一控制任务串行调用，标识符范围必须位于 0x000 至 0x7FF。
   * @note MAY 能力：Port 可能返回
   * false（未实现），此时接收全部标准帧，业务须自行软件过滤。
   */
  [[nodiscard]] static bool ConfigureStandardReceiveFilter(Channel channel,
                                                           bool enabled,
                                                           std::uint32_t firstIdentifier,
                                                           std::uint32_t lastIdentifier) noexcept
  {
    return CanPort_ConfigureStandardReceiveFilter(ToPortChannel(channel), enabled, firstIdentifier, lastIdentifier);
  }

  /**
   * @brief 为指定 CAN 通道注册接收通知函数及其上下文。
   * @param channel 要设置通知的逻辑通道。
   * @param notification 收到并入队新帧后调用的通知函数，传入 nullptr
   * 可取消通知。
   * @param context 调用通知函数时原样传回的用户上下文。
   * @return 通道有效且通知设置成功时返回 true。
   * @note 通知在中断上下文执行，只应用于轻量唤醒，帧数据仍需通过 TryReceive
   * 读取。
   * @note MAY 能力：Port 可能返回 false（未注册），业务应回退轮询 TryReceive。
   */
  [[nodiscard]] static bool
  SetReceiveNotification(Channel channel, ReceiveNotification notification, void *context) noexcept
  {
    return CanPort_SetReceiveNotification(ToPortChannel(channel), notification, context);
  }

  /**
   * @brief 尝试把一帧 Classic CAN 数据加入指定通道的硬件发送队列。
   * @param channel 目标逻辑通道。
   * @param frame 待发送帧，包含标识符类型、标识符、长度和数据。
   * @return 返回排队成功、队列已满、通道未就绪、Bus-Off 或参数错误等结果。
   * @note 本函数非阻塞；返回 SendResult::Queued 仅表示帧已进入硬件发送队列。
   */
  [[nodiscard]] static SendResult TrySend(Channel channel, const Frame &frame) noexcept
  {
    return ToSendResult(CanPort_TrySend(ToPortChannel(channel),
                                        frame.identifier,
                                        static_cast<CanPort_IdentifierType>(frame.identifierType),
                                        frame.length,
                                        frame.data.data()));
  }

  /**
   * @brief 尝试从指定通道的软件接收队列读取一帧 CAN 数据。
   * @param channel 要读取的逻辑通道。
   * @param frame 用于接收标识符类型、标识符、长度和数据的输出对象。
   * @return 返回读取成功、队列为空、通道未就绪、Bus-Off 或参数错误等结果。
   * @note 每个通道只允许一个任务作为软件队列消费者。
   */
  [[nodiscard]] static ReceiveResult TryReceive(Channel channel, Frame &frame) noexcept
  {
    CanPort_IdentifierType identifierType = CAN_PORT_IDENTIFIER_STANDARD;
    std::uint32_t identifier = 0U;
    std::uint8_t length = 0U;

    const ReceiveResult result = ToReceiveResult(CanPort_TryReceive(ToPortChannel(channel),
                                                                    &identifier,
                                                                    &identifierType,
                                                                    &length,
                                                                    frame.data.data(),
                                                                    static_cast<std::uint8_t>(frame.data.size())));

    if (result == ReceiveResult::Received)
    {
      frame.identifier = identifier;
      frame.identifierType = static_cast<IdentifierType>(identifierType);
      frame.length = length;
    }

    return result;
  }

  /**
   * @brief 读取指定 CAN 通道的接收丢弃、硬件丢失事件和 Bus-Off 统计。
   * @param channel 要查询的逻辑通道。
   * @param statistics 用于接收统计快照的输出对象。
   * @return 参数有效并成功取得统计时返回 true。
   * @note MAY 能力：Port 可能返回 false（未实现），业务应按全 0 统计处理。
   */
  [[nodiscard]] static bool GetStatistics(Channel channel, Statistics &statistics) noexcept
  {
    CanPort_Statistics portStatistics{};
    if (!CanPort_GetStatistics(ToPortChannel(channel), &portStatistics))
    {
      return false;
    }

    statistics.rxDroppedCount = portStatistics.rx_dropped_count;
    statistics.rxHardwareLossEventCount = portStatistics.rx_hardware_loss_event_count;
    statistics.busOffCount = portStatistics.bus_off_count;
    return true;
  }

private:
  /**
   * @brief 将 C++ 逻辑通道转换为 Detail C ABI 使用的通道值。
   * @param channel C++ 逻辑通道。
   * @return 对应的 C ABI 通道值。
   */
  [[nodiscard]] static constexpr CanPort_Channel ToPortChannel(Channel channel) noexcept
  {
    return static_cast<CanPort_Channel>(channel);
  }

  /**
   * @brief 将 Detail C ABI 的发送结果转换为 C++ 结果枚举。
   * @param result C ABI 发送结果。
   * @return 对应的 C++ 发送结果，未知值统一转换为 SendResult::Error。
   */
  [[nodiscard]] static constexpr SendResult ToSendResult(CanPort_SendResult result) noexcept
  {
    return result <= CAN_PORT_SEND_ERROR ? static_cast<SendResult>(result) : SendResult::Error;
  }

  /**
   * @brief 将 Detail C ABI 的接收结果转换为 C++ 结果枚举。
   * @param result C ABI 接收结果。
   * @return 对应的 C++ 接收结果，未知值统一转换为 ReceiveResult::Error。
   */
  [[nodiscard]] static constexpr ReceiveResult ToReceiveResult(CanPort_ReceiveResult result) noexcept
  {
    return result <= CAN_PORT_RECEIVE_ERROR ? static_cast<ReceiveResult>(result) : ReceiveResult::Error;
  }
};

} // namespace platform

#endif