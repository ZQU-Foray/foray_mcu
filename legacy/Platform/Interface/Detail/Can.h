#ifndef PLATFORM_INTERFACE_DETAIL_CAN_H
#define PLATFORM_INTERFACE_DETAIL_CAN_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Port 实现能力分级（约束 Port 移植；业务代码须按此兼容）：
 *   - MUST：任何 Port 实现都必须完整支持，业务代码可无条件依赖；
 *   - MAY ：Port 可以不实现。未实现时按该函数注释中的降级约定返回
 *           （返回 false，或返回 *_UNSUPPORTED），业务代码必须能处理该返回值。
 *   未标注能力分级的函数均为 MUST。
 */

#ifdef __cplusplus
extern "C"
{
#endif

  typedef uint8_t CanPort_Channel;
  typedef uint8_t CanPort_IdentifierType;
  typedef uint8_t CanPort_SendResult;
  typedef uint8_t CanPort_ReceiveResult;
  typedef void (*CanPort_ReceiveNotification)(void *context);

  enum
  {
    CAN_PORT_CHANNEL_1 = 0U,
    CAN_PORT_CHANNEL_2,
    CAN_PORT_CHANNEL_3,
    CAN_PORT_CHANNEL_COUNT
  };

  enum
  {
    CAN_PORT_IDENTIFIER_STANDARD = 0U,
    CAN_PORT_IDENTIFIER_EXTENDED
  };

  enum
  {
    CAN_PORT_SEND_QUEUED = 0U,
    CAN_PORT_SEND_QUEUE_FULL,
    CAN_PORT_SEND_NOT_READY,
    CAN_PORT_SEND_BUS_OFF,
    CAN_PORT_SEND_INVALID_ARGUMENT,
    CAN_PORT_SEND_ERROR
  };

  enum
  {
    CAN_PORT_RECEIVE_RECEIVED = 0U,
    CAN_PORT_RECEIVE_EMPTY,
    CAN_PORT_RECEIVE_NOT_READY,
    CAN_PORT_RECEIVE_BUS_OFF,
    CAN_PORT_RECEIVE_INVALID_ARGUMENT,
    CAN_PORT_RECEIVE_ERROR
  };

  enum
  {
    CAN_PORT_MAX_DATA_LENGTH = 8U
  };

  typedef struct
  {
    uint32_t rx_dropped_count;
    uint32_t rx_hardware_loss_event_count;
    uint32_t bus_off_count;
  } CanPort_Statistics;

  /**
 * @brief 初始化全部 CAN 通道、接收过滤器、中断通知和软件接收队列。
 * @return 全部通道均成功进入可用状态时返回 true，否则返回 false。
 * @note 应在 CubeMX 完成 FDCAN 外设初始化后、调度器启动前调用。
 */
  bool CanPort_Init(void);

  /**
 * @brief 查询指定 CAN 通道是否可以收发数据。
 * @param channel 要查询的逻辑 CAN 通道。
 * @return 通道有效且底层控制器已启动、未处于 Bus-Off 时返回 true。
 */
  bool CanPort_IsReady(CanPort_Channel channel);

  /**
 * @brief 配置指定通道的标准帧标识符范围过滤器。
 * @param channel 要配置的逻辑 CAN 通道。
 * @param enabled 为 true 时启用范围过滤，为 false 时禁用标准帧接收。
 * @param first_identifier 允许接收的最小标准标识符。
 * @param last_identifier 允许接收的最大标准标识符。
 * @return 参数有效且底层过滤器配置成功时返回 true。
 * @note 应由单一控制任务串行调用，标识符范围必须位于 0x000 至 0x7FF。
 * @note 能力分级：MAY。可不实现，返回 false；未实现时 Port 应保持接收全部标准帧
 * （0x000-0x7FF），由业务自行软件过滤。
 */
  bool CanPort_ConfigureStandardReceiveFilter(CanPort_Channel channel,
                                              bool enabled,
                                              uint32_t first_identifier,
                                              uint32_t last_identifier);

  /**
 * @brief 为指定 CAN 通道注册接收通知函数及其上下文。
 * @param channel 要设置通知的逻辑 CAN 通道。
 * @param notification 收到并入队新帧后调用的通知函数，传入 NULL 可取消通知。
 * @param context 调用通知函数时原样传回的用户上下文。
 * @return 通道有效且通知设置成功时返回 true。
 * @note 通知在中断上下文执行，只应用于轻量唤醒，帧数据仍需通过
 * CanPort_TryReceive 读取。
 * @note 能力分级：MAY。可不实现，返回 false 表示未注册通知；业务应回退为轮询
 * CanPort_TryReceive。
 */
  bool CanPort_SetReceiveNotification(CanPort_Channel channel, CanPort_ReceiveNotification notification, void *context);

  /**
 * @brief 尝试把一帧 Classic CAN 数据加入指定通道的硬件发送队列。
 * @param channel 目标逻辑 CAN 通道。
 * @param identifier 标准或扩展帧标识符。
 * @param identifier_type 标识符类型。
 * @param length 数据长度，允许范围为 0 至 CAN_PORT_MAX_DATA_LENGTH。
 * @param data 指向待发送数据的缓冲区。
 * @return 返回排队成功、队列已满、通道未就绪、Bus-Off 或参数错误等结果。
 * @note 本函数非阻塞；返回 CAN_PORT_SEND_QUEUED 仅表示已进入硬件发送队列。
 */
  CanPort_SendResult CanPort_TrySend(CanPort_Channel channel,
                                     uint32_t identifier,
                                     CanPort_IdentifierType identifier_type,
                                     uint8_t length,
                                     const uint8_t *data);

  /**
 * @brief 尝试从指定通道的软件接收队列读取一帧 CAN 数据。
 * @param channel 要读取的逻辑 CAN 通道。
 * @param identifier 用于接收帧标识符的输出指针。
 * @param identifier_type 用于接收标识符类型的输出指针。
 * @param length 用于接收数据长度的输出指针。
 * @param data 用于接收帧数据的缓冲区。
 * @param data_capacity data 缓冲区可容纳的字节数。
 * @return 返回读取成功、队列为空、通道未就绪、Bus-Off 或参数错误等结果。
 * @note 每个通道只允许一个任务作为软件队列消费者。
 */
  CanPort_ReceiveResult CanPort_TryReceive(CanPort_Channel channel,
                                           uint32_t *identifier,
                                           CanPort_IdentifierType *identifier_type,
                                           uint8_t *length,
                                           uint8_t *data,
                                           uint8_t data_capacity);

  /**
 * @brief 读取指定 CAN 通道的接收丢弃、硬件丢失事件和 Bus-Off 统计。
 * @param channel 要查询的逻辑 CAN 通道。
 * @param statistics 用于接收统计快照的输出指针。
 * @return 参数有效并成功取得统计时返回 true。
 * @note 能力分级：MAY。可不实现，返回 false；业务不得依赖统计，失败应按全 0
 * 处理。
 */
  bool CanPort_GetStatistics(CanPort_Channel channel, CanPort_Statistics *statistics);

#ifdef __cplusplus
}
#endif

#endif