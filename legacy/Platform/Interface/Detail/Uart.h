#ifndef PLATFORM_INTERFACE_DETAIL_UART_H
#define PLATFORM_INTERFACE_DETAIL_UART_H

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

  typedef uint8_t UartPort_Endpoint;
  typedef uint8_t UartPort_ReceiveResult;
  typedef uint8_t UartPort_TransmitResult;
  typedef void (*UartPort_ReceiveNotification)(void *context);
  typedef void (*UartPort_TransmitNotification)(void *context);

  enum
  {
    UART_PORT_ENDPOINT_REMOTE_RECEIVER = 0U,
    UART_PORT_ENDPOINT_DEBUG_CONSOLE,
    UART_PORT_ENDPOINT_COUNT
  };

  enum
  {
    UART_PORT_RECEIVE_RECEIVED = 0U,
    UART_PORT_RECEIVE_EMPTY,
    UART_PORT_RECEIVE_NOT_READY,
    UART_PORT_RECEIVE_INVALID_ARGUMENT,
    UART_PORT_RECEIVE_ERROR
  };

  enum
  {
    UART_PORT_TRANSMIT_STARTED = 0U,
    UART_PORT_TRANSMIT_BUSY,
    UART_PORT_TRANSMIT_NOT_READY,
    UART_PORT_TRANSMIT_NOT_SUPPORTED,
    UART_PORT_TRANSMIT_INVALID_ARGUMENT,
    UART_PORT_TRANSMIT_ERROR
  };

  enum
  {
    UART_PORT_MAX_TRANSMIT_SIZE = 256U
  };

  typedef struct
  {
    uint32_t received_byte_count;
    uint32_t dropped_byte_count;
    uint32_t error_event_count;
    uint32_t restart_failure_count;
    uint32_t transmitted_byte_count;
    uint32_t transmit_failure_count;
  } UartPort_Statistics;

  /**
 * @brief 初始化全部 UART 逻辑端点并启动 Receive-to-Idle DMA 接收。
 * @return 全部端点及其 DMA 均成功进入接收状态时返回 true。
 * @note 应在 CubeMX 完成 UART 和 DMA 初始化后、调度器启动前调用。
 */
  bool UartPort_Init(void);

  /**
 * @brief 查询指定 UART 逻辑端点是否已初始化且 DMA 接收保持活动。
 * @param endpoint 要查询的逻辑端点。
 * @return 端点有效并可接收数据时返回 true。
 */
  bool UartPort_IsReady(UartPort_Endpoint endpoint);

  /**
 * @brief 为指定 UART 端点注册接收通知函数及其上下文。
 * @param endpoint 要设置通知的逻辑端点。
 * @param notification 新数据进入软件队列后调用的通知函数，传入 NULL
 * 可取消通知。
 * @param context 调用通知函数时原样传回的用户上下文。
 * @return 端点有效且通知设置成功时返回 true。
 * @note 通知在中断上下文执行，只应用于轻量唤醒，数据仍需通过 UartPort_TryRead
 * 读取。
 * @note 能力分级：MAY。可不实现，返回 false 表示未注册通知；业务应回退为轮询
 * UartPort_TryRead。
 */
  bool
  UartPort_SetReceiveNotification(UartPort_Endpoint endpoint, UartPort_ReceiveNotification notification, void *context);

  /**
 * @brief 为指定 UART 端点注册发送终止通知函数及其上下文。
 * @param endpoint 要设置通知的逻辑端点。
 * @param notification DMA 发送完成或异步失败后调用的通知函数，传入 NULL
 * 可取消通知。
 * @param context 调用通知函数时原样传回的用户上下文。
 * @return 端点有效且通知设置成功时返回 true。
 * @note 通知在中断上下文执行，只应用于轻量唤醒。
 * @note 能力分级：MAY。可不实现，返回 false 表示未注册通知；业务应回退为轮询
 * 发送状态（忙时重试 UartPort_TryWrite）。
 */
  bool UartPort_SetTransmitNotification(UartPort_Endpoint endpoint,
                                        UartPort_TransmitNotification notification,
                                        void *context);

  /**
 * @brief 从指定 UART 端点的软件接收队列中非阻塞读取数据。
 * @param endpoint 要读取的逻辑端点。
 * @param data 用于接收数据的缓冲区。
 * @param data_capacity data 缓冲区可容纳的字节数。
 * @param length 用于接收实际读取字节数的输出指针。
 * @return 返回读取成功、队列为空、端点未就绪或参数错误等结果。
 * @note 每个端点只允许一个任务作为软件队列消费者。
 */
  UartPort_ReceiveResult
  UartPort_TryRead(UartPort_Endpoint endpoint, uint8_t *data, uint32_t data_capacity, uint32_t *length);

  /**
 * @brief 尝试通过指定 UART 端点启动一次 DMA 发送。
 * @param endpoint 目标逻辑端点。
 * @param data 指向待发送数据的缓冲区，函数返回后调用者可立即复用该缓冲区。
 * @param length 待发送字节数，不得超过 UART_PORT_MAX_TRANSMIT_SIZE。
 * @return 返回已启动、忙、未就绪、不支持或参数错误等结果。
 * @note 仅供任务上下文调用，每个端点只允许一帧在途。
 */
  UartPort_TransmitResult UartPort_TryWrite(UartPort_Endpoint endpoint, const uint8_t *data, uint32_t length);

  /**
 * @brief 读取指定 UART 端点的收发、丢弃和错误统计。
 * @param endpoint 要查询的逻辑端点。
 * @param statistics 用于接收统计快照的输出指针。
 * @return 参数有效并成功取得统计时返回 true。
 * @note 能力分级：MAY。可不实现，返回 false；业务不得依赖统计，失败应按全 0
 * 处理。
 */
  bool UartPort_GetStatistics(UartPort_Endpoint endpoint, UartPort_Statistics *statistics);

#ifdef __cplusplus
}
#endif

#endif