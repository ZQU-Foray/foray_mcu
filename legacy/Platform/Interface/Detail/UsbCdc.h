#ifndef PLATFORM_INTERFACE_DETAIL_USB_CDC_H
#define PLATFORM_INTERFACE_DETAIL_USB_CDC_H

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

  typedef uint8_t UsbCdcPort_ReceiveResult;
  typedef uint8_t UsbCdcPort_TransmitResult;
  typedef void (*UsbCdcPort_ReceiveNotification)(void *context);
  typedef void (*UsbCdcPort_TransmitNotification)(void *context);

  enum
  {
    USB_CDC_PORT_RECEIVE_RECEIVED = 0U,
    USB_CDC_PORT_RECEIVE_EMPTY,
    USB_CDC_PORT_RECEIVE_NOT_READY,
    USB_CDC_PORT_RECEIVE_INVALID_ARGUMENT,
    USB_CDC_PORT_RECEIVE_ERROR
  };

  enum
  {
    USB_CDC_PORT_TRANSMIT_STARTED = 0U,
    USB_CDC_PORT_TRANSMIT_BUSY,
    USB_CDC_PORT_TRANSMIT_NOT_READY,
    USB_CDC_PORT_TRANSMIT_INVALID_ARGUMENT,
    USB_CDC_PORT_TRANSMIT_ERROR
  };

  enum
  {
    USB_CDC_PORT_MAX_TRANSMIT_SIZE = 512U
  };

  typedef struct
  {
    uint32_t received_byte_count;
    uint32_t dropped_byte_count;
    uint32_t receive_error_count;
    uint32_t transmitted_byte_count;
    uint32_t transmit_failure_count;
    uint32_t connection_count;
    uint32_t disconnection_count;
  } UsbCdcPort_Statistics;

  /**
 * @brief 初始化 USB CDC Port 的软件队列、发送状态和运行统计。
 * @return 初始化成功时返回 true。
 * @note 应在 USB Device 协议栈初始化和启动前调用。
 */
  bool UsbCdcPort_Init(void);

  /**
 * @brief 查询 USB CDC Port 的软件状态是否已经初始化。
 * @return 已完成 Port 初始化时返回 true。
 */
  bool UsbCdcPort_IsReady(void);

  /**
 * @brief 查询 USB CDC 是否已由主机完成配置并可进行数据传输。
 * @return Port 已初始化且 USB Device 处于配置状态时返回 true。
 */
  bool UsbCdcPort_IsConnected(void);

  /**
 * @brief 注册 USB CDC 接收通知函数及其上下文。
 * @param notification 新数据进入软件队列后调用的通知函数，传入 NULL
 * 可取消通知。
 * @param context 调用通知函数时原样传回的用户上下文。
 * @return Port 已初始化且通知设置成功时返回 true。
 * @note 通知在 USB 中断上下文执行，只应用于轻量唤醒，数据仍需通过
 * UsbCdcPort_TryRead 读取。
 * @note 能力分级：MAY。可不实现，返回 false 表示未注册通知；业务应回退为轮询
 * UsbCdcPort_TryRead。
 */
  bool UsbCdcPort_SetReceiveNotification(UsbCdcPort_ReceiveNotification notification, void *context);

  /**
 * @brief 注册 USB CDC 发送完成通知函数及其上下文。
 * @param notification 当前发送完成后调用的通知函数，传入 NULL 可取消通知。
 * @param context 调用通知函数时原样传回的用户上下文。
 * @return Port 已初始化且通知设置成功时返回 true。
 * @note 通知在 USB 中断上下文执行，只应用于轻量唤醒。
 * @note 能力分级：MAY。可不实现，返回 false 表示未注册通知；业务应回退为轮询
 * 发送状态（忙时重试 UsbCdcPort_TryWrite）。
 */
  bool UsbCdcPort_SetTransmitNotification(UsbCdcPort_TransmitNotification notification, void *context);

  /**
 * @brief 从 USB CDC 软件接收队列中非阻塞读取数据。
 * @param data 用于接收数据的缓冲区。
 * @param data_capacity data 缓冲区可容纳的字节数。
 * @param length 用于接收实际读取字节数的输出指针。
 * @return 返回读取成功、队列为空、Port 未就绪或参数错误等结果。
 * @note 只允许一个任务作为软件队列消费者。
 */
  UsbCdcPort_ReceiveResult UsbCdcPort_TryRead(uint8_t *data, uint32_t data_capacity, uint32_t *length);

  /**
 * @brief 尝试启动一次 USB CDC 非阻塞发送。
 * @param data 指向待发送数据的缓冲区，函数返回后调用者可立即复用该缓冲区。
 * @param length 待发送字节数，不得超过 USB_CDC_PORT_MAX_TRANSMIT_SIZE。
 * @return 返回已启动、忙、未连接或参数错误等结果。
 * @note 仅供任务上下文调用，每次只允许一帧在途。
 */
  UsbCdcPort_TransmitResult UsbCdcPort_TryWrite(const uint8_t *data, uint32_t length);

  /**
 * @brief 读取 USB CDC 的收发、丢弃、错误和连接统计。
 * @param statistics 用于接收统计快照的输出指针。
 * @return Port 已初始化、参数有效且成功取得统计时返回 true。
 * @note 能力分级：MAY。可不实现，返回 false；业务不得依赖统计，失败应按全 0
 * 处理。
 */
  bool UsbCdcPort_GetStatistics(UsbCdcPort_Statistics *statistics);

#ifdef __cplusplus
}
#endif

#endif