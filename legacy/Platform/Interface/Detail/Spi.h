#ifndef PLATFORM_INTERFACE_DETAIL_SPI_H
#define PLATFORM_INTERFACE_DETAIL_SPI_H

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

  typedef uint8_t SpiPort_Device;
  typedef uint8_t SpiPort_Result;
  typedef void (*SpiPort_CompletionNotification)(void *context);

  enum
  {
    SPI_PORT_DEVICE_IMU_ACCELEROMETER = 0U,
    SPI_PORT_DEVICE_IMU_GYROSCOPE,
    SPI_PORT_DEVICE_ADDRESSABLE_LED,
    SPI_PORT_DEVICE_COUNT
  };

  enum
  {
    SPI_PORT_RESULT_COMPLETED = 0U,
    SPI_PORT_RESULT_BUSY,
    SPI_PORT_RESULT_TIMEOUT,
    SPI_PORT_RESULT_NOT_READY,
    SPI_PORT_RESULT_UNSUPPORTED,
    SPI_PORT_RESULT_INVALID_ARGUMENT,
    SPI_PORT_RESULT_ERROR,
    SPI_PORT_RESULT_STARTED
  };

  enum
  {
    SPI_PORT_MAX_ASYNC_LENGTH = 512U
  };

  /**
 * @brief 初始化 SPI 逻辑设备状态、DMA 缓冲区和共享总线所有权。
 * @return 全部逻辑设备及其底层 DMA 条件均满足要求时返回 true。
 * @note 应在 CubeMX 完成 SPI、DMA 和 GPIO 初始化后、调度器启动前调用一次。
 */
  bool SpiPort_Init(void);

  /**
 * @brief 查询指定 SPI 逻辑设备是否可以启动新事务。
 * @param device 要查询的逻辑设备。
 * @return 设备有效、底层外设就绪且共享总线空闲时返回 true。
 */
  bool SpiPort_IsReady(SpiPort_Device device);

  /**
 * @brief 以阻塞方式向指定 SPI 逻辑设备发送数据。
 * @param device 目标逻辑设备。
 * @param data 指向待发送数据的缓冲区。
 * @param length 待发送字节数。
 * @param timeout_ms 允许阻塞等待的最长毫秒数。
 * @return 返回完成、忙、超时、未就绪、不支持或参数错误等结果。
 * @note 仅在线程或初始化上下文调用，事务期间由 Port 管理片选和共享总线。
 */
  SpiPort_Result SpiPort_Transmit(SpiPort_Device device, const uint8_t *data, uint32_t length, uint32_t timeout_ms);

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
  SpiPort_Result SpiPort_Transfer(
      SpiPort_Device device, const uint8_t *transmit_data, uint8_t *receive_data, uint32_t length, uint32_t timeout_ms);

  /**
 * @brief 启动一次基于 DMA 的非阻塞 SPI 发送事务。
 * @note 能力分级：MAY。
 * @param device 目标逻辑设备。
 * @param data 指向待发送数据的缓冲区，函数返回后调用者可立即复用该缓冲区。
 * @param length 待发送字节数，不得超过 SPI_PORT_MAX_ASYNC_LENGTH。
 * @param notification 事务终止时调用的通知函数，允许传入 NULL。
 * @param context 调用通知函数时原样传回的用户上下文。
 * @return 返回已启动、忙、未就绪、不支持或参数错误等结果。
 * @note 通知在中断上下文执行，最终结果应通过 SpiPort_GetAsyncResult 查询。
 * @note 可不实现，返回 SPI_PORT_RESULT_UNSUPPORTED；业务应回退到阻塞
 * SpiPort_Transmit。
 */
  SpiPort_Result SpiPort_StartTransmitAsync(SpiPort_Device device,
                                            const uint8_t *data,
                                            uint32_t length,
                                            SpiPort_CompletionNotification notification,
                                            void *context);

  /**
 * @brief 启动一次基于 DMA 的非阻塞 SPI 全双工事务。
 * @note 能力分级：MAY。
 * @param device 目标逻辑设备。
 * @param transmit_data
 * 指向待发送数据的缓冲区，函数返回后调用者可立即复用该缓冲区。
 * @param receive_data 用于接收数据的缓冲区，必须保持有效直至事务终止。
 * @param length 同时发送和接收的字节数，不得超过 SPI_PORT_MAX_ASYNC_LENGTH。
 * @param notification 事务终止时调用的通知函数，允许传入 NULL。
 * @param context 调用通知函数时原样传回的用户上下文。
 * @return 返回已启动、忙、未就绪、不支持或参数错误等结果。
 * @note 通知在中断上下文执行，最终结果应通过 SpiPort_GetAsyncResult 查询。
 * @note 可不实现，返回 SPI_PORT_RESULT_UNSUPPORTED；业务应回退到阻塞
 * SpiPort_Transfer。
 */
  SpiPort_Result SpiPort_StartTransferAsync(SpiPort_Device device,
                                            const uint8_t *transmit_data,
                                            uint8_t *receive_data,
                                            uint32_t length,
                                            SpiPort_CompletionNotification notification,
                                            void *context);

  /**
 * @brief 查询指定 SPI 逻辑设备最近一次异步事务的当前或最终结果。
 * @param device 要查询的逻辑设备。
 * @return 事务进行中时返回忙，终止后返回完成或错误，无有效设备时返回参数错误。
 * @note 能力分级：MAY。异步未实现时返回 SPI_PORT_RESULT_UNSUPPORTED。
 */
  SpiPort_Result SpiPort_GetAsyncResult(SpiPort_Device device);

#ifdef __cplusplus
}
#endif

#endif