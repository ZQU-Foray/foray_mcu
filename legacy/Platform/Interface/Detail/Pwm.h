#ifndef PLATFORM_INTERFACE_DETAIL_PWM_H
#define PLATFORM_INTERFACE_DETAIL_PWM_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

  typedef uint8_t PwmPort_Channel;
  typedef uint8_t PwmPort_Result;

  enum
  {
    PWM_PORT_CHANNEL_BUZZER = 0U,
    PWM_PORT_CHANNEL_IMU_HEATER,
    PWM_PORT_CHANNEL_COUNT
  };

  enum
  {
    PWM_PORT_RESULT_COMPLETED = 0U,
    PWM_PORT_RESULT_NOT_READY,
    PWM_PORT_RESULT_INVALID_ARGUMENT,
    PWM_PORT_RESULT_ERROR
  };

  enum
  {
    PWM_PORT_DUTY_PERMILLE_MAX = 1000U
  };

  /**
 * @brief 初始化 PWM 逻辑通道并使所有输出保持无效状态。
 * @return 全部通道满足底层时钟、定时器和输出条件时返回 true。
 * @note 应在 CubeMX 完成定时器和 GPIO 初始化后调用一次。
 */
  bool PwmPort_Init(void);

  /**
 * @brief 查询指定 PWM 逻辑通道是否已经初始化。
 * @param channel 要查询的逻辑通道。
 * @return 通道有效且可以设置输出时返回 true。
 */
  bool PwmPort_IsReady(PwmPort_Channel channel);

  /**
 * @brief 设置指定 PWM 逻辑通道的频率和占空比。
 * @param channel 目标逻辑通道。
 * @param frequency_hz 输出频率，单位 Hz，必须大于 0。
 * @param duty_permille 占空比千分值，范围为 0 到 1000。
 * @return 返回完成、未就绪、参数错误或底层错误。
 */
  PwmPort_Result PwmPort_Set(PwmPort_Channel channel, uint32_t frequency_hz, uint16_t duty_permille);

  /**
 * @brief 将指定 PWM 逻辑通道的占空比设置为 0，使输出静音或失效。
 * @param channel 目标逻辑通道。
 * @return 返回完成、未就绪或参数错误。
 */
  PwmPort_Result PwmPort_Silence(PwmPort_Channel channel);

#ifdef __cplusplus
}
#endif

#endif
