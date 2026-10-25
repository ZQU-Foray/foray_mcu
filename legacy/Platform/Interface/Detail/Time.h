#ifndef PLATFORM_INTERFACE_DETAIL_TIME_H
#define PLATFORM_INTERFACE_DETAIL_TIME_H

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

  /**
 * @brief 初始化并启动平台高精度周期计数器。
 * @return 计数器存在、核心频率有效且启动成功时返回 true。
 * @note 应在系统时钟配置完成后调用；核心频率改变后需要重新初始化。
 */
  bool TimePort_Init(void);

  /**
 * @brief 查询高精度周期计数器是否已初始化并保持运行。
 * @return 计数器可用时返回 true，否则返回 false。
 */
  bool TimePort_IsReady(void);

  /**
 * @brief 获取高精度周期计数器的当前 32 位计数值。
 * @return 计数器可用时返回当前 Tick，不可用时返回 0。
 * @note 计数值会自然回绕，时间差应使用无符号减法计算。
 */
  uint32_t TimePort_NowTicks(void);

  /**
 * @brief 获取高精度周期计数器每秒产生的 Tick 数。
 * @return 计数器可用时返回频率，单位为 Hz；不可用时返回 0。
 */
  uint32_t TimePort_FrequencyHz(void);

  /**
 * @brief 执行阻塞式微秒延时。
 * @param delay_us 请求延时的微秒数，传入 0 时立即返回。
 * @note 仅用于确实需要忙等待的短时序，不应在中断或长周期任务中滥用。
 */
  void TimePort_DelayUs(uint32_t delay_us);

#ifdef __cplusplus
}
#endif

#endif