#ifndef APPLICATION_ENTRY_H
#define APPLICATION_ENTRY_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

  /**
   * @brief 应用核前初始化：校验平台能力（时基、GPIO、CAN、SPI、UART）并完成各任务的
   *        非 RTOS 部分初始化。
   * @return 全部前置条件满足且各任务初始化成功时返回 true。
   * @note 由 CubeMX 生成文件在 osKernelInitialize 之前调用（main.c 的 USER CODE 2）。
   * @note 本函数禁止创建线程或任何依赖 RTOS 内核状态的对象。
   */
  bool Application_Init(void);

  /**
   * @brief 应用 RTOS 初始化：按任务表创建全部应用线程。
   * @return 任务表全部创建成功时返回 true。
   * @note 设计口径是在 osKernelInitialize 之后、osKernelStart 之前调用
   *       （MX_FREERTOS_Init 的 USER CODE BEGIN RTOS_THREADS）。
   * @note 幂等：重复调用不会重复创建线程。若在 osKernelStart 之后才首次调用，
   *       仍可正常工作，但 TaskManager 会把创建时机记为 PostKernel，应作为
   *       接线缺陷处理（见 TaskManager::GetDiagnostics）。
   */
  bool Application_InitRtos(void);

  /**
   * @brief 应用运行态入口：确认启动完成并执行应用级周期工作。
   * @return 任务表已就绪并完成本拍工作时返回 true。
   * @note 由内核运行后的默认任务调用：首次调用完成任务表启动确认，此后每次调用
   *       执行应用级周期工作（当前为遥控接收解算）。任务表未就绪时直接返回 false，
   *       不执行任何周期工作。
   * @note 容错：若启动路径未接入 Application_InitRtos，本入口会在首次调用时补做
   *       线程创建，避免漏接一行导致整机静默不启动；此时 TaskManager 会把创建
   *       时机记为 PostKernel，应作为接线缺陷处理。
   */
  bool Application_Start(void);

#ifdef __cplusplus
}
#endif

#endif // APPLICATION_ENTRY_H
