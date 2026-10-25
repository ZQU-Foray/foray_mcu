#ifndef APPLICATION_TASK_IMU_TASK_HPP
#define APPLICATION_TASK_IMU_TASK_HPP

namespace application::task
{

/**
 * @brief BMI088 原始数据采集与姿态解算任务。
 * @note 本任务独占 BMI088 与恒温器；两路 DRDY 通过 ISR 线程标志唤醒任务。
 *       任务收割原始样本后旁路交给 AttitudeEstimator（单位/坐标映射 + VQF）
 *       与 ImuDiagnostics（仅调试构建的只读诊断）；本任务不做滤波、标定或
 *       任何形式的数据平滑。
 */
class ImuTask final
{
public:
  ImuTask() = delete;

  /**
   * @brief 在 RTOS 启动前初始化 BMI088 状态机与恒温器。
   * @return 底层恒温 PWM 就绪时返回 true。
   */
  [[nodiscard]] static bool Init() noexcept;

  /**
   * @brief CMSIS-RTOS2 任务入口，由 TaskManager 创建后调用。
   */
  [[noreturn]] static void Run(void *argument) noexcept;
};

} // namespace application::task

#endif
