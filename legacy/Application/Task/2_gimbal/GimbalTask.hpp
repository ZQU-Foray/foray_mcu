#ifndef APPLICATION_TASK_GIMBAL_TASK_HPP
#define APPLICATION_TASK_GIMBAL_TASK_HPP

#include "Application/Module/GimbalModule.hpp"
#include <cstdint>

namespace application::task
{

/**
 * @brief 云台双轴任务：独占一条 CAN 通道的 GM6020 会话，以 1ms 节拍驱动
 *        "收发—快照—闭环—指令"整条链，指令出口为输出轴力矩。
 *
 * 本任务只做**外设所有权与节拍**（独占电机组、`Process/ReadState/SetTorque`、
 * 线程生命周期），业务决策全部在 `application::module::GimbalModule`（配置校验、产品策略、
 * 诊断聚合）。因此本文件不包含任何控制律、限幅或失败策略。
 *
 * @note 电机指令在本周期计算、下个发送节拍生效，链路固有一步延迟。
 * @note 使能由外部（控制权裁决层/测试入口）通过 `SetEnabled` 给出：任务启动时**不再**
 *       强制使能——否则失能、急停与后续失联保护都会被下一拍覆盖。
 */
class GimbalTask final
{
public:
  using GimbalControlMode = module::GimbalModule::GimbalControlMode;

  GimbalTask() = delete;

  /**
   * @brief 初始化云台电机会话（含在役声明与保活）与业务模块。
   * @return 通道就绪、配置有效且限幅不超过执行器能力时返回 true。
   * @note 限幅能力由设备层查询后传给模块（`MotorGroup::MaximumTorqueNewtonMeter()`），
   *       模块因此不必依赖设备层类型。
   */
  [[nodiscard]] static bool Init() noexcept;

  /**
   * @brief CMSIS-RTOS2 任务入口，由外部创建线程后调用。
   */
  static void Run(void *argument) noexcept;

  [[nodiscard]] static bool IsReady() noexcept;

  /**
   * @brief 云台闭环参数（每轴一份，yaw/pitch 独立整定），亦供等价性测试引用。
   */
  [[nodiscard]] static module::GimbalModule::Config ControllerConfig() noexcept;

  /**
   * @brief 切换控制模式（力矩/转速/角度），语义见 `GimbalModule`。
   */
  static void SetMode(GimbalControlMode mode) noexcept;

  /**
   * @brief 设定单轴目标：Torque→N·m，Speed→输出轴 rpm，Angle→输出轴 deg。
   */
  static void SetGimbalTarget(std::uint8_t gimbal, float value) noexcept;

  /**
   * @brief 读取单轴目标（遥测与验收观测用，不改变任何状态）。
   * @note 云台失败策略是"保持最后目标"，该读数因此也是失败后行为的直接观测量。
   */
  [[nodiscard]] static float GimbalTarget(std::uint8_t gimbal) noexcept;

  /**
   * @brief 闭环总开关：关闭时全部电机力矩指令恒 0（由裁决层或测试入口给出）。
   */
  static void SetEnabled(bool enabled) noexcept;

  /**
   * @brief 业务模块只读访问，供诊断、遥测与验收脚本读取状态。
   */
  [[nodiscard]] static module::GimbalModule &Module() noexcept;

private:
  static constexpr std::uint32_t PeriodMs{1U};
};

} // namespace application::task

#endif // APPLICATION_TASK_GIMBAL_TASK_HPP
