#ifndef APPLICATION_TASK_CHASSIS_TASK_HPP
#define APPLICATION_TASK_CHASSIS_TASK_HPP

#include "Application/Task/1_chassis/ChassisController.hpp"
#include <cstdint>

namespace application::task
{

/**
 * @brief 底盘电机基础闭环任务：独占 CAN1 的 C620 会话，以 1ms 节拍驱动
 *        "收发—快照—闭环—指令"整条链，指令出口为输出轴力矩。
 * @note 电机指令在本周期计算、下个发送节拍生效，链路固有一步延迟。
 */
class ChassisTask final
{
public:
  using WheelControlMode = chassis::ChassisController::WheelControlMode;

  ChassisTask() = delete;

  /**
   * @brief 初始化 CAN1 电机会话（含在役声明与保活）与闭环控制器。
   * @return 通道与时基就绪且控制器配置有效时返回 true。
   */
  [[nodiscard]] static bool Init() noexcept;

  /**
   * @brief CMSIS-RTOS2 任务入口，由外部创建线程后调用。
   */
  [[noreturn]] static void Run(void *argument) noexcept;

  [[nodiscard]] static bool IsReady() noexcept;

  /**
   * @brief 底盘闭环参数（M2006/C610 输出轴整定），亦供等价性测试引用。
   */
  [[nodiscard]] static chassis::ChassisController::Config ControllerConfig() noexcept;

  /**
   * @brief 切换控制模式（力矩/转速/角度），语义见 ChassisController。
   */
  static void SetMode(WheelControlMode mode) noexcept;

  /**
   * @brief 设定单轮目标：Torque→N·m，Speed→输出轴 rpm，Angle→输出轴 deg。
   */
  static void SetWheelTarget(std::uint8_t wheel, float value) noexcept;

  /**
   * @brief 读取单轮目标（遥测与验收观测用，不改变任何状态）。
   * @note 底盘失败策略是"运动目标立即归零"，该读数因此也是失败后行为的直接观测量。
   */
  [[nodiscard]] static float WheelTarget(std::uint8_t wheel) noexcept;

  /**
   * @brief 闭环总开关：关闭时全部电机力矩指令恒 0。
   */
  static void SetEnabled(bool enabled) noexcept;

private:
  static constexpr std::uint32_t PeriodMs{1U};
};

} // namespace application::task

#endif
