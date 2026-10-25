#ifndef APPLICATION_TASK_CHASSIS_CONTROLLER_HPP
#define APPLICATION_TASK_CHASSIS_CONTROLLER_HPP

#include "Libraries/Component/AxisController.hpp"
#include "Libraries/Device/motor/Motor.hpp"
#include <cstddef>
#include <cstdint>

namespace application::chassis
{

/**
 * @brief 底盘闭环控制器：`component::AxisController<4>` 的产品薄壳。
 *
 * 对外统一输出轴语义（角度 deg / 转速输出轴 rpm / 力矩 N·m），按模式选择控制结构——
 * Torque：目标力矩开环直通（钳位后输出，不经过 PID）；
 * Speed ：输出轴转速环（rpm → N·m）；
 * Angle ：串级 角度环→转速环→力矩（角度环输出为目标输出轴转速）。
 *
 * 与骨架的分工：级联结构、限幅链、模式切换、失败处理与诊断都在
 * `component::AxisController` 里只有一份实现；本类只提供底盘的产品决策：
 *   - 失败策略 `ClearTargets`：命令失败时清空目标，上层必须重新下发，
 *     "底盘运动目标立即归零"；
 *   - 初始模式 `Speed`：目标 0 即停转，是底盘更保守的默认；
 *   - 目标预处理 `None`：车轮无机械限位，角度目标不做钳位。
 *
 * @note 纯策略对象：不访问硬件、不读取时间、不依赖 RTOS；dt 由调用方提供，可在
 *       宿主机单测闭环行为。目标值、使能位与模式为原子量，允许控制任务之外的单
 *       写者设定（遥控/决策任务）。
 * @note 模式切换（含切回原模式）即复位全部 PID 并清零目标，积分不跨模式泄漏；
 *       电机离线、控制器禁用或 dt 非法时输出恒 0 并复位。
 * @note 限幅链：角度环与转速目标钳 `maxWheelSpeedRpm`（输出轴 rpm）；转速环输出
 *       钳 `speedLoop.Maxout`（电调/热限幅，N·m）；最终力矩统一钳
 *       `maxTorqueNewtonMeter`（策略总限幅，N·m）。两级限幅都不得超过电机实际
 *       能力（由任务层用 `MotorGroup::MaximumTorqueNewtonMeter()` 校验）。
 * @note `angleLoop.Maxout` 会被 `maxWheelSpeedRpm` 覆盖（外环输出限幅即内环目标上限）。
 */
class ChassisController final
{
public:
  static constexpr std::size_t MotorCount{4U};

  using WheelControlMode = component::AxisControlMode;

  struct Config final
  {
    alg_controller::PID::Config speedLoop; // 输出轴 rpm → N·m
    alg_controller::PID::Config angleLoop; // 输出轴 deg → rpm
    float maxTorqueNewtonMeter{0.0F};      // 输出轴力矩总限幅
    float maxWheelSpeedRpm{0.0F};          // 输出轴转速目标限幅
  };

  ChassisController() noexcept = default;

  /**
   * @brief 校验配置并初始化全部 PID。
   * @return 配置全部有效时返回 true。
   */
  [[nodiscard]] bool Init(const Config &config) noexcept;

  /**
   * @brief 复位全部 PID 状态并清零目标，保留配置与模式。
   */
  void Reset() noexcept;

  void SetEnabled(bool enabled) noexcept;

  /**
   * @brief 切换控制模式（含切回原模式）：复位全部 PID 并清零目标。
   */
  void SetMode(WheelControlMode mode) noexcept;

  /**
   * @brief 设定单轮目标，语义随模式：Torque→N·m，Speed→输出轴 rpm，
   *        Angle→输出轴角度 deg。
   */
  void SetWheelTarget(std::size_t wheel, float value) noexcept;

  /**
   * @brief 命令下发失败时的统一处理：复位环并清空目标（底盘策略）。
   * @note 与 `Reset()` 的区别：语义上属于"失败响应"，由任务层在 `SetTorque` 失败时
   *       调用；目标的清空意味着上层必须重新下发。
   */
  void OnCommandFailure() noexcept;

  [[nodiscard]] bool IsReady() const noexcept;

  [[nodiscard]] bool IsEnabled() const noexcept;

  [[nodiscard]] WheelControlMode Mode() const noexcept;

  [[nodiscard]] float WheelTarget(std::size_t wheel) const noexcept;

  /**
   * @brief 最近一次 Update 的控制器级状态（未就绪/未使能/步长非法/运行中）。
   */
  [[nodiscard]] component::ControllerStatus GetStatus() const noexcept;

  /**
   * @brief 最近一次 Update 的单轮状态，回答"这一轮输出为什么是 0（或为何是开环）"。
   * @return 下标合法返回 true 并写出状态。
   */
  [[nodiscard]] bool GetAxisStatus(std::size_t wheel, component::AxisStatus &status) const noexcept;

  /** @brief 累计 PID 拒绝计算（数值溢出）次数。 */
  [[nodiscard]] std::uint32_t GetLoopFaultCount() const noexcept;

  /**
   * @brief 推进一个控制周期，输出各电机输出轴力矩指令（N·m）。
   * @param snapshots 各电机遥测快照（下标即轮序，对应逻辑编号 0~3）。
   * @param dt 本周期时长（秒）。
   * @param outTorqueNewtonMeter 输出力矩指令，恒为有限值。
   */
  void Update(const device::MotorState (&snapshots)[MotorCount],
              float dt,
              float (&outTorqueNewtonMeter)[MotorCount]) noexcept;

private:
  component::AxisController<MotorCount> axes_{};
};

} // namespace application::chassis

#endif // APPLICATION_TASK_CHASSIS_CONTROLLER_HPP
