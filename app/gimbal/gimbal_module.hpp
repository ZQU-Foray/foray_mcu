#ifndef GIMBAL_GIMBAL_MODULE_HPP
#define GIMBAL_GIMBAL_MODULE_HPP

#include "lib/control/pid.hpp"
#include "lib/component/axis_controller.hpp"
#include <cstddef>
#include <cstdint>

namespace application::module
{

/**
 * @brief 云台业务模块：yaw/pitch 双轴的完整业务逻辑，任务层只负责节拍与外设。
 *
 *   - 本模块 = 决策与计算：配置校验、产品策略（失败/目标钳位/默认模式）、诊断聚合；
 *   - `component::AxisController<2>` = 可复用的执行器骨架（模式、串级、限幅链、状态）；
 *   - `control::PID` = 纯算法；
 *   - `GimbalTask` = 外设所有权与 1 ms 节拍（Process/read_state/set_torque），不含业务决策。
 *
 * 模块**零硬件、零 RTOS、零时钟读取**：量测以 `component::AxisMeasurement` 传入，周期由
 * 调用方以 `dt` 注入，执行器力矩能力由调用方传入（`Init` 的第二个参数）。因此本模块的
 * 全部策略（含失败路径）都可以在宿主机上闭环验证。
 *
 *   - D1 失败策略 `HOLD_TARGETS`：命令下发失败时**保留操作者最后的目标**
 *     "云台保持最后目标并切入本地稳定模式"，只复位环避免积分残留；
 *   - D2 初始模式 `Angle`：上电即维持当前指向（目标默认 0 与首次反馈同值，不会突然运动）；
 *   - 角度目标按逐轴机械范围钳位（`TargetClamp::ABSOLUTE_RANGE`），yaw 与 pitch 独立整定；
 *   - D4b 两级限幅都不得超过执行器实际能力，越界在 `Init` 阶段被拒绝并给出原因。
 *
 * @note 目标值、使能位与模式为原子量，允许控制任务之外的单写者设定（遥控/决策任务）；
 *       模式与目标仍是两个独立原子量，跨任务"先设目标再切模式"的撕裂窗口由后续控制权
 *       裁决层统一处理（当前限制已登记）。
 * @note 角度目标必须与量测处于同一累计角系；跨圈归算与最短路径属于目标解算层职责，
 *       本模块不做隐式 wrap。
 */
class GimbalModule final
{
  public:
    static constexpr std::size_t MOTOR_COUNT{2U};

    using GimbalControlMode = component::AxisControlMode;

    /** @brief 单轴（单电机）配置：每轴一份，互不影响。 */
    struct AxisConfig final
    {
        control::PID::Config speed_loop; // 输出轴 rpm → N·m（Maxout＝电调/热限幅）
        control::PID::Config angle_loop; // 输出轴 deg → rpm（Maxout 被 max_gimbal_speed_rpm 覆盖）
        float min_angle_deg{0.0F};              // 该轴角度目标下限（机械限位）
        float max_angle_deg{0.0F};              // 该轴角度目标上限
    };

    /** @brief 模块配置（缺省值不可用：所有字段必须由任务层显式给出）。 */
    struct Config final
    {
        AxisConfig axis[MOTOR_COUNT]{};      // 每轴一份配置
        float max_torque_newton_meter{0.0F}; // 输出轴力矩总限幅（策略总限幅，全轴共享）
        float max_gimbal_speed_rpm{0.0F};    // 输出轴转速目标/外环输出限幅（全轴共享）
    };

    /** @brief Init 失败原因：让"启动失败"可定位，而不是只返回一个 false。 */
    enum class InitFailure : std::uint8_t
    {
        NONE = 0U,
        TORQUE_LIMIT_INVALID,        // 总限幅非有限或非正
        SPEED_LIMIT_INVALID,         // 转速限幅非有限或非正
        ANGLE_RANGE_INVALID,         // 某轴角度范围非法（非有限或下限不小于上限）
        TORQUE_LIMIT_EXCEEDS_DEVICE, // 总限幅超过执行器力矩能力
        SPEED_LIMIT_EXCEEDS_DEVICE,  // 某轴电调/热限幅超过执行器力矩能力
        CONTROLLER_REJECTED          // 骨架拒绝（PID 配置非法等）
    };

    GimbalModule() noexcept = default;

    /**
   * @brief 校验配置并初始化执行器骨架。
   * @param config 模块配置。
   * @param device_maximum_torque_newton_meter 执行器输出轴力矩能力（N·m），由调用方从设备层
   *        查询后传入（例如 `MotorGroup::maximum_torque_newton_meter()`），使本模块无需依赖
   *        设备层类型。能力非有限或非正时视为不可用，`Init` 失败并给出原因。
   * @return 全部校验通过并完成骨架初始化时返回 true；否则返回 false，原因见 get_init_failure。
   */
    [[nodiscard]] bool init(const Config &config, float device_maximum_torque_newton_meter) noexcept;

    /** @brief 复位全部 PID 状态并清零目标，保留配置与模式。 */
    void reset() noexcept;

    void set_enabled(bool enabled) noexcept;

    /** @brief 切换控制模式（含切回原模式）：复位全部 PID 并清零目标。 */
    void set_mode(GimbalControlMode mode) noexcept;

    /** @brief 设定单轴目标：Torque→N·m，Speed→输出轴 rpm，Angle→输出轴 deg。 */
    void set_target(std::size_t gimbal, float value) noexcept;

    /**
   * @brief 命令下发失败时的统一处理：复位环但**保留目标**（云台策略，D1）。
   * @note 与底盘的差异就在这里：要求云台保持最后目标，因此不清目标；
   *       恢复后无需上层重新下发。
   */
    void on_command_failure() noexcept;

    [[nodiscard]] bool is_ready() const noexcept;

    [[nodiscard]] bool is_enabled() const noexcept;

    [[nodiscard]] GimbalControlMode mode() const noexcept;

    [[nodiscard]] float target(std::size_t gimbal) const noexcept;

    /** @brief Init 失败原因；成功后恒为 None。 */
    [[nodiscard]] InitFailure get_init_failure() const noexcept;

    /** @brief 最近一次 Update 的控制器级状态（未就绪/未使能/步长非法/运行中）。 */
    [[nodiscard]] component::ControllerStatus get_status() const noexcept;

    /** @brief 最近一次 Update 的单轴状态，回答"这一轴输出为什么是 0（或为何是开环）"。 */
    [[nodiscard]] bool get_axis_status(std::size_t gimbal, component::AxisStatus &status) const noexcept;

    /** @brief 累计 PID 拒绝计算（数值溢出）次数。 */
    [[nodiscard]] std::uint32_t get_loop_fault_count() const noexcept;

    /** @brief 累计命令下发失败次数（`on_command_failure` 被调用的次数）。 */
    [[nodiscard]] std::uint32_t get_command_failure_count() const noexcept;

    /**
   * @brief 推进一个控制周期：输出各轴力矩指令。
   * @param measurements 各轴量测（下标即轴序，0=yaw、1=pitch）。
   * @param dt 本周期时长（秒），须满足 0 < dt < 1。
   * @param out_torque_newton_meter 输出力矩指令（输出轴 N·m），恒为有限值。
   */
    void update(const component::AxisMeasurement (&measurements)[MOTOR_COUNT],
                float dt,
                float (&out_torque_newton_meter)[MOTOR_COUNT]) noexcept;

  private:
    component::AxisController<MOTOR_COUNT> axes_{};
    InitFailure init_failure_{InitFailure::NONE};
    float device_maximum_torque_newton_meter_{0.0F};
    std::uint32_t command_failure_count_{0U};
};

} // namespace application::module

#endif // GIMBAL_GIMBAL_MODULE_HPP
