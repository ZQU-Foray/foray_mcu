#ifndef GIMBAL_AXIS_LOOP_HPP
#define GIMBAL_AXIS_LOOP_HPP

#include "lib/control/pid.hpp"

namespace gimbal
{

/**
 * @brief 单轴串级：角度环（rad → rad/s）串角速度环（rad/s → N·m）。纯计算、SI 单位，不读时间、不碰关节。
 *
 * - 两个环都是 `control::PID` 的 POSITION 模式，不开微分（陀螺本身就是角度的微分，再微一次只会放大噪声）；
 * - `update()` 给出这一拍的力矩，`Gimbal` 加上自己的前馈、做总限幅之后再调 `apply()` 告诉控制器实际下发了多少；
 * - 输入输出与以后换 LQR + ESO 相同：角度、角速度、目标、前馈、dt 进，力矩出。换控制律时只替换这个类，
 *   `Gimbal` 不用改。
 */
class AxisLoop final
{
  public:
    /** 一个环的增益。角度环的 output_limit 是角速度上限 rad/s，角速度环的是力矩上限 N·m。 */
    struct Gains
    {
        float kp;
        float ki;
        float integral_limit; // 积分项的上限（输出单位）
        float output_limit;   // 输出上限
    };

    /** 只给 `Report` 看的中间量；`Gimbal` 的控制逻辑不读它。换控制律时跟着换。 */
    struct Diagnostics
    {
        float rate_target;               // 角度环输出加上前馈、限幅后（rad/s）
        float compensation_newton_meter; // 为抵消扰动多给的力矩：PID 时是角速度环的积分项
    };

    struct Output
    {
        float torque; // 控制律给出的力矩（未加重力前馈、未做总限幅）。`Gimbal` 只用这一个
        Diagnostics diagnostics;
    };

    /** 换算成 `control::PID::Config`。增益非有限、为负，或 translate 失败时返回 false。 */
    [[nodiscard]] bool init(const Gains &angle, const Gains &rate) noexcept;

    void reset() noexcept;

    /**
     * @brief 算一拍。
     * @param angle 角度反馈（rad）。
     * @param angle_target 角度目标（rad）；yaw 要先用 `math::wrap_pi` 展开到 angle 附近。
     * @param rate 角速度反馈（rad/s）。
     * @param rate_feedforward 角速度前馈（rad/s），加在角度环输出上。
     * @param dt 这一拍的周期（s）；非正或过大时按 1 ms 算。
     * @note 输入非有限值时输出 0，环的内部状态不动。
     */
    [[nodiscard]] Output update(float angle, float angle_target, float rate, float rate_feedforward, float dt) noexcept;

    /**
     * @brief 告诉控制器本周期实际下发了多少力矩，用来做积分抗饱和。
     * @param torque 总限幅之后的力矩减去 `Gimbal` 自己加的前馈（重力）：属于本控制器的那一份。
     * @note 力矩被总限幅削掉、且这一拍积分的增量与控制器输出同号时，把这一拍的积分退回去。
     *       `control::PID` 自带的 integral_limit 只看它自己的输出上限，管不到 `Gimbal` 的总限幅，
     *       也不知道重力前馈占掉了多少。
     * @note 没被削时什么都不改。每次 `update()` 之后、`set_torque()` 之前调用一次。
     */
    void apply(float torque) noexcept;

  private:
    control::PID angle_pid_;
    control::PID rate_pid_;
    float integral_before_{0.0F}; // update() 之前角速度环的积分，apply() 回退用
    float torque_{0.0F};          // update() 给出的力矩
    bool initialized_{false};
};

} // namespace gimbal

#endif
