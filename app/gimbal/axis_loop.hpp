#ifndef GIMBAL_AXIS_LOOP_HPP
#define GIMBAL_AXIS_LOOP_HPP

#include "lib/control/pid.hpp"

namespace gimbal
{

/**
 * @brief 单轴串级控制律：角度（rad）→ 角速度（rad/s）→ 力矩（N·m）。纯计算、SI 单位、不读时间。
 *
 * 本类同时是"控制律"这一层的接口样例，以后换 LQR + ESO 等按同样的约定实现：
 * 1. 纯计算、SI 单位、不读时间，`dt` 由调用方给；
 * 2. `measurement.angle` 是连续的（yaw 不在 ±π 处跳变），`reference.angle` 已经展开到它附近，
 *    控制律内部不用管回绕；
 * 3. `reference.rate` 是角速度前馈；
 * 4. 每次 `update()` 之后、`set_torque()` 之前调用一次 `apply(总限幅之后的力矩 − 重力前馈)`：
 *    控制器可以据此做抗饱和、观测扰动；
 * 5. `reset()` 之后的第一次 `update()` 用当时的 `measurement` 初始化内部状态（观测器拿它做初值，
 *    否则一使能就有一下冲击）；
 * 6. 输入非有限时输出 0，内部状态不被污染；
 * 7. `init()` 自己检查参数，调用方不看 `Parameters` 里面有什么；
 * 8. `retune()` 换参数、不清内部状态，参数无效时返回 false 且什么都不改（在线调参用）；
 * 9. `Parameters` 只放 `float` / `std::uint32_t`：调试器要能在运行中改它，而 EmberProbe 只有写对齐的
 *    32 位量才不停核。
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

    /** 本控制律自己的参数。换控制律时这个结构跟着换，调用方不用改。 */
    struct Parameters
    {
        Gains angle; // rad → rad/s
        Gains rate;  // rad/s → N·m
    };

    /** 只给 `Report` 看，`Gimbal` 的控制逻辑不读。换控制律时 `compensation_newton_meter` 保留。 */
    struct Diagnostics
    {
        float compensation_newton_meter; // 为抵消扰动多给的力矩：本控制律是角速度环的积分项
        float rate_target;               // 角度环输出加前馈、限幅后（rad/s），本控制律才有
    };

    struct Measurement
    {
        float angle;          // rad，连续
        float rate;           // rad/s，欧拉角速度
        float joint_velocity; // rad/s，关节相对底座的角速度（本控制律不用）
    };

    struct Reference
    {
        float angle; // rad，已展开到 measurement.angle 附近
        float rate;  // rad/s，前馈
    };

    struct Output
    {
        float torque; // 控制律给出的力矩（未加重力前馈、未做总限幅）。`Gimbal` 只用这一个
        Diagnostics diagnostics;
    };

    /** 换算成 `control::PID::Config`。增益非有限、为负时返回 false。 */
    [[nodiscard]] bool init(const Parameters &parameters) noexcept;

    /** 清零。复位后第一次 `update()` 用当时的测量初始化（本控制律没有需要初始化的内部状态）。 */
    void reset() noexcept;

    /**
     * @brief 换参数，不清内部状态（PID 的积分；以后是观测器的状态）。
     * @return 参数无效或还没 `init()` 时返回 false，什么都不改。
     * @note 给在线调参用：换参数时积分不清零，pitch 不会一沉。
     */
    [[nodiscard]] bool retune(const Parameters &parameters) noexcept;

    /**
     * @brief 算一拍。
     * @param measurement 角度、角速度、关节角速度。
     * @param reference 角度目标（已展开到测量值附近）与角速度前馈。
     * @param dt 这一拍的周期（s）；非正或过大时按 1 ms 算。
     * @note 输入非有限值时输出 0，控制律的内部状态不动。
     */
    [[nodiscard]] Output update(const Measurement &measurement, const Reference &reference, float dt) noexcept;

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
