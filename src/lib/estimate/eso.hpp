#ifndef LIB_ESTIMATE_ESO_HPP
#define LIB_ESTIMATE_ESO_HPP

#include <array>
#include <cstddef>

namespace estimate
{

/**
 * @brief 线性扩张状态观测器（离散、当前估计形式）。
 *
 * 被控对象看成 Order 阶积分链：y^(Order) = b₀·u + f，f 是"总扰动"（b₀·u 以外的一切：摩擦、重力误差、耦合、
 * b₀ 估偏带来的 (b − b₀)·u、延迟的影响）。状态 z = [y, y′, …, y^(Order−1), f]，测量只有 y。增益按带宽 ω₀ 配置：
 * 误差的 Order + 1 个极点都在 e^(−ω₀·h)（h 是设计周期，由 init() 给）。
 *
 * 每次 update() 先用上一拍实际施加的输入预测、再用这一拍的测量修正；修正后的值就是这一拍的估计，
 * 可以直接用来算这一拍的输出（比"预测形式"少一拍滞后）。
 *
 * 用法：云台用 Order = 1、测量角速度（模型 ω̇ = b₀·τ + f）；只有位置测量的场合用 Order = 2。
 *
 * 纯计算，不读时间；不是线程安全的，单一所有者使用。
 */
template <std::size_t Order>
class Eso final
{
    static_assert(Order == 1U || Order == 2U, "只支持 1、2 阶：增益有闭式解");

  public:
    /** 状态个数：Order 个对象状态（y … y^(Order−1)）加一个扩张状态 f。 */
    static constexpr std::size_t STATE_COUNT{Order + 1U};

    struct Parameters
    {
        float bandwidth_rad_per_second; // ω₀，rad/s
        float input_gain;               // b₀：单位输入带来的 y^(Order) 变化（云台是 1/J）
        float sample_time_second;       // 设计周期 h，增益按它算
    };

    /**
     * @brief 检查参数并算增益。
     * @return ω₀、b₀、h 非有限或不为正，h > 0.1 s，或 ω₀·h > 1 时返回 false，原来的配置不变。
     * @note 只换参数与增益，不动状态估计：在线调参靠这一点；要清状态调 reset()。
     */
    [[nodiscard]] bool init(const Parameters &parameters) noexcept;

    /** 下一次 update() 用测量初始化：z₁ = y，其余为 0。 */
    void reset() noexcept;

    /**
     * @brief 走一拍。
     * @param measurement 这一拍的测量 y。
     * @param applied_input 上一拍到这一拍之间实际施加的输入（限幅之后的）。
     * @param dt 上一拍到这一拍的时间（s）。超过 10 倍设计周期时（中间丢了很多份）不跨着预测：
     *        对象状态用测量重新初始化（z₁ = y，其余为 0），扰动估计保留，否则云台一沉。
     * @return 未配置、输入非有限或 dt 非正时返回 false，状态不动。reset() 之后第一次调用只做初始化。
     */
    [[nodiscard]] bool update(float measurement, float applied_input, float dt) noexcept;

    /** 第 index 个状态的估计（0 是 y，Order − 1 是 y^(Order−1)）；越界返回 0。 */
    [[nodiscard]] float state(std::size_t index) const noexcept;

    /** 总扰动的估计 f̂（单位同 y^(Order)）。 */
    [[nodiscard]] float disturbance() const noexcept;

    /** 增益 L，给测试和调试器看。 */
    [[nodiscard]] const std::array<float, STATE_COUNT> &gains() const noexcept;

  private:
    std::array<float, STATE_COUNT> states_{};
    std::array<float, STATE_COUNT> gains_{};
    float input_gain_{0.0F};
    float sample_time_{0.0F};
    bool configured_{false};
    bool initialized_{false}; // reset() 之后为 false，第一次 update() 用测量初始化
};

} // namespace estimate

#endif
