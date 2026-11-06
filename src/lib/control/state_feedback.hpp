#ifndef LIB_CONTROL_STATE_FEEDBACK_HPP
#define LIB_CONTROL_STATE_FEEDBACK_HPP

#include <array>
#include <cmath>
#include <cstddef>

namespace control
{

/**
 * @brief 状态反馈 u = −K·(x − x_ref)。
 *
 * K 在电脑上算好（tool/control/lqr_design.py），作为数据给进来：行是输入、列是状态，与工具打印的顺序一致。
 * 可以每拍换 K（以后按腿长等参数调度）。纯计算。
 *
 * 不检查 K 能不能让闭环稳定：状态多时特征值在板上算不划算，稳定性由具体的控制律按它自己的模型检查。
 */
template <std::size_t States, std::size_t Inputs>
class StateFeedback final
{
  public:
    using State = std::array<float, States>;
    using Input = std::array<float, Inputs>;
    using Gain = std::array<std::array<float, States>, Inputs>;

    /** @return K 里有非有限值时返回 false，保留原来的 K。 */
    [[nodiscard]] bool set_gain(const Gain &gain) noexcept
    {
        for (const std::array<float, States> &row : gain)
        {
            for (const float value : row)
            {
                if (!std::isfinite(value))
                {
                    return false;
                }
            }
        }
        gain_ = gain;
        valid_ = true;
        return true;
    }

    /**
     * @brief 算一次输出。
     * @param state 当前状态
     * @param reference 期望状态
     * @param output u = −K·(state − reference)；没设过 K 或输入非有限时全为 0
     * @return 没设过 K、输入非有限、或算出的输出非有限（数值溢出）时返回 false
     */
    [[nodiscard]] bool compute(const State &state, const State &reference, Input &output) const noexcept
    {
        output = {};
        if (!valid_ || !all_finite(state) || !all_finite(reference))
        {
            return false;
        }
        for (std::size_t input = 0U; input < Inputs; ++input)
        {
            float sum = 0.0F;
            for (std::size_t state_index = 0U; state_index < States; ++state_index)
            {
                sum += gain_[input][state_index] * (state[state_index] - reference[state_index]);
            }
            if (!std::isfinite(sum))
            {
                output = {};
                return false;
            }
            output[input] = -sum;
        }
        return true;
    }

    [[nodiscard]] const Gain &gain() const noexcept
    {
        return gain_;
    }

  private:
    [[nodiscard]] static bool all_finite(const State &values) noexcept
    {
        for (const float value : values)
        {
            if (!std::isfinite(value))
            {
                return false;
            }
        }
        return true;
    }

    Gain gain_{};
    bool valid_{false};
};

} // namespace control

#endif
