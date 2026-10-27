#ifndef LIB_ALG_ESTIMATE_EXTENDED_STATE_OBSERVER_HPP
#define LIB_ALG_ESTIMATE_EXTENDED_STATE_OBSERVER_HPP

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace alg_estimate
{

/** @brief 本组件支持的最大对象阶数（工程选择）：RoboMaster 场景通常 1–4 阶，更高阶请自行实现并校验。 */
inline constexpr std::size_t EXTENDED_STATE_OBSERVER_MAX_ORDER{10U};

/**
 * @brief 校验 fal 参数是否合法：α 有限且 0 < α < 1，δ 有限且大于零。
 *
 * 该函数存在的理由是 Fal 对非法参数采取"静默退化为线性修正"的容错策略：
 * 调用方若把 Fal 注入观测器，参数写错时不会收到任何错误反馈，只会得到一个
 * 按线性修正运行的观测器（而增益可能按非线性整定）。因此注入前必须先校验。
 *
 * @param alpha 指数 α，必须满足 0 < α < 1。
 * @param linear_zone 线性区宽度 δ，必须大于零。
 * @return 参数合法返回 true。
 */
[[nodiscard]] inline bool fal_parameters_valid(float alpha, float linear_zone) noexcept
{
    return std::isfinite(alpha) && std::isfinite(linear_zone) && alpha > 0.0F && alpha < 1.0F && linear_zone > 0.0F;
}

/**
 * @brief 非线性 fal 修正函数，韩京清统一误差修正形式的核心环节。
 *
 * 定义：|e| ≤ δ 时 fal = e·δ^(α−1)；|e| > δ 时 fal = |e|^α·sign(e)。
 * 典型非线性 ESO 要求 0 < α < 1（小误差大增益、大误差小增益）；需要线性
 * 修正时直接用观测器默认线性模式，不要令 α 趋近 1。
 *
 * 安全约定：e 非有限时原样返回（观测器内部会拒绝该结果）；参数非法
 * （α 或 δ 非有限、α 不在开区间 (0,1)、δ ≤ 0）时退化为返回 e。
 * 线性区按等价形式 (e/δ)·δ^α 计算：|e/δ| ≤ 1 且 δ^α 有界，极小 δ 配
 * 极小 α 时也不会像直接计算 δ^(α−1) 那样中间溢出为 Inf。
 *
 * @param error 观测误差 e，本组件约定 e = z₁ − y（估计减量测）。
 * @param alpha 指数 α，必须满足 0 < α < 1。
 * @param linear_zone 线性区宽度 δ，必须大于零。
 * @note 非法参数静默退化为线性修正，观测器不会报错：注入前请用
 *       fal_parameters_valid 校验，否则整定结果与预期不符且无从察觉。
 * @note 本函数定义在头文件中，必须保持 inline，否则多编译单元包含即重复定义。
 */
[[nodiscard]] inline float fal(float error, float alpha, float linear_zone) noexcept
{
    // 参数非法时退化为线性修正，避免把非有限值或未定义幂引入观测器更新。
    if (!std::isfinite(error))
    {
        return error;
    }
    if (!fal_parameters_valid(alpha, linear_zone))
    {
        return error;
    }
    const float magnitude = std::fabs(error);
    if (magnitude <= linear_zone)
    {
        // 线性区等价形式 (e/δ)·δ^α：|e/δ| ≤ 1，δ^α 有限，结果恒有限。
        return (error / linear_zone) * std::pow(linear_zone, alpha);
    }
    return std::pow(magnitude, alpha) * ((error >= 0.0F) ? 1.0F : -1.0F);
}

/**
 * @brief 通用扩张状态观测器（ESO）：Order = n 为 n 阶积分链对象，对应 n+1 阶观测器。
 * @note 并发契约：本类不是线程安全的，属于单一所有者（与 QuaternionEkf
 *       一致），Update / Reset 与全部读取接口必须由同一任务串行调用。
 * @note 修正函数的 context 指针由调用方拥有，其生命周期必须长于观测器；
 *       拷贝构造会复制同一 context 指针，两个实例将共享该上下文。
 * @tparam Order 对象阶数 n，须满足 1 ≤ Order ≤ EXTENDED_STATE_OBSERVER_MAX_ORDER
 *               （本组件支持的最大阶数，工程选择而非普适稳定上限）。
 */
template <std::size_t Order> class ExtendedStateObserver final
{
    static_assert(Order >= 1U, "ESO 阶数必须大于等于 1");
    static_assert(Order <= EXTENDED_STATE_OBSERVER_MAX_ORDER,
                  "ESO 阶数超过本组件支持的最大阶数 ExtendedStateObserverMaxOrder，"
                  "更高阶观测器请自行实现并校验");

  public:
    /** @brief 被观测对象的阶数。 */
    static constexpr std::size_t ORDER_VALUE{Order};
    /** @brief 对象状态数（z₁ … zₙ）。 */
    static constexpr std::size_t STATE_COUNT{Order};
    /** @brief 观测器状态数与增益数（对象状态加扩张状态 zₙ₊₁）。 */
    static constexpr std::size_t GAIN_COUNT{Order + 1U};
    /** @brief 线性修正下前向欧拉的严格稳定上界：h·ω_o 必须小于该值。 */
    static constexpr float DISCRETE_STABILITY_BOUND{2.0F};
    /**
   * @brief 连续失败达到该次数即判定观测器发散（判据风格与 QuaternionEkf 一致）。
   * @note 1 kHz 调用下相当于 50 ms 持续不可用，足够区分瞬时坏样本与真发散。
   */
    static constexpr std::uint32_t DIVERGENCE_FAILURE_LIMIT{50U};

    /** @brief 一次 Update 的结果分类，供上层区分"输入坏"与"观测器坏"。 */
    enum class UpdateStatus : std::uint8_t
    {
        NOT_UPDATED = 0U,   // 尚未更新或已复位
        OK,                 // 最近一次更新成功
        NOT_CONFIGURED,     // 步长或增益未配置
        INVALID_INPUT,      // 量测或控制量非有限
        CORRECTION_INVALID, // 修正函数返回非有限
        DIVERGENCE          // 下一拍状态非有限（数值发散，本次更新被拒绝）
    };

    /**
   * @brief 运行诊断快照：最近一次结果与失败/发散计数。
   * @note 与 QuaternionEkf::State 的计数器风格一致，供上层判活、报警与复位决策。
   */
    struct Diagnostics final
    {
        UpdateStatus last_status{UpdateStatus::NOT_UPDATED}; // 最近一次 Update 的结果
        std::uint32_t update_count{0U};                      // 成功更新次数
        std::uint32_t failure_count{0U};                     // 累计失败次数
        std::uint32_t consecutive_failures{0U};              // 连续失败次数，成功即清零
        std::uint32_t divergence_count{0U};                  // 判定为发散的次数（进入发散的次数）
    };

    /**
     * @brief 统一误差修正函数指针，输入 e = z₁ − y，返回修正量。
     * @note 类型带 noexcept：修正函数禁止抛异常（嵌入式无异常环境）；
     *       不使用 std::function，避免动态分配；传空指针恢复线性修正。
     */
    using CorrectionFunction = float (*)(float error, void *context) noexcept;

    ExtendedStateObserver() noexcept = default;

    /**
   * @brief 设置离散化步长 h（秒），必须为有限正数。
   * @return 步长非有限或非正时返回 false；已配置带宽且线性修正下 h·ω_o ≥ 2
   *         时同样返回 false（该组合必然发散，见类注释），不改变原有配置。
   */
    bool set_step_time(float seconds) noexcept
    {
        if (!std::isfinite(seconds) || seconds <= 0.0F)
        {
            return false;
        }
        // 线性修正的误差极点为 1 − h·ω_o，h·ω_o ≥ 2 时必然发散，配置阶段直接拒绝。
        // 非线性修正与手动增益无法套用该判据（见类注释），此处不做检查。
        if (correction_ == nullptr && bandwidth_valid_ && seconds * bandwidth_ >= DISCRETE_STABILITY_BOUND)
        {
            return false;
        }
        step_time_ = seconds;
        step_time_valid_ = true;
        return true;
    }

    /**
     * @brief 按带宽法配置增益：βᵢ = C(Order+1, i)·ω_o^i，i = 1 … Order+1。
     *
     * "极点全配置在 −ω_o"只严格适用于默认线性修正，非线性修正下仅作
     * 调参初值。离散化稳定边界见类注释，ω_o 越大对 h 越敏感。
     *
     * @param observer_bandwidth 观测器带宽 ω_o（rad/s），有限正数。
     * @param input_gain 控制通道增益估计 b₀：取 0 表示忽略已知控制通道；
     *                  取非零值时符号须与真实 b 一致（符号错误会使外部
     *                  补偿反向），幅值越接近 b，zₙ₊₁ 越接近纯扰动。
     * @return 任一增益非有限（ω_o 过大溢出）或非正（过小下溢）时返回
     *         false 且不改变原有配置；已配置步长且线性修正下 h·ω_o ≥ 2
     *         时同样返回 false（该组合必然发散，见类注释）。
     * @note 非线性修正下不校验 h·ω_o（fal 在大误差上增益更小）：若要用
     *       非线性修正跑更大的带宽，请先 set_correction 再配置本函数/步长。
     */
    bool set_bandwidth(float observer_bandwidth, float input_gain) noexcept
    {
        if (!std::isfinite(observer_bandwidth) || observer_bandwidth <= 0.0F || !std::isfinite(input_gain))
        {
            return false;
        }
        float gains[GAIN_COUNT];
        // 逐级递推 β₁ = (n+1)·ω_o；βᵢ = βᵢ₋₁·ω_o·(n+2−i)/i，
        // 与 C(n+1, i)·ω_o^i 数学等价，中间幅度受控。
        gains[0] = static_cast<float>(GAIN_COUNT) * observer_bandwidth;
        for (std::size_t index = 2U; index <= GAIN_COUNT; ++index)
        {
            gains[index - 1U] = gains[index - 2U] * observer_bandwidth *
                                (static_cast<float>(GAIN_COUNT - index + 1U) / static_cast<float>(index));
        }
        for (std::size_t index = 0U; index < GAIN_COUNT; ++index)
        {
            if (!std::isfinite(gains[index]) || gains[index] <= 0.0F)
            {
                return false;
            }
        }
        // 线性修正下按带宽设计出的极点位置已知，可用 h·ω_o 判据预先拒绝必然发散的组合。
        if (correction_ == nullptr && step_time_valid_ && step_time_ * observer_bandwidth >= DISCRETE_STABILITY_BOUND)
        {
            return false;
        }
        for (std::size_t index = 0U; index < GAIN_COUNT; ++index)
        {
            gains_[index] = gains[index];
        }
        input_gain_ = input_gain;
        bandwidth_ = observer_bandwidth;
        gains_valid_ = true;
        bandwidth_valid_ = true;
        return true;
    }

    /**
     * @brief 手动给定各阶增益 β₁ … βₙ₊₁（须全为有限正数），b₀ 语义同 set_bandwidth。
     * @return 参数非法时返回 false 且不改变原有配置。
     * @note 手动增益无法反推 ω_o，因此不做 h·ω_o 判据校验（is_discrete_stable
     *       亦不适用），步长是否过大需由调用方自行评估。
     */
    bool set_gains(const float (&gains)[GAIN_COUNT], float input_gain) noexcept
    {
        if (!std::isfinite(input_gain))
        {
            return false;
        }
        for (std::size_t index = 0U; index < GAIN_COUNT; ++index)
        {
            if (!std::isfinite(gains[index]) || gains[index] <= 0.0F)
            {
                return false;
            }
        }
        for (std::size_t index = 0U; index < GAIN_COUNT; ++index)
        {
            gains_[index] = gains[index];
        }
        input_gain_ = input_gain;
        bandwidth_ = 0.0F;
        gains_valid_ = true;
        bandwidth_valid_ = false;
        return true;
    }

    /**
     * @brief 设置统一误差修正函数 φ(e)，把线性 ESO 扩展为非线性修正形式
     *        （所有增益共用同一 φ，如注入 Fal 得单 fal 形式 NESO）。
     * @param correction 修正函数指针，传 nullptr 恢复线性 φ(e) = e。
     * @param context 回调上下文指针，原样传给修正函数，生命周期须长于观测器。
     * @note 返回值非有限时该次 Update 被拒绝（UpdateStatus::CORRECTION_INVALID）。
     * @note 注入 Fal 前请先用 fal_parameters_valid 校验 α 与 δ：非法参数只会让
     *       Fal 静默退化为线性修正，本组件无法察觉。
     */
    void set_correction(CorrectionFunction correction, void *context) noexcept
    {
        correction_ = correction;
        correction_context_ = context;
    }

    /** @brief 复位全部状态（含扩张状态）为零并清空诊断计数，保留配置。 */
    void reset() noexcept
    {
        for (std::size_t index = 0U; index < GAIN_COUNT; ++index)
        {
            states_[index] = 0.0F;
        }
        last_status_ = UpdateStatus::NOT_UPDATED;
        update_count_ = 0U;
        failure_count_ = 0U;
        consecutive_failures_ = 0U;
        divergence_count_ = 0U;
    }

    /**
     * @brief 按量测复位：z₁ = measurement，其余清零。
     * @return measurement 非有限时返回 false 且状态不变。
     * @note 避免复位后 z₁ 与真实输出失配引起观测瞬态，云台/电机重启推荐使用。
     */
    bool reset(float measurement) noexcept
    {
        if (!std::isfinite(measurement))
        {
            return false;
        }
        reset();
        states_[0] = measurement;
        return true;
    }

    /**
     * @brief 按完整对象状态初值复位：zᵢ = initial_states[i−1]，扩张状态清零。
     * @return 任一元素非有限时返回 false 且状态不变。
     */
    bool reset(const float (&initial_states)[STATE_COUNT]) noexcept
    {
        for (std::size_t index = 0U; index < STATE_COUNT; ++index)
        {
            if (!std::isfinite(initial_states[index]))
            {
                return false;
            }
        }
        reset();
        for (std::size_t index = 0U; index < STATE_COUNT; ++index)
        {
            states_[index] = initial_states[index];
        }
        return true;
    }

    /** @brief 步长与增益均已配置时返回 true。 */
    [[nodiscard]] bool is_configured() const noexcept
    {
        return step_time_valid_ && gains_valid_;
    }

    /**
   * @brief 线性修正下的离散化稳定判据是否成立：h·ω_o < 2。
   * @return 步长与带宽均已配置、当前未注入非线性修正且满足判据时返回 true。
   * @note 判据不适用时（手动增益、非线性修正或配置不完整）返回 false，
   *       与"不满足判据"不可区分，故只作为诊断参考，不参与 Update 决策。
   */
    [[nodiscard]] bool is_discrete_stable() const noexcept
    {
        return step_time_valid_ && bandwidth_valid_ && correction_ == nullptr &&
               step_time_ * bandwidth_ < DISCRETE_STABILITY_BOUND;
    }

    /**
   * @brief 运行诊断快照，用于区分"输入坏"与"观测器坏"，以及判活与报警。
   */
    [[nodiscard]] Diagnostics get_diagnostics() const noexcept
    {
        Diagnostics diagnostics{};
        diagnostics.last_status = last_status_;
        diagnostics.update_count = update_count_;
        diagnostics.failure_count = failure_count_;
        diagnostics.consecutive_failures = consecutive_failures_;
        diagnostics.divergence_count = divergence_count_;
        return diagnostics;
    }

    /**
   * @brief 连续失败次数达到 DIVERGENCE_FAILURE_LIMIT 时返回 true。
   * @note 观测器不会自愈：判定发散后需由上层 Reset 并修正参数或输入。
   */
    [[nodiscard]] bool is_diverged() const noexcept
    {
        return consecutive_failures_ >= DIVERGENCE_FAILURE_LIMIT;
    }

    /**
     * @brief 执行一步观测更新。
     * @param measurement 对象输出量测 y（即 x₁）。
     * @param control 本步施加的控制量 u；b₀ = 0 时被忽略。
     * @return 未完成配置、输入非有限或更新结果非有限（发散）时返回
     *         false，且状态保持不变。
     * @note 失败原因与计数见 get_diagnostics / is_diverged：仅靠返回值无法
     *       区分"量测坏"与"观测器发散"，上层告警与复位策略需要成因信息。
     */
    bool update(float measurement, float control) noexcept
    {
        if (!is_configured())
        {
            return fail(UpdateStatus::NOT_CONFIGURED);
        }
        if (!std::isfinite(measurement) || !std::isfinite(control))
        {
            return fail(UpdateStatus::INVALID_INPUT);
        }
        // 误差约定 e = z₁ − y，修正项 −β·φ(e) 与教材 +β·φ(e) 等价。
        const float error = states_[0] - measurement;
        float corrected = error;
        if (correction_ != nullptr)
        {
            corrected = correction_(error, correction_context_);
            if (!std::isfinite(corrected))
            {
                return fail(UpdateStatus::CORRECTION_INVALID);
            }
        }
        // 先算出全部下一拍状态再整体提交：更新要么完整成功、要么不变。
        float next[GAIN_COUNT];
        for (std::size_t index = 0U; index < Order; ++index)
        {
            // 各阶导数 = 下一阶状态 − βᵢ·φ(e)。
            next[index] = states_[index] + step_time_ * (states_[index + 1U] - gains_[index] * corrected);
        }
        // 末阶对象状态导数含 b₀·u 项。
        next[Order - 1U] += step_time_ * input_gain_ * control;
        // 扩张状态导数只有 −βₙ₊₁·φ(e)。
        next[Order] = states_[Order] - step_time_ * gains_[Order] * corrected;
        for (std::size_t index = 0U; index < GAIN_COUNT; ++index)
        {
            if (!std::isfinite(next[index]))
            {
                return fail(UpdateStatus::DIVERGENCE);
            }
        }
        for (std::size_t index = 0U; index < GAIN_COUNT; ++index)
        {
            states_[index] = next[index];
        }
        last_status_ = UpdateStatus::OK;
        ++update_count_;
        consecutive_failures_ = 0U;
        return true;
    }

    /** @brief 编译期下标读取状态估计（0 … Order−1，对应 z₁ … zₙ），越界编译期报错。 */
    template <std::size_t Index> [[nodiscard]] float get_state() const noexcept
    {
        static_assert(Index < Order, "状态下标越界");
        return states_[Index];
    }

    /**
     * @brief 运行期下标读取状态估计。
     * @param index 状态下标（0 … Order−1，对应 z_{index+1}）。
     * @param estimate 输出估计值。
     * @return 下标合法返回 true。
     */
    [[nodiscard]] bool get_state(std::size_t index, float &estimate) const noexcept
    {
        if (index >= Order)
        {
            return false;
        }
        estimate = states_[index];
        return true;
    }

    /** @brief 输出量估计 z₁（即 x₁ 的估计）。 */
    [[nodiscard]] float get_output_estimate() const noexcept
    {
        return states_[0];
    }

    /**
     * @brief 集总扰动估计 zₙ₊₁（扩张状态），稳态含义（要求扰动及其导数
     *        有界，详见类注释）：
     *        b₀ = b 时为纯扰动 f；b₀ ≠ b 时为 f + (b − b₀)·u；
     *        b₀ = 0 时为 f + b·u 的合成量，不可用于 u = (u₀ − zₙ₊₁)/b₀
     *        形式的扰动补偿（要求 b₀ 非零且符号正确）。
     */
    [[nodiscard]] float get_total_disturbance() const noexcept
    {
        return states_[Order];
    }

    /**
     * @brief 读取观测器增益（下标 0 … GAIN_COUNT−1 对应 β₁ … βₙ₊₁）。
     * @return 下标合法返回 true。
     */
    [[nodiscard]] bool get_gain(std::size_t index, float &gain) const noexcept
    {
        if (index >= GAIN_COUNT)
        {
            return false;
        }
        gain = gains_[index];
        return true;
    }

    /** @brief 当前步长 h（秒），未配置时为零。 */
    [[nodiscard]] float get_step_time() const noexcept
    {
        return step_time_;
    }

    /** @brief 当前控制通道增益估计 b₀。 */
    [[nodiscard]] float get_input_gain() const noexcept
    {
        return input_gain_;
    }

    /** @brief 当前观测器带宽 ω_o（rad/s）；手动给增益或未配置时为零。 */
    [[nodiscard]] float get_bandwidth() const noexcept
    {
        return bandwidth_valid_ ? bandwidth_ : 0.0F;
    }

    /** @brief 观测状态数组首地址，长度 GAIN_COUNT（z₁ … zₙ₊₁，连续存储）。 */
    [[nodiscard]] const float *state_data() const noexcept
    {
        return states_;
    }

  private:
    /**
   * @brief 记录一次失败并返回 false，统一维护失败与发散计数。
   * @note 连续失败次数恰好达到阈值时记一次"进入发散"，之后继续失败不再重复计数。
   */
    bool fail(UpdateStatus status) noexcept
    {
        last_status_ = status;
        ++failure_count_;
        if (++consecutive_failures_ == DIVERGENCE_FAILURE_LIMIT)
        {
            ++divergence_count_;
        }
        return false;
    }

    /** 观测状态：z₁ … zₙ 与扩张状态 zₙ₊₁。 */
    float states_[GAIN_COUNT]{};
    /** 观测器增益 β₁ … βₙ₊₁。 */
    float gains_[GAIN_COUNT]{};
    /** 离散化步长 h（秒）。 */
    float step_time_{0.0F};
    /** 控制通道增益估计 b₀。 */
    float input_gain_{0.0F};
    /** 带宽法配置的观测器带宽 ω_o（rad/s），手动给增益时无效。 */
    float bandwidth_{0.0F};
    /** 非线性修正函数，空表示线性。 */
    CorrectionFunction correction_{nullptr};
    /** 修正函数上下文。 */
    void *correction_context_{nullptr};
    /** 最近一次 Update 的结果。 */
    UpdateStatus last_status_{UpdateStatus::NOT_UPDATED};
    /** 成功更新次数。 */
    std::uint32_t update_count_{0U};
    /** 累计失败次数。 */
    std::uint32_t failure_count_{0U};
    /** 连续失败次数，成功即清零。 */
    std::uint32_t consecutive_failures_{0U};
    /** 判定为发散的次数。 */
    std::uint32_t divergence_count_{0U};
    /** 步长已配置标志。 */
    bool step_time_valid_{false};
    /** 增益已配置标志。 */
    bool gains_valid_{false};
    /** 带宽已由 set_bandwidth 配置标志（决定 h·ω_o 判据是否适用）。 */
    bool bandwidth_valid_{false};
};

} // namespace alg_estimate

#endif // LIB_ALG_ESTIMATE_EXTENDED_STATE_OBSERVER_HPP
