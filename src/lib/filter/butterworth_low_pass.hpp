#ifndef LIB_FILTER_BUTTERWORTH_LOW_PASS_HPP
#define LIB_FILTER_BUTTERWORTH_LOW_PASS_HPP

#include <cmath>
#include <cstddef>
#include <limits>

namespace filter
{

/**
 * @brief 二阶 Butterworth 低通：时间常数参数化、双精度状态、支持多通道。
 *
 *   1. **初始化阶段**：先用"时间常数"秒对输入做**时间平均**作为输出，避免首样本跳变
 *      污染下游；随后用该均值取稳态初值（等价 `scipy.signal.lfilter_zi`），进入正常阶段。
 *   2. **正常阶段**：转置直接 II 型二阶差分方程（`a0` 归一为 1）。
 *   3. `时间常数 < 采样周期/2` 时退化为直通（截止频率超过约 90% 奈奎斯特，避免不稳定）。
 *
 * @note 状态与系数一律 `double`：浮点状态在长时运行下会出数值问题（上游明确结论，
 *       本工程沿用）。输入/输出保持 `float`，与工程数据类型一致。
 * @note 与同目录 `ButterworthFilter` 的关系：后者按**截止频率**参数化、`float` 状态、
 *       标量、从零初值起步，且当前**无任何消费者**。本组件是为"热路径 + 多通道 +
 *       初始化平均"语义新增的公共实现，两者暂并存；统一时以本组件为准，详见
 *       `Libraries/README.md`。两者描述的是同一个双线性变换公式，参数换算为
 *       `f_c = √2 / (2π·τ)`。
 * @note 无动态分配、无硬件/时间依赖。
 *
 * @tparam Channels 通道数（一次 `Filter` 调用同时处理的轴数）。取 2、3、9 等值。
 */
template <std::size_t Channels> class ButterworthLowPass final
{
    static_assert(Channels >= 2U, "初始化阶段按上游语义组织状态，要求至少两个通道");

  public:
    /** @brief 配置：时间常数与采样周期，单位均为秒。 */
    struct Config final
    {
        float time_constant_seconds{0.0F};
        float sample_period_seconds{0.0F};
    };

    /**
   * @brief 计算系数并复位状态。
   * @param config 时间常数与采样周期。
   * @return 两者有限且为正时返回 true；否则保持未就绪并返回 false。
   */
    [[nodiscard]] bool init(const Config &config) noexcept
    {
        initialized_ = false;
        if (!std::isfinite(config.time_constant_seconds) || !(config.time_constant_seconds > 0.0F) ||
            !std::isfinite(config.sample_period_seconds) || !(config.sample_period_seconds > 0.0F))
        {
            return false;
        }

        time_constant_seconds_ = static_cast<double>(config.time_constant_seconds);
        sample_period_seconds_ = static_cast<double>(config.sample_period_seconds);
        compute_coefficients();
        reset();
        initialized_ = true;
        return true;
    }

    /** @brief 复位状态并重新进入初始化平均阶段。 */
    void reset() noexcept
    {
        for (std::size_t index = 0U; index < 2U * Channels; ++index)
        {
            state_[index] = 0.0;
        }
        for (std::size_t index = 0U; index < Channels; ++index)
        {
            average_sum_[index] = 0.0;
        }
        average_count_ = 0U;
        priming_ = true;
    }

    [[nodiscard]] bool is_initialized() const noexcept
    {
        return initialized_;
    }

    /** @brief 是否仍处于初始化平均阶段（此时输出是时间平均，不是滤波结果）。 */
    [[nodiscard]] bool is_priming() const noexcept
    {
        return priming_;
    }

    /**
   * @brief 组件内部状态快照（调试、离线回放逐段续算、检查点用）。
   * @note 快照里带上滤波器系数：恢复时用它校验"状态与配置同源"，避免用不同
   *       时间常数/采样周期恢复出一条静默发散的轨迹（上游 `VQFState` 不做该校验，
   *       这里是对上游的加固）。
   */
    struct InternalState final
    {
        double state[2U * Channels]{};
        double average_sum[Channels]{};
        double coefficients_b[3]{};
        double coefficients_a[2]{};
        std::size_t average_count{0U};
        bool priming{true};
    };

    [[nodiscard]] InternalState get_internal_state() const noexcept
    {
        InternalState snapshot{};
        for (std::size_t index = 0U; index < 2U * Channels; ++index)
        {
            snapshot.state[index] = state_[index];
        }
        for (std::size_t index = 0U; index < Channels; ++index)
        {
            snapshot.average_sum[index] = average_sum_[index];
        }
        for (std::size_t index = 0U; index < 3U; ++index)
        {
            snapshot.coefficients_b[index] = coefficients_b_[index];
        }
        for (std::size_t index = 0U; index < 2U; ++index)
        {
            snapshot.coefficients_a[index] = coefficients_a_[index];
        }
        snapshot.average_count = average_count_;
        snapshot.priming = priming_;
        return snapshot;
    }

    /**
   * @brief 校验快照是否与当前配置同源（**不修改**任何状态）。
   * @return 组件已就绪、且快照系数与当前 Config/采样周期一致时返回 true。
   * @note 系数比较用相对容差（1e-12），允许跨构建/跨优化级别的末位差异，
   *       但能拦住"换了时间常数却用旧快照恢复"这类错误。
   * @note 提供给调用方做"先全部校验、再整体写入"的两阶段恢复，避免部分子状态
   *       已被写入、其余仍为旧值的混合态。
   */
    [[nodiscard]] bool is_internal_state_compatible(const InternalState &snapshot) const noexcept
    {
        if (!initialized_)
        {
            return false;
        }
        const auto nearly_equal = [](double first, double second) noexcept
        { return std::fabs(first - second) <= 1e-12 * (1.0 + std::fabs(second)); };
        for (std::size_t index = 0U; index < 3U; ++index)
        {
            if (!nearly_equal(snapshot.coefficients_b[index], coefficients_b_[index]))
            {
                return false;
            }
        }
        for (std::size_t index = 0U; index < 2U; ++index)
        {
            if (!nearly_equal(snapshot.coefficients_a[index], coefficients_a_[index]))
            {
                return false;
            }
        }
        return true;
    }

    /**
   * @brief 恢复内部状态。
   * @param snapshot 先前由 `get_internal_state` 读出的快照。
   * @return 校验通过（见 `is_internal_state_compatible`）时写入并返回 true；
   *         否则不改动任何状态并返回 false。
   */
    [[nodiscard]] bool set_internal_state(const InternalState &snapshot) noexcept
    {
        if (!is_internal_state_compatible(snapshot))
        {
            return false;
        }

        for (std::size_t index = 0U; index < 2U * Channels; ++index)
        {
            state_[index] = snapshot.state[index];
        }
        for (std::size_t index = 0U; index < Channels; ++index)
        {
            average_sum_[index] = snapshot.average_sum[index];
        }
        average_count_ = snapshot.average_count;
        priming_ = snapshot.priming;
        return true;
    }

    /** @brief 输入输出分离滤波。 */
    void filter(const float input[Channels], float output[Channels]) noexcept
    {
        if (!initialized_ || input == nullptr || output == nullptr)
        {
            return;
        }
        filter_core(input, output);
    }

    /** @brief 就地滤波（本工程热路径主入口）。 */
    void filter(float values[Channels]) noexcept
    {
        if (!initialized_ || values == nullptr)
        {
            return;
        }
        filter_core(values, values);
    }

  private:
    /**
   * @brief 热路径核心（两个公开入口共用，强制内联以消除包装层）。
   * @note `-O0` 下普通成员函数不会被内联，而本组件在每个样本上要被调用 1–3 次；
   *       实测多一层包装调用会带来约 10% 的低通开销。`always_inline` 让两个公开
   *       入口不引入额外调用层，同时避免把循环体展开成两份。
   */
    [[gnu::always_inline]] inline void filter_core(const float input[Channels], float output[Channels]) noexcept
    {
        if (priming_)
        {
            prime(input, output);
            return;
        }

        // 缓存系数 + 状态指针递进：避免每通道重复计算成员偏移与下标。
        const double b0 = coefficients_b_[0];
        const double b1 = coefficients_b_[1];
        const double b2 = coefficients_b_[2];
        const double a1 = coefficients_a_[0];
        const double a2 = coefficients_a_[1];
        double *state = state_;
        for (std::size_t channel = 0U; channel < Channels; ++channel)
        {
            const double x = static_cast<double>(input[channel]);
            const double y = b0 * x + state[0];
            state[0] = b1 * x - a1 * y + state[1];
            state[1] = b2 * x - a2 * y;
            output[channel] = static_cast<float>(y);
            state += 2U;
        }
    }

    /** @brief 二阶 Butterworth 系数（`b0 b1 b2` / `a1 a2`，`a0 = 1`）。 */
    void compute_coefficients() noexcept
    {
        constexpr double PI{3.14159265358979323846264338327950288};
        constexpr double SQRT2{1.41421356237309504880168872420969808};

        if (time_constant_seconds_ < sample_period_seconds_ / 2.0)
        {
            coefficients_b_[0] = 1.0;
            coefficients_b_[1] = 0.0;
            coefficients_b_[2] = 0.0;
            coefficients_a_[0] = 0.0;
            coefficients_a_[1] = 0.0;
            return;
        }

        const double cutoff_hz = (SQRT2 / (2.0 * PI)) / time_constant_seconds_;
        const double c = std::tan(PI * cutoff_hz * sample_period_seconds_);
        const double denominator = c * c + SQRT2 * c + 1.0;
        const double b0 = c * c / denominator;
        coefficients_b_[0] = b0;
        coefficients_b_[1] = 2.0 * b0;
        coefficients_b_[2] = b0;
        coefficients_a_[0] = 2.0 * (c * c - 1.0) / denominator;       // a1
        coefficients_a_[1] = (1.0 - SQRT2 * c + c * c) / denominator; // a2
    }

    /** @brief 初始化平均阶段：累加求均值，达到时间常数后用均值取稳态初值。 */
    void prime(const float input[Channels], float output[Channels]) noexcept
    {
        ++average_count_;
        for (std::size_t channel = 0U; channel < Channels; ++channel)
        {
            average_sum_[channel] += static_cast<double>(input[channel]);
            const double average = average_sum_[channel] / static_cast<double>(average_count_);
            output[channel] = static_cast<float>(average);
        }
        if (static_cast<double>(average_count_) * sample_period_seconds_ >= time_constant_seconds_)
        {
            for (std::size_t channel = 0U; channel < Channels; ++channel)
            {
                // 稳态初值：在 y = x = x0 处解出滤波器状态，避免进入正常阶段时的跳变。
                state_[2U * channel] = static_cast<double>(output[channel]) * (1.0 - coefficients_b_[0]);
                state_[2U * channel + 1U] =
                    static_cast<double>(output[channel]) * (coefficients_b_[2] - coefficients_a_[1]);
            }
            priming_ = false;
        }
    }

    double coefficients_b_[3]{};
    double coefficients_a_[2]{};
    double state_[2U * Channels]{};
    double average_sum_[Channels]{};
    double time_constant_seconds_{0.0};
    double sample_period_seconds_{0.0};
    std::size_t average_count_{0U};
    bool initialized_{false};
    bool priming_{true};
};

} // namespace filter

#endif // LIB_FILTER_BUTTERWORTH_LOW_PASS_HPP
