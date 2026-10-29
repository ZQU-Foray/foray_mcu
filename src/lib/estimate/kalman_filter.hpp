#ifndef LIB_ESTIMATE_KALMAN_FILTER_HPP
#define LIB_ESTIMATE_KALMAN_FILTER_HPP

#include "utils/math/matrix.hpp"
#include <cstddef>
#include <cstdint>

namespace estimate
{

/**
 * @brief 引擎内部矩阵运算状态。
 */
enum class KalmanMatrixStatus : std::int8_t
{
    SUCCESS = 0,
    SINGULAR = -1,
    NOT_FINITE = -2
};

/**
 * @brief 通用卡尔曼滤波引擎：黄金五式 + 用户步骤替换 + 单步跳过。
 *
 * 标准步骤（既可整步跳过，也可由用户函数替换任意一步）：
 *   0. 取量测：把量测输入搬进 z 并清空输入；
 *   1. 先验状态：x̂'(k) = F·x̂(k-1)，控制维数不为零时再加 B·u；
 *   2. 先验协方差：P'(k) = F·P(k-1)·Fᵀ + Q；
 *   3. 增益：K(k) = P'(k)·Hᵀ·(H·P'(k)·Hᵀ + R)⁻¹；
 *   4. 后验状态：x̂(k) = x̂'(k) + K(k)·(z(k) − H·x̂'(k))；
 *   5. 后验协方差：P(k) = P'(k) − K(k)·H·P'(k)。
 * 收尾把 P 的对角线抬到状态方差下限，并把 x̂ 搬到 filtered_value。
 *
 * 用户步骤函数的调用点与标准步骤的编号一一对应：0 在取量测之后、1 在步骤 1 之后、
 * 2 在步骤 2 之后、3 在步骤 3 之后、4 在步骤 4 之后、5 在步骤 5 之后、6 在收尾之后。
 *
 * 与通用实现的关键差异：全部矩阵容量在编译期确定，不使用堆分配；量测自动调整在固定
 * 容量内用零填充实现，被剔除量测对应的 H 行、R 行列与 z 元素恰好为零，因此结果与
 * "动态缩小维数"的实现等价。数据成员公开，是为了让用户步骤函数能够直接改写
 * F/H/Q/R 等矩阵，这正是本引擎的扩展机制。
 *
 * @tparam StateDim 状态维数，必须大于零。
 * @tparam MeasureDim 量测维数容量，必须大于零。
 * @tparam ControlDim 控制维数，取 0 表示没有控制输入。
 */
template <std::size_t StateDim, std::size_t MeasureDim, std::size_t ControlDim = 0U>
class KalmanFilter final
{
    static_assert(StateDim > 0U, "状态维数必须大于零");
    static_assert(MeasureDim > 0U, "量测维数必须大于零");

  public:
    static constexpr std::size_t STATE_SIZE{StateDim};
    static constexpr std::size_t MEASURE_SIZE{MeasureDim};
    static constexpr std::size_t CONTROL_SIZE{ControlDim};
    static constexpr std::size_t STEP_COUNT{5U};
    static constexpr std::size_t USER_STEP_COUNT{7U};
    // 控制维数为零时仍然保留 1 维的存储，避免出现零长度矩阵类型；此时不参与任何运算。
    static constexpr std::size_t CONTROL_STORAGE{ControlDim > 0U ? ControlDim : 1U};

    using StateVector = math::Matrix<StateDim, 1U>;
    using StateMatrix = math::Matrix<StateDim, StateDim>;
    using MeasureVector = math::Matrix<MeasureDim, 1U>;
    using MeasureMatrix = math::Matrix<MeasureDim, MeasureDim>;
    using MeasurementMatrix = math::Matrix<MeasureDim, StateDim>;
    using GainMatrix = math::Matrix<StateDim, MeasureDim>;
    using ControlVector = math::Matrix<CONTROL_STORAGE, 1U>;
    using ControlInputMatrix = math::Matrix<StateDim, CONTROL_STORAGE>;

    /**
     * @brief 用户步骤函数指针，context 在注册时给出。
     * @note 不使用 std::function，避免动态分配与额外的间接分发开销。
     */
    using UserStepFunction = void (*)(KalmanFilter &filter, void *context);

    /**
     * @brief 自动量测调整的映射表。state_index 从 1 开始，与该机制的前身一致。
     */
    struct MeasurementLayout final
    {
        std::uint8_t state_index[MeasureDim]{};
        float degree[MeasureDim]{};
        float variance[MeasureDim]{};
    };

    KalmanFilter() noexcept
    {
        reset();
    }

    /**
     * @brief 复位为确定初值：矩阵清零、F 设为单位矩阵、跳过标志与用户函数清空。
     * @note 初值状态、初值协方差、Q、R 由调用方在复位后写入。
     */
    void reset() noexcept
    {
        state = StateVector{};
        predicted_state = StateVector{};
        filtered_value = StateVector{};
        covariance = StateMatrix{};
        predicted_covariance = StateMatrix{};
        transition = StateMatrix::identity();
        process_noise = StateMatrix{};
        measurement = MeasurementMatrix{};
        measurement_noise = MeasureMatrix{};
        innovation_covariance = MeasureMatrix{};
        innovation_inverse = MeasureMatrix{};
        gain = GainMatrix{};
        measurement_input = MeasureVector{};
        measurement_vector = MeasureVector{};
        control_input = ControlVector{};
        control = ControlVector{};
        control_matrix = ControlInputMatrix{};
        minimum_variance = StateVector{};
        layout_ = MeasurementLayout{};
        valid_measurement_count = 0U;
        auto_measurement_adjustment = false;
        for (std::size_t index = 0U; index < STEP_COUNT; ++index)
        {
            skip_step_[index] = false;
        }
        for (std::size_t index = 0U; index < USER_STEP_COUNT; ++index)
        {
            user_steps_[index] = nullptr;
            user_contexts_[index] = nullptr;
        }
        status_ = KalmanMatrixStatus::SUCCESS;
    }

    /** @brief 注册用户步骤，索引 0 至 6 与标准流程中的调用点一一对应。 */
    void set_user_step(std::size_t index, UserStepFunction function, void *context) noexcept
    {
        if (index < USER_STEP_COUNT)
        {
            user_steps_[index] = function;
            user_contexts_[index] = context;
        }
    }

    /**
     * @brief 跳过某个标准步骤。
     * @param index 步骤编号 1 至 5：先验状态、先验协方差、增益、后验状态、后验协方差。
     */
    void set_skip_step(std::size_t index, bool skip) noexcept
    {
        if (index >= 1U && index <= STEP_COUNT)
        {
            skip_step_[index - 1U] = skip;
        }
    }

    [[nodiscard]] bool is_skip_step(std::size_t index) const noexcept
    {
        return (index >= 1U && index <= STEP_COUNT) ? skip_step_[index - 1U] : false;
    }

    /**
     * @brief 开启自动量测调整：量测输入中为零的分量视为无效并被剔除。
     */
    void enable_auto_measurement_adjustment(const MeasurementLayout &measurement_layout) noexcept
    {
        layout_ = measurement_layout;
        auto_measurement_adjustment = true;
    }

    void disable_auto_measurement_adjustment() noexcept
    {
        auto_measurement_adjustment = false;
    }

    /** @brief 本次更新出现过的最严重矩阵运算状态。 */
    [[nodiscard]] KalmanMatrixStatus get_status() const noexcept
    {
        return status_;
    }

    /** @brief 有效量测个数：关闭自动调整时等于量测维数，开启时等于非零量测个数。 */
    [[nodiscard]] std::uint8_t get_valid_measurement_count() const noexcept
    {
        return valid_measurement_count;
    }

    /**
     * @brief 执行一次完整的滤波更新，结果位于 state 与 filtered_value。
     * @note 量测输入需要在调用前写入 measurement_input；调用后该输入被清空，
     *       因此每一轮都需要重新写入，这与该引擎前身的一次性量测语义一致。
     */
    void update() noexcept
    {
        status_ = KalmanMatrixStatus::SUCCESS;
        take_measurement();
        call_user_step(0U);

        if (!skip_step_[0])
        {
            predict_state();
        }
        call_user_step(1U);

        if (!skip_step_[1])
        {
            predict_covariance();
        }
        call_user_step(2U);

        if (valid_measurement_count != 0U || !auto_measurement_adjustment)
        {
            if (!skip_step_[2])
            {
                compute_gain();
            }
            call_user_step(3U);

            if (!skip_step_[3])
            {
                correct_state();
            }
            call_user_step(4U);

            if (!skip_step_[4])
            {
                update_covariance();
            }
        }
        else
        {
            // 没有有效量测：只做预测，不虚构修正。
            state = predicted_state;
            covariance = predicted_covariance;
        }

        call_user_step(5U);

        for (std::size_t index = 0U; index < StateDim; ++index)
        {
            if (covariance.at(index, index) < minimum_variance.at(index, 0U))
            {
                covariance.at(index, index) = minimum_variance.at(index, 0U);
            }
        }

        filtered_value = state;
        call_user_step(6U);
    }

    // 状态与矩阵：调用方初始化并读写，用户步骤函数直接改写这里的矩阵
    StateVector state{};                   // x̂(k|k)
    StateVector predicted_state{};         // x̂(k|k-1)
    StateVector filtered_value{};          // 本轮更新后的 x̂
    StateMatrix covariance{};              // P(k|k)
    StateMatrix predicted_covariance{};    // P(k|k-1)
    StateMatrix transition{};              // F
    StateMatrix process_noise{};           // Q
    MeasurementMatrix measurement{};       // H
    MeasureMatrix measurement_noise{};     // R
    MeasureMatrix innovation_covariance{}; // S = H·P'·Hᵀ + R
    MeasureMatrix innovation_inverse{};    // S⁻¹，供用户步骤（如卡方检验）复用
    GainMatrix gain{};                     // K
    MeasureVector measurement_input{};     // 调用方写入的量测，取量测后清空
    MeasureVector measurement_vector{};    // z
    ControlVector control_input{};         // 调用方写入的控制量
    ControlVector control{};               // u，每轮从 control_input 复制
    ControlInputMatrix control_matrix{};   // B
    StateVector minimum_variance{};        // 状态方差下限，全零表示不限制
    std::uint8_t valid_measurement_count{0U};
    bool auto_measurement_adjustment{false};

  private:
    void call_user_step(std::size_t index) noexcept
    {
        if (index < USER_STEP_COUNT && user_steps_[index] != nullptr)
        {
            user_steps_[index](*this, user_contexts_[index]);
        }
    }

    void take_measurement() noexcept
    {
        if (auto_measurement_adjustment)
        {
            adjust_measurement();
        }
        else
        {
            measurement_vector = measurement_input;
            valid_measurement_count = static_cast<std::uint8_t>(MeasureDim);
        }
        measurement_input = MeasureVector{};
        if constexpr (ControlDim > 0U)
        {
            control = control_input;
        }
    }

    /**
     * @brief 自动量测调整：剔除为零的量测，并把 z、H、R 依次压缩到固定容量的前若干行/列。
     * @note 被剔除量测的 H 行为零、z 为零，因此对状态与协方差没有贡献；其 R 对角元
     *       保留为 1 而不是 0，以保证零填充后的新息协方差仍然可逆（否则该量测对应
     *       的块会让 S 奇异）。
     */
    void adjust_measurement() noexcept
    {
        measurement_vector = MeasureVector{};
        measurement = MeasurementMatrix{};
        measurement_noise = MeasureMatrix::identity();

        std::size_t slot = 0U;
        for (std::size_t index = 0U; index < MeasureDim; ++index)
        {
            const float value = measurement_input.at(index, 0U);
            if (value == 0.0f)
            {
                continue;
            }
            measurement_vector.at(slot, 0U) = value;
            measurement_noise.at(slot, slot) = layout_.variance[index];
            const std::uint8_t state_index = layout_.state_index[index];
            if (state_index >= 1U && state_index <= StateDim)
            {
                measurement.at(slot, state_index - 1U) = layout_.degree[index];
            }
            ++slot;
        }
        valid_measurement_count = static_cast<std::uint8_t>(slot);
        measurement_input = MeasureVector{};
    }

    void predict_state() noexcept
    {
        predicted_state = StateVector::template multiply<StateDim>(transition, state);
        if constexpr (ControlDim > 0U)
        {
            predicted_state =
                predicted_state.added(StateVector::template multiply<ControlDim>(control_matrix, control));
        }
    }

    void predict_covariance() noexcept
    {
        const StateMatrix intermediate = StateMatrix::template multiply<StateDim>(transition, covariance);
        predicted_covariance =
            StateMatrix::template multiply<StateDim>(intermediate, transition.transposed()).added(process_noise);
    }

    void compute_gain() noexcept
    {
        const MeasurementMatrix intermediate =
            MeasurementMatrix::template multiply<StateDim>(measurement, predicted_covariance);
        innovation_covariance =
            MeasureMatrix::template multiply<StateDim>(intermediate, measurement.transposed()).added(measurement_noise);

        MeasureMatrix inverse{};
        if (!MeasureMatrix::inverted(innovation_covariance, inverse))
        {
            record_status(KalmanMatrixStatus::SINGULAR);
            innovation_inverse = MeasureMatrix{};
            gain = GainMatrix{};
            return;
        }
        innovation_inverse = inverse;

        const GainMatrix numerator =
            GainMatrix::template multiply<StateDim>(predicted_covariance, measurement.transposed());
        gain = GainMatrix::template multiply<MeasureDim>(numerator, inverse);
        if (!gain.is_finite())
        {
            record_status(KalmanMatrixStatus::NOT_FINITE);
            gain = GainMatrix{};
        }
    }

    void correct_state() noexcept
    {
        const MeasureVector prediction = MeasureVector::template multiply<StateDim>(measurement, predicted_state);
        const MeasureVector residual = measurement_vector.subtracted(prediction);
        state = predicted_state.added(StateVector::template multiply<MeasureDim>(gain, residual));
        if (!state.is_finite())
        {
            record_status(KalmanMatrixStatus::NOT_FINITE);
            state = predicted_state;
        }
    }

    void update_covariance() noexcept
    {
        const StateMatrix gain_measurement = StateMatrix::template multiply<MeasureDim>(gain, measurement);
        covariance = predicted_covariance.subtracted(
            StateMatrix::template multiply<StateDim>(gain_measurement, predicted_covariance));
    }

    /** @brief 记录最严重的矩阵运算状态：成功不覆盖已记录的失败。 */
    void record_status(KalmanMatrixStatus value) noexcept
    {
        if (value != KalmanMatrixStatus::SUCCESS)
        {
            status_ = value;
        }
    }

    MeasurementLayout layout_{};
    UserStepFunction user_steps_[USER_STEP_COUNT]{};
    void *user_contexts_[USER_STEP_COUNT]{};
    bool skip_step_[STEP_COUNT]{};
    KalmanMatrixStatus status_{KalmanMatrixStatus::SUCCESS};
};

} // namespace estimate

#endif
