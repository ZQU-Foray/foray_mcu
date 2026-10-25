#ifndef LIBRARIES_ALGORITHM_ALG_ESTIMATE_KALMAN_FILTER_HPP
#define LIBRARIES_ALGORITHM_ALG_ESTIMATE_KALMAN_FILTER_HPP

#include "Libraries/Algorithm/alg_math/Matrix.h"
#include <cstddef>
#include <cstdint>

namespace alg_estimate
{

/**
 * @brief 引擎内部矩阵运算状态。
 */
enum class KalmanMatrixStatus : std::int8_t
{
  Success = 0,
  Singular = -1,
  NotFinite = -2
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
 * 收尾把 P 的对角线抬到状态方差下限，并把 x̂ 搬到 filteredValue。
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
template <std::size_t StateDim, std::size_t MeasureDim, std::size_t ControlDim = 0U> class KalmanFilter final
{
  static_assert(StateDim > 0U, "状态维数必须大于零");
  static_assert(MeasureDim > 0U, "量测维数必须大于零");

public:
  static constexpr std::size_t StateSize{StateDim};
  static constexpr std::size_t MeasureSize{MeasureDim};
  static constexpr std::size_t ControlSize{ControlDim};
  static constexpr std::size_t StepCount{5U};
  static constexpr std::size_t UserStepCount{7U};
  // 控制维数为零时仍然保留 1 维的存储，避免出现零长度矩阵类型；此时不参与任何运算。
  static constexpr std::size_t ControlStorage{ControlDim > 0U ? ControlDim : 1U};

  using StateVector = alg_math::Matrix<StateDim, 1U>;
  using StateMatrix = alg_math::Matrix<StateDim, StateDim>;
  using MeasureVector = alg_math::Matrix<MeasureDim, 1U>;
  using MeasureMatrix = alg_math::Matrix<MeasureDim, MeasureDim>;
  using MeasurementMatrix = alg_math::Matrix<MeasureDim, StateDim>;
  using GainMatrix = alg_math::Matrix<StateDim, MeasureDim>;
  using ControlVector = alg_math::Matrix<ControlStorage, 1U>;
  using ControlInputMatrix = alg_math::Matrix<StateDim, ControlStorage>;

  /**
     * @brief 用户步骤函数指针，context 在注册时给出。
     * @note 不使用 std::function，避免动态分配与额外的间接分发开销。
     */
  using UserStepFunction = void (*)(KalmanFilter &filter, void *context);

  /**
     * @brief 自动量测调整的映射表。stateIndex 从 1 开始，与该机制的前身一致。
     */
  struct MeasurementLayout final
  {
    std::uint8_t stateIndex[MeasureDim]{};
    float degree[MeasureDim]{};
    float variance[MeasureDim]{};
  };

  KalmanFilter() noexcept
  {
    Reset();
  }

  /**
     * @brief 复位为确定初值：矩阵清零、F 设为单位矩阵、跳过标志与用户函数清空。
     * @note 初值状态、初值协方差、Q、R 由调用方在复位后写入。
     */
  void Reset() noexcept
  {
    state = StateVector{};
    predictedState = StateVector{};
    filteredValue = StateVector{};
    covariance = StateMatrix{};
    predictedCovariance = StateMatrix{};
    transition = StateMatrix::Identity();
    processNoise = StateMatrix{};
    measurement = MeasurementMatrix{};
    measurementNoise = MeasureMatrix{};
    innovationCovariance = MeasureMatrix{};
    innovationInverse = MeasureMatrix{};
    gain = GainMatrix{};
    measurementInput = MeasureVector{};
    measurementVector = MeasureVector{};
    controlInput = ControlVector{};
    control = ControlVector{};
    controlMatrix = ControlInputMatrix{};
    minimumVariance = StateVector{};
    layout = MeasurementLayout{};
    validMeasurementCount = 0U;
    autoMeasurementAdjustment = false;
    for (std::size_t index = 0U; index < StepCount; ++index)
    {
      skipStep[index] = false;
    }
    for (std::size_t index = 0U; index < UserStepCount; ++index)
    {
      userSteps[index] = nullptr;
      userContexts[index] = nullptr;
    }
    status = KalmanMatrixStatus::Success;
  }

  /** @brief 注册用户步骤，索引 0 至 6 与标准流程中的调用点一一对应。 */
  void SetUserStep(std::size_t index, UserStepFunction function, void *context) noexcept
  {
    if (index < UserStepCount)
    {
      userSteps[index] = function;
      userContexts[index] = context;
    }
  }

  /**
     * @brief 跳过某个标准步骤。
     * @param index 步骤编号 1 至 5：先验状态、先验协方差、增益、后验状态、后验协方差。
     */
  void SetSkipStep(std::size_t index, bool skip) noexcept
  {
    if (index >= 1U && index <= StepCount)
    {
      skipStep[index - 1U] = skip;
    }
  }

  [[nodiscard]] bool IsSkipStep(std::size_t index) const noexcept
  {
    return (index >= 1U && index <= StepCount) ? skipStep[index - 1U] : false;
  }

  /**
     * @brief 开启自动量测调整：量测输入中为零的分量视为无效并被剔除。
     */
  void EnableAutoMeasurementAdjustment(const MeasurementLayout &measurementLayout) noexcept
  {
    layout = measurementLayout;
    autoMeasurementAdjustment = true;
  }

  void DisableAutoMeasurementAdjustment() noexcept
  {
    autoMeasurementAdjustment = false;
  }

  /** @brief 本次更新出现过的最严重矩阵运算状态。 */
  [[nodiscard]] KalmanMatrixStatus GetStatus() const noexcept
  {
    return status;
  }

  /** @brief 有效量测个数：关闭自动调整时等于量测维数，开启时等于非零量测个数。 */
  [[nodiscard]] std::uint8_t GetValidMeasurementCount() const noexcept
  {
    return validMeasurementCount;
  }

  /**
     * @brief 执行一次完整的滤波更新，结果位于 state 与 filteredValue。
     * @note 量测输入需要在调用前写入 measurementInput；调用后该输入被清空，
     *       因此每一轮都需要重新写入，这与该引擎前身的一次性量测语义一致。
     */
  void Update() noexcept
  {
    status = KalmanMatrixStatus::Success;
    TakeMeasurement();
    CallUserStep(0U);

    if (!skipStep[0])
    {
      PredictState();
    }
    CallUserStep(1U);

    if (!skipStep[1])
    {
      PredictCovariance();
    }
    CallUserStep(2U);

    if (validMeasurementCount != 0U || !autoMeasurementAdjustment)
    {
      if (!skipStep[2])
      {
        ComputeGain();
      }
      CallUserStep(3U);

      if (!skipStep[3])
      {
        CorrectState();
      }
      CallUserStep(4U);

      if (!skipStep[4])
      {
        UpdateCovariance();
      }
    }
    else
    {
      // 没有有效量测：只做预测，不虚构修正。
      state = predictedState;
      covariance = predictedCovariance;
    }

    CallUserStep(5U);

    for (std::size_t index = 0U; index < StateDim; ++index)
    {
      if (covariance.At(index, index) < minimumVariance.At(index, 0U))
      {
        covariance.At(index, index) = minimumVariance.At(index, 0U);
      }
    }

    filteredValue = state;
    CallUserStep(6U);
  }

  // 状态与矩阵：调用方初始化并读写，用户步骤函数直接改写这里的矩阵
  StateVector state{};                  // x̂(k|k)
  StateVector predictedState{};         // x̂(k|k-1)
  StateVector filteredValue{};          // 本轮更新后的 x̂
  StateMatrix covariance{};             // P(k|k)
  StateMatrix predictedCovariance{};    // P(k|k-1)
  StateMatrix transition{};             // F
  StateMatrix processNoise{};           // Q
  MeasurementMatrix measurement{};      // H
  MeasureMatrix measurementNoise{};     // R
  MeasureMatrix innovationCovariance{}; // S = H·P'·Hᵀ + R
  MeasureMatrix innovationInverse{};    // S⁻¹，供用户步骤（如卡方检验）复用
  GainMatrix gain{};                    // K
  MeasureVector measurementInput{};     // 调用方写入的量测，取量测后清空
  MeasureVector measurementVector{};    // z
  ControlVector controlInput{};         // 调用方写入的控制量
  ControlVector control{};              // u，每轮从 controlInput 复制
  ControlInputMatrix controlMatrix{};   // B
  StateVector minimumVariance{};        // 状态方差下限，全零表示不限制
  std::uint8_t validMeasurementCount{0U};
  bool autoMeasurementAdjustment{false};

private:
  void CallUserStep(std::size_t index) noexcept
  {
    if (index < UserStepCount && userSteps[index] != nullptr)
    {
      userSteps[index](*this, userContexts[index]);
    }
  }

  void TakeMeasurement() noexcept
  {
    if (autoMeasurementAdjustment)
    {
      AdjustMeasurement();
    }
    else
    {
      measurementVector = measurementInput;
      validMeasurementCount = static_cast<std::uint8_t>(MeasureDim);
    }
    measurementInput = MeasureVector{};
    if constexpr (ControlDim > 0U)
    {
      control = controlInput;
    }
  }

  /**
     * @brief 自动量测调整：剔除为零的量测，并把 z、H、R 依次压缩到固定容量的前若干行/列。
     * @note 被剔除量测的 H 行为零、z 为零，因此对状态与协方差没有贡献；其 R 对角元
     *       保留为 1 而不是 0，以保证零填充后的新息协方差仍然可逆（否则该量测对应
     *       的块会让 S 奇异）。
     */
  void AdjustMeasurement() noexcept
  {
    measurementVector = MeasureVector{};
    measurement = MeasurementMatrix{};
    measurementNoise = MeasureMatrix::Identity();

    std::size_t slot = 0U;
    for (std::size_t index = 0U; index < MeasureDim; ++index)
    {
      const float value = measurementInput.At(index, 0U);
      if (value == 0.0f)
      {
        continue;
      }
      measurementVector.At(slot, 0U) = value;
      measurementNoise.At(slot, slot) = layout.variance[index];
      const std::uint8_t stateIndex = layout.stateIndex[index];
      if (stateIndex >= 1U && stateIndex <= StateDim)
      {
        measurement.At(slot, stateIndex - 1U) = layout.degree[index];
      }
      ++slot;
    }
    validMeasurementCount = static_cast<std::uint8_t>(slot);
    measurementInput = MeasureVector{};
  }

  void PredictState() noexcept
  {
    predictedState = StateVector::template Multiply<StateDim>(transition, state);
    if constexpr (ControlDim > 0U)
    {
      predictedState = predictedState.Added(StateVector::template Multiply<ControlDim>(controlMatrix, control));
    }
  }

  void PredictCovariance() noexcept
  {
    const StateMatrix intermediate = StateMatrix::template Multiply<StateDim>(transition, covariance);
    predictedCovariance =
        StateMatrix::template Multiply<StateDim>(intermediate, transition.Transposed()).Added(processNoise);
  }

  void ComputeGain() noexcept
  {
    const MeasurementMatrix intermediate =
        MeasurementMatrix::template Multiply<StateDim>(measurement, predictedCovariance);
    innovationCovariance =
        MeasureMatrix::template Multiply<StateDim>(intermediate, measurement.Transposed()).Added(measurementNoise);

    MeasureMatrix inverse{};
    if (!MeasureMatrix::Inverted(innovationCovariance, inverse))
    {
      RecordStatus(KalmanMatrixStatus::Singular);
      innovationInverse = MeasureMatrix{};
      gain = GainMatrix{};
      return;
    }
    innovationInverse = inverse;

    const GainMatrix numerator = GainMatrix::template Multiply<StateDim>(predictedCovariance, measurement.Transposed());
    gain = GainMatrix::template Multiply<MeasureDim>(numerator, inverse);
    if (!gain.IsFinite())
    {
      RecordStatus(KalmanMatrixStatus::NotFinite);
      gain = GainMatrix{};
    }
  }

  void CorrectState() noexcept
  {
    const MeasureVector prediction = MeasureVector::template Multiply<StateDim>(measurement, predictedState);
    const MeasureVector residual = measurementVector.Subtracted(prediction);
    state = predictedState.Added(StateVector::template Multiply<MeasureDim>(gain, residual));
    if (!state.IsFinite())
    {
      RecordStatus(KalmanMatrixStatus::NotFinite);
      state = predictedState;
    }
  }

  void UpdateCovariance() noexcept
  {
    const StateMatrix gainMeasurement = StateMatrix::template Multiply<MeasureDim>(gain, measurement);
    covariance =
        predictedCovariance.Subtracted(StateMatrix::template Multiply<StateDim>(gainMeasurement, predictedCovariance));
  }

  /** @brief 记录最严重的矩阵运算状态：成功不覆盖已记录的失败。 */
  void RecordStatus(KalmanMatrixStatus value) noexcept
  {
    if (value != KalmanMatrixStatus::Success)
    {
      status = value;
    }
  }

  MeasurementLayout layout{};
  UserStepFunction userSteps[UserStepCount]{};
  void *userContexts[UserStepCount]{};
  bool skipStep[StepCount]{};
  KalmanMatrixStatus status{KalmanMatrixStatus::Success};
};

} // namespace alg_estimate

#endif
