#ifndef APPLICATION_IMU_GYRO_BIAS_CALIBRATION_HPP
#define APPLICATION_IMU_GYRO_BIAS_CALIBRATION_HPP

#include "Libraries/Algorithm/alg_math/Vector3.h"
#include <cstdint>

namespace application
{

/**
 * @brief 上电静止零偏标定：等恒温到位后，用一段静止窗口估计三轴陀螺零偏。
 *
 *   温度门控：零偏随温度变化，因此必须等恒温目标到位才开窗；温度离开容差
 *   会清空当前窗口重新计时。
 *   温度稳定门控：进带还不够，还要把带内极差压在 temperatureStabilityCelsius
 *   以内并保持 temperatureSettleMs 才允许开窗——升温/过冲瞬态里采到的零偏
 *   与稳态可以差 0.008 °/s（≈0.5 °/min），这是静态漂移的主项。
 *   静止门控：加速度范数必须接近重力；窗口内任一轴的极差超过上限（被碰、
 *   振动）或部分均值超过上限（缓慢转动）都判定为运动，窗口重新开始。
 *   时间上限：累计处于采样状态超过 timeoutMs 仍未得到合格窗口，则放弃并
 *   置为超时状态，此时调用方保持零偏不修正（退化到未标定行为），不阻塞业务。
 *   预算按累计采样时间算，窗口重启不重置，因此"开机后被搬动几秒"不会
 *   花掉整个预算。
 *   本类只做"估计"，不修改任何输入，也不负责把零偏作用到样本上。
 */
class GyroBiasCalibration final
{
public:
  /** @brief 标定状态，取值写入调试器存档，顺序不可随意调整。 */
  enum class State : std::uint32_t
  {
    WaitingForTemperature = 0U, // 恒温未到位
    Sampling = 1U,              // 正在积累静止窗口
    Completed = 2U,             // 已完成，零偏有效
    TimedOut = 3U               // 超时放弃，零偏无效
  };

  struct Config final
  {
    std::uint32_t tickFrequencyHz{480000000U}; // 与 platform::Time 一致
    float targetCelsius{50.0F};                // 与恒温目标一致，且必须等于
                                               // Bmi088Heater::DefaultTargetCelsius
    float temperatureToleranceCelsius{0.5F};
    // 温度稳定门控：进带后还要在 temperatureSettleMs 内把极差压在
    // temperatureStabilityCelsius 以内才开窗。实测：本板温度量化步长 0.125 ℃，
    // 升温瞬态 0.2 ℃/min 时 20 s 滚动极差中位 0.25 ℃，稳定平台约 0.125~0.25 ℃，
    // 因此 0.25 ℃ 能挡住升温尾巴又不会把稳态挡掉。
    std::uint32_t temperatureSettleMs{20000U};
    float temperatureStabilityCelsius{0.25F};
    // 静止门控阈值已按板上实测冻结，不要凭直觉收紧：
    //   - 均值上限 0.05 rad/s（2.9 °/s）：本板 y 轴零偏实测 -0.52 °/s，BMI088
    //     零偏随板/温度可到 0.5 °/s 量级，收紧会拒掉合法窗口；
    //   - 极差上限 1.0 rad/s（57 °/s）：静止时窗内极差实测 25~31 °/s，主要来自
    //     传感器自身高频噪声（单样本 σ≈4 °/s）而非运动。
    //   - 加速度模长容差 0.5 m/s^2：本板 |a| 实测 9.706
    //   m/s^2（比标称低 1.0%），
    //     本就有 0.10 m/s^2 系统偏差，而且 |a| 对水平加速度是二阶不敏感
    //     （1 m/s^2 水平加速度只改变 0.05 m/s^2），收紧它几乎不增加选窗能力。
    float maximumMeanRateRadPerSec{0.05F};
    float maximumSpreadRadPerSec{1.0F};
    float maximumAccelNormErrorMps2{0.5F};
    std::uint32_t windowMs{5000U};   // 静止窗口长度
    std::uint32_t timeoutMs{20000U}; // 累计处于采样状态的时间上限（非墙钟）
  };

  struct Snapshot final
  {
    State state{State::WaitingForTemperature};
    alg_math::Vector3 biasRadPerSec{};   // 标定结果
    alg_math::Vector3 spreadRadPerSec{}; // 窗口内每轴极差
    std::uint32_t samples{0U};           // 当前窗口已积累样本数
    std::uint32_t restarts{0U};          // 窗口因运动/温度被重启的次数
    std::uint32_t elapsedMs{0U};         // 累计处于采样状态的时间
    std::uint32_t temperatureHoldMs{0U}; // 带内温度连续稳定保持的时长
    float temperatureCelsius{0.0F};
    bool biasValid{false};
  };

  GyroBiasCalibration() noexcept = default;

  /** @brief 设置参数并复位状态。 */
  void Init(const Config &config) noexcept;

  /** @brief 复位到等待温度状态，保留参数。 */
  void Reset() noexcept;

  /**
   * @brief 每个陀螺样本调用一次。
   * @param gyroRadPerSec 机体系角速度，未经任何零偏修正。
   * @param accelMetersPerSec2 最近一次机体系加速度，用于静止门控。
   * @param temperatureCelsius 最近一次温度读数；无有效读数时传 NaN。
   * @param tick 采样时刻的高精度计数（允许回绕）。
   */
  void Process(const alg_math::Vector3 &gyroRadPerSec,
               const alg_math::Vector3 &accelMetersPerSec2,
               float temperatureCelsius,
               std::uint32_t tick) noexcept;

  [[nodiscard]] Snapshot GetSnapshot() const noexcept;

  /** @brief 标定完成且结果通过检验时返回 true。 */
  [[nodiscard]] bool IsBiasValid() const noexcept
  {
    return biasValid_;
  }

  [[nodiscard]] const alg_math::Vector3 &GetBiasRadPerSec() const noexcept
  {
    return bias_;
  }

private:
  void RestartWindow() noexcept;
  void Accumulate(const alg_math::Vector3 &gyroRadPerSec, std::uint32_t deltaTicks) noexcept;
  void Finalize() noexcept;
  /** @brief 毫秒换算为 64 位 tick 数，不做 32 位饱和。 */
  [[nodiscard]] std::uint64_t TicksFromMs(std::uint32_t milliseconds) const noexcept;

  static constexpr std::uint32_t PartialCheckInterval{512U};

  Config config_{};
  State state_{State::WaitingForTemperature};
  alg_math::Vector3 bias_{};
  alg_math::Vector3 spread_{};
  double sumX_{0.0};
  double sumY_{0.0};
  double sumZ_{0.0};
  alg_math::Vector3 minimum_{};
  alg_math::Vector3 maximum_{};
  std::uint32_t count_{0U};
  std::uint32_t lastTick_{0U};
  std::uint32_t restarts_{0U};
  std::uint64_t samplingTicks_{0U}; // 采样状态累计，用于超时预算
  std::uint64_t windowTicks_{0U};   // 当前窗口累计
  std::uint64_t holdTicks_{0U};     // 温度连续稳定保持累计
  float holdMinimum_{0.0F};
  float holdMaximum_{0.0F};
  float temperatureCelsius_{0.0F};
  bool holdActive_{false};
  bool timingAnchored_{false}; // 首个样本只建立时基，不产生时间增量
  bool biasValid_{false};
};

} // namespace application

#endif
