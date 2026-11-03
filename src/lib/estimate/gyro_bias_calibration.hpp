#ifndef LIB_ESTIMATE_GYRO_BIAS_CALIBRATION_HPP
#define LIB_ESTIMATE_GYRO_BIAS_CALIBRATION_HPP

#include "utils/math/vector3.hpp"

#include <cstdint>

namespace estimate
{

/**
 * @brief 上电静止零偏标定：恒温到位并稳定后，用一段静止窗口估计三轴陀螺零偏。
 *
 * 由迁移源的 application::GyroBiasCalibration（tag archive/pre-rebuild，app/imu）改写：时间改为 µs 时间戳，
 * 累加改为 float（规则 14：没有双精度 FPU 的板也要能跑；1 万个样本的舍入误差约 4e-8 rad/s，远小于
 * 零偏估计本身约 7e-4 rad/s 的不确定度），门控与参数不变。
 *
 * - 温度门控：零偏随温度变化，恒温目标到位才开窗；温度离开容差就清空窗口重新计时。
 * - 温度稳定门控：进带后还要把带内极差压在 temperature_stability_celsius 以内并保持 temperature_settle_us，
 *   升温、过冲瞬态里采到的零偏与稳态可差 0.008 °/s（约 0.5 °/min），是静态漂移的主项。
 * - 静止门控：加速度模长接近重力；窗口内任一轴的极差超限（被碰、振动）或部分均值超限（缓慢转动）都判为运动，
 *   窗口重新开始。
 * - 时间上限：累计处于采样状态超过 timeout_us 仍没有合格窗口，放弃并置为超时，调用方按未标定运行。
 *   预算按累计采样时间算，窗口重启不重置，开机后被搬动几秒不会花掉整个预算。
 * 只做估计，不修改输入，也不负责把零偏用到样本上。
 */
class GyroBiasCalibration final
{
  public:
    enum class State : std::uint8_t
    {
        WAITING_FOR_TEMPERATURE = 0U, // 恒温未到位或未稳定
        SAMPLING = 1U,                // 正在积累静止窗口
        COMPLETED = 2U,               // 已完成，零偏有效
        TIMED_OUT = 3U                // 超时放弃，零偏无效
    };

    struct Config
    {
        float target_celsius{50.0F}; // 与加热目标相同
        float temperature_tolerance_celsius{0.5F};
        // 温度稳定门控。旧板实测：温度量化步长 0.125 ℃，稳定平台的 20 s 滚动极差 0.125–0.25 ℃
        std::uint32_t temperature_settle_us{20000000U};
        float temperature_stability_celsius{0.25F};
        // 静止门控（旧板实测冻结，不要凭直觉收紧）：
        //   - 均值上限 0.05 rad/s：BMI088 零偏随板与温度可到 0.5 °/s 量级；
        //   - 极差上限 1.0 rad/s：静止时窗内极差实测 25–31 °/s，主要是传感器噪声（单样本 σ 约 4 °/s）；
        //   - 加速度模长容差 0.5 m/s²：|a| 实测 9.706 m/s²，对水平加速度只二阶敏感。
        float maximum_mean_rate_rad_per_sec{0.05F};
        float maximum_spread_rad_per_sec{1.0F};
        float maximum_accel_norm_error_mps2{0.5F};
        std::uint32_t window_us{5000000U};   // 静止窗口长度
        std::uint32_t timeout_us{20000000U}; // 累计处于采样状态的时间上限
    };

    struct Snapshot
    {
        State state{State::WAITING_FOR_TEMPERATURE};
        math::Vector3 bias_rad_per_sec{};      // 标定结果
        math::Vector3 spread_rad_per_sec{};    // 窗口内每轴极差
        std::uint32_t samples{0U};             // 当前窗口已积累的样本
        std::uint32_t restarts{0U};            // 窗口因运动、温度重启的次数
        std::uint32_t sampling_us{0U};         // 累计处于采样状态的时间
        std::uint32_t temperature_hold_us{0U}; // 带内温度连续稳定的时长
        float temperature_celsius{0.0F};
        bool bias_valid{false};
    };

    /** 设置参数并复位。参数无效（周期为 0、容差为负或非有限值）时返回 false。 */
    [[nodiscard]] bool init(const Config &config) noexcept;

    /** 复位到等待温度，保留参数。 */
    void reset() noexcept;

    /**
     * @brief 每个陀螺样本调用一次。
     * @param gyro_rad_per_sec 角速度，未扣零偏。
     * @param accel_meters_per_sec2 最近一次加速度，用于静止门控。
     * @param temperature_celsius 最近一次温度；没有有效读数时传 NaN。
     * @param timestamp_us 采样时刻（µs，按 2^32 回绕）。第一个样本只建立时基。
     */
    void process(const math::Vector3 &gyro_rad_per_sec,
                 const math::Vector3 &accel_meters_per_sec2,
                 float temperature_celsius,
                 std::uint32_t timestamp_us) noexcept;

    [[nodiscard]] Snapshot snapshot() const noexcept;

    [[nodiscard]] State state() const noexcept
    {
        return state_;
    }

    [[nodiscard]] bool is_bias_valid() const noexcept
    {
        return bias_valid_;
    }

    [[nodiscard]] const math::Vector3 &bias_rad_per_sec() const noexcept
    {
        return bias_;
    }

  private:
    static constexpr std::uint32_t PARTIAL_CHECK_INTERVAL{512U};

    void restart_window() noexcept;
    void accumulate(const math::Vector3 &gyro_rad_per_sec, std::uint32_t delta_us) noexcept;
    void finalize() noexcept;
    [[nodiscard]] bool mean_within_limit(const math::Vector3 &mean) const noexcept;

    Config config_{};
    State state_{State::WAITING_FOR_TEMPERATURE};
    math::Vector3 bias_{};
    math::Vector3 spread_{};
    math::Vector3 sum_{};
    math::Vector3 minimum_{};
    math::Vector3 maximum_{};
    std::uint32_t count_{0U};
    std::uint32_t last_timestamp_us_{0U};
    std::uint32_t restarts_{0U};
    std::uint64_t sampling_us_{0U}; // 采样状态累计，用于超时预算
    std::uint64_t window_us_{0U};   // 当前窗口累计
    std::uint64_t hold_us_{0U};     // 温度连续稳定的累计
    float hold_minimum_{0.0F};
    float hold_maximum_{0.0F};
    float temperature_celsius_{0.0F};
    bool hold_active_{false};
    bool timing_anchored_{false}; // 第一个样本只建立时基，不产生时间增量
    bool bias_valid_{false};
};

} // namespace estimate

#endif // LIB_ESTIMATE_GYRO_BIAS_CALIBRATION_HPP
