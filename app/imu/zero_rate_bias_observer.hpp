#ifndef IMU_ZERO_RATE_BIAS_OBSERVER_HPP
#define IMU_ZERO_RATE_BIAS_OBSERVER_HPP

#include "utils/math/vector3.hpp"
#include <cstdint>

namespace application
{

/**
 * @brief 零速零偏观测器（ZARU）：静止时把"机体角速度为零"当作观测量，
 *        用一维卡尔曼形式持续修正 **z 轴（yaw 轴）陀螺零偏**。
 *
 *   1. 按固定样本数滚动一个统计窗口（默认 2048 样本，2 kHz 下约 1.02 s），
 *      窗口内累加三轴角速度的均值/方差与加速度模长的均值/方差；
 *   2. 静止判据：窗口均值、窗口标准差、加速度模长及其标准差逐项合格，
 *      且窗口内没有任何超过瞬时上限的样本（撞击/异常样本令整窗作废）；
 *   3. 连续合格静止时长达到 confirm_ms 之前不做任何修正（避免被瞬态骗到）；
 *   4. 满足条件后执行一次零速观测：量测值 = 窗口内的 z
 * 均值（静止时应等于零偏）， 量测方差 =
 * 该均值的方差（由窗口样本方差与样本数换算），
 *      过程噪声按距上次观测的真实经过时间累加，增益与协方差由卡尔曼形式给出；
 *   5. 单次修正量限幅，避免一次被误判的窗口把 trim 拉偏。
 *
 * 与"直接给 gyro[2] 加死区"的区别：死区是把角速度硬切 0（会吃掉真实慢转），
 * 本类只在判据成立时**修正零偏本身**，运动期间只做预测（协方差长大），
 * 因此恢复静止后能自愈，且不会截断真实转动。
 *
 *   - 静止判据的均值门槛必须**小于业务里最小的真实 yaw 速率**，否则慢速真实转动
 *     会被当成静止、把零偏拉偏。默认 0.005 rad/s（约 17 °/min），可按业务收紧。
 *   - 修正只影响以后的积分速率，不会让姿态角跳变；但 trim 变化会让 yaw
 * **角速度** 出现小台阶，所以单次修正限幅是必要的。
 *   - 本类不依赖任何平台设施，tick 频率由 Config 给出，可在主机上确定性测试；
 *     tick 差值按无符号回绕计算，时长用 64 位累计。
 */
class ZeroRateBiasObserver final
{
  public:
    struct Config final
    {
        std::uint32_t tick_frequency_hz{480000000U}; // 与 platform::Time 一致

        // 静止判据
        /** 单样本角速度上限：只用于拒绝撞击/异常样本（整窗作废），不是主要判据。 */
        float maximum_instantaneous_rate_rad_per_sec{1.0F};
        /**
     * z 轴窗口均值上限：拒绝 yaw 真实转动的主要保护，**只作用于被观测的 z
     * 轴**。 默认 0.004 rad/s ≈ 0.229 °/s ≈ 13.7 °/min。
     * 实测依据：本板该轴窗口均值噪声 σ≈0.045 °/s（0.00079 rad/s），
     * 所以 0.229 °/s 约 5.1σ，误拒可忽略（实测拒窗率 0）。
     * 调法：应 ≤ 业务最小真实 yaw 速率的 1/2；再收紧不要低于 ~2σ（0.0016
     * rad/s）， 否则会频繁误拒、反而抑制观测更新。
     */
        float maximum_mean_rate_rad_per_sec{0.004F};
        /**
     * x/y 轴窗口均值上限（宽松）。**不能**把 z 轴那个严格门槛用到这两轴：
     * 板上实测 y 轴零偏就有 0.52 °/s（0.0091 rad/s），比严格门槛还大，
     * 会把所有窗口都拒掉（实测 rejected_windows 持续增长、observations 恒为
     * 0）。 慢速倾斜不影响 z 轴零偏的可观测性，这两轴只需排除"大幅翻滚"，
     * 因此给 0.05 rad/s（约 2.9 °/s）即可。
     */
        float maximum_cross_axis_mean_rate_rad_per_sec{0.05F};
        /** 窗口标准差上限：质量门，挡明显抖动；按实测单样本噪声水平给得较松。 */
        float maximum_standard_deviation_rad_per_sec{0.2F};
        /** 加速度模长与重力的容差；本板实测 |a| 有约 0.10 m/s^2 系统偏差。 */
        float maximum_accel_norm_error_mps2{0.25F};
        float maximum_accel_standard_deviation_mps2{0.3F};
        /** 统计窗口长度（样本数）。2 kHz 下 2048 样本约 1.02 s。 */
        std::uint32_t window_samples{2048U};
        /**
     * 子块长度（样本数）：逐样本只用单精度累加子块，子块满才升到双精度总量。
     * M7 没有双精度 FPU，逐样本双精度加乘是热路径上的主要开销（实测因此多出
     * 约 2/s 的 rejFlags 与 1.8% 的加速度样本损失）；分块后双精度运算量降到
     * 1/64， 而单个子块内的单精度累加误差可忽略。
     */
        std::uint32_t sub_block_samples{64U};
        /** 连续合格静止时长门槛；达到之前不做任何修正。 */
        std::uint32_t confirm_ms{2000U};

        // 零速观测器（一维卡尔曼）
        /**
     * 零偏随机游走方差率 q，单位 (rad/s)^2/s。
     * 与量测方差 R、观测周期 ΔT 一起决定跟踪时间常数 τ ≈ sqrt(R·ΔT/q)。
     * 默认 7e-11：既等价于 R≈(0.0005 rad/s)^2、ΔT=1 s 时 τ≈60 s，
     * 也与板上实测的零偏抖动（数分钟尺度约 0.005 °/s）一致。
     */
        float process_noise_rad_per_sec2_per_sec{7.0e-11F};
        /** 窗口均值方差下限，防止量测方差过小使增益恒等于 1。 */
        float minimum_measure_variance_rad_per_sec2{1.0e-9F};
        /** 协方差上限，防止长时间运动后数值无界增长。 */
        float maximum_variance_rad_per_sec2{1.0e-4F};
        /** 初值协方差，约 (0.5 °/s)^2，使首次合格窗口能快速定位零偏。 */
        float initial_variance_rad_per_sec2{7.6e-5F};
        /** 单次观测允许的最大修正量，约 6.9 °/min。 */
        float maximum_correction_step_rad_per_sec{0.002F};
        /**
     * 是否把估计值用于修正。false 时只统计与发布（monitor-only），
     * 用于先验证判据不会在真实运动中误触发，再开启修正。
     */
        bool apply_correction{true};
    };

    struct Snapshot final
    {
        bool static_qualified{false};       // 当前窗口判据是否合格
        bool correction_applied{false};     // 是否已产生过可用的零偏修正
        float bias_rad_per_sec{0.0F};       // z 轴零偏估计（绝对原始零偏）
        float variance_rad_per_sec2{0.0F};  // 该估计的协方差
        float window_mean_rad_per_sec[3]{}; // 最近一个完整窗口的三轴均值
        float window_std_rad_per_sec[3]{};  // 最近一个完整窗口的三轴标准差
        float accel_norm_mean_mps2{0.0F};
        float accel_norm_std_mps2{0.0F};
        std::uint32_t static_ms{0U};         // 连续合格静止时长
        std::uint32_t qualified_windows{0U}; // 累计合格窗口数
        std::uint32_t observation_count{0U}; // 累计执行的零速观测次数
        std::uint32_t rejected_windows{0U};  // 累计被拒窗口数
    };

    ZeroRateBiasObserver() noexcept = default;

    /** @brief 设置参数并复位。 */
    void init(const Config &config) noexcept;

    /** @brief 复位状态，保留参数。 */
    void reset() noexcept;

    /**
   * @brief 每个陀螺样本调用一次。
   * @param gyro_rad_per_sec 机体系角速度，**未经任何零偏修正**。
   * @param accel_meters_per_sec2 最近一次机体系加速度，用于静止判据。
   * @param tick 采样时刻的高精度计数（允许回绕）。
   */
    void process(const math::Vector3 &gyro_rad_per_sec,
                 const math::Vector3 &accel_meters_per_sec2,
                 std::uint32_t tick) noexcept;

    [[nodiscard]] Snapshot get_snapshot() const noexcept;

    /** @brief 已经产生过至少一次零速观测时返回 true（估计值可用于修正）。 */
    [[nodiscard]] bool has_bias() const noexcept
    {
        return observation_count_ > 0U;
    }

    /**
   * @brief 已完成的统计窗口总数（合格 + 被拒）。
   * @note 只在窗口结束时变化（约 1 Hz），供调用方判断是否需要刷新诊断存档，
   *       避免每拍都拷贝结构体。
   */
    [[nodiscard]] std::uint32_t get_window_count() const noexcept
    {
        return qualified_windows_ + rejected_windows_;
    }

    /** @brief 当前 z 轴零偏估计（绝对原始零偏），单位 rad/s。 */
    [[nodiscard]] float get_bias_rad_per_sec() const noexcept
    {
        return bias_rad_per_sec_;
    }

    /** @brief 是否正在把估计值用于修正。 */
    [[nodiscard]] bool is_correction_applied() const noexcept
    {
        return config_.apply_correction && observation_count_ > 0U;
    }

  private:
    void finish_window() noexcept;
    void apply_zero_rate_observation() noexcept;
    /** @brief 把当前单精度子块累加值并入双精度窗口总量并清零。 */
    void fold_block() noexcept;
    /** @brief 毫秒换算为 64 位 tick 数，不做 32 位饱和。 */
    [[nodiscard]] std::uint64_t ticks_from_ms(std::uint32_t ms) const noexcept;

    Config config_{};
    float bias_rad_per_sec_{0.0F};       // 仅 z 轴零偏，绝对原始零偏
    float variance_{0.0F};               // 零偏估计协方差
    float process_noise_per_tick_{0.0F}; // q / tick 频率，逐样本累加用
    float block_sum_[3]{};               // 当前子块：单精度逐样本累加
    float block_sum_squares_[3]{};
    float block_accel_sum_{0.0F};
    float block_accel_sum_squares_{0.0F};
    double sum_[3]{0.0, 0.0, 0.0}; // 已折叠子块的双精度总量
    double sum_squares_[3]{0.0, 0.0, 0.0};
    double accel_norm_sum_{0.0};
    double accel_norm_sum_squares_{0.0};
    float window_mean_[3]{};
    float window_std_[3]{};
    float accel_norm_mean_{0.0F};
    float accel_norm_std_{0.0F};
    std::uint32_t count_{0U}; // 当前窗口样本数
    std::uint32_t last_tick_{0U};
    std::uint32_t qualified_windows_{0U};
    std::uint32_t observation_count_{0U};
    std::uint32_t rejected_windows_{0U};
    std::uint64_t window_ticks_{0U}; // 当前窗口累计时长
    std::uint64_t static_ticks_{0U}; // 连续合格静止累计时长
    bool window_valid_{true};
    bool static_qualified_{false};
    bool timing_anchored_{false};
};

} // namespace application

#endif
