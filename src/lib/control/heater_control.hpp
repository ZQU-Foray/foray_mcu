#ifndef LIB_CONTROL_HEATER_CONTROL_HPP
#define LIB_CONTROL_HEATER_CONTROL_HPP

#include <array>
#include <cstddef>
#include <cstdint>

namespace control
{

/**
 * @brief 加热恒温控制（Decision-20261101-05）：用于"加热器 → 被加热区 → 更新慢、有量化的温度传感器"，
 *        如 BMI088 的温度寄存器（1.28 s 更新一次、0.125 ℃）。
 *
 * 模型（u 为占空比，单位 ‰）：
 *   被加热区  dx1/dt = (K·u + Ta − x1) / τ1
 *   传感器    dx2/dt = (x1 − x2) / τ2
 * 读数是 x2 在传感器更新时刻的截断采样。加热器很强、读数很慢时，只看读数的反馈看到温度时已经冲过了，
 * 所以预热按模型预测提前收功率。
 *
 * 过程：
 *   1. 第一个读数到来时认为加热器之前关着、各处温度相同（x1 = x2 = Ta = 读数）。板子热着时复位，这样会把环境温度
 *      估高；之后两处修正它（Decision-20261103-02）：读数比"加热器一直关着"的预测还低时按差值下调；恒温时积分
 *      长期偏大时慢慢转进环境估计。
 *   2. 探测级：用预热功率加热，每个周期预测"之后保持在本级目标所需功率时传感器的最高温度"，到本级目标就收功率，
 *      最后一个周期按插值只补到恰好。本级目标只走剩余温差的一部分（probe_fraction），加热器比模型强时也不越过目标。
 *   3. 收功率后等几次读数更新，温度停在平台上；实际温升 / 预测温升就是加热器强度的修正系数（供电电压、个体差异）。
 *   4. 逼近级：用修正后的模型瞄准目标下方 approach_margin_c。
 *   5. 恒温：维持功率（模型前馈）+ 比例（模型估计的传感器温度）+ 积分（读数）。
 *   读数低于目标 rewarm_now_below_c 以上（如加热电源断开又恢复）时按当前读数重新预热；低于 rewarm_below_c 以上时
 *   要持续 rewarm_delay_s 且读数不升才重新预热。
 *
 * 只做计算，不碰硬件：输入目标、读数、读数距今的时间与周期，输出占空比。
 */
class HeaterControl final
{
  public:
    /** 热模型（每块板实测辨识）。 */
    struct Model
    {
        float gain_c_per_permille;    // 稳态增益：每 1‰ 占空比，被加热区比环境高多少 ℃
        float zone_time_constant_s;   // 被加热区的时间常数 τ1
        float sensor_time_constant_s; // 传感器滞后于被加热区的时间常数 τ2
    };

    /** 温度传感器（由驱动给出）。 */
    struct Sensor
    {
        float period_s;     // 读数的更新周期
        float resolution_c; // 读数分辨率（截断，估计时加半个分辨率）
        float read_lag_s; // 看到读数变化时，距传感器真正更新的平均滞后（驱动读取周期的一半）
    };

    /** 控制参数。默认值在加热器强度 ±25%、时间常数 ±10–25% 的仿真下整定（Decision-20261101-05）。 */
    struct Tuning
    {
        float boost_permille{300.0F};             // 预热功率
        float hold_max_permille{100.0F};          // 恒温功率上限
        float probe_fraction{0.7F};               // 探测级走剩余温差的比例
        float approach_margin_c{0.8F};            // 逼近级瞄准目标下方多少
        float stage_gap_min_c{0.5F};              // 剩余温差小于它就直接恒温
        std::uint8_t probe_settle_updates{4U};    // 探测级收功率后等几次读数更新（平台）
        std::uint8_t approach_settle_updates{3U}; // 逼近级收功率后等几次读数更新
        std::uint8_t max_stages{3U};              // 一次预热最多几级（探测 + 逼近）
        float calibration_rise_min_c{3.0F};       // 探测级预测温升至少这么大才修正加热器强度
        float kp_permille_per_c{16.0F};           // 恒温比例
        float ki_permille_per_c_s{1.0F};          // 恒温积分
        float integral_limit_permille{30.0F};     // 积分限幅
        float observer_zone_gain{0.2F};           // 读数误差修正被加热区估计的比例
        float observer_sensor_gain{0.8F};         // 读数误差修正传感器估计的比例
        float rewarm_below_c{2.0F}; // 恒温时读数低于目标这么多（持续 rewarm_delay_s、读数不升）就重新预热
        float rewarm_now_below_c{4.0F}; // 低于目标这么多立即重新预热
        float rewarm_delay_s{2.0F};     // 见 rewarm_below_c
        float horizon_s{15.0F};         // 预测峰值的最长时间
        float ambient_margin_c{0.25F}; // 读数比"加热器一直关着"的预测低这么多以上，才下调环境温度估计
        float integral_offload_time_s{10.0F}; // 恒温积分转成环境修正的时间常数
    };

    struct Config
    {
        Model model;
        Sensor sensor;
        Tuning tuning;
    };

    enum class Phase : std::uint8_t
    {
        IDLE,            // 还没有读数
        PROBE,           // 探测级预热
        PROBE_SETTLE,    // 探测级收功率，等平台
        APPROACH,        // 逼近级预热
        APPROACH_SETTLE, // 逼近级收功率，等到位
        REGULATE         // 恒温
    };

    [[nodiscard]] bool init(const Config &config) noexcept;

    /** 回到 IDLE：下一次 update() 按读数重新开始（加热器强度修正系数也重置）。 */
    void reset() noexcept;

    /**
     * @brief 每个控制周期调用一次。
     * @param target_c 目标温度
     * @param reading_c 最近一次读数
     * @param reading_age_s 读数最近一次变化距今的时间（从看到变化的那次读取算起）
     * @param dt_s 距上次调用的时间，(0, 1] s
     * @return 到下次调用之间的占空比（‰），未初始化或参数无效时为 0
     */
    [[nodiscard]] float update(float target_c, float reading_c, float reading_age_s, float dt_s) noexcept;

    [[nodiscard]] Phase phase() const noexcept
    {
        return phase_;
    }
    /** 加热器强度修正系数（实际 / 模型）。 */
    [[nodiscard]] float gain_scale() const noexcept
    {
        return gain_scale_;
    }
    [[nodiscard]] float zone_estimate_c() const noexcept
    {
        return zone_c_;
    }
    [[nodiscard]] float sensor_estimate_c() const noexcept
    {
        return sensor_c_;
    }
    /** 环境温度估计（第一次读数起步，之后按 Decision-20261103-02 修正）。 */
    [[nodiscard]] float ambient_estimate_c() const noexcept
    {
        return ambient_c_;
    }
    [[nodiscard]] float integral_permille() const noexcept
    {
        return integral_;
    }
    [[nodiscard]] float output_permille() const noexcept
    {
        return output_;
    }

  private:
    static constexpr std::size_t HISTORY_SIZE{32U};

    struct HistoryEntry
    {
        float sensor_c; // 传感器估计
        float dt_s;     // 距上一条的时间
    };

    void start(float reading_c) noexcept;
    void start_stage(float target_c) noexcept;
    void advance(float duty_permille, float dt_s) noexcept;
    void push_history(float dt_s) noexcept;
    [[nodiscard]] float sensor_at_age(float age_s) const noexcept;
    void correct(float reading_c, float age_s) noexcept;
    [[nodiscard]] float hold_duty(float level_c) const noexcept;
    [[nodiscard]] float peak_if(float duty_now, float dt_s, float level_c) const noexcept;
    [[nodiscard]] float stage_duty(float dt_s) noexcept;

    Config config_{};
    bool initialized_{false};
    Phase phase_{Phase::IDLE};

    // 模型状态
    float zone_c_{0.0F};
    float sensor_c_{0.0F};
    float ambient_c_{0.0F};
    float gain_scale_{1.0F};

    // 读数与传感器更新相位
    float last_reading_c_{0.0F};
    std::uint32_t update_count_{0U};
    std::array<HistoryEntry, HISTORY_SIZE> history_{};
    std::size_t history_head_{0U};
    std::size_t history_count_{0U};

    // 分级预热
    bool stage_pending_{false};     // 下一步要开始新的一级（或转入恒温）
    bool has_anchor_{false};        // 已由读数跳变锁定传感器更新相位
    float next_update_age_s_{0.0F}; // 下一次待计入的传感器更新距今的时间（负数表示还没到）
    std::uint8_t stages_{0U};
    float stage_base_c_{0.0F};
    float stage_goal_c_{0.0F};
    std::uint32_t stage_end_count_{0U};

    // 恒温
    float integral_{0.0F};
    float below_s_{0.0F};       // 恒温时低于 rewarm_below_c 已持续多久（读数在升时重新计）
    float below_start_c_{0.0F}; // 开始计时时的读数
    float unheated_zone_c_{0.0F}; // "加热器一直关着"的模型：从上次读数更新（或预热开始）的读数出发
    float unheated_sensor_c_{0.0F};
    float output_{0.0F};
};

} // namespace control

#endif // LIB_CONTROL_HEATER_CONTROL_HPP
