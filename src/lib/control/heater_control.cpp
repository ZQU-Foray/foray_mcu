#include "lib/control/heater_control.hpp"

#include <cmath>

namespace control
{

namespace
{

constexpr float MAX_DUTY_PERMILLE{1000.0F};
constexpr float MIN_GAIN_SCALE{0.4F};
constexpr float MAX_GAIN_SCALE{2.5F};
constexpr float MIN_GAIN_CORRECTION{0.5F}; // 一次平台校正最多改到原来的 0.5–2 倍；更小视为加热器没电
constexpr float MAX_GAIN_CORRECTION{2.0F};
constexpr float OFFLOAD_INTEGRAL_FRACTION{0.5F}; // 积分超过限幅的这个比例才转成环境修正

constexpr float clamp(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

bool positive(float value)
{
    return std::isfinite(value) && value > 0.0F;
}

} // namespace

bool HeaterControl::init(const Config &config) noexcept
{
    const Model &model = config.model;
    const Sensor &sensor = config.sensor;
    const Tuning &tuning = config.tuning;
    initialized_ =
        positive(model.gain_c_per_permille) && positive(model.zone_time_constant_s) &&
        positive(model.sensor_time_constant_s) && positive(sensor.period_s) && positive(sensor.resolution_c) &&
        std::isfinite(sensor.read_lag_s) && sensor.read_lag_s >= 0.0F && positive(tuning.boost_permille) &&
        tuning.boost_permille <= MAX_DUTY_PERMILLE && positive(tuning.hold_max_permille) &&
        tuning.hold_max_permille <= tuning.boost_permille && tuning.probe_fraction > 0.0F &&
        tuning.probe_fraction < 1.0F && tuning.approach_margin_c >= 0.0F && positive(tuning.stage_gap_min_c) &&
        tuning.probe_settle_updates > 0U && tuning.approach_settle_updates > 0U && tuning.max_stages > 0U &&
        tuning.kp_permille_per_c >= 0.0F && tuning.ki_permille_per_c_s >= 0.0F &&
        tuning.integral_limit_permille >= 0.0F && tuning.observer_zone_gain >= 0.0F &&
        tuning.observer_sensor_gain >= 0.0F && tuning.observer_sensor_gain <= 1.0F && positive(tuning.rewarm_below_c) &&
        positive(tuning.horizon_s) && positive(tuning.ambient_margin_c) && positive(tuning.integral_offload_time_s) &&
        std::isfinite(tuning.rewarm_delay_s) && tuning.rewarm_delay_s >= 0.0F &&
        std::isfinite(tuning.rewarm_now_below_c) && tuning.rewarm_now_below_c >= tuning.rewarm_below_c;
    config_ = config;
    reset();
    return initialized_;
}

void HeaterControl::reset() noexcept
{
    phase_ = Phase::IDLE;
    gain_scale_ = 1.0F;
    integral_ = 0.0F;
    output_ = 0.0F;
    below_s_ = 0.0F;
    stage_pending_ = false;
    has_anchor_ = false;
    stages_ = 0U;
    history_count_ = 0U;
}

float HeaterControl::update(float target_c, float reading_c, float reading_age_s, float dt_s) noexcept
{
    if (!initialized_ || !std::isfinite(target_c) || !std::isfinite(reading_c) || !std::isfinite(reading_age_s) ||
        !(dt_s > 0.0F) || dt_s > 1.0F)
    {
        output_ = 0.0F;
        return output_;
    }
    const Sensor &sensor = config_.sensor;
    const Tuning &tuning = config_.tuning;
    const float measured = reading_c + sensor.resolution_c * 0.5F; // 读数是截断的，估计时取区间中点

    if (phase_ == Phase::IDLE)
    {
        ambient_c_ = measured; // 第一个读数：认为加热器之前关着、各处温度相同
        start(measured);
        last_reading_c_ = reading_c;
    }
    else if (has_anchor_)
    {
        next_update_age_s_ += dt_s;
    }

    // 传感器更新相位：读数跳变的那次读取之前约 read_lag 是一次更新，之后每 period 一次
    if (std::fabs(reading_c - last_reading_c_) > sensor.resolution_c * 0.25F)
    {
        last_reading_c_ = reading_c;
        has_anchor_ = true;
        next_update_age_s_ = reading_age_s + sensor.read_lag_s;
    }
    if (has_anchor_)
    {
        while (next_update_age_s_ >= sensor.read_lag_s)
        {
            correct(measured, next_update_age_s_);
            next_update_age_s_ -= sensor.period_s;
            ++update_count_;
        }
    }

    // 恒温时温度掉得太多（如加热电源断开又恢复）：按当前读数重新预热，积分不在没电时累积。
    // 低于目标 rewarm_now_below_c 以上立即重新预热；只低 rewarm_below_c 到 rewarm_now_below_c 时，要持续
    // rewarm_delay_s 且读数没有上升才重新预热：环境温度估高时恒温刚开始会差 2 ℃ 左右，有电的话比例项会把温度
    // 拉上来，立即重新预热会清掉积分、一直到不了目标（Decision-20261103-02）
    const float gap = target_c - measured;
    if (phase_ == Phase::REGULATE && gap > tuning.rewarm_below_c)
    {
        if (below_s_ == 0.0F || measured > below_start_c_ + sensor.resolution_c * 0.5F)
        {
            below_s_ = 0.0F; // 刚低于门限，或读数在升：重新计时
            below_start_c_ = measured;
        }
        below_s_ += dt_s;
        if (gap > tuning.rewarm_now_below_c || below_s_ >= tuning.rewarm_delay_s)
        {
            start(measured);
        }
    }
    else
    {
        below_s_ = 0.0F;
    }
    if (stage_pending_)
    {
        start_stage(target_c);
    }

    float duty = 0.0F;
    switch (phase_)
    {
    case Phase::PROBE:
    case Phase::APPROACH:
        duty = stage_duty(dt_s);
        break;
    case Phase::PROBE_SETTLE:
    case Phase::APPROACH_SETTLE:
    {
        duty = hold_duty(stage_goal_c_);
        const bool probe = phase_ == Phase::PROBE_SETTLE;
        const std::uint32_t needed = probe ? tuning.probe_settle_updates : tuning.approach_settle_updates;
        if (update_count_ - stage_end_count_ >= needed)
        {
            // 温度停在平台上：实际温升 / 预测温升 = 加热器强度的修正系数
            const float predicted = stage_goal_c_ - stage_base_c_;
            if (probe && predicted >= tuning.calibration_rise_min_c)
            {
                // 温升不到预测的一半：多半是加热器没电（如 24 V 断开），这次不修正，免得来电后功率翻倍
                const float ratio = (measured - stage_base_c_) / predicted;
                if (ratio >= MIN_GAIN_CORRECTION)
                {
                    gain_scale_ = clamp(gain_scale_ * clamp(ratio, MIN_GAIN_CORRECTION, MAX_GAIN_CORRECTION),
                                        MIN_GAIN_SCALE,
                                        MAX_GAIN_SCALE);
                }
            }
            zone_c_ = measured;
            sensor_c_ = measured;
            history_count_ = 0U;
            push_history(0.0F);
            stage_pending_ = true;
        }
        break;
    }
    case Phase::REGULATE:
        integral_ = clamp(integral_ + tuning.ki_permille_per_c_s * (target_c - measured) * dt_s,
                          -tuning.integral_limit_permille,
                          tuning.integral_limit_permille);
        if (std::fabs(gap) < tuning.rewarm_below_c &&
            std::fabs(integral_) > OFFLOAD_INTEGRAL_FRACTION * tuning.integral_limit_permille)
        {
            // 积分长期偏大说明维持功率前馈不对（多半是环境温度估错了）：慢慢转成对环境估计的修正，
            // 前馈与模型预测随之变对，比例项也不再和积分对着干（Decision-20261103-02）
            const float moved = integral_ * dt_s / tuning.integral_offload_time_s;
            integral_ -= moved;
            ambient_c_ -= config_.model.gain_c_per_permille * gain_scale_ * moved;
        }
        duty = hold_duty(target_c) + tuning.kp_permille_per_c * (target_c - sensor_c_) + integral_;
        duty = clamp(duty, 0.0F, tuning.hold_max_permille);
        break;
    case Phase::IDLE:
        break;
    }

    advance(duty, dt_s);
    push_history(dt_s);
    output_ = duty;
    return output_;
}

void HeaterControl::start(float measured_c) noexcept
{
    zone_c_ = measured_c;
    sensor_c_ = measured_c;
    unheated_zone_c_ = measured_c;
    unheated_sensor_c_ = measured_c;
    below_s_ = 0.0F;
    history_count_ = 0U;
    push_history(0.0F);
    stages_ = 0U;
    integral_ = 0.0F;
    stage_pending_ = true;
    phase_ = Phase::PROBE; // 由 start_stage() 定下实际阶段
}

void HeaterControl::start_stage(float target_c) noexcept
{
    stage_pending_ = false;
    const Tuning &tuning = config_.tuning;
    const float gap = target_c - sensor_c_;
    if (gap > tuning.stage_gap_min_c && stages_ < tuning.max_stages)
    {
        // 第一级只走一部分（探测加热器强度），之后直接瞄准目标下方
        const float step = stages_ == 0U ? tuning.probe_fraction * gap : gap - tuning.approach_margin_c;
        stage_base_c_ = sensor_c_;
        stage_goal_c_ = sensor_c_ + (step > 0.0F ? step : 0.0F);
        phase_ = stages_ == 0U ? Phase::PROBE : Phase::APPROACH;
        ++stages_;
        return;
    }
    phase_ = Phase::REGULATE;
    integral_ = 0.0F;
}

float HeaterControl::stage_duty(float dt_s) noexcept
{
    const float boost = config_.tuning.boost_permille;
    const float full = peak_if(boost, dt_s, stage_goal_c_);
    if (full <= stage_goal_c_)
    {
        return boost;
    }
    // 最后一个预热周期：在维持功率与预热功率之间插值，使预测峰值恰好到本级目标
    const float hold = hold_duty(stage_goal_c_);
    const float low = peak_if(hold, dt_s, stage_goal_c_);
    float duty = hold;
    if (low < stage_goal_c_ && full > low)
    {
        duty = hold + (boost - hold) * (stage_goal_c_ - low) / (full - low);
    }
    phase_ = phase_ == Phase::PROBE ? Phase::PROBE_SETTLE : Phase::APPROACH_SETTLE;
    stage_end_count_ = update_count_;
    return duty;
}

float HeaterControl::hold_duty(float level_c) const noexcept
{
    const float gain = config_.model.gain_c_per_permille * gain_scale_;
    return clamp((level_c - ambient_c_) / gain, 0.0F, config_.tuning.hold_max_permille);
}

float HeaterControl::peak_if(float duty_now, float dt_s, float level_c) const noexcept
{
    const Model &model = config_.model;
    const float gain = model.gain_c_per_permille * gain_scale_;
    const float hold = hold_duty(level_c);
    float zone = zone_c_;
    float sensor = sensor_c_;
    zone += (gain * duty_now + ambient_c_ - zone) / model.zone_time_constant_s * dt_s;
    sensor += (zone - sensor) / model.sensor_time_constant_s * dt_s;
    float peak = sensor;
    const auto steps = static_cast<std::uint32_t>(config_.tuning.horizon_s / dt_s);
    for (std::uint32_t step = 0U; step < steps; ++step)
    {
        zone += (gain * hold + ambient_c_ - zone) / model.zone_time_constant_s * dt_s;
        sensor += (zone - sensor) / model.sensor_time_constant_s * dt_s;
        if (sensor < peak)
        {
            break; // 过了峰值
        }
        peak = sensor;
    }
    return peak;
}

void HeaterControl::advance(float duty_permille, float dt_s) noexcept
{
    const Model &model = config_.model;
    const float gain = model.gain_c_per_permille * gain_scale_;
    zone_c_ += (gain * duty_permille + ambient_c_ - zone_c_) / model.zone_time_constant_s * dt_s;
    sensor_c_ += (zone_c_ - sensor_c_) / model.sensor_time_constant_s * dt_s;
    unheated_zone_c_ += (ambient_c_ - unheated_zone_c_) / model.zone_time_constant_s * dt_s;
    unheated_sensor_c_ += (unheated_zone_c_ - unheated_sensor_c_) / model.sensor_time_constant_s * dt_s;
}

void HeaterControl::push_history(float dt_s) noexcept
{
    history_head_ = (history_head_ + 1U) % HISTORY_SIZE;
    history_[history_head_] = HistoryEntry{sensor_c_, dt_s};
    if (history_count_ < HISTORY_SIZE)
    {
        ++history_count_;
    }
}

float HeaterControl::sensor_at_age(float age_s) const noexcept
{
    // 从最新一条往回找，相邻两条之间线性插值；超出记录范围时取最旧的一条
    std::size_t index = history_head_;
    float newer_age = 0.0F;
    float newer_value = history_[index].sensor_c;
    if (age_s <= 0.0F || history_count_ <= 1U)
    {
        return newer_value;
    }
    for (std::size_t count = 1U; count < history_count_; ++count)
    {
        const float span = history_[index].dt_s;
        const std::size_t older = (index + HISTORY_SIZE - 1U) % HISTORY_SIZE;
        const float older_age = newer_age + span;
        const float older_value = history_[older].sensor_c;
        if (age_s <= older_age && span > 0.0F)
        {
            return newer_value + (older_value - newer_value) * (age_s - newer_age) / span;
        }
        index = older;
        newer_age = older_age;
        newer_value = older_value;
    }
    return newer_value;
}

void HeaterControl::correct(float measured_c, float age_s) noexcept
{
    const float error = measured_c - sensor_at_age(age_s);
    zone_c_ += config_.tuning.observer_zone_gain * error;
    sensor_c_ += config_.tuning.observer_sensor_gain * error;

    // 读数比"加热器一直关着"的预测还低：就算加热器没电也降不到这么低，只能是环境温度估高了（如板子热着时复位）。
    // 按差值下调环境估计，再把这个模型拉回读数，下次只比较一个更新周期内的下降。读数落在"有电"与"没电"之间时
    // 分不清是不是断电，不动环境估计（Decision-20261103-02）
    const float excess = unheated_sensor_c_ - measured_c - config_.tuning.ambient_margin_c;
    if (excess > 0.0F)
    {
        ambient_c_ -= excess;
    }
    unheated_zone_c_ = measured_c;
    unheated_sensor_c_ = measured_c;
}

} // namespace control
