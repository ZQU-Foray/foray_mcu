#include "lib/alg_controller/pid.hpp"
#include "lib/alg_math/basic_math.hpp"
#include <cmath>

namespace
{

bool is_finite(float value)
{
    return std::isfinite(value);
}

bool is_valid_config(const alg_controller::PID::Config &config)
{
    const bool mode_valid =
        config.mode == alg_controller::PIDMode::POSITION || config.mode == alg_controller::PIDMode::DELTA;
    const bool param_finite = is_finite(config.kp) && is_finite(config.ki) && is_finite(config.kd) &&
                              is_finite(config.kf) && is_finite(config.default_dt) && is_finite(config.maxout) &&
                              is_finite(config.feed_forward_limit) && is_finite(config.integral_limit) &&
                              is_finite(config.dead_zone) && is_finite(config.i_var_a) && is_finite(config.i_var_b) &&
                              is_finite(config.d_lpf_rc) && is_finite(config.out_lpf_rc);
    const bool feed_forward_valid = config.feed_forward_limit >= 0.0f && config.feed_forward_limit <= config.maxout &&
                                    (config.kf == 0.0f || config.feed_forward_limit > 0.0f);

    return mode_valid && param_finite && feed_forward_valid && config.default_dt > 0.0f && config.default_dt < 1.0f &&
           config.maxout >= 0.0f && config.integral_limit >= 0.0f && config.dead_zone >= 0.0f &&
           config.i_var_a >= 0.0f && config.i_var_b >= 0.0f && config.d_lpf_rc >= 0.0f && config.out_lpf_rc >= 0.0f;
}

} // namespace

namespace alg_controller
{

bool PID::init(const Config &config)
{
    cfg_ = Config{};
    initialized_ = false;
    reset();

    if (!is_valid_config(config))
    {
        return false;
    }

    cfg_ = config;
    initialized_ = true;
    return true;
}

void PID::reset()
{
    target_ = 0.0f;
    measure_ = 0.0f;
    last_measure_ = 0.0f;
    last_last_measure_ = 0.0f;
    err_ = 0.0f;
    last_err_ = 0.0f;
    last_last_err_ = 0.0f;
    pout_ = 0.0f;
    iout_ = 0.0f;
    dout_ = 0.0f;
    i_term_ = 0.0f;
    output_ = 0.0f;
    last_output_ = 0.0f;
    last_feed_forward_ = 0.0f;
    last_dout_ = 0.0f;
    measure_history_initialized_ = false;
    dt_ = 0.0f;
}

bool PID::calculate_loop(float measure, float target, float dt)
{
    if (!initialized_ || !is_finite(measure) || !is_finite(target))
    {
        return false;
    }

    const PID backup = *this;
    dt_ = (is_finite(dt) && dt > 0.0f && dt < 1.0f) ? dt : cfg_.default_dt;
    measure_ = measure;
    target_ = target;
    err_ = target - measure;
    const float feed_forward = feed_forward_term(target_);
    if (!measure_history_initialized_)
    {
        last_measure_ = measure_;
        last_last_measure_ = measure_;
        measure_history_initialized_ = true;
    }

    if (cfg_.improve.dead_band && std::fabs(err_) < cfg_.dead_zone)
    {
        output_ = (cfg_.mode == PIDMode::DELTA) ? last_output_ - last_feed_forward_ + feed_forward : feed_forward;
        i_term_ = 0.0f;
        last_last_err_ = last_err_;
        last_last_measure_ = last_measure_;
        last_measure_ = measure_;
        last_output_ = output_;
        last_feed_forward_ = feed_forward;
        last_dout_ = dout_;
        last_err_ = err_;
        return true;
    }

    switch (cfg_.mode)
    {
    case PIDMode::POSITION:
        pout_ = cfg_.kp * err_;
        i_term_ = cfg_.ki * err_ * dt_;

        if (cfg_.improve.trapezoid)
        {
            trapezoid_integral();
        }
        if (cfg_.improve.changing_rate)
        {
            changing_rate_integration();
        }
        if (cfg_.improve.deriv_on_meas)
        {
            derivative_on_measurement();
        }
        else
        {
            dout_ = cfg_.kd * (err_ - last_err_) / dt_;
        }
        if (cfg_.improve.deriv_filter)
        {
            derivative_filter();
        }
        if (cfg_.improve.integral_limit)
        {
            integral_limit();
        }

        iout_ += i_term_;
        if (!cfg_.improve.integral_limit)
        {
            iout_ = alg_math::limit(iout_, -cfg_.integral_limit, cfg_.integral_limit);
        }

        output_ = pout_ + iout_ + dout_ + feed_forward;
        if (cfg_.improve.out_filter)
        {
            output_filter();
        }
        break;

    case PIDMode::DELTA:
        pout_ = cfg_.kp * (err_ - last_err_);
        i_term_ = cfg_.ki * err_ * dt_;

        if (cfg_.improve.changing_rate)
        {
            changing_rate_integration();
        }
        iout_ = i_term_;

        if (cfg_.improve.deriv_on_meas)
        {
            dout_ = cfg_.kd * (2.0f * last_measure_ - measure_ - last_last_measure_) / dt_;
        }
        else
        {
            dout_ = cfg_.kd * (err_ - 2.0f * last_err_ + last_last_err_) / dt_;
        }
        if (cfg_.improve.deriv_filter)
        {
            derivative_filter();
        }

        output_ = last_output_ + pout_ + iout_ + dout_ + feed_forward - last_feed_forward_;
        if (cfg_.improve.out_filter)
        {
            output_filter();
        }
        break;

    default:
        *this = backup;
        return false;
    }

    output_ = alg_math::limit(output_, -cfg_.maxout, cfg_.maxout);
    if (!is_finite(err_) || !is_finite(pout_) || !is_finite(iout_) || !is_finite(dout_) || !is_finite(i_term_) ||
        !is_finite(output_))
    {
        *this = backup;
        return false;
    }

    last_last_err_ = last_err_;
    last_last_measure_ = last_measure_;
    last_measure_ = measure_;
    last_output_ = output_;
    last_feed_forward_ = feed_forward;
    last_dout_ = dout_;
    last_err_ = err_;
    return true;
}

float PID::get_integral_error() const
{
    return iout_;
}

float PID::get_out() const
{
    return output_;
}

const PID::Config &PID::get_config() const
{
    return cfg_;
}

bool PID::is_initialized() const
{
    return initialized_;
}

bool PID::set_param(float kp, float ki, float kd)
{
    if (!initialized_ || !is_finite(kp) || !is_finite(ki) || !is_finite(kd))
    {
        return false;
    }
    cfg_.kp = kp;
    cfg_.ki = ki;
    cfg_.kd = kd;
    return true;
}

bool PID::set_kp(float kp)
{
    return set_param(kp, cfg_.ki, cfg_.kd);
}

bool PID::set_ki(float ki)
{
    return set_param(cfg_.kp, ki, cfg_.kd);
}

bool PID::set_kd(float kd)
{
    return set_param(cfg_.kp, cfg_.ki, kd);
}

bool PID::set_kf(float kf)
{
    if (!initialized_ || !is_finite(kf))
    {
        return false;
    }
    cfg_.kf = kf;
    return true;
}

bool PID::set_i_out_max(float integral_limit)
{
    if (!initialized_ || !is_finite(integral_limit) || integral_limit < 0.0f)
    {
        return false;
    }
    cfg_.integral_limit = integral_limit;
    iout_ = alg_math::limit(iout_, -integral_limit, integral_limit);
    return true;
}

bool PID::set_out_max(float maxout)
{
    if (!initialized_ || !is_finite(maxout) || maxout < 0.0f)
    {
        return false;
    }
    cfg_.maxout = maxout;
    output_ = alg_math::limit(output_, -maxout, maxout);
    last_output_ = alg_math::limit(last_output_, -maxout, maxout);
    return true;
}

bool PID::set_i_var_a(float i_var_a)
{
    if (!initialized_ || !is_finite(i_var_a) || i_var_a < 0.0f)
    {
        return false;
    }
    cfg_.i_var_a = i_var_a;
    return true;
}

bool PID::set_i_var_b(float i_var_b)
{
    if (!initialized_ || !is_finite(i_var_b) || i_var_b < 0.0f)
    {
        return false;
    }
    cfg_.i_var_b = i_var_b;
    return true;
}

bool PID::set_target(float target)
{
    if (!initialized_ || !is_finite(target))
    {
        return false;
    }
    target_ = target;
    return true;
}

bool PID::set_now(float measure)
{
    if (!initialized_ || !is_finite(measure))
    {
        return false;
    }
    measure_ = measure;
    last_measure_ = measure;
    last_last_measure_ = measure;
    measure_history_initialized_ = true;
    return true;
}

bool PID::set_integral_error(float iout)
{
    if (!initialized_ || !is_finite(iout))
    {
        return false;
    }
    iout_ = alg_math::limit(iout, -cfg_.integral_limit, cfg_.integral_limit);
    return true;
}

float PID::feed_forward_term(float target) const
{
    return alg_math::limit(cfg_.kf * target, -cfg_.feed_forward_limit, cfg_.feed_forward_limit);
}

void PID::trapezoid_integral()
{
    i_term_ = cfg_.ki * ((err_ + last_err_) * 0.5f) * dt_;
}

void PID::changing_rate_integration()
{
    if (err_ * iout_ <= 0.0f)
    {
        return;
    }

    const float abs_err = std::fabs(err_);
    if (abs_err <= cfg_.i_var_b)
    {
        return;
    }
    if (cfg_.i_var_a > 0.0f && abs_err <= cfg_.i_var_a + cfg_.i_var_b)
    {
        i_term_ *= (cfg_.i_var_a - abs_err + cfg_.i_var_b) / cfg_.i_var_a;
    }
    else
    {
        i_term_ = 0.0f;
    }
}

void PID::integral_limit()
{
    const float temp_iout = iout_ + i_term_;
    const float temp_output = pout_ + temp_iout + dout_ + feed_forward_term(target_);

    if (std::fabs(temp_output) > cfg_.maxout && err_ * temp_iout > 0.0f)
    {
        i_term_ = 0.0f;
    }

    if (temp_iout > cfg_.integral_limit)
    {
        i_term_ = 0.0f;
        iout_ = cfg_.integral_limit;
    }
    else if (temp_iout < -cfg_.integral_limit)
    {
        i_term_ = 0.0f;
        iout_ = -cfg_.integral_limit;
    }
}

void PID::derivative_on_measurement()
{
    dout_ = cfg_.kd * (last_measure_ - measure_) / dt_;
}

void PID::derivative_filter()
{
    const float alpha = dt_ / (cfg_.d_lpf_rc + dt_);
    dout_ = alpha * dout_ + (1.0f - alpha) * last_dout_;
}

void PID::output_filter()
{
    const float alpha = dt_ / (cfg_.out_lpf_rc + dt_);
    output_ = alpha * output_ + (1.0f - alpha) * last_output_;
}

} // namespace alg_controller
