#include "lib/control/slope.hpp"
#include <cmath>

namespace
{

bool is_finite(float value)
{
    return std::isfinite(value);
}

float move_towards(float current, float target, float max_delta)
{
    const float delta = target - current;

    if (std::fabs(delta) <= max_delta)
    {
        return target;
    }

    return current + ((delta > 0.0f) ? max_delta : -max_delta);
}

bool is_between(float value, float first, float second)
{
    return (first <= value && value <= second) || (second <= value && value <= first);
}

} // namespace

namespace control
{

bool Slope::init(const Config &config)
{
    cfg_ = Config{};
    output_ = 0.0f;
    now_real_ = 0.0f;
    target_ = 0.0f;
    initialized_ = false;

    const bool first_valid = config.first == SlopeFirst::REAL || config.first == SlopeFirst::TARGET;
    if (!first_valid || !is_finite(config.increase_rate) || !is_finite(config.decrease_rate) ||
        config.increase_rate <= 0.0f || config.decrease_rate <= 0.0f)
    {
        return false;
    }

    cfg_ = config;
    initialized_ = true;
    return true;
}

void Slope::reset()
{
    output_ = 0.0f;
    now_real_ = 0.0f;
    target_ = 0.0f;
}

bool Slope::reset(float output)
{
    if (!is_finite(output))
    {
        return false;
    }

    output_ = output;
    now_real_ = output;
    target_ = output;
    return true;
}

bool Slope::set_now_real(float now_real)
{
    if (!initialized_ || !is_finite(now_real))
    {
        return false;
    }

    now_real_ = now_real;
    return true;
}

bool Slope::set_increase_rate(float increase_rate)
{
    if (!initialized_ || !is_finite(increase_rate) || increase_rate <= 0.0f)
    {
        return false;
    }

    cfg_.increase_rate = increase_rate;
    return true;
}

bool Slope::set_decrease_rate(float decrease_rate)
{
    if (!initialized_ || !is_finite(decrease_rate) || decrease_rate <= 0.0f)
    {
        return false;
    }

    cfg_.decrease_rate = decrease_rate;
    return true;
}

bool Slope::calculate_loop(float target, float dt)
{
    if (!initialized_ || !is_finite(target) || !is_finite(dt) || dt <= 0.0f)
    {
        return false;
    }

    target_ = target;

    if (cfg_.first == SlopeFirst::REAL && is_between(now_real_, output_, target_))
    {
        output_ = now_real_;
    }

    if (output_ == target_)
    {
        return true;
    }

    const bool direction_reversed = (output_ > 0.0f && target_ < 0.0f) || (output_ < 0.0f && target_ > 0.0f);
    if (direction_reversed)
    {
        const float time_to_zero = std::fabs(output_) / cfg_.decrease_rate;

        if (dt <= time_to_zero)
        {
            output_ = move_towards(output_, 0.0f, cfg_.decrease_rate * dt);
            return true;
        }

        const float remaining_time = dt - time_to_zero;
        output_ = move_towards(0.0f, target_, cfg_.increase_rate * remaining_time);
        return true;
    }

    const bool magnitude_increasing = std::fabs(target_) > std::fabs(output_);
    const float rate = magnitude_increasing ? cfg_.increase_rate : cfg_.decrease_rate;
    output_ = move_towards(output_, target_, rate * dt);

    return true;
}

float Slope::get_out() const
{
    return output_;
}

float Slope::get_target() const
{
    return target_;
}

const Slope::Config &Slope::get_config() const
{
    return cfg_;
}

bool Slope::is_initialized() const
{
    return initialized_;
}

} // namespace control
