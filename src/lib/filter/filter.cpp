#include "lib/filter/filter.hpp"
#include <cmath>
#include <limits>

namespace filter
{
namespace
{
constexpr float PI = 3.14159265358979f;
constexpr float TWO_PI = 6.28318530718f;
constexpr float SQRT2 = 1.41421356237f;

bool is_valid_cutoff(float cutoff_freq, float dt)
{
    return std::isfinite(cutoff_freq) && std::isfinite(dt) && cutoff_freq > 0.0f && dt > 0.0f &&
           cutoff_freq * dt < 0.5f;
}
} // namespace

bool ButterworthFilter::init(const Config &config)
{
    cfg_ = Config{};
    for (float &param : param_)
    {
        param = 0.0f;
    }
    initialized_ = false;
    reset();

    if (!is_valid_cutoff(config.cutoff_freq, config.dt))
    {
        return false;
    }

    const float k = std::tan(PI * config.cutoff_freq * config.dt);
    const float k2 = k * k;
    const float denominator = 1.0f + SQRT2 * k + k2;
    if (!std::isfinite(k) || !std::isfinite(denominator) || denominator <= 0.0f)
    {
        return false;
    }

    float param[5] = {};
    param[0] = k2 / denominator;
    param[1] = 2.0f * param[0];
    param[2] = param[0];
    param[3] = 2.0f * (k2 - 1.0f) / denominator;
    param[4] = (1.0f - SQRT2 * k + k2) / denominator;

    for (uint8_t i = 0; i < 5; i++)
    {
        if (!std::isfinite(param[i]))
        {
            return false;
        }
        param_[i] = param[i];
    }

    cfg_ = config;
    initialized_ = true;
    return true;
}

float ButterworthFilter::calc(float input)
{
    if (!initialized_ || !std::isfinite(input))
    {
        return state_[2];
    }

    const float output = param_[0] * input + param_[1] * state_[0] + param_[2] * state_[1] - param_[3] * state_[2] -
                         param_[4] * state_[3];
    if (!std::isfinite(output))
    {
        return state_[2];
    }

    state_[3] = state_[2];
    state_[2] = output;
    state_[1] = state_[0];
    state_[0] = input;
    return output;
}

void ButterworthFilter::reset()
{
    for (float &state : state_)
    {
        state = 0.0f;
    }
}

bool ButterworthFilter::is_initialized() const
{
    return initialized_;
}

bool LowPassFilter::init(const Config &config)
{
    cfg_ = Config{};
    alpha_ = 0.0f;
    initialized_ = false;
    reset();

    if (!is_valid_cutoff(config.cutoff_freq, config.dt))
    {
        return false;
    }

    const float alpha = config.dt / (1.0f / (TWO_PI * config.cutoff_freq) + config.dt);
    if (!std::isfinite(alpha) || alpha <= 0.0f || alpha >= 1.0f)
    {
        return false;
    }

    cfg_ = config;
    alpha_ = alpha;
    initialized_ = true;
    return true;
}

float LowPassFilter::calc(float input)
{
    if (!initialized_ || !std::isfinite(input))
    {
        return output_;
    }

    const float output = alpha_ * input + (1.0f - alpha_) * output_;
    if (std::isfinite(output))
    {
        output_ = output;
    }
    return output_;
}

void LowPassFilter::reset()
{
    output_ = 0.0f;
}

bool LowPassFilter::is_initialized() const
{
    return initialized_;
}

bool HighPassFilter::init(const Config &config)
{
    cfg_ = Config{};
    alpha_ = 0.0f;
    initialized_ = false;
    reset();

    if (!is_valid_cutoff(config.cutoff_freq, config.dt))
    {
        return false;
    }

    const float alpha = 1.0f / (1.0f + TWO_PI * config.cutoff_freq * config.dt);
    if (!std::isfinite(alpha) || alpha <= 0.0f || alpha >= 1.0f)
    {
        return false;
    }

    cfg_ = config;
    alpha_ = alpha;
    initialized_ = true;
    return true;
}

float HighPassFilter::calc(float input)
{
    if (!initialized_ || !std::isfinite(input))
    {
        return output_;
    }

    const float output = alpha_ * (output_ + input - last_input_);
    if (!std::isfinite(output))
    {
        return output_;
    }

    last_input_ = input;
    output_ = output;
    return output_;
}

void HighPassFilter::reset()
{
    last_input_ = 0.0f;
    output_ = 0.0f;
}

bool HighPassFilter::is_initialized() const
{
    return initialized_;
}

bool BandPassFilter::init(const Config &config)
{
    cfg_ = Config{};
    hp_alpha_ = 0.0f;
    lp_alpha_ = 0.0f;
    initialized_ = false;
    reset();

    if (!is_valid_cutoff(config.low_cutoff, config.dt) || !is_valid_cutoff(config.high_cutoff, config.dt) ||
        config.high_cutoff <= config.low_cutoff)
    {
        return false;
    }

    const float hp_alpha = 1.0f / (1.0f + TWO_PI * config.low_cutoff * config.dt);
    const float lp_alpha = config.dt / (1.0f / (TWO_PI * config.high_cutoff) + config.dt);
    if (!std::isfinite(hp_alpha) || !std::isfinite(lp_alpha) || hp_alpha <= 0.0f || hp_alpha >= 1.0f ||
        lp_alpha <= 0.0f || lp_alpha >= 1.0f)
    {
        return false;
    }

    cfg_ = config;
    hp_alpha_ = hp_alpha;
    lp_alpha_ = lp_alpha;
    initialized_ = true;
    return true;
}

float BandPassFilter::calc(float input)
{
    if (!initialized_ || !std::isfinite(input))
    {
        return lp_output_;
    }

    const float hp_output = hp_alpha_ * (hp_output_ + input - hp_last_input_);
    const float lp_output = lp_alpha_ * hp_output + (1.0f - lp_alpha_) * lp_output_;
    if (!std::isfinite(hp_output) || !std::isfinite(lp_output))
    {
        return lp_output_;
    }

    hp_last_input_ = input;
    hp_output_ = hp_output;
    lp_output_ = lp_output;
    return lp_output_;
}

void BandPassFilter::reset()
{
    hp_last_input_ = 0.0f;
    hp_output_ = 0.0f;
    lp_output_ = 0.0f;
}

bool BandPassFilter::is_initialized() const
{
    return initialized_;
}

bool MovingAvgFilter::init(const Config &config)
{
    cfg_ = Config{};
    initialized_ = false;
    reset();

    if (config.window_size == 0 || config.window_size > MAX_WINDOW)
    {
        return false;
    }

    cfg_ = config;
    initialized_ = true;
    return true;
}

float MovingAvgFilter::calc(float input)
{
    if (!initialized_ || !std::isfinite(input))
    {
        return (count_ == 0) ? 0.0f : sum_ / static_cast<float>(count_);
    }

    const float sum = sum_ + input - buf_[index_];
    if (!std::isfinite(sum))
    {
        return (count_ == 0) ? 0.0f : sum_ / static_cast<float>(count_);
    }

    sum_ = sum;
    buf_[index_] = input;
    index_ = (index_ + 1U) % cfg_.window_size;
    if (count_ < cfg_.window_size)
    {
        count_++;
    }
    return sum_ / static_cast<float>(count_);
}

void MovingAvgFilter::reset()
{
    for (float &value : buf_)
    {
        value = 0.0f;
    }
    sum_ = 0.0f;
    count_ = 0;
    index_ = 0;
}

bool MovingAvgFilter::is_initialized() const
{
    return initialized_;
}

float AverageFilter::calc(float input)
{
    if (!std::isfinite(input))
    {
        return (count_ == 0) ? 0.0f : sum_ / static_cast<float>(count_);
    }

    if (count_ == std::numeric_limits<uint32_t>::max())
    {
        sum_ *= 0.5f;
        count_ /= 2U;
    }

    const float sum = sum_ + input;
    if (!std::isfinite(sum))
    {
        return (count_ == 0) ? 0.0f : sum_ / static_cast<float>(count_);
    }

    sum_ = sum;
    count_++;
    return sum_ / static_cast<float>(count_);
}

void AverageFilter::reset()
{
    sum_ = 0.0f;
    count_ = 0;
}

} // namespace filter
