#include "Filter.h"
#include <cmath>
#include <limits>

namespace alg_filter
{
namespace
{
constexpr float pi = 3.14159265358979f;
constexpr float twoPi = 6.28318530718f;
constexpr float sqrt2 = 1.41421356237f;

bool IsValidCutoff(float cutoffFreq, float dt)
{
    return std::isfinite(cutoffFreq) && std::isfinite(dt) && cutoffFreq > 0.0f && dt > 0.0f && cutoffFreq * dt < 0.5f;
}
} // namespace

bool ButterworthFilter::Init(const Config &config)
{
    Cfg = Config{};
    for (float &param : Param)
    {
        param = 0.0f;
    }
    Initialized = false;
    Reset();

    if (!IsValidCutoff(config.CutoffFreq, config.Dt))
    {
        return false;
    }

    const float k = std::tan(pi * config.CutoffFreq * config.Dt);
    const float k2 = k * k;
    const float denominator = 1.0f + sqrt2 * k + k2;
    if (!std::isfinite(k) || !std::isfinite(denominator) || denominator <= 0.0f)
    {
        return false;
    }

    float param[5] = {};
    param[0] = k2 / denominator;
    param[1] = 2.0f * param[0];
    param[2] = param[0];
    param[3] = 2.0f * (k2 - 1.0f) / denominator;
    param[4] = (1.0f - sqrt2 * k + k2) / denominator;

    for (uint8_t i = 0; i < 5; i++)
    {
        if (!std::isfinite(param[i]))
        {
            return false;
        }
        Param[i] = param[i];
    }

    Cfg = config;
    Initialized = true;
    return true;
}

float ButterworthFilter::Calc(float input)
{
    if (!Initialized || !std::isfinite(input))
    {
        return State[2];
    }

    const float output =
        Param[0] * input + Param[1] * State[0] + Param[2] * State[1] - Param[3] * State[2] - Param[4] * State[3];
    if (!std::isfinite(output))
    {
        return State[2];
    }

    State[3] = State[2];
    State[2] = output;
    State[1] = State[0];
    State[0] = input;
    return output;
}

void ButterworthFilter::Reset()
{
    for (float &state : State)
    {
        state = 0.0f;
    }
}

bool ButterworthFilter::IsInitialized() const
{
    return Initialized;
}

bool LowPassFilter::Init(const Config &config)
{
    Cfg = Config{};
    Alpha = 0.0f;
    Initialized = false;
    Reset();

    if (!IsValidCutoff(config.CutoffFreq, config.Dt))
    {
        return false;
    }

    const float alpha = config.Dt / (1.0f / (twoPi * config.CutoffFreq) + config.Dt);
    if (!std::isfinite(alpha) || alpha <= 0.0f || alpha >= 1.0f)
    {
        return false;
    }

    Cfg = config;
    Alpha = alpha;
    Initialized = true;
    return true;
}

float LowPassFilter::Calc(float input)
{
    if (!Initialized || !std::isfinite(input))
    {
        return Output;
    }

    const float output = Alpha * input + (1.0f - Alpha) * Output;
    if (std::isfinite(output))
    {
        Output = output;
    }
    return Output;
}

void LowPassFilter::Reset()
{
    Output = 0.0f;
}

bool LowPassFilter::IsInitialized() const
{
    return Initialized;
}

bool HighPassFilter::Init(const Config &config)
{
    Cfg = Config{};
    Alpha = 0.0f;
    Initialized = false;
    Reset();

    if (!IsValidCutoff(config.CutoffFreq, config.Dt))
    {
        return false;
    }

    const float alpha = 1.0f / (1.0f + twoPi * config.CutoffFreq * config.Dt);
    if (!std::isfinite(alpha) || alpha <= 0.0f || alpha >= 1.0f)
    {
        return false;
    }

    Cfg = config;
    Alpha = alpha;
    Initialized = true;
    return true;
}

float HighPassFilter::Calc(float input)
{
    if (!Initialized || !std::isfinite(input))
    {
        return Output;
    }

    const float output = Alpha * (Output + input - LastInput);
    if (!std::isfinite(output))
    {
        return Output;
    }

    LastInput = input;
    Output = output;
    return Output;
}

void HighPassFilter::Reset()
{
    LastInput = 0.0f;
    Output = 0.0f;
}

bool HighPassFilter::IsInitialized() const
{
    return Initialized;
}

bool BandPassFilter::Init(const Config &config)
{
    Cfg = Config{};
    HpAlpha = 0.0f;
    LpAlpha = 0.0f;
    Initialized = false;
    Reset();

    if (!IsValidCutoff(config.LowCutoff, config.Dt) || !IsValidCutoff(config.HighCutoff, config.Dt) ||
        config.HighCutoff <= config.LowCutoff)
    {
        return false;
    }

    const float hpAlpha = 1.0f / (1.0f + twoPi * config.LowCutoff * config.Dt);
    const float lpAlpha = config.Dt / (1.0f / (twoPi * config.HighCutoff) + config.Dt);
    if (!std::isfinite(hpAlpha) || !std::isfinite(lpAlpha) || hpAlpha <= 0.0f || hpAlpha >= 1.0f || lpAlpha <= 0.0f ||
        lpAlpha >= 1.0f)
    {
        return false;
    }

    Cfg = config;
    HpAlpha = hpAlpha;
    LpAlpha = lpAlpha;
    Initialized = true;
    return true;
}

float BandPassFilter::Calc(float input)
{
    if (!Initialized || !std::isfinite(input))
    {
        return LpOutput;
    }

    const float hpOutput = HpAlpha * (HpOutput + input - HpLastInput);
    const float lpOutput = LpAlpha * hpOutput + (1.0f - LpAlpha) * LpOutput;
    if (!std::isfinite(hpOutput) || !std::isfinite(lpOutput))
    {
        return LpOutput;
    }

    HpLastInput = input;
    HpOutput = hpOutput;
    LpOutput = lpOutput;
    return LpOutput;
}

void BandPassFilter::Reset()
{
    HpLastInput = 0.0f;
    HpOutput = 0.0f;
    LpOutput = 0.0f;
}

bool BandPassFilter::IsInitialized() const
{
    return Initialized;
}

bool MovingAvgFilter::Init(const Config &config)
{
    Cfg = Config{};
    Initialized = false;
    Reset();

    if (config.WindowSize == 0 || config.WindowSize > maxWindow)
    {
        return false;
    }

    Cfg = config;
    Initialized = true;
    return true;
}

float MovingAvgFilter::Calc(float input)
{
    if (!Initialized || !std::isfinite(input))
    {
        return (Count == 0) ? 0.0f : Sum / static_cast<float>(Count);
    }

    const float sum = Sum + input - Buf[Index];
    if (!std::isfinite(sum))
    {
        return (Count == 0) ? 0.0f : Sum / static_cast<float>(Count);
    }

    Sum = sum;
    Buf[Index] = input;
    Index = (Index + 1U) % Cfg.WindowSize;
    if (Count < Cfg.WindowSize)
    {
        Count++;
    }
    return Sum / static_cast<float>(Count);
}

void MovingAvgFilter::Reset()
{
    for (float &value : Buf)
    {
        value = 0.0f;
    }
    Sum = 0.0f;
    Count = 0;
    Index = 0;
}

bool MovingAvgFilter::IsInitialized() const
{
    return Initialized;
}

float AverageFilter::Calc(float input)
{
    if (!std::isfinite(input))
    {
        return (Count == 0) ? 0.0f : Sum / static_cast<float>(Count);
    }

    if (Count == std::numeric_limits<uint32_t>::max())
    {
        Sum *= 0.5f;
        Count /= 2U;
    }

    const float sum = Sum + input;
    if (!std::isfinite(sum))
    {
        return (Count == 0) ? 0.0f : Sum / static_cast<float>(Count);
    }

    Sum = sum;
    Count++;
    return Sum / static_cast<float>(Count);
}

void AverageFilter::Reset()
{
    Sum = 0.0f;
    Count = 0;
}

} // namespace alg_filter
