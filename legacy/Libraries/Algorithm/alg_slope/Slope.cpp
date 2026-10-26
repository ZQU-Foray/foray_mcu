#include "Slope.h"
#include <cmath>

namespace
{

bool IsFinite(float value)
{
    return std::isfinite(value);
}

float MoveTowards(float current, float target, float maxDelta)
{
    const float delta = target - current;

    if (std::fabs(delta) <= maxDelta)
    {
        return target;
    }

    return current + ((delta > 0.0f) ? maxDelta : -maxDelta);
}

bool IsBetween(float value, float first, float second)
{
    return (first <= value && value <= second) || (second <= value && value <= first);
}

} // namespace

namespace alg_slope
{

bool Slope::Init(const Config &config)
{
    Cfg = Config{};
    Output = 0.0f;
    NowReal = 0.0f;
    Target = 0.0f;
    Initialized = false;

    const bool firstValid = config.First == SlopeFirst::Real || config.First == SlopeFirst::Target;
    if (!firstValid || !IsFinite(config.IncreaseRate) || !IsFinite(config.DecreaseRate) ||
        config.IncreaseRate <= 0.0f || config.DecreaseRate <= 0.0f)
    {
        return false;
    }

    Cfg = config;
    Initialized = true;
    return true;
}

void Slope::Reset()
{
    Output = 0.0f;
    NowReal = 0.0f;
    Target = 0.0f;
}

bool Slope::Reset(float output)
{
    if (!IsFinite(output))
    {
        return false;
    }

    Output = output;
    NowReal = output;
    Target = output;
    return true;
}

bool Slope::SetNowReal(float nowReal)
{
    if (!Initialized || !IsFinite(nowReal))
    {
        return false;
    }

    NowReal = nowReal;
    return true;
}

bool Slope::SetIncreaseRate(float increaseRate)
{
    if (!Initialized || !IsFinite(increaseRate) || increaseRate <= 0.0f)
    {
        return false;
    }

    Cfg.IncreaseRate = increaseRate;
    return true;
}

bool Slope::SetDecreaseRate(float decreaseRate)
{
    if (!Initialized || !IsFinite(decreaseRate) || decreaseRate <= 0.0f)
    {
        return false;
    }

    Cfg.DecreaseRate = decreaseRate;
    return true;
}

bool Slope::CalculateLoop(float target, float dt)
{
    if (!Initialized || !IsFinite(target) || !IsFinite(dt) || dt <= 0.0f)
    {
        return false;
    }

    Target = target;

    if (Cfg.First == SlopeFirst::Real && IsBetween(NowReal, Output, Target))
    {
        Output = NowReal;
    }

    if (Output == Target)
    {
        return true;
    }

    const bool directionReversed = (Output > 0.0f && Target < 0.0f) || (Output < 0.0f && Target > 0.0f);
    if (directionReversed)
    {
        const float timeToZero = std::fabs(Output) / Cfg.DecreaseRate;

        if (dt <= timeToZero)
        {
            Output = MoveTowards(Output, 0.0f, Cfg.DecreaseRate * dt);
            return true;
        }

        const float remainingTime = dt - timeToZero;
        Output = MoveTowards(0.0f, Target, Cfg.IncreaseRate * remainingTime);
        return true;
    }

    const bool magnitudeIncreasing = std::fabs(Target) > std::fabs(Output);
    const float rate = magnitudeIncreasing ? Cfg.IncreaseRate : Cfg.DecreaseRate;
    Output = MoveTowards(Output, Target, rate * dt);

    return true;
}

float Slope::GetOut() const
{
    return Output;
}

float Slope::GetTarget() const
{
    return Target;
}

const Slope::Config &Slope::GetConfig() const
{
    return Cfg;
}

bool Slope::IsInitialized() const
{
    return Initialized;
}

} // namespace alg_slope
