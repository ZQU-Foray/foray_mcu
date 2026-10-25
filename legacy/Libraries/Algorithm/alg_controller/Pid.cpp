#include "Pid.h"
#include "Libraries/Algorithm/alg_math/BasicMath.h"
#include <cmath>

namespace
{

bool IsFinite(float value)
{
  return std::isfinite(value);
}

bool IsValidConfig(const alg_controller::PID::Config &config)
{
  const bool modeValid =
      config.Mode == alg_controller::PIDMode::Position || config.Mode == alg_controller::PIDMode::Delta;
  const bool paramFinite = IsFinite(config.Kp) && IsFinite(config.Ki) && IsFinite(config.Kd) && IsFinite(config.Kf) &&
                           IsFinite(config.DefaultDt) && IsFinite(config.Maxout) && IsFinite(config.FeedForwardLimit) &&
                           IsFinite(config.IntegralLimit) && IsFinite(config.DeadZone) && IsFinite(config.IVarA) &&
                           IsFinite(config.IVarB) && IsFinite(config.DLpfRc) && IsFinite(config.OutLpfRc);
  const bool feedForwardValid = config.FeedForwardLimit >= 0.0f && config.FeedForwardLimit <= config.Maxout &&
                                (config.Kf == 0.0f || config.FeedForwardLimit > 0.0f);

  return modeValid && paramFinite && feedForwardValid && config.DefaultDt > 0.0f && config.DefaultDt < 1.0f &&
         config.Maxout >= 0.0f && config.IntegralLimit >= 0.0f && config.DeadZone >= 0.0f && config.IVarA >= 0.0f &&
         config.IVarB >= 0.0f && config.DLpfRc >= 0.0f && config.OutLpfRc >= 0.0f;
}

} // namespace

namespace alg_controller
{

bool PID::Init(const Config &config)
{
  Cfg = Config{};
  Initialized = false;
  Reset();

  if (!IsValidConfig(config))
  {
    return false;
  }

  Cfg = config;
  Initialized = true;
  return true;
}

void PID::Reset()
{
  Target = 0.0f;
  Measure = 0.0f;
  LastMeasure = 0.0f;
  LastLastMeasure = 0.0f;
  Err = 0.0f;
  LastErr = 0.0f;
  LastLastErr = 0.0f;
  Pout = 0.0f;
  Iout = 0.0f;
  Dout = 0.0f;
  ITerm = 0.0f;
  Output = 0.0f;
  LastOutput = 0.0f;
  LastFeedForward = 0.0f;
  LastDout = 0.0f;
  MeasureHistoryInitialized = false;
  Dt = 0.0f;
}

bool PID::CalculateLoop(float measure, float target, float dt)
{
  if (!Initialized || !IsFinite(measure) || !IsFinite(target))
  {
    return false;
  }

  const PID backup = *this;
  Dt = (IsFinite(dt) && dt > 0.0f && dt < 1.0f) ? dt : Cfg.DefaultDt;
  Measure = measure;
  Target = target;
  Err = target - measure;
  const float feedForward = FeedForwardTerm(Target);
  if (!MeasureHistoryInitialized)
  {
    LastMeasure = Measure;
    LastLastMeasure = Measure;
    MeasureHistoryInitialized = true;
  }

  if (Cfg.Improve.DeadBand && std::fabs(Err) < Cfg.DeadZone)
  {
    Output = (Cfg.Mode == PIDMode::Delta) ? LastOutput - LastFeedForward + feedForward : feedForward;
    ITerm = 0.0f;
    LastLastErr = LastErr;
    LastLastMeasure = LastMeasure;
    LastMeasure = Measure;
    LastOutput = Output;
    LastFeedForward = feedForward;
    LastDout = Dout;
    LastErr = Err;
    return true;
  }

  switch (Cfg.Mode)
  {
  case PIDMode::Position:
    Pout = Cfg.Kp * Err;
    ITerm = Cfg.Ki * Err * Dt;

    if (Cfg.Improve.Trapezoid)
    {
      TrapezoidIntegral();
    }
    if (Cfg.Improve.ChangingRate)
    {
      ChangingRateIntegration();
    }
    if (Cfg.Improve.DerivOnMeas)
    {
      DerivativeOnMeasurement();
    }
    else
    {
      Dout = Cfg.Kd * (Err - LastErr) / Dt;
    }
    if (Cfg.Improve.DerivFilter)
    {
      DerivativeFilter();
    }
    if (Cfg.Improve.IntegralLimit)
    {
      IntegralLimit();
    }

    Iout += ITerm;
    if (!Cfg.Improve.IntegralLimit)
    {
      Iout = alg_math::Limit(Iout, -Cfg.IntegralLimit, Cfg.IntegralLimit);
    }

    Output = Pout + Iout + Dout + feedForward;
    if (Cfg.Improve.OutFilter)
    {
      OutputFilter();
    }
    break;

  case PIDMode::Delta:
    Pout = Cfg.Kp * (Err - LastErr);
    ITerm = Cfg.Ki * Err * Dt;

    if (Cfg.Improve.ChangingRate)
    {
      ChangingRateIntegration();
    }
    Iout = ITerm;

    if (Cfg.Improve.DerivOnMeas)
    {
      Dout = Cfg.Kd * (2.0f * LastMeasure - Measure - LastLastMeasure) / Dt;
    }
    else
    {
      Dout = Cfg.Kd * (Err - 2.0f * LastErr + LastLastErr) / Dt;
    }
    if (Cfg.Improve.DerivFilter)
    {
      DerivativeFilter();
    }

    Output = LastOutput + Pout + Iout + Dout + feedForward - LastFeedForward;
    if (Cfg.Improve.OutFilter)
    {
      OutputFilter();
    }
    break;

  default:
    *this = backup;
    return false;
  }

  Output = alg_math::Limit(Output, -Cfg.Maxout, Cfg.Maxout);
  if (!IsFinite(Err) || !IsFinite(Pout) || !IsFinite(Iout) || !IsFinite(Dout) || !IsFinite(ITerm) || !IsFinite(Output))
  {
    *this = backup;
    return false;
  }

  LastLastErr = LastErr;
  LastLastMeasure = LastMeasure;
  LastMeasure = Measure;
  LastOutput = Output;
  LastFeedForward = feedForward;
  LastDout = Dout;
  LastErr = Err;
  return true;
}

float PID::GetIntegralError() const
{
  return Iout;
}

float PID::GetOut() const
{
  return Output;
}

const PID::Config &PID::GetConfig() const
{
  return Cfg;
}

bool PID::IsInitialized() const
{
  return Initialized;
}

bool PID::SetParam(float kp, float ki, float kd)
{
  if (!Initialized || !IsFinite(kp) || !IsFinite(ki) || !IsFinite(kd))
  {
    return false;
  }
  Cfg.Kp = kp;
  Cfg.Ki = ki;
  Cfg.Kd = kd;
  return true;
}

bool PID::SetKp(float kp)
{
  return SetParam(kp, Cfg.Ki, Cfg.Kd);
}

bool PID::SetKi(float ki)
{
  return SetParam(Cfg.Kp, ki, Cfg.Kd);
}

bool PID::SetKd(float kd)
{
  return SetParam(Cfg.Kp, Cfg.Ki, kd);
}

bool PID::SetKf(float kf)
{
  if (!Initialized || !IsFinite(kf))
  {
    return false;
  }
  Cfg.Kf = kf;
  return true;
}

bool PID::SetIOutMax(float integralLimit)
{
  if (!Initialized || !IsFinite(integralLimit) || integralLimit < 0.0f)
  {
    return false;
  }
  Cfg.IntegralLimit = integralLimit;
  Iout = alg_math::Limit(Iout, -integralLimit, integralLimit);
  return true;
}

bool PID::SetOutMax(float maxout)
{
  if (!Initialized || !IsFinite(maxout) || maxout < 0.0f)
  {
    return false;
  }
  Cfg.Maxout = maxout;
  Output = alg_math::Limit(Output, -maxout, maxout);
  LastOutput = alg_math::Limit(LastOutput, -maxout, maxout);
  return true;
}

bool PID::SetIVarA(float iVarA)
{
  if (!Initialized || !IsFinite(iVarA) || iVarA < 0.0f)
  {
    return false;
  }
  Cfg.IVarA = iVarA;
  return true;
}

bool PID::SetIVarB(float iVarB)
{
  if (!Initialized || !IsFinite(iVarB) || iVarB < 0.0f)
  {
    return false;
  }
  Cfg.IVarB = iVarB;
  return true;
}

bool PID::SetTarget(float target)
{
  if (!Initialized || !IsFinite(target))
  {
    return false;
  }
  Target = target;
  return true;
}

bool PID::SetNow(float measure)
{
  if (!Initialized || !IsFinite(measure))
  {
    return false;
  }
  Measure = measure;
  LastMeasure = measure;
  LastLastMeasure = measure;
  MeasureHistoryInitialized = true;
  return true;
}

bool PID::SetIntegralError(float iout)
{
  if (!Initialized || !IsFinite(iout))
  {
    return false;
  }
  Iout = alg_math::Limit(iout, -Cfg.IntegralLimit, Cfg.IntegralLimit);
  return true;
}

float PID::FeedForwardTerm(float target) const
{
  return alg_math::Limit(Cfg.Kf * target, -Cfg.FeedForwardLimit, Cfg.FeedForwardLimit);
}

void PID::TrapezoidIntegral()
{
  ITerm = Cfg.Ki * ((Err + LastErr) * 0.5f) * Dt;
}

void PID::ChangingRateIntegration()
{
  if (Err * Iout <= 0.0f)
  {
    return;
  }

  const float absErr = std::fabs(Err);
  if (absErr <= Cfg.IVarB)
  {
    return;
  }
  if (Cfg.IVarA > 0.0f && absErr <= Cfg.IVarA + Cfg.IVarB)
  {
    ITerm *= (Cfg.IVarA - absErr + Cfg.IVarB) / Cfg.IVarA;
  }
  else
  {
    ITerm = 0.0f;
  }
}

void PID::IntegralLimit()
{
  const float tempIout = Iout + ITerm;
  const float tempOutput = Pout + tempIout + Dout + FeedForwardTerm(Target);

  if (std::fabs(tempOutput) > Cfg.Maxout && Err * tempIout > 0.0f)
  {
    ITerm = 0.0f;
  }

  if (tempIout > Cfg.IntegralLimit)
  {
    ITerm = 0.0f;
    Iout = Cfg.IntegralLimit;
  }
  else if (tempIout < -Cfg.IntegralLimit)
  {
    ITerm = 0.0f;
    Iout = -Cfg.IntegralLimit;
  }
}

void PID::DerivativeOnMeasurement()
{
  Dout = Cfg.Kd * (LastMeasure - Measure) / Dt;
}

void PID::DerivativeFilter()
{
  const float alpha = Dt / (Cfg.DLpfRc + Dt);
  Dout = alpha * Dout + (1.0f - alpha) * LastDout;
}

void PID::OutputFilter()
{
  const float alpha = Dt / (Cfg.OutLpfRc + Dt);
  Output = alpha * Output + (1.0f - alpha) * LastOutput;
}

} // namespace alg_controller
