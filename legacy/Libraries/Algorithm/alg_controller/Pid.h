#ifndef PID_H
#define PID_H

#include <cstdint>

namespace alg_controller
{

enum class PIDMode : uint8_t
{
  Position,
  Delta
};

struct PIDImprove
{
  uint8_t IntegralLimit : 1;
  uint8_t DerivOnMeas : 1;
  uint8_t Trapezoid : 1;
  uint8_t ChangingRate : 1;
  uint8_t DerivFilter : 1;
  uint8_t OutFilter : 1;
  uint8_t DeadBand : 1;
  uint8_t Reserved : 1;
};

class PID
{
public:
  struct Config
  {
    PIDMode Mode = PIDMode::Position;
    PIDImprove Improve = {};

    float Kp = 0.0f;
    float Ki = 0.0f;
    float Kd = 0.0f;
    float Kf = 0.0f; // 参考前馈增益，单位＝输出量/输入量（速度环 N·m/rpm，角度环 rpm/deg）
    float DefaultDt = 0.01f;

    float Maxout = 0.0f;           // 输出总限幅，PI、D 与前馈之和不得超过它
    float FeedForwardLimit = 0.0f; // 前馈独立限幅，Kf 非零时必须满足 0 < FeedForwardLimit ≤ Maxout
    float IntegralLimit = 0.0f;

    float DeadZone = 0.0f;
    float IVarA = 0.0f;
    float IVarB = 0.0f;
    float DLpfRc = 0.0f;
    float OutLpfRc = 0.0f;
  };

  bool Init(const Config &config);

  bool CalculateLoop(float measure, float target, float dt);

  void Reset();

  float GetIntegralError() const;

  float GetOut() const;

  const Config &GetConfig() const;

  bool IsInitialized() const;

  bool SetParam(float kp, float ki, float kd);

  bool SetKp(float kp);

  bool SetKi(float ki);

  bool SetKd(float kd);

  bool SetKf(float kf);

  bool SetIOutMax(float integralLimit);

  bool SetOutMax(float maxout);

  bool SetIVarA(float iVarA);

  bool SetIVarB(float iVarB);

  bool SetTarget(float target);

  bool SetNow(float measure);

  bool SetIntegralError(float iout);

protected:
  Config Cfg{};

  float Target = 0.0f;
  float Measure = 0.0f;
  float Err = 0.0f;

  float Pout = 0.0f;
  float Iout = 0.0f;
  float Dout = 0.0f;
  float ITerm = 0.0f;

  float Output = 0.0f;
  bool Initialized = false;

private:
  float LastMeasure = 0.0f;
  float LastLastMeasure = 0.0f;
  float LastErr = 0.0f;
  float LastLastErr = 0.0f;
  float LastOutput = 0.0f;
  float LastFeedForward = 0.0f;
  float LastDout = 0.0f;
  bool MeasureHistoryInitialized = false;

  float Dt = 0.0f;

  float FeedForwardTerm(float target) const;

  void TrapezoidIntegral();

  void ChangingRateIntegration();

  void IntegralLimit();

  void DerivativeOnMeasurement();

  void DerivativeFilter();

  void OutputFilter();
};

} // namespace alg_controller

#endif
