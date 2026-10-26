#ifndef SLOPE_H
#define SLOPE_H

#include <cstdint>

namespace alg_slope
{

enum class SlopeFirst : uint8_t
{
    Real = 0,
    Target,
};

class Slope
{
  public:
    struct Config
    {
        // 增大和减小绝对值时的变化率，单位/秒
        float IncreaseRate = 0.0f;
        float DecreaseRate = 0.0f;

        SlopeFirst First = SlopeFirst::Target;
    };

    bool Init(const Config &config);

    void Reset();

    bool Reset(float output);

    bool SetNowReal(float nowReal);

    bool SetIncreaseRate(float increaseRate);

    bool SetDecreaseRate(float decreaseRate);

    bool CalculateLoop(float target, float dt);

    float GetOut() const;

    float GetTarget() const;

    const Config &GetConfig() const;

    bool IsInitialized() const;

  private:
    Config Cfg{};

    float Output = 0.0f;
    float NowReal = 0.0f;
    float Target = 0.0f;

    bool Initialized = false;
};

} // namespace alg_slope

#endif
