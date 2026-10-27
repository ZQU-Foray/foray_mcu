#ifndef LIB_SLOPE_SLOPE_HPP
#define LIB_SLOPE_SLOPE_HPP

#include <cstdint>

namespace slope
{

enum class SlopeFirst : uint8_t
{
    REAL = 0,
    TARGET,
};

class Slope
{
  public:
    struct Config
    {
        // 增大和减小绝对值时的变化率，单位/秒
        float increase_rate = 0.0f;
        float decrease_rate = 0.0f;

        SlopeFirst first = SlopeFirst::TARGET;
    };

    bool init(const Config &config);

    void reset();

    bool reset(float output);

    bool set_now_real(float now_real);

    bool set_increase_rate(float increase_rate);

    bool set_decrease_rate(float decrease_rate);

    bool calculate_loop(float target, float dt);

    float get_out() const;

    float get_target() const;

    const Config &get_config() const;

    bool is_initialized() const;

  private:
    Config cfg_{};

    float output_ = 0.0f;
    float now_real_ = 0.0f;
    float target_ = 0.0f;

    bool initialized_ = false;
};

} // namespace slope

#endif
