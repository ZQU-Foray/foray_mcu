#ifndef PID_H
#define PID_H

#include <cstdint>

namespace alg_controller
{

enum class PIDMode : uint8_t
{
    POSITION,
    DELTA
};

struct PIDImprove
{
    uint8_t integral_limit : 1;
    uint8_t deriv_on_meas : 1;
    uint8_t trapezoid : 1;
    uint8_t changing_rate : 1;
    uint8_t deriv_filter : 1;
    uint8_t out_filter : 1;
    uint8_t dead_band : 1;
    uint8_t reserved : 1;
};

class PID
{
  public:
    struct Config
    {
        PIDMode mode = PIDMode::POSITION;
        PIDImprove improve = {};

        float kp = 0.0f;
        float ki = 0.0f;
        float kd = 0.0f;
        float kf = 0.0f; // 参考前馈增益，单位＝输出量/输入量（速度环 N·m/rpm，角度环 rpm/deg）
        float default_dt = 0.01f;

        float maxout = 0.0f;           // 输出总限幅，PI、D 与前馈之和不得超过它
        float feed_forward_limit = 0.0f; // 前馈独立限幅，Kf 非零时必须满足 0 < feed_forward_limit ≤ Maxout
        float integral_limit = 0.0f;

        float dead_zone = 0.0f;
        float i_var_a = 0.0f;
        float i_var_b = 0.0f;
        float d_lpf_rc = 0.0f;
        float out_lpf_rc = 0.0f;
    };

    bool init(const Config &config);

    bool calculate_loop(float measure, float target, float dt);

    void reset();

    float get_integral_error() const;

    float get_out() const;

    const Config &get_config() const;

    bool is_initialized() const;

    bool set_param(float kp, float ki, float kd);

    bool set_kp(float kp);

    bool set_ki(float ki);

    bool set_kd(float kd);

    bool set_kf(float kf);

    bool set_i_out_max(float integral_limit);

    bool set_out_max(float maxout);

    bool set_i_var_a(float i_var_a);

    bool set_i_var_b(float i_var_b);

    bool set_target(float target);

    bool set_now(float measure);

    bool set_integral_error(float iout);

  protected:
    Config cfg_{};

    float target_ = 0.0f;
    float measure_ = 0.0f;
    float err_ = 0.0f;

    float pout_ = 0.0f;
    float iout_ = 0.0f;
    float dout_ = 0.0f;
    float i_term_ = 0.0f;

    float output_ = 0.0f;
    bool initialized_ = false;

  private:
    float last_measure_ = 0.0f;
    float last_last_measure_ = 0.0f;
    float last_err_ = 0.0f;
    float last_last_err_ = 0.0f;
    float last_output_ = 0.0f;
    float last_feed_forward_ = 0.0f;
    float last_dout_ = 0.0f;
    bool measure_history_initialized_ = false;

    float dt_ = 0.0f;

    float feed_forward_term(float target) const;

    void trapezoid_integral();

    void changing_rate_integration();

    void integral_limit();

    void derivative_on_measurement();

    void derivative_filter();

    void output_filter();
};

} // namespace alg_controller

#endif
