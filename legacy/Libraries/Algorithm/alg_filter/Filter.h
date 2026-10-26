#ifndef FILTER_H
#define FILTER_H

#include <cstdint>

namespace alg_filter
{

class ButterworthFilter
{
  public:
    struct Config
    {
        float cutoff_freq = 0.0f; // 截止频率
        float dt = 0.001f;        // 采样周期
    };

    bool init(const Config &config);

    float calc(float input);

    void reset();

    bool is_initialized() const;

  private:
    Config cfg_{};
    float state_[4] = {}; // 滤波器状态
    float param_[5] = {}; // 滤波器系数
    bool initialized_ = false;
};

class LowPassFilter
{
  public:
    struct Config
    {
        float cutoff_freq = 0.0f; // 截止频率
        float dt = 0.001f;        // 采样周期
    };

    bool init(const Config &config); // 内部由 cutoff_freq 和 dt 计算 alpha

    float calc(float input);

    void reset();

    bool is_initialized() const;

  private:
    Config cfg_{};
    float alpha_ = 0.0f;  // 滤波系数（Init 时计算）
    float output_ = 0.0f; // 上次输出
    bool initialized_ = false;
};

class HighPassFilter
{
  public:
    struct Config
    {
        float cutoff_freq = 0.0f; // 截止频率
        float dt = 0.001f;        // 采样周期
    };

    bool init(const Config &config); // 内部由 cutoff_freq 和 dt 计算 alpha

    float calc(float input);

    void reset();

    bool is_initialized() const;

  private:
    Config cfg_{};
    float alpha_ = 0.0f;      // 滤波系数（Init 时计算）
    float last_input_ = 0.0f; // 上次输入
    float output_ = 0.0f;     // 上次输出
    bool initialized_ = false;
};

class BandPassFilter
{
  public:
    struct Config
    {
        float low_cutoff = 0.0f;  // 下限截止频率
        float high_cutoff = 0.0f; // 上限截止频率
        float dt = 0.001f;        // 采样周期
    };

    bool init(const Config &config); // 内部计算高通/低通两级系数

    float calc(float input);

    void reset();

    bool is_initialized() const;

  private:
    Config cfg_{};
    // 两级一阶滤波串联（先高后低）的系数与状态
    float hp_alpha_ = 0.0f;      // 高通系数
    float hp_last_input_ = 0.0f; // 高通上次输入
    float hp_output_ = 0.0f;     // 高通输出（低通输入）
    float lp_alpha_ = 0.0f;      // 低通系数
    float lp_output_ = 0.0f;     // 低通输出（最终输出）
    bool initialized_ = false;
};

class MovingAvgFilter
{
  public:
    static constexpr uint16_t MAX_WINDOW = 64; // 缓冲最大窗口

    struct Config
    {
        uint16_t window_size = 10; // 滑动窗口长度（≤ MAX_WINDOW）
    };

    bool init(const Config &config);

    float calc(float input);

    void reset();

    bool is_initialized() const;

  private:
    Config cfg_{};
    float buf_[MAX_WINDOW] = {}; // 环形缓冲
    float sum_ = 0.0f;           // 窗口内和
    uint16_t count_ = 0;         // 已采样数（窗口未满时用于归一化）
    uint16_t index_ = 0;         // 环形缓冲写指针
    bool initialized_ = false;
};

class AverageFilter
{
  public:
    float calc(float input);

    void reset();

  private:
    float sum_ = 0.0f;   // 累加和
    uint32_t count_ = 0; // 样本数
};

} // namespace alg_filter

#endif
