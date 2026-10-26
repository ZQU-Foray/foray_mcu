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
        float CutoffFreq = 0.0f; // 截止频率
        float Dt = 0.001f;       // 采样周期
    };

    bool Init(const Config &config);

    float Calc(float input);

    void Reset();

    bool IsInitialized() const;

  private:
    Config Cfg{};
    float State[4] = {}; // 滤波器状态
    float Param[5] = {}; // 滤波器系数
    bool Initialized = false;
};

class LowPassFilter
{
  public:
    struct Config
    {
        float CutoffFreq = 0.0f; // 截止频率
        float Dt = 0.001f;       // 采样周期
    };

    bool Init(const Config &config); // 内部由 cutoffFreq 和 dt 计算 alpha

    float Calc(float input);

    void Reset();

    bool IsInitialized() const;

  private:
    Config Cfg{};
    float Alpha = 0.0f;  // 滤波系数（Init 时计算）
    float Output = 0.0f; // 上次输出
    bool Initialized = false;
};

class HighPassFilter
{
  public:
    struct Config
    {
        float CutoffFreq = 0.0f; // 截止频率
        float Dt = 0.001f;       // 采样周期
    };

    bool Init(const Config &config); // 内部由 cutoffFreq 和 dt 计算 alpha

    float Calc(float input);

    void Reset();

    bool IsInitialized() const;

  private:
    Config Cfg{};
    float Alpha = 0.0f;     // 滤波系数（Init 时计算）
    float LastInput = 0.0f; // 上次输入
    float Output = 0.0f;    // 上次输出
    bool Initialized = false;
};

class BandPassFilter
{
  public:
    struct Config
    {
        float LowCutoff = 0.0f;  // 下限截止频率
        float HighCutoff = 0.0f; // 上限截止频率
        float Dt = 0.001f;       // 采样周期
    };

    bool Init(const Config &config); // 内部计算高通/低通两级系数

    float Calc(float input);

    void Reset();

    bool IsInitialized() const;

  private:
    Config Cfg{};
    // 两级一阶滤波串联（先高后低）的系数与状态
    float HpAlpha = 0.0f;     // 高通系数
    float HpLastInput = 0.0f; // 高通上次输入
    float HpOutput = 0.0f;    // 高通输出（低通输入）
    float LpAlpha = 0.0f;     // 低通系数
    float LpOutput = 0.0f;    // 低通输出（最终输出）
    bool Initialized = false;
};

class MovingAvgFilter
{
  public:
    static constexpr uint16_t maxWindow = 64; // 缓冲最大窗口

    struct Config
    {
        uint16_t WindowSize = 10; // 滑动窗口长度（≤ maxWindow）
    };

    bool Init(const Config &config);

    float Calc(float input);

    void Reset();

    bool IsInitialized() const;

  private:
    Config Cfg{};
    float Buf[maxWindow] = {}; // 环形缓冲
    float Sum = 0.0f;          // 窗口内和
    uint16_t Count = 0;        // 已采样数（窗口未满时用于归一化）
    uint16_t Index = 0;        // 环形缓冲写指针
    bool Initialized = false;
};

class AverageFilter
{
  public:
    float Calc(float input);

    void Reset();

  private:
    float Sum = 0.0f;   // 累加和
    uint32_t Count = 0; // 样本数
};

} // namespace alg_filter

#endif
