#ifndef LIB_ESTIMATE_BIAS_STORE_HPP
#define LIB_ESTIMATE_BIAS_STORE_HPP

#include "utils/math/vector3.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace estimate
{

/**
 * @brief 零偏-温度记录：把每次标定得到的 (温度, 零偏) 追加进一块 NVS 区，下次上电直接取出 b(T)。
 *
 * 为什么要它：上电时云台可能在动，短静止窗口拿不到零偏（进 DEGRADED），而 Flash 里上一次的标定
 * 就在手边。主控板在比赛里会断电，所以 RAM 保持不住，必须落 Flash。
 *
 * 记录（每条 32 B，正好一个 H7 Flash word）：
 *   magic "VSNB"、版本、窗口长度（作为拟合权重）、序号、温度、三轴零偏、保留、CRC-16。
 * 记录按追加写，区内的顺序就是时间顺序；读时从 0 号槽往后扫，遇到擦除态就停，遇到 CRC 不对、
 * 数值越界或版本来路不对的记录也停（后面的记录不再采用）；停下的位置若是擦除态就接着写，
 * 否则（写坏的中间记录）整区压缩。
 *
 * 模型：只有一个温度点时按常数（按窗口长度加权平均）；两个以上温度点时按窗口长度加权拟合直线
 * b = k·T + c，求值时把温度夹在观测范围外最多 extrapolation_celsius；斜率超过上限时退回常数（护栏）。
 * 全部 float、无堆分配。
 *
 * 区写满时**不在运行期擦除**（擦一个 128 KB 扇区要 CPU 停 1–2 s，会打乱 1 kHz 控制环），
 * record() 直接返回 false；下次 init() 在启动期压缩（保留最新的 maximum_records 条重写）。
 */
class BiasStore final
{
  public:
    static constexpr std::size_t RECORD_SIZE{32U};
    static constexpr std::size_t MAXIMUM_POINTS{16U};

    /**
     * @brief 记录层的存储后端：一块可以整区擦除的块设备。
     *
     * 用函数指针而不是虚函数：没有分配、没有运行期分派，lib 也不依赖 hal（记录层只管算法，
     * Flash 由调用方接进来）。语义与 hal::Nvs 相同：写只能写在擦除态上、按 granularity 对齐、
     * erase() 是整区。context 由调用方解释（能力里放 hal::Nvs）。
     */
    struct Storage
    {
        void *context;
        std::size_t (*size)(void *context) noexcept;
        std::size_t (*granularity)(void *context) noexcept;
        bool (*read)(void *context, std::size_t offset, std::uint8_t *data, std::size_t length) noexcept;
        bool (*write)(void *context, std::size_t offset, const std::uint8_t *data, std::size_t length) noexcept;
        bool (*erase)(void *context) noexcept;
    };

    /** 来自 profile 的参数。 */
    struct Config
    {
        std::size_t maximum_records{8U};                   // 内存里保留的最新记录数，1–MAXIMUM_POINTS
        float temperature_span_celsius{0.5F};              // 观测温度范围小于它时按常数
        float extrapolation_celsius{3.0F};                 // 直线最多外推到观测范围外多少度
        float maximum_slope_rad_per_sec_per_celsius{0.1F}; // 超过它认为拟合不可信，退回常数
        float minimum_temperature_celsius{-20.0F};         // 记录的温度必须在范围内
        float maximum_temperature_celsius{80.0F};
    };

    /** 统计与诊断，调试器可读。 */
    struct Snapshot
    {
        bool available;          // NVS 能读写（参数、区大小、粒度都对）
        bool has_bias;           // 至少有一条有效记录
        bool fit_is_line;        // 当前模型是直线（否则是常数）
        std::size_t records;     // 内存里保留的有效记录数
        std::size_t written;     // 本次上电写进去的记录数
        std::size_t compactions; // 本次上电做的整区压缩次数（0 或 1）
        std::size_t append_slot; // 下一条写到第几个 32 B 槽
        std::size_t capacity;    // 这一区能放多少条记录
        std::uint32_t sequence;  // 下一条记录的序号
        float newest_temperature_celsius;
        math::Vector3 newest_bias_rad_per_sec;
        math::Vector3 slope_rad_per_sec_per_celsius; // 直线模型的斜率（常数模型为 0）
        math::Vector3 bias_at_newest;                // 在最新温度处的模型值
    };

    /**
     * @brief 校验参数（返回 false 表示参数非法），再扫描 NVS 并建立模型。
     * @note 区满或中间有坏记录时在这里压缩：**会整区擦除**（DM-MC02 上 CPU 停 1–2 s），只在启动期调用。
     *       设备不可用时返回 true，但 available 为 false（能力照常运行，只是没有存储零偏）。
     */
    [[nodiscard]] bool init(const Storage &storage, const Config &config) noexcept;

    [[nodiscard]] bool available() const noexcept
    {
        return snapshot_.available;
    }

    [[nodiscard]] bool has_bias() const noexcept
    {
        return snapshot_.has_bias;
    }

    /** 最新一条记录的温度（没有记录时为 0）。 */
    [[nodiscard]] float newest_temperature_celsius() const noexcept
    {
        return snapshot_.newest_temperature_celsius;
    }

    /**
     * @brief 模型在 temperature_celsius 处的零偏；没有记录时返回 0。
     * @note 温度不是有限数时按最新一条记录的温度算。
     */
    [[nodiscard]] math::Vector3 bias_at(float temperature_celsius) const noexcept;

    /**
     * @brief 追加一条记录并更新模型。
     * @param temperature_celsius 这条记录对应的温度（必须有限且在配置范围内）
     * @param bias_rad_per_sec 静止窗口的均值
     * @param window_us 窗口长度，作为拟合权重（0 按 1 计）
     * @return 设备不可用、区满（等下次启动压缩）或写入失败时返回 false，内存里的模型不变。
     */
    [[nodiscard]] bool
    record(float temperature_celsius, const math::Vector3 &bias_rad_per_sec, std::uint32_t window_us) noexcept;

    [[nodiscard]] const Snapshot &snapshot() const noexcept
    {
        return snapshot_;
    }

  private:
    struct Point
    {
        float temperature_celsius;
        math::Vector3 bias_rad_per_sec;
        std::uint16_t weight;
        std::uint32_t sequence;
    };

    [[nodiscard]] bool config_valid(const Config &config) const noexcept;
    void scan() noexcept;
    void compact() noexcept;
    void fit() noexcept;
    void append_point(const Point &point) noexcept;
    [[nodiscard]] bool write_point(std::size_t slot, const Point &point) noexcept;
    [[nodiscard]] bool read_record(std::size_t slot, Point &point) const noexcept;
    [[nodiscard]] bool slot_erased(std::size_t slot) const noexcept;

    Storage storage_{};
    Config config_{};
    Snapshot snapshot_{};
    std::array<Point, MAXIMUM_POINTS> points_{};
    std::size_t count_{0U};
    std::size_t stride_{RECORD_SIZE};
    std::uint32_t sequence_{1U};
    float minimum_temperature_{0.0F};
    float maximum_temperature_{0.0F};
    math::Vector3 constant_bias_{};
    math::Vector3 slope_{};
};

} // namespace estimate

#endif // LIB_ESTIMATE_BIAS_STORE_HPP
