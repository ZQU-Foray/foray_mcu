#include "Application/Imu/ImuDiagnostics.hpp"
#include "Libraries/Device/bmi088/Bmi088Heater.hpp"
#include "Platform/Interface/Time.hpp"
#include <cmath>
#include <cstdint>
#include <limits>

namespace application
{
#if defined(IMU_ENABLE_DIAGNOSTICS)
namespace
{
constexpr std::uint32_t CAPTURE_SAMPLES{20000U};
constexpr std::uint32_t STORED_SAMPLES{1024U};
struct RawRecord
{
    std::uint32_t tick;
    std::uint32_t sequence;
    std::int16_t xyz[3];
    std::uint16_t reserved;
};
static_assert(sizeof(RawRecord) == 16U);
struct ChannelCapture
{
    std::uint32_t count{0U}, read_gaps{0U}, drdy_gaps{0U};
    std::uint32_t last_read{0U}, last_drdy{0U}, last_tick{0U};
    std::uint32_t coalesced_start{0U}, coalesced_end{0U};
    std::uint32_t errors_start{0U}, errors_end{0U};
    std::uint32_t overflow_start{0U}, overflow_end{0U}, high_water{0U};
    std::uint32_t late_starts{0U}, ambiguous{0U};
    std::uint32_t max_start_ticks{0U}, max_complete_ticks{0U}, max_harvest_ticks{0U};
    std::uint64_t elapsed_ticks{0U}, sum_start_ticks{0U}, sum_complete_ticks{0U};
};
static_assert(sizeof(ChannelCapture) == 96U);
struct Capture
{
    std::uint32_t version{2U};
    std::uint32_t state{0U}; // 0：等待温度；1：记录中；2：已冻结
    std::uint32_t count{0U};
    std::uint32_t stored{0U};
    std::uint32_t tick_hz{0U};
    std::uint32_t first_tick{0U};
    std::uint32_t last_tick{0U};
    std::uint32_t sequence_gaps{0U};
    std::uint32_t min_interval{UINT32_MAX};
    std::uint32_t max_interval{0U};
    std::uint32_t drdy_start{0U}, drdy_end{0U};
    std::uint32_t coalesced_start{0U}, coalesced_end{0U};
    std::uint32_t errors_start{0U}, errors_end{0U};
    float temperature_start{0.0F}, temperature_end{0.0F};
    std::int64_t sum[3]{};
    std::uint64_t sum_squared[3]{};
    std::int16_t minimum[3]{INT16_MAX, INT16_MAX, INT16_MAX};
    std::int16_t maximum[3]{INT16_MIN, INT16_MIN, INT16_MIN};
    std::uint32_t last_sequence{0U};
    std::uint64_t elapsed_ticks{0U}; // 采集时长可能超过 32 位 DWT 回绕周期
    RawRecord records[STORED_SAMPLES]{};
    ChannelCapture channels[2]{}; // 加速度计、陀螺仪；保留 v1 前缀
};
static_assert(sizeof(Capture) == 16720U);
// 头部非零初始化使其落入普通 .data（AXI SRAM），
// 不占用 DTCM/RTOS 堆。冻结状态只在全部数据
// 就绪后写入。
volatile Capture g_gyro_capture{};
// 1：带温度条件重新武装；2：仅计时，无温度门限。
// 两条命令都不改变加热器或传感器设置。
volatile std::uint32_t g_gyro_capture_request{0U};
platform::Time::Tick g_temperature_since{0U};
bool g_temperature_qualified{false};

// 显式武装的只读热特性采集。一块约一秒；本报告的任何数值
// 都不会回馈进 IMU 或估计器。
constexpr std::uint32_t THERMAL_PROFILE_BLOCKS{360U};
struct ProfileMoments
{
    std::uint32_t count, reserved;
    std::int64_t sum[3];
    std::uint64_t sum_squared[3];
};
static_assert(sizeof(ProfileMoments) == 56U);
struct ThermalProfileBlock
{
    std::uint32_t first_tick, last_tick;
    float temperature_min, temperature_max;
    std::uint32_t temperature_reads_start, temperature_reads_end;
    std::uint32_t sample_flags, reserved;
    ProfileMoments accelerometer, gyroscope;
};
static_assert(sizeof(ThermalProfileBlock) == 144U);
struct ThermalProfile
{
    std::uint32_t version;
    std::uint32_t state; // 0 空闲，1 记录中，2 已冻结
    std::uint32_t completed_blocks;
    std::uint32_t target_blocks;
    std::uint32_t tick_hz;
    std::uint32_t temperature_reads_start, temperature_reads_end;
    std::uint32_t gyro_errors_start, gyro_errors_end;
    std::uint32_t accel_errors_start, accel_errors_end;
    std::uint32_t gyro_overflow_start, gyro_overflow_end;
    std::uint32_t accel_overflow_start, accel_overflow_end;
    std::uint32_t gyro_coalesced_start, gyro_coalesced_end;
    std::uint32_t accel_coalesced_start, accel_coalesced_end;
    ThermalProfileBlock blocks[THERMAL_PROFILE_BLOCKS];
};
static_assert(sizeof(ThermalProfile) == 51920U);
// 现有链接脚本只提供 NOLOAD 的 AXI-SRAM（RAM_D1）段 `.dma_buffer*`，本诊断存档
// 由 CPU 独占、从不交给 DMA，因此单独用一个子段名 `.dma_buffer.imu_diag`
// 与真正的 DMA 缓冲区分开，同时仍落在 RAM_D1（DTCM 放不下这 52 KB）。
__attribute__((section(".dma_buffer.imu_diag"), aligned(32))) volatile ThermalProfile g_gyro_thermal_profile{};
// 调试器邮箱：复位后写 1 一次即开始新一轮采集。
volatile std::uint32_t g_gyro_thermal_profile_request{0U};

void reset_moments(volatile ProfileMoments &moments)
{
    moments.count = moments.reserved = 0U;
    for (unsigned i = 0; i < 3; ++i)
    {
        moments.sum[i] = 0;
        moments.sum_squared[i] = 0U;
    }
}

void add_moments(volatile ProfileMoments &moments, const device::Bmi088SampleRecord &record)
{
    ++moments.count;
    for (unsigned i = 0; i < 3; ++i)
    {
        const auto value = static_cast<std::int32_t>(record.xyz[i]);
        moments.sum[i] += value;
        moments.sum_squared[i] += static_cast<std::uint32_t>(value * value);
    }
}

void reset_thermal_block(volatile ThermalProfileBlock &block,
                       const device::Bmi088SampleRecord &record,
                       float temperature,
                       std::uint32_t temperature_reads)
{
    block.first_tick = block.last_tick = record.drdy_tick;
    block.temperature_min = block.temperature_max = temperature;
    block.temperature_reads_start = block.temperature_reads_end = temperature_reads;
    block.sample_flags = block.reserved = 0U;
    reset_moments(block.accelerometer);
    reset_moments(block.gyroscope);
}

void freeze_thermal_profile(const device::Bmi088 &imu)
{
    g_gyro_thermal_profile.temperature_reads_end = imu.get_temperature().get_read_count();
    g_gyro_thermal_profile.gyro_errors_end = imu.get_gyroscope().get_error_count();
    g_gyro_thermal_profile.accel_errors_end = imu.get_accelerometer().get_error_count();
    g_gyro_thermal_profile.gyro_overflow_end = imu.get_gyroscope_queue().overflow_count();
    g_gyro_thermal_profile.accel_overflow_end = imu.get_accelerometer_queue().overflow_count();
    g_gyro_thermal_profile.gyro_coalesced_end = imu.get_gyroscope_coalesced_event_count();
    g_gyro_thermal_profile.accel_coalesced_end = imu.get_accelerometer_coalesced_event_count();
    g_gyro_thermal_profile.state = 2U;
}

void observe_thermal_profile(const device::Bmi088 &imu,
                           const device::Bmi088SampleRecord &record,
                           bool gyroscope,
                           float temperature)
{
    if (g_gyro_thermal_profile_request == 1U)
    {
        g_gyro_thermal_profile_request = 0U;
        g_gyro_thermal_profile.version = 3U;
        g_gyro_thermal_profile.state = 1U;
        g_gyro_thermal_profile.completed_blocks = 0U;
        g_gyro_thermal_profile.target_blocks = THERMAL_PROFILE_BLOCKS;
        g_gyro_thermal_profile.tick_hz = platform::Time::tick_frequency_hz();
        g_gyro_thermal_profile.temperature_reads_start = g_gyro_thermal_profile.temperature_reads_end =
            imu.get_temperature().get_read_count();
        g_gyro_thermal_profile.gyro_errors_start = g_gyro_thermal_profile.gyro_errors_end = imu.get_gyroscope().get_error_count();
        g_gyro_thermal_profile.accel_errors_start = g_gyro_thermal_profile.accel_errors_end =
            imu.get_accelerometer().get_error_count();
        g_gyro_thermal_profile.gyro_overflow_start = g_gyro_thermal_profile.gyro_overflow_end =
            imu.get_gyroscope_queue().overflow_count();
        g_gyro_thermal_profile.accel_overflow_start = g_gyro_thermal_profile.accel_overflow_end =
            imu.get_accelerometer_queue().overflow_count();
        g_gyro_thermal_profile.gyro_coalesced_start = g_gyro_thermal_profile.gyro_coalesced_end =
            imu.get_gyroscope_coalesced_event_count();
        g_gyro_thermal_profile.accel_coalesced_start = g_gyro_thermal_profile.accel_coalesced_end =
            imu.get_accelerometer_coalesced_event_count();
        if (g_gyro_thermal_profile.tick_hz == 0U || !std::isfinite(temperature))
        {
            g_gyro_thermal_profile.state = 0U;
            return;
        }
        reset_thermal_block(g_gyro_thermal_profile.blocks[0], record, temperature, imu.get_temperature().get_read_count());
    }
    if (g_gyro_thermal_profile.state != 1U || !std::isfinite(temperature))
        return;
    auto index = g_gyro_thermal_profile.completed_blocks;
    if (index >= g_gyro_thermal_profile.target_blocks)
    {
        freeze_thermal_profile(imu);
        return;
    }
    auto &block = g_gyro_thermal_profile.blocks[index];
    if (static_cast<std::uint32_t>(record.drdy_tick - block.first_tick) >= g_gyro_thermal_profile.tick_hz)
    {
        block.temperature_reads_end = imu.get_temperature().get_read_count();
        ++g_gyro_thermal_profile.completed_blocks;
        index = g_gyro_thermal_profile.completed_blocks;
        if (index >= g_gyro_thermal_profile.target_blocks)
        {
            freeze_thermal_profile(imu);
            return;
        }
        reset_thermal_block(g_gyro_thermal_profile.blocks[index], record, temperature, imu.get_temperature().get_read_count());
    }
    auto &current = g_gyro_thermal_profile.blocks[index];
    current.last_tick = record.drdy_tick;
    current.temperature_reads_end = imu.get_temperature().get_read_count();
    if (temperature < current.temperature_min)
        current.temperature_min = temperature;
    if (temperature > current.temperature_max)
        current.temperature_max = temperature;
    current.sample_flags |= record.flags;
    add_moments(gyroscope ? current.gyroscope : current.accelerometer, record);
}

void observe_channel(const device::Bmi088 &imu, const device::Bmi088SampleRecord &record, bool gyro)
{
    auto &c = g_gyro_capture.channels[gyro ? 1 : 0];
    const auto &queue = gyro ? imu.get_gyroscope_queue() : imu.get_accelerometer_queue();
    const auto coalesced = gyro ? imu.get_gyroscope_coalesced_event_count() : imu.get_accelerometer_coalesced_event_count();
    const auto errors = gyro ? imu.get_gyroscope().get_error_count() : imu.get_accelerometer().get_error_count();
    if (c.count == 0U)
    {
        c.coalesced_start = coalesced;
        c.errors_start = errors;
        c.overflow_start = queue.overflow_count();
    }
    else
    {
        c.read_gaps += record.sequence - c.last_read - 1U;
        c.drdy_gaps += record.drdy_sequence - c.last_drdy - 1U;
        c.elapsed_ticks += static_cast<std::uint32_t>(record.drdy_tick - c.last_tick);
    }
    c.last_read = record.sequence;
    c.last_drdy = record.drdy_sequence;
    c.last_tick = record.drdy_tick;
    c.coalesced_end = coalesced;
    c.errors_end = errors;
    c.overflow_end = queue.overflow_count();
    c.high_water = queue.high_water();
    if ((record.flags & device::Bmi088SampleRecord::LATE_START) != 0U)
        ++c.late_starts;
    if ((record.flags & device::Bmi088SampleRecord::NEWER_EVENT_BEFORE_COMPLETION) != 0U)
        ++c.ambiguous;
    const auto start = record.start_tick - record.drdy_tick;
    const auto complete = record.completed_tick - record.drdy_tick;
    const auto harvest = record.harvested_tick - record.completed_tick;
    if (start > c.max_start_ticks)
        c.max_start_ticks = start;
    if (complete > c.max_complete_ticks)
        c.max_complete_ticks = complete;
    if (harvest > c.max_harvest_ticks)
        c.max_harvest_ticks = harvest;
    c.sum_start_ticks += start;
    c.sum_complete_ticks += complete;
    ++c.count;
}
} // namespace

void ImuDiagnostics::observe(const device::Bmi088 &imu,
                             const device::Bmi088SampleRecord &record,
                             bool gyroscope) noexcept
{
    const auto request = g_gyro_capture_request;
    if (request == 1U || request == 2U)
    {
        g_gyro_capture_request = 0U;
        g_gyro_capture.count = g_gyro_capture.stored = 0U;
        g_gyro_capture.sequence_gaps = 0U;
        g_gyro_capture.elapsed_ticks = 0U;
        g_gyro_capture.min_interval = UINT32_MAX;
        g_gyro_capture.max_interval = 0U;
        for (auto &c : g_gyro_capture.channels)
        {
            c.count = c.read_gaps = c.drdy_gaps = 0U;
            c.last_read = c.last_drdy = c.last_tick = 0U;
            c.coalesced_start = c.coalesced_end = 0U;
            c.errors_start = c.errors_end = 0U;
            c.overflow_start = c.overflow_end = c.high_water = 0U;
            c.late_starts = c.ambiguous = 0U;
            c.max_start_ticks = c.max_complete_ticks = c.max_harvest_ticks = 0U;
            c.elapsed_ticks = c.sum_start_ticks = c.sum_complete_ticks = 0U;
        }
        for (unsigned i = 0; i < 3; ++i)
        {
            g_gyro_capture.sum[i] = 0;
            g_gyro_capture.sum_squared[i] = 0;
            g_gyro_capture.minimum[i] = INT16_MAX;
            g_gyro_capture.maximum[i] = INT16_MIN;
        }
        g_temperature_qualified = false;
        g_gyro_capture.tick_hz = platform::Time::tick_frequency_hz();
        g_gyro_capture.state = request == 2U ? 1U : 0U;
    }
    device::Bmi088Temperature::Sample temperature{};
    if (!imu.get_temperature().try_get_sample(temperature))
        return;
    const float celsius = device::Bmi088Temperature::to_celsius(temperature.raw);
    observe_thermal_profile(imu, record, gyroscope, celsius);
    if (g_gyro_capture.state == 2U)
        return;
    if (g_gyro_capture.state == 0U)
    {
        // 温度资格与服务点保持一致，否则本诊断在服务点变更后永远不会就绪。
        if (!std::isfinite(celsius) || std::fabs(celsius - device::Bmi088Heater::DEFAULT_TARGET_CELSIUS) > 0.5F)
        {
            g_temperature_qualified = false;
            return;
        }
        if (!g_temperature_qualified)
        {
            g_temperature_since = platform::Time::now_ticks();
            g_temperature_qualified = true;
        }
        if (!platform::Time::has_elapsed_ms(g_temperature_since, 3000U))
            return;
        g_gyro_capture.tick_hz = platform::Time::tick_frequency_hz();
        g_gyro_capture.state = 1U;
    }
    observe_channel(imu, record, gyroscope);
    if (!gyroscope)
        return;
    const auto &gyro = imu.get_gyroscope();
    const auto sequence = record.sequence;
    const auto tick = record.drdy_tick; // MCU 的 DRDY 观测时刻，并非传感器内部时刻
    const auto &xyz = record.xyz;
    const auto count = g_gyro_capture.count;
    if (count == 0U)
    {
        g_gyro_capture.first_tick = tick;
        g_gyro_capture.temperature_start = celsius;
        g_gyro_capture.drdy_start = imu.get_gyroscope_data_ready_count();
        g_gyro_capture.coalesced_start = imu.get_gyroscope_coalesced_event_count();
        g_gyro_capture.errors_start = gyro.get_error_count();
    }
    else
    {
        const auto interval = tick - g_gyro_capture.last_tick;
        g_gyro_capture.elapsed_ticks += interval;
        if (interval < g_gyro_capture.min_interval)
            g_gyro_capture.min_interval = interval;
        if (interval > g_gyro_capture.max_interval)
            g_gyro_capture.max_interval = interval;
        g_gyro_capture.sequence_gaps += sequence - g_gyro_capture.last_sequence - 1U;
    }
    for (unsigned i = 0; i < 3; ++i)
    {
        const std::int64_t value = xyz[i];
        g_gyro_capture.sum[i] += value;
        g_gyro_capture.sum_squared[i] += static_cast<std::uint64_t>(value * value);
        if (xyz[i] < g_gyro_capture.minimum[i])
            g_gyro_capture.minimum[i] = xyz[i];
        if (xyz[i] > g_gyro_capture.maximum[i])
            g_gyro_capture.maximum[i] = xyz[i];
    }
    if (count < STORED_SAMPLES)
    {
        g_gyro_capture.records[count].tick = tick;
        g_gyro_capture.records[count].sequence = sequence;
        for (unsigned i = 0; i < 3; ++i)
            g_gyro_capture.records[count].xyz[i] = xyz[i];
        g_gyro_capture.records[count].reserved = record.flags;
        g_gyro_capture.stored = count + 1U;
    }
    g_gyro_capture.last_tick = tick;
    g_gyro_capture.last_sequence = sequence;
    g_gyro_capture.count = count + 1U;
    if (g_gyro_capture.count == CAPTURE_SAMPLES)
    {
        g_gyro_capture.temperature_end = celsius;
        g_gyro_capture.drdy_end = imu.get_gyroscope_data_ready_count();
        g_gyro_capture.coalesced_end = imu.get_gyroscope_coalesced_event_count();
        g_gyro_capture.errors_end = gyro.get_error_count();
        g_gyro_capture.state = 2U;
    }
}
#else
void ImuDiagnostics::observe(const device::Bmi088 &, const device::Bmi088SampleRecord &, bool) noexcept {}
#endif
} // namespace application
