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
constexpr std::uint32_t CaptureSamples{20000U};
constexpr std::uint32_t StoredSamples{1024U};
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
  std::uint32_t count{0U}, readGaps{0U}, drdyGaps{0U};
  std::uint32_t lastRead{0U}, lastDrdy{0U}, lastTick{0U};
  std::uint32_t coalescedStart{0U}, coalescedEnd{0U};
  std::uint32_t errorsStart{0U}, errorsEnd{0U};
  std::uint32_t overflowStart{0U}, overflowEnd{0U}, highWater{0U};
  std::uint32_t lateStarts{0U}, ambiguous{0U};
  std::uint32_t maxStartTicks{0U}, maxCompleteTicks{0U}, maxHarvestTicks{0U};
  std::uint64_t elapsedTicks{0U}, sumStartTicks{0U}, sumCompleteTicks{0U};
};
static_assert(sizeof(ChannelCapture) == 96U);
struct Capture
{
  std::uint32_t version{2U};
  std::uint32_t state{0U}; // 0：等待温度；1：记录中；2：已冻结
  std::uint32_t count{0U};
  std::uint32_t stored{0U};
  std::uint32_t tickHz{0U};
  std::uint32_t firstTick{0U};
  std::uint32_t lastTick{0U};
  std::uint32_t sequenceGaps{0U};
  std::uint32_t minInterval{UINT32_MAX};
  std::uint32_t maxInterval{0U};
  std::uint32_t drdyStart{0U}, drdyEnd{0U};
  std::uint32_t coalescedStart{0U}, coalescedEnd{0U};
  std::uint32_t errorsStart{0U}, errorsEnd{0U};
  float temperatureStart{0.0F}, temperatureEnd{0.0F};
  std::int64_t sum[3]{};
  std::uint64_t sumSquared[3]{};
  std::int16_t minimum[3]{INT16_MAX, INT16_MAX, INT16_MAX};
  std::int16_t maximum[3]{INT16_MIN, INT16_MIN, INT16_MIN};
  std::uint32_t lastSequence{0U};
  std::uint64_t elapsedTicks{0U}; // 采集时长可能超过 32 位 DWT 回绕周期
  RawRecord records[StoredSamples]{};
  ChannelCapture channels[2]{}; // 加速度计、陀螺仪；保留 v1 前缀
};
static_assert(sizeof(Capture) == 16720U);
// 头部非零初始化使其落入普通 .data（AXI SRAM），
// 不占用 DTCM/RTOS 堆。冻结状态只在全部数据
// 就绪后写入。
volatile Capture gyroCapture{};
// 1：带温度条件重新武装；2：仅计时，无温度门限。
// 两条命令都不改变加热器或传感器设置。
volatile std::uint32_t gyroCaptureRequest{0U};
platform::Time::Tick temperatureSince{0U};
bool temperatureQualified{false};

// 显式武装的只读热特性采集。一块约一秒；本报告的任何数值
// 都不会回馈进 IMU 或估计器。
constexpr std::uint32_t ThermalProfileBlocks{360U};
struct ProfileMoments
{
  std::uint32_t count, reserved;
  std::int64_t sum[3];
  std::uint64_t sumSquared[3];
};
static_assert(sizeof(ProfileMoments) == 56U);
struct ThermalProfileBlock
{
  std::uint32_t firstTick, lastTick;
  float temperatureMin, temperatureMax;
  std::uint32_t temperatureReadsStart, temperatureReadsEnd;
  std::uint32_t sampleFlags, reserved;
  ProfileMoments accelerometer, gyroscope;
};
static_assert(sizeof(ThermalProfileBlock) == 144U);
struct ThermalProfile
{
  std::uint32_t version;
  std::uint32_t state; // 0 空闲，1 记录中，2 已冻结
  std::uint32_t completedBlocks;
  std::uint32_t targetBlocks;
  std::uint32_t tickHz;
  std::uint32_t temperatureReadsStart, temperatureReadsEnd;
  std::uint32_t gyroErrorsStart, gyroErrorsEnd;
  std::uint32_t accelErrorsStart, accelErrorsEnd;
  std::uint32_t gyroOverflowStart, gyroOverflowEnd;
  std::uint32_t accelOverflowStart, accelOverflowEnd;
  std::uint32_t gyroCoalescedStart, gyroCoalescedEnd;
  std::uint32_t accelCoalescedStart, accelCoalescedEnd;
  ThermalProfileBlock blocks[ThermalProfileBlocks];
};
static_assert(sizeof(ThermalProfile) == 51920U);
// 现有链接脚本只提供 NOLOAD 的 AXI-SRAM（RAM_D1）段 `.dma_buffer*`，本诊断存档
// 由 CPU 独占、从不交给 DMA，因此单独用一个子段名 `.dma_buffer.imu_diag`
// 与真正的 DMA 缓冲区分开，同时仍落在 RAM_D1（DTCM 放不下这 52 KB）。
__attribute__((section(".dma_buffer.imu_diag"), aligned(32))) volatile ThermalProfile gyroThermalProfile{};
// 调试器邮箱：复位后写 1 一次即开始新一轮采集。
volatile std::uint32_t gyroThermalProfileRequest{0U};

void ResetMoments(volatile ProfileMoments &moments)
{
  moments.count = moments.reserved = 0U;
  for (unsigned i = 0; i < 3; ++i)
  {
    moments.sum[i] = 0;
    moments.sumSquared[i] = 0U;
  }
}

void AddMoments(volatile ProfileMoments &moments, const device::Bmi088SampleRecord &record)
{
  ++moments.count;
  for (unsigned i = 0; i < 3; ++i)
  {
    const auto value = static_cast<std::int32_t>(record.xyz[i]);
    moments.sum[i] += value;
    moments.sumSquared[i] += static_cast<std::uint32_t>(value * value);
  }
}

void ResetThermalBlock(volatile ThermalProfileBlock &block,
                       const device::Bmi088SampleRecord &record,
                       float temperature,
                       std::uint32_t temperatureReads)
{
  block.firstTick = block.lastTick = record.drdyTick;
  block.temperatureMin = block.temperatureMax = temperature;
  block.temperatureReadsStart = block.temperatureReadsEnd = temperatureReads;
  block.sampleFlags = block.reserved = 0U;
  ResetMoments(block.accelerometer);
  ResetMoments(block.gyroscope);
}

void FreezeThermalProfile(const device::Bmi088 &imu)
{
  gyroThermalProfile.temperatureReadsEnd = imu.GetTemperature().GetReadCount();
  gyroThermalProfile.gyroErrorsEnd = imu.GetGyroscope().GetErrorCount();
  gyroThermalProfile.accelErrorsEnd = imu.GetAccelerometer().GetErrorCount();
  gyroThermalProfile.gyroOverflowEnd = imu.GetGyroscopeQueue().OverflowCount();
  gyroThermalProfile.accelOverflowEnd = imu.GetAccelerometerQueue().OverflowCount();
  gyroThermalProfile.gyroCoalescedEnd = imu.GetGyroscopeCoalescedEventCount();
  gyroThermalProfile.accelCoalescedEnd = imu.GetAccelerometerCoalescedEventCount();
  gyroThermalProfile.state = 2U;
}

void ObserveThermalProfile(const device::Bmi088 &imu,
                           const device::Bmi088SampleRecord &record,
                           bool gyroscope,
                           float temperature)
{
  if (gyroThermalProfileRequest == 1U)
  {
    gyroThermalProfileRequest = 0U;
    gyroThermalProfile.version = 3U;
    gyroThermalProfile.state = 1U;
    gyroThermalProfile.completedBlocks = 0U;
    gyroThermalProfile.targetBlocks = ThermalProfileBlocks;
    gyroThermalProfile.tickHz = platform::Time::TickFrequencyHz();
    gyroThermalProfile.temperatureReadsStart = gyroThermalProfile.temperatureReadsEnd =
        imu.GetTemperature().GetReadCount();
    gyroThermalProfile.gyroErrorsStart = gyroThermalProfile.gyroErrorsEnd = imu.GetGyroscope().GetErrorCount();
    gyroThermalProfile.accelErrorsStart = gyroThermalProfile.accelErrorsEnd = imu.GetAccelerometer().GetErrorCount();
    gyroThermalProfile.gyroOverflowStart = gyroThermalProfile.gyroOverflowEnd = imu.GetGyroscopeQueue().OverflowCount();
    gyroThermalProfile.accelOverflowStart = gyroThermalProfile.accelOverflowEnd =
        imu.GetAccelerometerQueue().OverflowCount();
    gyroThermalProfile.gyroCoalescedStart = gyroThermalProfile.gyroCoalescedEnd = imu.GetGyroscopeCoalescedEventCount();
    gyroThermalProfile.accelCoalescedStart = gyroThermalProfile.accelCoalescedEnd =
        imu.GetAccelerometerCoalescedEventCount();
    if (gyroThermalProfile.tickHz == 0U || !std::isfinite(temperature))
    {
      gyroThermalProfile.state = 0U;
      return;
    }
    ResetThermalBlock(gyroThermalProfile.blocks[0], record, temperature, imu.GetTemperature().GetReadCount());
  }
  if (gyroThermalProfile.state != 1U || !std::isfinite(temperature))
    return;
  auto index = gyroThermalProfile.completedBlocks;
  if (index >= gyroThermalProfile.targetBlocks)
  {
    FreezeThermalProfile(imu);
    return;
  }
  auto &block = gyroThermalProfile.blocks[index];
  if (static_cast<std::uint32_t>(record.drdyTick - block.firstTick) >= gyroThermalProfile.tickHz)
  {
    block.temperatureReadsEnd = imu.GetTemperature().GetReadCount();
    ++gyroThermalProfile.completedBlocks;
    index = gyroThermalProfile.completedBlocks;
    if (index >= gyroThermalProfile.targetBlocks)
    {
      FreezeThermalProfile(imu);
      return;
    }
    ResetThermalBlock(gyroThermalProfile.blocks[index], record, temperature, imu.GetTemperature().GetReadCount());
  }
  auto &current = gyroThermalProfile.blocks[index];
  current.lastTick = record.drdyTick;
  current.temperatureReadsEnd = imu.GetTemperature().GetReadCount();
  if (temperature < current.temperatureMin)
    current.temperatureMin = temperature;
  if (temperature > current.temperatureMax)
    current.temperatureMax = temperature;
  current.sampleFlags |= record.flags;
  AddMoments(gyroscope ? current.gyroscope : current.accelerometer, record);
}

void ObserveChannel(const device::Bmi088 &imu, const device::Bmi088SampleRecord &record, bool gyro)
{
  auto &c = gyroCapture.channels[gyro ? 1 : 0];
  const auto &queue = gyro ? imu.GetGyroscopeQueue() : imu.GetAccelerometerQueue();
  const auto coalesced = gyro ? imu.GetGyroscopeCoalescedEventCount() : imu.GetAccelerometerCoalescedEventCount();
  const auto errors = gyro ? imu.GetGyroscope().GetErrorCount() : imu.GetAccelerometer().GetErrorCount();
  if (c.count == 0U)
  {
    c.coalescedStart = coalesced;
    c.errorsStart = errors;
    c.overflowStart = queue.OverflowCount();
  }
  else
  {
    c.readGaps += record.sequence - c.lastRead - 1U;
    c.drdyGaps += record.drdySequence - c.lastDrdy - 1U;
    c.elapsedTicks += static_cast<std::uint32_t>(record.drdyTick - c.lastTick);
  }
  c.lastRead = record.sequence;
  c.lastDrdy = record.drdySequence;
  c.lastTick = record.drdyTick;
  c.coalescedEnd = coalesced;
  c.errorsEnd = errors;
  c.overflowEnd = queue.OverflowCount();
  c.highWater = queue.HighWater();
  if ((record.flags & device::Bmi088SampleRecord::LateStart) != 0U)
    ++c.lateStarts;
  if ((record.flags & device::Bmi088SampleRecord::NewerEventBeforeCompletion) != 0U)
    ++c.ambiguous;
  const auto start = record.startTick - record.drdyTick;
  const auto complete = record.completedTick - record.drdyTick;
  const auto harvest = record.harvestedTick - record.completedTick;
  if (start > c.maxStartTicks)
    c.maxStartTicks = start;
  if (complete > c.maxCompleteTicks)
    c.maxCompleteTicks = complete;
  if (harvest > c.maxHarvestTicks)
    c.maxHarvestTicks = harvest;
  c.sumStartTicks += start;
  c.sumCompleteTicks += complete;
  ++c.count;
}
} // namespace

void ImuDiagnostics::Observe(const device::Bmi088 &imu,
                             const device::Bmi088SampleRecord &record,
                             bool gyroscope) noexcept
{
  const auto request = gyroCaptureRequest;
  if (request == 1U || request == 2U)
  {
    gyroCaptureRequest = 0U;
    gyroCapture.count = gyroCapture.stored = 0U;
    gyroCapture.sequenceGaps = 0U;
    gyroCapture.elapsedTicks = 0U;
    gyroCapture.minInterval = UINT32_MAX;
    gyroCapture.maxInterval = 0U;
    for (auto &c : gyroCapture.channels)
    {
      c.count = c.readGaps = c.drdyGaps = 0U;
      c.lastRead = c.lastDrdy = c.lastTick = 0U;
      c.coalescedStart = c.coalescedEnd = 0U;
      c.errorsStart = c.errorsEnd = 0U;
      c.overflowStart = c.overflowEnd = c.highWater = 0U;
      c.lateStarts = c.ambiguous = 0U;
      c.maxStartTicks = c.maxCompleteTicks = c.maxHarvestTicks = 0U;
      c.elapsedTicks = c.sumStartTicks = c.sumCompleteTicks = 0U;
    }
    for (unsigned i = 0; i < 3; ++i)
    {
      gyroCapture.sum[i] = 0;
      gyroCapture.sumSquared[i] = 0;
      gyroCapture.minimum[i] = INT16_MAX;
      gyroCapture.maximum[i] = INT16_MIN;
    }
    temperatureQualified = false;
    gyroCapture.tickHz = platform::Time::TickFrequencyHz();
    gyroCapture.state = request == 2U ? 1U : 0U;
  }
  device::Bmi088Temperature::Sample temperature{};
  if (!imu.GetTemperature().TryGetSample(temperature))
    return;
  const float celsius = device::Bmi088Temperature::ToCelsius(temperature.raw);
  ObserveThermalProfile(imu, record, gyroscope, celsius);
  if (gyroCapture.state == 2U)
    return;
  if (gyroCapture.state == 0U)
  {
    // 温度资格与服务点保持一致，否则本诊断在服务点变更后永远不会就绪。
    if (!std::isfinite(celsius) || std::fabs(celsius - device::Bmi088Heater::DefaultTargetCelsius) > 0.5F)
    {
      temperatureQualified = false;
      return;
    }
    if (!temperatureQualified)
    {
      temperatureSince = platform::Time::NowTicks();
      temperatureQualified = true;
    }
    if (!platform::Time::HasElapsedMs(temperatureSince, 3000U))
      return;
    gyroCapture.tickHz = platform::Time::TickFrequencyHz();
    gyroCapture.state = 1U;
  }
  ObserveChannel(imu, record, gyroscope);
  if (!gyroscope)
    return;
  const auto &gyro = imu.GetGyroscope();
  const auto sequence = record.sequence;
  const auto tick = record.drdyTick; // MCU 的 DRDY 观测时刻，并非传感器内部时刻
  const auto &xyz = record.xyz;
  const auto count = gyroCapture.count;
  if (count == 0U)
  {
    gyroCapture.firstTick = tick;
    gyroCapture.temperatureStart = celsius;
    gyroCapture.drdyStart = imu.GetGyroscopeDataReadyCount();
    gyroCapture.coalescedStart = imu.GetGyroscopeCoalescedEventCount();
    gyroCapture.errorsStart = gyro.GetErrorCount();
  }
  else
  {
    const auto interval = tick - gyroCapture.lastTick;
    gyroCapture.elapsedTicks += interval;
    if (interval < gyroCapture.minInterval)
      gyroCapture.minInterval = interval;
    if (interval > gyroCapture.maxInterval)
      gyroCapture.maxInterval = interval;
    gyroCapture.sequenceGaps += sequence - gyroCapture.lastSequence - 1U;
  }
  for (unsigned i = 0; i < 3; ++i)
  {
    const std::int64_t value = xyz[i];
    gyroCapture.sum[i] += value;
    gyroCapture.sumSquared[i] += static_cast<std::uint64_t>(value * value);
    if (xyz[i] < gyroCapture.minimum[i])
      gyroCapture.minimum[i] = xyz[i];
    if (xyz[i] > gyroCapture.maximum[i])
      gyroCapture.maximum[i] = xyz[i];
  }
  if (count < StoredSamples)
  {
    gyroCapture.records[count].tick = tick;
    gyroCapture.records[count].sequence = sequence;
    for (unsigned i = 0; i < 3; ++i)
      gyroCapture.records[count].xyz[i] = xyz[i];
    gyroCapture.records[count].reserved = record.flags;
    gyroCapture.stored = count + 1U;
  }
  gyroCapture.lastTick = tick;
  gyroCapture.lastSequence = sequence;
  gyroCapture.count = count + 1U;
  if (gyroCapture.count == CaptureSamples)
  {
    gyroCapture.temperatureEnd = celsius;
    gyroCapture.drdyEnd = imu.GetGyroscopeDataReadyCount();
    gyroCapture.coalescedEnd = imu.GetGyroscopeCoalescedEventCount();
    gyroCapture.errorsEnd = gyro.GetErrorCount();
    gyroCapture.state = 2U;
  }
}
#else
void ImuDiagnostics::Observe(const device::Bmi088 &, const device::Bmi088SampleRecord &, bool) noexcept {}
#endif
} // namespace application
