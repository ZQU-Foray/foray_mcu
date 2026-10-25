#include "Application/Imu/AttitudeEstimator.hpp"
#include "Libraries/Algorithm/alg_estimate/ImuBodyMapping.hpp"
#include "Libraries/Algorithm/alg_estimate/Vqf.hpp"
#include "Platform/Interface/Time.hpp"
#include <cmath>
#include <cstddef>

namespace application
{

namespace
{

alg_estimate::Vqf estimator;
GyroBiasCalibration calibration;
bool ready{false};
bool calibrationInjected{false};
std::int32_t yawTurns{0};
float lastYawDegrees{0.0F};
bool yawAnchored{false};

// 时序状态：陀螺节拍与最近的加速度样本。
bool gyroAnchored{false};
std::uint32_t lastGyroTick{0U};
std::uint32_t lastGyroSequence{0U};
std::uint32_t lastAccelSequence{0U};
std::uint32_t lastAccelTick{0U};
bool accelAvailable{false};
alg_math::Vector3 accelBody{};

// 调试器可见存档。写读都走字节拷贝，避免逐字段复制漏项。
volatile AttitudeEstimator::Output outputArchive{};
volatile AttitudeEstimator::Statistics statisticsArchive{};
static_assert(sizeof(AttitudeEstimator::Output) == sizeof(outputArchive), "存档布局必须一致");

// 整字访问必须允许别名到浮点字段，否则优化构建可能读到过期状态。
typedef std::uint32_t ArchiveWord __attribute__((__may_alias__));

void PublishArchive(const AttitudeEstimator::Output &source) noexcept
{
  // 整字拷贝：本函数在 2 kHz 陀螺节拍上执行，逐字节写 volatile 会让固定开销翻几倍。
  // Output 全由 32 位成员组成（sizeof 已被 static_assert 钉死为 120），对齐有保证。
  static_assert((sizeof(AttitudeEstimator::Output) % sizeof(std::uint32_t)) == 0U, "存档布局必须能按 32 位整字拷贝");
  const ArchiveWord *sourceWords = reinterpret_cast<const ArchiveWord *>(&source);
  volatile ArchiveWord *targetWords = reinterpret_cast<volatile ArchiveWord *>(&outputArchive);
  for (std::size_t index = 0U; index < sizeof(source) / sizeof(ArchiveWord); ++index)
  {
    targetWords[index] = sourceWords[index];
  }
}

AttitudeEstimator::Output ReadArchive() noexcept
{
  AttitudeEstimator::Output result{};
  ArchiveWord *targetWords = reinterpret_cast<ArchiveWord *>(&result);
  const volatile ArchiveWord *sourceWords = reinterpret_cast<const volatile ArchiveWord *>(&outputArchive);
  for (std::size_t index = 0U; index < sizeof(result) / sizeof(ArchiveWord); ++index)
  {
    targetWords[index] = sourceWords[index];
  }
  return result;
}

void ResetArchives() noexcept
{
  AttitudeEstimator::Output output{};
  PublishArchive(output);
  AttitudeEstimator::Statistics statistics{};
  const std::uint8_t *bytes = reinterpret_cast<const std::uint8_t *>(&statistics);
  volatile std::uint8_t *target = reinterpret_cast<volatile std::uint8_t *>(&statisticsArchive);
  for (std::size_t index = 0U; index < sizeof(statistics); ++index)
  {
    target[index] = bytes[index];
  }
}

/** 时间戳标志不确定的样本（晚启动或完成前又来了新的 DRDY）视为时序不可信。 */
bool HasUncertainTiming(const device::Bmi088SampleRecord &record)
{
  return (record.flags &
          (device::Bmi088SampleRecord::LateStart | device::Bmi088SampleRecord::NewerEventBeforeCompletion)) != 0U;
}

void PublishOutput(const device::Bmi088SampleRecord &record)
{
  const alg_estimate::Vqf::State state = estimator.GetState();
  const alg_estimate::Vqf::RestObservables rest = estimator.GetRestObservables();
  const GyroBiasCalibration::Snapshot calibrationSnapshot = calibration.GetSnapshot();
  const bool valid = state.valid && alg_math::IsFinite(state.quaternion) &&
                     alg_math::IsFinite(state.gyroBiasRadPerSec) && alg_math::IsFinite(state.eulerDegrees) &&
                     std::isfinite(state.biasUncertaintyRadPerSec);

  if (valid)
  {
    if (yawAnchored)
    {
      const float delta = state.eulerDegrees.x - lastYawDegrees;
      if (delta > 180.0F)
      {
        --yawTurns;
      }
      else if (delta < -180.0F)
      {
        ++yawTurns;
      }
    }
    yawAnchored = true;
    lastYawDegrees = state.eulerDegrees.x;
  }

  AttitudeEstimator::Output output{};
  output.version = AttitudeEstimator::OutputVersion;
  output.updateTick = record.drdyTick;
  output.quaternion[0] = state.quaternion.w;
  output.quaternion[1] = state.quaternion.x;
  output.quaternion[2] = state.quaternion.y;
  output.quaternion[3] = state.quaternion.z;
  output.eulerDegrees[0] = state.eulerDegrees.x;
  output.eulerDegrees[1] = state.eulerDegrees.y;
  output.eulerDegrees[2] = state.eulerDegrees.z;
  output.gyroBiasRadPerSec[0] = state.gyroBiasRadPerSec.x;
  output.gyroBiasRadPerSec[1] = state.gyroBiasRadPerSec.y;
  output.gyroBiasRadPerSec[2] = state.gyroBiasRadPerSec.z;
  output.yawTotalDegrees = 360.0F * static_cast<float>(yawTurns) + lastYawDegrees;
  output.biasUncertaintyRadPerSec = state.biasUncertaintyRadPerSec;
  output.updateCount = state.updateCount;
  output.gyroscopeDeviationRatio = rest.gyroscopeDeviationRatio;
  output.valid = valid ? 1U : 0U;
  output.restDetected = state.restDetected ? 1U : 0U;
  output.lastAccelCorrectionRadians = state.lastAccelCorrectionRadians;
  output.accelerometerDeviationRatio = rest.accelerometerDeviationRatio;
  output.calibrationBiasRadPerSec[0] = calibrationSnapshot.biasRadPerSec.x;
  output.calibrationBiasRadPerSec[1] = calibrationSnapshot.biasRadPerSec.y;
  output.calibrationBiasRadPerSec[2] = calibrationSnapshot.biasRadPerSec.z;
  output.calibrationSpreadRadPerSec[0] = calibrationSnapshot.spreadRadPerSec.x;
  output.calibrationSpreadRadPerSec[1] = calibrationSnapshot.spreadRadPerSec.y;
  output.calibrationSpreadRadPerSec[2] = calibrationSnapshot.spreadRadPerSec.z;
  output.calibrationState = static_cast<std::uint32_t>(calibrationSnapshot.state);
  output.calibrationSamples = calibrationSnapshot.samples;
  output.calibrationTemperatureCelsius = calibrationSnapshot.temperatureCelsius;
  output.calibrationElapsedMs = calibrationSnapshot.elapsedMs;
  PublishArchive(output);

  statisticsArchive.calibrationRestarts = calibrationSnapshot.restarts;
}

void ProcessGyroscope(const device::Bmi088SampleRecord &record, float temperatureCelsius) noexcept
{
  if (record.sequence <= lastGyroSequence)
  {
    ++statisticsArchive.rejectedSequence;
    return;
  }
  lastGyroSequence = record.sequence;

  if (!gyroAnchored)
  {
    // 首个陀螺样本只建立时间基准，不积分。
    lastGyroTick = record.drdyTick;
    gyroAnchored = true;
    return;
  }

  const std::uint32_t elapsedTicks = record.drdyTick - lastGyroTick;
  lastGyroTick = record.drdyTick;
  const float stepSeconds = platform::Time::TicksToSeconds(elapsedTicks);
  if (!(stepSeconds > 0.0F) || stepSeconds > AttitudeEstimator::GyroGapLimitSeconds)
  {
    // 断流：不虚构缺失运动，仅重新建立基准。
    ++statisticsArchive.gyroGaps;
    return;
  }

  float gyroBody[3]{};
  alg_estimate::GyroRecordToBody(record, gyroBody);
  const alg_math::Vector3 gyro{gyroBody[0], gyroBody[1], gyroBody[2]};

  const alg_math::Vector3 gateAccel = accelAvailable ? accelBody : alg_math::Vector3{};
  calibration.Process(gyro, gateAccel, temperatureCelsius, record.drdyTick);
  if (!calibrationInjected && calibration.IsBiasValid())
  {
    estimator.SetBiasEstimate(calibration.GetBiasRadPerSec());
    calibrationInjected = true;
  }

  if (!accelAvailable || platform::Time::TicksToSeconds(record.drdyTick - lastAccelTick) >
                             AttitudeEstimator::AccelerometerStaleLimitSeconds)
  {
    ++statisticsArchive.accelStale;
  }
  estimator.UpdateGyroscope(gyro);
  ++statisticsArchive.gyroConsumed;
  PublishOutput(record);
}

void ProcessAccelerometer(const device::Bmi088SampleRecord &record) noexcept
{
  if (record.sequence <= lastAccelSequence)
  {
    ++statisticsArchive.rejectedSequence;
    return;
  }
  lastAccelSequence = record.sequence;
  lastAccelTick = record.drdyTick;
  float accel[3]{};
  alg_estimate::AccelRecordToBody(record, accel);
  accelBody = alg_math::Vector3{accel[0], accel[1], accel[2]};
  accelAvailable = true;
  estimator.UpdateAccelerometer(accelBody);
  ++statisticsArchive.accelConsumed;
}

} // namespace

bool AttitudeEstimator::Init() noexcept
{
  GyroBiasCalibration::Config calibrationConfig{};
  calibrationConfig.tickFrequencyHz = platform::Time::TickFrequencyHz();
  calibration.Init(calibrationConfig);

  alg_estimate::Vqf::Config config{};
  // 532 Hz 带宽下本板静止三轴瞬时偏离高于上游 2°/s 默认门槛；先按板测候选值验收。
  config.restThresholdGyrRadPerSec = 0.35F;
  // 标定允许的三轴均值不能在 VQF 首次零偏更新时被限幅截断。
  config.biasClipRadPerSec = calibrationConfig.maximumMeanRateRadPerSec;
  constexpr float gyroPeriodSeconds =
      device::Bmi088Gyro::BandwidthRegisterValue == device::Bmi088Gyro::GyroBandwidth2000Hz532Hz ? 1.0F / 2000.0F
                                                                                                 : 1.0F / 1000.0F;
  ready = estimator.Init(config, gyroPeriodSeconds, 1.0F / 1600.0F);

  calibrationInjected = false;
  yawTurns = 0;
  lastYawDegrees = 0.0F;
  yawAnchored = false;
  gyroAnchored = false;
  lastGyroTick = 0U;
  lastGyroSequence = 0U;
  lastAccelSequence = 0U;
  lastAccelTick = 0U;
  accelAvailable = false;
  accelBody = alg_math::Vector3{};
  ResetArchives();
  return ready;
}

bool AttitudeEstimator::IsReady() noexcept
{
  return ready;
}

void AttitudeEstimator::Process(const device::Bmi088SampleRecord &record,
                                bool gyroscope,
                                float temperatureCelsius) noexcept
{
  if (gyroscope)
  {
    ++statisticsArchive.gyroReceived;
  }
  else
  {
    ++statisticsArchive.accelReceived;
  }

  if (!ready)
  {
    return;
  }
  if (HasUncertainTiming(record))
  {
    ++statisticsArchive.rejectedFlags;
    return;
  }

  if (gyroscope)
  {
    ProcessGyroscope(record, temperatureCelsius);
  }
  else
  {
    ProcessAccelerometer(record);
  }
}

AttitudeEstimator::Output AttitudeEstimator::GetOutput() noexcept
{
  return ReadArchive();
}

AttitudeEstimator::Statistics AttitudeEstimator::GetStatistics() noexcept
{
  Statistics result{};
  std::uint8_t *bytes = reinterpret_cast<std::uint8_t *>(&result);
  const volatile std::uint8_t *source = reinterpret_cast<const volatile std::uint8_t *>(&statisticsArchive);
  for (std::size_t index = 0U; index < sizeof(result); ++index)
  {
    bytes[index] = source[index];
  }
  return result;
}

} // namespace application
