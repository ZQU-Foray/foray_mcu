#include "Application/Task/0_imu/ImuTask.hpp"
#include "Application/Imu/AttitudeEstimator.hpp"
#include "Application/Imu/GyroBiasCalibration.hpp"
#include "Application/Imu/ImuDiagnostics.hpp"
#include "Application/Task/0_imu/ImuDataReadyMailbox.hpp"
#include "Libraries/Device/bmi088/Bmi088.hpp"
#include "Libraries/Device/bmi088/Bmi088Heater.hpp"
#include "Platform/Interface/Gpio.hpp"
#include "cmsis_os2.h"
#include <atomic>
#include <cstdint>
#include <cstring>
#include <limits>

namespace application::task
{

namespace
{

constexpr std::uint32_t TaskPeriodTicks{1U};
constexpr std::uint32_t AccelerometerDataReadyFlag{1UL << 0U};
constexpr std::uint32_t GyroscopeDataReadyFlag{1UL << 1U};
constexpr std::uint32_t TransferCompletedFlag{1UL << 2U};
constexpr std::uint32_t DataReadyFlags{AccelerometerDataReadyFlag | GyroscopeDataReadyFlag | TransferCompletedFlag};

// 上电零偏标定的温度门控必须与恒温服务点一致；两者不一致时标定永远进不了
// 采样状态，z 轴零偏完全不会被扣除，yaw 直接积分原始零偏。
static_assert(application::GyroBiasCalibration::Config{}.targetCelsius == device::Bmi088Heater::DefaultTargetCelsius,
              "零偏标定温度门控必须与恒温目标一致");

// 一次唤醒内推进的采集轮数：每轮只做常数时间的边沿通知与传输衔接，
// 两轮足以在同一个唤醒里把加速度计与陀螺两路挂起边沿都接上。
// 这里刻意**不做**"只要还有标志就继续"的自旋：本任务优先级最高，
// 而 DRDY 边沿每 500/625 us 就会重来一次，自旋会让它永远不回等待点，
// 从而无界占用 CPU 并饿死 1 ms 的底盘/云台任务（现象是电机拿不到
// 力矩指令、完全不转）。
constexpr std::uint32_t AcquisitionRounds{2U};

// 慢路径每轮搬进跨任务队列的记录数上限：只做常数时间的 40 字节拷贝，
// 使采集任务每轮占用的长度与样本内容无关。本任务优先级高于 1 ms 的底盘/云台
// 电机任务，这个上限就是它们 CPU 预算的下界。
constexpr std::uint32_t SampleBudgetPerIteration{4U};

device::Bmi088 bmi088;
device::Bmi088Heater bmi088Heater;
ImuDataReadyMailbox accelerometerInterrupts, gyroscopeInterrupts;
bool initialized{false};
osThreadId_t taskHandle{nullptr};

#if defined(IMU_ENABLE_DIAGNOSTICS)
// 仅供调试器的暂存温度邮箱：先写 IEEE-754 位，再置
// command=1。默认/Release 行为仍是 50 C 服务约定。
std::atomic<std::uint32_t> bmi088HeaterDiagnosticTargetBits{0U};
std::atomic<std::uint32_t> bmi088HeaterDiagnosticCommand{0U};
struct HeaterDiagnosticReport
{
  std::uint32_t version{1U}, requests{0U}, accepted{0U}, rejected{0U};
  float requestedCelsius{0.0F}, activeCelsius{device::Bmi088Heater::DefaultTargetCelsius};
  std::uint32_t lastResult{0U}; // 0 无，1 已接受，2 已拒绝
};
volatile HeaterDiagnosticReport bmi088HeaterDiagnosticReport{};
#endif

void AccelerometerDataReadyCallback(void *context)
{
  (void)context;
  accelerometerInterrupts.Publish(platform::Time::NowTicks());
  if (taskHandle != nullptr)
  {
    (void)osThreadFlagsSet(taskHandle, AccelerometerDataReadyFlag);
  }
}

void GyroscopeDataReadyCallback(void *context)
{
  (void)context;
  gyroscopeInterrupts.Publish(platform::Time::NowTicks());
  if (taskHandle != nullptr)
  {
    (void)osThreadFlagsSet(taskHandle, GyroscopeDataReadyFlag);
  }
}

void TransferCompletedCallback(void *context)
{
  (void)context;
  if (taskHandle != nullptr)
    (void)osThreadFlagsSet(taskHandle, TransferCompletedFlag);
}

} // namespace

bool ImuTask::Init() noexcept
{
  bmi088.Init();
  if (!bmi088Heater.Init())
  {
    initialized = false;
    return false;
  }

  // 姿态解算不是采集的前置条件：滤波器参数无效时保持未就绪，
  // 由 AttitudeEstimator 的统计暴露，不阻断采集与恒温。
  (void)AttitudeEstimator::Init();

  bmi088Heater.SetEnabled(true);
  taskHandle = nullptr;
  initialized = true;
  return true;
}

[[noreturn]] void ImuTask::Run(void *argument) noexcept
{
  (void)argument;

  if (!initialized)
  {
    osThreadExit();
  }

  taskHandle = osThreadGetId();
  bmi088.SetCompletionNotification(TransferCompletedCallback, nullptr);
  if (!platform::Gpio::SetInterruptCallback(platform::Gpio::Pin::ImuAccelerometerInterrupt,
                                            AccelerometerDataReadyCallback) ||
      !platform::Gpio::SetInterruptCallback(platform::Gpio::Pin::ImuGyroscopeInterrupt, GyroscopeDataReadyCallback))
  {
    bmi088Heater.SetEnabled(false);
    osThreadExit();
  }

  for (;;)
  {
    // osThreadFlagsWait 会原子清掉本次唤醒对应的标志，不存在"Get 与 Clear 之间
    // 丢边沿"的竞态；即使漏掉一次唤醒也不会丢事件——DRDY 事件本体由邮箱的累计
    // 序号承载，NotifyDataReady 会按序号补齐。
    (void)osThreadFlagsWait(DataReadyFlags, osFlagsWaitAny, TaskPeriodTicks);

    // 采集衔接：固定轮数，只做常数时间工作；慢路径另有限量，保证一次唤醒的
    // 占用长度有上界，不会把低优先级的电机任务饿死。
    for (std::uint32_t round = 0U; round < AcquisitionRounds; ++round)
    {
      bmi088.NotifyDataReady(accelerometerInterrupts.Snapshot(), gyroscopeInterrupts.Snapshot());
      bmi088.Process();
    }

    const auto &temperature = bmi088.GetTemperature();
#if defined(IMU_ENABLE_DIAGNOSTICS)
    if (bmi088HeaterDiagnosticCommand.exchange(0U, std::memory_order_acq_rel) == 1U)
    {
      const auto bits = bmi088HeaterDiagnosticTargetBits.load(std::memory_order_relaxed);
      float target{};
      std::memcpy(&target, &bits, sizeof(target));
      ++bmi088HeaterDiagnosticReport.requests;
      bmi088HeaterDiagnosticReport.requestedCelsius = target;
      if (bmi088Heater.SetTargetCelsius(target))
      {
        ++bmi088HeaterDiagnosticReport.accepted;
        bmi088HeaterDiagnosticReport.activeCelsius = target;
        bmi088HeaterDiagnosticReport.lastResult = 1U;
      }
      else
      {
        ++bmi088HeaterDiagnosticReport.rejected;
        bmi088HeaterDiagnosticReport.lastResult = 2U;
      }
    }
#endif
    bmi088Heater.Process(temperature);

    // 上电零偏标定的温度门控需要温度读数；温度变化缓慢，每轮取一次即可。
    // 温度在这里抓取并随记录入队，消费者不再回头读驱动状态。
    device::Bmi088Temperature::Sample temperatureSample{};
    float temperatureCelsius = std::numeric_limits<float>::quiet_NaN();
    if (temperature.TryGetSample(temperatureSample))
    {
      temperatureCelsius = device::Bmi088Temperature::ToCelsius(temperatureSample.raw);
    }

    // 慢路径：诊断与姿态解算，每轮最多固定条数，剩余样本留到下一轮，
    // 队列容量足以吸收，保证采集链路不会被长时间的批处理阻塞。
    device::Bmi088SampleRecord record{};
    bool gyroscope{false};
    for (std::uint32_t processed = 0U; processed < SampleBudgetPerIteration; ++processed)
    {
      if (!bmi088.PopNextSample(record, gyroscope))
      {
        break;
      }
      application::ImuDiagnostics::Observe(bmi088, record, gyroscope);
      application::AttitudeEstimator::Process(record, gyroscope, temperatureCelsius);
    }
  }
}

} // namespace application::task
