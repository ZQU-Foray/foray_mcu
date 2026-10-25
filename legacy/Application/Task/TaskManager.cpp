#include "Application/Task/TaskManager.hpp"
#include "Application/Task/0_imu/ImuTask.hpp"
#include "Application/Task/1_chassis/ChassisTask.hpp"
#include "Application/Task/2_gimbal/GimbalTask.hpp"
#include "Platform/Interface/Time.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include <cstddef>
#include <cstdint>

namespace application::task
{

namespace
{

constexpr std::uint32_t ImuTaskStackSizeBytes{3072U};
constexpr std::uint32_t TaskStackSizeBytes{4096U};

/**
 * @brief 应用任务表条目：把"入口、属性、句柄落在哪里"绑定在一起。
 * @note 句柄指针指向文件作用域变量，便于调试器与验收脚本直接读取。
 */
struct TaskSpec final
{
  const char *name;
  osThreadFunc_t entry;
  const osThreadAttr_t *attributes;
  osThreadId_t *handle;
};

osThreadId_t imuTaskHandle{nullptr};
osThreadId_t chassisTaskHandle{nullptr};
osThreadId_t gimbalTaskHandle{nullptr};

const osThreadAttr_t imuTaskAttributes{
    "imuTask", 0U, nullptr, 0U, nullptr, ImuTaskStackSizeBytes, osPriorityHigh, 0U, 0U};
const osThreadAttr_t chassisTaskAttributes{
    "chassisTask", 0U, nullptr, 0U, nullptr, TaskStackSizeBytes, osPriorityAboveNormal, 0U, 0U};
const osThreadAttr_t gimbalTaskAttributes{
    "gimbalTask", 0U, nullptr, 0U, nullptr, TaskStackSizeBytes, osPriorityNormal, 0U, 0U};

constexpr std::size_t TaskCount{3U};
const TaskSpec taskSpecs[TaskCount]{
    {"imuTask", &ImuTask::Run, &imuTaskAttributes, &imuTaskHandle},
    {"chassisTask", &ChassisTask::Run, &chassisTaskAttributes, &chassisTaskHandle},
    {"gimbalTask", &GimbalTask::Run, &gimbalTaskAttributes, &gimbalTaskHandle},
};

static_assert(static_cast<int>(osPriorityHigh) > static_cast<int>(osPriorityAboveNormal) &&
                  static_cast<int>(osPriorityAboveNormal) > static_cast<int>(osPriorityNormal),
              "任务优先级次序必须保持 IMU > 底盘 > 云台");

TaskManager::State managerState{TaskManager::State::Uninitialized};
TaskManager::RtosInitTiming rtosInitTiming{TaskManager::RtosInitTiming::None};
TaskManager::FailureStage failureStage{TaskManager::FailureStage::None};
std::uint32_t createdTaskCount{0U};
std::uint32_t startRefusedCount{0U};

// 栈余量巡检：1 Hz 限频，快照写入外部链接的 volatile 存档，调试器与验收脚本
// 可直接按符号读取（与 AttitudeEstimator 的存档同一模式）。
constexpr std::uint32_t StackHealthSamplePeriodMs{1000U};
// 告警阈值：剩余不足 256 B 时置位 lowMarginTaskIndex，提示余量已进危险区。
constexpr std::uint32_t StackHealthWarningFreeWords{64U};
std::uint32_t stackHealthSampleCount{0U};
platform::Time::Tick lastStackHealthTick{0U};
bool stackHealthAnchored{false};

/** @brief 记录失败环节并进入不可自动恢复的 Error 状态。 */
void EnterError(TaskManager::FailureStage stage) noexcept
{
  failureStage = stage;
  managerState = TaskManager::State::Error;
}

} // namespace

volatile TaskManager::StackHealth stackHealthArchive{};

bool TaskManager::Init() noexcept
{
  if (managerState == State::PreKernelReady || managerState == State::RtosReady || managerState == State::Running)
  {
    return true;
  }
  if (managerState == State::Error)
  {
    return false;
  }

  // 三个任务都以 DWT 时基为 dt 来源：时基不可用时任务会在每拍判 dt 非法并静默
  // 输出零力矩，必须在启动路径上直接拦住。
  if (!platform::Time::IsReady())
  {
    EnterError(FailureStage::Platform);
    return false;
  }

  // 按任务表依次初始化各任务的非 RTOS 部分。
  if (!ImuTask::Init())
  {
    EnterError(FailureStage::ImuTask);
    return false;
  }
  if (!ChassisTask::Init())
  {
    EnterError(FailureStage::ChassisTask);
    return false;
  }
  if (!GimbalTask::Init())
  {
    EnterError(FailureStage::GimbalTask);
    return false;
  }

  managerState = State::PreKernelReady;
  return true;
}

bool TaskManager::InitRtos() noexcept
{
  if (managerState == State::RtosReady || managerState == State::Running)
  {
    return true;
  }
  if (!Init())
  {
    return false;
  }

  // 记录创建时机：PreKernel 是设计口径，PostKernel 说明启动路径少接了一个入口。
  rtosInitTiming = (osKernelGetState() == osKernelRunning) ? RtosInitTiming::PostKernel : RtosInitTiming::PreKernel;

  for (std::size_t index = 0U; index < TaskCount; ++index)
  {
    *taskSpecs[index].handle = osThreadNew(taskSpecs[index].entry, nullptr, taskSpecs[index].attributes);
    if (*taskSpecs[index].handle == nullptr)
    {
      // 已创建的线程不在此处回收：进入 Error 后由上层在启动日志中看到失败环节，
      // 且不再推进到 Running，任务表不会以"部分就绪"的形态运行。
      EnterError(FailureStage::ThreadCreation);
      return false;
    }
    ++createdTaskCount;
  }

  managerState = State::RtosReady;
  return true;
}

bool TaskManager::Start() noexcept
{
  if (managerState == State::Running)
  {
    return true;
  }
  if (managerState != State::RtosReady || osKernelGetState() != osKernelRunning)
  {
    ++startRefusedCount;
    return false;
  }

  managerState = State::Running;
  return true;
}

TaskManager::State TaskManager::GetState() noexcept
{
  return managerState;
}

TaskManager::Diagnostics TaskManager::GetDiagnostics() noexcept
{
  Diagnostics diagnostics{};
  diagnostics.state = managerState;
  diagnostics.rtosInitTiming = rtosInitTiming;
  diagnostics.failure = failureStage;
  diagnostics.expectedTaskCount = static_cast<std::uint32_t>(TaskCount);
  diagnostics.createdTaskCount = createdTaskCount;
  diagnostics.startRefusedCount = startRefusedCount;
  return diagnostics;
}

void TaskManager::SampleStackHealth() noexcept
{
  // 任务表就绪前没有句柄可查，也不算一次采样。
  if (managerState != State::Running || createdTaskCount == 0U)
  {
    return;
  }

  const platform::Time::Tick now = platform::Time::NowTicks();
  if (stackHealthAnchored && !platform::Time::HasElapsedMs(lastStackHealthTick, StackHealthSamplePeriodMs))
  {
    return; // 限频：调用方可以每拍调用
  }
  stackHealthAnchored = true;
  lastStackHealthTick = now;

  StackHealth health{};
  health.minimumFreeWords = UINT32_MAX;
  health.lowMarginTaskIndex = MarginOkIndex;
  for (std::size_t index = 0U; index < TaskCount && index < MaximumSampledTasks; ++index)
  {
    const osThreadId_t handle = *taskSpecs[index].handle;
    if (handle == nullptr)
    {
      continue;
    }
    const UBaseType_t freeWords = uxTaskGetStackHighWaterMark(reinterpret_cast<TaskHandle_t>(handle));
    health.freeWords[index] = static_cast<std::uint32_t>(freeWords);
    ++health.sampledTaskCount;
    if (health.freeWords[index] < health.minimumFreeWords)
    {
      health.minimumFreeWords = health.freeWords[index];
      health.lowMarginTaskIndex = static_cast<std::uint32_t>(index);
    }
  }
  if (health.sampledTaskCount == 0U)
  {
    return; // 没有可查句柄：不发布误导性的全零快照
  }
  if (health.minimumFreeWords >= StackHealthWarningFreeWords)
  {
    health.lowMarginTaskIndex = MarginOkIndex;
  }
  health.sampleCount = ++stackHealthSampleCount;

  // 整字拷贝发布：读侧（调试器）可能随时读取，不追求原子性，只保证布局与快照一致。
  static_assert((sizeof(StackHealth) % sizeof(std::uint32_t)) == 0U, "栈余量快照必须可按 32 位整字拷贝");
  const std::uint32_t *source = reinterpret_cast<const std::uint32_t *>(&health);
  volatile std::uint32_t *target = reinterpret_cast<volatile std::uint32_t *>(&stackHealthArchive);
  for (std::size_t index = 0U; index < sizeof(health) / sizeof(std::uint32_t); ++index)
  {
    target[index] = source[index];
  }
}

TaskManager::StackHealth TaskManager::GetStackHealth() noexcept
{
  StackHealth health{};
  const volatile std::uint32_t *source = reinterpret_cast<const volatile std::uint32_t *>(&stackHealthArchive);
  std::uint32_t *target = reinterpret_cast<std::uint32_t *>(&health);
  for (std::size_t index = 0U; index < sizeof(health) / sizeof(std::uint32_t); ++index)
  {
    target[index] = source[index];
  }
  return health;
}

} // namespace application::task
