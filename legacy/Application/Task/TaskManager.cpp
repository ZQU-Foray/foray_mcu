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

constexpr std::uint32_t IMU_TASK_STACK_SIZE_BYTES{3072U};
constexpr std::uint32_t TASK_STACK_SIZE_BYTES{4096U};

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

osThreadId_t g_imu_task_handle{nullptr};
osThreadId_t g_chassis_task_handle{nullptr};
osThreadId_t g_gimbal_task_handle{nullptr};

const osThreadAttr_t IMU_TASK_ATTRIBUTES{
    "imuTask", 0U, nullptr, 0U, nullptr, IMU_TASK_STACK_SIZE_BYTES, osPriorityHigh, 0U, 0U};
const osThreadAttr_t CHASSIS_TASK_ATTRIBUTES{
    "chassisTask", 0U, nullptr, 0U, nullptr, TASK_STACK_SIZE_BYTES, osPriorityAboveNormal, 0U, 0U};
const osThreadAttr_t GIMBAL_TASK_ATTRIBUTES{
    "gimbalTask", 0U, nullptr, 0U, nullptr, TASK_STACK_SIZE_BYTES, osPriorityNormal, 0U, 0U};

constexpr std::size_t TASK_COUNT{3U};
const TaskSpec TASK_SPECS[TASK_COUNT]{
    {"imuTask", &ImuTask::run, &IMU_TASK_ATTRIBUTES, &g_imu_task_handle},
    {"chassisTask", &ChassisTask::run, &CHASSIS_TASK_ATTRIBUTES, &g_chassis_task_handle},
    {"gimbalTask", &GimbalTask::run, &GIMBAL_TASK_ATTRIBUTES, &g_gimbal_task_handle},
};

static_assert(static_cast<int>(osPriorityHigh) > static_cast<int>(osPriorityAboveNormal) &&
                  static_cast<int>(osPriorityAboveNormal) > static_cast<int>(osPriorityNormal),
              "任务优先级次序必须保持 IMU > 底盘 > 云台");

TaskManager::State g_manager_state{TaskManager::State::UNINITIALIZED};
TaskManager::RtosInitTiming g_rtos_init_timing{TaskManager::RtosInitTiming::NONE};
TaskManager::FailureStage g_failure_stage{TaskManager::FailureStage::NONE};
std::uint32_t g_created_task_count{0U};
std::uint32_t g_start_refused_count{0U};

// 栈余量巡检：1 Hz 限频，快照写入外部链接的 volatile 存档，调试器与验收脚本
// 可直接按符号读取（与 AttitudeEstimator 的存档同一模式）。
constexpr std::uint32_t STACK_HEALTH_SAMPLE_PERIOD_MS{1000U};
// 告警阈值：剩余不足 256 B 时置位 low_margin_task_index，提示余量已进危险区。
constexpr std::uint32_t STACK_HEALTH_WARNING_FREE_WORDS{64U};
std::uint32_t g_stack_health_sample_count{0U};
platform::Time::Tick g_last_stack_health_tick{0U};
bool g_stack_health_anchored{false};

/** @brief 记录失败环节并进入不可自动恢复的 Error 状态。 */
void enter_error(TaskManager::FailureStage stage) noexcept
{
    g_failure_stage = stage;
    g_manager_state = TaskManager::State::ERROR;
}

} // namespace

volatile TaskManager::StackHealth g_stack_health_archive{};

bool TaskManager::init() noexcept
{
    if (g_manager_state == State::PRE_KERNEL_READY || g_manager_state == State::RTOS_READY ||
        g_manager_state == State::RUNNING)
    {
        return true;
    }
    if (g_manager_state == State::ERROR)
    {
        return false;
    }

    // 三个任务都以 DWT 时基为 dt 来源：时基不可用时任务会在每拍判 dt 非法并静默
    // 输出零力矩，必须在启动路径上直接拦住。
    if (!platform::Time::is_ready())
    {
        enter_error(FailureStage::PLATFORM);
        return false;
    }

    // 按任务表依次初始化各任务的非 RTOS 部分。
    if (!ImuTask::init())
    {
        enter_error(FailureStage::IMU_TASK);
        return false;
    }
    if (!ChassisTask::init())
    {
        enter_error(FailureStage::CHASSIS_TASK);
        return false;
    }
    if (!GimbalTask::init())
    {
        enter_error(FailureStage::GIMBAL_TASK);
        return false;
    }

    g_manager_state = State::PRE_KERNEL_READY;
    return true;
}

bool TaskManager::init_rtos() noexcept
{
    if (g_manager_state == State::RTOS_READY || g_manager_state == State::RUNNING)
    {
        return true;
    }
    if (!init())
    {
        return false;
    }

    // 记录创建时机：PRE_KERNEL 是设计口径，POST_KERNEL 说明启动路径少接了一个入口。
    g_rtos_init_timing =
        (osKernelGetState() == osKernelRunning) ? RtosInitTiming::POST_KERNEL : RtosInitTiming::PRE_KERNEL;

    for (std::size_t index = 0U; index < TASK_COUNT; ++index)
    {
        *TASK_SPECS[index].handle = osThreadNew(TASK_SPECS[index].entry, nullptr, TASK_SPECS[index].attributes);
        if (*TASK_SPECS[index].handle == nullptr)
        {
            // 已创建的线程不在此处回收：进入 Error 后由上层在启动日志中看到失败环节，
            // 且不再推进到 Running，任务表不会以"部分就绪"的形态运行。
            enter_error(FailureStage::THREAD_CREATION);
            return false;
        }
        ++g_created_task_count;
    }

    g_manager_state = State::RTOS_READY;
    return true;
}

bool TaskManager::start() noexcept
{
    if (g_manager_state == State::RUNNING)
    {
        return true;
    }
    if (g_manager_state != State::RTOS_READY || osKernelGetState() != osKernelRunning)
    {
        ++g_start_refused_count;
        return false;
    }

    g_manager_state = State::RUNNING;
    return true;
}

TaskManager::State TaskManager::get_state() noexcept
{
    return g_manager_state;
}

TaskManager::Diagnostics TaskManager::get_diagnostics() noexcept
{
    Diagnostics diagnostics{};
    diagnostics.state = g_manager_state;
    diagnostics.rtos_init_timing = g_rtos_init_timing;
    diagnostics.failure = g_failure_stage;
    diagnostics.expected_task_count = static_cast<std::uint32_t>(TASK_COUNT);
    diagnostics.created_task_count = g_created_task_count;
    diagnostics.start_refused_count = g_start_refused_count;
    return diagnostics;
}

void TaskManager::sample_stack_health() noexcept
{
    // 任务表就绪前没有句柄可查，也不算一次采样。
    if (g_manager_state != State::RUNNING || g_created_task_count == 0U)
    {
        return;
    }

    const platform::Time::Tick now = platform::Time::now_ticks();
    if (g_stack_health_anchored &&
        !platform::Time::has_elapsed_ms(g_last_stack_health_tick, STACK_HEALTH_SAMPLE_PERIOD_MS))
    {
        return; // 限频：调用方可以每拍调用
    }
    g_stack_health_anchored = true;
    g_last_stack_health_tick = now;

    StackHealth health{};
    health.minimum_free_words = UINT32_MAX;
    health.low_margin_task_index = MARGIN_OK_INDEX;
    for (std::size_t index = 0U; index < TASK_COUNT && index < MAXIMUM_SAMPLED_TASKS; ++index)
    {
        const osThreadId_t handle = *TASK_SPECS[index].handle;
        if (handle == nullptr)
        {
            continue;
        }
        const UBaseType_t free_words = uxTaskGetStackHighWaterMark(reinterpret_cast<TaskHandle_t>(handle));
        health.free_words[index] = static_cast<std::uint32_t>(free_words);
        ++health.sampled_task_count;
        if (health.free_words[index] < health.minimum_free_words)
        {
            health.minimum_free_words = health.free_words[index];
            health.low_margin_task_index = static_cast<std::uint32_t>(index);
        }
    }
    if (health.sampled_task_count == 0U)
    {
        return; // 没有可查句柄：不发布误导性的全零快照
    }
    if (health.minimum_free_words >= STACK_HEALTH_WARNING_FREE_WORDS)
    {
        health.low_margin_task_index = MARGIN_OK_INDEX;
    }
    health.sample_count = ++g_stack_health_sample_count;

    // 整字拷贝发布：读侧（调试器）可能随时读取，不追求原子性，只保证布局与快照一致。
    static_assert((sizeof(StackHealth) % sizeof(std::uint32_t)) == 0U, "栈余量快照必须可按 32 位整字拷贝");
    const std::uint32_t *source = reinterpret_cast<const std::uint32_t *>(&health);
    volatile std::uint32_t *target = reinterpret_cast<volatile std::uint32_t *>(&g_stack_health_archive);
    for (std::size_t index = 0U; index < sizeof(health) / sizeof(std::uint32_t); ++index)
    {
        target[index] = source[index];
    }
}

TaskManager::StackHealth TaskManager::get_stack_health() noexcept
{
    StackHealth health{};
    const volatile std::uint32_t *source = reinterpret_cast<const volatile std::uint32_t *>(&g_stack_health_archive);
    std::uint32_t *target = reinterpret_cast<std::uint32_t *>(&health);
    for (std::size_t index = 0U; index < sizeof(health) / sizeof(std::uint32_t); ++index)
    {
        target[index] = source[index];
    }
    return health;
}

} // namespace application::task
