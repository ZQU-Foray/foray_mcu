#ifndef COMMON_TASK_RUNNER_HPP
#define COMMON_TASK_RUNNER_HPP

#include "hal/os/clock.hpp"
#include "hal/os/signal.hpp"
#include "middleware/topic/topic.hpp"

#include "FreeRTOS.h"
#include "task.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

// 任务运行器（架构 v2 §3.5，Decision-20261031-12）：按组合根的任务表创建 FreeRTOS 任务，用原生 API。
//
// - 每个任务按顺序调用一组 step，step 是 void (*)(hal::os::Microseconds now)；同一轮的各 step 拿到同一个 now。
//   组合根用普通函数或不捕获的 lambda 调能力对象：void monitor_step(hal::os::Microseconds now) { g_monitor.step(now); }
// - 触发方式：every_ms(周期) 按 RTOS 节拍周期运行；on_topic(topic) 在 topic 每次发布后运行一轮
//   （多次发布可能合并成一轮，step 用 update() 读最新值）；on_topic(topic, 超时) 另外在超过这么久没有发布时
//   也运行一轮（例如驱动的初始化要按时间推进，Decision-20261101-03）。
// - 栈、任务控制块与等待用的 Signal 都是本对象的成员（xTaskCreateStatic），不用堆。
// - 任务表作模板实参：common::TaskRunner<TASKS> g_runner; 任务数、栈总字数与任务表的检查都在编译期完成。
// - 三个入口对应启动入口：init() 在 boot_init()，start() 在 boot_init_rtos()，sample_stack_health() 在 boot_run()。

namespace common
{

/** 任务的一步：调用一个能力的 step(now)。 */
using Step = void (*)(hal::os::Microseconds now);

/** 每个任务最多的 step 数。 */
inline constexpr std::size_t MAX_STEPS{8U};

/** 栈高水位的采样周期。 */
inline constexpr hal::os::Microseconds STACK_SAMPLE_PERIOD_US{hal::os::MICROSECONDS_PER_SECOND};

/** 任务的触发方式，用 every_ms()、on_topic() 构造。 */
struct Trigger
{
    enum class Kind : std::uint8_t
    {
        PERIODIC,
        TOPIC
    };

    Kind kind;
    std::uint32_t period_ms; // PERIODIC：周期（ms），换成 RTOS 节拍后不能为 0；TOPIC：超时（ms），0 表示不超时
    topic::TopicBase *source; // TOPIC：每次发布后运行一轮
};

/** 按周期运行：xTaskDelayUntil，错过周期计入 TaskStatus::overrun_count。 */
[[nodiscard]] constexpr Trigger every_ms(std::uint32_t period_ms) noexcept
{
    return Trigger{Trigger::Kind::PERIODIC, period_ms, nullptr};
}

/**
 * @brief 按 topic 运行：topic 每次发布后运行一轮。占 topic 的一个等待槽。
 * @param timeout_ms 不为 0 时，超过这么久没有发布也运行一轮（计入 TaskStatus::timeout_count）。
 */
[[nodiscard]] constexpr Trigger on_topic(topic::TopicBase &source, std::uint32_t timeout_ms = 0U) noexcept
{
    return Trigger{Trigger::Kind::TOPIC, timeout_ms, &source};
}

/** 任务表的一行。任务表是组合根里的 constexpr std::array<TaskSpec, N>。 */
struct TaskSpec
{
    const char *name;                  // 任务名（字符串字面量）
    std::uint32_t stack_words;         // 栈大小，单位字（StackType_t）
    UBaseType_t priority;              // FreeRTOS 优先级，数字越大越高
    Trigger trigger;                   // 触发方式
    std::array<Step, MAX_STEPS> steps; // 按顺序调用；第一个 nullptr 之后的忽略
};

/** 任务表全部任务的栈字数之和。 */
template <std::size_t TaskCount>
[[nodiscard]] constexpr std::uint32_t total_stack_words(const std::array<TaskSpec, TaskCount> &tasks) noexcept
{
    std::uint32_t total = 0U;
    for (const TaskSpec &task : tasks)
    {
        total += task.stack_words;
    }
    return total;
}

/**
 * @brief 每个任务的观测计数，调试器可读。每个字段只有一个写者（任务自己，或采样栈的默认任务），
 *        用"读一次、写一次"更新（规则 13）。
 */
struct TaskStatus
{
    std::atomic<std::uint32_t> run_count{0U};          // 已运行的轮数
    std::atomic<hal::os::Microseconds> max_run_us{0U}; // 单轮最长耗时（全部 step）
    std::atomic<std::uint32_t> overrun_count{0U};      // PERIODIC：错过周期的次数
    std::atomic<std::uint32_t> timeout_count{0U};      // TOPIC（带超时）：因超时而运行的轮数
    std::atomic<std::uint32_t> stack_free_words{0U};   // 最近一次采样的剩余栈（字），0 表示还没采样
};

// ---- 任务表的编译期检查（Decision-20261031-12）：写错的任务表编译不过，不留到上电才发现 ----

template <std::size_t TaskCount>
[[nodiscard]] constexpr bool all_have_steps(const std::array<TaskSpec, TaskCount> &tasks) noexcept
{
    for (const TaskSpec &task : tasks)
    {
        if (task.name == nullptr || task.steps[0] == nullptr)
        {
            return false;
        }
    }
    return true;
}

template <std::size_t TaskCount>
[[nodiscard]] constexpr bool all_stacks_fit(const std::array<TaskSpec, TaskCount> &tasks) noexcept
{
    for (const TaskSpec &task : tasks)
    {
        if (task.stack_words < configMINIMAL_STACK_SIZE)
        {
            return false;
        }
    }
    return true;
}

template <std::size_t TaskCount>
[[nodiscard]] constexpr bool all_priorities_valid(const std::array<TaskSpec, TaskCount> &tasks) noexcept
{
    for (const TaskSpec &task : tasks)
    {
        if (task.priority == tskIDLE_PRIORITY || task.priority >= configMAX_PRIORITIES)
        {
            return false;
        }
    }
    return true;
}

template <std::size_t TaskCount>
[[nodiscard]] constexpr bool all_triggers_valid(const std::array<TaskSpec, TaskCount> &tasks) noexcept
{
    for (const TaskSpec &task : tasks)
    {
        const bool valid = task.trigger.kind == Trigger::Kind::PERIODIC ? pdMS_TO_TICKS(task.trigger.period_ms) != 0U
                                                                        : task.trigger.source != nullptr;
        if (!valid)
        {
            return false;
        }
    }
    return true;
}

/**
 * @brief 按任务表 Tasks 创建并运行任务。
 * @tparam Tasks 组合根里的 constexpr std::array<TaskSpec, N>（静态存储）。任务数与栈总字数由它在编译期求出，
 *         任务表本身也在编译期检查。
 * @note 全部成员初值为零，作全局对象时放在 .bss，不占 Flash，也不依赖全局构造顺序。
 */
template <const auto &Tasks>
class TaskRunner final
{
    static constexpr std::size_t TASK_COUNT{Tasks.size()};
    static constexpr std::uint32_t STACK_WORDS{total_stack_words(Tasks)};

    static_assert(TASK_COUNT > 0U, "任务表不能为空");
    static_assert(all_have_steps(Tasks), "每个任务要有名字，且至少有一个 step");
    static_assert(all_stacks_fit(Tasks), "任务栈不能小于 configMINIMAL_STACK_SIZE");
    static_assert(all_priorities_valid(Tasks), "优先级要高于空闲任务、低于 configMAX_PRIORITIES");
    static_assert(all_triggers_valid(Tasks), "周期换成 RTOS 节拍后不能为 0；按 topic 触发要给出 topic");

  public:
    constexpr TaskRunner() noexcept = default;

    TaskRunner(const TaskRunner &) = delete;
    TaskRunner &operator=(const TaskRunner &) = delete;
    TaskRunner(TaskRunner &&) = delete;
    TaskRunner &operator=(TaskRunner &&) = delete;
    ~TaskRunner() = default;

    /**
     * @brief 登记 topic 等待者。在 boot_init() 里调用，只能调用一次。
     * @return 某个 topic 的等待槽已满（topic::WAIT_SLOTS）时返回 false。
     */
    [[nodiscard]] bool init() noexcept
    {
        if (initialized_)
        {
            return false;
        }
        std::uint32_t offset = 0U;
        for (std::size_t index = 0U; index < TASK_COUNT; ++index)
        {
            const TaskSpec &task = Tasks[index];
            if (task.trigger.kind == Trigger::Kind::TOPIC && !task.trigger.source->add_waiter(signals_[index]))
            {
                return false;
            }
            stack_offsets_[index] = offset;
            offset += task.stack_words;
            contexts_[index] = Context{this, index};
        }
        initialized_ = true;
        return true;
    }

    /** @brief 创建全部任务。在 boot_init_rtos() 里调用（内核启动之前），先调用过 init()。 */
    [[nodiscard]] bool start() noexcept
    {
        if (!initialized_ || started_)
        {
            return false;
        }
        for (std::size_t index = 0U; index < TASK_COUNT; ++index)
        {
            const TaskSpec &task = Tasks[index];
            handles_[index] = xTaskCreateStatic(entry,
                                                task.name,
                                                task.stack_words,
                                                &contexts_[index],
                                                task.priority,
                                                &stacks_[stack_offsets_[index]],
                                                &control_blocks_[index]);
            if (handles_[index] == nullptr)
            {
                return false;
            }
        }
        started_ = true;
        return true;
    }

    /**
     * @brief 采样各任务的栈高水位，按 STACK_SAMPLE_PERIOD_US 限频。在 boot_run() 里每拍调用。
     * @note 只能由一个任务调用（默认任务）。
     */
    void sample_stack_health(hal::os::Microseconds now) noexcept
    {
        if (!started_ || (sampled_ && !hal::os::has_elapsed(last_sample_us_, STACK_SAMPLE_PERIOD_US, now)))
        {
            return;
        }
        sampled_ = true;
        last_sample_us_ = now;
        for (std::size_t index = 0U; index < TASK_COUNT; ++index)
        {
            status_[index].stack_free_words.store(uxTaskGetStackHighWaterMark(handles_[index]),
                                                  std::memory_order_relaxed);
        }
    }

    /** 第 index 个任务（与任务表同序）的观测计数。 */
    [[nodiscard]] const TaskStatus &status(std::size_t index) const noexcept
    {
        return status_[index];
    }

  private:
    struct Context
    {
        TaskRunner *runner;
        std::size_t index;
    };

    static void entry(void *argument) noexcept
    {
        const Context &context = *static_cast<const Context *>(argument);
        context.runner->run(context.index);
    }

    [[noreturn]] void run(std::size_t index) noexcept
    {
        const TaskSpec &task = Tasks[index];
        TaskStatus &status = status_[index];
        if (task.trigger.kind == Trigger::Kind::PERIODIC)
        {
            const TickType_t period = pdMS_TO_TICKS(task.trigger.period_ms);
            TickType_t last_wake = xTaskGetTickCount();
            for (;;)
            {
                run_steps(task, status);
                if (xTaskDelayUntil(&last_wake, period) == pdFALSE)
                {
                    status.overrun_count.store(status.overrun_count.load(std::memory_order_relaxed) + 1U,
                                               std::memory_order_relaxed);
                }
            }
        }
        const hal::os::Microseconds timeout = task.trigger.period_ms == 0U
                                                  ? hal::os::WAIT_FOREVER
                                                  : task.trigger.period_ms * hal::os::MICROSECONDS_PER_MILLISECOND;
        for (;;)
        {
            const bool published = signals_[index].wait(timeout);
            if (!published)
            {
                if (timeout == hal::os::WAIT_FOREVER)
                {
                    continue;
                }
                status.timeout_count.store(status.timeout_count.load(std::memory_order_relaxed) + 1U,
                                           std::memory_order_relaxed);
            }
            run_steps(task, status);
        }
    }

    static void run_steps(const TaskSpec &task, TaskStatus &status) noexcept
    {
        const hal::os::Microseconds now = hal::os::now_us();
        for (const Step step : task.steps)
        {
            if (step == nullptr)
            {
                break;
            }
            step(now);
        }
        const hal::os::Microseconds duration = hal::os::elapsed_us(now);
        if (duration > status.max_run_us.load(std::memory_order_relaxed))
        {
            status.max_run_us.store(duration, std::memory_order_relaxed);
        }
        status.run_count.store(status.run_count.load(std::memory_order_relaxed) + 1U, std::memory_order_relaxed);
    }

    std::array<StackType_t, STACK_WORDS> stacks_{};
    std::array<StaticTask_t, TASK_COUNT> control_blocks_{};
    std::array<TaskHandle_t, TASK_COUNT> handles_{};
    std::array<hal::os::Signal, TASK_COUNT> signals_{};
    std::array<Context, TASK_COUNT> contexts_{};
    std::array<std::uint32_t, TASK_COUNT> stack_offsets_{};
    std::array<TaskStatus, TASK_COUNT> status_{};
    hal::os::Microseconds last_sample_us_{0U};
    bool sampled_{false};
    bool initialized_{false};
    bool started_{false};
};

} // namespace common

#endif // COMMON_TASK_RUNNER_HPP
