#include "tasks/imu_task.hpp"
#include "Application/Imu/AttitudeEstimator.hpp"
#include "Application/Imu/GyroBiasCalibration.hpp"
#include "Application/Imu/ImuDiagnostics.hpp"
#include "tasks/imu_data_ready_mailbox.hpp"
#include "driver/bmi088/bmi088.hpp"
#include "driver/bmi088/bmi088_heater.hpp"
#include "hal/gpio.hpp"
#include "cmsis_os2.h"
#include <atomic>
#include <cstdint>
#include <cstring>
#include <limits>

namespace application::task
{

namespace
{

constexpr std::uint32_t TASK_PERIOD_TICKS{1U};
constexpr std::uint32_t ACCELEROMETER_DATA_READY_FLAG{1UL << 0U};
constexpr std::uint32_t GYROSCOPE_DATA_READY_FLAG{1UL << 1U};
constexpr std::uint32_t TRANSFER_COMPLETED_FLAG{1UL << 2U};
constexpr std::uint32_t DATA_READY_FLAGS{ACCELEROMETER_DATA_READY_FLAG | GYROSCOPE_DATA_READY_FLAG |
                                         TRANSFER_COMPLETED_FLAG};

// 上电零偏标定的温度门控必须与恒温服务点一致；两者不一致时标定永远进不了
// 采样状态，z 轴零偏完全不会被扣除，yaw 直接积分原始零偏。
static_assert(application::GyroBiasCalibration::Config{}.target_celsius == device::Bmi088Heater::DEFAULT_TARGET_CELSIUS,
              "零偏标定温度门控必须与恒温目标一致");

// 一次唤醒内推进的采集轮数：每轮只做常数时间的边沿通知与传输衔接，
// 两轮足以在同一个唤醒里把加速度计与陀螺两路挂起边沿都接上。
// 这里刻意**不做**"只要还有标志就继续"的自旋：本任务优先级最高，
// 而 DRDY 边沿每 500/625 us 就会重来一次，自旋会让它永远不回等待点，
// 从而无界占用 CPU 并饿死 1 ms 的底盘/云台任务（现象是电机拿不到
// 力矩指令、完全不转）。
constexpr std::uint32_t ACQUISITION_ROUNDS{2U};

// 慢路径每轮搬进跨任务队列的记录数上限：只做常数时间的 40 字节拷贝，
// 使采集任务每轮占用的长度与样本内容无关。本任务优先级高于 1 ms 的底盘/云台
// 电机任务，这个上限就是它们 CPU 预算的下界。
constexpr std::uint32_t SAMPLE_BUDGET_PER_ITERATION{4U};

device::Bmi088 g_bmi088;
device::Bmi088Heater g_bmi088_heater;
ImuDataReadyMailbox g_accelerometer_interrupts, g_gyroscope_interrupts;
bool g_initialized{false};
osThreadId_t g_task_handle{nullptr};

#if defined(IMU_ENABLE_DIAGNOSTICS)
// 仅供调试器的暂存温度邮箱：先写 IEEE-754 位，再置
// command=1。默认/Release 行为仍是 50 C 服务约定。
std::atomic<std::uint32_t> g_bmi088_heater_diagnostic_target_bits{0U};
std::atomic<std::uint32_t> g_bmi088_heater_diagnostic_command{0U};
struct HeaterDiagnosticReport
{
    std::uint32_t version{1U}, requests{0U}, accepted{0U}, rejected{0U};
    float requested_celsius{0.0F}, active_celsius{device::Bmi088Heater::DEFAULT_TARGET_CELSIUS};
    std::uint32_t last_result{0U}; // 0 无，1 已接受，2 已拒绝
};
volatile HeaterDiagnosticReport g_bmi088_heater_diagnostic_report{};
#endif

void accelerometer_data_ready_callback(void *context)
{
    (void)context;
    g_accelerometer_interrupts.publish(platform::Time::now_ticks());
    if (g_task_handle != nullptr)
    {
        (void)osThreadFlagsSet(g_task_handle, ACCELEROMETER_DATA_READY_FLAG);
    }
}

void gyroscope_data_ready_callback(void *context)
{
    (void)context;
    g_gyroscope_interrupts.publish(platform::Time::now_ticks());
    if (g_task_handle != nullptr)
    {
        (void)osThreadFlagsSet(g_task_handle, GYROSCOPE_DATA_READY_FLAG);
    }
}

void transfer_completed_callback(void *context)
{
    (void)context;
    if (g_task_handle != nullptr)
        (void)osThreadFlagsSet(g_task_handle, TRANSFER_COMPLETED_FLAG);
}

} // namespace

bool ImuTask::init() noexcept
{
    g_bmi088.init();
    if (!g_bmi088_heater.init())
    {
        g_initialized = false;
        return false;
    }

    // 姿态解算不是采集的前置条件：滤波器参数无效时保持未就绪，
    // 由 AttitudeEstimator 的统计暴露，不阻断采集与恒温。
    (void)AttitudeEstimator::init();

    g_bmi088_heater.set_enabled(true);
    g_task_handle = nullptr;
    g_initialized = true;
    return true;
}

[[noreturn]] void ImuTask::run(void *argument) noexcept
{
    (void)argument;

    if (!g_initialized)
    {
        osThreadExit();
    }

    g_task_handle = osThreadGetId();
    g_bmi088.set_completion_notification(transfer_completed_callback, nullptr);
    if (!platform::Gpio::set_interrupt_callback(platform::Gpio::Pin::IMU_ACCELEROMETER_INTERRUPT,
                                                accelerometer_data_ready_callback) ||
        !platform::Gpio::set_interrupt_callback(platform::Gpio::Pin::IMU_GYROSCOPE_INTERRUPT,
                                                gyroscope_data_ready_callback))
    {
        g_bmi088_heater.set_enabled(false);
        osThreadExit();
    }

    for (;;)
    {
        // osThreadFlagsWait 会原子清掉本次唤醒对应的标志，不存在"Get 与 Clear 之间
        // 丢边沿"的竞态；即使漏掉一次唤醒也不会丢事件——DRDY 事件本体由邮箱的累计
        // 序号承载，notify_data_ready 会按序号补齐。
        (void)osThreadFlagsWait(DATA_READY_FLAGS, osFlagsWaitAny, TASK_PERIOD_TICKS);

        // 采集衔接：固定轮数，只做常数时间工作；慢路径另有限量，保证一次唤醒的
        // 占用长度有上界，不会把低优先级的电机任务饿死。
        for (std::uint32_t round = 0U; round < ACQUISITION_ROUNDS; ++round)
        {
            g_bmi088.notify_data_ready(g_accelerometer_interrupts.snapshot(), g_gyroscope_interrupts.snapshot());
            g_bmi088.process();
        }

        const auto &temperature = g_bmi088.get_temperature();
#if defined(IMU_ENABLE_DIAGNOSTICS)
        if (g_bmi088_heater_diagnostic_command.exchange(0U, std::memory_order_acq_rel) == 1U)
        {
            const auto bits = g_bmi088_heater_diagnostic_target_bits.load(std::memory_order_relaxed);
            float target{};
            std::memcpy(&target, &bits, sizeof(target));
            ++g_bmi088_heater_diagnostic_report.requests;
            g_bmi088_heater_diagnostic_report.requested_celsius = target;
            if (g_bmi088_heater.set_target_celsius(target))
            {
                ++g_bmi088_heater_diagnostic_report.accepted;
                g_bmi088_heater_diagnostic_report.active_celsius = target;
                g_bmi088_heater_diagnostic_report.last_result = 1U;
            }
            else
            {
                ++g_bmi088_heater_diagnostic_report.rejected;
                g_bmi088_heater_diagnostic_report.last_result = 2U;
            }
        }
#endif
        g_bmi088_heater.process(temperature);

        // 上电零偏标定的温度门控需要温度读数；温度变化缓慢，每轮取一次即可。
        // 温度在这里抓取并随记录入队，消费者不再回头读驱动状态。
        device::Bmi088Temperature::Sample temperature_sample{};
        float temperature_celsius = std::numeric_limits<float>::quiet_NaN();
        if (temperature.try_get_sample(temperature_sample))
        {
            temperature_celsius = device::Bmi088Temperature::to_celsius(temperature_sample.raw);
        }

        // 慢路径：诊断与姿态解算，每轮最多固定条数，剩余样本留到下一轮，
        // 队列容量足以吸收，保证采集链路不会被长时间的批处理阻塞。
        device::Bmi088SampleRecord record{};
        bool gyroscope{false};
        for (std::uint32_t processed = 0U; processed < SAMPLE_BUDGET_PER_ITERATION; ++processed)
        {
            if (!g_bmi088.pop_next_sample(record, gyroscope))
            {
                break;
            }
            application::ImuDiagnostics::observe(g_bmi088, record, gyroscope);
            application::AttitudeEstimator::process(record, gyroscope, temperature_celsius);
        }
    }
}

} // namespace application::task
