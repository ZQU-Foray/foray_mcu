#ifndef BRINGUP_IMU_CHECK_HPP
#define BRINGUP_IMU_CHECK_HPP

#include "driver/bmi088/bmi088.hpp"
#include "driver/bmi088/bmi088_heater.hpp"
#include "hal/gpio.hpp"
#include "hal/os/clock.hpp"
#include "lib/estimate/imu_body_mapping.hpp"
#include "lib/estimate/vqf.hpp"
#include "middleware/topic/topic.hpp"
#include "topics/bringup.hpp"

#include <cstdint>

namespace bringup
{

/**
 * @brief IMU 检查：板载 BMI088 → 姿态，验证中断里发布 topic、SPI 异步、GPIO 外部中断、
 *        PWM（加热），也是阶段 A 的 IMU 能力的最小示范。
 *
 * - 中断（框架中断）：两个数据就绪中断与 SPI 事务完成中断各自更新累计次数与时刻，发布 bringup.imu_event；
 * - 任务（按 bringup.imu_event 触发，带 1 ms 左右的超时让驱动的初始化按时间推进）：step() 把事件交给驱动，
 *   收割样本，换算到机体坐标系，喂给 VQF，限频发布 bringup.imu；加热器按温度样本恒温。
 *
 * 能力零 RTOS；驱动对象与引脚句柄由组合根注入。
 */
class ImuCheck final
{
  public:
    /** 来自 profile 的参数（params.hpp）。 */
    struct Parameters
    {
        device::Bmi088::Config driver;           // 陀螺输出速率与带宽
        estimate::AxisMapping mounting;          // 传感器 → 机体的安装方向
        hal::os::Microseconds publish_period_us; // bringup.imu 的最短发布间隔
        hal::os::Microseconds latency_limit_us;  // 中断发布到任务读到的延迟上限，超过计入 late
        float heater_target_celsius;             // 加热目标温度；加热器为空时不用
    };

    /** 组合根注入的依赖与参数。 */
    struct Config
    {
        device::Bmi088 *imu;
        device::Bmi088Heater *heater; // 可以为空：不加热
        hal::Gpio accel_interrupt;    // 加速度计数据就绪（外部中断）
        hal::Gpio gyro_interrupt;     // 陀螺仪数据就绪（外部中断）
        Parameters parameters;
    };

    /** 统计结果，调试器可读（只有运行 step() 的任务写）。 */
    struct Report
    {
        std::uint32_t events; // 读到的 bringup.imu_event 次数
        std::uint32_t lost_events; // 被覆盖、没读到的事件（Subscriber::lost_count，多个中断合并成一轮属正常）
        std::uint32_t gyro_samples;  // 处理的陀螺样本
        std::uint32_t accel_samples; // 处理的加速度计样本
        std::uint32_t late;          // 延迟超过 latency_limit_us 的次数
        hal::os::Microseconds last_latency_us;
        hal::os::Microseconds max_latency_us;
        std::uint32_t published; // 发布 bringup.imu 的次数
        float temperature_celsius;
        bool ready; // BMI088 两个传感器都已配置好
        bool error; // BMI088 进入错误状态
    };

    [[nodiscard]] bool init(const Config &config) noexcept;

    void step(hal::os::Microseconds now) noexcept;

    [[nodiscard]] const Report &report() const noexcept
    {
        return report_;
    }

  private:
    // ---- 框架中断里调用 ----
    static void on_accel_ready(void *context) noexcept;
    static void on_gyro_ready(void *context) noexcept;
    static void on_transfer_complete(void *context) noexcept;
    void publish_event(bool accel, bool gyro, bool transfer) noexcept;

    void handle_sample(const device::Bmi088SampleRecord &record, bool gyroscope) noexcept;

    Config config_{nullptr, nullptr, hal::Gpio{0U}, hal::Gpio{0U}, Parameters{}};
    topic::Publisher<topics::bringup::ImuEvent> event_publisher_{};
    topic::Subscriber<topics::bringup::ImuEvent> event_{topics::bringup::imu_event};
    topic::Publisher<topics::bringup::Imu> imu_publisher_{};
    estimate::Vqf vqf_{};

    // 只在中断里读写：三个回调都在框架中断里，更新与取快照在临界区内
    topics::bringup::ImuEvent interrupt_state_{};

    topics::bringup::Imu latest_{};
    bool has_new_gyro_{false};
    hal::os::Microseconds last_publish_us_{0U};
    std::uint32_t last_heater_reads_{0U};
    Report report_{};
};

} // namespace bringup

#endif // BRINGUP_IMU_CHECK_HPP
