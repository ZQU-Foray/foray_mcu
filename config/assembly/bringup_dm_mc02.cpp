// 组合根：bringup_dm_mc02（框架验证，Decision-20261031-12、Decision-20261101-03）。
// 构造驱动与能力对象、注入板级句柄与参数、定义任务表、实现启动入口 boot_*；不含行为。
#include "board.hpp"
#include "boot/boot.h"
#include "bringup/can_check.hpp"
#include "bringup/dm_check.hpp"
#include "bringup/heartbeat.hpp"
#include "bringup/imu_check.hpp"
#include "bringup/indicator.hpp"
#include "bringup/monitor.hpp"
#include "bringup/power_outputs.hpp"
#include "bringup/sbus_check.hpp"
#include "bringup/usb_report.hpp"
#include "common/task_runner.hpp"
#include "driver/bmi088/bmi088.hpp"
#include "driver/bmi088/bmi088_heater.hpp"
#include "driver/buzzer/buzzer.hpp"
#include "driver/motor/dm/dm_motor.hpp"
#include "driver/ws2812/ws2812.hpp"
#include "hal/init.hpp"
#include "hal/os/clock.hpp"
#include "params.hpp"
#include "topics/bringup.hpp"

#include <array>

namespace
{

// 可选能力的编译期检查（架构 §3.1"能力分级"）：BMI088 驱动只用 SPI 异步事务
static_assert(!device::Bmi088::REQUIRES_SPI_ASYNC || board::SPI_ASYNC, "BMI088 驱动需要板支持 SPI 异步");

// ---- 驱动：板级句柄在这里注入 ----
device::Bmi088 g_imu{board::SPI_IMU_ACCEL, board::SPI_IMU_GYRO};
device::Bmi088Heater g_heater{board::PWM_IMU_HEATER, params::HEATER};
device::Buzzer g_buzzer{board::PWM_BUZZER};
device::Ws2812 g_led{board::SPI_LED};
// 达妙电机：CAN1 一台 DM-J4310（Decision-20261102-02）
device::DmMotor g_dm_motor{
    device::DmMotor::Config{board::CAN_1, device::DmMotor::Profile::j4310(), {{params::DM_CHECK.motor}}, 1U}};

// ---- 能力 ----
// 板上两路 24 V 输出，高电平打开（Decision-20261101-06）
constexpr std::array POWER_24V_OUTPUTS{board::POWER_24V_0, board::POWER_24V_1};
bringup::PowerOutputs g_power_outputs;
bringup::Heartbeat g_heartbeat;
bringup::Monitor g_monitor;
bringup::ImuCheck g_imu_check;
// CAN1 的接收由达妙驱动占住，收发都由达妙检查验证；CAN2、CAN3 只收
bringup::DmCheck g_dm_check;
bringup::CanCheck g_can_2{board::CAN_2};
bringup::CanCheck g_can_3{board::CAN_3};
bringup::SbusCheck g_sbus{board::UART_SBUS};
bringup::UsbReport g_usb_report{board::USB_CDC};
bringup::Indicator g_indicator;

// 任务表里的 step：每个只调一个能力的 step(now)
void heartbeat_step(hal::os::Microseconds now)
{
    g_heartbeat.step(now);
}

void monitor_step(hal::os::Microseconds now)
{
    g_monitor.step(now);
}

void imu_step(hal::os::Microseconds now)
{
    g_imu_check.step(now);
}

void can_step(hal::os::Microseconds now)
{
    g_can_2.step(now);
    g_can_3.step(now);
}

void dm_step(hal::os::Microseconds now)
{
    g_dm_check.step(now);
}

void sbus_step(hal::os::Microseconds now)
{
    g_sbus.step(now);
}

void usb_report_step(hal::os::Microseconds now)
{
    g_usb_report.step(now);
}

void indicator_step(hal::os::Microseconds now)
{
    g_indicator.step(now);
}

// 优先级：IMU 最高（数据就绪 2 kHz，按中断发布的 topic 触发；1 ms 超时让驱动的初始化按时间推进）；
// 读者（monitor）高于写者（heartbeat），Decision-20261029-02 修好的情形；其余外设检查 5 ms 一轮。
// CubeMX 的默认任务是 24，只调 boot_run。达妙检查 1 ms 一轮（驱动每轮收发一次），排在最后，前四个任务的序号不变。
constexpr std::array<common::TaskSpec, 5> TASKS{{
    {"imu", 1024U, 28U, common::on_topic(topics::bringup::imu_event, 1U), {imu_step}},
    {"monitor", 256U, 26U, common::on_topic(topics::bringup::heartbeat), {monitor_step}},
    {"heartbeat", 256U, 25U, common::every_ms(1U), {heartbeat_step}},
    {"io", 768U, 20U, common::every_ms(5U), {can_step, sbus_step, usb_report_step, indicator_step}},
    {"dm", 512U, 23U, common::every_ms(1U), {dm_step}},
}};

common::TaskRunner<TASKS> g_runner;

} // namespace

bool boot_init(void)
{
    // 电源输出最先打开：达妙电机上电自检约 1 s
    return hal::init() &&
           g_power_outputs.init(bringup::PowerOutputs::Config{
               POWER_24V_OUTPUTS.data(), POWER_24V_OUTPUTS.size(), hal::Gpio::Level::HIGH, params::POWER_OUTPUTS}) &&
           g_heartbeat.init() && g_monitor.init(params::MONITOR) &&
           g_imu_check.init(bringup::ImuCheck::Config{
               &g_imu, &g_heater, board::IMU_ACCEL_INTERRUPT, board::IMU_GYRO_INTERRUPT, params::IMU}) &&
           g_dm_check.init(bringup::DmCheck::Config{&g_dm_motor, params::DM_CHECK}) && g_can_2.init() &&
           g_can_3.init() && g_sbus.init() && g_usb_report.init(params::USB_REPORT) &&
           g_indicator.init(bringup::Indicator::Config{&g_buzzer, &g_led, params::INDICATOR}) && g_runner.init();
}

bool boot_init_rtos(void)
{
    return g_runner.start();
}

void boot_run(void)
{
    g_runner.sample_stack_health(hal::os::now_us());
}
