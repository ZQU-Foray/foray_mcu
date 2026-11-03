// 组合根：link_dm_mc02（上位机链路联调，Decision-20261103-01）。
// 构造驱动与能力对象、注入板级句柄与参数、定义任务表、实现启动入口 boot_*；不含行为。
#include "board.hpp"
#include "boot/boot.h"
#include "common/task_runner.hpp"
#include "driver/bmi088/bmi088.hpp"
#include "driver/bmi088/bmi088_heater.hpp"
#include "hal/init.hpp"
#include "hal/os/clock.hpp"
#include "imu/imu.hpp"
#include "params.hpp"
#include "pc_link/pc_link.hpp"
#include "topics/imu.hpp"

#include <array>

namespace
{

// 可选能力的编译期检查（架构 §3.1"能力分级"）：BMI088 驱动只用 SPI 异步事务
static_assert(!device::Bmi088::REQUIRES_SPI_ASYNC || board::SPI_ASYNC, "BMI088 驱动需要板支持 SPI 异步");

// 驱动：板级句柄在这里注入
device::Bmi088 g_bmi088{board::SPI_IMU_ACCEL, board::SPI_IMU_GYRO};
device::Bmi088Heater g_heater{board::PWM_IMU_HEATER, params::HEATER};

// 能力
imu::Imu g_imu;
pc_link::PcLink g_pc_link{board::USB_CDC};

// 任务表里的 step：每个只调一个能力的 step(now)
void imu_step(hal::os::Microseconds now)
{
    g_imu.step(now);
}

void pc_link_step(hal::os::Microseconds now)
{
    g_pc_link.step(now);
}

// 优先级：IMU 最高（数据就绪 2 kHz，按中断发布的 imu.event 触发；1 ms 超时让驱动初始化按时间推进）；
// 链路按 imu.attitude 触发（最多 1 kHz），姿态一出来就发；1 ms 超时保证没有姿态时握手、状态、保活照常。
// CubeMX 的默认任务是 24，只调 boot_run。
constexpr std::array<common::TaskSpec, 2> TASKS{{
    {"imu", 1024U, 28U, common::on_topic(topics::imu::event, 1U), {imu_step}},
    {"pc_link", 1024U, 27U, common::on_topic(topics::imu::attitude, 1U), {pc_link_step}},
}};

common::TaskRunner<TASKS> g_runner;

} // namespace

bool boot_init(void)
{
    return hal::init() &&
           g_imu.init(imu::Imu::Config{
               &g_bmi088, &g_heater, board::IMU_ACCEL_INTERRUPT, board::IMU_GYRO_INTERRUPT, params::IMU}) &&
           g_pc_link.init(params::PC_LINK) && g_runner.init();
}

bool boot_init_rtos(void)
{
    return g_runner.start();
}

void boot_run(void)
{
    g_runner.sample_stack_health(hal::os::now_us());
}
