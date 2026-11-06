// 组合根：infantry_dm_mc02（步兵 × DM-MC02）。
// 构造驱动与能力对象、注入板级句柄与参数、定义任务表、实现启动入口 boot_*；不含行为。
#include "actuator/power_outputs.hpp"
#include "actuator/safety_gate.hpp"
#include "board.hpp"
#include "boot/boot.h"
#include "command/arbiter.hpp"
#include "common/task_runner.hpp"
#include "driver/bmi088/bmi088.hpp"
#include "driver/bmi088/bmi088_heater.hpp"
#include "driver/can_bus/can_bus.hpp"
#include "driver/motor/dm/dm_models.hpp"
#include "driver/motor/dm/dm_motor.hpp"
#include "driver/motor/joint.hpp"
#include "gimbal/gimbal.hpp"
#include "hal/init.hpp"
#include "hal/os/clock.hpp"
#include "imu/imu.hpp"
#include "params.hpp"
#include "pc_link/pc_link.hpp"
#include "remote/remote.hpp"
#include "topics/imu.hpp"

#include <array>
#include <cstddef>

namespace
{

// 可选能力的编译期检查（架构 §3.1"能力分级"）：BMI088 驱动只用 SPI 异步事务
static_assert(!device::Bmi088::REQUIRES_SPI_ASYNC || board::SPI_ASYNC, "BMI088 驱动需要板支持 SPI 异步");

// 驱动：板级句柄在这里注入
device::Bmi088 g_bmi088{board::SPI_IMU_ACCEL, board::SPI_IMU_GYRO};
device::Bmi088Heater g_heater{board::PWM_IMU_HEATER, params::HEATER};
// CAN1：总线分发与两台 DM-J4310（云台 yaw、pitch）
device::CanBus g_can_1{board::CAN_1};
device::Joint g_yaw;
device::Joint g_pitch;
constexpr device::DmMotor::Config gimbal_motors()
{
    device::DmMotor::Config config{};
    config.count = 2U;
    config.motors[0] = device::DmMotor::Motor{
        &g_yaw, device::dm::DM_J4310, params::YAW_ESC_ID, params::YAW_MASTER_ID, params::YAW_MECHANICS};
    config.motors[1] = device::DmMotor::Motor{
        &g_pitch, device::dm::DM_J4310, params::PITCH_ESC_ID, params::PITCH_MASTER_ID, params::PITCH_MECHANICS};
    return config;
}
device::DmMotor g_dm_can_1{g_can_1, gimbal_motors()};
// 总线负载：1 kHz 下两台达妙电机每秒 4000 帧，约 54%（编译期检查）
static_assert(device::can_load_percent(device::DmMotor::frames_per_second(2U, 1000U)) <= 80U, "CAN1 在 1 kHz 下超载");

// 能力
// 板上两路 24 V 输出，高电平打开
constexpr std::array POWER_24V_OUTPUTS{board::POWER_24V_0, board::POWER_24V_1};
actuator::PowerOutputs g_power_outputs;
imu::Imu g_imu;
gimbal::Gimbal g_gimbal;
// 云台录制的存储：160 KB，放 AXI SRAM（DTCM 放不下）；启动时不清零，云台只读自己写过的部分
constexpr std::size_t GIMBAL_CAPTURE_CAPACITY{4096U};
__attribute__((section(".axi_bss"))) std::array<gimbal::Gimbal::CaptureSample, GIMBAL_CAPTURE_CAPACITY>
    g_gimbal_capture;
pc_link::PcLink g_pc_link{board::USB_CDC};
remote::Remote g_remote{board::UART_SBUS};
command::Arbiter g_command;

// 安全门管的关节：云台的两个。上电锁住，遥控确认过才放开
constexpr std::array<device::Joint *, 2> GIMBAL_JOINTS{&g_yaw, &g_pitch};
actuator::SafetyGate g_safety_gate;

// 任务表里的 step：每个只转调一个对象
void imu_step(hal::os::Microseconds now)
{
    g_imu.step(now);
}

void gimbal_receive(hal::os::Microseconds)
{
    g_can_1.receive();
}

// 仲裁排在云台之前：急停当轮生效，云台用的是这一轮的指令
void command_step(hal::os::Microseconds now)
{
    g_command.step(now);
}

void gimbal_step(hal::os::Microseconds now)
{
    g_gimbal.step(now);
}

void gimbal_transmit(hal::os::Microseconds now)
{
    g_can_1.transmit(now);
}

void pc_link_step(hal::os::Microseconds now)
{
    g_pc_link.step(now);
}

void remote_step(hal::os::Microseconds now)
{
    g_remote.step(now);
}

// 安全门排在云台之后、CAN1 发之前：急停当轮就生效
void safety_step(hal::os::Microseconds now)
{
    g_safety_gate.step(now);
}

// 优先级：IMU 最高（数据就绪 2 kHz，按中断发布的 imu.event 触发；1 ms 超时让驱动初始化按时间推进）；
// 云台按 imu.attitude 触发，CAN1 的收、云台、安全门、CAN1 的发在同一个任务里（一条总线只归一个任务），
// 2 ms 超时保证没有姿态时总线照常收发；链路低于云台，同一份姿态先算控制、再发给上位机；
// 遥控最低，每 5 ms 发一次 safety.stop。
// CubeMX 的默认任务是 24，只调 boot_run。
constexpr std::array<common::TaskSpec, 4> TASKS{{
    {"imu", 1024U, 28U, common::on_topic(topics::imu::event, 1U), {imu_step}},
    {"gimbal",
     1024U,
     27U,
     common::on_topic(topics::imu::attitude, 2U),
     {gimbal_receive, command_step, gimbal_step, safety_step, gimbal_transmit}},
    {"pc_link", 1024U, 26U, common::on_topic(topics::imu::attitude, 1U), {pc_link_step}},
    {"remote", 512U, 25U, common::every_ms(5U), {remote_step}},
}};

common::TaskRunner<TASKS> g_runner;

} // namespace

bool boot_init(void)
{
    // 电源输出最先打开：达妙电机上电自检约 1 s。达妙驱动的 init() 接上关节，所以排在云台之前
    return hal::init() &&
           g_power_outputs.init(actuator::PowerOutputs::Config{
               POWER_24V_OUTPUTS.data(), POWER_24V_OUTPUTS.size(), hal::Gpio::Level::HIGH, params::POWER_OUTPUTS}) &&
           g_imu.init(imu::Imu::Config{
               &g_bmi088, &g_heater, board::IMU_ACCEL_INTERRUPT, board::IMU_GYRO_INTERRUPT, params::IMU}) &&
           g_can_1.init() && g_dm_can_1.init() &&
           g_safety_gate.init(
               actuator::SafetyGate::Config{GIMBAL_JOINTS.data(), GIMBAL_JOINTS.size(), params::SAFETY}) &&
           g_gimbal.init(gimbal::Gimbal::Config{
               &g_yaw, &g_pitch, params::GIMBAL, g_gimbal_capture.data(), g_gimbal_capture.size()}) &&
           g_remote.init(params::REMOTE) && g_pc_link.init(params::PC_LINK) && g_runner.init();
}

bool boot_init_rtos(void)
{
    return g_runner.start();
}

void boot_run(void)
{
    g_runner.sample_stack_health(hal::os::now_us());
}
