#include "pc_link/pc_link.hpp"

#include "middleware/robolink/messages.hpp"

#include <algorithm>
#include <array>
#include <cstddef>

namespace pc_link
{

namespace
{

constexpr std::size_t READ_CHUNK{64U}; // USB 全速的端点包大小

topics::pc_link::Control to_topic(const robolink::ControlSnapshot &snapshot, hal::os::Microseconds received_us)
{
    topics::pc_link::Control control{};
    control.control_seq = snapshot.control_seq;
    control.source = static_cast<std::uint8_t>(snapshot.source);
    control.mode = static_cast<std::uint8_t>(snapshot.mode);
    control.flags = snapshot.flags;
    control.chassis_vx = snapshot.chassis_vx;
    control.chassis_vy = snapshot.chassis_vy;
    control.chassis_wz = snapshot.chassis_wz;
    control.gimbal_yaw = snapshot.gimbal_yaw;
    control.gimbal_pitch = snapshot.gimbal_pitch;
    control.gimbal_yaw_rate = snapshot.gimbal_yaw_rate;
    control.gimbal_pitch_rate = snapshot.gimbal_pitch_rate;
    control.friction_rads = snapshot.friction_rads;
    control.feeder_rads = snapshot.feeder_rads;
    control.validity_hint_ms = snapshot.validity_hint_ms;
    control.received_us = received_us;
    return control;
}

robolink::GimbalAttitude to_message(const topics::imu::Attitude &attitude)
{
    robolink::GimbalAttitude message{};
    message.sample_time_us = attitude.timestamp_us;
    message.quaternion_wxyz = attitude.quaternion;
    message.gyro_rad_per_sec = attitude.gyro_rad_per_sec;
    // topics::imu::State 与协议的 imu_state 取值相同
    message.imu_state = static_cast<robolink::ImuState>(static_cast<std::uint8_t>(attitude.state));
    message.attitude_valid = attitude.valid;
    if (!message.attitude_valid)
    {
        message.quaternion_wxyz = {0.0F, 0.0F, 0.0F, 0.0F}; // 无效时不给可能误用的值
        message.gyro_rad_per_sec = {0.0F, 0.0F, 0.0F};
    }
    return message;
}

static_assert(static_cast<std::uint8_t>(topics::imu::State::SETTLING) ==
                      static_cast<std::uint8_t>(robolink::ImuState::SETTLING) &&
                  static_cast<std::uint8_t>(topics::imu::State::READY) ==
                      static_cast<std::uint8_t>(robolink::ImuState::READY) &&
                  static_cast<std::uint8_t>(topics::imu::State::DEGRADED) ==
                      static_cast<std::uint8_t>(robolink::ImuState::DEGRADED),
              "imu.attitude 的状态取值要与 RoboLink 的 imu_state 相同");

} // namespace

bool PcLink::init(const Parameters &parameters) noexcept
{
    if (parameters.max_reads_per_step == 0U)
    {
        return false;
    }
    // 不检查 usb_.is_ready()：DM-MC02 的 USB 在内核启动后才由生成区的默认任务初始化，连上之前 step() 只等待
    if (!session_.init(parameters.protocol, hal::os::now_us()))
    {
        return false;
    }
    control_publisher_ = topics::pc_link::control.claim_publisher();
    if (!control_publisher_)
    {
        return false;
    }
    parameters_ = parameters;
    // 第一版只记账：没有能力接受、执行控制快照（Decision-20261103-01 第 8 点）
    session_.set_status(robolink::Session::StatusContent{});
    report_ = Report{};
    return true;
}

void PcLink::step(hal::os::Microseconds now) noexcept
{
    (void)now; // 帧头时刻要尽量贴近真正发出的时刻：各处现取 hal::os::now_us()
    report_.connected = usb_.is_connected();
    session_.set_connected(report_.connected, hal::os::now_us());
    if (report_.connected)
    {
        receive();
    }
    session_.process(hal::os::now_us());

    robolink::ControlSnapshot snapshot{};
    std::uint32_t received_us = 0U;
    if (session_.take_control(snapshot, received_us))
    {
        control_publisher_.publish(to_topic(snapshot, received_us));
        ++report_.controls;
    }

    topics::imu::Attitude attitude{};
    if (attitude_.update(attitude))
    {
        session_.set_attitude(to_message(attitude));
        ++report_.attitudes;
    }

    if (report_.connected)
    {
        session_.set_receive_overflow(usb_.statistics().dropped_byte_count);
        transmit();
    }
    report_.state = session_.state();
    report_.peer_accepts_attitude = session_.peer_accepts_attitude();
    report_.session = session_.statistics();
}

void PcLink::receive() noexcept
{
    std::array<std::uint8_t, READ_CHUNK> chunk{};
    for (std::uint8_t read = 0U; read < parameters_.max_reads_per_step; ++read)
    {
        const std::size_t space = std::min(session_.receive_space(), chunk.size());
        if (space == 0U)
        {
            break; // 解析缓冲区满：先处理，余下的留在 USB 接收队列里
        }
        const std::size_t length = usb_.read(chunk.data(), space);
        if (length == 0U)
        {
            break;
        }
        report_.received_bytes += static_cast<std::uint32_t>(session_.receive(chunk.data(), length));
    }
}

void PcLink::transmit() noexcept
{
    const hal::os::Microseconds now = hal::os::now_us();
    const std::size_t size = session_.prepare(now);
    if (size == 0U)
    {
        return;
    }
    switch (usb_.write(session_.frame(), size))
    {
    case hal::UsbCdc::WriteResult::STARTED:
        session_.commit(now);
        ++report_.written_frames;
        break;
    case hal::UsbCdc::WriteResult::BUSY:
        session_.report_busy();
        ++report_.write_busy;
        break;
    default:
        session_.report_busy();
        ++report_.write_errors;
        break;
    }
}

} // namespace pc_link
