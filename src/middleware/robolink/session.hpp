#ifndef MIDDLEWARE_ROBOLINK_SESSION_HPP
#define MIDDLEWARE_ROBOLINK_SESSION_HPP

#include "middleware/robolink/messages.hpp"
#include "middleware/robolink/wire.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

// RoboLink v0.2 的 MCU 端连接会话（协议 §5，Decision-20261103-01 第 4–8 点）。
// 只处理内存里的字节与调用方给的时刻：不碰外设、不分配内存、不阻塞。一个对象只归一个任务。

namespace robolink
{

/**
 * @brief MCU 端的一条 RoboLink 连接：握手、判活、序号、发送调度与计数。
 *
 * 每轮的用法（时刻都是 MCU 的单调 µs，按 2^32 回绕；比较用有符号差，两次调用间隔不超过约 35 分钟）：
 *   1. set_connected(传输层连上了吗, now)；
 *   2. 把收到的字节交给 receive()（一次最多 receive_space() 字节），再 process(now)；
 *   3. take_control() 取最新的控制快照；set_status()、set_attitude() 更新要发的内容；
 *   4. prepare(now) 组下一帧，交给传输层发：发出去了调用 commit(now)，传输层忙调用 report_busy()。
 *      没 commit 的帧不算发出：握手应答下一轮重发，状态与姿态下一轮按最新值重组。
 *
 * 状态（§5.1，即 SYSTEM_STATUS 的 link_state）：
 *   - DISCONNECTED：传输层没连上；
 *   - USB_CONNECTED：连上了，等 PC 的 HELLO，什么都不发；
 *   - READY：回了 HELLO_ACK（OK），序号已归零；先发一次带诊断尾部的完整状态；
 *   - ACTIVE：完整状态已发出，或收到了第一份有效的控制快照。
 * READY、ACTIVE 时 link_timeout_us 内收不到任何合法帧，回到 USB_CONNECTED。再收到 HELLO 时重新握手。
 */
class Session final
{
  public:
    struct Config
    {
        std::array<std::uint8_t, 3> fw_version{};      // 本端 HELLO：固件版本 主.次.修订
        std::uint32_t git_short{0U};                   // 本端 HELLO：提交号低 32 位，0 表示未知
        std::uint32_t device_id_digest{0U};            // 本端 HELLO：设备标识摘要，0 表示未提供
        std::uint16_t heartbeat_period_ms{20U};        // 本端心跳周期：这么久什么都没发时发保活帧
        std::uint32_t status_period_us{5000U};         // SYSTEM_STATUS 的周期（200 Hz）
        std::uint32_t diagnostics_period_us{1000000U}; // 附诊断尾部的周期
        std::uint32_t link_timeout_us{500000U};        // 判活门限（§5.1：500 ms）
        std::uint8_t max_frames_per_process{8U};       // 每次 process() 最多处理的帧（§1.4）
    };

    /** SYSTEM_STATUS 里由产品决定的字段；uptime、last_rx_control_seq、link_state 与诊断尾部由会话填写。 */
    struct StatusContent
    {
        Mode mode{Mode::IDLE};
        ControlSource control_source{ControlSource::NONE};
        SafetyState safety_state{SafetyState::NO_SNAPSHOT};
        std::uint32_t last_accepted_control_seq{0U};
        std::uint32_t last_applied_control_seq{0U};
        std::uint16_t control_msg_age_ms{0xFFFFU};
        std::uint32_t fault_flags{0U};
    };

    /** 从 init() 起累计，按 2^32 回绕。调试器可读。 */
    struct Statistics
    {
        std::uint32_t rx_frames{0U};  // 收到的合法帧
        std::uint32_t tx_frames{0U};  // 发出（commit）的帧
        std::uint32_t crc_errors{0U}; // 以下四项来自字节流解析
        std::uint32_t header_errors{0U};
        std::uint32_t frame_errors{0U};
        std::uint32_t discarded_bytes{0U};
        std::uint32_t handshakes{0U};      // 握手成功（进入 READY）的次数
        std::uint32_t rejected_hellos{0U}; // 拒绝的 HELLO：版本不兼容、能力无交集、设备类型不是 PC
        std::uint32_t link_timeouts{0U};   // 判活超时
        std::uint32_t control_snapshots{0U};   // 收到的有效控制快照
        std::uint32_t invalid_payloads{0U};    // 帧合法、消息内容按协议拒收
        std::uint32_t unexpected_messages{0U}; // 方向不对，或没握手就来的控制快照
        std::uint32_t peer_errors{0U};         // PC 发来的 ERROR
        std::uint32_t tx_busy_drop{0U};        // 组好了帧、传输层忙没发出去
        std::uint32_t statuses_sent{0U};
        std::uint32_t attitudes_sent{0U};
        std::uint32_t keepalives_sent{0U};
    };

    /** 本端支持的消息（HELLO 的 capabilities）：第一版的 8 条。 */
    static constexpr std::uint64_t CAPABILITIES{
        capability_bit(MessageType::HELLO) | capability_bit(MessageType::HELLO_ACK) |
        capability_bit(MessageType::HEARTBEAT) | capability_bit(MessageType::ACK) | capability_bit(MessageType::ERROR) |
        capability_bit(MessageType::SYSTEM_STATUS) | capability_bit(MessageType::CONTROL_SNAPSHOT) |
        capability_bit(MessageType::GIMBAL_ATTITUDE)};

    /**
     * @brief 设置参数并复位到 DISCONNECTED。
     * @param now_us 用来起算开机时长：时基从上电开始计时，init 时的读数就是已开机的时长。
     * @return 周期为 0、或每次处理帧数为 0 时返回 false。
     */
    [[nodiscard]] bool init(const Config &config, std::uint32_t now_us) noexcept;

    /** 传输层是否连上（USB 已配置）。断开时回到 DISCONNECTED，丢弃收到一半的字节。 */
    void set_connected(bool connected, std::uint32_t now_us) noexcept;

    /** 交给会话收到的字节，返回收下了多少（解析缓冲区满时少于 length，余下的下一轮再给）。 */
    std::size_t receive(const std::uint8_t *data, std::size_t length) noexcept;

    [[nodiscard]] std::size_t receive_space() const noexcept;

    /** 解析收到的帧（最多 max_frames_per_process 帧），处理握手、控制快照与判活。 */
    void process(std::uint32_t now_us) noexcept;

    /** 取上次 take 之后收到的最新控制快照；received_us 是处理它的时刻。没有新的返回 false。 */
    [[nodiscard]] bool take_control(ControlSnapshot &snapshot, std::uint32_t &received_us) noexcept;

    void set_status(const StatusContent &content) noexcept;

    /** 新的姿态：PC 声明支持 GIMBAL_ATTITUDE 时，下一次 prepare() 发出去。 */
    void set_attitude(const GimbalAttitude &attitude) noexcept;

    /** 传输层接收溢出的累计字节数，填进诊断尾部的 rx_overflow。 */
    void set_receive_overflow(std::uint32_t dropped_bytes) noexcept;

    /** 组下一帧（不超过 96 B），返回长度；这一轮不用发返回 0。 */
    [[nodiscard]] std::size_t prepare(std::uint32_t now_us) noexcept;

    [[nodiscard]] const std::uint8_t *frame() const noexcept
    {
        return transmit_.data();
    }

    /** prepare() 组好的帧已经交给传输层。 */
    void commit(std::uint32_t now_us) noexcept;

    /** prepare() 组好的帧没能发出（传输层忙）。 */
    void report_busy() noexcept;

    [[nodiscard]] LinkState state() const noexcept
    {
        return state_;
    }

    /** PC 的 HELLO 声明了 GIMBAL_ATTITUDE（0x30）。 */
    [[nodiscard]] bool peer_accepts_attitude() const noexcept
    {
        return (peer_.capabilities & capability_bit(MessageType::GIMBAL_ATTITUDE)) != 0U;
    }

    /** 最近一次握手成功时 PC 的 HELLO。 */
    [[nodiscard]] const Hello &peer() const noexcept
    {
        return peer_;
    }

    [[nodiscard]] Statistics statistics() const noexcept;

  private:
    // prepare() 放进帧里的内容，commit() 时据此更新状态
    struct Prepared
    {
        bool handshake{false};
        bool status{false};
        bool diagnostics{false};
        bool attitude{false};
        bool keepalive{false};
    };

    // 本端发送的消息各自的序号
    enum SequenceIndex : std::uint8_t
    {
        SEQUENCE_HELLO = 0U,
        SEQUENCE_HELLO_ACK = 1U,
        SEQUENCE_STATUS = 2U,
        SEQUENCE_ATTITUDE = 3U,
        SEQUENCE_COUNT = 4U
    };

    void advance_clock(std::uint32_t now_us) noexcept;
    void handle_slot(const Slot &slot, std::uint32_t now_us) noexcept;
    void handle_hello(const Hello &hello, std::uint16_t sequence, std::uint32_t now_us) noexcept;
    void enter_ready(const Hello &hello, std::uint32_t now_us) noexcept;
    void drop_session(LinkState next) noexcept;
    [[nodiscard]] SystemStatus build_status(bool diagnostics) const noexcept;
    [[nodiscard]] bool due(std::uint32_t since_us, std::uint32_t period_us, std::uint32_t now_us) const noexcept;

    Config config_{};
    LinkState state_{LinkState::DISCONNECTED};
    FrameParser parser_{};
    std::array<std::uint8_t, MCU_MAX_FRAME_SIZE> transmit_{};
    FrameBuilder builder_{transmit_.data(), transmit_.size()};
    Prepared prepared_{};

    std::uint16_t frame_sequence_{0U};
    std::array<std::uint16_t, SEQUENCE_COUNT> message_sequence_{};

    // 握手应答：等着发
    bool handshake_pending_{false};
    HelloAck handshake_ack_{};
    // 握手后的第一份完整状态：等着发
    bool full_status_pending_{false};

    Hello peer_{};
    StatusContent status_{};
    GimbalAttitude attitude_{};
    bool attitude_pending_{false};
    ControlSnapshot control_{};
    std::uint32_t control_received_us_{0U};
    bool control_pending_{false};
    std::uint32_t last_rx_control_seq_{0U};

    std::uint32_t last_now_us_{0U};
    std::uint64_t uptime_us_{0U};
    std::uint32_t last_rx_us_{0U};
    std::uint32_t last_tx_us_{0U};
    std::uint32_t last_status_us_{0U};
    std::uint32_t last_diagnostics_us_{0U};
    std::uint32_t receive_overflow_{0U};
    Statistics statistics_{};
};

} // namespace robolink

#endif // MIDDLEWARE_ROBOLINK_SESSION_HPP
