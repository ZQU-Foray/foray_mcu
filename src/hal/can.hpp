#ifndef HAL_CAN_HPP
#define HAL_CAN_HPP

#include "hal/callback.hpp"
#include "hal/os/clock.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

// 硬件契约：Classic CAN 总线。句柄由各板的 board.hpp 给出，实现由 port 提供，
// 宿主测试由 platform/mock 提供。

namespace hal
{

/**
 * @brief 一条 CAN 总线的句柄（Classic CAN，最多 8 字节数据）。
 *
 * - 发送：send() 把一帧放进硬件发送队列就返回，不等总线应答；
 * - 接收：port 在中断里把收到的帧放进软件接收队列，接收方用 receive() 取。软件队列只有一个消费者，
 *   所以每条总线只有一个接收方，它在初始化阶段用 claim_receiver() 占住总线；
 * - port 接收全部标准帧与扩展帧（不做硬件过滤），远程帧与 CAN FD 帧丢弃并计入 rx_dropped_count；
 * - 收到的帧带接收时刻 received_us：port 在中断里把帧放进软件队列时写入；
 * - 进入 Bus-Off 后 port 自动发起恢复（按 ISO 11898 等待 128 × 11 个隐性位后重新加入总线），每次计入 bus_off_count。
 *
 * 全部函数都是必选能力。句柄只含一个编号，可以拷贝，按值存进驱动的配置。
 */
class Can final
{
  public:
    static constexpr std::size_t MAX_DATA_LENGTH{8U};

    enum class IdentifierType : std::uint8_t
    {
        STANDARD = 0U, // 11 位，0x000–0x7FF
        EXTENDED = 1U  // 29 位，0x00000000–0x1FFFFFFF
    };

    enum class SendResult : std::uint8_t
    {
        QUEUED = 0U,           // 已放进硬件发送队列
        QUEUE_FULL = 1U,       // 硬件发送队列已满，这一帧没有发
        NOT_READY = 2U,        // 总线没有启动（hal::init() 失败或还没调用）
        BUS_OFF = 3U,          // 正处于 Bus-Off，等待自动恢复
        INVALID_ARGUMENT = 4U, // 编号无效、标识符超出范围或长度超过 MAX_DATA_LENGTH
        ERROR = 5U
    };
    static constexpr std::size_t SEND_RESULT_COUNT{6U};

    enum class ReceiveResult : std::uint8_t
    {
        RECEIVED = 0U,
        EMPTY = 1U, // 软件接收队列为空
        NOT_READY = 2U,
        BUS_OFF = 3U, // 队列为空且正处于 Bus-Off
        INVALID_ARGUMENT = 4U,
        ERROR = 5U
    };

    struct Frame final
    {
        std::uint32_t identifier{0U};
        IdentifierType identifier_type{IdentifierType::STANDARD};
        std::uint8_t length{0U};
        std::array<std::uint8_t, MAX_DATA_LENGTH> data{};
        // 接收：帧放进软件接收队列的时刻（在中断里取 hal::os::now_us()）；发送时忽略
        hal::os::Microseconds received_us{0U};
    };

    /** 计数从 hal::init() 起累计，按 2^32 回绕。 */
    struct Statistics final
    {
        std::uint32_t rx_dropped_count{0U};             // 软件接收队列满或帧格式不支持而丢弃的帧
        std::uint32_t rx_hardware_loss_event_count{0U}; // 硬件接收 FIFO 溢出的次数（丢了多少帧未知）
        std::uint32_t bus_off_count{0U};                // 进入 Bus-Off 的次数
    };

    constexpr explicit Can(std::uint8_t index) noexcept : index_{index} {}

    /**
     * @brief 总线已启动且不在 Bus-Off。
     * @note 任务与框架中断里都能调用，不阻塞。
     */
    [[nodiscard]] bool is_ready() const noexcept;

    /**
     * @brief 占住这条总线的接收：每条总线只有第一次调用成功。
     * @return 编号无效或已被占用时返回 false。
     * @note 在初始化阶段由接收方调用一次（任务里），成功后才能 receive()。只发不收的一方不用调用。
     */
    [[nodiscard]] bool claim_receiver() const noexcept;

    /**
     * @brief 把一帧放进硬件发送队列，不阻塞。
     * @note 在任务里调用；多个任务可以向同一条总线发送（port 用临界区保护硬件队列）。
     */
    [[nodiscard]] SendResult send(const Frame &frame) const noexcept;

    /**
     * @brief 从软件接收队列取一帧，不阻塞。
     * @note 只有 claim_receiver() 成功的一方在任务里调用。
     */
    [[nodiscard]] ReceiveResult receive(Frame &frame) const noexcept;

    /**
     * @brief 登记接收通知：有帧放进软件接收队列后，在框架中断里调用 callback(context)。
     *        帧仍要用 receive() 取；通知只用来叫醒接收任务（如 Signal::notify()）。
     * @param callback 传 nullptr 取消登记。
     * @return 编号无效时返回 false。
     * @note 在任务里调用，一般在初始化阶段。
     */
    [[nodiscard]] bool set_receive_notification(Callback callback, void *context) const noexcept;

    /**
     * @brief 读统计计数的一致快照。编号无效时返回全 0。
     * @note 在任务里调用，不阻塞。
     */
    [[nodiscard]] Statistics statistics() const noexcept;

  private:
    std::uint8_t index_; // 由 port 或 mock 解释
};

} // namespace hal

#endif // HAL_CAN_HPP
