#ifndef HAL_UART_HPP
#define HAL_UART_HPP

#include "hal/callback.hpp"

#include <cstddef>
#include <cstdint>

// 硬件契约：UART 字节流（Decision-20261101-01）。句柄由各板的 board.hpp 给出，实现由 port 提供，
// 宿主测试由 platform/mock 提供。

namespace hal
{

/**
 * @brief 一个 UART 端口的句柄：字节流的收与发，波特率与帧格式由板级配置决定（见 board.hpp）。
 *
 * - 接收：hal::init() 起一直在收，port 在中断里把收到的字节放进软件接收队列，read() 取；
 *   队列满时丢弃并计入 dropped_byte_count。软件接收队列只有一个消费者。
 * - 发送：write() 把数据拷走后开始 DMA 发送就返回，同时只有一次发送在进行；结束（成功或失败）时调用发送通知。
 * - 能力按端口区分（board.hpp 写明）：只接了 RX 的端口 write() 返回 UNSUPPORTED。
 *
 * 读写接口与 hal::UsbCdc 同形：处理字节流的代码可以对两者写成同一个模板。
 * 句柄只含一个编号，可以拷贝，按值存进驱动的配置。
 */
class Uart final
{
  public:
    /** 单次 write() 最多这么多字节。 */
    static constexpr std::size_t MAX_WRITE_LENGTH{256U};

    enum class WriteResult : std::uint8_t
    {
        STARTED = 0U,          // 已拷走并开始发送
        BUSY = 1U,             // 上一次发送还没结束
        NOT_READY = 2U,        // 端口没有启动
        UNSUPPORTED = 3U,      // 这个端口不能发送
        INVALID_ARGUMENT = 4U, // 编号无效、缓冲区为空、长度为 0 或超过 MAX_WRITE_LENGTH
        ERROR = 5U
    };

    /** 计数从 hal::init() 起累计，按 2^32 回绕。 */
    struct Statistics final
    {
        std::uint32_t received_byte_count{0U}; // 收到的字节（含丢弃的）
        std::uint32_t dropped_byte_count{0U};  // 软件接收队列满而丢弃的字节
        std::uint32_t error_event_count{0U};   // 帧错误、噪声、溢出等硬件错误事件
        std::uint32_t restart_failure_count{0U}; // 出错后重新开始接收失败的次数（之后 is_ready() 为 false）
        std::uint32_t transmitted_byte_count{0U}; // 发送成功的字节
        std::uint32_t transmit_failure_count{0U}; // 发送失败的次数
    };

    constexpr explicit Uart(std::uint8_t index) noexcept : index_{index} {}

    /**
     * @brief 端口已启动且正在接收。
     * @note 任务与框架中断里都能调用，不阻塞。
     */
    [[nodiscard]] bool is_ready() const noexcept;

    /**
     * @brief 从软件接收队列取最多 capacity 字节，不阻塞。
     * @return 取到的字节数；没有数据、编号无效或参数无效时为 0。
     * @note 只有一个消费者在任务里调用。
     */
    [[nodiscard]] std::size_t read(std::uint8_t *data, std::size_t capacity) const noexcept;

    /**
     * @brief 开始发送 length 字节，不阻塞。data 在返回前已经拷走。
     * @note 在任务里调用；同一端口只应有一个发送方。
     */
    [[nodiscard]] WriteResult write(const std::uint8_t *data, std::size_t length) const noexcept;

    /**
     * @brief 登记接收通知：有字节放进软件接收队列后，在框架中断里调用 callback(context)。
     * @param callback 传 nullptr 取消登记。
     * @return 编号无效时返回 false。
     * @note 在任务里调用，一般在初始化阶段。
     */
    [[nodiscard]] bool set_receive_notification(Callback callback, void *context) const noexcept;

    /**
     * @brief 登记发送通知：一次 write() 的发送结束（成功或失败）后，在框架中断里调用 callback(context)。
     * @note 同 set_receive_notification()。
     */
    [[nodiscard]] bool set_write_notification(Callback callback, void *context) const noexcept;

    /**
     * @brief 读统计计数的一致快照。编号无效时返回全 0。
     * @note 在任务里调用，不阻塞。
     */
    [[nodiscard]] Statistics statistics() const noexcept;

  private:
    std::uint8_t index_; // 由 port 或 mock 解释
};

} // namespace hal

#endif // HAL_UART_HPP
