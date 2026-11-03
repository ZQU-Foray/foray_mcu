#ifndef HAL_USB_CDC_HPP
#define HAL_USB_CDC_HPP

#include "hal/callback.hpp"

#include <cstddef>
#include <cstdint>

// 硬件契约：USB CDC（虚拟串口）字节流。句柄由各板的 board.hpp 给出，实现由 port 提供，
// 宿主测试由 platform/mock 提供。

namespace hal
{

/**
 * @brief 一个 USB CDC 接口的句柄：设备端的虚拟串口。
 *
 * - 主机打开端口（USB 配置完成）之后才算连上（is_connected()）；断开时丢弃接收队列，进行中的发送算失败。
 * - 接收：port 在 USB 中断里把收到的字节放进软件接收队列，read() 取；队列满时丢弃并计数。
 * - 发送：write() 把数据拷走后开始发送就返回，同时只有一次发送在进行；结束（成功或失败）时调用发送通知。
 *
 * 读写接口与 hal::Uart 同形：处理字节流的代码可以对两者写成同一个模板。
 * 句柄只含一个编号，可以拷贝，按值存进驱动的配置。
 */
class UsbCdc final
{
  public:
    /** 单次 write() 最多这么多字节。 */
    static constexpr std::size_t MAX_WRITE_LENGTH{512U};

    enum class WriteResult : std::uint8_t
    {
        STARTED = 0U,          // 已拷走并开始发送
        BUSY = 1U,             // 上一次发送还没结束
        NOT_READY = 2U,        // 没有连上主机
        UNSUPPORTED = 3U,      // 保留，与 hal::Uart 同形
        INVALID_ARGUMENT = 4U, // 编号无效、缓冲区为空、长度为 0 或超过 MAX_WRITE_LENGTH
        ERROR = 5U
    };

    /** 计数从上电起累计，按 2^32 回绕。 */
    struct Statistics final
    {
        std::uint32_t received_byte_count{0U};    // 收到的字节（含丢弃的）
        std::uint32_t dropped_byte_count{0U};     // 软件接收队列满而丢弃的字节
        std::uint32_t receive_error_count{0U};    // 没连上时收到数据、重新开始接收失败等
        std::uint32_t transmitted_byte_count{0U}; // 发送成功的字节
        std::uint32_t transmit_failure_count{0U}; // 发送失败（含断开时进行中的发送）
        std::uint32_t connection_count{0U};       // 连上主机的次数
        std::uint32_t disconnection_count{0U};    // 断开的次数
    };

    constexpr explicit UsbCdc(std::uint8_t index) noexcept : index_{index} {}

    /**
     * @brief 接口已初始化（与有没有连上主机无关）。
     * @note 任务与框架中断里都能调用，不阻塞。
     */
    [[nodiscard]] bool is_ready() const noexcept;

    /**
     * @brief 主机已打开端口，可以收发。
     * @note 任务与框架中断里都能调用，不阻塞。
     */
    [[nodiscard]] bool is_connected() const noexcept;

    /**
     * @brief 从软件接收队列取最多 capacity 字节，不阻塞。
     * @return 取到的字节数；没有数据、编号无效或参数无效时为 0。
     * @note 只有一个消费者在任务里调用。
     */
    [[nodiscard]] std::size_t read(std::uint8_t *data, std::size_t capacity) const noexcept;

    /**
     * @brief 开始发送 length 字节，不阻塞。data 在返回前已经拷走。
     * @note 在任务里调用；同一接口只应有一个发送方。没有连上主机时返回 NOT_READY。
     */
    [[nodiscard]] WriteResult write(const std::uint8_t *data, std::size_t length) const noexcept;

    /**
     * @brief 登记接收通知：有字节放进软件接收队列后，在框架中断里调用 callback(context)。
     * @param callback 传 nullptr 取消登记。
     * @return 编号无效或接口没有初始化时返回 false。
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

#endif // HAL_USB_CDC_HPP
