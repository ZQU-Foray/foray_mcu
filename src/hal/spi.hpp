#ifndef HAL_SPI_HPP
#define HAL_SPI_HPP

#include "hal/callback.hpp"
#include "hal/os/clock.hpp"

#include <cstddef>
#include <cstdint>

// 硬件契约：SPI 设备。句柄由各板的 board.hpp 给出，实现由 port 提供，
// 宿主测试由 platform/mock 提供。

namespace hal
{

/**
 * @brief 一个 SPI 设备的句柄：一条 SPI 总线加上这个设备的片选，片选由 port 管。
 *
 * 能力分级（各板的 board.hpp 写明）：
 * - 必选：is_ready()、阻塞的 transmit()；
 * - 可选（按设备）：全双工 transfer()（只接了 MOSI 的设备不支持）；
 * - 可选（按板）：异步的 start_transmit()、start_transfer()、async_result()（用 DMA，板级常量 SPI_ASYNC）。
 *   支持异步的板至少接受 MAX_ASYNC_LENGTH 字节。
 * 不支持时返回 UNSUPPORTED。
 *
 * 同一条总线上的设备共用总线：一个事务没结束，同总线的其他设备得到 BUSY。
 * 句柄只含一个编号，可以拷贝，按值存进驱动的配置。
 */
class Spi final
{
  public:
    /** 支持异步的板，单次异步事务至少接受这么多字节。 */
    static constexpr std::size_t MAX_ASYNC_LENGTH{512U};

    enum class Result : std::uint8_t
    {
        COMPLETED = 0U,        // 事务完成
        STARTED = 1U,          // 异步事务已开始，结束时调用回调
        BUSY = 2U,             // 总线正被本设备或同总线的其他设备占用，稍后再试
        TIMEOUT = 3U,          // 阻塞事务超时
        NOT_READY = 4U,        // hal::init() 还没有成功初始化
        UNSUPPORTED = 5U,      // 这个设备或这块板不支持该操作
        INVALID_ARGUMENT = 6U, // 编号无效、缓冲区为空、长度为 0 或超出上限
        ERROR = 7U
    };

    constexpr explicit Spi(std::uint8_t index) noexcept : index_{index} {}

    /**
     * @brief 设备已初始化，且所在总线空闲，可以开始新事务。
     * @note 任务与框架中断里都能调用，不阻塞。
     */
    [[nodiscard]] bool is_ready() const noexcept;

    /**
     * @brief 阻塞发送：选中设备、发出 length 字节、取消选中。
     * @param timeout 最长等待时间，超时返回 TIMEOUT（按实现的计时粒度向上取整，DM-MC02 上是 1 ms）。
     * @note 只在任务里调用；总线忙时立即返回 BUSY，不等待。
     */
    [[nodiscard]] Result
    transmit(const std::uint8_t *data, std::size_t length, os::Microseconds timeout) const noexcept;

    /**
     * @brief 阻塞全双工：发出 transmit_data 的同时收进 receive_data，各 length 字节。
     * @note 同 transmit()；设备不支持全双工时返回 UNSUPPORTED。
     */
    [[nodiscard]] Result transfer(const std::uint8_t *transmit_data,
                                  std::uint8_t *receive_data,
                                  std::size_t length,
                                  os::Microseconds timeout) const noexcept;

    /**
     * @brief 开始一次异步发送（可选能力）。返回 STARTED 后，事务结束时在框架中断里调用 callback(context)，
     *        最终结果用 async_result() 读。
     * @param data 返回前已经拷走，调用方可以立即复用。
     * @param callback 可以为 nullptr（只轮询 async_result()）。回调可能在本函数返回之前就被调用。
     * @note 在任务里调用，不阻塞。总线忙返回 BUSY；不支持异步返回 UNSUPPORTED。
     */
    [[nodiscard]] Result
    start_transmit(const std::uint8_t *data, std::size_t length, Callback callback, void *context) const noexcept;

    /**
     * @brief 开始一次异步全双工（可选能力）。
     * @param transmit_data 返回前已经拷走。
     * @param receive_data 事务结束、回调被调用之前收齐；在那之前必须一直有效，调用方不能读写。
     * @note 其余同 start_transmit()；设备不支持全双工时返回 UNSUPPORTED。
     */
    [[nodiscard]] Result start_transfer(const std::uint8_t *transmit_data,
                                        std::uint8_t *receive_data,
                                        std::size_t length,
                                        Callback callback,
                                        void *context) const noexcept;

    /**
     * @brief 本设备最近一次异步事务的结果：进行中为 BUSY，结束后为 COMPLETED 或 ERROR；
     *        还没有开始过异步事务为 NOT_READY；不支持异步为 UNSUPPORTED。
     * @note 任务与框架中断里都能调用，不阻塞。
     */
    [[nodiscard]] Result async_result() const noexcept;

  private:
    std::uint8_t index_; // 由 port 或 mock 解释
};

} // namespace hal

#endif // HAL_SPI_HPP
