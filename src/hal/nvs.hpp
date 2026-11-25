#ifndef HAL_NVS_HPP
#define HAL_NVS_HPP

#include <cstddef>
#include <cstdint>

// 硬件契约：非易失存储区（一块可以反复擦写的 Flash 区）。句柄由各板的 board.hpp 给出，
// 实现由 port 提供，宿主测试由 platform/mock 提供。

namespace hal
{

/**
 * @brief 一块非易失存储区的句柄：一个编号对应 port 外设表里的一块 Flash 区。
 *
 * 语义按 Flash 来（不隐藏物理约束，上层的日志式记录要自己利用它们）：
 * - read()：任意偏移、任意长度，读的是 Flash 里的原始内容；任务与框架中断里都能调用，不阻塞；
 * - write()：只能往**擦除态**（0xFF）上写，偏移与长度都要按 write_granularity() 对齐（DM-MC02 为 32 B，
 *   H7 的 Flash word）；地址被占用时返回 NOT_ERASED，不会覆盖旧数据；
 * - erase()：整区擦除（一个扇区），任务里调用；DM-MC02 的 128 KB 扇区要 1–2 s，这期间 CPU 取指会停，
 *   所以只在启动期、还没有任务跑的时候调用。擦到一半掉电会让整区内容不可用（记录层用 CRC 兜住）。
 *
 * 句柄只含一个编号，可以拷贝，按值存进能力或驱动。编号只有 port 与 mock 能读。
 */
class Nvs final
{
  public:
    enum class Result : std::uint8_t
    {
        OK = 0U,               // 成功
        NOT_READY = 1U,        // hal::init() 还没成功，或这块板的这一区没有实现
        INVALID_ARGUMENT = 2U, // 编号无效、指针为空、长度为 0 或超出这一区
        NOT_ALIGNED = 3U,      // 写偏移或长度不是 write_granularity() 的整数倍
        NOT_ERASED = 4U,       // 要写的位置不是擦除态，必须先整区擦除
        UNSUPPORTED = 5U,      // 编号超出 port 的外设表
        ERROR = 6U             // Flash 操作失败（写保护、ECC、供电跌落等）
    };

    constexpr explicit Nvs(std::uint8_t index) noexcept : index_{index} {}

    /**
     * @brief 这一区的字节数（0 表示编号无效或这块板没有这一区）。
     * @note 任务与框架中断里都能调用，不阻塞。
     */
    [[nodiscard]] std::size_t size() const noexcept;

    /**
     * @brief 最小可写对齐，字节数（DM-MC02 为 32）。
     * @note 任务与框架中断里都能调用，不阻塞。
     */
    [[nodiscard]] std::size_t write_granularity() const noexcept;

    /**
     * @brief 读原始内容。Flash 是内存映射的，读不阻塞，任务与框架中断里都能调用。
     * @return 成功返回 OK；编号无效或这块板没有这一区返回 UNSUPPORTED；
     *         指针为空、长度为 0、offset + length 超出这一区返回 INVALID_ARGUMENT。
     */
    [[nodiscard]] Result read(std::size_t offset, std::uint8_t *data, std::size_t length) const noexcept;

    /**
     * @brief 把 data 写到 offset，要求 offset 与 length 按 write_granularity() 对齐、目标已经是擦除态。
     * @return 成功返回 OK；对齐不符返回 NOT_ALIGNED；目标不是擦除态返回 NOT_ERASED；其余同 read()。
     * @note 只在任务里调用。写一个 32 B 的 Flash word 约几十 µs，期间 CPU 取指会停；
     *       不要在框架中断里调用，也不要在 DMA 正在读这块 Flash 时调用。
     */
    [[nodiscard]] Result write(std::size_t offset, const std::uint8_t *data, std::size_t length) const noexcept;

    /**
     * @brief 把整区擦成 0xFF。
     * @return 成功返回 OK；编号无效或没有这一区返回 UNSUPPORTED；其余失败返回 ERROR。
     * @note 只在任务里调用，而且只在启动期（还没有任务与 DMA 在跑）调用：DM-MC02 上要 1–2 s。
     */
    [[nodiscard]] Result erase() const noexcept;

  private:
    std::uint8_t index_; // 由 port 或 mock 解释
};

} // namespace hal

#endif // HAL_NVS_HPP
