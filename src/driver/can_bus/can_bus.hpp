#ifndef DRIVER_CAN_BUS_CAN_BUS_HPP
#define DRIVER_CAN_BUS_CAN_BUS_HPP

#include "hal/can.hpp"
#include "hal/os/clock.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace device
{

/**
 * @brief 一条 CAN 总线上的帧分发（Decision-20261102-03 D9）：这条总线唯一的接收方，把收到的帧按帧 ID 分给登记的驱动。
 *        同一条总线上因此可以挂几个驱动（DJI 电机、达妙电机、超级电容……），互不知道对方。
 *
 * - 驱动在初始化阶段用 attach() 登记：一个 Client（上下文 + 收帧回调 + 每周期回调）与它要收的帧 ID 区间；
 *   区间与别人已登记的重叠时 attach() 失败（装配错误，boot 应失败）。
 * - 没有登记项认领的帧交给兜底（attach_fallback()，可选），否则计数后丢掉。
 * - receive() 取空接收队列并分发；transmit(now) 按登记顺序调用每个驱动的每周期回调（老化、状态机、发送）。
 *   驱动直接用 bus() 发送：hal::Can::send() 允许多方调用。
 * - 一条总线只归一个任务（D3）：receive()、transmit() 与读写这条总线上关节的能力在同一个任务里调用，不做跨任务同步。
 *
 * 开销：每帧查一遍登记表（最多 MAXIMUM_FILTERS 项）加一次间接调用。1 Mbps 下一条总线最多约 9000 帧/秒，可以忽略。
 */
class CanBus final
{
  public:
    static constexpr std::size_t MAXIMUM_CLIENTS{4U};
    static constexpr std::size_t MAXIMUM_FILTERS{16U};

    /** 收到登记的帧时调用（在 receive() 里，同一个任务）。 */
    using FrameHandler = void (*)(void *context, const hal::Can::Frame &frame) noexcept;
    /** 每周期调用一次（在 transmit() 里）；只收不发的驱动可以不给。 */
    using TransmitHandler = void (*)(void *context, hal::os::Microseconds now) noexcept;

    struct Client final
    {
        void *context{nullptr};
        FrameHandler on_frame{nullptr};
        TransmitHandler on_transmit{nullptr};
    };

    /** 帧 ID 区间（含两端）。 */
    struct Filter final
    {
        std::uint32_t first{0U};
        std::uint32_t last{0U};
        hal::Can::IdentifierType type{hal::Can::IdentifierType::STANDARD};
    };

    struct Statistics final
    {
        std::uint32_t received_count{0U};  // 取出并分发的帧（含交给兜底的）
        std::uint32_t unclaimed_count{0U}; // 没人认领、也没有兜底而丢掉的帧
        hal::Can::Statistics port{};       // port 的计数
    };

    constexpr explicit CanBus(hal::Can bus) noexcept : bus_{bus} {}
    CanBus(const CanBus &) = delete;
    CanBus &operator=(const CanBus &) = delete;
    CanBus(CanBus &&) = delete;
    CanBus &operator=(CanBus &&) = delete;

    /**
     * @brief 登记一个驱动与它要收的帧 ID 区间（初始化阶段）。
     * @return 参数无效（没有收帧回调、区间颠倒或超出标识符范围）、与已登记的区间重叠、登记表满时返回 false，什么都不登记。
     */
    [[nodiscard]] bool attach(const Client &client, const Filter *filters, std::size_t count) noexcept;

    /** @brief 登记兜底：收下没有登记项认领的帧（总线记录器、协议转换）。只能有一个。 */
    [[nodiscard]] bool attach_fallback(const Client &client) noexcept;

    /**
     * @brief 占住这条总线的接收（只占一次，可以重复调用）。在调度开始前调用，attach() 之前之后都可以。
     * @return 总线没启动或接收已被别人占用时返回 false。
     */
    [[nodiscard]] bool init() noexcept;

    [[nodiscard]] bool is_ready() const noexcept
    {
        return claimed_;
    }

    /** @brief 取空接收队列，按帧 ID 分给登记项。init() 成功之前什么都不做。 */
    void receive() noexcept;

    /** @brief 按登记顺序调用每个驱动的每周期回调。init() 成功之前什么都不做。 */
    void transmit(hal::os::Microseconds now) noexcept;

    /** @brief 驱动用它发送。 */
    [[nodiscard]] constexpr hal::Can bus() const noexcept
    {
        return bus_;
    }

    [[nodiscard]] Statistics statistics() const noexcept;

  private:
    struct Route final
    {
        Filter filter{};
        std::uint8_t client{0U};
    };

    [[nodiscard]] bool add_client(const Client &client, std::uint8_t &index) noexcept;

    hal::Can bus_;
    std::array<Client, MAXIMUM_CLIENTS> clients_{};
    std::size_t client_count_{0U};
    std::array<Route, MAXIMUM_FILTERS> routes_{};
    std::size_t route_count_{0U};
    Client fallback_{};
    bool has_fallback_{false};
    bool claimed_{false};
    std::uint32_t received_count_{0U};
    std::uint32_t unclaimed_count_{0U};
};

/**
 * @brief 每秒 frames_per_second 帧 8 字节标准帧占这条总线的百分比（向上取整）。
 *        每帧按位填充最坏情况 135 位计（含帧间隔）。组合根用它做编译期负载检查（Decision-20261102-03）。
 */
[[nodiscard]] constexpr std::uint32_t can_load_percent(std::uint32_t frames_per_second,
                                                       std::uint32_t bit_rate = 1000000U) noexcept
{
    constexpr std::uint64_t BITS_PER_FRAME{135U};
    const std::uint64_t bits = static_cast<std::uint64_t>(frames_per_second) * BITS_PER_FRAME * 100U;
    return bit_rate == 0U ? 0U : static_cast<std::uint32_t>((bits + bit_rate - 1U) / bit_rate);
}

} // namespace device

#endif // DRIVER_CAN_BUS_CAN_BUS_HPP
