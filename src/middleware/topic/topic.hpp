#ifndef MIDDLEWARE_TOPIC_TOPIC_HPP
#define MIDDLEWARE_TOPIC_TOPIC_HPP

#include "hal/os/clock.hpp"
#include "hal/os/critical_section.hpp"
#include "hal/os/signal.hpp"
#include "utils/concurrency/latest_value.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>

// topic：一种数据的发布/订阅通道（架构 v2 §3.4）。
//
// - 最新值语义：发布覆盖旧值，订阅者读最新一份；不排队，丢了几次可以观测。
// - 每个 topic 只有一个发布者：claim_publisher() 每个 topic 只能成功一次，在初始化阶段调用。
// - 发布在任务与框架中断里都能做；update() 在任务与框架中断里都能用；wait() 只在任务里用。
// - 只用 utils 与 hal/os，裸机与宿主测试都能用；对象可以常量初始化，作全局对象时不依赖构造顺序。
//
// 读写竞态：发布时用 hal::os::CriticalSection 包住写入。
// 写入期间任何读者都打断不了写者，所以读者不会在写入中途无限重试：高优先级任务读低优先级任务
// 发布的 topic、框架中断读任务发布的 topic 都安全。代价是每次发布关一次框架中断，时长约等于一次消息拷贝，
// 所以消息大小有上限 MAX_MESSAGE_SIZE。框架线以上的中断不得发布，也不得读 topic（与 hal/os 的规则一致）。

namespace topic
{

/**
 * 消息大小上限（字节）：发布时在临界区内拷贝消息，上限限制了关中断的时长。
 * 暂定值，F2 在板上实测临界区时长后再定。
 */
inline constexpr std::size_t MAX_MESSAGE_SIZE{64U};

/** 每个 topic 的等待槽数：最多这么多个订阅者能在同一个 topic 上用 wait()。 */
inline constexpr std::size_t WAIT_SLOTS{4U};

template <typename T>
class Topic;

/**
 * @brief 与消息类型无关的部分：名字、发布者检查、等待槽、观测计数。
 *
 * 调试通道经它列出全部 topic：清单里放 `const TopicBase *` 的数组（见 app/topics/README.md）。
 * 观测接口在任何上下文都能调用，读到的是近似值：两个计数之间、计数与消息之间不保证一致。
 */
class TopicBase
{
  public:
    TopicBase(const TopicBase &) = delete;
    TopicBase &operator=(const TopicBase &) = delete;
    TopicBase(TopicBase &&) = delete;
    TopicBase &operator=(TopicBase &&) = delete;

    /** topic 的名字，如 "imu.attitude"。 */
    [[nodiscard]] const char *name() const noexcept
    {
        return name_;
    }

    /** 发布次数，按 2^32 回绕。0 表示还没发布过。 */
    [[nodiscard]] std::uint32_t publish_count() const noexcept
    {
        return publish_count_.load(std::memory_order_relaxed);
    }

    /** 最后一次发布的时刻（hal::os::now_us()）；还没发布过时为 0。 */
    [[nodiscard]] hal::os::Microseconds last_publish_us() const noexcept
    {
        return last_publish_us_.load(std::memory_order_relaxed);
    }

    /**
     * @brief 占一个等待槽：此后每次发布都会叫醒 signal。
     * @return false：槽已满（WAIT_SLOTS 个）。同一个 signal 重复登记只占一个槽；槽只增不减。
     * @note 订阅者经 Subscriber::enable_wait() 登记；任务运行器按 topic 触发任务时直接登记自己的 Signal。
     *       初始化阶段调用；任务与框架中断里都能调用，不阻塞。
     */
    [[nodiscard]] bool add_waiter(hal::os::Signal &signal) noexcept
    {
        const hal::os::CriticalSection critical_section;
        for (auto &slot : waiters_)
        {
            hal::os::Signal *const current = slot.load(std::memory_order_relaxed);
            if (current == &signal)
            {
                return true;
            }
            if (current == nullptr)
            {
                // release：发布者在临界区外读到这个指针时，Signal 本身已经构造好
                slot.store(&signal, std::memory_order_release);
                return true;
            }
        }
        return false;
    }

  protected:
    /** @param name 必须是字符串字面量或其他静态存储的字符串，topic 只保存指针。 */
    constexpr explicit TopicBase(const char *name) noexcept : name_{name} {}
    ~TopicBase() = default;

    /** 发布者检查：第一次调用返回 true，以后都返回 false。 */
    bool claim_publisher_slot() noexcept
    {
        const hal::os::CriticalSection critical_section;
        if (publisher_claimed_)
        {
            return false;
        }
        publisher_claimed_ = true;
        return true;
    }

    /** 记录一次发布。调用方在临界区内，计数只有发布者写，"读一次、写一次"代替原子读改写（规则 13）。 */
    void record_publish(hal::os::Microseconds now) noexcept
    {
        publish_count_.store(publish_count_.load(std::memory_order_relaxed) + 1U, std::memory_order_relaxed);
        last_publish_us_.store(now, std::memory_order_relaxed);
    }

    /** 叫醒全部已登记的等待者。必须在临界区外调用：Signal 不能在临界区内使用。 */
    void notify_waiters() noexcept
    {
        // 槽按顺序占用、从不释放，遇到第一个空槽就说明后面都是空的
        for (const auto &slot : waiters_)
        {
            hal::os::Signal *const signal = slot.load(std::memory_order_acquire);
            if (signal == nullptr)
            {
                break;
            }
            signal->notify();
        }
    }

  private:
    const char *name_;
    bool publisher_claimed_{false}; // 只在临界区内读写
    std::atomic<std::uint32_t> publish_count_{0U};
    std::atomic<hal::os::Microseconds> last_publish_us_{0U};
    std::array<std::atomic<hal::os::Signal *>, WAIT_SLOTS> waiters_{}; // 只增不减，在临界区内写
};

/**
 * @brief 发布句柄：由 Topic::claim_publisher() 取得，每个 topic 只有一个有效句柄。
 *
 * 默认构造的、被移走的、claim_publisher() 失败时返回的句柄都无效，publish() 什么也不做。
 * 可移动、不可拷贝，避免同一个 topic 出现两个发布者。
 */
template <typename T>
class Publisher final
{
  public:
    constexpr Publisher() noexcept = default;
    ~Publisher() = default;

    Publisher(const Publisher &) = delete;
    Publisher &operator=(const Publisher &) = delete;

    Publisher(Publisher &&other) noexcept : topic_{other.topic_}
    {
        other.topic_ = nullptr;
    }

    Publisher &operator=(Publisher &&other) noexcept
    {
        if (this != &other)
        {
            topic_ = other.topic_;
            other.topic_ = nullptr;
        }
        return *this;
    }

    /** 句柄是否有效。初始化时检查，无效则初始化失败。 */
    [[nodiscard]] constexpr explicit operator bool() const noexcept
    {
        return topic_ != nullptr;
    }

    /**
     * @brief 发布一条消息：覆盖旧值，叫醒正在 wait() 的订阅者。
     * @note 任务与框架中断里都能调用，不阻塞；不能在 CriticalSection 内调用（要叫醒等待者）。
     * @note 关框架中断的时长约等于一次消息拷贝。
     */
    void publish(const T &message) noexcept
    {
        if (topic_ != nullptr)
        {
            topic_->publish(message);
        }
    }

  private:
    friend class Topic<T>;

    constexpr explicit Publisher(Topic<T> &source) noexcept : topic_{&source} {}

    Topic<T> *topic_{nullptr};
};

/**
 * @brief 订阅者：读 topic 的最新值，或阻塞等到下一次发布。
 *
 * - 一个订阅者对象只给一个上下文（一个任务或一个中断）用；别的上下文要读，各建一个订阅者。
 * - 记住自己读到的版本：update()、wait() 只在有没读过的新数据时拷贝；中间错过的发布计入 lost_count()。
 *   订阅者构造后还没读过时，此前的发布都算错过。
 * - 要用 wait() 的，初始化阶段先调用 enable_wait() 占一个等待槽，失败则初始化失败。
 * - 构造函数是 constexpr，可以常量初始化；不可拷贝、不可移动（内含 hal::os::Signal）。
 */
template <typename T>
class Subscriber final
{
  public:
    constexpr explicit Subscriber(Topic<T> &source) noexcept : topic_{&source} {}
    ~Subscriber() = default;

    Subscriber(const Subscriber &) = delete;
    Subscriber &operator=(const Subscriber &) = delete;
    Subscriber(Subscriber &&) = delete;
    Subscriber &operator=(Subscriber &&) = delete;

    /**
     * @brief 在 topic 上占一个等待槽，此后每次发布都会叫醒这个订阅者的 wait()。
     * @return false：槽已满（WAIT_SLOTS 个）。重复调用只占一个槽。
     * @note 初始化阶段调用；任务与框架中断里都能调用，不阻塞。
     */
    [[nodiscard]] bool enable_wait() noexcept
    {
        return topic_->add_waiter(signal_);
    }

    /**
     * @brief 有没读过的新数据就拷到 out 并返回 true；没有返回 false，out 不变。
     * @note 任务与框架中断里都能调用，不阻塞（读者遇到写入中途会重试，写入在临界区内，不会无限重试）。
     */
    bool update(T &out) noexcept
    {
        const concurrency::LatestValue<T> &value = topic_->value_;
        if (value.version() == last_version_)
        {
            return false;
        }
        const std::uint32_t version = value.read(out);
        // 版本号按 2^31 回绕，差值要先相减再按位与
        const std::uint32_t skipped = (version - last_version_ - 1U) & concurrency::LatestValue<T>::VERSION_MASK;
        if (skipped != 0U)
        {
            // 只有本订阅者写 lost_count_，观测者只读，"读一次、写一次"代替原子读改写（规则 13）
            lost_count_.store(lost_count_.load(std::memory_order_relaxed) + skipped, std::memory_order_relaxed);
        }
        last_version_ = version;
        return true;
    }

    /**
     * @brief 有没读过的新数据就立即拷到 out；没有就睡眠，直到下一次发布或超时。
     * @param timeout 最长等待时间（µs）：hal::os::NO_WAIT 只检查不睡眠，hal::os::WAIT_FOREVER 一直等。
     *        超时的粒度同 hal::os::Signal::wait()。
     * @return true：拷到了新数据；false：超时，out 不变。
     * @note 只能在任务里调用，不能在中断里、CriticalSection 内调用。
     * @note 没有调用过 enable_wait() 时，发布叫不醒它，只能等到超时；超时时有新数据仍返回 true。
     */
    [[nodiscard]] bool wait(T &out, hal::os::Microseconds timeout) noexcept
    {
        // 先检查再睡眠：检查之后来的发布会在 Signal 里留下通知，下面的 wait() 立即返回，不会漏掉
        if (update(out))
        {
            return true;
        }
        const hal::os::Microseconds start = hal::os::now_us();
        for (;;)
        {
            hal::os::Microseconds remaining = hal::os::WAIT_FOREVER;
            if (timeout != hal::os::WAIT_FOREVER)
            {
                const hal::os::Microseconds elapsed = hal::os::elapsed_us(start);
                if (elapsed >= timeout)
                {
                    return false;
                }
                remaining = timeout - elapsed;
            }
            const bool notified = signal_.wait(remaining);
            if (update(out))
            {
                return true;
            }
            if (!notified)
            {
                return false; // 超时，期间也没有新数据
            }
            // 通知是旧的：对应的数据已经被前面的 update() 读过，接着等
        }
    }

    /** 错过的发布次数：两次读取之间被覆盖、没读到的消息数。任何上下文都能读。 */
    [[nodiscard]] std::uint32_t lost_count() const noexcept
    {
        return lost_count_.load(std::memory_order_relaxed);
    }

  private:
    Topic<T> *topic_;
    hal::os::Signal signal_{};
    std::uint32_t last_version_{0U}; // 读到的最后一个版本；0 表示还没读过（版本号从 1 开始）
    std::atomic<std::uint32_t> lost_count_{0U};
};

/**
 * @brief 一种数据的 topic：保存最新一条消息。
 *
 * 只在产品的清单（app/topics）里定义，作全局对象，构造函数是 constexpr，保证常量初始化：
 *
 * ```cpp
 * inline topic::Topic<Attitude> attitude{"imu.attitude"};
 * ```
 *
 * 消息必须可平凡拷贝，只用标准整数、浮点、bool 与定长数组，不超过 MAX_MESSAGE_SIZE 字节。
 */
template <typename T>
class Topic final : public TopicBase
{
    static_assert(std::is_trivially_copyable_v<T>, "topic 的消息必须可平凡拷贝（规则 17）");
    static_assert(sizeof(T) <= MAX_MESSAGE_SIZE,
                  "topic 的消息超过 MAX_MESSAGE_SIZE：发布时关中断的时长与消息大小成正比");

  public:
    using Message = T;

    /** @param name 必须是字符串字面量或其他静态存储的字符串。 */
    constexpr explicit Topic(const char *name) noexcept : TopicBase{name} {}
    ~Topic() = default;

    Topic(const Topic &) = delete;
    Topic &operator=(const Topic &) = delete;
    Topic(Topic &&) = delete;
    Topic &operator=(Topic &&) = delete;

    /**
     * @brief 取得发布句柄。每个 topic 只有第一次调用成功，以后返回无效句柄。
     * @note 初始化阶段调用，返回无效句柄时初始化失败（boot_init() 返回 false）。
     */
    [[nodiscard]] Publisher<T> claim_publisher() noexcept
    {
        if (!claim_publisher_slot())
        {
            return Publisher<T>{};
        }
        return Publisher<T>{*this};
    }

  private:
    friend class Publisher<T>;
    friend class Subscriber<T>;

    void publish(const T &message) noexcept
    {
        const hal::os::Microseconds now = hal::os::now_us();
        {
            const hal::os::CriticalSection critical_section;
            value_.write(message);
            record_publish(now);
        }
        notify_waiters();
    }

    concurrency::LatestValue<T> value_{};
};

} // namespace topic

#endif // MIDDLEWARE_TOPIC_TOPIC_HPP
