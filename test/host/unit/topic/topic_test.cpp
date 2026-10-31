// middleware_topic（架构 v2 §3.4，Decision-20261031-05），用 platform_mock 的 hal_os 验证：
// 常量初始化与清单写法、单发布者、update() 的新数据判断与丢失计数、观测计数、发布只进一次临界区、
// wait() 的立即返回、跨线程唤醒、超时（跨回绕）、旧通知不提前结束等待、等待槽上限、直接登记的 Signal、
// 一次发布叫醒全部等待者，
// 以及一个发布线程对三个订阅者（阻塞等待、轮询、模拟中断）的压力测试。
#include "hal/os/clock.hpp"
#include "hal/os/critical_section.hpp"
#include "hal/os/signal.hpp"
#include "middleware/topic/topic.hpp"
#include "mock_os.hpp"

#include <array>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <thread>
#include <utility>
#include <vector>

namespace
{

using hal::os::NO_WAIT;
using hal::os::WAIT_FOREVER;

// 消息正好 MAX_MESSAGE_SIZE 字节：每个字都由序号算出，读到写了一半的数据就能发现
struct Sample
{
    std::uint32_t sequence;
    std::array<std::uint32_t, 15> words;
};
static_assert(sizeof(Sample) == topic::MAX_MESSAGE_SIZE);

Sample make_sample(std::uint32_t sequence)
{
    Sample sample{};
    sample.sequence = sequence;
    for (std::uint32_t i = 0U; i < sample.words.size(); ++i)
    {
        sample.words[i] = sequence * (i + 3U) + i;
    }
    return sample;
}

bool is_consistent(const Sample &sample)
{
    for (std::uint32_t i = 0U; i < sample.words.size(); ++i)
    {
        if (sample.words[i] != sample.sequence * (i + 3U) + i)
        {
            return false;
        }
    }
    return true;
}

struct Small
{
    float value;
    bool valid;
};

// ---- 常量初始化与清单写法（app/topics/README.md） ----

// 构造函数都是 constexpr，能在编译期求值；作全局对象时就是常量初始化
constexpr bool constructs_at_compile_time()
{
    topic::Topic<Small> local_topic{"test.local"};
    topic::Subscriber<Small> subscriber{local_topic};
    static_cast<void>(subscriber);
    const topic::Publisher<Small> publisher{};
    return !publisher;
}
static_assert(constructs_at_compile_time());

// 全局对象的写法同 app/topics（产品清单里的实例不加 g_，命名检查的处理见 README）
topic::Topic<Sample> g_sample{"test.sample"};
topic::Topic<Small> g_small{"test.small"};
topic::Subscriber<Small> g_small_subscriber{g_small};

// 调试通道用的清单：全部 topic 的观测接口，编译期常量
constexpr std::array<const topic::TopicBase *, 2> ALL_TOPICS{&g_sample, &g_small};

// 等到有 count 个线程在 wait() 里睡下；5 s 真实时间内没等到就失败
void wait_for_sleepers(std::uint32_t count)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{5};
    while (mock::waiting_count() != count)
    {
        assert(std::chrono::steady_clock::now() < deadline);
        std::this_thread::yield();
    }
}

// 给等待者一段真实时间，让它有机会（错误地）返回
void let_waiter_run()
{
    std::this_thread::sleep_for(std::chrono::milliseconds{20});
}

void manifest_lists_topics()
{
    assert(ALL_TOPICS[0]->name() == g_sample.name());
    assert(ALL_TOPICS[1]->name() == g_small.name());
    for (const topic::TopicBase *entry : ALL_TOPICS)
    {
        assert(entry->publish_count() == 0U);
        assert(entry->last_publish_us() == 0U);
    }
}

void only_one_publisher()
{
    mock::reset();
    topic::Topic<Small> small{"test.single"};
    topic::Publisher<Small> first = small.claim_publisher();
    assert(first);
    topic::Publisher<Small> second = small.claim_publisher();
    assert(!second);

    // 无效句柄发布什么也不做
    second.publish(Small{1.0F, true});
    assert(small.publish_count() == 0U);

    // 移动转移有效性，被移走的句柄无效
    topic::Publisher<Small> moved = std::move(first);
    assert(moved);
    assert(!first); // 检查移走后的状态
    first.publish(Small{2.0F, true});
    assert(small.publish_count() == 0U);
    moved.publish(Small{3.0F, true});
    assert(small.publish_count() == 1U);

    // 被移走以后也不能再取得：发布者只有一个
    assert(!small.claim_publisher());

    // 全局对象同样
    topic::Publisher<Small> global_publisher = g_small.claim_publisher();
    assert(global_publisher);
    assert(!g_small.claim_publisher());
    global_publisher.publish(Small{4.0F, true});
    Small small_message{};
    assert(g_small_subscriber.update(small_message));
    assert(small_message.value == 4.0F && small_message.valid);
}

void update_reads_new_data_and_counts_lost()
{
    mock::reset();
    topic::Topic<Sample> sample_topic{"test.update"};
    topic::Publisher<Sample> publisher = sample_topic.claim_publisher();
    topic::Subscriber<Sample> fast{sample_topic};
    topic::Subscriber<Sample> slow{sample_topic};

    Sample message = make_sample(99U);
    assert(!fast.update(message)); // 还没发布过：不拷贝
    assert(message.sequence == 99U);

    mock::set_now_us(1000U);
    publisher.publish(make_sample(1U));
    assert(sample_topic.publish_count() == 1U);
    assert(sample_topic.last_publish_us() == 1000U);
    assert(fast.update(message));
    assert(message.sequence == 1U && is_consistent(message));
    assert(!fast.update(message)); // 读过了
    assert(fast.lost_count() == 0U);

    mock::set_now_us(2000U);
    publisher.publish(make_sample(2U));
    publisher.publish(make_sample(3U));
    publisher.publish(make_sample(4U));
    assert(sample_topic.publish_count() == 4U);
    assert(sample_topic.last_publish_us() == 2000U);
    assert(fast.update(message));
    assert(message.sequence == 4U && is_consistent(message)); // 读最新的，不排队
    assert(fast.lost_count() == 2U);                          // 2、3 被覆盖

    // 订阅者各记各的：slow 从没读过，前三次发布都算错过
    assert(slow.update(message));
    assert(message.sequence == 4U);
    assert(slow.lost_count() == 3U);
}

void publish_enters_critical_section_once()
{
    mock::reset();
    topic::Topic<Sample> sample_topic{"test.critical"};
    topic::Publisher<Sample> publisher = sample_topic.claim_publisher();
    topic::Subscriber<Sample> subscriber{sample_topic};
    assert(subscriber.enable_wait());

    // 有等待者时 publish() 要叫醒它：Signal::notify() 在临界区内调用的话 mock 会 abort()
    const std::uint32_t before = mock::critical_entry_count();
    publisher.publish(make_sample(1U));
    assert(mock::critical_entry_count() == before + 1U);

    // 读不进临界区
    Sample message{};
    assert(subscriber.update(message));
    assert(subscriber.wait(message, NO_WAIT) == false);
    assert(mock::critical_entry_count() == before + 1U);
}

void wait_returns_unread_data_immediately()
{
    mock::reset();
    topic::Topic<Sample> sample_topic{"test.immediate"};
    topic::Publisher<Sample> publisher = sample_topic.claim_publisher();
    topic::Subscriber<Sample> subscriber{sample_topic};
    assert(subscriber.enable_wait());

    Sample message{};
    assert(!subscriber.wait(message, NO_WAIT));
    publisher.publish(make_sample(7U));
    assert(subscriber.wait(message, NO_WAIT));
    assert(message.sequence == 7U);
    assert(!subscriber.wait(message, NO_WAIT));

    // 有没读过的数据时，一直等的 wait() 也立即返回，不用推进时间
    publisher.publish(make_sample(8U));
    assert(subscriber.wait(message, WAIT_FOREVER));
    assert(message.sequence == 8U);
    assert(mock::waiting_count() == 0U);
}

void publish_wakes_waiter()
{
    mock::reset();
    topic::Topic<Sample> sample_topic{"test.wake"};
    topic::Publisher<Sample> publisher = sample_topic.claim_publisher();
    topic::Subscriber<Sample> subscriber{sample_topic};
    assert(subscriber.enable_wait());

    Sample received{};
    std::atomic<bool> result{false};
    std::thread waiter{[&]() { result.store(subscriber.wait(received, WAIT_FOREVER)); }};
    wait_for_sleepers(1U);
    publisher.publish(make_sample(42U));
    waiter.join();
    assert(result.load());
    assert(received.sequence == 42U && is_consistent(received));
    assert(hal::os::now_us() == 0U); // 没有推进时间
}

void wait_times_out()
{
    mock::reset();
    // 起点放在回绕前，超时时刻落在回绕之后
    mock::set_now_us(0xFFFFFF00U);
    topic::Topic<Sample> sample_topic{"test.timeout"};
    topic::Subscriber<Sample> subscriber{sample_topic};
    assert(subscriber.enable_wait());

    Sample received = make_sample(5U);
    std::atomic<bool> done{false};
    std::atomic<bool> result{true};
    std::thread waiter{[&]()
                       {
                           result.store(subscriber.wait(received, 0x200U));
                           done.store(true);
                       }};
    wait_for_sleepers(1U);

    mock::advance_us(0x1FFU); // 差 1 µs
    let_waiter_run();
    assert(!done.load());

    mock::advance_us(1U);
    waiter.join();
    assert(!result.load());
    assert(received.sequence == 5U); // 超时不改 out
}

// 发布留下的通知在数据被 update() 读走后就是旧的；wait() 醒来发现没有新数据，要接着睡
void stale_notification_does_not_end_wait()
{
    mock::reset();
    topic::Topic<Sample> sample_topic{"test.stale"};
    topic::Publisher<Sample> publisher = sample_topic.claim_publisher();
    topic::Subscriber<Sample> subscriber{sample_topic};
    assert(subscriber.enable_wait());

    Sample received{};
    publisher.publish(make_sample(1U));
    assert(subscriber.update(received)); // 通知还留在 Signal 里

    std::atomic<bool> done{false};
    std::atomic<bool> result{false};
    std::thread waiter{[&]()
                       {
                           result.store(subscriber.wait(received, 1000U));
                           done.store(true);
                       }};
    wait_for_sleepers(1U); // 取走旧通知以后又睡下了
    let_waiter_run();
    assert(!done.load());

    publisher.publish(make_sample(2U));
    waiter.join();
    assert(result.load());
    assert(received.sequence == 2U);
}

// 没调用 enable_wait() 的订阅者，发布叫不醒它，只能等到超时；超时时有新数据仍返回 true
void wait_without_slot_only_times_out()
{
    mock::reset();
    topic::Topic<Sample> sample_topic{"test.no_slot"};
    topic::Publisher<Sample> publisher = sample_topic.claim_publisher();
    topic::Subscriber<Sample> subscriber{sample_topic};

    Sample received{};
    std::atomic<bool> done{false};
    std::atomic<bool> result{false};
    std::thread waiter{[&]()
                       {
                           result.store(subscriber.wait(received, 1000U));
                           done.store(true);
                       }};
    wait_for_sleepers(1U);
    publisher.publish(make_sample(3U));
    let_waiter_run();
    assert(!done.load());

    mock::advance_us(1000U);
    waiter.join();
    assert(result.load());
    assert(received.sequence == 3U);
}

void wait_slots_are_limited()
{
    mock::reset();
    topic::Topic<Small> small{"test.slots"};
    std::array<topic::Subscriber<Small>, topic::WAIT_SLOTS + 1U> subscribers{topic::Subscriber<Small>{small},
                                                                             topic::Subscriber<Small>{small},
                                                                             topic::Subscriber<Small>{small},
                                                                             topic::Subscriber<Small>{small},
                                                                             topic::Subscriber<Small>{small}};
    for (std::size_t i = 0U; i < topic::WAIT_SLOTS; ++i)
    {
        assert(subscribers[i].enable_wait());
    }
    assert(subscribers[0].enable_wait()); // 重复登记不占新槽
    assert(!subscribers[topic::WAIT_SLOTS].enable_wait());
}

// 任务运行器的用法（Decision-20261031-12）：直接在 topic 上登记一个 Signal，发布就叫醒它；
// 与订阅者的 enable_wait() 共用 WAIT_SLOTS 个等待槽
void signal_registered_on_topic_is_notified()
{
    mock::reset();
    topic::Topic<Small> small{"test.runner"};
    topic::Publisher<Small> publisher = small.claim_publisher();
    hal::os::Signal trigger;
    assert(small.add_waiter(trigger));
    assert(small.add_waiter(trigger)); // 重复登记不占新槽
    assert(!trigger.wait(NO_WAIT));

    publisher.publish(Small{1.0F, true});
    assert(trigger.wait(NO_WAIT));
    assert(!trigger.wait(NO_WAIT)); // 多次发布合并、取走后清除

    std::array<topic::Subscriber<Small>, topic::WAIT_SLOTS> subscribers{topic::Subscriber<Small>{small},
                                                                        topic::Subscriber<Small>{small},
                                                                        topic::Subscriber<Small>{small},
                                                                        topic::Subscriber<Small>{small}};
    for (std::size_t i = 0U; i + 1U < topic::WAIT_SLOTS; ++i)
    {
        assert(subscribers[i].enable_wait());
    }
    assert(!subscribers[topic::WAIT_SLOTS - 1U].enable_wait()); // 第一个槽给了 trigger
}

void one_publish_wakes_all_waiters()
{
    mock::reset();
    topic::Topic<Sample> sample_topic{"test.broadcast"};
    topic::Publisher<Sample> publisher = sample_topic.claim_publisher();
    topic::Subscriber<Sample> first{sample_topic};
    topic::Subscriber<Sample> second{sample_topic};
    assert(first.enable_wait());
    assert(second.enable_wait());

    Sample first_received{};
    Sample second_received{};
    std::atomic<bool> first_result{false};
    std::atomic<bool> second_result{false};
    std::thread first_waiter{[&]() { first_result.store(first.wait(first_received, WAIT_FOREVER)); }};
    wait_for_sleepers(1U);
    std::thread second_waiter{[&]() { second_result.store(second.wait(second_received, WAIT_FOREVER)); }};
    wait_for_sleepers(2U);
    publisher.publish(make_sample(9U));
    first_waiter.join();
    second_waiter.join();
    assert(first_result.load() && second_result.load());
    assert(first_received.sequence == 9U && second_received.sequence == 9U);
}

// 一个发布线程，三个订阅线程：
//   - 阻塞等待：wait(WAIT_FOREVER) 直到读到最后一条，漏了最后一次唤醒就会一直睡（mock 在 10 s 后 abort()）；
//   - 轮询：update()；
//   - 模拟中断：每次读之前先进临界区，相当于"只在没关中断时才能运行"的中断服务程序；
//     读完紧接着再读一次，不应有新数据（写入不在临界区内时，写者会在两次读之间写完）。
// 每个订阅者检查：数据自洽（没有读到写了一半的）、序号只增不减、读到的条数加丢失次数等于最后读到的序号。
void stress_publisher_against_subscribers()
{
    mock::reset();
    constexpr std::uint32_t PUBLISH_COUNT = 200000U;
    topic::Topic<Sample> sample_topic{"test.stress"};
    topic::Publisher<Sample> publisher = sample_topic.claim_publisher();
    topic::Subscriber<Sample> waiting{sample_topic};
    topic::Subscriber<Sample> polling{sample_topic};
    topic::Subscriber<Sample> interrupt{sample_topic};
    assert(waiting.enable_wait());

    struct Result
    {
        std::uint32_t received{0U};
        std::uint32_t last{0U};
        bool ok{true};
    };
    auto check = [](Result &result, const Sample &message)
    {
        result.ok = result.ok && is_consistent(message) && message.sequence > result.last;
        result.last = message.sequence;
        ++result.received;
    };

    Result waiting_result;
    Result polling_result;
    Result interrupt_result;
    std::vector<std::thread> threads;
    threads.emplace_back(
        [&]()
        {
            Sample message{};
            while (waiting_result.last != PUBLISH_COUNT)
            {
                if (waiting.wait(message, WAIT_FOREVER))
                {
                    check(waiting_result, message);
                }
            }
        });
    threads.emplace_back(
        [&]()
        {
            Sample message{};
            while (polling_result.last != PUBLISH_COUNT)
            {
                if (polling.update(message))
                {
                    check(polling_result, message);
                }
            }
        });
    threads.emplace_back(
        [&]()
        {
            Sample message{};
            Sample again{};
            while (interrupt_result.last != PUBLISH_COUNT)
            {
                bool updated = false;
                {
                    const hal::os::CriticalSection critical_section;
                    updated = interrupt.update(message);
                    // 写入在临界区内：关着中断时写者动不了，紧接着再读不会有新数据
                    interrupt_result.ok = interrupt_result.ok && !interrupt.update(again);
                }
                if (updated)
                {
                    check(interrupt_result, message);
                }
            }
        });

    for (std::uint32_t sequence = 1U; sequence <= PUBLISH_COUNT; ++sequence)
    {
        publisher.publish(make_sample(sequence));
    }
    for (std::thread &thread : threads)
    {
        thread.join();
    }

    assert(sample_topic.publish_count() == PUBLISH_COUNT);
    const std::array<std::pair<const Result *, const topic::Subscriber<Sample> *>, 3> all{
        {{&waiting_result, &waiting}, {&polling_result, &polling}, {&interrupt_result, &interrupt}}};
    for (const auto &[result, subscriber] : all)
    {
        assert(result->ok);
        assert(result->last == PUBLISH_COUNT);
        assert(result->received + subscriber->lost_count() == PUBLISH_COUNT);
    }
}

} // namespace

int main()
{
    manifest_lists_topics();
    only_one_publisher();
    update_reads_new_data_and_counts_lost();
    publish_enters_critical_section_once();
    wait_returns_unread_data_immediately();
    publish_wakes_waiter();
    wait_times_out();
    stale_notification_does_not_end_wait();
    wait_without_slot_only_times_out();
    wait_slots_are_limited();
    signal_registered_on_topic_is_notified();
    one_publish_wakes_all_waiters();
    stress_publisher_against_subscribers();
    return 0;
}
