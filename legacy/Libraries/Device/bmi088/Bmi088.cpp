#include "Libraries/Device/bmi088/Bmi088.hpp"

namespace device
{

namespace
{
// 两个事件的时间间隔必须小于 32 位 tick 回绕周期的一半。
bool before(std::uint32_t a, std::uint32_t b) noexcept
{
    return ((a - b) & 0x80000000U) != 0U;
}
} // namespace

bool Bmi088::pop_next_sample(Bmi088SampleRecord &record, bool &gyroscope) noexcept
{
    Bmi088SampleRecord accel{}, gyro{};
    const bool has_accel = accelerometer_queue_.peek(accel);
    const bool has_gyro = gyroscope_queue_.peek(gyro);
    if (!has_accel && !has_gyro)
        return false;
    // 时间戳相同时先交付加速度，使修正恰好落在陀螺区间端点，
    // 而不是滞后一个区间。
    gyroscope = has_gyro && (!has_accel || before(gyro.drdy_tick, accel.drdy_tick));
    const auto candidate = gyroscope ? gyro.drdy_tick : accel.drdy_tick;
    const auto not_later = [candidate](std::uint32_t tick) { return tick == candidate || before(tick, candidate); };
    if (accelerometer_.is_ready() &&
        ((accelerometer_data_ready_pending_ && not_later(accelerometer_event_.tick)) ||
         (accelerometer_.has_pending_transfer() && not_later(accelerometer_inflight_.drdy_tick))))
        return false;
    if (gyroscope_.is_ready() && ((gyroscope_data_ready_pending_ && before(gyroscope_event_.tick, candidate)) ||
                                 (gyroscope_.has_pending_transfer() && before(gyroscope_inflight_.drdy_tick, candidate))))
        return false;
    return gyroscope ? gyroscope_queue_.pop(record) : accelerometer_queue_.pop(record);
}

void Bmi088::init() noexcept
{
    accelerometer_.init();
    gyroscope_.init();
    temperature_.init();
    accelerometer_queue_.reset();
    gyroscope_queue_.reset();
    accelerometer_event_ = gyroscope_event_ = {};
    accelerometer_inflight_ = gyroscope_inflight_ = {};
    accelerometer_data_ready_pending_ = gyroscope_data_ready_pending_ = false;
    turn_ = 0U;
    accelerometer_data_ready_count_ = gyroscope_data_ready_count_ = 0U;
    accelerometer_coalesced_event_count_ = gyroscope_coalesced_event_count_ = 0U;
    last_temperature_start_tick_ = platform::Time::now_ticks();
}

void Bmi088::set_completion_notification(platform::Spi::CompletionNotification callback, void *context) noexcept
{
    accelerometer_.set_completion_notification(callback, context);
    gyroscope_.set_completion_notification(callback, context);
    temperature_.set_completion_notification(callback, context);
}

template <typename Sensor>
void Bmi088::publish(Sensor &sensor,
                     std::uint32_t previous_reads,
                     Bmi088SampleRecord &inflight,
                     Bmi088DataReady latest,
                     Bmi088SampleQueue<> &queue) noexcept
{
    if (sensor.get_read_count() == previous_reads)
        return;
    typename Sensor::Sample sample{};
    if (!sensor.try_get_sample(sample))
        return;
    inflight.sequence = sensor.get_read_count();
    inflight.completed_tick = sensor.get_completion_tick();
    inflight.harvested_tick = platform::Time::now_ticks();
    inflight.xyz[0] = sample.x_axis;
    inflight.xyz[1] = sample.y_axis;
    inflight.xyz[2] = sample.z_axis;
    // 传输完成前下一次任务迭代又观测到更晚的边沿时，归属变得
    // 有歧义。这是保守告警，并非丢失的证明。
    if (latest.sequence != inflight.drdy_sequence &&
        static_cast<std::uint32_t>(latest.tick - inflight.drdy_tick) <=
            static_cast<std::uint32_t>(inflight.completed_tick - inflight.drdy_tick))
    {
        inflight.flags |= Bmi088SampleRecord::NEWER_EVENT_BEFORE_COMPLETION;
    }
    (void)queue.push(inflight);
}

bool Bmi088::start_accelerometer() noexcept
{
    if (!accelerometer_data_ready_pending_ || temperature_.has_pending_transfer())
        return false;
    const auto start = platform::Time::now_ticks();
    if (!accelerometer_.try_start())
        return false;
    accelerometer_inflight_ = {};
    accelerometer_inflight_.drdy_sequence = accelerometer_event_.sequence;
    accelerometer_inflight_.drdy_tick = accelerometer_event_.tick;
    accelerometer_inflight_.start_tick = start;
    if (static_cast<std::uint32_t>(start - accelerometer_event_.tick) >= platform::Time::tick_frequency_hz() / 1600U)
    {
        accelerometer_inflight_.flags |= Bmi088SampleRecord::LATE_START;
    }
    accelerometer_data_ready_pending_ = false;
    return true;
}

bool Bmi088::start_gyroscope() noexcept
{
    if (!gyroscope_data_ready_pending_)
        return false;
    const auto start = platform::Time::now_ticks();
    if (!gyroscope_.try_start())
        return false;
    gyroscope_inflight_ = {};
    gyroscope_inflight_.drdy_sequence = gyroscope_event_.sequence;
    gyroscope_inflight_.drdy_tick = gyroscope_event_.tick;
    gyroscope_inflight_.start_tick = start;
    constexpr auto ODR = Bmi088Gyro::BANDWIDTH_REGISTER_VALUE == Bmi088Gyro::GYRO_BANDWIDTH_2000_HZ_532_HZ ? 2000U : 1000U;
    if (static_cast<std::uint32_t>(start - gyroscope_event_.tick) >= platform::Time::tick_frequency_hz() / ODR)
    {
        gyroscope_inflight_.flags |= Bmi088SampleRecord::LATE_START;
    }
    gyroscope_data_ready_pending_ = false;
    return true;
}

void Bmi088::process() noexcept
{
    const auto accel_reads = accelerometer_.get_read_count();
    const auto gyro_reads = gyroscope_.get_read_count();
    accelerometer_.harvest();
    gyroscope_.harvest();
    temperature_.harvest();
    publish(accelerometer_, accel_reads, accelerometer_inflight_, accelerometer_event_, accelerometer_queue_);
    publish(gyroscope_, gyro_reads, gyroscope_inflight_, gyroscope_event_, gyroscope_queue_);

    if (!temperature_.is_enabled() && accelerometer_.is_ready())
        temperature_.enable();

    if (!is_ready())
    {
        // 一侧传感器正在配置时，就绪的另一侧不得执行无触发读取。
        if ((turn_ % 2U) == 0U)
        {
            if (!accelerometer_.is_ready())
                (void)accelerometer_.try_start();
        }
        else
        {
            if (!gyroscope_.is_ready())
                (void)gyroscope_.try_start();
        }
        ++turn_;
        return;
    }

    if (platform::Time::has_elapsed_ms(last_temperature_start_tick_, TEMPERATURE_PERIOD_MS) && temperature_.is_enabled() &&
        !accelerometer_.has_pending_transfer() && !temperature_.has_pending_transfer() && temperature_.try_start())
    {
        last_temperature_start_tick_ = platform::Time::now_ticks();
    }

    // 按边沿年龄调度：先服务等待更久的挂起边沿，再试另一侧。固定的两侧交替
    // 会让 2000 Hz 的陀螺长期占先，使周期更长（625 us）的加速度计边沿偶发被
    // 合并；改为最老优先后，两路都能在自己的周期内被服务。
    // 首选一侧无法启动时改试另一侧；Busy 或先前传输仍在进行时绝不消费挂起的边沿。
    const std::uint32_t now = platform::Time::now_ticks();
    bool prefer_accelerometer = accelerometer_data_ready_pending_;
    if (accelerometer_data_ready_pending_ && gyroscope_data_ready_pending_)
    {
        prefer_accelerometer = static_cast<std::uint32_t>(now - accelerometer_event_.tick) >=
                              static_cast<std::uint32_t>(now - gyroscope_event_.tick);
    }
    if (prefer_accelerometer)
    {
        if (!start_accelerometer())
            (void)start_gyroscope();
    }
    else
    {
        if (!start_gyroscope())
            (void)start_accelerometer();
    }
    ++turn_;
}

void Bmi088::notify_data_ready(Bmi088DataReady accelerometer, Bmi088DataReady gyroscope) noexcept
{
    const auto accel_events = accelerometer.sequence - accelerometer_event_.sequence;
    const auto gyro_events = gyroscope.sequence - gyroscope_event_.sequence;
    accelerometer_data_ready_count_ += accel_events;
    gyroscope_data_ready_count_ += gyro_events;
    if (accel_events > 0U)
    {
        accelerometer_coalesced_event_count_ += accel_events - (accelerometer_data_ready_pending_ ? 0U : 1U);
        accelerometer_data_ready_pending_ = true;
        accelerometer_event_ = accelerometer;
    }
    if (gyro_events > 0U)
    {
        gyroscope_coalesced_event_count_ += gyro_events - (gyroscope_data_ready_pending_ ? 0U : 1U);
        gyroscope_data_ready_pending_ = true;
        gyroscope_event_ = gyroscope;
    }
}

Bmi088::State Bmi088::get_state() const noexcept
{
    if (accelerometer_.get_state() == Bmi088Accel::State::ERROR || gyroscope_.get_state() == Bmi088Gyro::State::ERROR)
        return State::ERROR;
    return is_ready() ? State::READY : State::INITIALIZING;
}
} // namespace device
