// BMI088 驱动的采集流程：初始化、进行中的数据就绪、共享 SPI、合并、有界队列、提前完成、时基回绕、DMA 错误。
// 旧 app 的数据就绪邮箱（app/tasks）与诊断采集（app/imu）两段随旧 app 归档删除（Decision-20261031-13）：
// 邮箱由 utils 的 LatestValue 与 topic 取代，诊断采集随 IMU 能力在阶段 A 重建。
#include "driver/bmi088/bmi088.hpp"
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>

#include "mock_hal.hpp"
#include "mock_os.hpp"

// SPI 经 platform_mock 的 hal::Spi 替身：加速度计（设备 0）与陀螺仪（设备 1）放在同一条总线 0 上，
// 由 respond() 扮演 BMI088 的寄存器；时间经 hal_os 替身，单位 µs。
namespace
{
constexpr std::uint8_t ACCEL{0U};
constexpr std::uint8_t GYRO{1U};
constexpr std::uint8_t BUS{0U};

std::uint32_t g_callbacks{};
std::uint8_t g_registers[2][128]{};

// 设备应答：读（地址最高位为 1）从寄存器表取数据，加速度计多一个哑字节；写只取第一个数据字节。
// 每个事务耗时 10 µs。
void respond(void *, std::uint8_t device, const std::uint8_t *transmit, std::uint8_t *receive, std::size_t length)
{
    const auto address = transmit[0] & 0x7f;
    if ((transmit[0] & 0x80) != 0)
    {
        const auto skip = device == ACCEL ? 2U : 1U;
        for (auto i = skip; i < length; ++i)
            receive[i] = g_registers[device][address + i - skip];
    }
    else
        g_registers[device][address] = transmit[1];
    mock::advance_us(10U);
}

std::uint32_t now()
{
    return hal::os::now_us();
}

bool busy()
{
    return mock::spi::busy(BUS);
}

std::uint8_t active_device()
{
    std::uint8_t device = 0xFFU;
    assert(mock::spi::busy(BUS, &device));
    return device;
}

void complete(bool error = false)
{
    assert(busy());
    if (error)
        mock::advance_us(10U); // 出错的事务同样耗时，应答函数不会被调用
    assert(mock::spi::complete(BUS, error));
}

void wake(void *) noexcept
{
    ++g_callbacks;
}

void initialize(device::Bmi088 &imu)
{
    mock::set_now_us(0U);
    mock::spi::reset();
    mock::spi::set_bus(GYRO, BUS);
    mock::spi::set_responder(ACCEL, respond, nullptr);
    mock::spi::set_responder(GYRO, respond, nullptr);
    g_callbacks = 0U;
    std::memset(g_registers, 0, sizeof(g_registers));
    g_registers[0][0] = device::Bmi088Accel::EXPECTED_CHIP_ID;
    g_registers[1][0] = device::Bmi088Gyro::EXPECTED_CHIP_ID;
    g_registers[0][0x22] = 27; // 50 Celsius
    g_registers[1][2] = 42;
    imu.init();
    imu.set_completion_notification(wake, nullptr);
    for (unsigned i = 0; i < 500 && !imu.is_ready(); ++i)
    {
        mock::advance_us(1000U);
        imu.process();
        if (busy())
            complete();
    }
    assert(imu.is_ready());
    // Finish any due temperature read. No untriggered accel/gyro reads allowed.
    for (int i = 0; i < 3; ++i)
    {
        if (busy())
            complete();
        imu.process();
    }
    assert(imu.get_gyroscope().get_read_count() == 0);
    assert(imu.get_accelerometer().get_read_count() == 0);
}
} // namespace

int main()
{
    mock::reset();
    device::Bmi088 imu{hal::Spi{ACCEL}, hal::Spi{GYRO}};
    device::Bmi088SampleRecord record{};
    initialize(imu);
    const auto before_callbacks = g_callbacks;
    imu.notify_data_ready({}, {1U, now()});
    imu.process();
    assert(busy() && active_device() == GYRO);
    const auto first_tick = now();
    mock::advance_us(500U);
    imu.notify_data_ready({}, {2U, now()});
    imu.process(); // earlier transfer still active
    assert(!imu.pop_gyroscope_sample(record));
    complete();
    imu.process();
    assert(busy()); // second edge MUST remain pending until a new transfer starts
    assert(imu.pop_gyroscope_sample(record));
    assert(record.drdy_sequence == 1 && record.drdy_us == first_tick);
    assert(record.xyz[0] == 42);
    assert((record.flags & device::Bmi088SampleRecord::NEWER_EVENT_BEFORE_COMPLETION) != 0);
    complete();
    imu.process();
    assert(imu.pop_gyroscope_sample(record) && record.drdy_sequence == 2);
    assert(g_callbacks == before_callbacks + 2);
    assert(imu.get_gyroscope_coalesced_event_count() == 0);

    // Both pending on a shared bus: no false consumption of the blocked peer.
    imu.notify_data_ready({1U, now()}, {3U, now()});
    imu.process();
    complete();
    imu.process();
    assert(busy());
    complete();
    imu.process();
    assert(imu.pop_accelerometer_sample(record) && record.drdy_sequence == 1);
    assert(imu.pop_gyroscope_sample(record) && record.drdy_sequence == 3);
    imu.notify_data_ready({1U, now()}, {6U, now()});
    imu.process();
    complete();
    imu.process();
    assert(imu.pop_gyroscope_sample(record) && record.drdy_sequence == 6);
    assert(imu.get_gyroscope_coalesced_event_count() == 2);

    // A newer completed gyro must wait for a known older acceleration that has
    // not yet been harvested, then cross-sensor output follows DRDY time.
    initialize(imu);
    bool is_gyro{};
    const auto epoch = now();
    imu.notify_data_ready({}, {1U, epoch + 20U});
    mock::advance_us(20U);
    imu.process();
    complete();
    imu.notify_data_ready({1U, epoch + 10U}, {1U, epoch + 20U});
    imu.process();
    assert(!imu.pop_next_sample(record, is_gyro));
    complete();
    imu.process();
    assert(imu.pop_next_sample(record, is_gyro) && !is_gyro && record.drdy_us == epoch + 10U);
    assert(imu.pop_next_sample(record, is_gyro) && is_gyro && record.drdy_us == epoch + 20U);
    assert(!imu.pop_next_sample(record, is_gyro));

    // Equal epochs deliver acceleration first even if gyro completed first.
    initialize(imu);
    imu.notify_data_ready({}, {1U, now()});
    imu.process();
    complete();
    imu.notify_data_ready({1U, now() - 10U}, {1U, now() - 10U});
    imu.process();
    assert(!imu.pop_next_sample(record, is_gyro));
    complete();
    imu.process();
    assert(imu.pop_next_sample(record, is_gyro) && !is_gyro);
    assert(imu.pop_next_sample(record, is_gyro) && is_gyro);

    // Queue loss is separate from DRDY coalescing; FIFO order survives wrap.
    initialize(imu);
    for (std::uint32_t i = 1; i <= 40; ++i)
    {
        mock::advance_us(500U);
        imu.notify_data_ready({}, {i, now()});
        imu.process();
        complete();
        imu.process();
    }
    assert(imu.get_gyroscope_queue().overflow_count() == 8);
    assert(imu.get_gyroscope_queue().high_water() == 32);
    for (std::uint32_t i = 1; i <= 32; ++i)
    {
        assert(imu.pop_gyroscope_sample(record) && record.drdy_sequence == i);
    }
    assert(!imu.pop_gyroscope_sample(record));

    device::Bmi088SampleQueue<2> small_queue;
    for (std::uint32_t i = 0; i < 10; ++i)
    {
        record.sequence = i;
        assert(small_queue.push(record));
        assert(small_queue.pop(record) && record.sequence == i);
    }
    assert(small_queue.overflow_count() == 0 && small_queue.high_water() == 1);

    // Cumulative interrupt counts also wrap without interpreting an old edge as new.
    initialize(imu);
    imu.notify_data_ready({}, {0xffffffffU, now()});
    imu.process();
    complete();
    imu.process();
    assert(imu.pop_gyroscope_sample(record) && record.drdy_sequence == 0xffffffffU);
    imu.notify_data_ready({}, {0U, now()});
    imu.process();
    complete();
    imu.process();
    assert(imu.pop_gyroscope_sample(record) && record.drdy_sequence == 0U);
    assert(imu.get_gyroscope_data_ready_count() == 0U);

    // Completion can interrupt start_transfer_async before it returns to the driver.
    initialize(imu);
    mock::spi::set_complete_immediately(true);
    mock::set_now_us(0xfffffff8U);
    imu.notify_data_ready({}, {1U, now()});
    imu.process();
    imu.process();
    assert(imu.pop_gyroscope_sample(record));
    assert(static_cast<std::uint32_t>(record.completed_us - record.start_us) == 10U);
    assert(record.completed_us < record.drdy_us); // actual 32-bit wrap
    assert(record.sequence == 1);

    initialize(imu);
    const auto late_tick = now();
    mock::advance_us(600U);
    imu.notify_data_ready({}, {1U, late_tick});
    imu.process();
    complete();
    imu.process();
    assert(imu.pop_gyroscope_sample(record));
    assert((record.flags & device::Bmi088SampleRecord::LATE_START) != 0);

    // DMA error wakes the owner, raises an error, and never publishes bogus data.
    initialize(imu);
    imu.notify_data_ready({}, {1U, now()});
    imu.process();
    complete(true);
    imu.process();
    assert(imu.get_gyroscope().get_error_count() == 1);
    assert(!imu.pop_gyroscope_sample(record));
    assert(imu.get_state() == device::Bmi088::State::ERROR);

    std::cout << "PASS: initialization, in-flight DRDY, shared SPI, "
                 "coalescing, bounded queue, early completion, time wrap, DMA error\n";
}
