#include "Libraries/Device/bmi088/Bmi088.hpp"
#include "Application/Task/0_imu/ImuDataReadyMailbox.hpp"
// Include the recorder in this test translation unit to inspect its private
// frozen debugger ABI without adding a production-facing test interface.
#define IMU_ENABLE_DIAGNOSTICS
#include "Application/Imu/ImuDiagnostics.cpp"
#include <cassert>
#include <cstring>
#include <iostream>
#include <thread>

namespace
{
std::uint32_t g_tick{}, g_callbacks{};
std::uint8_t g_registers[2][128]{};
SpiPort_Result g_results[2]{};
bool g_busy{}, g_complete_immediately{};
SpiPort_Device g_active_device{};
std::uint8_t g_tx[8]{}, *g_rx{};
std::uint32_t g_transfer_length{};
SpiPort_CompletionNotification g_notification{};
void *g_notification_context{};

void complete(bool error = false)
{
    assert(g_busy);
    const auto address = g_tx[0] & 0x7f;
    if (!error)
    {
        std::memset(g_rx, 0, g_transfer_length);
        if ((g_tx[0] & 0x80) != 0)
        {
            const auto skip = g_active_device == SPI_PORT_DEVICE_IMU_ACCELEROMETER ? 2U : 1U;
            for (auto i = skip; i < g_transfer_length; ++i)
                g_rx[i] = g_registers[g_active_device][address + i - skip];
        }
        else
            g_registers[g_active_device][address] = g_tx[1];
    }
    g_tick += 10U;
    g_results[g_active_device] = error ? SPI_PORT_RESULT_ERROR : SPI_PORT_RESULT_COMPLETED;
    g_busy = false;
    const auto callback = g_notification;
    const auto context = g_notification_context;
    if (callback)
        callback(context);
}
void wake(void *)
{
    ++g_callbacks;
}
void initialize(device::Bmi088 &imu)
{
    g_tick = g_callbacks = 0U;
    g_busy = g_complete_immediately = false;
    std::memset(g_registers, 0, sizeof(g_registers));
    g_registers[0][0] = device::Bmi088Accel::EXPECTED_CHIP_ID;
    g_registers[1][0] = device::Bmi088Gyro::EXPECTED_CHIP_ID;
    g_registers[0][0x22] = 27; // 50 Celsius
    g_registers[1][2] = 42;
    imu.init();
    imu.set_completion_notification(wake, nullptr);
    for (unsigned i = 0; i < 500 && !imu.is_ready(); ++i)
    {
        g_tick += 1000U;
        imu.process();
        if (g_busy)
            complete();
    }
    assert(imu.is_ready());
    // Finish any due temperature read. No untriggered accel/gyro reads allowed.
    for (int i = 0; i < 3; ++i)
    {
        if (g_busy)
            complete();
        imu.process();
    }
    assert(imu.get_gyroscope().get_read_count() == 0);
    assert(imu.get_accelerometer().get_read_count() == 0);
}
} // namespace

extern "C" std::uint32_t TimePort_NowTicks()
{
    return g_tick;
}
extern "C" std::uint32_t TimePort_FrequencyHz()
{
    return 1000000U;
}
extern "C" SpiPort_Result SpiPort_GetAsyncResult(SpiPort_Device d)
{
    return g_results[d];
}
extern "C" SpiPort_Result SpiPort_StartTransferAsync(SpiPort_Device d,
                                                     const std::uint8_t *transmit,
                                                     std::uint8_t *receive,
                                                     std::uint32_t length,
                                                     SpiPort_CompletionNotification callback,
                                                     void *context)
{
    if (g_busy)
        return SPI_PORT_RESULT_BUSY;
    assert(d < 2 && length <= sizeof(g_tx));
    g_busy = true;
    g_active_device = d;
    g_rx = receive;
    g_transfer_length = length;
    std::memcpy(g_tx, transmit, length);
    g_notification = callback;
    g_notification_context = context;
    g_results[d] = SPI_PORT_RESULT_BUSY;
    if (g_complete_immediately)
        complete();
    return SPI_PORT_RESULT_STARTED;
}

int main()
{
    // Latest-edge snapshot must never pair a tick with the wrong sequence.
    application::task::ImuDataReadyMailbox mailbox;
    std::thread writer(
        [&]
        {
            for (std::uint32_t i = 1; i <= 100000U; ++i)
                mailbox.publish(i * 7U);
        });
    for (int i = 0; i < 100000; ++i)
    {
        const auto event = mailbox.snapshot();
        assert(event.tick == event.sequence * 7U);
    }
    writer.join();
    assert(mailbox.snapshot().sequence == 100000U);

    device::Bmi088 imu;
    device::Bmi088SampleRecord record{};
    initialize(imu);
    const auto before_callbacks = g_callbacks;
    imu.notify_data_ready({}, {1U, g_tick});
    imu.process();
    assert(g_busy && g_active_device == 1);
    const auto first_tick = g_tick;
    g_tick += 500U;
    imu.notify_data_ready({}, {2U, g_tick});
    imu.process(); // earlier transfer still active
    assert(!imu.pop_gyroscope_sample(record));
    complete();
    imu.process();
    assert(g_busy); // second edge MUST remain pending until a new transfer starts
    assert(imu.pop_gyroscope_sample(record));
    assert(record.drdy_sequence == 1 && record.drdy_tick == first_tick);
    assert(record.xyz[0] == 42);
    assert((record.flags & device::Bmi088SampleRecord::NEWER_EVENT_BEFORE_COMPLETION) != 0);
    complete();
    imu.process();
    assert(imu.pop_gyroscope_sample(record) && record.drdy_sequence == 2);
    assert(g_callbacks == before_callbacks + 2);
    assert(imu.get_gyroscope_coalesced_event_count() == 0);

    // Both pending on a shared bus: no false consumption of the blocked peer.
    imu.notify_data_ready({1U, g_tick}, {3U, g_tick});
    imu.process();
    complete();
    imu.process();
    assert(g_busy);
    complete();
    imu.process();
    assert(imu.pop_accelerometer_sample(record) && record.drdy_sequence == 1);
    assert(imu.pop_gyroscope_sample(record) && record.drdy_sequence == 3);
    imu.notify_data_ready({1U, g_tick}, {6U, g_tick});
    imu.process();
    complete();
    imu.process();
    assert(imu.pop_gyroscope_sample(record) && record.drdy_sequence == 6);
    assert(imu.get_gyroscope_coalesced_event_count() == 2);

    // A newer completed gyro must wait for a known older acceleration that has
    // not yet been harvested, then cross-sensor output follows DRDY time.
    initialize(imu);
    bool is_gyro{};
    const auto epoch = g_tick;
    imu.notify_data_ready({}, {1U, epoch + 20U});
    g_tick += 20U;
    imu.process();
    complete();
    imu.notify_data_ready({1U, epoch + 10U}, {1U, epoch + 20U});
    imu.process();
    assert(!imu.pop_next_sample(record, is_gyro));
    complete();
    imu.process();
    assert(imu.pop_next_sample(record, is_gyro) && !is_gyro && record.drdy_tick == epoch + 10U);
    assert(imu.pop_next_sample(record, is_gyro) && is_gyro && record.drdy_tick == epoch + 20U);
    assert(!imu.pop_next_sample(record, is_gyro));

    // Equal epochs deliver acceleration first even if gyro completed first.
    initialize(imu);
    imu.notify_data_ready({}, {1U, g_tick});
    imu.process();
    complete();
    imu.notify_data_ready({1U, g_tick - 10U}, {1U, g_tick - 10U});
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
        g_tick += 500U;
        imu.notify_data_ready({}, {i, g_tick});
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
    imu.notify_data_ready({}, {0xffffffffU, g_tick});
    imu.process();
    complete();
    imu.process();
    assert(imu.pop_gyroscope_sample(record) && record.drdy_sequence == 0xffffffffU);
    imu.notify_data_ready({}, {0U, g_tick});
    imu.process();
    complete();
    imu.process();
    assert(imu.pop_gyroscope_sample(record) && record.drdy_sequence == 0U);
    assert(imu.get_gyroscope_data_ready_count() == 0U);

    // Completion can interrupt start_transfer_async before it returns to the driver.
    initialize(imu);
    g_complete_immediately = true;
    g_tick = 0xfffffff8U;
    imu.notify_data_ready({}, {1U, g_tick});
    imu.process();
    imu.process();
    assert(imu.pop_gyroscope_sample(record));
    assert(static_cast<std::uint32_t>(record.completed_tick - record.start_tick) == 10U);
    assert(record.completed_tick < record.drdy_tick); // actual 32-bit wrap
    assert(record.sequence == 1);

    initialize(imu);
    const auto late_tick = g_tick;
    g_tick += 600U;
    imu.notify_data_ready({}, {1U, late_tick});
    imu.process();
    complete();
    imu.process();
    assert(imu.pop_gyroscope_sample(record));
    assert((record.flags & device::Bmi088SampleRecord::LATE_START) != 0);

    // DMA error wakes the owner, raises an error, and never publishes bogus data.
    initialize(imu);
    imu.notify_data_ready({}, {1U, g_tick});
    imu.process();
    complete(true);
    imu.process();
    assert(imu.get_gyroscope().get_error_count() == 1);
    assert(!imu.pop_gyroscope_sample(record));
    assert(imu.get_state() == device::Bmi088::State::ERROR);

    initialize(imu);
    application::g_gyro_capture_request = 2U;
    record = {};
    for (std::uint32_t i = 1; i <= 20000; ++i)
    {
        record.sequence = record.drdy_sequence = i;
        record.drdy_tick = i * 500U;
        record.start_tick = record.drdy_tick + 5U;
        record.completed_tick = record.drdy_tick + 15U;
        record.harvested_tick = record.drdy_tick + 20U;
        application::ImuDiagnostics::observe(imu, record, true);
    }
    assert(application::g_gyro_capture.state == 2U);
    assert(application::g_gyro_capture.channels[1].count == 20000U);
    assert(application::g_gyro_capture.channels[1].max_start_ticks == 5U);
    assert(application::g_gyro_capture.channels[1].max_complete_ticks == 15U);
    assert(application::g_gyro_capture.channels[1].elapsed_ticks == 19999ULL * 500U);
    application::g_gyro_capture_request = 1U;
    application::ImuDiagnostics::observe(imu, record, true);
    assert(application::g_gyro_capture.state == 0U && application::g_gyro_capture.count == 0U);
    application::g_gyro_capture_request = 2U;
    application::ImuDiagnostics::observe(imu, record, true);
    assert(application::g_gyro_capture.state == 1U && application::g_gyro_capture.count == 1U);
    assert(application::g_gyro_capture.channels[1].read_gaps == 0U);
    std::cout << "PASS: initialization, coherent ISR mailbox, in-flight DRDY, shared SPI, "
                 "coalescing, bounded queue, early completion, tick wrap, DMA error, capture/rearm\n";
}
