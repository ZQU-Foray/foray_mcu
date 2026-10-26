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

namespace {
std::uint32_t tick{}, callbacks{};
std::uint8_t registers[2][128]{};
SpiPort_Result results[2]{};
bool busy{}, completeImmediately{};
SpiPort_Device activeDevice{};
std::uint8_t tx[8]{}, *rx{};
std::uint32_t transferLength{};
SpiPort_CompletionNotification notification{};
void *notificationContext{};

void Complete(bool error = false) {
  assert(busy);
  const auto address = tx[0] & 0x7f;
  if (!error) {
    std::memset(rx, 0, transferLength);
    if ((tx[0] & 0x80) != 0) {
      const auto skip = activeDevice == SPI_PORT_DEVICE_IMU_ACCELEROMETER ? 2U : 1U;
      for (auto i = skip; i < transferLength; ++i) rx[i] = registers[activeDevice][address+i-skip];
    } else registers[activeDevice][address] = tx[1];
  }
  tick += 10U;
  results[activeDevice] = error ? SPI_PORT_RESULT_ERROR : SPI_PORT_RESULT_COMPLETED;
  busy = false;
  const auto callback = notification;
  const auto context = notificationContext;
  if (callback) callback(context);
}
void Wake(void *) { ++callbacks; }
void Initialize(device::Bmi088 &imu) {
  tick = callbacks = 0U;
  busy = completeImmediately = false;
  std::memset(registers, 0, sizeof(registers));
  registers[0][0] = device::Bmi088Accel::ExpectedChipId;
  registers[1][0] = device::Bmi088Gyro::ExpectedChipId;
  registers[0][0x22] = 27; // 50 Celsius
  registers[1][2] = 42;
  imu.Init();
  imu.SetCompletionNotification(Wake, nullptr);
  for (unsigned i = 0; i < 500 && !imu.IsReady(); ++i) {
    tick += 1000U;
    imu.Process();
    if (busy) Complete();
  }
  assert(imu.IsReady());
  // Finish any due temperature read. No untriggered accel/gyro reads allowed.
  for (int i = 0; i < 3; ++i) { if (busy) Complete(); imu.Process(); }
  assert(imu.GetGyroscope().GetReadCount() == 0);
  assert(imu.GetAccelerometer().GetReadCount() == 0);
}
}

extern "C" std::uint32_t TimePort_NowTicks() { return tick; }
extern "C" std::uint32_t TimePort_FrequencyHz() { return 1000000U; }
extern "C" SpiPort_Result SpiPort_GetAsyncResult(SpiPort_Device d) { return results[d]; }
extern "C" SpiPort_Result SpiPort_StartTransferAsync(SpiPort_Device d,
    const std::uint8_t *transmit, std::uint8_t *receive, std::uint32_t length,
    SpiPort_CompletionNotification callback, void *context) {
  if (busy) return SPI_PORT_RESULT_BUSY;
  assert(d < 2 && length <= sizeof(tx));
  busy = true; activeDevice = d; rx = receive; transferLength = length;
  std::memcpy(tx, transmit, length);
  notification = callback; notificationContext = context;
  results[d] = SPI_PORT_RESULT_BUSY;
  if (completeImmediately) Complete();
  return SPI_PORT_RESULT_STARTED;
}

int main() {
  // Latest-edge snapshot must never pair a tick with the wrong sequence.
  application::task::ImuDataReadyMailbox mailbox;
  std::thread writer([&] { for (std::uint32_t i = 1; i <= 100000U; ++i) mailbox.Publish(i * 7U); });
  for (int i = 0; i < 100000; ++i) {
    const auto event = mailbox.Snapshot();
    assert(event.tick == event.sequence * 7U);
  }
  writer.join();
  assert(mailbox.Snapshot().sequence == 100000U);

  device::Bmi088 imu;
  device::Bmi088SampleRecord record{};
  Initialize(imu);
  const auto beforeCallbacks = callbacks;
  imu.NotifyDataReady({}, {1U, tick}); imu.Process();
  assert(busy && activeDevice == 1);
  const auto firstTick = tick;
  tick += 500U;
  imu.NotifyDataReady({}, {2U, tick}); imu.Process(); // earlier transfer still active
  assert(!imu.PopGyroscopeSample(record));
  Complete(); imu.Process();
  assert(busy); // second edge MUST remain pending until a new transfer starts
  assert(imu.PopGyroscopeSample(record));
  assert(record.drdySequence == 1 && record.drdyTick == firstTick);
  assert(record.xyz[0] == 42);
  assert((record.flags & device::Bmi088SampleRecord::NewerEventBeforeCompletion) != 0);
  Complete(); imu.Process();
  assert(imu.PopGyroscopeSample(record) && record.drdySequence == 2);
  assert(callbacks == beforeCallbacks + 2);
  assert(imu.GetGyroscopeCoalescedEventCount() == 0);

  // Both pending on a shared bus: no false consumption of the blocked peer.
  imu.NotifyDataReady({1U, tick}, {3U, tick}); imu.Process();
  Complete(); imu.Process(); assert(busy); Complete(); imu.Process();
  assert(imu.PopAccelerometerSample(record) && record.drdySequence == 1);
  assert(imu.PopGyroscopeSample(record) && record.drdySequence == 3);
  imu.NotifyDataReady({1U, tick}, {6U, tick}); imu.Process();
  Complete(); imu.Process();
  assert(imu.PopGyroscopeSample(record) && record.drdySequence == 6);
  assert(imu.GetGyroscopeCoalescedEventCount() == 2);

  // A newer completed gyro must wait for a known older acceleration that has
  // not yet been harvested, then cross-sensor output follows DRDY time.
  Initialize(imu);
  bool isGyro{};
  const auto epoch = tick;
  imu.NotifyDataReady({}, {1U, epoch + 20U}); tick += 20U; imu.Process();
  Complete();
  imu.NotifyDataReady({1U, epoch + 10U}, {1U, epoch + 20U});
  imu.Process();
  assert(!imu.PopNextSample(record, isGyro));
  Complete(); imu.Process();
  assert(imu.PopNextSample(record, isGyro) && !isGyro && record.drdyTick == epoch + 10U);
  assert(imu.PopNextSample(record, isGyro) && isGyro && record.drdyTick == epoch + 20U);
  assert(!imu.PopNextSample(record, isGyro));

  // Equal epochs deliver acceleration first even if gyro completed first.
  Initialize(imu);
  imu.NotifyDataReady({}, {1U, tick}); imu.Process(); Complete();
  imu.NotifyDataReady({1U, tick - 10U}, {1U, tick - 10U}); imu.Process();
  assert(!imu.PopNextSample(record, isGyro));
  Complete(); imu.Process();
  assert(imu.PopNextSample(record, isGyro) && !isGyro);
  assert(imu.PopNextSample(record, isGyro) && isGyro);

  // Queue loss is separate from DRDY coalescing; FIFO order survives wrap.
  Initialize(imu);
  for (std::uint32_t i = 1; i <= 40; ++i) {
    tick += 500U; imu.NotifyDataReady({}, {i, tick}); imu.Process();
    Complete(); imu.Process();
  }
  assert(imu.GetGyroscopeQueue().OverflowCount() == 8);
  assert(imu.GetGyroscopeQueue().HighWater() == 32);
  for (std::uint32_t i = 1; i <= 32; ++i) {
    assert(imu.PopGyroscopeSample(record) && record.drdySequence == i);
  }
  assert(!imu.PopGyroscopeSample(record));

  device::Bmi088SampleQueue<2> smallQueue;
  for (std::uint32_t i = 0; i < 10; ++i) {
    record.sequence = i;
    assert(smallQueue.Push(record));
    assert(smallQueue.Pop(record) && record.sequence == i);
  }
  assert(smallQueue.OverflowCount() == 0 && smallQueue.HighWater() == 1);

  // Cumulative interrupt counts also wrap without interpreting an old edge as new.
  Initialize(imu);
  imu.NotifyDataReady({}, {0xffffffffU, tick}); imu.Process(); Complete(); imu.Process();
  assert(imu.PopGyroscopeSample(record) && record.drdySequence == 0xffffffffU);
  imu.NotifyDataReady({}, {0U, tick}); imu.Process(); Complete(); imu.Process();
  assert(imu.PopGyroscopeSample(record) && record.drdySequence == 0U);
  assert(imu.GetGyroscopeDataReadyCount() == 0U);

  // Completion can interrupt StartTransferAsync before it returns to the driver.
  Initialize(imu); completeImmediately = true;
  tick = 0xfffffff8U;
  imu.NotifyDataReady({}, {1U, tick}); imu.Process(); imu.Process();
  assert(imu.PopGyroscopeSample(record));
  assert(static_cast<std::uint32_t>(record.completedTick - record.startTick) == 10U);
  assert(record.completedTick < record.drdyTick); // actual 32-bit wrap
  assert(record.sequence == 1);

  Initialize(imu);
  const auto lateTick = tick;
  tick += 600U;
  imu.NotifyDataReady({}, {1U, lateTick}); imu.Process(); Complete(); imu.Process();
  assert(imu.PopGyroscopeSample(record));
  assert((record.flags & device::Bmi088SampleRecord::LateStart) != 0);

  // DMA error wakes the owner, raises an error, and never publishes bogus data.
  Initialize(imu);
  imu.NotifyDataReady({}, {1U, tick}); imu.Process(); Complete(true); imu.Process();
  assert(imu.GetGyroscope().GetErrorCount() == 1);
  assert(!imu.PopGyroscopeSample(record));
  assert(imu.GetState() == device::Bmi088::State::Error);

  Initialize(imu);
  application::gyroCaptureRequest = 2U;
  record = {};
  for (std::uint32_t i = 1; i <= 20000; ++i) {
    record.sequence = record.drdySequence = i;
    record.drdyTick = i * 500U;
    record.startTick = record.drdyTick + 5U;
    record.completedTick = record.drdyTick + 15U;
    record.harvestedTick = record.drdyTick + 20U;
    application::ImuDiagnostics::Observe(imu, record, true);
  }
  assert(application::gyroCapture.state == 2U);
  assert(application::gyroCapture.channels[1].count == 20000U);
  assert(application::gyroCapture.channels[1].maxStartTicks == 5U);
  assert(application::gyroCapture.channels[1].maxCompleteTicks == 15U);
  assert(application::gyroCapture.channels[1].elapsedTicks == 19999ULL * 500U);
  application::gyroCaptureRequest = 1U;
  application::ImuDiagnostics::Observe(imu, record, true);
  assert(application::gyroCapture.state == 0U && application::gyroCapture.count == 0U);
  application::gyroCaptureRequest = 2U;
  application::ImuDiagnostics::Observe(imu, record, true);
  assert(application::gyroCapture.state == 1U && application::gyroCapture.count == 1U);
  assert(application::gyroCapture.channels[1].readGaps == 0U);
  std::cout << "PASS: initialization, coherent ISR mailbox, in-flight DRDY, shared SPI, "
               "coalescing, bounded queue, early completion, tick wrap, DMA error, capture/rearm\n";
}
