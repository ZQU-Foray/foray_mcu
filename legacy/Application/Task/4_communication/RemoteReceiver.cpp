#include "Application/Task/4_communication/RemoteReceiver.hpp"
#include "Time.hpp"
#include "Uart.hpp"
#include <array>
#include <atomic>

namespace application
{
namespace
{

using RemoteEndpoint = platform::Uart::Endpoint;
using ReceiveResult = platform::Uart::ReceiveResult;

static_assert(std::atomic<std::uint32_t>::is_always_lock_free,
              "Remote receiver snapshot requires lock-free 32-bit atomics");

protocol::SbusParser parser{};
protocol::SbusParser::Frame latestFrame{};
bool hasAcceptedFrame{false};
bool linkTimedOut{true};
std::uint32_t lastAcceptedFrameTick{0U};

std::atomic<std::uint32_t> snapshotSequence{0U};
std::array<std::atomic<std::uint32_t>, RemoteReceiver::ChannelCount> snapshotChannels{};
std::atomic<std::uint32_t> snapshotFlags{0U};
std::atomic<std::uint32_t> snapshotConnected{0U};
std::atomic<std::uint32_t> snapshotAcceptedFrameCount{0U};
std::atomic<std::uint32_t> snapshotRejectedFrameCount{0U};
std::atomic<std::uint32_t> snapshotDiscardedByteCount{0U};
std::atomic<std::uint32_t> snapshotUartDroppedByteCount{0U};
std::atomic<std::uint32_t> snapshotUartErrorEventCount{0U};
std::atomic<std::uint32_t> remoteReady{0U};

constexpr std::uint32_t DigitalChannel17Flag{1U << 0U};
constexpr std::uint32_t DigitalChannel18Flag{1U << 1U};
constexpr std::uint32_t FrameLostFlag{1U << 2U};
constexpr std::uint32_t FailsafeFlag{1U << 3U};

void PublishSnapshot(const platform::Uart::Statistics &uartStatistics) noexcept
{
  snapshotSequence.fetch_add(1U, std::memory_order_acq_rel);

  for (std::size_t channel = 0U; channel < RemoteReceiver::ChannelCount; ++channel)
  {
    snapshotChannels[channel].store(latestFrame.channels[channel], std::memory_order_relaxed);
  }

  std::uint32_t flags = 0U;
  flags |= latestFrame.digitalChannel17 ? DigitalChannel17Flag : 0U;
  flags |= latestFrame.digitalChannel18 ? DigitalChannel18Flag : 0U;
  flags |= latestFrame.frameLost ? FrameLostFlag : 0U;
  flags |= latestFrame.failsafe ? FailsafeFlag : 0U;
  snapshotFlags.store(flags, std::memory_order_relaxed);
  snapshotConnected.store(hasAcceptedFrame && !linkTimedOut && !latestFrame.failsafe ? 1U : 0U,
                          std::memory_order_relaxed);

  const protocol::SbusParser::Statistics &parserStatistics = parser.GetStatistics();
  snapshotAcceptedFrameCount.store(parserStatistics.acceptedFrameCount, std::memory_order_relaxed);
  snapshotRejectedFrameCount.store(parserStatistics.rejectedFrameCount, std::memory_order_relaxed);
  snapshotDiscardedByteCount.store(parserStatistics.discardedByteCount, std::memory_order_relaxed);
  snapshotUartDroppedByteCount.store(uartStatistics.droppedByteCount, std::memory_order_relaxed);
  snapshotUartErrorEventCount.store(uartStatistics.errorEventCount, std::memory_order_relaxed);

  snapshotSequence.fetch_add(1U, std::memory_order_release);
}

} // namespace

bool RemoteReceiver::Init() noexcept
{
  remoteReady.store(0U, std::memory_order_relaxed);
  parser.Reset();
  latestFrame = {};
  hasAcceptedFrame = false;
  linkTimedOut = true;
  lastAcceptedFrameTick = platform::Time::NowTicks();

  platform::Uart::Statistics uartStatistics{};
  PublishSnapshot(uartStatistics);
  if (!platform::Time::IsReady() || !platform::Uart::IsReady(RemoteEndpoint::RemoteReceiver))
  {
    return false;
  }

  remoteReady.store(1U, std::memory_order_release);
  return true;
}

void RemoteReceiver::Process() noexcept
{
  if (!IsReady())
  {
    return;
  }

  std::array<std::uint8_t, 64U> receivedBytes{};
  bool receivedAnyBytes = false;
  for (;;)
  {
    std::size_t receivedLength = 0U;
    const ReceiveResult result = platform::Uart::TryRead(RemoteEndpoint::RemoteReceiver, receivedBytes, receivedLength);
    if (result == ReceiveResult::Empty)
    {
      break;
    }

    if (result != ReceiveResult::Received)
    {
      if (result == ReceiveResult::NotReady || result == ReceiveResult::Error)
      {
        remoteReady.store(0U, std::memory_order_release);
      }
      return;
    }

    receivedAnyBytes = true;
    if (parser.Input(receivedBytes.data(), receivedLength, latestFrame) > 0U)
    {
      hasAcceptedFrame = true;
      linkTimedOut = false;
      lastAcceptedFrameTick = platform::Time::NowTicks();
    }
  }

  bool linkStateChanged = false;
  if (hasAcceptedFrame && !linkTimedOut && platform::Time::HasElapsedMs(lastAcceptedFrameTick, LinkTimeoutMs))
  {
    linkTimedOut = true;
    linkStateChanged = true;
  }

  if (!receivedAnyBytes && !linkStateChanged)
  {
    return;
  }

  // 统计为 MAY 能力：Port 未实现时保持全 0 快照，不影响链路状态
  platform::Uart::Statistics uartStatistics{};
  (void)platform::Uart::GetStatistics(RemoteEndpoint::RemoteReceiver, uartStatistics);
  PublishSnapshot(uartStatistics);
}

bool RemoteReceiver::IsReady() noexcept
{
  return remoteReady.load(std::memory_order_acquire) != 0U && platform::Time::IsReady() &&
         platform::Uart::IsReady(RemoteEndpoint::RemoteReceiver);
}

bool RemoteReceiver::GetSnapshot(Snapshot &snapshot) noexcept
{
  if (!IsReady())
  {
    snapshot = {};
    return false;
  }

  std::uint32_t sequenceBefore = 0U;
  std::uint32_t sequenceAfter = 0U;
  std::uint32_t flags = 0U;
  std::uint32_t connected = 0U;
  do
  {
    sequenceBefore = snapshotSequence.load(std::memory_order_acquire);
    if ((sequenceBefore & 1U) != 0U)
    {
      continue;
    }

    for (std::size_t channel = 0U; channel < ChannelCount; ++channel)
    {
      snapshot.channels[channel] =
          static_cast<std::uint16_t>(snapshotChannels[channel].load(std::memory_order_relaxed));
    }
    flags = snapshotFlags.load(std::memory_order_relaxed);
    connected = snapshotConnected.load(std::memory_order_relaxed);
    snapshot.acceptedFrameCount = snapshotAcceptedFrameCount.load(std::memory_order_relaxed);
    snapshot.rejectedFrameCount = snapshotRejectedFrameCount.load(std::memory_order_relaxed);
    snapshot.discardedByteCount = snapshotDiscardedByteCount.load(std::memory_order_relaxed);
    snapshot.uartDroppedByteCount = snapshotUartDroppedByteCount.load(std::memory_order_relaxed);
    snapshot.uartErrorEventCount = snapshotUartErrorEventCount.load(std::memory_order_relaxed);
    sequenceAfter = snapshotSequence.load(std::memory_order_acquire);
  } while (sequenceBefore != sequenceAfter || (sequenceAfter & 1U) != 0U);

  snapshot.digitalChannel17 = (flags & DigitalChannel17Flag) != 0U;
  snapshot.digitalChannel18 = (flags & DigitalChannel18Flag) != 0U;
  snapshot.frameLost = (flags & FrameLostFlag) != 0U;
  snapshot.failsafe = (flags & FailsafeFlag) != 0U;
  snapshot.connected = connected != 0U;
  return true;
}

float RemoteReceiver::NormalizeChannel(const Snapshot &snapshot, std::size_t channel) noexcept
{
  if (channel >= ChannelCount)
  {
    return 0.0F;
  }

  std::uint16_t value = snapshot.channels[channel];
  if (value < ChannelMinimum)
  {
    value = ChannelMinimum;
  }
  else if (value > ChannelMaximum)
  {
    value = ChannelMaximum;
  }

  return static_cast<float>(static_cast<std::int32_t>(value) - static_cast<std::int32_t>(ChannelMiddle)) /
         static_cast<float>(ChannelMaximum - ChannelMiddle);
}

} // namespace application