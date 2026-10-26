#pragma once
#include "Libraries/Device/motor/dji/DjiMotor.hpp"

#include <array>
#include <cassert>
#include <cstdint>

// ---- 平台 C ABI 桩：被测代码经 platform::Can / platform::Time 调用 ----
namespace {

struct CanStub {
  bool ready{true};
  CanPort_Channel lastTxChannel{CAN_PORT_CHANNEL_1};
  std::uint8_t rxCount{0U};
  std::uint8_t rxIndex{0U};
  struct RxFrame {
    std::uint32_t identifier{0U};
    std::array<std::uint8_t, 8U> data{};
  };
  std::array<RxFrame, 8U> rx{};
  std::uint32_t txCount{0U};
  std::uint32_t lastTxIdentifier{0U};
  std::array<std::uint8_t, 8U> lastTxData{};
  CanPort_SendResult txResult{CAN_PORT_SEND_QUEUED};
  CanPort_Statistics statistics{};
} canStub;

struct TimeStub {
  std::uint32_t ticks{0U};
} timeStub;

void FeedRx(std::uint32_t identifier, const std::array<std::uint8_t, 8U> &data) {
  canStub.rx[canStub.rxCount] = CanStub::RxFrame{identifier, data};
  ++canStub.rxCount;
}

void ResetRx() {
  canStub.rxCount = 0U;
  canStub.rxIndex = 0U;
}

} // namespace

extern "C" {

bool CanPort_Init(void) { return true; }

bool CanPort_IsReady(CanPort_Channel) { return canStub.ready; }

bool CanPort_ConfigureStandardReceiveFilter(CanPort_Channel, bool,
                                            std::uint32_t, std::uint32_t) {
  return false;
}

bool CanPort_SetReceiveNotification(CanPort_Channel, CanPort_ReceiveNotification,
                                    void *) {
  return false;
}

CanPort_SendResult CanPort_TrySend(CanPort_Channel channel, std::uint32_t identifier,
                                   CanPort_IdentifierType, std::uint8_t length,
                                   const std::uint8_t *data) {
  canStub.lastTxChannel = channel;
  ++canStub.txCount;
  canStub.lastTxIdentifier = identifier;
  for (std::uint8_t index = 0U; index < 8U; ++index) {
    canStub.lastTxData[index] = index < length ? data[index] : 0U;
  }
  return canStub.txResult;
}

CanPort_ReceiveResult CanPort_TryReceive(CanPort_Channel,
                                         std::uint32_t *identifier,
                                         CanPort_IdentifierType *identifierType,
                                         std::uint8_t *length,
                                         std::uint8_t *data,
                                         std::uint8_t dataCapacity) {
  if (canStub.rxIndex >= canStub.rxCount) {
    return CAN_PORT_RECEIVE_EMPTY;
  }
  const CanStub::RxFrame &frame = canStub.rx[canStub.rxIndex];
  ++canStub.rxIndex;
  *identifier = frame.identifier;
  *identifierType = CAN_PORT_IDENTIFIER_STANDARD;
  *length = 8U;
  for (std::uint8_t index = 0U; index < 8U && index < dataCapacity; ++index) {
    data[index] = frame.data[index];
  }
  return CAN_PORT_RECEIVE_RECEIVED;
}

bool CanPort_GetStatistics(CanPort_Channel, CanPort_Statistics *statistics) {
  *statistics = canStub.statistics;
  return true;
}

bool TimePort_Init(void) { return true; }
bool TimePort_IsReady(void) { return true; }
std::uint32_t TimePort_NowTicks(void) { return timeStub.ticks; }
std::uint32_t TimePort_FrequencyHz(void) { return 1000U; }
void TimePort_DelayUs(std::uint32_t) {}

} // extern "C"

