#include "Libraries/Device/motor/dji/DjiMotor.hpp"

#include <array>
#include <cassert>
#include <cstdint>

// ---- 平台 C ABI 桩：被测代码经 platform::Can / platform::Time 调用 ----
namespace {

struct CanStub {
  bool ready{true};
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

CanPort_SendResult CanPort_TrySend(CanPort_Channel, std::uint32_t identifier,
                                   CanPort_IdentifierType, std::uint8_t length,
                                   const std::uint8_t *data) {
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

using Bus = device::DjiMotor;

int main() {
  // ---- C620 会话：保活发送（C610/C620 反馈的前提）----
  Bus c620Bus{platform::Can::Channel::Channel1, Bus::Profile::C620()};
  assert(c620Bus.Init());
  assert(c620Bus.IsReady());
  assert(c620Bus.MaximumDeviceCount() == 8U);
  assert(c620Bus.EnableMotor(1U));
  assert(!c620Bus.EnableMotor(0U));
  assert(!c620Bus.EnableMotor(9U));

  ResetRx();
  c620Bus.Process();
  assert(canStub.txCount == 1U);
  assert(canStub.lastTxIdentifier == 0x200U);
  assert(canStub.lastTxData == (std::array<std::uint8_t, 8U>{}));

  // ---- 反馈解析与单圈换算（0x201 = 设备 1）----
  timeStub.ticks = 100U;
  ResetRx();
  FeedRx(0x201U, {0x10U, 0x00U, 0x00U, 0x64U, 0x00U, 0x32U, 30U, 0U});
  c620Bus.Process();
  Bus::Snapshot snapshot{};
  assert(c620Bus.GetSnapshot(1U, snapshot));
  assert(snapshot.deviceId == 1U);
  assert(snapshot.enabled && snapshot.everReceived && snapshot.online);
  assert(snapshot.lastUpdateTick == 100U);
  assert(snapshot.acceptedFrameCount == 1U);
  assert(snapshot.rotorAngleRaw == 0x1000U);
  assert(snapshot.rotorAngleDegrees == 180.0F);
  assert(snapshot.rotorSpeedRpm == 100);
  assert(snapshot.torqueCurrentRaw == 50);
  assert(snapshot.motorTemperatureCelsius == 30U);
  assert(snapshot.totalAngleCounts == 0); // 首帧只建立多圈基准

  // 未启用/未见过反馈的设备：快照可读但离线
  assert(c620Bus.GetSnapshot(3U, snapshot));
  assert(!snapshot.enabled && !snapshot.online && !snapshot.everReceived);
  assert(!c620Bus.GetSnapshot(0U, snapshot));
  assert(!c620Bus.GetSnapshot(9U, snapshot));

  // ---- 多圈累计：+4095 后跨零前进 +16，净 +4111 ----
  timeStub.ticks = 101U;
  ResetRx();
  FeedRx(0x201U, {0x1FU, 0xFFU, 0, 0, 0, 0, 0, 0});
  c620Bus.Process();
  assert(c620Bus.GetSnapshot(1U, snapshot));
  assert(snapshot.totalAngleCounts == 4095);

  timeStub.ticks = 102U;
  ResetRx();
  FeedRx(0x201U, {0x00U, 0x0FU, 0, 0, 0, 0, 0, 0});
  c620Bus.Process();
  assert(c620Bus.GetSnapshot(1U, snapshot));
  assert(snapshot.totalAngleCounts == 4111);
  assert(snapshot.totalTurns == 0);

  // ---- 指令换算：安培 / 比例 / 原始值，全部钳位 ----
  assert(c620Bus.SetCurrentAmpere(1U, 5.0F)); // 5A/20A*16384 = 4096
  c620Bus.Process();
  assert(canStub.lastTxIdentifier == 0x200U);
  assert(canStub.lastTxData[0] == 0x10U && canStub.lastTxData[1] == 0x00U);

  assert(c620Bus.SetCurrentAmpere(1U, 999.0F)); // 钳位到 +16384
  c620Bus.Process();
  assert(canStub.lastTxData[0] == 0x40U && canStub.lastTxData[1] == 0x00U);

  assert(c620Bus.SetTorqueRatio(1U, -1.0F)); // -16384
  c620Bus.Process();
  assert(canStub.lastTxData[0] == 0xC0U && canStub.lastTxData[1] == 0x00U);

  assert(!c620Bus.SetCurrentAmpere(0U, 1.0F));
  assert(!c620Bus.SetCurrentAmpere(9U, 1.0F));

  assert(c620Bus.SetTorque(1U, 1.5F)); // 1.5 N·m / 0.3 = 5A → 4096
  c620Bus.Process();
  assert(canStub.lastTxIdentifier == 0x200U);
  assert(canStub.lastTxData[0] == 0x10U && canStub.lastTxData[1] == 0x00U);
  assert(!c620Bus.SetTorque(9U, 1.0F));

  // ---- 第二组聚合：设备 6 走 0x1FF 槽位 1 ----
  Bus secondBus{platform::Can::Channel::Channel2, Bus::Profile::C620()};
  assert(secondBus.Init());
  assert(secondBus.EnableMotor(6U));
  assert(secondBus.SetRawCommand(6U, 1000));
  ResetRx();
  secondBus.Process();
  assert(canStub.lastTxIdentifier == 0x1FFU);
  assert(canStub.lastTxData[2] == 0x03U && canStub.lastTxData[3] == 0xE8U);
  Bus::Statistics secondStatistics{};
  assert(secondBus.GetStatistics(secondStatistics));
  assert(secondStatistics.txResultCounts[0] == 1U);

  // ---- GM6020 电压模式：无电流接口、比例走电压、ID 上限 7、范围外 ID 未知 ----
  Bus gmBus{platform::Can::Channel::Channel3, Bus::Profile::Gm6020Voltage()};
  assert(gmBus.Init());
  assert(gmBus.MaximumDeviceCount() == 7U);
  assert(gmBus.EnableMotor(1U));
  assert(!gmBus.EnableMotor(8U));
  assert(!gmBus.SetCurrentAmpere(1U, 1.0F)); // 电压模式无电流满量程
  assert(!gmBus.SetTorque(1U, 1.0F));         // 电压模式力矩接口同样不可用
  assert(gmBus.SetTorqueRatio(1U, 0.5F));    // 0.5*25000 = 12500 = 0x30D4
  ResetRx();
  gmBus.Process(); // 仅设备 1 在役：单帧 0x1FF
  assert(canStub.lastTxIdentifier == 0x1FFU);
  assert(canStub.lastTxData[0] == 0x30U && canStub.lastTxData[1] == 0xD4U);

  assert(gmBus.EnableMotor(7U)); // 第二组开始随节拍发送 0x2FF
  timeStub.ticks = 103U;
  ResetRx();
  FeedRx(0x205U, {0x08U, 0x00U, 0x00U, 0x64U, 0xFFU, 0xF4U, 50U, 0U});
  FeedRx(0x201U, {0, 0, 0, 0, 0, 0, 0, 0}); // C 家族 ID 对 GM 方言为未知
  gmBus.Process(); // 双组各一帧：0x1FF 之后 0x2FF，末帧为设备 7 所在组
  assert(canStub.lastTxIdentifier == 0x2FFU);
  assert(canStub.lastTxData == (std::array<std::uint8_t, 8U>{}));
  Bus::Statistics gmStatistics{};
  assert(gmBus.GetStatistics(gmStatistics));
  assert(gmStatistics.acceptedFrameCount == 1U);
  assert(gmStatistics.unknownIdentifierCount == 1U);
  assert(gmStatistics.txResultCounts[0] == 3U); // 1 帧 + 双组 2 帧
  assert(gmBus.GetSnapshot(1U, snapshot));
  assert(snapshot.online);
  assert(snapshot.rotorAngleRaw == 0x0800U);
  assert(snapshot.rotorAngleDegrees == 90.0F);
  assert(snapshot.rotorSpeedRpm == 100);
  assert(snapshot.torqueCurrentRaw == -12);
  assert(snapshot.motorTemperatureCelsius == 50U);

  // ---- C610：温度槽手册标注 Null，快照恒 0 ----
  Bus c610Bus{platform::Can::Channel::Channel1, Bus::Profile::C610()};
  assert(c610Bus.Init());
  assert(c610Bus.EnableMotor(1U));
  timeStub.ticks = 110U;
  ResetRx();
  FeedRx(0x201U, {0x00U, 0x01U, 0, 0, 0, 0, 40U, 0U});
  c610Bus.Process();
  assert(c610Bus.GetSnapshot(1U, snapshot));
  assert(snapshot.online);
  assert(snapshot.motorTemperatureCelsius == 0U);

  assert(c610Bus.SetTorque(1U, 0.5F)); // 0.5 N·m / 0.18 ≈ 2.778A → 2777
  c610Bus.Process();
  assert(canStub.lastTxIdentifier == 0x200U);
  assert(canStub.lastTxData[0] == 0x0AU && canStub.lastTxData[1] == 0xD9U);

  // ---- 链路老化：静默超过 10ms 后离线 ----
  timeStub.ticks = 200U;
  ResetRx();
  c620Bus.Process();
  assert(c620Bus.GetSnapshot(1U, snapshot));
  assert(!snapshot.online);
  assert(snapshot.everReceived);
  assert(snapshot.acceptedFrameCount == 3U);

  // ---- 总线统计：发送结果分布与解码计数 ----
  Bus::Statistics c620Statistics{};
  assert(c620Bus.GetStatistics(c620Statistics));
  assert(c620Statistics.txResultCounts[0] == 9U); // 含一次力矩指令发送
  assert(c620Statistics.acceptedFrameCount == 3U);
  assert(c620Statistics.unknownIdentifierCount == 0U);
  assert(c620Statistics.rxDroppedCount == 0U);

  // ---- 清零指令 ----
  c620Bus.ClearAllCommands();
  ResetRx();
  c620Bus.Process();
  assert(canStub.lastTxData == (std::array<std::uint8_t, 8U>{}));
  return 0;
}
