#include "Libraries/Device/motor/dji/DjiMotorModelMap.hpp"
#include "Libraries/Device/motor/MotorGroup.hpp"
#include "Tests/Motor/Support/DjiPlatformStub.hpp"
#include <cmath>
#include <type_traits>

using device::MotorGroup;
using device::MotorModel;
using Channel = platform::Can::Channel;
using Small = MotorGroup<MotorModel::M2006,4>;
using Large = MotorGroup<MotorModel::M3508,1>;
using Gimbal = MotorGroup<MotorModel::GM6020Current,1>;
static_assert(Small::Count()==4);
static_assert(!std::is_copy_constructible_v<Small> && !std::is_move_constructible_v<Small>);

int main() {
  // 无效映射不能占住通道；成功 Init 的对象一直存活到程序结束。
  Small invalid{Channel::Channel1, {{{1,1},{1,1},{3,1},{4,1}}}};
  assert(!invalid.Init());
  Small chassis{Channel::Channel1, {{{1,1},{2,1},{3,1},{4,1}}}};
  canStub.ready=false;
  assert(!chassis.Init());
  canStub.ready=true;
  assert(chassis.Init() && chassis.Init());
  Large other{Channel::Channel2, {{{1,-1}}}};
  Gimbal gimbal{Channel::Channel3, {{{1,1}}}};
  assert(other.Init() && gimbal.Init());
  // 不同型号也共享通道占用检查，失败的实例不能接收或发送。
  Large conflict{Channel::Channel1, {{{1,1}}}};
  assert(!conflict.Init());
  const auto sent=canStub.txCount;
  conflict.Process();
  assert(canStub.txCount==sent);
  assert(!conflict.SetTorque(0,0.1F));

  assert(chassis.SetTorque(0,0.18F));
  assert(other.SetTorque(0,0.3F));
  assert(gimbal.SetTorque(0,0.741F));
  assert(canStub.txCount==sent);
  chassis.Process();
  assert(canStub.lastTxChannel==CAN_PORT_CHANNEL_1);
  assert(canStub.lastTxIdentifier==0x200);
  assert(canStub.lastTxData[0]==3 && canStub.lastTxData[1]==0xE8); // 1A ->1000
  assert(canStub.txCount==sent+1); // four slots still one frame
  other.Process();
  assert(canStub.lastTxChannel==CAN_PORT_CHANNEL_2);
  const auto raw=static_cast<std::int16_t>((canStub.lastTxData[0]<<8)|canStub.lastTxData[1]);
  assert(raw==-819); // C620 -1A; direction applies to torque
  gimbal.Process();
  assert(canStub.lastTxChannel==CAN_PORT_CHANNEL_3);
  assert(canStub.lastTxIdentifier==0x1FE);
  assert(canStub.lastTxData[0]==0x15 && canStub.lastTxData[1]==0x55); // 1A ->5461

  device::MotorState state{};
  ResetRx(); FeedRx(0x201,{0,0,0x0E,0x10,0,0,0,0}); // 3600 rpm
  chassis.Process();
  assert(chassis.ReadState(0,state) && state.speedRpm==100);
  ResetRx(); FeedRx(0x201,{0,0,0x07,0x80,0,0,0,0}); //1920 rpm
  other.Process();
  assert(other.ReadState(0,state) && std::fabs(state.speedRpm+100)<1e-5F);
  ResetRx(); FeedRx(0x205,{0,0,0,100,0,0,0,0});
  gimbal.Process();
  assert(gimbal.ReadState(0,state) && state.speedRpm==100);
  assert(chassis.ReadState(0,state) && state.speedRpm==100); // independent snapshots
  ResetRx(); FeedRx(0x201,{0x08,0,0,0,0,0,0,0}); //+90 rotor degrees
  other.Process();
  assert(other.ReadState(0,state) && std::fabs(state.angleDegrees+90.0F/19.2F)<1e-5F);
  chassis.ClearCommands(); chassis.Process();
  assert(canStub.lastTxData==(std::array<std::uint8_t,8>{}));
  other.Process(); // clearing one group does not affect another
  assert(canStub.lastTxData[0]==0xFC && canStub.lastTxData[1]==0xCD);
}
