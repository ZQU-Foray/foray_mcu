# 电机：关节、CanBus 与品牌驱动

2026-11-02 起的结构（Decision-20261102-03，方案见 [docs/ai/decisions/2026-11-02-电机架构方案](../../../docs/ai/decisions/2026-11-02-电机架构方案/README.md)）。
原来的电机组 `MotorGroup<型号, N>` 已删除。

```text
能力（云台、底盘、发射……）       只读写关节 Joint：SI 单位，不知道品牌
        ↕ 同一个任务里直接读写
品牌驱动 DjiMotor、DmMotor        每条总线上每个品牌一个：状态机、换算、发送节拍
        ↕ 用协议层编解码（protocol::DjiEsc、DmEsc）
CanBus（src/driver/can_bus）      每条总线一个：唯一的接收方，按帧 ID 分给登记的驱动
        ↕
hal::Can
```

## 文件

| 文件 | 内容 |
|---|---|
| `joint.hpp` | `Joint`（能力用）、`JointState`、`JointCommand`、`JointMechanics`、`JointLink`（只给品牌驱动用）、`enable_all()` |
| `dji/dji_motor.hpp` | `DjiMotor`：一条总线上的 C610、C620、GM6020（电流模式），可以混接 |
| `dji/dji_models.hpp` | 型号常量 `device::dji::M2006`、`M3508`、`GM6020_CURRENT` |
| `dm/dm_motor.hpp` | `DmMotor`：一条总线上的达妙电机（MIT），启动回读核对 |
| `dm/dm_models.hpp` | 型号常量 `device::dm::DM_J4310` |

## 关节

- **单位**：rad、rad/s、N·m。
- **机构**：`JointMechanics` 写方向、外接传动比、零点、指令超时（默认 3 ms），与型号无关，由 profile 给。
  - 关节位置 q = 方向 × (电机位置 − 零点) / 传动比；
  - 关节力矩 = 方向 × 电机力矩 × 传动比。
- **出力**：
  - 先 `enable()`，没有请求出力时 `set_torque()` 返回 false；
  - 电机故障、离线、使能失败时驱动撤销请求，能力要再 `enable()`；
  - 安全门 `actuator::SafetyGate` 用 `set_inhibited()` 统一禁止；
  - `enable_all()`：一组关节要么都使能、要么一个都不使能。
- **指令**：`set_torque()` 全部型号都支持；`set_impedance()`（电机内阻抗，τ = Kp(p* − p) + Kd(v* − v) + τff）只有达妙支持，用 `supports()` 查。
  指令超过 `command_timeout_us` 没刷新，驱动按 0 力矩发。
- **位置**：`state().position_absolute` 为真（达妙、GM6020）时上电即绝对位置，否则（M3508、M2006）从第一帧反馈起算。

## 两个品牌的差别

| | DJI（`DjiMotor`） | 达妙（`DmMotor`） |
|---|---|---|
| 一条总线最多 | 8 台（C610/C620 1–8 号、GM6020 1–7 号，可混接） | 4 台（ESC_ID 1–15，MST_ID 可以共用） |
| 使能 | 电调没有使能命令，`enable()` 之后即出力 | 先回读核对（模式、量程、写通讯超时），通过后清错 → 使能 → 按反馈确认 |
| 指令 | 力矩（换成电流） | 力矩，或阻抗（MIT） |
| 反馈 | 电调固定 1 kHz 自发 | 一问一答：每发一帧回一帧 |
| 故障信息 | 没有 | 状态码（`fault_code`），故障时撤销出力 |
| 每秒帧数（负载检查） | 每台 1000 + 每个控制帧 × 控制频率 | 每台 2 × 控制频率 |

## 装配

一条总线上的同一品牌电机放进同一个驱动（DJI 的控制帧一帧带四台，两个实例会互相覆盖，第二个实例登记时被拒绝），
关节可以分给不同的能力。

```cpp
// 组合根（示意）：CAN1 一台 DM-J4310，CAN2 四台 M3508 加一台 M2006
device::CanBus g_can_1{board::CAN_1};
device::Joint g_yaw;
constexpr device::DmMotor::Config yaw_config()
{
    device::DmMotor::Config config{};
    config.count = 1U;
    config.motors[0] = {&g_yaw, device::dm::DM_J4310, params::YAW.esc_id, params::YAW.master_id, params::YAW.mechanics};
    return config;
}
device::DmMotor g_can_1_dm{g_can_1, yaw_config()};
static_assert(device::can_load_percent(device::DmMotor::frames_per_second(1U, 1000U)) <= 80U, "CAN1 超载");

// 任务表：收 → 控制 → 发，在同一个任务里
void gimbal_receive(hal::os::Microseconds) { g_can_1.receive(); }
void gimbal_transmit(hal::os::Microseconds now) { g_can_1.transmit(now); }
// {"gimbal", 1024U, 27U, common::on_topic(topics::imu::attitude, 2U), {gimbal_receive, gimbal_step, gimbal_transmit}}

// boot_init()：g_can_1.init() && g_can_1_dm.init() && …
```

规则：
- **一条总线只归一个任务**（D3）。总线的收、发与用这些关节的控制计算在同一个任务里，关节不做跨任务同步。
- 要给别的任务看的状态，由持有关节的能力发布成 topic。

## 加型号、加品牌

- **加型号**：在品牌目录的 `*_models.hpp` 里加一条 `constexpr` 常量（DJI：电调档案、原生减速比、位置是否绝对；达妙：量程、峰值扭矩、通讯超时）。
- **加品牌**：写一个驱动：
  1. 构造时接收 `CanBus&`；
  2. `init()` 里用 `CanBus::attach()` 登记要收的帧 ID 与收帧、每周期两个回调，再用 `JointLink::bind()` 接上关节；
  3. 收到反馈时用 `JointLink::publish()`、`set_status()` 写状态；
  4. 每周期用 `JointLink::command()` 取指令、编码、发送。
  能力与组合根的写法不变。

## 验证

- 宿主测试：`joint`、`can_bus`、`dji_motor`、`angle_wrap`、`dm_motor`、`bringup_dm`、`safety_gate`（`test/host/README.md`）。
- 上板：达妙电机（[2026-11-02 验证记录](../../../docs/ai/decisions/2026-11-02-达妙电机驱动上板验证/README.md)）、
  1 kHz 控制环基准（[重构前后对照](../../../docs/ai/decisions/2026-11-02-电机控制环基准/README.md)）。DJI 的新驱动还没有上板。
