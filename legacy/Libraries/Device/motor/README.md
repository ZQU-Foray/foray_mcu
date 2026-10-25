# 任务内选型的电机组

每个任务在自己的 cpp 内持有 MotorGroup<MotorModel, N>，不再需要
Application/Configuration 或全局静态 Motor 绑定。底盘的型号、通道、设备编号、
方向和 PID 参数均位于 Application/Task/1_chassis/ChassisTask.cpp。

    using ChassisMotors = device::MotorGroup<device::MotorModel::M2006, 4>;
    ChassisMotors motors{platform::Can::Channel::Channel1,
                         {{{1U, 1}, {2U, 1}, {3U, 1}, {4U, 1}}}};

后续代码统一使用 motors.Init / Process / ReadState / SetTorque / ClearCommands，
控制器继续只接收 MotorState 并输出原生输出轴力矩，不包含型号分支。

## 文件组织

通用代码只保留两个头文件：

- Motor.hpp：MotorState、MotorModel、MotorConnection，控制器只包含此文件。
- MotorGroup.hpp：任务使用的编译期电机组接口。

电机组直接持有编译期选定的品牌适配器：MotorGroup.hpp 只声明型号→适配器的
映射定制点（MotorAdapterFor），不含任何品牌依赖。DJI 型号映射位于
dji/DjiMotorModelMap.hpp（档案与原生减速比等品牌默认参数在此装配），任务通过
包含该映射头完成品牌选择；品牌专属内容全部在品牌目录，不保留后端转发层、
Interface/Detail 目录或旧路径转发头。

## 型号与自动适配

MotorModel 是任务允许知道的装配选择；品牌映射头（如 dji/DjiMotorModelMap.hpp）
在编译期把型号特化为适配器类型与默认参数，MotorGroup 直接持有适配器实例，
无虚函数、函数指针派发或运行时型号 switch。

| 型号          | 适配器          | 自动减速比 | 档案输出轴 Kt（N·m/A） | 最大数量 |
| ------------- | --------------- | ---------: | ---------------------: | -------: |
| M2006         | C610            |         36 |                   0.18 |        8 |
| M3508         | C620            |       19.2 |                    0.3 |        8 |
| GM6020Current | GM6020 电流模式 |          1 |                  0.741 |        7 |

这些值沿用工程已有档案和约定。GM6020 必须实际配置为电流模式，软件选型不会更改
电调固件或工作模式。未支持型号和数量越界在编译期拒绝；非法编号或方向在 Init 拒绝。

- 更换已有型号：修改任务顶部的品牌映射包含与 MotorModel，检查设备 ID 与接线。
- 新增品牌：在品牌目录提供兼容适配器（Config 构造 + Init/Process/ReadState/
  SetTorque/ClearCommands 契约 + MaximumMotors）和一个 MotorAdapterFor 特化
  映射头；任务包含该映射头即可使用，电机组与控制器零改动
  （见 Tests/Motor/BrandExtensionTest.cpp 的最小示例）。
- 控制参数：修改同一任务内 ControllerConfig()，参数为输出轴量纲；型号不会自动
  推导负载、PID 增益、业务限幅或安全策略。该只读函数也供测试验证实际任务参数。
- 新增品牌：增加通用型号及对应后端特化，保持电机组操作契约；业务循环和控制器不变。
- 接口输出以型号原生输出轴为准。外接传动、安装零位等不由型号推断，需要另行适配。

## 实例与总线所有权

不同任务可在不同 CAN 通道创建不同型号的电机组，各组拥有独立状态和命令槽位。
一组独占一个通道。所有组须在调度器启动前 Init；同一通道被第二个电机组占用时
返回 false，包括同型号情况，避免抢走反馈或重复发送共享控制帧。

成功 Init 的对象须保持至固件结束，建议如底盘一样定义在任务 cpp 的匿名命名空间。
禁止复制或移动；首版不提供运行期释放通道或热插拔对象。未成功初始化不占通道。
不能同时绕过电机组直接在同一 CAN 上创建原始驱动消费者。多任务共享 CAN 仍需
单独的接收分发与发送聚合服务，本版不提供该能力。

## 契约

| 入口          | 行为                                                                       |
| ------------- | -------------------------------------------------------------------------- |
| Init          | 调度前调用；成功后幂等，失败可重试；校验通道、映射、方向、减速比与力矩能力 |
| Count         | 编译期常量，返回模板指定数量                                               |
| Process       | 接收、老化、聚合发送、发布快照；不等待 I/O，处理时间随待收帧数变化         |
| ReadState     | 逻辑编号从 0 开始；失败清空状态，无反馈时读取成功但有效性与在线均为 false  |
| SetTorque     | 暂存输出轴 N·m；越界编号失败，非有限输入将有效槽位置零并返回 false         |
| ClearCommands | 清零暂存命令，下一次 Process 发出；不停止保活，不代表硬件失能或制动        |

初始化后，Process、读状态及写命令归同一底盘任务所有。外部任务继续使用
ChassisTask 的目标／模式设定入口；此重构未改变原有跨任务发布机制。

状态角度为首次反馈以来的输出轴累计 deg，转速为输出轴 rpm。方向配置同时作用于
角度、速度、力矩。lastUpdateTick 是 platform::Time 的原始 Tick，只有收到过反馈
后才有效，按无符号时间差和平台频率换算；控制器不使用此字段。
feedbackValid 表示曾收到有效反馈，online 表示未超过现有 50ms 老化阈值；离线时
可保留最后数值，但不能用作在线闭环输入。

DjiMotor::Profile 的 Kt 已是输出轴 N·m/A；适配器仅改变力矩方向，不重复乘减速比。
通用型号列表不提供电压模式；底层适配器仍会在初始化时拒绝电压档案。控制器遇到离线、无效反馈、非法数值或 PID
计算失败会输出零并复位对应 PID；任务遇到命令写入失败清零整组并复位控制器目标。

## 控制参数迁移

当前配置由原转子 rpm→安培控制等效换算而来：速度环增益乘 36×0.18，输出与积分
上限乘 0.18；角度环增益和转速上限除以 36。得到速度 Kp=0.1296、Ki=0.648、
输出上限 0.54 N·m、积分上限 0.27 N·m，角度 Kp=4/36，转速上限 500 rpm。
力矩直通仍单独限制在 ±1 N·m。新型号需要重新确定其动力学参数与增益，不能照搬。

继续执行“发送已有命令→读取状态→闭环→暂存新命令”，不改变约 1ms 调度和一步
发送延迟。四台在役设备仍合成一帧 0x200；设置命令不触发单独发送。

## 角度边界

相邻编码器差值按 8192 计数一圈归一化：大于 4096 减 8192，小于 -4096 加 8192。
恰好半圈保留差值符号，但方向不可判定；准确累计仍要求相邻反馈机械角差小于半圈。
断线期间圈数不推断。原驱动的 int32 累计范围和 float 长时间位置精度限制仍保留，
本次未加入重新标零或绝对位置恢复功能。

## 验证

在 WSL 项目根目录执行：

    cmake -S Tests/Motor -B /tmp/controller-motor-tests -G Ninja -DCMAKE_BUILD_TYPE=Debug
    cmake --build /tmp/controller-motor-tests -j 4
    ctest --test-dir /tmp/controller-motor-tests --output-on-failure

独立入口启用断言与严格告警，配置阶段检查业务／公共接口不存在 DJI 依赖。
替换构建目录并设置 Release 可验证优化构建。实际结果和资源比较见
[验证记录](../../../Tests/Motor/VALIDATION.md)。
