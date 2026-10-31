# F2 第 4 步启动入口的上板验证（2026-10-31）

验证 Decision-20261031-11 的改动：生成区只调 `boot_*`，`hal::init()` 启动 TIM2，旧 app 照常运行；同时检查 Decision-20261031-10 中已经在固件里运行的部分（TIM2 时基与调试冻结、任务通知槽改为 2 个之后的内核）。

## 固件

- 分支 `f2/boot`，代码与提交 `9ca0711` 相同；profile `baseline`，arm-debug（`-O0 -g3`）
- `firmware_baseline.bin` 的 sha256：`e198afb3519500e0bd2197962cedd5eb44e5473c337163abbc94487426293a5d`
- 调试器：Vllink Basic2（CMSIS-DAP，序列号 8D86347A76），OpenOCD 0.12.0，SWD 1800 kHz
- 烧录前的板上固件已只读备份到仓库外的 `foray_mcu_archive/2026-10-31_DM-MC02_flash_before_f2/`，与 2026-10-26 的存档逐字节相同（原固件，含丢失的链路协议与遥控任务）
- 烧录：`program <elf> verify reset exit`，`Verified OK`（`flash.log`）。只擦除了 0x08000000–0x0803FFFF；0x08040000–0x08045AE4 仍留着原固件的尾部，不会被执行

## 方法

不停核，用 OpenOCD 的 `read_memory` 隔 1 s 读两次（`probe_runtime.tcl`、`probe_imu.tcl`），地址取自同一个 ELF 的符号表。读数见 `probe.log`。

## 结果

| 检查 | 读数 | 结论 |
|---|---|---|
| TIM2 时基 | `TIM2_CNT` 增量 1,003,654；同期 `xTickCount` 增量 1004 | 1 µs 计数，与 1 ms 的 RTOS 节拍一致（每节拍 999.7 µs，差值来自两次读取的先后） |
| TIM2 配置 | `CR1 = 1`、`PSC = 239`、`ARR = 0xFFFFFFFF`；`DBGMCU_APB1LFZ1` 的 bit 0 置位 | 已启动；调试暂停时冻结 |
| 内核 | `xSchedulerRunning = 1`，6 个任务 | 调度器运行（空闲、定时器、默认任务与 3 个应用任务） |
| 启动顺序 | TaskManager `state = RUNNING`、`rtos_init_timing = PRE_KERNEL`、`failure = NONE`、`created = 3`、`start_refused = 0` | `boot_init` → `boot_init_rtos`（内核启动前）→ `boot_run` 的顺序正确 |
| 栈余量 | 3 个任务剩余 420、857、861 字，`low_margin_task_index = 0xFFFFFFFF`；采样计数约 1 Hz 增长 | 余量充足，默认任务的 `boot_run` 按拍运行 |
| 没有停在错误处理 | `DWT_PCSR` 采样 16 次都在空闲任务、DJI 电机适配器、IMU 滤波、时基读取等正常代码中，没有 `Error_Handler`（0x08003E58） | 正常运行 |
| 外设中断链路 | 1 s 内陀螺采样序号增加 2000、加速度计增加 1605 | BMI088 的 EXTI、SPI 回调正常（与 2 kHz、1.6 kHz 的输出频率一致） |

未检查：USB CDC（板子的 USB 口没有接到主机）、CAN 电机与 SBUS（没有接外设）。
