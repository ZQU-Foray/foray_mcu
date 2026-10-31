# F2 第 4 步 bringup 固件的上板验证（2026-10-31）

验证 Decision-20261031-12：任务运行器按任务表创建任务；周期任务发布 topic；按 topic 触发的任务收到每一次发布；
读者优先级高于写者时（Decision-20261029-02 修好的情形）没有卡死、没有读到写了一半的数据。
同时验证 Decision-20261031-10 的 hal/os 实现在板上运行：`Signal`（1 号任务通知槽）、`CriticalSection`（BASEPRI）、时基。

## 固件

- 分支 `f2/bringup`，提交 `bc26a4f`；profile `bringup_dm_mc02`，arm-debug（`-O0 -g3`）
- `firmware_bringup_dm_mc02.bin` 的 sha256：`a7a15f0ac85b9bdc95f8890bc5a14032bb2be228aaee085189a1a666e7b6ef41`
- 调试器 Vllink Basic2（CMSIS-DAP），OpenOCD 0.12.0，SWD 1800 kHz；`program … verify reset exit`，`Verified OK`（`flash.log`）
- 板上原固件的备份见 `../2026-10-31-F2-启动入口上板验证/README.md`

## 方法

不停核，用 OpenOCD 的 `read_memory` 隔 1 s 读两次（`probe_bringup.tcl`）。地址由同一个 ELF 的调试信息离线算出（gdb 不连目标）：

| 读什么 | 地址 |
|---|---|
| `TIM2_CNT`、`xTickCount` | 0x40000024、0x2000447C |
| `g_heartbeat.sequence_` | 0x20014CC0 |
| `topics::bringup::heartbeat` 的 `publish_count_` | 0x24000168 |
| `g_runner.status_`（两个任务，每个 4 字：轮数、最长耗时 µs、错过周期、剩余栈） | 0x200155B4 |
| `g_monitor.report_`（收到、丢失、乱序、超时、最近延迟、最大延迟、最近序号） | 0x240001A4 |

## 结果（`probe.log`）

| 检查 | 读数 | 结论 |
|---|---|---|
| 时基 | `TIM2_CNT` 增量 1,006,511；`xTickCount` 增量 1006 | 每节拍 1000.5 µs |
| 周期任务 | 心跳序号增加 1005，topic 发布次数同步增加；heartbeat 任务错过周期 0 次，单轮最长 16 µs | 1 kHz 按表运行 |
| 按 topic 触发 | monitor 任务的轮数与 heartbeat 相同（两次读数都是 0x30DC） | 每次发布都叫醒了等待的任务，没有合并、没有漏 |
| topic 订阅 | 收到 = 轮数；丢失 0、乱序 0、超过 100 µs 0 次；最近延迟 7 µs，最大 9 µs | 高优先级读者、低优先级写者下读取正常 |
| 栈 | heartbeat 剩余 129 字、monitor 剩余 143 字（各 256 字） | 余量充足 |
| 没有停在错误处理 | `DWT_PCSR` 采样 16 次都在空闲任务里（`Error_Handler` 在 0x08003204） | 负载很轻，正常运行 |

未验证：F2 验收里的"中断里发布的 topic 能被阻塞等待的任务收到"——还没有中断源（Decision-20261031-12 第 7 点）。
