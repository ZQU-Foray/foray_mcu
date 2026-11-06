# 调试工具（tool/debug）

通过 SWD 读取运行中的固件，不停核、不复位、不写入目标。需要 OpenOCD 0.12 与 CMSIS-DAP 调试器（默认配置：`interface/cmsis-dap.cfg` + `target/stm32h7x.cfg`）；Python 3.10+，只用标准库。

| 工具 | 用途 |
|---|---|
| `bringup_probe.py` | bringup 固件的上板检查：从 ELF 算出各能力统计的地址（gdb，不连目标），隔 1 s 读两次，打印增量与判定（Decision-20261101-03，清单见 `docs/ai/decisions/2026-11-01-F2-外设上板检查清单.md`）；需要 arm-none-eabi-gdb |
| `gimbal_probe.py` | 云台固件（`infantry_dm_mc02`）的上板调参：读云台的模式、目标、力矩与补偿力矩，安全门与遥控的状态，达妙统计与 gimbal 任务的轮数/耗时（Decision-20261105-01）；与 `bringup_probe.py` 共用读内存的代码 |
| `swd_poll.py` | 周期读取若干内存区域写成文本日志；可先等目标断电、上电后立即开始（记录冷启动过程）；结束后可整块转储内存 |
| `imu_capture_decode.py` | 解码 `ImuDiagnostics` 的陀螺采集结构（Capture 版本 2），输出统计摘要、原始样本 CSV 与原始结构体 |
| `heater_log_decode.py` | 解析 `swd_poll.py` 记录的 BMI088 加热器日志，输出逐行 CSV 与升温指标（到温时间、过冲、稳定时间、稳态波动） |

## 示例

```sh
# bringup 固件：读出各能力的统计并判定（ELF 必须是板上正在跑的那一个）
python3 tool/debug/bringup_probe.py --elf build/arm-debug/firmware_bringup_dm_mc02.elf \
    --gdb /opt/xpack-arm-none-eabi-gcc-15.2.1-1.1/bin/arm-none-eabi-gdb

# 云台固件：调参时读云台、安全门、遥控与任务统计
python3 tool/debug/gimbal_probe.py --elf build/arm-debug-infantry/firmware_infantry_dm_mc02.elf \
    --gdb /opt/xpack-arm-none-eabi-gcc-15.2.1-1.1/bin/arm-none-eabi-gdb --interval 0.5
```

在 WSL 中调用 Windows 版 OpenOCD 时，用 `--openocd` 指向 `openocd.exe`，并用 `--cwd /mnt/c` 让它在 Windows 路径下运行。

```sh
# 记录一次冷启动：断电后上电，5 Hz 读取 DWT 计数、加热器对象与陀螺采集头部 15 分钟，结束后转储 AXI SRAM
python3 tool/debug/swd_poll.py --openocd <openocd 路径> \
    --region 0xE0001004:1 --region 0x2400490c:40 --region 0x24000788:18 \
    --rate 5 --duration 900 --wait-power-cycle --out heater.log \
    --dump 0x24000000:0x50000:axi_after.bin

# 解析加热器日志（区域顺序：CYCCNT、加热器对象、采集头部）
python3 tool/debug/heater_log_decode.py heater.log --csv heater.csv --json metrics.json

# 在 RAM 快照中查找并解码陀螺采集
python3 tool/debug/imu_capture_decode.py axi_after.bin --base 0x24000000 --find
python3 tool/debug/imu_capture_decode.py axi_after.bin --base 0x24000000 \
    --address 0x24000788 --csv samples.csv --json summary.json
```

上例中的地址属于芯片原固件（见 `docs/ai/decisions/2026-10-26-丢失功能与恢复依据.md`）；`a2cfd16` 的地址可从 `docs/ai/decisions/baseline-a2cfd16/Debug.nm.txt` 查到。

注意：

- 日志第一列是主机单调时间，只表示到达先后（OpenOCD 输出可能成批到达）；精确时间请把 DWT_CYCCNT（`0xE0001004`）作为一个区域一起读取，以它为准。
- OpenOCD 连接时会写调试寄存器（DBGMCU），这些寄存器断电即恢复，不影响 Flash 与应用状态。

## 上板调参与取数：EmberProbe（VS Code 插件）

在线调参、看波形、触发录制、把录制缓冲导出成 CSV 用 EmberProbe（`bakesheep.emberprobe` 0.9.1，
经 SWD、不停核），比本目录的轮询工具更适合这几件事；本目录的工具留着写日志、脚本化与冷启动记录。

**在线调参**（PID 点亮时就能用）：

1. 侧栏"ELF 全部变量"搜 `tuning_`，把要调的项（如 `g_gimbal.tuning_.pitch.angle.kp`）与
   `g_gimbal.tuning_.sequence` 加进写入列表；
2. 波形窗口加 `g_gimbal.report_` 里的 `pitch_rad`、`pitch_target_rad`、`pitch_torque_newton_meter`、
   `pitch_diagnostics.compensation_newton_meter`，采样 100–200 Hz；
3. 改数值，再把 `sequence` 加 1；`applied_sequence` 跟上、`rejected_count` 没涨就是生效了
   （被拒时块里的值会改回当前生效的）；
4. 调完读 `g_gimbal.tuning_`，把值抄回 `params.hpp`。

**录制**（一段 4 s、1 kHz；一条 40 字节）：

1. 写 `g_gimbal.capture_.request = 4000`；
2. 等 `g_gimbal.capture_.count` 到 4000；
3. 按 `g_gimbal_capture[0:99]`、`[100:199]`……分片导出到 `build/capture/<段名>/`（一次最多读 4096 字节，
   一片约 100 条；4000 条约 40 片）；
4. `tool/control/identify_axis.py build/capture/<段名> --axis pitch` 辨识。

**限制与注意**：

- 运行中只写对齐的 32 位量（`float` / `std::uint32_t`）；写 `bool`、`uint8_t`、枚举会短暂停核，
  CAN 指令会断、加热 PWM 可能卡在高电平；
- 一个探针同一时刻只能有一个 OpenOCD：用本目录的 `*_probe.py` 或 `tool/flash` 之前，先在 EmberProbe
  里停止采样；反过来也一样；
- 名字有歧义时写成 `infantry_dm_mc02.cpp::g_gimbal.tuning_.sequence` 这种带文件名的形式；
- ELF 必须与板上固件一致（用 EmberProbe 烧录一次，读写结果会标 `match`）；Release 从 2026-11-06 起
  带调试信息（`-Os -g3`），EmberProbe 能展开 `g_gimbal.report_`、`g_gimbal.tuning_`；
- 一次读取最多 4096 字节、大数组按切片读（`LIVE_READ_BUDGET_EXCEEDED` 就是这个限制）；
- 写入前先在对话里确认变量、旧值、新值；超时的写入算"结果未知"，先读回再决定，不自动重发；
- `.axi_bss` 是 `NOLOAD`、启动时不清零：录制只读自己写过的部分；以后开 D-cache 要用 MPU 把这一段
  设成不缓存，否则调试器读到旧数据。
