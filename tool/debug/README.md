# 调试工具（tool/debug）

通过 SWD 读取运行中的固件，不停核、不复位、不写入目标。需要 OpenOCD 0.12 与 CMSIS-DAP 调试器（默认配置：`interface/cmsis-dap.cfg` + `target/stm32h7x.cfg`）；Python 3.10+，只用标准库。

| 工具 | 用途 |
|---|---|
| `swd_poll.py` | 周期读取若干内存区域写成文本日志；可先等目标断电、上电后立即开始（记录冷启动过程）；结束后可整块转储内存 |
| `imu_capture_decode.py` | 解码 `ImuDiagnostics` 的陀螺采集结构（Capture 版本 2），输出统计摘要、原始样本 CSV 与原始结构体 |
| `heater_log_decode.py` | 解析 `swd_poll.py` 记录的 BMI088 加热器日志，输出逐行 CSV 与升温指标（到温时间、过冲、稳定时间、稳态波动） |

## 示例

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
