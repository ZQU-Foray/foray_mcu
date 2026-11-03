# tool/robolink：RoboLink 标准帧与对账

上下位机链路协议 RoboLink v0.2（`foray_interfaces` 的 `protocol/robolink.md`，提交 `5973436`）在本仓只有一处编解码：`src/middleware/robolink`。防止两端漂移靠两份独立的实现对同一批标准帧（Decision-20261103-01）：

| 文件 | 作用 |
|---|---|
| `make_golden.py` | 只照协议文本，用 `zlib.crc32` 与 `struct` 生成标准帧 `test/data/robolink/golden_v0_2.txt`。不读本仓的 C++ |
| `pc_crosscheck.cpp` | 用 PC 端 `foray_platform` 的编解码核对同一批标准帧。不进本仓构建 |
| `link_probe.py` | 上板探测：在 PC 上扮演上位机连 `link_dm_mc02` 固件，统计握手、频率、帧序号、样本停留时间、IMU 状态变化，可选 1 kHz 控制快照与拔线重连；`--follow` 实时打印姿态的 yaw/pitch/roll |
| `pc_board_check.cpp` | 用 PC 端 `foray_platform` 的链路状态机直接连板子（视觉组实际用的代码）。不进本仓构建 |
| `pc_session_check.cpp` | PC 端的链路状态机（`RoboLinkHostLink` 与串口类）和 MCU 端的 `robolink::Session` 在伪终端上联调：握手、1 kHz 姿态与控制快照、两个方向的卡死与恢复。不进本仓构建 |

MCU 端的宿主测试 `robolink_wire`、`robolink_messages`、`robolink_session` 逐字节对照标准帧。

## 改协议

1. 先改 `make_golden.py`（照新的协议文本），重新生成：

   ```bash
   tool/robolink/make_golden.py            # 写 test/data/robolink/golden_v0_2.txt
   tool/robolink/make_golden.py --check    # 只核对文件与脚本一致
   ```

2. 再改 `src/middleware/robolink`，直到宿主测试通过。

## 上板

板子烧 `link_dm_mc02` 固件（`-DPROFILE=link_dm_mc02`），USB 接电脑。IMU 要恒温才能完成零偏标定，板子要接 24 V（XT30）。

```bash
tool/robolink/link_probe.py --duration 60 --control --reconnect   # 自动找 DM-MC02 的虚拟串口
tool/robolink/link_probe.py --follow                              # 实时打印 yaw/pitch/roll（度），Ctrl-C 停止
./pc_board_check /dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_* 30 --control
```

`--follow` 每 0.5 s 打一行（`--interval` 可改），只在 `IMU 状态` 为 `READY` 时 yaw 才扣过零点；`SETTLING` 时数值会漂。

串口属于 `dialout` 组。没有读写权限时加一条 udev 规则（一次性，拔插、烧录后都有效）：

```bash
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", TAG+="uaccess"' | sudo tee /etc/udev/rules.d/70-dm-mc02-cdc.rules
sudo udevadm control --reload && sudo udevadm trigger
```

## 与 PC 端对账

PC 端代码更新后重跑（`<fp>` 是 `foray_platform` 仓库里的 `foray_platform` 包目录）：

```bash
g++ -std=c++17 -I<fp>/include tool/robolink/pc_crosscheck.cpp \
    <fp>/src/link/robolink_codec.cpp <fp>/src/link/robolink_crc.cpp <fp>/src/link/robolink_frame_parser.cpp \
    -o pc_crosscheck
./pc_crosscheck test/data/robolink/golden_v0_2.txt

g++ -std=c++17 -I<fp>/include -Isrc tool/robolink/pc_session_check.cpp \
    <fp>/src/link/robolink_host_link.cpp <fp>/src/link/robolink_byte_channel.cpp <fp>/src/link/usb_cdc_port.cpp \
    <fp>/src/link/robolink_codec.cpp <fp>/src/link/robolink_crc.cpp <fp>/src/link/robolink_frame_parser.cpp \
    src/middleware/robolink/{wire,messages,session}.cpp src/utils/crc/crc32.cpp -o pc_session_check
./pc_session_check

g++ -std=c++17 -I<fp>/include tool/robolink/pc_board_check.cpp \
    <fp>/src/link/robolink_host_link.cpp <fp>/src/link/robolink_byte_channel.cpp <fp>/src/link/usb_cdc_port.cpp \
    <fp>/src/link/robolink_codec.cpp <fp>/src/link/robolink_crc.cpp <fp>/src/link/robolink_frame_parser.cpp \
    -o pc_board_check
```

2026-11-03 对 `foray_platform` `dev` @ `de496bd` 的结果见 [对账记录](../../docs/ai/decisions/2026-11-03-RoboLink-对账/README.md)。
