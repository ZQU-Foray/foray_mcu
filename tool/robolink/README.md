# tool/robolink：RoboLink 标准帧与对账

上下位机链路协议 RoboLink v0.2（`foray_interfaces` 的 `protocol/robolink.md`，提交 `5973436`）在本仓只有一处编解码：`src/middleware/robolink`。防止两端漂移靠两份独立的实现对同一批标准帧（Decision-20261103-01）：

| 文件 | 作用 |
|---|---|
| `make_golden.py` | 只照协议文本，用 `zlib.crc32` 与 `struct` 生成标准帧 `test/data/robolink/golden_v0_2.txt`。不读本仓的 C++ |
| `pc_crosscheck.cpp` | 用 PC 端 `foray_platform` 的编解码核对同一批标准帧。不进本仓构建 |
| `pc_session_check.cpp` | PC 端的链路状态机（`RoboLinkHostLink` 与串口类）和 MCU 端的 `robolink::Session` 在伪终端上联调：握手、1 kHz 姿态与控制快照、两个方向的卡死与恢复。不进本仓构建 |

MCU 端的宿主测试 `robolink_wire`、`robolink_messages`、`robolink_session` 逐字节对照标准帧。

## 改协议

1. 先改 `make_golden.py`（照新的协议文本），重新生成：

   ```bash
   tool/robolink/make_golden.py            # 写 test/data/robolink/golden_v0_2.txt
   tool/robolink/make_golden.py --check    # 只核对文件与脚本一致
   ```

2. 再改 `src/middleware/robolink`，直到宿主测试通过。

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
```

2026-11-03 对 `foray_platform` `dev` @ `de496bd` 的结果见 [对账记录](../../docs/ai/decisions/2026-11-03-RoboLink-对账/README.md)。
