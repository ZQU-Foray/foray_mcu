# IMU 快速启动与零偏存储的上板验证（2026-11-25）

> Decision-20261125-01（恢复 5 V 输出）、-02（快速 READY）、-03（零偏-温度存储）。
> 原始记录在仓库外：`/mnt/data/projects/foray_mcu_archive/2026-11-25_bias_store/`
> （烧录前的芯片 Flash 备份 `chip_read_1.bin`、`chip_read_2.bin`，两遍一致，sha256 `56550572…`）。

## 固件

- `infantry_dm_mc02` Release，`dev` @ `00fc11b`；
- `.bin` 128,620 B，sha256 `d44ddcedda66a28f93b0247fc1f7a9aa104ec7861631b1f58eac918a71c3ea38`；
- 烧录用仓库的 `cmake --build build/arm-release-infantry --target flash`（OpenOCD，program + verify + reset；只擦镜像覆盖的扇区 `0x08000000–0x0801FFFF`）；
- 本次 EmberProbe 的 Agent Bridge 不可用（扩展未激活），读数改用 OpenOCD 短暂停机读出
  `g_imu.report_`（符号 `_ZN12_GLOBAL__N_1L5g_imuE` @ `0x24000828`，`report_` 偏移 2300），
  读完整即 `resume`。安全门一直锁着、遥控没动，云台没有出力。

## 烧录前

- 最后一个 Flash 扇区 `0x080E0000–0x080FFFFF` 全是 `0xFF`（1,048,576 B 里该扇区非 `0xFF` 字节 0 个），
  所以第一次上电时记录区是空的、不会触发压缩。

## 读数（`mdw` 解出来的关键字段）

| 场景 | `imu_state` | 零偏来源 | `settled_us` | 记录 | 说明 |
|---|---|---|---|---|---|
| 冷启动（记录区已擦） | READY | QUICK | **661,209 µs = 0.66 s** | 空区，写 1 条 | 0.5 s 快窗口 + 驱动初始化；在 46.625 ℃ 上取零偏 |
| 热复位（2 条记录） | READY | STORED | **161,571 µs = 0.16 s** | 2 条，写 1 条 | b(50.125 ℃)，直线模型（两点跨越 0.5 ℃） |
| 热复位（再启动） | READY | STORED | **162,031 µs = 0.16 s** | 2 条（跨度 0.4 ℃），写 1 条 | 跨度小于 0.5 ℃ → 常数模型 |
| 重刷同一固件后 | READY | STORED | **161,614 µs = 0.16 s** | 3 条，写 1 条 | 记录在重刷后仍在 |

同一时刻的其它字段：`precise_applied = 0`、`calibration = WAITING_FOR_TEMPERATURE`（板子 48.6 ℃，加热还在升温，
精确标定按设计等 50 ℃ 的稳定平台）；`rest_detected = 1`；`yaw_zero` 0.06–0.11°。
**精确标定转入后台后，READY 不再依赖它**——这是这次改动最直接的证据。

与改动前对比：冷启动 42 s → 0.66 s，热复位/复活 25 s → 0.16 s（42 s 的拆分与出处见
Decision-20261125-02 的 Context）。

## 5 V 输出

`GPIOC_ODR`（`0x58020814`）= `0x0000E009`：bit 15（DC5）、bit 14/13（两路 24 V）都是 1。
无线烧录器现在有供电；本次仍然用 USB 线烧录。

## 没做的

- 云台一直锁着（遥控没拨），所以没有测"READY 后立刻出力"的时延与前一分钟的 yaw 漂移；
  前一分钟漂移取决于 b(T₀) 的准确度，需要等零偏-温度记录攒到多个温度点后再测；
- 断电（拔 24 V 与 USB）后的冷启动计时：这次测的是"复位"，不是"断电再上电"，
  断电时传感器温度会往下掉，b(T₀) 的误差更大，要单独测；
- 记录区的压缩（擦除 1–2 s）没有在上板触发：要写满 4096 条（约 2000 次上电），
  只在宿主 `bias_store` 测试里覆盖。
