# 步兵小云台辨识数据（2026-11-06）

台架上两台 DM-J4310 小云台在 PID 闭环下录的 1 kHz 数据，用来辨识 J、摩擦、重力与延迟（Decision-20261106-01，`tool/control/identify_axis.py`）。每个轴挑了三段：2 Hz 与 4 Hz 的正弦扫描各一段，用来回归；手动快推一段，用来交叉验证。完整的 13 段（另有静止、0.5/1 Hz 扫描、手拍）在仓外的存档 `foray_mcu_archive/2026-11-06_gimbal_identification/`。

## 来源

| 项 | 值 |
|---|---|
| 机构 | pitch 输出轴直连；yaw 经同步带传动，约 1:1。IMU（DM-MC02 板载 BMI088）装在 pitch 上 |
| 固件 | infantry_dm_mc02 Release，提交 `dcbe725`，ELF SHA-256 前 16 位 `2f4fcdb71d1a0432` |
| 控制律 | 提交时 `params.hpp` 的 PID：pitch 角度环 kp 20，角速度环 0.5 / 2.5，积分上限 0.3 N·m；yaw 角度环 kp 8（角速度上限 3 rad/s），角速度环 0.6 / 3.0，积分上限 0.3 N·m；总限幅 1 N·m，重力前馈 0 |
| 激励 | 扫描：`tool/robolink/link_probe.py --sweep A,F --sweep-axis <轴>`，PC 以 1 kHz 发目标，自瞄接管（TRACK）。手动：右摇杆快速来回推（MANUAL） |
| 录制 | `Gimbal::Capture` 录 4000 条（每份新姿态一条，约 1.003 ms），经 EmberProbe 工作区 skills 脚本每次 32 条导出 |
| 精简 | 只留该轴的列；float 按 8 位有效数字写（原值是 float32）。上面这几段用 `identify_axis.py` 算出来的结果和原始导出逐位相同 |

## 文件

| 文件 | 段 | 幅度 | 模式 |
|---|---|---|---|
| `pitch_p4_sweep_2hz.csv` | pitch 正弦 2 Hz | ±0.15 rad | TRACK |
| `pitch_p5_sweep_4hz.csv` | pitch 正弦 4 Hz | ±0.15 rad（实际跟到 ±0.17） | TRACK |
| `pitch_p6_manual.csv` | pitch 摇杆快推，−30° ~ +20° | 碰到软限位 | MANUAL |
| `yaw_y4_sweep_2hz.csv` | yaw 正弦 2 Hz | ±0.10 rad | TRACK |
| `yaw_y5_sweep_4hz.csv` | yaw 正弦 4 Hz | ±0.05 rad（实际跟到 ±0.04） | TRACK |
| `yaw_y6_manual.csv` | yaw 摇杆快推，±1.5 rad | 碰到软限位、力矩打满约 1% | MANUAL |

列：`index`、`time_us`（采样时刻，32 位 µs）、`mode`（2 HOLD、3 TRACK、4 MANUAL）、`<轴>_rad`、`<轴>_rate`（rad/s，欧拉角速度）、`<轴>_torque_newton_meter`（下发的总力矩，含限幅）、`<轴>_feedback_torque_newton_meter`（电机反馈）。

## 用法与对照值

```sh
~/.local/venvs/control/bin/python tool/control/identify_axis.py test/data/gimbal/pitch_p4_sweep_2hz.csv test/data/gimbal/pitch_p5_sweep_4hz.csv --axis pitch
~/.local/venvs/control/bin/python tool/control/identify_axis.py test/data/gimbal/yaw_y4_sweep_2hz.csv test/data/gimbal/yaw_y5_sweep_4hz.csv --axis yaw
```

| 数据 | 延迟（插值） | R² | J（kg·m²） |
|---|---|---|---|
| pitch 2 Hz + 4 Hz | 4.56 拍 | 0.807 | 0.00398 |
| yaw 2 Hz + 4 Hz | 8.87 拍 | 0.833 | 0.00307 |

改了辨识工具以后，用这两条命令核对结果有没有变。四段扫描（0.5–4 Hz）合在一起的结果与交叉验证见 `docs/ai/decisions/2026-11-06-云台辨识/README.md`。

## 注意

- **电机反馈力矩不是真实力矩**：反馈和下发的比值随频率下降（pitch 0.5 Hz 0.78、4 Hz 0.38），像是电机固件做了低通。回归用下发力矩。
- **单点力矩尖峰**：录制用的固件里，仲裁还没有外推 PC 目标。USB 帧晚到再补上时，云台的目标平滑会把前馈打到上限，力矩打满一拍，约占 0.1% 的样本。`bc90450` 起仲裁按角速度外推目标。
- **yaw 的延迟大约是 pitch 的两倍**：同步带有弹性，相当于额外的滞后，延迟要扫到 9 拍以上。
- 录制前，yaw 同步带曾因松动跳齿。拉紧后重新设了零点，上面的 yaw 数据都是拉紧之后录的；录完检查过，IMU yaw 与关节角之差只变了 1.3°，没有再跳齿。
