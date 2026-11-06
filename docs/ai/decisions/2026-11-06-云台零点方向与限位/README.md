# 2026-11-06 云台电机零点、方向、限位与 IMU 安装方向（上板记录）

对应 plan.md"云台"第 2 步的上板核对。负责人用手转云台，AI 经 SWD 读数；全程电机失能、不出力，没有烧录。

## 条件

- 板上固件：`link_dm_mc02` Release。Flash 内容与本机 `build/arm-release-link-local` 当天的 `.bin` 逐字节相同。它不用 CAN1，所以可以经 SWD 往 CAN1 注入帧。
- 两台 DM-J4310 都在 CAN1：yaw 是 ESC 1 / MST 0x11，pitch 是 ESC 2 / MST 0x12。电机的 24 V 来自板上两路可开关输出，经 GPIOC_BSRR 打开。
- 读数方法：
  - 电机位置：每轮给两台各发一帧失能（`FF×7 FD`），取回一帧反馈。
  - 姿态：读 `topics::imu::attitude` 的四元数，按 `gimbal.cpp` 的 ZYX 分解成 yaw、pitch、roll。
  - 每秒约 12 组。
- 原始日志与脚本在仓外归档 `foray_mcu_archive/2026-11-06_dm_zero/`（`read1/zero1/cycle1/session.log`、`can.tcl`、`watch.tcl`、`live.py`）。

## 结果

| 项目 | 测得 | 结论 |
|---|---|---|
| 零点 | 云台摆成枪管水平、朝正前时，yaw 读 +1.4364 rad，pitch 读 −0.0605 rad | 发 SAVE_ZERO（`FF×7 FE`）后两台都读 0；24 V 断电 3 s 再上电仍为 0，零点存在电机里，`zero_offset_rad` 取 0 |
| yaw 方向 | 左转：关节 +34.5°，IMU yaw +35.5° | `direction` +1 |
| pitch 方向 | 低头：关节 +15.0° | `direction` +1 |
| IMU 安装 | 低头 15° 时 IMU pitch 几乎不变，roll 变了 +14.8° | 板子水平转了 90°：BMI088 的 x 指向云台左、y 指向后、z 朝上。映射改为 source {1, 0, 2}、sign {−1, 1, 1} |
| pitch 挡块 | 低头 +0.378 rad（+21.7°），抬头 −0.581 rad（−33.3°） | 软限位 −0.52 ~ +0.32（每边留 0.06 rad） |
| yaw 两端（无滑环） | 左 +1.574 rad（+90.2°），右 −1.573 rad（−90.1°） | 软限位 −1.50 ~ +1.50（每边留 0.07 rad）；总宽不到一圈，上电读数没有 2π 的歧义 |

### 映射的核对

用录到的四元数按新映射离线换算云台系姿态（R_wb = R_ws · Mᵀ），在零点、低头 15°、低头挡块、抬头挡块、yaw 两端这几处停住时：

- 云台 pitch 与 pitch 关节角相差 +0.5° ~ +1.0°；
- 云台 roll 约为 −1.5° ~ −3.0°。

这两项都是台架与板子安装的倾斜，限位换算时作为底座倾斜扣掉。

还没有用新映射的固件上板复核，下次烧 infantry 固件时顺便看：云台静止时 `report_.pitch_rad` 与 `pitch_joint_rad` 应只差约 1°。

## 改动

- `config/profiles/infantry_dm_mc02/params.hpp`：IMU 映射；yaw、pitch 的限位；方向与零点的注释。
- `config/profiles/link_dm_mc02/params.hpp`：IMU 映射（板子装在云台上时，发给上位机的姿态也要用云台系）。
- `bringup_dm_mc02` 不改：它是板级自检，不假定板子怎么装。

## 测完后的板子状态

- 两路 24 V 是开着的：电机通电，处于失能状态。
- CAN1 的发送错误计数偏高：断电验证时发出的一帧没有应答。link 固件不用 CAN1，所以没有影响。
- 复位后都恢复原样。
