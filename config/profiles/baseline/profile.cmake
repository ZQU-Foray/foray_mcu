# profile：一台具体机器人用哪块板、哪些能力、什么参数，只含数据（评审报告 §5.4）。
# 由根 CMakeLists.txt 按 -DPROFILE=<名称> 加载；名称即本目录名。
#
# baseline：迁移基线 a2cfd16 的固件原样（DM-MC02；IMU、底盘、云台、遥控接收），
# 用于 P1–P2.5 不改行为阶段的符号对照。P3b 起再按兵种建立正式 profile。

set(PROFILE_ID baseline)

# 板卡：platform/ 下的相对目录，其中 board.cmake 声明板标识、CPU 与链接脚本等。
set(BOARD st/stm32h7/board_dm)

# 能力集合：P3b 起决定 assembly 链接哪些能力库；P1 只登记，装配仍是迁移源的固定组合。
set(FEATURES imu chassis gimbal remote)
