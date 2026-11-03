# profile：上位机链路联调（DM-MC02）：IMU 云台姿态经 RoboLink 发给上位机，辅助视觉做手眼标定；不接电机
# （Decision-20261103-01 第 10 点）。临时的联调配置，第一个兵种 profile 由负责人定名。
# 由根 CMakeLists.txt 按 -DPROFILE=<名称> 加载；名称即本目录名。只含数据。

set(PROFILE_ID link_dm_mc02)

# 板卡：platform/ 下的路径（厂商/板，Decision-20261101-04），其中 board.cmake 声明板标识、CPU 与烧录配置。
set(BOARD st/dm_mc02)

# 链接的能力：app_<能力>。组合根是 config/assembly/link_dm_mc02.cpp。
set(CAPABILITIES imu pc_link)
