# profile：框架验证用的 bringup（DM-MC02），不含 RM 能力（架构 v2 §3.6）。
# 由根 CMakeLists.txt 按 -DPROFILE=<名称> 加载；名称即本目录名。只含数据。

set(PROFILE_ID bringup_dm_mc02)

# 板卡：platform/ 下的路径（厂商/板），其中 board.cmake 声明板标识、CPU 与烧录配置。
set(BOARD st/dm_mc02)

# 链接的能力：app_<能力>。组合根是 config/assembly/bringup_dm_mc02.cpp。
set(CAPABILITIES bringup)
