# 框架（评审报告 §5.9）：平台无关、可宿主编译；不得引用 app/、config/ 的路径或 target（规则 15）。
# 固件构建与宿主构建都会加入本目录。
#
# 迁移期（P1–P2.5）源码仍在 legacy/ 原地，下面各 target 按目标位置组织、引用 legacy/ 的路径；
# P2 搬迁时文件移到对应目录，这里只改路径（docs/ai/decisions/2026-10-26-P1-构建骨架/）。
add_subdirectory(hal)
add_subdirectory(boot)
add_subdirectory(utils)
add_subdirectory(lib)
add_subdirectory(driver)
