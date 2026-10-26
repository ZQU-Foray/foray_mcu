# boot：生成区可以调用的启动入口（extern "C"，评审报告 §5.6）。
# P1 只建立 target；迁移期的启动入口仍是 legacy/Application/Entry.h 的 Application_*（例外 X-09），
# P3b 落地 boot_init / boot_init_rtos / boot_run 后，生成区只认本目录的头文件。
add_library(boot INTERFACE)
target_include_directories(boot INTERFACE "${PROJECT_SOURCE_DIR}/src")
