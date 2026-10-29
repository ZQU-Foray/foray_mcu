# 宿主机构建（宿主测试、框架独立构建）：使用本机的 GCC 或 Clang，不交叉编译。
# 根 CMakeLists.txt 据 CMAKE_CROSSCOMPILING 区分固件构建与宿主构建；本文件只是显式声明，
# 让 presets 三种构建都经由 cmake/toolchains/ 选择。MSVC 不支持（根 CMakeLists.txt 会检查）。
set(TOOLCHAIN_CPU host)
