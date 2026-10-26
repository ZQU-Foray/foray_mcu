# Arm GNU 交叉编译的公共部分，由 arm-gcc-cortex-*.cmake 在设置 TOOLCHAIN_CPU、TOOLCHAIN_CPU_FLAGS 之后包含。
#
# 工具链根目录依次取自：缓存变量 ARM_GCC_ROOT、环境变量 ARM_GCC_ROOT、PATH 中的 arm-none-eabi-gcc。
# 本机路径放 CMakeUserPresets.json（不入库），CI 用环境变量；不在这里写死任何人的路径。
# GNU 专属的编译、链接选项只出现在工具链与 platform 的 CMake 中（评审报告 §5.4）。
#
# 编译选项与迁移源 legacy/Platform/STM32H7/DM_MC02/cmake/gcc-arm-none-eabi.cmake 一致（P1 的符号对照依赖这一点）：
#   - Debug 为 -O0 -g3，Release 为 -Os -g0，Release 不定义 NDEBUG（断言保留）；
#   - 汇编的配置选项沿用 CMake 默认值；
#   - -Wall 作用于全部 C/C++ 代码（含厂商与生成代码）：GCC 15 在 -O0 下 -Wall 会改变生成的代码
#     （空死循环里多一条 nop，2026-10-26 实测），只给部分代码加会使符号大小与迁移源不同；
#     其余告警（-Wextra -Werror）由 cmake/targets.cmake 按 target 设置。

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

if(NOT ARM_GCC_ROOT)
    if(DEFINED ENV{ARM_GCC_ROOT})
        set(ARM_GCC_ROOT "$ENV{ARM_GCC_ROOT}")
    else()
        find_program(ARM_GCC_EXECUTABLE arm-none-eabi-gcc)
        if(ARM_GCC_EXECUTABLE)
            get_filename_component(ARM_GCC_ROOT "${ARM_GCC_EXECUTABLE}" DIRECTORY)
            get_filename_component(ARM_GCC_ROOT "${ARM_GCC_ROOT}" DIRECTORY)
        endif()
    endif()
endif()
if(NOT ARM_GCC_ROOT)
    message(FATAL_ERROR
        "找不到 Arm GNU 工具链：设置 ARM_GCC_ROOT（缓存变量或环境变量），或把 arm-none-eabi-gcc 加入 PATH")
endif()
file(TO_CMAKE_PATH "${ARM_GCC_ROOT}" ARM_GCC_ROOT)
set(ARM_GCC_ROOT "${ARM_GCC_ROOT}" CACHE PATH "Arm GNU 工具链根目录（含 bin/arm-none-eabi-gcc）")
# try_compile 会在独立工程里重新执行本文件，需要把这几个变量带过去。
set(CMAKE_TRY_COMPILE_PLATFORM_VARIABLES ARM_GCC_ROOT TOOLCHAIN_CPU TOOLCHAIN_CPU_FLAGS)

if(CMAKE_HOST_WIN32)
    set(TOOLCHAIN_SUFFIX ".exe")
else()
    set(TOOLCHAIN_SUFFIX "")
endif()
set(TOOLCHAIN_PREFIX "${ARM_GCC_ROOT}/bin/arm-none-eabi-")
set(CMAKE_C_COMPILER "${TOOLCHAIN_PREFIX}gcc${TOOLCHAIN_SUFFIX}")
set(CMAKE_CXX_COMPILER "${TOOLCHAIN_PREFIX}g++${TOOLCHAIN_SUFFIX}")
set(CMAKE_ASM_COMPILER "${CMAKE_C_COMPILER}")
set(CMAKE_AR "${TOOLCHAIN_PREFIX}ar${TOOLCHAIN_SUFFIX}")
set(CMAKE_RANLIB "${TOOLCHAIN_PREFIX}ranlib${TOOLCHAIN_SUFFIX}")
set(CMAKE_OBJCOPY "${TOOLCHAIN_PREFIX}objcopy${TOOLCHAIN_SUFFIX}")
set(CMAKE_SIZE "${TOOLCHAIN_PREFIX}size${TOOLCHAIN_SUFFIX}")
foreach(tool CMAKE_C_COMPILER CMAKE_CXX_COMPILER CMAKE_AR CMAKE_OBJCOPY CMAKE_SIZE)
    if(NOT EXISTS "${${tool}}")
        message(FATAL_ERROR "Arm GNU 工具链缺少 ${${tool}}（ARM_GCC_ROOT=${ARM_GCC_ROOT}）")
    endif()
endforeach()

# 裸机目标不能链接出宿主可执行文件，编译器探测只编静态库。
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_EXECUTABLE_SUFFIX_C ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_ASM ".elf")

set(CMAKE_C_FLAGS "${TOOLCHAIN_CPU_FLAGS} -Wall -fdata-sections -ffunction-sections -fstack-usage")
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")
set(CMAKE_ASM_FLAGS "${TOOLCHAIN_CPU_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")

set(CMAKE_EXE_LINKER_FLAGS
    "${TOOLCHAIN_CPU_FLAGS} --specs=nano.specs -Wl,--gc-sections -Wl,--print-memory-usage")
# 生成 map 文件的链接选项前缀，由 cmake/targets.cmake 的 add_firmware_outputs() 拼上路径。
set(TOOLCHAIN_MAP_FILE_OPTION "-Wl,-Map=")
